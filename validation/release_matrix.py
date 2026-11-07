"""Release target contract and identity checks shared by packaging and collection."""
import argparse
import hashlib
import json
import re
import tempfile
from pathlib import Path

from archive_objects import archive_machines, pe_machine, ELF_MACHINES, COFF_MACHINES
from release_package import minimum_transport_version, unpack_archive


def load_targets(path):
    targets = json.loads(Path(path).read_text(encoding='utf-8'))
    linux = targets['linux']
    windows = targets['windows']
    if len(linux) != 8 or {(r['ubuntu'], r['arch']) for r in linux} != {
            (u, a) for u in ('20.04', '22.04', '24.04', '26.04') for a in ('x86_64', 'aarch64')}:
        raise ValueError('Expected exactly eight Ubuntu targets')
    if any(r['distribution'] != 'ubuntu' + r['ubuntu'] for r in linux):
        raise ValueError('Ubuntu distribution identity mismatch')
    if len(windows) != 4 or {(r['arch'], r['configuration']) for r in windows} != {
            (a, c) for a in ('x64', 'ARM64') for c in ('Debug', 'Release')}:
        raise ValueError('Expected exactly four Windows targets')
    if any(r['runtime'] != {'Debug': 'MDd', 'Release': 'MD'}[r['configuration']]
           or r['toolset'] != '143' for r in windows):
        raise ValueError('Windows runtime/toolset identity mismatch')
    return targets


def identity(info):
    system, arch, config = (info[k] for k in ('system', 'architecture', 'configuration'))
    version = info['version']
    if not re.fullmatch(r'\d+\.\d+\.\d+', version):
        raise ValueError('Invalid package version')
    if info['package'] not in ('linkerbot-hand-cpp', 'linkerbot-comm', 'linkerbot-transport'):
        raise ValueError('Unknown SDK package')
    if (info.get('schema_version'), info.get('cxx_standard'), info.get('pic'),
            info.get('instrumented'), info.get('source_dirty')) != (1, 17, True, False, False):
        raise ValueError('Release requires clean C++17 PIC uninstrumented metadata')
    compiler = info['compiler']
    if system == 'Linux':
        if arch not in ELF_MACHINES or config != 'Release':
            raise ValueError('Unsupported Linux architecture/configuration')
        if (info['standard_library'], info['libstdcxx_dual_abi']) != ('libstdcxx', '1'):
            raise ValueError('Linux release requires libstdc++ ABI1')
        if not re.fullmatch(r'ubuntu(20|22|24|26)\.04', info['distribution']):
            raise ValueError('Unsupported Ubuntu distribution')
        if not re.fullmatch(r'GNU \d+(\.\d+)+', compiler):
            raise ValueError('Ubuntu release requires GNU compiler identity')
        return (system, info['distribution'], arch, config)
    if system == 'Windows':
        if arch not in COFF_MACHINES or config not in ('Debug', 'Release'):
            raise ValueError('Unsupported Windows architecture/configuration')
        if (info['toolset'], info['standard_library'], info['distribution'], info['runtime']) != (
                '143', 'msvc', 'Windows', {'Debug': 'MDd', 'Release': 'MD'}[config]):
            raise ValueError('Windows release requires matching v143 runtime and standard library')
        if not re.fullmatch(r'MSVC \d+(\.\d+)+', compiler):
            raise ValueError('Windows release requires MSVC compiler identity')
        return (system, 'Windows', arch, config)
    raise ValueError('Unsupported release system')


def archive_name(info):
    identity(info)
    compiler = info['compiler'].replace(' ', '')
    stem = f"{info['package']}-{info['version']}-{info['system']}-{info['architecture']}"
    if info['system'] == 'Linux':
        return f"{stem}-{info['distribution']}-{compiler}-libstdcxx-abi1-Release.tar.gz"
    compiler = info['compiler'].split()[1]
    return f"{stem}-msvc{info['toolset']}-{compiler}-{info['configuration']}-{info['runtime']}.zip"


def verify_objects(prefix, info, library):
    system, arch = info['system'], info['architecture']
    path = prefix / 'lib' / (library + '.lib' if system == 'Windows' else 'lib' + library + '.a')
    expected = (ELF_MACHINES if system == 'Linux' else COFF_MACHINES)[arch]
    machines = archive_machines(path, system)
    if any(machine != expected for machine in machines):
        raise ValueError(f'Archive object architecture mismatch: {path.name}')
    if system == 'Windows':
        for binary in (prefix / 'bin').glob('*'):
            if binary.suffix.lower() in ('.exe', '.dll') and pe_machine(binary) != expected:
                raise ValueError(f'PE architecture mismatch: {binary.name}')
    return len(machines)


def compatible_versions(hand, comm):
    h, c = tuple(map(int, hand.split('.'))), tuple(map(int, comm.split('.')))
    minimum = tuple(map(int, minimum_transport_version(Path(__file__).resolve().parents[1]).split('.')))
    if h[:2] != c[:2] or c < minimum:
        raise ValueError('Transport must meet the SDK minimum in the same minor line')


def collect(directory, targets_path, version, hand_sha, comm_sha):
    targets = load_targets(targets_path)
    expected = {('Linux', r['distribution'], r['arch'], 'Release') for r in targets['linux']}
    expected |= {('Windows', 'Windows', r['arch'], r['configuration']) for r in targets['windows']}
    seen, packages = set(), []
    archives = sorted([*directory.glob('*.tar.gz'), *directory.glob('*.zip')])
    if len(archives) != 12:
        raise ValueError('Release requires exactly twelve hand SDK archives')
    for archive in archives:
        with tempfile.TemporaryDirectory() as temporary:
            prefix, sha = unpack_archive(archive, Path(temporary) / 'sdk')
            hand, comm = [json.loads((prefix / 'share' / p / 'build-info.json').read_text(encoding='utf-8'))
                          for p in ('linkerbot-hand-cpp', 'linkerbot-comm')]
            key = identity(hand)
            if key not in expected or key in seen or identity(comm) != key:
                raise ValueError('Duplicate/unexpected target or mismatched transport identity')
            if hand['package'] != 'linkerbot-hand-cpp' or comm['package'] not in ('linkerbot-comm', 'linkerbot-transport'):
                raise ValueError('Package identity mismatch')
            if archive.name != archive_name(hand):
                raise ValueError('Archive filename does not match build metadata')
            if (hand['version'], hand['hand_sha'], hand['comm_sha'], comm['comm_sha']) != (
                    version, hand_sha, comm_sha, comm_sha):
                raise ValueError('Source/version identity mismatch')
            compatible_versions(hand['version'], comm['version'])
            for field in ('compiler', 'standard_library', 'libstdcxx_dual_abi', 'runtime', 'toolset'):
                if hand[field] != comm[field]:
                    raise ValueError('Hand/transport toolchain mismatch: ' + field)
            counts = {p: verify_objects(prefix, info, lib) for p, info, lib in (
                ('hand', hand, 'linkerbot-cpp-sdk'), ('comm', comm, 'linkerbot-comm'))}
            seen.add(key)
            packages.append({'file': archive.name, 'sha256': sha, 'bytes': archive.stat().st_size,
                             'hand': hand, 'transport': comm, 'verified_objects': counts})
    if seen != expected:
        raise ValueError('Incomplete release matrix')
    manifest = {'schema_version': 1, 'version': version, 'hand_sha': hand_sha,
                'comm_sha': comm_sha, 'packages': packages}
    (directory / 'release-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    assets = [*archives, *(p.with_name(p.name + '.sha256') for p in archives), directory / 'release-manifest.json']
    allowed = {p.name for p in assets} | {'SHA256SUMS.txt'}
    if {p.name for p in directory.iterdir()} - allowed:
        raise ValueError('Unexpected files in release asset directory')
    (directory / 'SHA256SUMS.txt').write_text(''.join(
        hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.name + '\n' for p in sorted(assets)), encoding='utf-8')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--targets', type=Path, default=Path(__file__).resolve().parents[1] / 'ci/package-targets.json')
    parser.add_argument('--version', required=True)
    parser.add_argument('--hand-sha', required=True)
    parser.add_argument('--comm-sha', required=True)
    args = parser.parse_args()
    collect(args.directory, args.targets, args.version, args.hand_sha, args.comm_sha)
