"""Exercise the complete release set using small ELF/COFF SDK archives."""
import hashlib
import json
import shutil
import struct
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path

from release_matrix import archive_name, collect, compatible_versions, identity, load_targets, verify_objects

ROOT = Path(__file__).resolve().parents[1]


def static_archive(machine, windows=False):
    if windows:
        body = struct.pack('<HHIIIHH', machine, 1, 0, 0, 0, 0, 0)
    else:
        body = b'\x7fELF\x02\x01' + bytes(10) + struct.pack('<HH', 1, machine) + bytes(44)
    header = f'{"member.o/":<16}{0:<12}{0:<6}{0:<6}{100644:<8}{len(body):<10}`\n'.encode()
    return b'!<arch>\n' + header + body + (b'\n' if len(body) % 2 else b'')


def metadata(system='Linux', arch='x86_64', distribution='ubuntu24.04', config='Release'):
    return dict(schema_version=1, package='linkerbot-hand-cpp', version='0.3.0', hand_sha='a' * 40,
                comm_sha='b' * 40, system=system, architecture=arch, distribution=distribution,
                configuration=config, compiler='GNU 13.3.0' if system == 'Linux' else 'MSVC 19.44.35229.0',
                standard_library='libstdcxx' if system == 'Linux' else 'msvc',
                libstdcxx_dual_abi='1' if system == 'Linux' else 'unknown',
                toolset='' if system == 'Linux' else '143',
                runtime='platform-default' if system == 'Linux' else 'MDd' if config == 'Debug' else 'MD',
                pic=True, cxx_standard=17, source_dirty=False, instrumented=False)


class ReleaseMatrixTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.assets = self.root / 'assets'
        self.assets.mkdir()
        self.targets_path = ROOT / 'ci/package-targets.json'
        self.targets = load_targets(self.targets_path)

    def package(self, info, transport_package='linkerbot-comm'):
        prefix = self.root / 'sdk'
        if prefix.exists():
            shutil.rmtree(prefix)
        for key, library in [('hand', 'linkerbot-cpp-sdk'), ('comm', 'linkerbot-comm')]:
            doc = dict(info)
            if key == 'comm':
                doc.update(package=transport_package, version='0.3.4')
            share = prefix / 'share' / ('linkerbot-comm' if key == 'comm' else doc['package'])
            share.mkdir(parents=True)
            (share / 'build-info.json').write_text(json.dumps(doc))
            lib = prefix / 'lib'
            lib.mkdir(exist_ok=True)
            windows = info['system'] == 'Windows'
            machine = {'x86_64': 62, 'aarch64': 183, 'x64': 0x8664, 'ARM64': 0xAA64}[info['architecture']]
            name = library + '.lib' if windows else 'lib' + library + '.a'
            (lib / name).write_bytes(static_archive(machine, windows))
        archive = self.assets / archive_name(info)
        if archive.suffix == '.zip':
            with zipfile.ZipFile(archive, 'w') as packed:
                for path in prefix.rglob('*'):
                    if path.is_file():
                        packed.write(path, path.relative_to(prefix.parent))
        else:
            with tarfile.open(archive, 'w:gz') as packed:
                packed.add(prefix, arcname='sdk')
        archive.with_name(archive.name + '.sha256').write_text(
            hashlib.sha256(archive.read_bytes()).hexdigest() + '  ' + archive.name + '\n')
        return archive

    def complete_set(self, transport_package='linkerbot-comm'):
        for row in self.targets['linux']:
            self.package(metadata(arch=row['arch'], distribution=row['distribution']), transport_package)
        for row in self.targets['windows']:
            self.package(metadata('Windows', row['arch'], 'Windows', row['configuration']), transport_package)

    def collect(self):
        return collect(self.assets, self.targets_path, '0.3.0', 'a' * 40, 'b' * 40)

    def test_exact_twelve_packages_produce_twenty_six_assets(self):
        self.complete_set()
        result = self.collect()
        self.assertEqual(len(result['packages']), 12)
        self.assertEqual(len(list(self.assets.iterdir())), 26)
        self.assertEqual(len((self.assets / 'SHA256SUMS.txt').read_text().splitlines()), 25)
        for row in result['packages']:
            self.assertEqual(row['verified_objects'], {'hand': 1, 'comm': 1})

    def test_transport_033_metadata_in_legacy_share_is_accepted(self):
        self.complete_set(transport_package='linkerbot-transport')
        result = self.collect()
        self.assertEqual(len(result['packages']), 12)
        for row in result['packages']:
            self.assertEqual(row['transport']['package'], 'linkerbot-transport')
            self.assertEqual(row['transport']['version'], '0.3.4')
            self.assertEqual(row['verified_objects'], {'hand': 1, 'comm': 1})

    def test_unknown_transport_package_is_rejected(self):
        self.complete_set(transport_package='unrelated-package')
        with self.assertRaisesRegex(ValueError, 'Unknown SDK package'):
            self.collect()

    def test_missing_archive_is_rejected(self):
        self.complete_set()
        next(self.assets.glob('*.zip')).unlink()
        with self.assertRaisesRegex(ValueError, 'exactly twelve'):
            self.collect()

    def test_renamed_archive_cannot_claim_another_target(self):
        self.complete_set()
        archive = next(self.assets.glob('*.tar.gz'))
        renamed = archive.with_name('wrong-name.tar.gz')
        archive.rename(renamed)
        archive.with_name(archive.name + '.sha256').rename(renamed.with_name(renamed.name + '.sha256'))
        with self.assertRaisesRegex(ValueError, 'filename'):
            self.collect()

    def test_tampering_is_rejected(self):
        self.complete_set()
        archive = next(self.assets.glob('*.zip'))
        archive.write_bytes(archive.read_bytes() + b'changed')
        with self.assertRaisesRegex(RuntimeError, 'SHA256'):
            self.collect()

    def test_duplicate_matrix_target_is_rejected(self):
        self.targets['linux'][-1] = self.targets['linux'][0]
        path = self.root / 'targets.json'
        path.write_text(json.dumps(self.targets))
        with self.assertRaisesRegex(ValueError, 'eight'):
            load_targets(path)

    def test_wrong_member_machine_is_rejected(self):
        info = metadata('Windows', 'ARM64', 'Windows')
        lib = self.root / 'lib'
        lib.mkdir()
        (lib / 'hand.lib').write_bytes(static_archive(0x8664, True))
        with self.assertRaisesRegex(ValueError, 'architecture mismatch'):
            verify_objects(self.root, info, 'hand')

    def test_invalid_runtime_abi_and_dirty_inputs_are_rejected(self):
        for info in [dict(metadata(), libstdcxx_dual_abi='0'), dict(metadata(), source_dirty=True),
                     dict(metadata('Windows', 'ARM64', 'Windows'), runtime='MDd')]:
            with self.subTest(info=info), self.assertRaises(ValueError):
                identity(info)

    def test_hand_and_transport_versions_are_independent_but_compatible(self):
        compatible_versions('0.3.3', '0.3.4')
        compatible_versions('0.3.3', '0.3.5')
        for comm in ['0.2.9', '0.4.0']:
            with self.assertRaisesRegex(ValueError, 'same minor'):
                compatible_versions('0.3.0', comm)
        with self.assertRaises(ValueError):
            compatible_versions('0.3.3', '0.3.3')
        compatible_versions('0.3.5', '0.3.4')
        compatible_versions('0.3.6', '0.3.5')


if __name__ == '__main__':
    unittest.main()
