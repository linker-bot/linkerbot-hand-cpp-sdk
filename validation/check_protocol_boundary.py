#!/usr/bin/env python3
"""Verify protocol entry headers transitively exclude runtime and I/O headers."""
import argparse, pathlib, subprocess, shlex, re
p = argparse.ArgumentParser()
p.add_argument('--source', type=pathlib.Path, required=True)
p.add_argument('--comm-include', type=pathlib.Path, required=True)
p.add_argument('--compiler', default='c++')
a = p.parse_args()
entries = ['protocol/transaction.hpp', 'o6/protocol_backend.hpp', 'l6/protocol_backend.hpp',
           'o30/transaction.hpp', 'o30/v25.hpp', 'l30/protocol.hpp', 'l6/hop_protocol.hpp', 'protocol/hop.hpp']
# Additional model plans are included once introduced by their migration.
for model in ['l20lite', 'l20', 'l30']:
    for name in ['protocol_backend.hpp', 'transaction.hpp']:
        if (a.source / 'include/linkerbot/hand' / model / name).exists(): entries.append(model + '/' + name)
units = [(header, None) for header in entries]
for model in ['l6', 'o6', 'l20lite', 'l20', 'l30', 'o30']:
    for filename in ['protocol.cpp', 'protocol_backend.cpp', 'transaction.cpp']:
        source = a.source / 'src/hand' / model / filename
        if source.exists(): units.append((str(source), source))
for header, source in units:
    command = [a.compiler, '-std=c++17', '-MM', '-x', 'c++',
        '-I', str(a.source / 'include'), '-I', str(a.comm_include)]
    if source:
        output = subprocess.check_output(command + [str(source)], text=True)
    else:
        output = subprocess.check_output(command + ['-'],
            input=f'#include <linkerbot/hand/{header}>\n', text=True)
    deps = shlex.split(output.replace('\\\n', ' '))[1:]
    forbidden = [d for d in deps if '/transport/' in d or '/hand/runtime/' in d or
                 '/can/' in d and not d.endswith(('/message.hpp', '/frame.hpp', '/errors.hpp'))]
    if forbidden:
        raise SystemExit(f'{header}: forbidden protocol dependency: {forbidden}')
    # Check execution primitives in our transitive code too: a mutex hidden in
    # an assembler is still an execution dependency even without an I/O header.
    for dependency in deps:
        path = pathlib.Path(dependency)
        if not path.is_file(): continue
        code = path.read_text()
        code = re.sub(r'/\*.*?\*/|//[^\n]*', '', code, flags=re.S)
        if re.search(r'#\s*include\s*[<"](?:thread|mutex|shared_mutex|condition_variable|future|semaphore)[>"]', code):
            raise SystemExit(f'{header}: execution header in {path}')
        code = re.sub(r'"(?:\\.|[^"\\])*"', '""', code)
        if re.search(r'\bstd::(?:thread|jthread|mutex|timed_mutex|recursive_mutex|shared_mutex|condition_variable|this_thread|async)\b|\b(?:steady_clock|system_clock|Clock)::now\s*\(|\b(?:sleep_for|sleep_until|wait_for|wait_until)\s*\(', code):
            raise SystemExit(f'{header}: execution primitive in {path}')
    print(header, 'OK')
