#!/usr/bin/env python3
"""Build a project-private native LIO executable; never install or start services."""
import argparse
import concurrent.futures
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / '.run/go2w/native_lio_build'
BUILD = ROOT / 'genisom_roamerx_open/build/robot_slam'
SOURCE = ROOT / 'genisom_roamerx_open/src/slam/src'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    return {str(p.relative_to(ROOT)): digest(p) for p in sorted(SOURCE.rglob('*'))
            if p.is_file() and p.suffix in ('.cpp', '.cc', '.h', '.hpp', '.hxx')}


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def adopt(manifest_path, output, before):
    evidence = json.loads(manifest_path.read_text())
    executable = Path(evidence['executable']).resolve()
    if evidence.get('link_exit_code') != 0 or any(evidence.get('compile_exit_codes', [1])):
        raise ValueError('Verified build manifest must record successful compilation and linkage')
    if digest(executable) != evidence['executable_sha256']:
        raise ValueError('Verified executable SHA256 differs from the manifest')
    required = {ROOT / 'genisom_roamerx_open/src/slam/src/src/mapping_alg.cpp',
                ROOT / 'genisom_roamerx_open/src/slam/src/src/process/imu_process.cpp',
                ROOT / 'genisom_roamerx_open/src/slam/src/include/mapping_alg.h'}
    recorded = evidence.get('source_sha256', {})
    if not required.issubset({Path(p).resolve() for p in recorded}):
        raise ValueError('Verified manifest does not include both changed sources and their header')
    for name, expected in recorded.items():
        if digest(Path(name)) != expected:
            raise ValueError('Source changed since verified build: ' + name)
    temporary = output / 'mapping.tmp'
    shutil.copy2(executable, temporary)
    if source_hashes() != before:
        temporary.unlink()
        raise ValueError('Native source changed while adopting the executable')
    temporary.replace(output / 'mapping')
    return {'mode': 'adopt_verified_private_build', 'verified_build_manifest': str(manifest_path),
            'verified_manifest_sha256': digest(manifest_path), 'verified_executable': str(executable),
            'verified_changed_source_hashes': recorded,
            'object_provenance': 'Verified candidate relinked modified sources with unchanged objects from the existing native CMake build; full private rebuild is available by invoking this script without --adopt-verified-build.'}


def compile_private(output, before, jobs):
    database_path = BUILD / 'compile_commands.json'
    link_path = BUILD / 'CMakeFiles/mapping.dir/link.txt'
    entries = json.loads(database_path.read_text())
    link = shlex.split(link_path.read_text())
    native_objects = [arg for arg in link if arg.endswith('.o')]
    commands = []
    fingerprint = hashlib.sha256(json.dumps(before, sort_keys=True).encode()).hexdigest()
    cache = output / 'objects'
    cache.mkdir(exist_ok=True)
    manifest_commands = []
    for old_object in native_objects:
        absolute = (BUILD / old_object).resolve() if not Path(old_object).is_absolute() else Path(old_object).resolve()
        matches = []
        for entry in entries:
            args = shlex.split(entry['command']) if 'command' in entry else entry['arguments'][:]
            destination = Path(args[args.index('-o') + 1])
            if not destination.is_absolute():
                destination = Path(entry['directory']) / destination
            if destination.resolve() == absolute:
                matches.append((entry, args))
        if len(matches) != 1:
            raise ValueError('Expected one compile command for ' + old_object)
        entry, args = matches[0]
        filename = Path(entry['file']).name + '_' + hashlib.sha256(entry['file'].encode()).hexdigest()[:8] + '.o'
        target = cache / filename
        stamp_path = target.with_suffix('.json')
        key = hashlib.sha256((fingerprint + json.dumps(args)).encode()).hexdigest()
        hit = target.exists() and stamp_path.exists() and json.loads(stamp_path.read_text()).get('key') == key
        args[args.index('-o') + 1] = str(target)
        link[link.index(old_object)] = str(target)
        manifest_commands.append({'source': entry['file'], 'arguments': args, 'cache_hit': hit})
        if not hit:
            commands.append((entry, args, target, stamp_path, key))

    def build_one(item):
        entry, args, target, stamp_path, key = item
        with target.with_suffix('.log').open('w') as log:
            result = subprocess.run(args, cwd=entry['directory'], stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError('Compile failed; inspect ' + str(target.with_suffix('.log')))
        atomic_json(stamp_path, {'key': key, 'source': entry['file'], 'object_sha256': digest(target)})

    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        list(pool.map(build_one, commands))
    temporary = output / 'mapping.tmp'
    link[link.index('-o') + 1] = str(temporary)
    with (output / 'link.log').open('w') as log:
        result = subprocess.run(link, cwd=BUILD, stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError('Link failed; inspect ' + str(output / 'link.log'))
    subprocess.run(['strip', '--strip-debug', str(temporary)], check=True)
    if source_hashes() != before:
        temporary.unlink()
        raise ValueError('Native source changed during compilation; previous executable was preserved')
    temporary.replace(output / 'mapping')
    return {'mode': 'private_compile_database_build', 'compile_database': str(database_path),
            'compile_database_sha256': digest(database_path), 'commands': manifest_commands,
            'objects_recompiled': len(commands), 'objects_total': len(native_objects),
            'link_arguments': link, 'link_exit_code': 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--jobs', type=int, default=2)
    parser.add_argument('--adopt-verified-build', type=Path,
                        help='Atomically copy an already verified private executable after checking its recorded source and executable hashes.')
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8:
        parser.error('--jobs must be 1..8')
    output = args.output.expanduser().resolve()
    installed = ROOT / 'genisom_roamerx_open/install/robot_slam/lib/robot_slam/mapping'
    if output == installed.parent or installed == output / 'mapping':
        parser.error('Refusing to replace the installed native executable')
    output.mkdir(parents=True, exist_ok=True)
    with (output / 'build.lock').open('a') as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        start = time.monotonic()
        installed_before = digest(installed) if installed.exists() else None
        before = source_hashes()
        detail = adopt(args.adopt_verified_build.resolve(), output, before) if args.adopt_verified_build else compile_private(output, before, args.jobs)
        if installed.exists() and digest(installed) != installed_before:
            raise RuntimeError('Installed executable changed independently during the build')
        manifest = {'project_root': str(ROOT), 'executable': str(output / 'mapping'),
                    'executable_sha256': digest(output / 'mapping'),
                    'executable_bytes': (output / 'mapping').stat().st_size,
                    'source_sha256': before, 'builder_sha256': digest(Path(__file__)),
                    'installed_executable': str(installed), 'installed_sha256_before': installed_before,
                    'installed_sha256_after': installed_before, 'service_actions': [],
                    'elapsed_seconds': time.monotonic() - start,
                    'diagnostic_env': 'OFFICE_LIO_TRACE_PATH (default unset/off)',
                    'accuracy_claim': 'Build and code-path hygiene only; native vertical accuracy requires a separate real-motion validation.',
                    **detail}
        atomic_json(output / 'build_manifest.json', manifest)
        print(json.dumps({key: manifest[key] for key in ('executable', 'executable_sha256', 'executable_bytes', 'mode', 'elapsed_seconds')}, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print('Native LIO private build failed: ' + str(error), file=sys.stderr)
        sys.exit(1)
