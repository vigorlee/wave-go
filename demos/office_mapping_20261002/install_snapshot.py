#!/usr/bin/env python3
"""Verify the source snapshot; optionally install it over a prepared simulator."""
import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((ROOT/'source_manifest.json').read_text())
    sources = []
    for row in manifest['files']:
        source = (ROOT/row['path']).resolve()
        if not source.is_relative_to(ROOT):
            raise ValueError('Manifest path escapes snapshot')
        if hashlib.sha256(source.read_bytes()).hexdigest() != row['packaged_sha256']:
            raise ValueError(f"Checksum mismatch: {row['path']}")
        if row['path'].startswith('runtime/'):
            sources.append((source, source.relative_to(ROOT/'runtime')))
    print(f"Verified {len(manifest['files'])} source/data/evidence files; {len(sources)} runtime files.")
    if args.runtime is None:
        if args.apply:
            parser.error('--apply requires --runtime')
        return
    target = args.runtime.expanduser().resolve()
    if target == ROOT/'runtime' or target.is_relative_to(ROOT):
        raise ValueError('Installation target must be outside this snapshot')
    required = ['matrix/src/robot_mujoco/zsibot_robots/go2w/go2w.xml',
                'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w',
                'genisom_roamerx_open/install/setup.bash']
    for name in required:
        if not (target/name).exists():
            raise FileNotFoundError(f'Prepared runtime dependency missing: {target/name}')
    changes = []
    for source, relative in sources:
        destination = target/relative
        if not destination.exists() or destination.read_bytes() != source.read_bytes():
            if args.apply and not destination.resolve().is_relative_to(target):
                raise ValueError(f'Refusing to replace a shared external dependency: {relative}')
            changes.append((source, relative, destination))
    print(f'{len(changes)} files differ at {target}')
    if not args.apply:
        print('Check only; no files modified. --apply backs up differences before copying.')
        return
    for unit in ['go2w-demo', 'go2w-course', 'go2w-live-dense-mapping',
                 'go2w-live-dense-capture', 'go2w-anything-models', 'go2w-semantic-map']:
        if subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit+'.service']).returncode == 0:
            raise RuntimeError(f'Stop {unit}.service before replacing its source')
    backup = target/'.backups'/('mapping_20261002_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    for source, relative, destination in changes:
        if destination.exists():
            saved = backup/relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(destination, saved)
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    for source, relative in sources:
        destination = target/relative
        if destination.resolve().is_relative_to(target):
            shutil.copymode(source, destination)
    print(f'Installed {len(changes)} files. Backup: {backup}')
    print('Rebuild the bridge and robot_slam before live simulation; see README.md.')


if __name__ == '__main__':
    main()
