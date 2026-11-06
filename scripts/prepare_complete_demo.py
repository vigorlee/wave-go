#!/usr/bin/python3
"""Attach installed workstation dependencies without starting robot services."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = Path('/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916')
GROUPS = ['matrix', 'genisom_roamerx_open/install', 'genisom_roamerx_open/build',
          'build', 'models', '.local', '.venv-anything', '.venv-door-ocr']
SKIP = {'.git', '.run', '__pycache__', 'logs', 'log', 'Saved'}

def attach(source, target, linked):
    if source.name in SKIP:
        return
    if not target.exists() and not target.is_symlink():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(source.resolve(), target_is_directory=source.is_dir())
        linked.append(str(target.relative_to(ROOT)))
        return
    if source.is_dir() and target.is_dir() and not target.is_symlink():
        for child in sorted(source.iterdir()):
            attach(child, target / child.name, linked)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--check', action='store_true', help='Read-only validation; create no dependency links.')
    args = parser.parse_args()
    source = args.source.expanduser().resolve()
    if not args.check and (source == ROOT or source.is_relative_to(ROOT) or ROOT.is_relative_to(source)):
        parser.error('Source and checkout must be separate directories')
    linked = []
    if not args.check:
        if not source.is_dir():
            parser.error('Prepared MATRiX runtime not found; use --source /path/to/prepared-runtime')
        for name in GROUPS:
            if (source / name).exists():
                attach(source / name, ROOT / name, linked)
    required = [
        '.run/go2w/native_lio_build/mapping', 'install/bin/go2w_rl_bridge',
        'genisom_roamerx_open/install/setup.bash', '.venv-anything/bin/python',
        'matrix/src/robot_mujoco/simulate/build/robot_mujoco',
        'matrix/src/UeSim/Linux/zsibot_mujoco_ue.sh',
        'matrix/src/UeSim/Linux/Engine/Content/Renderer/TessellationTable.bin',
        'matrix/src/robot_mujoco/zsibot_robots/xgw',
        'models/anything/depth-anything-v2-small/config.json',
        'models/anything/grounding-dino-tiny/config.json',
        'models/anything/sam2.1-hiera-tiny/config.json',
        'matrix/scene/scene_go2w_global_compare.json',
        'matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_office_inline.xml',
        'scripts/start_office_global_auto.sh', 'scripts/global_compare/navigation_surface.py',
        'scripts/global_compare/flat_escape.py', 'config/global_compare/door_routes.json']
    missing = [name for name in required if not (ROOT / name).exists()]
    tools = ['bash', 'systemctl', 'systemd-run', 'wmctrl', 'xwininfo', 'ffmpeg', 'rviz2']
    missing_tools = [name for name in tools if not shutil.which(name) and not (Path('/opt/ros/humble/bin')/name).is_file()]
    manifest = json.loads((ROOT/'complete_demo_manifest.json').read_text())
    bad_hashes = [row['path'] for row in manifest['files']
                  if not (ROOT/row['path']).is_file() or
                  hashlib.sha256((ROOT/row['path']).read_bytes()).hexdigest() != row['sha256']]
    report = {'checked_root': str(ROOT), 'dependency_source': str(source), 'linked_paths': linked,
              'missing_paths': missing, 'missing_tools': missing_tools, 'checksum_errors': bad_hashes,
              'services_started': [], 'passed': not (missing or missing_tools or bad_hashes)}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report['passed']:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
