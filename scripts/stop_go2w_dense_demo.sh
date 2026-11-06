#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
systemctl --user stop go2w-dense-observers.service >/dev/null 2>&1 || true
# Match only this project's known observer command and working directory.
/usr/bin/python3 - "$ROOT_DIR" <<'PY'
import os,signal,sys
from pathlib import Path
root=Path(sys.argv[1]).resolve()
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0')
  if (p/'cwd').resolve()!=root:continue
  if len(args)>1 and args[1].decode() in ('scripts/go2w_dense_depth.py','scripts/go2w_live_perception.py','scripts/capture_go2w_semantic_cloud.py'):
   os.kill(int(p.name),signal.SIGTERM)
 except (OSError,UnicodeError):pass
PY
bash "$ROOT_DIR/scripts/stop_demo.sh"
