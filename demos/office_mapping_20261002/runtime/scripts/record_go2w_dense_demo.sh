#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/dense_depth_$(date +%Y%m%d_%H%M%S)}"
export ROAMERX_RVIZ_CONFIG="${ROAMERX_RVIZ_CONFIG:-$ROOT_DIR/config/go2w_semantic_navigation.rviz}"
export MATRIX_DEPTH_WIDTH=640 MATRIX_DEPTH_HEIGHT=480 MATRIX_DEPTH_FREQUENCY=5
export MATRIX_UE_NO_RHI_THREAD=1 ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/genisom_roamerx_open/install/setup.bash"
set -u
/usr/bin/python3 - <<'PYCONFIG'
import json,os
from pathlib import Path
p=Path('matrix/config/config.json');d=json.loads(p.read_text())
s=d['robot']['sensors']['depth_sensor'];s.update(width=640,height=480,frequency=5,cloudmode=False,topic='/go2w/depth/image_raw',fov=90,position=dict(x=13.011,y=2.329,z=17.598),rotation=dict(roll=0,pitch=0,yaw=0))
if os.environ.get('GO2W_PRETRAINED')=='1':
 s['position']=d['robot']['sensors']['camera']['position'].copy();s['rotation']=d['robot']['sensors']['camera']['rotation'].copy()
p.write_text(json.dumps(d,indent=2)+'\n')
# Adapter starts before UE, so its initial subscription uses the same topic.
t=Path('matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/config/config.json');t.write_text(p.read_text())
PYCONFIG
pids=()
cleanup() {
  local status=$?
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; done
  if [[ "${GO2W_KEEP_LIVE:-0}" != 1 || "$status" != 0 ]]; then
    systemctl --user stop go2w-dense-observers.service >/dev/null 2>&1 || true
  fi
  return "$status"
}
trap cleanup EXIT
/usr/bin/python3 scripts/capture_go2w_semantic_cloud.py --output "$COSMOS_VLN_ARTIFACT_DIR/pointcloud" --scene "$ROOT_DIR/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml" > "$COSMOS_VLN_ARTIFACT_DIR/semantic_capture.log" 2>&1 &
pids+=("$!")
systemctl --user stop go2w-dense-observers.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-dense-observers --setenv="GO2W_PRETRAINED=${GO2W_PRETRAINED:-0}" \
  --property=KillMode=control-group --property="WorkingDirectory=$ROOT_DIR" \
  --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/observers.log" \
  --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/observers.log" \
  /bin/bash "$ROOT_DIR/scripts/run_go2w_dense_observers.sh"
bash scripts/record_cosmos_extended_desktop_demo.sh
/usr/bin/python3 scripts/render_go2w_semantic_evidence.py "$COSMOS_VLN_ARTIFACT_DIR/pointcloud"

/usr/bin/python3 scripts/validate_go2w_dense_evidence.py "$COSMOS_VLN_ARTIFACT_DIR" > "$COSMOS_VLN_ARTIFACT_DIR/evidence_validation.log"
