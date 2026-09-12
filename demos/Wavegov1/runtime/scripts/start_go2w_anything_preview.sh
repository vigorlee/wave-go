#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
export MATRIX_ROBOT_INITIAL_X=${MATRIX_ROBOT_INITIAL_X:-0.2} MATRIX_ROBOT_INITIAL_Y=${MATRIX_ROBOT_INITIAL_Y:-0} MATRIX_ROBOT_INITIAL_YAW_DEG=${MATRIX_ROBOT_INITIAL_YAW_DEG:-90}
export MATRIX_UE_OFFSCREEN=0 MATRIX_UE_NO_RHI_THREAD=1 MATRIX_DISABLE_IMAGE_SENSORS=0
export MATRIX_CAMERA_WIDTH=960 MATRIX_CAMERA_HEIGHT=540 MATRIX_CAMERA_FREQUENCY=5
export MATRIX_DEPTH_WIDTH=640 MATRIX_DEPTH_HEIGHT=480 MATRIX_DEPTH_FREQUENCY=5
export MATRIX_MUJOCO_SCENE="${MATRIX_MUJOCO_SCENE:-go2w_lcm/scene_terrain_yard_extended_nav.xml}"
export MATRIX_UE_SCENE_OVERRIDE="${MATRIX_UE_SCENE_OVERRIDE:-$ROOT_DIR/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_extended_nav.xml}"
export MATRIX_SCENE_JSON_OVERRIDE="${MATRIX_SCENE_JSON_OVERRIDE:-$ROOT_DIR/matrix/scene/scene_go2w_extended_nav.json}"
export GO2W_WORLD_SCENE="${GO2W_WORLD_SCENE:-$ROOT_DIR/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_extended_nav.xml}"
export GO2W_SCENE_JSON="$MATRIX_SCENE_JSON_OVERRIDE" GO2W_MJ_STATE_RELAY="${GO2W_MJ_STATE_RELAY:-1}"
export ROAMERX_RVIZ_CONFIG="$ROOT_DIR/config/go2w_anything_navigation.rviz"
OUT="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/anything_live_preview}"
mkdir -p "$OUT"
/usr/bin/python3 - <<'PY'
import json, os
from pathlib import Path
p=Path('matrix/config/config.json');d=json.loads(p.read_text());s=d['robot']['sensors']['depth_sensor'];c=d['robot']['sensors']['camera'];s.update(position=c['position'].copy(),rotation=c['rotation'].copy(),topic='/go2w/depth/image_raw',cloudmode=False)
draw=os.environ.get('MATRIX_LIDAR_DRAW_POINTS','0')
if draw not in ('0','1'):
 raise ValueError('MATRIX_LIDAR_DRAW_POINTS must be 0 or 1')
lidar_type=os.environ.get('MATRIX_LIDAR_TYPE',d['robot']['sensors']['lidar'].get('sensor_type','mid360'))
if lidar_type not in ('mid360','airy'):raise ValueError('Unsupported LiDAR type')
d['robot']['sensors']['lidar'].update(sensor_type=lidar_type,frequency=10,draw_points=draw=='1',random_scan=os.environ.get('MATRIX_LIDAR_RANDOM_SCAN','0')=='1')
for component in ('roll','pitch','yaw'):
 value=os.environ.get('MATRIX_LIDAR_'+component.upper())
 if value is not None:d['robot']['sensors']['lidar']['rotation'][component]=float(value)
p.write_text(json.dumps(d,indent=2)+'\n')
PY
bash scripts/start_go2w_systemd.sh
systemctl --user stop go2w-anything-models.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-anything-models --property=KillMode=control-group \
 --property="WorkingDirectory=$ROOT_DIR" --property="StandardOutput=append:$OUT/models.log" --property="StandardError=append:$OUT/models.log" \
 /bin/bash "$ROOT_DIR/scripts/run_go2w_anything_service.sh" "$OUT/anything"
