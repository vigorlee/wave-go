#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
systemctl --user stop go2w-course.service >/dev/null 2>&1 || true
/usr/bin/python3 - <<'PY'
from pathlib import Path
import sys,copy,xml.etree.ElementTree as ET
sys.path.insert(0,'scripts')
from sync_go2w_extended_ue_scene import ue_scene_root
p=Path('matrix/src/robot_mujoco/zsibot_robots/go2w');u=Path('matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w')
r=ET.parse(p/'scene_terrain_yard_extended_nav.xml').getroot()
for b in list(r.find('worldbody').findall('body')):
 if b.get('name','').startswith('pedestrian_'):r.find('worldbody').remove(b)
r.set('model','go2w original stairs ramp obstacles');r.find('statistic').set('center','0 4 1');r.find('statistic').set('extent','12')
for dest,tree in [(p,r),(u,ue_scene_root(r))]:
 ET.indent(tree,space='  ');(dest/'scene_terrain_yard_mid360_course.xml').write_text(ET.tostring(tree,encoding='unicode')+'\n')
PY
export MATRIX_LIDAR_TYPE="${MATRIX_LIDAR_TYPE:-mid360}"
export MATRIX_LIDAR_DRAW_POINTS="${MATRIX_LIDAR_DRAW_POINTS:-0}"
export MATRIX_LIDAR_RANDOM_SCAN="${MATRIX_LIDAR_RANDOM_SCAN:-0}"
export MATRIX_LIDAR_PITCH="${MATRIX_LIDAR_PITCH:-0}" MATRIX_LIDAR_YAW="${MATRIX_LIDAR_YAW:-0}"
export MATRIX_LIDAR_ROLL="${MATRIX_LIDAR_ROLL:-0}"
export GO2W_NATIVE_PEOPLE=1 GO2W_MJ_STATE_RELAY=0 MATRIX_MAP_ID=3
export GO2W_WORLD_SCENE="$ROOT_DIR/matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_yard_mid360_course.xml"
export MATRIX_MUJOCO_SCENE=go2w_lcm/scene_terrain_yard_mid360_course.xml
export MATRIX_UE_SCENE_OVERRIDE="$ROOT_DIR/matrix/src/UeSim/Linux/zsibot_mujoco_ue/Content/model/go2w/scene_terrain_yard_mid360_course.xml"
export MATRIX_WORLD_MODEL_VALIDATOR="$ROOT_DIR/scripts/validate_go2w_three_scenes.py"
export MATRIX_SCENE_JSON_OVERRIDE="$ROOT_DIR/matrix/scene/scene_go2w_static_course.json"
export ROAMERX_MAP="$ROOT_DIR/genisom_roamerx_open/map/map.yaml"
export MATRIX_ROBOT_INITIAL_X=.65 MATRIX_ROBOT_INITIAL_Y=0 MATRIX_ROBOT_INITIAL_YAW_DEG=90
export GO2W_RL_STAIR_ROUTE_CENTER_X=.65
export GO2W_RL_STOCHASTIC=1
export COSMOS_VLN_ARTIFACT_DIR="${COSMOS_VLN_ARTIFACT_DIR:-$ROOT_DIR/artifacts/three_scenes_live_$(date +%Y%m%d_%H%M%S)}"
bash scripts/start_go2w_anything_preview.sh
bash scripts/show_go2w_four_windows.sh
