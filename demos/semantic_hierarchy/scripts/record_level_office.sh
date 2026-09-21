#!/usr/bin/env bash
# Fresh OfficeWorld sensor recording and continuous simulated route; no video.
set -eo pipefail
SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_ROOT=/home/unitree/Go2w+cosmos3Nav/matrix_go2w_office_20260916
SCENE_RUN="$SCENE_ROOT/outputs/level_fix/${1:-fresh_$(date +%Y%m%d_%H%M%S)}"
[[ ! -e "$SCENE_RUN" ]] || { echo 'Choose a new run name; existing data is preserved.'; exit 1; }
[[ -x "$SCENE_ROOT/.runtime/level_lio_install/robot_slam/lib/robot_slam/mapping" ]]
mkdir -p "$SCENE_RUN/config_before"
cp "$SIM_ROOT/matrix/config/config.json" "$SCENE_RUN/config_before/"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export COSMOS_VLN_ARTIFACT_DIR="$SCENE_RUN/simulation"
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR"
cd "$SIM_ROOT"
systemctl --user stop go2w-manual.service go2w-course.service go2w-office-record.service go2w-live-dense-capture.service go2w-live-dense-mapping.service go2w-semantic-map.service go2w-replay-router.service >/dev/null 2>&1 || true
bash scripts/start_go2w_office_inline.sh > "$SCENE_RUN/startup.log" 2>&1
/usr/bin/python3 scripts/office_ue_console.py 'LoadPackage /Game/BluePrint/BP_SceneLoader' > "$SCENE_RUN/ue_console.log"
/usr/bin/python3 scripts/office_ue_console.py 'summon /Game/BluePrint/BP_SceneLoader.BP_SceneLoader_C' >> "$SCENE_RUN/ue_console.log"
/usr/bin/python3 scripts/validate_go2w_office_inline_ue.py > "$SCENE_RUN/ue_actor_validation.json"
cp matrix/config/config.json "$SCENE_RUN/sensor_config.json"
source /opt/ros/humble/setup.bash
source genisom_roamerx_open/install/setup.bash
source "$SCENE_ROOT/.runtime/level_lio_install/setup.bash"
export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp
unset ROS_LOCALHOST_ONLY
SCENE_PIDS=()
cleanup() {
  for pid in "${SCENE_PIDS[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${SCENE_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
"$SCENE_ROOT/.runtime/level_lio_install/robot_slam/lib/robot_slam/mapping" --ros-args --params-file "$SCENE_ROOT/config/level_lio.yaml" -r /world_points:=/wavego/level/registered_points -r /slam_odom:=/wavego/level/odom -r /tf:=/wavego/level/tf > "$SCENE_RUN/lio.log" 2>&1 &
SCENE_PIDS+=($!)
/usr/bin/python3 "$SCENE_ROOT/scripts/collect_level_map.py" --output "$SCENE_RUN/capture" > "$SCENE_RUN/collector.log" 2>&1 &
SCENE_PIDS+=($!)
ros2 bag record -o "$SCENE_RUN/sensors_rosbag2" /livox/lidar_raw /imu/data_raw /wavego/level/registered_points /wavego/level/odom /odom/mujoco_odom /image_raw/compressed /go2w/depth/image_raw > "$SCENE_RUN/bag.log" 2>&1 &
SCENE_PIDS+=($!)
ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 3}' > "$SCENE_RUN/start_service.log" 2>&1
# Require a real map before transferring control to the automatic route.
/usr/bin/python3 - "$SCENE_RUN" <<'PY'
import sys,json,time
from pathlib import Path
p=Path(sys.argv[1]);end=time.monotonic()+40
while time.monotonic()<end:
 try:
  if json.loads((p/'capture/status.json').read_text())['scans']>=40:break
 except (FileNotFoundError,json.JSONDecodeError):pass
 time.sleep(.5)
else:raise RuntimeError('LIO did not initialize; route was not started')
PY
if [[ -f .run/go2w/manual_control ]]; then mv .run/go2w/manual_control "$SCENE_RUN/config_before/manual_control"; fi
/usr/bin/python3 "$SCENE_ROOT/scripts/level_mapping_route.py" --output "$SCENE_RUN/course" > "$SCENE_RUN/course.log" 2>&1 &
SCENE_PIDS+=($!)
/usr/bin/python3 - "$SCENE_RUN" <<'PY'
import sys,time,json
from pathlib import Path
p=Path(sys.argv[1]);end=time.monotonic()+520;last=None
while time.monotonic()<end:
 try:
  r=json.loads((p/'course/result.json').read_text());state=(r['state'],r['completed_stages'])
  if state!=last:print(state,flush=True);last=state
  if state[0] in ('succeeded','failed'):break
 except (FileNotFoundError,json.JSONDecodeError):pass
 time.sleep(1)
else:raise RuntimeError('Route monitor timed out')
time.sleep(5)
PY
