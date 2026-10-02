#!/usr/bin/env bash
# Start a fresh Office sensor run. No automatic movement: record_office_inline_take starts it.
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export COSMOS_VLN_ARTIFACT_DIR="$PWD/artifacts/${1:-office_$(date +%Y%m%d_%H%M%S)}"
[[ ! -e "$COSMOS_VLN_ARTIFACT_DIR" ]] || { echo "Output already exists; choose a new run name." >&2; exit 1; }
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR"
mkdir -p .run/go2w
printf 'office_inline\n' > .run/go2w/manual_context
export ROAMERX_RVIZ_CONFIG="$PWD/config/office_demo.rviz"
export GO2W_SEMANTIC_FRAME=lio_map GO2W_SEMANTIC_ODOM_TOPIC=/wavego/navigation/odom
systemctl --user stop go2w-manual.service go2w-office-physics-view.service go2w-office-rviz.service go2w-course.service go2w-office-record.service go2w-live-dense-capture.service go2w-live-dense-mapping.service go2w-semantic-map.service go2w-replay-router.service >/dev/null 2>&1 || true
bash scripts/start_go2w_office_inline.sh > "$COSMOS_VLN_ARTIFACT_DIR/startup.log" 2>&1
/usr/bin/python3 scripts/office_ue_console.py 'LoadPackage /Game/BluePrint/BP_SceneLoader'
/usr/bin/python3 scripts/office_ue_console.py 'summon /Game/BluePrint/BP_SceneLoader.BP_SceneLoader_C'
/usr/bin/python3 scripts/validate_go2w_office_inline_ue.py > "$COSMOS_VLN_ARTIFACT_DIR/ue_actor_validation.json"
launch_service() {
 local unit_name="$1" command_text="$2" output_name="$3"
 systemd-run --user --quiet --collect --unit="$unit_name" --property=KillMode=control-group --property=KillSignal=SIGINT --property=TimeoutStopSec=60 --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --setenv="GO2W_MAPPING_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR/mapping" --setenv="OFFICE_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR" /bin/bash -c "source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec $command_text"
}
launch_service go2w-live-dense-mapping 'ros2 launch scripts/office_live/dense_mapping.launch.py' mapping
launch_service go2w-semantic-map '/usr/bin/python3 scripts/office_live/semantic_compact.py --output "$OFFICE_OUTPUT/semantic"' semantic
launch_service go2w-live-dense-capture '/usr/bin/python3 scripts/office_live/live_capture.py --output "$OFFICE_OUTPUT/capture"' capture
bash scripts/show_go2w_four_windows.sh
printf 'Ready. Record with: bash scripts/record_office_inline_take.sh %q\n' "$COSMOS_VLN_ARTIFACT_DIR"
