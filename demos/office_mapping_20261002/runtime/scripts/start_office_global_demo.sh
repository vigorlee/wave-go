#!/usr/bin/env bash
# Start a fresh Office sensor run. No automatic movement: record_office_inline_take starts it.
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
RUN_NAME="${1:-office}"
[[ "$RUN_NAME" =~ 20[0-9]{6} ]] || RUN_NAME="${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
export COSMOS_VLN_ARTIFACT_DIR="$PWD/artifacts/$RUN_NAME"
[[ ! -e "$COSMOS_VLN_ARTIFACT_DIR" ]] || { echo "Output already exists; choose a new run name." >&2; exit 1; }
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR"
mkdir -p .run/go2w
printf 'global_compare\n' > .run/go2w/manual_context
export ROAMERX_RVIZ_CONFIG="$PWD/config/global_compare/office_demo.rviz"
export GO2W_SEMANTIC_FRAME=lio_map GO2W_SEMANTIC_ODOM_TOPIC=/wavego/navigation/odom
systemctl --user stop go2w-manual.service go2w-global-display.service go2w-office-physics-view.service go2w-office-rviz.service go2w-course.service go2w-office-record.service go2w-live-dense-capture.service go2w-live-dense-mapping.service go2w-semantic-map.service go2w-replay-router.service >/dev/null 2>&1 || true
bash scripts/start_go2w_global_compare_world.sh > "$COSMOS_VLN_ARTIFACT_DIR/startup.log" 2>&1
/usr/bin/python3 scripts/global_compare/ue_console.py 'LoadPackage /Game/BluePrint/BP_SceneLoader'
/usr/bin/python3 scripts/global_compare/ue_console.py 'summon /Game/BluePrint/BP_SceneLoader.BP_SceneLoader_C'
# The native blueprint uses normalized movement input. Cap its cm/s maximum
# so JSON velocity 0.5 / 0.4 produces walking speeds close to 0.5 / 0.4 m/s.
/usr/bin/python3 scripts/global_compare/ue_console.py 'set CharacterMovementComponent MaxWalkSpeed 100'
/usr/bin/python3 scripts/global_compare/ue_console.py 'set BP_Human_C DisThresh 15'
/usr/bin/python3 scripts/validate_go2w_office_inline_ue.py > "$COSMOS_VLN_ARTIFACT_DIR/ue_actor_validation.json"
/usr/bin/python3 scripts/global_compare/validate_pedestrians.py --output "$COSMOS_VLN_ARTIFACT_DIR/pedestrian_validation.json"
launch_service() {
 local unit_name="$1" command_text="$2" output_name="$3"
 systemd-run --user --quiet --collect --unit="$unit_name" --property=KillMode=control-group --property=KillSignal=SIGINT --property=TimeoutStopSec=60 --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --setenv="GO2W_MAPPING_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR/mapping" --setenv="OFFICE_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR" /bin/bash -c "source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec $command_text"
}
launch_service go2w-live-dense-mapping 'ros2 launch scripts/office_live/dense_mapping.launch.py' mapping
launch_service go2w-semantic-map '/usr/bin/python3 scripts/office_live/semantic_compact.py --output "$OFFICE_OUTPUT/semantic"' semantic
launch_service go2w-live-dense-capture '/usr/bin/python3 scripts/global_compare/dense_capture.py --output "$OFFICE_OUTPUT/capture" --alignment "$OFFICE_OUTPUT/frame_alignment.json"' capture
launch_service go2w-global-display '/usr/bin/python3 scripts/global_compare/visualize_routes.py --routes artifacts/global_compare_20261001/plans/routes.json --alignment "$OFFICE_OUTPUT/frame_alignment.json"' global_display
bash scripts/show_go2w_four_windows.sh
printf 'Ready. Record with: bash scripts/record_office_global_take.sh %q NavFnDijkstra\n' "$COSMOS_VLN_ARTIFACT_DIR"
