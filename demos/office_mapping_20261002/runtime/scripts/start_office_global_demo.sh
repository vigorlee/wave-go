#!/usr/bin/env bash
# Start a fresh Office sensor run. The auto or recording launcher starts movement.
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/office_ros_environment.sh
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
RUN_NAME="${1:-office}"
[[ "$RUN_NAME" =~ 20[0-9]{6} ]] || RUN_NAME="${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
export COSMOS_VLN_ARTIFACT_DIR="$PWD/artifacts/$RUN_NAME"
[[ ! -e "$COSMOS_VLN_ARTIFACT_DIR" ]] || { echo "Output already exists; choose a new run name." >&2; exit 1; }
mkdir -p "$COSMOS_VLN_ARTIFACT_DIR"
mkdir -p .run/go2w
printf '%s\n' "$COSMOS_VLN_ARTIFACT_DIR" > .run/go2w/current_output_dir
printf 'global_compare\n' > .run/go2w/manual_context
export ROAMERX_RVIZ_CONFIG="$PWD/config/global_compare/office_demo.rviz"
export GO2W_SEMANTIC_FRAME=lio_map GO2W_SEMANTIC_ODOM_TOPIC=/wavego/navigation/odom
systemctl --user stop go2w-auto-handoff.service go2w-live-audit.service go2w-router-watchdog.service go2w-manual.service go2w-global-display.service go2w-office-physics-view.service go2w-office-rviz.service go2w-course.service go2w-office-record.service go2w-live-dense-capture.service go2w-live-dense-mapping.service go2w-semantic-map.service go2w-replay-router.service >/dev/null 2>&1 || true
bash scripts/start_go2w_global_compare_world.sh > "$COSMOS_VLN_ARTIFACT_DIR/startup.log" 2>&1
systemd-run --user --quiet --collect --unit=go2w-router-watchdog \
 --property=KillMode=control-group --property=Restart=always --property=RestartSec=1 \
 --property="WorkingDirectory=$PWD" \
 --setenv="ZENOH_ROUTER_CONFIG_URI=$ZENOH_ROUTER_CONFIG_URI" \
 --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/router.log" \
 --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/router.log" \
 /bin/bash "$PWD/scripts/keep_office_router.sh"
# The native blueprint uses normalized movement input. Cap its cm/s maximum
# so JSON velocity 0.5 / 0.4 produces walking speeds close to 0.5 / 0.4 m/s.
/usr/bin/python3 scripts/validate_go2w_office_inline_ue.py > "$COSMOS_VLN_ARTIFACT_DIR/ue_actor_validation.json"
/usr/bin/python3 scripts/global_compare/validate_pedestrians.py --startup-log --output "$COSMOS_VLN_ARTIFACT_DIR/pedestrian_validation.json"
bash scripts/ensure_office_door_routes.sh "$COSMOS_VLN_ARTIFACT_DIR/plans"
export OFFICE_ROUTES="$PWD/config/global_compare/door_routes.json"
launch_service() {
 local unit_name="$1" command_text="$2" output_name="$3"
 local estimator_env=()
 if [[ "$unit_name" == go2w-live-dense-mapping ]]; then
  [[ -z "${OFFICE_LIO_EXECUTABLE:-}" ]] || estimator_env+=("--setenv=OFFICE_LIO_EXECUTABLE=$OFFICE_LIO_EXECUTABLE")
  if [[ "${OFFICE_LIO_TRACE:-0}" == 1 ]]; then
   estimator_env+=("--setenv=OFFICE_LIO_TRACE_PATH=$COSMOS_VLN_ARTIFACT_DIR/native_lio_trace.csv")
  fi
 fi
 systemd-run --user --quiet --collect --unit="$unit_name" "${estimator_env[@]}" --property=KillMode=control-group --property=KillSignal=SIGINT --property=TimeoutStopSec=60 --property="WorkingDirectory=$PWD" --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/$output_name.log" --setenv="ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI" --setenv="GO2W_MAPPING_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR/mapping" --setenv="OFFICE_OUTPUT=$COSMOS_VLN_ARTIFACT_DIR" --setenv="OFFICE_ROUTES=$OFFICE_ROUTES" /bin/bash -c "source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec $command_text"
}
launch_service go2w-live-dense-mapping 'ros2 launch scripts/office_live/dense_mapping.launch.py' mapping
launch_service go2w-semantic-map '/usr/bin/python3 scripts/office_live/semantic_compact.py --output "$OFFICE_OUTPUT/semantic"' semantic
launch_service go2w-live-dense-capture '/usr/bin/python3 scripts/global_compare/dense_capture.py --output "$OFFICE_OUTPUT/capture" --alignment "$OFFICE_OUTPUT/frame_alignment.json"' capture
launch_service go2w-global-display '/usr/bin/python3 scripts/global_compare/visualize_routes.py --routes "$OFFICE_ROUTES" --alignment "$OFFICE_OUTPUT/frame_alignment.json"' global_display
bash scripts/show_go2w_four_windows.sh
printf 'Ready. Record with: bash scripts/record_office_global_take.sh %q NavFnDijkstra\n' "$COSMOS_VLN_ARTIFACT_DIR"
