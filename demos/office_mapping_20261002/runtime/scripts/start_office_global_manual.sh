#!/usr/bin/env bash
# Start a fresh OfficeWorld run and give its Go2-W to the focused keyboard panel.
set -euo pipefail
cd "$(dirname "$0")/.."
RUN_NAME="${1:-office_manual}"
[[ "$RUN_NAME" =~ 20[0-9]{6} ]] || RUN_NAME="${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
export COSMOS_VLN_ARTIFACT_DIR="$PWD/artifacts/$RUN_NAME"

# The saved-map review runs on a separate ROS domain; close only its viewer
# before opening the live four-window simulation.
systemctl --user stop go2w-lio-rviz-20261002.service go2w-lio-review-20261002.service >/dev/null 2>&1 || true
bash scripts/start_office_global_demo.sh "$RUN_NAME"

systemctl --user stop go2w-course.service go2w-manual.service >/dev/null 2>&1 || true
systemd-run --user --quiet --collect --unit=go2w-manual \
  --property=KillMode=control-group --property="WorkingDirectory=$PWD" \
  --property="StandardOutput=append:$COSMOS_VLN_ARTIFACT_DIR/manual.log" \
  --property="StandardError=append:$COSMOS_VLN_ARTIFACT_DIR/manual.log" \
  /bin/bash "$PWD/scripts/start_go2w_manual.sh"

for _ in {1..30}; do
  if systemctl --user is-active --quiet go2w-manual.service &&
     wmctrl -l | grep -Fq 'Go2-W 手动驾驶'; then
    printf '手动控制已打开：%s\n' "$COSMOS_VLN_ARTIFACT_DIR"
    exit 0
  fi
  sleep 1
done
echo '手动面板未就绪，查看当前运行目录下的 manual.log' >&2
exit 1
