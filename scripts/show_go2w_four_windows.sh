#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
if [[ "$(cat .run/go2w/manual_context 2>/dev/null)" == global_compare ]]; then source scripts/office_ros_environment.sh; fi
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
if ! wmctrl -l | grep -Fq 'Depth Anything V2 + Grounded SAM 2'; then
 VIEW_EXTRA_ENV=()
 if [[ -n "${ZENOH_SESSION_CONFIG_URI:-}" ]]; then VIEW_EXTRA_ENV+=("--setenv=ZENOH_SESSION_CONFIG_URI=$ZENOH_SESSION_CONFIG_URI"); fi
 systemctl --user stop go2w-anything-view.service >/dev/null 2>&1 || true
 systemd-run --user --quiet --collect --unit=go2w-anything-view "${VIEW_EXTRA_ENV[@]}" --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" --property="WorkingDirectory=$ROOT_DIR" /bin/bash -c 'source /opt/ros/humble/setup.bash; source genisom_roamerx_open/install/setup.bash; export ROS_DOMAIN_ID=89 RMW_IMPLEMENTATION=rmw_zenoh_cpp; exec /usr/bin/python3 scripts/go2w_anything_image_view.py'
fi
/usr/bin/python3 - <<'PY'
import subprocess,time,re,os
patterns=['MuJoCo : ',os.path.basename(os.environ.get('ROAMERX_RVIZ_CONFIG','go2w_anything_navigation.rviz')),'Depth Anything V2 + Grounded SAM 2','zsibot_mujoco_ue (64-bit']
for _ in range(30):
 rows=subprocess.check_output(['wmctrl','-l'],text=True).splitlines();ids=[next((r.split()[0] for r in rows if p in r),None) for p in patterns]
 physics=next((r.split()[0] for r in rows if 'MuJoCo Physics |' in r),None)
 if physics:ids[0]=physics
 if all(ids):break
 time.sleep(1)
if not all(ids):raise RuntimeError(f'Missing windows: {dict(zip(patterns,ids))}')
s=subprocess.check_output(['xwininfo','-root'],text=True);sw=int(re.search(r'Width:\s+(\d+)',s)[1]);sh=int(re.search(r'Height:\s+(\d+)',s)[1]);w=min(1700,(sw-260)//2);h=min(850,(sh-440)//2)
for wid,(x,y) in zip(ids,[(100,60),(sw//2+30,60),(100,sh//2+30),(sw//2+30,sh//2+30)]):
 for cmd in [['wmctrl','-ir',wid,'-b','remove,maximized_vert,maximized_horz'],['wmctrl','-ir',wid,'-e',f'0,{x},{y},{w},{h}'],['wmctrl','-ir',wid,'-b','add,above'],['wmctrl','-ia',wid]]:subprocess.run(cmd,check=False)
print('Four windows ready:',dict(zip(patterns,ids)))
PY
