#!/usr/bin/env bash
set -euo pipefail

# ros2 run starts the publisher through a Python wrapper. Killing only that
# wrapper can leave the real publisher adopted by the user systemd process.
# Match the executable itself so a parent shell containing these arguments is
# never mistaken for a publisher.
pids=()
domain_id="${ROS_DOMAIN_ID:-89}"

for proc_dir in /proc/[0-9]*; do
  pid="${proc_dir##*/}"
  executable="$(readlink -f "${proc_dir}/exe" 2>/dev/null || true)"
  [[ "${executable##*/}" == "static_transform_publisher" ]] || continue
  [[ -r "/proc/${pid}/cmdline" ]] || continue
  if ! tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null | \
       grep -Fxq "ROS_DOMAIN_ID=${domain_id}"; then
    continue
  fi
  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline")"
  if [[ "${cmdline}" == *"static_transform_publisher"* &&
        ( "${cmdline}" == *"--frame-id base_link"* ||
          "${cmdline}" == *"--frame-id=base_link"* ) &&
        ( "${cmdline}" == *"--child-frame-id lidar"* ||
          "${cmdline}" == *"--child-frame-id=lidar"* ) ]]; then
    pids+=("${pid}")
  fi
done

if (( ${#pids[@]} == 0 )); then
  exit 0
fi

kill -TERM "${pids[@]}" 2>/dev/null || true
for _ in {1..20}; do
  alive=()
  for pid in "${pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null && alive+=("${pid}")
  done
  (( ${#alive[@]} == 0 )) && exit 0
  sleep 0.05
done

kill -KILL "${alive[@]}" 2>/dev/null || true
