#!/usr/bin/env python3
"""Isolated Go2-W DreamWaQ locomotion trial on the scanned-stair MuJoCo proxy.

This exercises the real MATRiX physics binary and existing RL controller.
It does not run Nav2 or claim navigation through the full scanned building.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
MATRIX = Path('/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo')
TELEMETRY = re.compile(r'stage=([^ ]+) pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\]')


def run(name, duration, velocity):
    output = ROOT / 'outputs' / f'imported_{name}_20260926' / 'z5_features' / 'mujoco_stair_proxy'
    output.mkdir(exist_ok=True)
    config = MATRIX / 'matrix/src/robot_mujoco/simulate/config.yaml'
    original_config = config.read_bytes()
    scene = MATRIX / 'matrix/src/robot_mujoco/zsibot_robots/go2w' / f'scene_z5_{name}_stair_proxy.xml'
    if not scene.exists():
        raise FileNotFoundError(f'Run export_z5_stair_mujoco.py first: {scene}')
    with (output / 'measured_profile.csv').open(newline='') as stream:
        measured_top = max(float(row['proxy_top_above_floor_m'])
                           for row in csv.DictReader(stream))
    # The goal tests the body on the measured upper surface.  A fixed 3.8 m
    # threshold missed a real 3.795 m upper-platform pose by 5 mm in MT.
    goal_body_height = measured_top + .25
    if subprocess.run(['pgrep', '-x', 'robot_mujoco'], capture_output=True).returncode == 0:
        raise RuntimeError('Another MATRiX physics process is running')
    if subprocess.run(['pgrep', '-x', 'go2w_rl_bridge'], capture_output=True).returncode == 0:
        raise RuntimeError('Another Go2-W RL controller is running')
    bridge_binary = MATRIX / 'install/bin/go2w_rl_bridge'
    physics_binary = MATRIX / 'matrix/src/robot_mujoco/simulate/build/robot_mujoco'
    if not bridge_binary.exists() or not physics_binary.exists():
        raise FileNotFoundError('Built Go2-W controller / MATRiX physics binary missing')
    environment = os.environ.copy()
    environment.update(GO2W_RL_TEST_VX=str(velocity), GO2W_RL_TEST_DELAY='1.5',
                       GO2W_RL_TEST_DURATION=str(duration - 3),
                       GO2W_RL_TEST_PLATFORM_DISTANCE='8.1',
                       GO2W_RL_TEST_PLATFORM_HEIGHT=str(goal_body_height),
                       GO2W_RL_STOCHASTIC='0', GO2W_RL_HISTORY_MODE='train',
                       DISPLAY=environment.get('DISPLAY', ':0'))
    build = physics_binary.parent
    environment['LD_LIBRARY_PATH'] = ':'.join([
        str(build), '/opt/ros/humble/opt/zenoh_cpp_vendor/lib',
        '/opt/ros/humble/lib', '/opt/ros/humble/lib/x86_64-linux-gnu',
        str(MATRIX / 'matrix/src/UeSim/Linux/zsibot_mujoco_ue/Binaries/Linux'),
        str(MATRIX / 'matrix/src/UeSim/Linux/Engine/Binaries/Linux'),
        environment.get('LD_LIBRARY_PATH', '')])
    config.write_text(re.sub(r'^robot_scene: .*$',
                             f'robot_scene: "go2w_lcm/{scene.name}"',
                             original_config.decode(), flags=re.MULTILINE))
    files = []
    processes = []
    started = time.monotonic()
    try:
        for filename, command, cwd in [
            ('bridge.log', ['stdbuf', '-oL', '-eL', str(bridge_binary)], MATRIX),
            ('physics.log', [str(physics_binary)], build),
        ]:
            stream = (output / filename).open('w')
            files.append(stream)
            proc = subprocess.Popen(command, cwd=cwd, env=environment,
                                    stdout=stream, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            processes.append(proc)
            if filename == 'bridge.log':
                time.sleep(1)
        print('trial_started', name, 'bridge', processes[0].pid,
              'physics', processes[1].pid, flush=True)
        stop_reason = 'duration'
        while time.monotonic() - started < duration:
            if any(p.poll() is not None for p in processes):
                stop_reason = 'process_exited'
                break
            text = (output / 'bridge.log').read_text(errors='replace')
            if 'STAIR_SUCCESS' in text:
                stop_reason = 'controller_success'
                break
            if 'stage=safety' in text:
                stop_reason = 'safety_stop'
                break
            time.sleep(.5)
        time.sleep(1)
        log = (output / 'bridge.log').read_text(errors='replace')
        records = [{'stage': m[1], 'x': float(m[2]), 'y': float(m[3]),
                    'z': float(m[4])} for m in TELEMETRY.finditer(log)]
        report = {'scene': str(scene), 'scope': 'MATRiX MuJoCo + DreamWaQ velocity trial',
                  'command_vx_m_s': velocity,
                  'goal_body_height_m': goal_body_height,
                  'measured_proxy_top_m': measured_top,
                  'nav2_action': False, 'full_building_map_collision': False,
                  'controller_policy_active': 'Policy control enabled' in log,
                  'controller_success': 'STAIR_SUCCESS' in log,
                  'stop_reason': stop_reason, 'elapsed_s': round(time.monotonic() - started, 1),
                  'samples': len(records), 'start': records[0] if records else None,
                  'end': records[-1] if records else None,
                  'max_x_m': max((r['x'] for r in records), default=None),
                  'max_z_m': max((r['z'] for r in records), default=None),
                  'safety_stop': any(r['stage'] == 'safety' for r in records)}
        (output / 'locomotion_trial.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=5)
        for stream in files:
            stream.close()
        config.write_bytes(original_config)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', choices=['12', 'mt'], default='12')
    parser.add_argument('--duration', type=int, default=55)
    parser.add_argument('--vx', type=float, default=.75)
    args = parser.parse_args()
    if not .1 <= args.vx <= 1.2:
        parser.error('--vx must be in [0.1, 1.2] m/s')
    run(args.name, args.duration, args.vx)
