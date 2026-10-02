#!/usr/bin/env python3
"""Capture four genuine MATRiX MuJoCo window frames during the RL stair trial."""
import json
from pathlib import Path
import re
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'outputs/imported_12_20260926/z5_features/mujoco_stair_proxy'
STAGES = [('01_start', 0.0), ('02_lower_stairs', 2.5),
          ('03_upper_stairs', 5.3), ('04_upper_landing', 8.2)]
POSITION = re.compile(r'stage=([^ ]+) pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\]')


def window():
    listing = subprocess.run(['wmctrl', '-l'], capture_output=True, text=True)
    for line in listing.stdout.splitlines():
        if 'MuJoCo : z5 measured stair collision proxy' not in line:
            continue
        wid = line.split()[0]
        info = subprocess.run(['xwininfo', '-id', wid], capture_output=True, text=True)
        if info.returncode:
            continue
        width = int(re.search(r'Width:\s*(\d+)', info.stdout)[1])
        height = int(re.search(r'Height:\s*(\d+)', info.stdout)[1])
        return wid, width, height
    return None


def frame(wid, width, height, path):
    command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
               '-f', 'x11grab', '-window_id', wid,
               '-video_size', f'{width}x{height}', '-i', ':0',
               '-frames:v', '1', str(path)]
    subprocess.run(command, check=True, timeout=8)
    # Fixed crop keeps the actual robot and entire stair in view and removes
    # MuJoCo's control panels.  No simulated or synthetic pixels are added.
    crop = (f'crop={min(900, width-250)}:{min(520, height-170)}:'
            f'{min(500, width-901)}:{min(200, height-521)},scale=1600:-2')
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-i', str(path), '-vf', crop, '-frames:v', '1',
                    str(path.with_name(path.stem + '_focus.png'))],
                   check=True, timeout=8)


def main():
    frames = OUTPUT / 'live_frames'
    frames.mkdir(exist_ok=True)
    trial_log = (frames / 'trial_command.log').open('w')
    proc = subprocess.Popen(['python3', str(ROOT / 'scripts/test_z5_stair_proxy.py'),
                             '--name', '12', '--duration', '58'],
                            cwd=ROOT, stdout=trial_log, stderr=subprocess.STDOUT)
    started = time.monotonic()
    records = []
    try:
        for filename, threshold in STAGES:
            deadline = started + 65
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f'Trial ended before {filename}')
                log = (OUTPUT / 'bridge.log').read_text(errors='replace') if (
                    OUTPUT / 'bridge.log').exists() else ''
                matches = list(POSITION.finditer(log))
                view = window()
                if matches and view:
                    last = matches[-1]
                    x, y, z = map(float, last.group(2, 3, 4))
                    ready_start = (filename != '01_start' or
                                   (last.group(1) in ('policy', 'idle') and
                                    .45 < z < 1.1 and x < 1.5))
                    if x >= threshold and ready_start:
                        path = frames / (filename + '.png')
                        frame(*view, path)
                        records.append({'stage': filename, 'x_m': x, 'y_m': y,
                                        'z_m': z, 'controller_stage': last.group(1),
                                        'elapsed_wall_s': round(time.monotonic()-started, 1),
                                        'raw_image': str(path),
                                        'focus_image': str(path.with_name(
                                            path.stem + '_focus.png'))})
                        print('CAPTURE', filename, x, y, z, flush=True)
                        break
                time.sleep(.25)
            else:
                raise TimeoutError(f'No frame at {filename}')
        proc.wait(timeout=70)
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=10)
        trial_log.close()
        (frames / 'frame_provenance.json').write_text(
            json.dumps({'source': 'live MATRiX MuJoCo X11 window',
                        'collision_scene': 'z5_12 measured stair proxy',
                        'frames': records,
                        'trial_result': str(OUTPUT / 'locomotion_trial.json')},
                       ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
