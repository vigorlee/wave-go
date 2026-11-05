#!/usr/bin/env python3
"""Record four selected experiment windows without capturing the desktop."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from typing import Any


FPS = 12
OUTPUT_SIZE = (1920, 1080)
FONT = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
WINDOWS = (
    ('MuJoCo Physics |', '01  MuJoCo - robot motion'),
    ('office_demo.rviz', '02  RViz - dense 3D map and navigation'),
    ('Depth Anything V2 + Grounded SAM 2', '03  Depth Anything V2 + Grounded SAM 2'),
    ('zsibot_mujoco_ue (64-bit', '04  OfficeWorld - RGB and pedestrians'),
)


def positive_duration(value: str) -> float:
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise argparse.ArgumentTypeError('duration must be a positive, finite number')
    return duration


def resolve_windows() -> list[dict[str, Any]]:
    """Read IDs and client sizes; never raise, reposition, or resize windows."""
    rows = subprocess.check_output(['wmctrl', '-l'], text=True).splitlines()
    sources = []
    for pattern, label in WINDOWS:
        matches = [row for row in rows if pattern in row]
        if len(matches) != 1:
            raise RuntimeError(
                f'Expected exactly one window matching {pattern!r}, found {len(matches)}'
            )
        row = matches[0]
        window_hex = row.split()[0]
        window_id = int(window_hex, 16)
        if window_id == 0:
            raise RuntimeError('Root/desktop window ID 0 is prohibited')
        info = subprocess.check_output(['xwininfo', '-id', window_hex], text=True)
        width = re.search(r'^\s*Width:\s+(\d+)', info, re.MULTILINE)
        height = re.search(r'^\s*Height:\s+(\d+)', info, re.MULTILINE)
        if not width or not height:
            raise RuntimeError(f'Cannot determine client size for {pattern}')
        if not re.search(r'Map State:\s+IsViewable', info):
            raise RuntimeError(f'Window is hidden or minimized: {pattern}')
        sources.append({
            'window_id_hex': window_hex,
            'window_id_decimal': window_id,
            'title': row.split(None, 3)[3],
            'match': pattern,
            'label': label,
            'width': int(width.group(1)),
            'height': int(height.group(1)),
        })
    return sources


def build_command(
    sources: list[dict[str, Any]], display: str, output: Path, duration: float,
) -> list[str]:
    if len(sources) != 4:
        raise ValueError('Exactly four selected window inputs are required')
    command = [
        'ffmpeg', '-hide_banner', '-nostdin', '-loglevel', 'warning', '-n',
        '-filter_complex_threads', '2',
    ]
    filters = []
    for index, source in enumerate(sources):
        if int(source['window_id_decimal']) <= 0:
            raise ValueError('Root/desktop capture is prohibited')
        if int(source['width']) <= 0 or int(source['height']) <= 0:
            raise ValueError('Window client dimensions must be positive')
        command.extend([
            '-thread_queue_size', '64', '-f', 'x11grab',
            '-window_id', str(source['window_id_decimal']),
            '-draw_mouse', '0', '-framerate', str(FPS),
            '-video_size', f"{source['width']}x{source['height']}",
            '-i', display,
        ])
        # Labels are fixed project strings, rather than untrusted window titles.
        filters.append(
            f'[{index}:v]setpts=PTS-STARTPTS,fps={FPS},'
            'scale=960:504:force_original_aspect_ratio=decrease:force_divisible_by=2,'
            'pad=960:540:(ow-iw)/2:36+(504-ih)/2:color=0x0b1421,setsar=1,'
            f"drawtext=fontfile={FONT}:text='{WINDOWS[index][1]}':"
            f'x=18:y=9:fontsize=18:fontcolor=0xe7f1ff[q{index}]'
        )
    filters.append(
        '[q0][q1][q2][q3]xstack=inputs=4:'
        'layout=0_0|960_0|0_540|960_540:shortest=1[out]'
    )
    command.extend([
        '-filter_complex', ';'.join(filters), '-map', '[out]', '-an',
        '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '21',
        '-pix_fmt', 'yuv420p', '-r', str(FPS), '-movflags', '+faststart',
        '-t', f'{duration:g}', str(output),
    ])
    return command


def save_manifest(path: Path, manifest: dict[str, Any]) -> None:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


def verify_container(path: Path) -> dict[str, Any]:
    """A nonempty file may still lack its MP4 trailer; require a readable stream."""
    completed = subprocess.run(
        ['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
         'stream=width,height,avg_frame_rate,nb_frames:format=duration', '-of', 'json',
         str(path)], capture_output=True, text=True, timeout=10, check=True,
    )
    info = json.loads(completed.stdout)
    stream = info['streams'][0]
    duration = float(info['format']['duration'])
    if (stream['width'], stream['height']) != OUTPUT_SIZE or not duration > 0:
        raise ValueError('Recorded video dimensions or duration are invalid')
    return dict(width=stream['width'], height=stream['height'], duration_seconds=duration,
                average_frame_rate=stream['avg_frame_rate'],
                frames=int(stream['nb_frames']) if stream.get('nb_frames', '').isdigit() else None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True,
                        help='Explicit directory for this recording and its metadata')
    parser.add_argument('--duration', type=positive_duration, default=300.,
                        help='Maximum duration in seconds (default: 300)')
    parser.add_argument('--display', default=os.environ.get('DISPLAY', ':0'))
    parser.add_argument('--dry-run', action='store_true',
                        help='Validate the four windows and print the command; do not record')
    args = parser.parse_args()

    try:
        for binary in ('ffmpeg', 'ffprobe', 'wmctrl', 'xwininfo'):
            if not shutil.which(binary):
                raise RuntimeError(f'Missing required executable: {binary}')
        if not FONT.is_file():
            raise RuntimeError(f'Label font not found: {FONT}')
        sources = resolve_windows()
    except (OSError, subprocess.CalledProcessError, RuntimeError) as exc:
        parser.exit(1, f'Recording was not started: {exc}\n')

    output_dir = args.output_dir.expanduser().resolve()
    # User-facing run labels always use the project's Asia/Shanghai timezone.
    start = dt.datetime.now(dt.timezone(dt.timedelta(hours=8)))
    stamp = start.strftime('%Y%m%d_%H%M%S_%f')[:-3]
    output = output_dir / f'office_four_windows_{stamp}.mp4'
    manifest_path = output_dir / f'office_four_windows_{stamp}_manifest.json'
    command_path = output_dir / f'office_four_windows_{stamp}_command.sh'
    log_path = output_dir / f'office_four_windows_{stamp}_ffmpeg.log'
    command = build_command(sources, args.display, output, args.duration)
    manifest = {
        'status': 'prepared', 'started_at': start.isoformat(),
        'source': 'Four selected X11 window drawable IDs; no root or desktop input',
        'timing': 'Independent window streams aligned to one output timeline; '
                  'not hardware-synchronized sensor recording',
        'window_position_changes': False,
        'windows': sources, 'output': str(output),
        'output_dimensions': list(OUTPUT_SIZE), 'fps': FPS,
        'duration_limit_seconds': args.duration,
        'command': command, 'ffmpeg_log': str(log_path),
    }
    if args.dry_run:
        print(json.dumps(manifest, indent=2, ensure_ascii=False))
        print(shlex.join(command))
        return 0

    output_dir.mkdir(parents=True, exist_ok=True)
    command_path.write_text('#!/usr/bin/env bash\nexec ' + shlex.join(command) + '\n')
    save_manifest(manifest_path, manifest)
    print(f'Recording four selected windows at {FPS} fps: {output}', flush=True)
    print(f'Manifest: {manifest_path}', flush=True)
    started = time.monotonic()
    stop_requested = False
    old_handlers = {}
    with log_path.open('w') as log:
        # A terminal Ctrl-C must reach this Python supervisor only. Under systemd,
        # KillMode=mixed has the same first-signal ownership. The child remains in
        # the unit's cgroup for final forced cleanup if it ever hangs.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)

        def request_stop(signum: int, _frame: Any) -> None:
            nonlocal stop_requested
            if stop_requested:
                return
            stop_requested = True
            if process.poll() is None:
                # SIGINT lets ffmpeg finish the MP4 trailer before exiting.
                process.send_signal(signal.SIGINT)

        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        manifest.update(status='recording', ffmpeg_pid=process.pid,
                        ffmpeg_has_separate_session=True,
                        stop_policy='Supervisor forwards SIGINT once; systemd KillMode=mixed')
        save_manifest(manifest_path, manifest)
        try:
            return_code = process.wait()
        finally:
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)
    manifest.update(
        status='stopped_by_request' if stop_requested else
               ('completed' if return_code == 0 else 'failed'),
        exit_code=return_code,
        elapsed_wall_seconds=round(time.monotonic() - started, 3),
        ended_at=dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
        output_bytes=output.stat().st_size if output.exists() else 0,
    )
    save_manifest(manifest_path, manifest)
    if return_code != 0 and not stop_requested:
        print(f'Recording failed ({return_code}); inspect {log_path}', file=sys.stderr)
        return 1
    if not output.exists() or output.stat().st_size == 0:
        print(f'No video was written; inspect {log_path}', file=sys.stderr)
        return 1
    try:
        manifest['video_validation'] = verify_container(output)
        manifest['container_valid'] = True
    except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError) as exc:
        manifest.update(status='failed', container_valid=False,
                        validation_error=str(exc))
        save_manifest(manifest_path, manifest)
        print(f'Video container could not be read; inspect {log_path}', file=sys.stderr)
        return 1
    save_manifest(manifest_path, manifest)
    print(f'Video saved: {output}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
