#!/usr/bin/env python3
"""Save one dated, lossless four-window experiment screenshot on demand."""

import argparse
import datetime as dt
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parents[1]
WINDOWS = (
    ("MuJoCo", "MuJoCo : "),
    ("RViz | 3D point cloud", "office_demo.rviz"),
    ("Depth Anything V2 + Grounded SAM 2", "Depth Anything V2 + Grounded SAM 2"),
    ("OfficeWorld | UE", "zsibot_mujoco_ue (64-bit"),
)
TILE_W, TILE_H, HEADER_H = 1920, 1080, 56


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, help="Save here instead of the current dated run")
    args = parser.parse_args()

    if args.output_dir:
        output_dir = args.output_dir.expanduser().resolve()
    elif (ROOT / '.run/go2w/current_output_dir').is_file():
        output_dir = Path((ROOT / '.run/go2w/current_output_dir').read_text().strip()) / 'screenshots'
    else:
        runs = list((ROOT / "artifacts").glob("office_manual_20*"))
        if not runs:
            parser.error("No current run found; start the four-window demo first")
        output_dir = max(runs, key=lambda p: p.stat().st_mtime) / "screenshots"
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = subprocess.check_output(["wmctrl", "-l"], text=True).splitlines()
    ids = []
    for label, pattern in WINDOWS:
        matches = [row.split()[0] for row in rows if pattern in row]
        if label == 'MuJoCo':
            matches = [row.split()[0] for row in rows if 'MuJoCo Physics |' in row] or matches
        if not matches:
            parser.error(f"Missing experiment window: {label}")
        ids.append(matches[0])

    timestamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    canvas = Image.new("RGB", (TILE_W * 2, TILE_H * 2), "#0b1220")
    draw = ImageDraw.Draw(canvas)
    font_file = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_file, 32)
    small_font = ImageFont.truetype(font_file, 23)
    with tempfile.TemporaryDirectory(prefix="go2w_capture_") as temp:
        for index, ((label, _), window_id) in enumerate(zip(WINDOWS, ids)):
            frame_path = Path(temp) / f"window_{index}.png"
            subprocess.run(
                ["ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                 "-f", "x11grab", "-window_id", str(int(window_id, 16)),
                 "-draw_mouse", "0", "-i", ":0", "-frames:v", "1",
                 "-update", "1", "-y", str(frame_path)],
                check=True,
            )
            x, y = (index % 2) * TILE_W, (index // 2) * TILE_H
            draw.rectangle((x, y, x + TILE_W - 1, y + HEADER_H - 1), fill="#182943")
            draw.text((x + 28, y + 9), label, font=font, fill="white")
            if index == 0:
                draw.text((x + 1400, y + 15), timestamp, font=small_font, fill="#a9c2e6")
            with Image.open(frame_path) as frame:
                fitted = ImageOps.contain(frame.convert("RGB"), (TILE_W, TILE_H - HEADER_H))
                canvas.paste(fitted, (x + (TILE_W - fitted.width) // 2,
                                      y + HEADER_H + (TILE_H - HEADER_H - fitted.height) // 2))

    destination = output_dir / f"office_four_windows_{timestamp}.png"
    canvas.save(destination, format="PNG", optimize=True)
    print(destination)


if __name__ == "__main__":
    main()
