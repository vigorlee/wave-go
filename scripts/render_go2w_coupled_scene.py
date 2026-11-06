#!/usr/bin/env python3
"""Render the coupled Go2W terrain scene to a portable pixmap (PPM)."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    ROOT_DIR
    / "matrix/src/robot_mujoco/zsibot_robots/go2w/scene_terrain_coupled_nav.xml"
)


def write_ppm(path: Path, rgb) -> None:
    height, width, channels = rgb.shape
    if channels != 3:
        raise ValueError("renderer must return RGB pixels")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        stream.write(rgb.tobytes())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
    data = mujoco.MjData(model)
    # Place the robot at the start of the combined course and settle its pose.
    data.qpos[0:3] = (0.0, 0.65, 0.64)
    mujoco.mj_forward(model, data)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 8.8, 0.38)
    camera.distance = 15.8
    camera.azimuth = 166.0
    camera.elevation = -38.0

    with mujoco.Renderer(model, height=args.height, width=args.width) as renderer:
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
    write_ppm(args.output, pixels)
    print(
        f"rendered {args.scene.name}: {args.width}x{args.height}, "
        f"ngeom={model.ngeom}, nu={model.nu} -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
