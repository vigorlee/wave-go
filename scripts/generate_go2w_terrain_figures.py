#!/usr/bin/env python3
"""Generate deterministic thesis PNGs for terrain policy and run evidence."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT_DIR / "artifacts/go2w_terrain_policy_extension_20260824"
FONT_REGULAR = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
FONT_BOLD = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc")
HISTORICAL = ROOT_DIR / "artifacts/cosmos_farthest_navigation_20260723_175227"

INK = "#16324A"
BLUE = "#176B9B"
CYAN = "#36A7C8"
TEAL = "#2B8A78"
ORANGE = "#E98B2A"
RED = "#CC4B4B"
LIGHT = "#F4F8FA"
MID = "#DCE9EF"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = FONT_BOLD if bold and FONT_BOLD.exists() else FONT_REGULAR
    return ImageFont.truetype(str(path), size=size)


def centered(draw: ImageDraw.ImageDraw, xy, text: str, fnt, fill=INK) -> None:
    box = draw.textbbox((0, 0), text, font=fnt)
    width = box[2] - box[0]
    height = box[3] - box[1]
    draw.text((xy[0] - width / 2, xy[1] - height / 2), text, font=fnt, fill=fill)


def rounded_box(draw, box, fill, outline=BLUE, radius=20, width=3) -> None:
    draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline, width=width)


def arrow(draw, start, end, color=CYAN, width=6) -> None:
    draw.line([start, end], fill=color, width=width)
    x2, y2 = end
    x1, y1 = start
    length = max(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5, 1.0)
    ux, uy = (x2 - x1) / length, (y2 - y1) / length
    px, py = -uy, ux
    tip = (x2, y2)
    left = (x2 - 18 * ux + 9 * px, y2 - 18 * uy + 9 * py)
    right = (x2 - 18 * ux - 9 * px, y2 - 18 * uy - 9 * py)
    draw.polygon([tip, left, right], fill=color)


def draw_policy_flow(output: Path) -> None:
    image = Image.new("RGB", (1800, 1120), "white")
    draw = ImageDraw.Draw(image)
    title = font(44, bold=True)
    body = font(28)
    small = font(23)
    centered(draw, (900, 64), "分层语义地图驱动的多地形策略切换与安全投影", title)

    rounded_box(draw, (80, 145, 430, 340), LIGHT)
    centered(draw, (255, 185), "语义地形观测", font(32, True), BLUE)
    for i, line in enumerate(("坡度 / 台阶方向", "通道净空 / 障碍密度", "动态 TTC / 定位置信度")):
        centered(draw, (255, 235 + i * 42), line, small)

    rounded_box(draw, (570, 145, 1230, 340), "#EEF6FB")
    centered(draw, (900, 185), "证据优先级选择器", font(32, True), BLUE)
    centered(draw, (900, 235), "失效安全 → 动态阻塞 → 楼梯 → 斜坡 → 窄通道 → 复杂路障 → 平地", small)
    centered(draw, (900, 288), "迟滞阈值抑制模式抖动；低置信度直接停车", small, RED)

    rounded_box(draw, (1370, 145, 1720, 340), LIGHT)
    centered(draw, (1545, 185), "安全投影", font(32, True), TEAL)
    for i, line in enumerate(("速度 / 角速度限幅", "俯仰角 / 机身高度", "净空约束 / 重规划")):
        centered(draw, (1545, 235 + i * 42), line, small)
    arrow(draw, (430, 243), (570, 243))
    arrow(draw, (1230, 243), (1370, 243))

    profiles = [
        ("平地滚动", "rolling", "0.45 m/s", "#E8F3F8", BLUE),
        ("上坡 / 下坡", "slope_up / down", "0.20 / 0.16 m/s", "#E8F6F1", TEAL),
        ("上楼 / 下楼", "stair_up / down", "0.10 / 0.08 m/s", "#FFF3E5", ORANGE),
        ("复杂路障", "complex_obstacle", "0.14 m/s + 重规划", "#FFF3E5", ORANGE),
        ("窄通道", "narrow_passage", "0.12 m/s", "#F0F0F5", "#6E6E85"),
        ("动态等待 / 安全停车", "dynamic_wait / safe_stop", "0 m/s", "#FBECEC", RED),
    ]
    positions = [(90, 475), (650, 475), (1210, 475), (90, 770), (650, 770), (1210, 770)]
    for (name, key, detail, bg, color), (x, y) in zip(profiles, positions):
        rounded_box(draw, (x, y, x + 500, y + 210), bg, outline=color)
        centered(draw, (x + 250, y + 52), name, font(31, True), color)
        centered(draw, (x + 250, y + 108), key, body)
        centered(draw, (x + 250, y + 162), detail, small)
        arrow(draw, (900, 340), (x + 250, y), color=MID, width=4)

    centered(
        draw,
        (900, 1057),
        "工程原型仅实现策略接口与约束逻辑；连续 Actor 可在后续训练中替换各模式内部控制器",
        small,
        "#536B7A",
    )
    image.save(output, format="PNG", optimize=True)


def fit_cover(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    target_w, target_h = size
    scale = max(target_w / image.width, target_h / image.height)
    resized = image.resize((round(image.width * scale), round(image.height * scale)), Image.Resampling.LANCZOS)
    left = (resized.width - target_w) // 2
    top = (resized.height - target_h) // 2
    return resized.crop((left, top, left + target_w, top + target_h))


def draw_evidence(output: Path) -> None:
    image = Image.new("RGB", (1800, 1120), "white")
    draw = ImageDraw.Draw(image)
    centered(draw, (900, 64), "Go2W + Cosmos3Nav 多阶段导航运行证据", font(44, True))
    shots = [
        ("visual_01_climb_to_upper_landing.jpg", "楼梯入口与模式切换", "阶段 1 / 11"),
        ("visual_04_approach_remote_obstacles.jpg", "远端复杂路障接近", "阶段 4 / 11"),
        ("visual_05_clear_remote_obstacles.jpg", "路障清除与后续走廊", "阶段 5 / 11"),
    ]
    panel_w, panel_h = 520, 500
    for index, (filename, label, stage) in enumerate(shots):
        x = 80 + index * 570
        y = 130
        with Image.open(HISTORICAL / filename) as source:
            panel = fit_cover(source.convert("RGB"), (panel_w, panel_h))
        image.paste(panel, (x, y))
        draw.rectangle((x, y + panel_h - 70, x + panel_w, y + panel_h), fill=(13, 24, 34, 230))
        draw.text((x + 18, y + panel_h - 59), label, font=font(25, True), fill="white")
        draw.text((x + panel_w - 145, y + panel_h - 57), stage, font=font(22), fill="#65D6A3")
        draw.rounded_rectangle((x, y, x + panel_w, y + panel_h), radius=12, outline=BLUE, width=4)

    rounded_box(draw, (80, 700, 1720, 990), "#F3F8FA", outline=MID, radius=24, width=3)
    centered(draw, (900, 744), "本机复跑与扩展校验", font(31, True), BLUE)
    checks = [
        ("主栈", "MuJoCo / UE5 / Go2-W bridge / Zenoh", TEAL),
        ("导航链", "TF / LiDAR / MPPI / cmd_vel / NavigateToPose", BLUE),
        ("Cosmos3Nav", "桥接 / 任务节点 / 可视化 / 11 阶段路线", ORANGE),
        ("验证", "29 项测试通过；组合场景 MuJoCo 编译通过", RED),
    ]
    for i, (key, value, color) in enumerate(checks):
        y = 805 + i * 44
        draw.rounded_rectangle((145, y - 5, 330, y + 34), radius=10, fill=color)
        centered(draw, (237, y + 14), key, font(22, True), "white")
        draw.text((370, y - 4), value, font=font(24), fill=INK)
        draw.ellipse((1590, y + 3, 1614, y + 27), fill="#2FAD6B")
        draw.text((1630, y - 5), "OK", font=font(22, True), fill="#267D52")

    centered(
        draw,
        (900, 1060),
        "历史关键帧用于说明既有多阶段路线；新增斜坡与组合路障场景仍需后续闭环训练和真机验证",
        font(23),
        "#536B7A",
    )
    image.save(output, format="PNG", optimize=True)


def convert_ppm(ppm: Path, png: Path) -> None:
    with Image.open(ppm) as source:
        source.save(png, format="PNG", optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--scene-ppm", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    draw_policy_flow(args.output_dir / "go2w_terrain_policy_flow.png")
    draw_evidence(args.output_dir / "go2w_cosmos3nav_runtime_evidence.png")
    if args.scene_ppm:
        convert_ppm(args.scene_ppm, args.output_dir / "go2w_coupled_scene_render.png")
    print(f"generated thesis figures in {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
