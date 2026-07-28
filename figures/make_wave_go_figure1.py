#!/usr/bin/env python3
"""Render the journal-scale WAVE-Go Figure 1 at its final 183 mm width."""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle

from make_wave_go_overview import (
    ASSETS,
    BLUE,
    BLUE_LIGHT,
    CHARGE_IMAGE,
    GRAY,
    GRAY_LIGHT,
    GREEN,
    GREEN_LIGHT,
    GRID,
    INK,
    MUTED,
    ORANGE,
    ORANGE_LIGHT,
    PAPER,
    PURPLE,
    PURPLE_LIGHT,
    ROBOT_IMAGE,
    SEARCH_IMAGE,
    VERIFY_IMAGE,
    VERMILLION,
    VERMILLION_LIGHT,
    add_image,
    add_robot,
    arrow,
    chip,
    label,
    rounded_box,
    vector_check,
)


HERE = Path(__file__).resolve().parent


def panel_title(
    ax: plt.Axes,
    letter: str,
    x: float,
    y: float,
    title: str,
    *,
    title_size: float = 8.6,
) -> None:
    label(ax, x, y, letter, size=10.0, weight="bold")
    label(ax, x + 0.022, y, title, size=title_size, weight="bold")


def draw_seeded_chunks(ax: plt.Axes, x: float, y: float, w: float, h: float) -> None:
    rounded_box(ax, x, y, w, h, face=ORANGE_LIGHT, edge=ORANGE, radius=0.004)
    label(ax, x + w / 2, y + h - 0.016, "SEED SAMPLING", size=6.0, color=ORANGE, weight="bold", ha="center")
    label(ax, x + w / 2, y + 0.052, "pool  0 · 2 · 3 · 5", size=5.4, color=MUTED, ha="center")
    label(ax, x + 0.027, y + 0.032, "search", size=4.9, color=MUTED, ha="center")
    label(ax, x + w - 0.027, y + 0.032, "approach", size=4.9, color=MUTED, ha="center")
    chip(
        ax,
        x + 0.015,
        y + 0.006,
        0.024,
        0.017,
        "×1",
        face=PAPER,
        edge=ORANGE,
        color=ORANGE,
        size=5.1,
        weight="bold",
    )
    chip(
        ax,
        x + w - 0.039,
        y + 0.006,
        0.024,
        0.017,
        "×2",
        face=PAPER,
        edge=GREEN,
        color=GREEN,
        size=5.1,
        weight="bold",
    )


def draw_adapter(ax: plt.Axes, x: float, y: float, w: float, h: float) -> None:
    rounded_box(ax, x, y, w, h, face=PAPER, edge=ORANGE, radius=0.004)
    label(ax, x + w / 2, y + h - 0.018, "SE(3) ADAPTER", size=6.4, color=ORANGE, weight="bold", ha="center")
    label(ax, x + 0.010, y + 0.044, r"$r_{6D}\ \rightarrow\ SO(3)$", size=6.3)
    label(ax, x + 0.010, y + 0.027, r"camera $\rightarrow$ body", size=6.3)
    label(ax, x + 0.010, y + 0.010, r"pose $\rightarrow\ (v_x,\omega_z)$", size=6.3)


def draw_preview_select(ax: plt.Axes, x: float, y: float, w: float, h: float) -> None:
    rounded_box(ax, x, y, w, h, face=VERMILLION_LIGHT, edge=VERMILLION, radius=0.004)
    label(ax, x + w / 2, y + h - 0.020, "FILTER + SCORE", size=6.3, color=VERMILLION, weight="bold", ha="center")
    label(ax, x + w / 2, y + 0.031, "reject violations", size=6.0, color=INK, ha="center")
    label(ax, x + w / 2, y + 0.013, "score candidates", size=6.0, color=INK, ha="center")


def draw_prefix(ax: plt.Axes, x: float, y: float, w: float, h: float) -> None:
    rounded_box(ax, x, y, w, h, face=GREEN_LIGHT, edge=GREEN, radius=0.004)
    label(ax, x + 0.010, y + h - 0.020, "RISK-ADAPTIVE PREFIX", size=6.4, color=GREEN, weight="bold")
    token_x = x + 0.010
    token_y = y + 0.044
    token_gap = 0.0017
    token_w = (w - 0.020 - 15 * token_gap) / 16
    for index in range(16):
        face = GREEN if index < 8 else GRAY_LIGHT
        edge = GREEN if index < 8 else "#D5DDE1"
        rounded_box(
            ax,
            token_x + index * (token_w + token_gap),
            token_y,
            token_w,
            0.017,
            face=face,
            edge=edge,
            linewidth=0.45,
            radius=0.0015,
            zorder=5,
        )
    label(ax, x + w - 0.010, y + 0.078, "example K = 8", size=5.1, color=MUTED, ha="right")
    label(ax, x + 0.010, y + 0.024, "search  K = 16 / 12 / 8", size=6.1, color=INK)
    label(ax, x + 0.010, y + 0.009, "other stages: K ≤ 8", size=6.0, color=INK)


def draw_step_shield(ax: plt.Axes, x: float, y: float, w: float, h: float) -> None:
    rounded_box(ax, x, y, w, h, face=VERMILLION_LIGHT, edge=VERMILLION, radius=0.004)
    label(ax, x + w / 2, y + h - 0.020, "PER-STEP SHIELD", size=6.4, color=VERMILLION, weight="bold", ha="center")
    label(ax, x + w / 2, y + 0.034, "KEEP  ·  LIMIT  ·  ZERO", size=6.0, color=VERMILLION, weight="bold", ha="center")
    label(ax, x + w / 2, y + 0.013, "re-check every 0.1 s", size=6.0, color=INK, ha="center")


def draw_and_gate(ax: plt.Axes, x: float, y: float, scale: float) -> None:
    points = [
        (x, y + scale),
        (x + scale, y),
        (x, y - scale),
        (x - scale, y),
    ]
    ax.add_patch(
        Polygon(
            points,
            closed=True,
            facecolor=PAPER,
            edgecolor=INK,
            linewidth=1.0,
            zorder=6,
        )
    )
    label(ax, x, y, "AND", size=6.3, weight="bold", ha="center")


def draw_gate_lane(
    ax: plt.Axes,
    *,
    x: float,
    y: float,
    w: float,
    h: float,
    title: str,
    lines: tuple[str, ...],
    face: str,
    edge: str,
    footer: str,
) -> None:
    rounded_box(ax, x, y, w, h, face=face, edge=edge, radius=0.004)
    label(ax, x + w / 2, y + h - 0.020, title, size=6.5, color=edge, weight="bold", ha="center")
    line_y = y + h - 0.055
    for value in lines:
        ax.add_patch(
            Circle(
                (x + 0.017, line_y),
                0.0035,
                facecolor=edge,
                edgecolor="none",
                zorder=7,
            )
        )
        label(ax, x + 0.028, line_y, value, size=5.6, color=INK, weight="bold")
        line_y -= 0.030
    label(ax, x + w / 2, y + 0.013, footer, size=5.4, color=edge, weight="bold", ha="center")


def add_marker_callout(
    ax: plt.Axes,
    box: tuple[float, float, float, float],
) -> None:
    x, y, w, h = box
    # A leader points to the recorded marker; it is an annotation, not a
    # reconstructed detector bounding box.
    marker_x = x + 0.424 * w
    marker_y = y + 0.292 * h
    outline = Rectangle(
        (marker_x - 0.013 * w, marker_y - 0.025 * h),
        0.026 * w,
        0.050 * h,
        facecolor="none",
        edgecolor=GREEN,
        linewidth=0.8,
        zorder=10,
    )
    ax.add_patch(outline)
    arrow(
        ax,
        (x + 0.500 * w, y + 0.600 * h),
        (marker_x + 0.010 * w, marker_y + 0.020 * h),
        color=GREEN,
        linewidth=0.8,
        mutation_scale=7.0,
    )
    label(
        ax,
        x + 0.510 * w,
        y + 0.625 * h,
        "decoded ID 560",
        size=5.5,
        color=GREEN,
        weight="bold",
        ha="left",
    )


def build_figure() -> plt.Figure:
    mpl.rcParams.update(
        {
            "font.family": "Liberation Sans",
            "font.size": 7.0,
            "svg.fonttype": "none",
            "svg.image_inline": True,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )

    # 183 x 132 mm: designed at final double-column publication size.
    figure = plt.figure(figsize=(7.205, 5.197), facecolor=PAPER)
    ax = figure.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    label(ax, 0.022, 0.970, "WAVE-Go", size=13.0, weight="bold")
    label(
        ax,
        0.155,
        0.970,
        "World-model action adaptation with verified execution",
        size=7.5,
        color=MUTED,
    )
    label(
        ax,
        0.978,
        0.970,
        "no map-based motion planner  ·  no Go2-W fine-tuning",
        size=5.9,
        color=MUTED,
        weight="bold",
        ha="right",
    )

    # Journal-style panel bands use near-square corners and no decorative shadow.
    rounded_box(ax, 0.020, 0.395, 0.225, 0.530, face="#F7FBFD", edge="#C7DDE9", radius=0.003)
    rounded_box(ax, 0.255, 0.395, 0.475, 0.530, face="#FFFCF5", edge="#E6D7B2", radius=0.003)
    rounded_box(ax, 0.740, 0.395, 0.240, 0.530, face="#F7FBF9", edge="#C5DDD3", radius=0.003)
    rounded_box(ax, 0.020, 0.035, 0.960, 0.340, face="#F8F9FA", edge="#D8DEE2", radius=0.003)

    # a | NWM inputs and two inference modes.
    panel_title(ax, "a", 0.033, 0.900, "Perceive and generate")
    add_image(ax, SEARCH_IMAGE, (0.034, 0.760, 0.090, 0.095), edge=BLUE, radius=0.003)
    label(ax, 0.079, 0.744, "egocentric RGB", size=6.4, color=BLUE, weight="bold", ha="center")
    rounded_box(ax, 0.136, 0.760, 0.093, 0.095, face=PAPER, edge=PURPLE, radius=0.003)
    label(ax, 0.146, 0.835, "TASK", size=6.1, color=PURPLE, weight="bold")
    label(ax, 0.146, 0.799, "Find station\nwith marker\nID 560.", size=5.9)
    chip(ax, 0.081, 0.704, 0.105, 0.028, "stage context", face=GRAY_LIGHT, edge=GRAY, color=MUTED, size=6.1, weight="bold")
    arrow(ax, (0.080, 0.744), (0.065, 0.676), color=BLUE)
    arrow(ax, (0.182, 0.758), (0.202, 0.676), color=PURPLE)
    arrow(ax, (0.134, 0.704), (0.134, 0.676), color=GRAY)

    rounded_box(ax, 0.034, 0.460, 0.195, 0.212, face=PAPER, edge=INK, linewidth=1.0, radius=0.003)
    label(ax, 0.132, 0.652, "PRETRAINED VISION-ACTION NWM", size=6.0, weight="bold", ha="center")
    rounded_box(ax, 0.048, 0.573, 0.167, 0.060, face=ORANGE_LIGHT, edge=ORANGE, radius=0.003)
    label(ax, 0.061, 0.613, "Generator", size=7.0, color=ORANGE, weight="bold")
    label(ax, 0.203, 0.594, "16-step\n9-D chunks", size=6.2, color=INK, weight="bold", ha="right")
    rounded_box(ax, 0.048, 0.493, 0.167, 0.060, face=BLUE_LIGHT, edge=BLUE, radius=0.003)
    label(ax, 0.061, 0.533, "Reasoner", size=7.0, color=BLUE, weight="bold")
    label(ax, 0.132, 0.507, "DOCK  →  APPROACH", size=6.0, color=INK, weight="bold", ha="center")
    label(ax, 0.132, 0.474, "one backbone  ·  two inference modes", size=6.0, color=MUTED, ha="center")
    label(ax, 0.132, 0.433, "Generator = sole nominal motion source", size=6.1, color=ORANGE, weight="bold", ha="center")
    ax.plot([0.215, 0.239, 0.239, 0.262], [0.603, 0.603, 0.781, 0.781], color=ORANGE, linewidth=1.1, zorder=5)
    arrow(ax, (0.262, 0.781), (0.270, 0.781), color=ORANGE, linewidth=1.1)
    label(ax, 0.132, 0.564, "gated: exact ID + stable stop", size=5.4, color=BLUE, weight="bold", ha="center")

    # b | One traceable action chain.
    panel_title(ax, "b", 0.268, 0.900, "Adapt and execute")
    draw_seeded_chunks(ax, 0.270, 0.734, 0.098, 0.094)
    draw_adapter(ax, 0.382, 0.724, 0.104, 0.114)
    draw_preview_select(ax, 0.500, 0.724, 0.104, 0.114)
    rounded_box(ax, 0.618, 0.742, 0.090, 0.078, face=GREEN_LIGHT, edge=GREEN, radius=0.003)
    label(ax, 0.663, 0.795, "SELECT", size=6.5, color=GREEN, weight="bold", ha="center")
    label(ax, 0.663, 0.766, "best admissible\ncandidate", size=5.8, ha="center")
    arrow(ax, (0.368, 0.781), (0.382, 0.781), color=ORANGE)
    arrow(ax, (0.486, 0.781), (0.500, 0.781), color=ORANGE)
    arrow(ax, (0.604, 0.781), (0.618, 0.781), color=ORANGE)

    # Elbow connector from selected candidate into the execution row.
    ax.plot([0.663, 0.663, 0.342], [0.742, 0.694, 0.694], color=GREEN, linewidth=1.0, zorder=5)
    arrow(ax, (0.342, 0.694), (0.342, 0.674), color=GREEN)
    label(ax, 0.506, 0.705, "selected chunk", size=6.0, color=GREEN, weight="bold", ha="center")

    draw_prefix(ax, 0.270, 0.555, 0.150, 0.118)
    draw_step_shield(ax, 0.438, 0.555, 0.135, 0.118)
    rounded_box(ax, 0.591, 0.565, 0.077, 0.098, face=GREEN_LIGHT, edge=GREEN, radius=0.003)
    label(ax, 0.630, 0.633, "DreamWaQ", size=6.5, color=GREEN, weight="bold", ha="center")
    label(ax, 0.630, 0.595, "low-level\ntracking", size=6.0, ha="center")
    add_robot(ax, (0.675, 0.560, 0.047, 0.097))
    label(ax, 0.699, 0.548, "Go2-W", size=6.3, weight="bold", ha="center")
    arrow(ax, (0.420, 0.614), (0.438, 0.614), color=GREEN)
    arrow(ax, (0.573, 0.614), (0.591, 0.614), color=GREEN)
    arrow(ax, (0.668, 0.614), (0.678, 0.614), color=GREEN)

    label(ax, 0.273, 0.517, "EXECUTION FEEDBACK  (not an NWM image input)", size=6.1, color=MUTED, weight="bold")
    chip(ax, 0.270, 0.470, 0.085, 0.032, "RGB-D", face=ORANGE_LIGHT, edge=ORANGE, color=ORANGE, size=6.2, weight="bold")
    chip(ax, 0.363, 0.470, 0.085, 0.032, "LiDAR", face=VERMILLION_LIGHT, edge=VERMILLION, color=VERMILLION, size=6.2, weight="bold")
    chip(ax, 0.456, 0.470, 0.098, 0.032, "odometry", face=PURPLE_LIGHT, edge=PURPLE, color=PURPLE, size=6.2, weight="bold")
    chip(ax, 0.562, 0.470, 0.090, 0.032, "attitude", face=GRAY_LIGHT, edge=GRAY, color=MUTED, size=6.2, weight="bold")
    ax.plot([0.312, 0.312, 0.505, 0.505], [0.503, 0.530, 0.530, 0.555], color=MUTED, linewidth=0.8, linestyle="--")
    ax.plot([0.405, 0.405, 0.505], [0.503, 0.530, 0.530], color=MUTED, linewidth=0.8, linestyle="--")
    ax.plot([0.505, 0.505], [0.503, 0.555], color=MUTED, linewidth=0.8, linestyle="--")
    ax.plot([0.607, 0.607, 0.505], [0.503, 0.530, 0.530], color=MUTED, linewidth=0.8, linestyle="--")
    label(ax, 0.270, 0.432, "shield may keep, limit or zero — never synthesize motion", size=6.1, color=VERMILLION, weight="bold")

    # c | Two evidence authorities merge only at the completion gate.
    panel_title(ax, "c", 0.753, 0.900, "Evidence-gated completion", title_size=7.8)
    draw_gate_lane(
        ax,
        x=0.754,
        y=0.650,
        w=0.100,
        h=0.185,
        title="SEMANTIC",
        lines=("ID 560 + stop", "target = dock", "approach only"),
        face=BLUE_LIGHT,
        edge=BLUE,
        footer="no completion",
    )
    draw_gate_lane(
        ax,
        x=0.864,
        y=0.650,
        w=0.105,
        h=0.185,
        title="PHYSICAL",
        lines=("ID + range", "clearance, pose", "align., travel", "3 fresh pairs"),
        face=GREEN_LIGHT,
        edge=GREEN,
        footer="arrival evidence",
    )
    arrow(ax, (0.804, 0.648), (0.850, 0.610), color=BLUE, linestyle="--")
    arrow(ax, (0.916, 0.648), (0.870, 0.610), color=GREEN)
    draw_and_gate(ax, 0.860, 0.594, 0.022)
    label(ax, 0.870, 0.568, "arrival gates pass", size=5.6, color=INK, weight="bold")
    arrow(ax, (0.860, 0.572), (0.860, 0.542), color=INK)
    chip(ax, 0.782, 0.502, 0.156, 0.040, "STOP + RE-CHECK  ≈3 s", face=PAPER, edge=VERMILLION, color=VERMILLION, size=6.1, weight="bold")
    arrow(ax, (0.860, 0.501), (0.860, 0.446), color=GREEN)
    label(ax, 0.870, 0.474, "completion authorized", size=5.4, color=GREEN, weight="bold")
    chip(ax, 0.764, 0.397, 0.192, 0.040, "SIMULATED CHARGE POSTURE", face=GREEN, edge=GREEN, color=PAPER, size=6.0, weight="bold")

    # d | Full-resolution representative frames at journal scale.
    panel_title(ax, "d", 0.033, 0.348, "Representative successful HouseWorld run")
    evidence_y = 0.128
    caption_y = 0.108
    arrow_y = 0.203
    add_image(ax, SEARCH_IMAGE, (0.034, evidence_y, 0.145, 0.150), edge=BLUE, radius=0.003)
    chip(ax, 0.042, evidence_y + 0.128, 0.067, 0.024, "1  SEARCH", face=BLUE, edge=BLUE, color=PAPER, size=6.0, weight="bold")
    label(ax, 0.107, caption_y, "world-model exploration", size=6.1, color=MUTED, ha="center")

    arrow(ax, (0.181, arrow_y), (0.205, arrow_y), color=BLUE)
    verify_box = (0.210, evidence_y, 0.145, 0.150)
    add_image(ax, VERIFY_IMAGE, verify_box, edge=GREEN, radius=0.003)
    add_marker_callout(ax, verify_box)
    chip(ax, 0.218, evidence_y + 0.128, 0.076, 0.024, "2  ID 560", face=GREEN, edge=GREEN, color=PAPER, size=6.2, weight="bold")
    label(ax, 0.283, caption_y, "exact marker candidate", size=6.1, color=MUTED, ha="center")

    arrow(ax, (0.357, arrow_y), (0.381, arrow_y), color=GREEN)
    rounded_box(ax, 0.386, evidence_y + 0.008, 0.130, 0.134, face=BLUE_LIGHT, edge=BLUE, radius=0.003)
    label(ax, 0.451, evidence_y + 0.120, "SEMANTIC CHECK", size=6.0, color=BLUE, weight="bold", ha="center")
    label(ax, 0.451, evidence_y + 0.085, "charging dock", size=6.8, color=INK, weight="bold", ha="center")
    label(ax, 0.451, evidence_y + 0.058, "confidence  0.90", size=6.3, color=BLUE, weight="bold", ha="center")
    label(ax, 0.451, evidence_y + 0.027, "authorizes approach only", size=5.9, color=MUTED, ha="center")
    label(ax, 0.451, caption_y, "strict six-field JSON", size=6.1, color=MUTED, ha="center")

    arrow(ax, (0.518, arrow_y), (0.542, arrow_y), color=ORANGE)
    add_image(ax, CHARGE_IMAGE, (0.547, evidence_y, 0.165, 0.150), crop=(0, 0, 960, 545), edge=ORANGE, radius=0.003)
    chip(ax, 0.555, evidence_y + 0.128, 0.119, 0.024, "3  CLOSE-RANGE GATE", face=ORANGE, edge=ORANGE, color=PAPER, size=5.9, weight="bold")
    label(ax, 0.630, caption_y, "RGB-D 0.357 m  ·  exact 3/3", size=6.1, color=MUTED, ha="center")

    arrow(ax, (0.714, arrow_y), (0.738, arrow_y), color=GREEN)
    rounded_box(ax, 0.743, evidence_y, 0.222, 0.150, face=GREEN_LIGHT, edge=GREEN, radius=0.003)
    label(ax, 0.854, evidence_y + 0.127, "4  CHARGE POSTURE VERIFIED", size=6.3, color=GREEN, weight="bold", ha="center")
    vector_check(ax, 0.790, evidence_y + 0.073, scale=0.032, color=GREEN, linewidth=2.2)
    label(ax, 0.825, evidence_y + 0.087, "succeeded", size=7.0, color=GREEN, weight="bold")
    label(ax, 0.825, evidence_y + 0.057, "body height  0.222 m", size=6.3, color=INK)
    label(ax, 0.825, evidence_y + 0.027, "linear & angular speed ≈ 0", size=6.0, color=MUTED)
    label(ax, 0.854, caption_y, "stopped, then crouched", size=6.1, color=MUTED, ha="center")

    return figure


def main() -> None:
    for asset in (SEARCH_IMAGE, VERIFY_IMAGE, CHARGE_IMAGE, ROBOT_IMAGE):
        if not Path(asset).is_file():
            raise FileNotFoundError(asset)

    figure = build_figure()
    title = "WAVE-Go: World-model action adaptation with verified execution"
    subject = "Journal-scale scientific overview for map-independent Go2-W charging"
    pdf_metadata = {
        "Title": title,
        "Author": "WAVE-Go project",
        "Subject": subject,
        "Keywords": "world model, cross-embodiment, verified execution",
    }
    svg_metadata = {
        "Title": title,
        "Description": subject,
        "Creator": "WAVE-Go project",
        "Keywords": pdf_metadata["Keywords"],
    }
    figure.savefig(HERE / "wave_go_figure1.svg", facecolor=PAPER, metadata=svg_metadata)
    figure.savefig(HERE / "wave_go_figure1.pdf", facecolor=PAPER, metadata=pdf_metadata)
    figure.savefig(
        HERE / "wave_go_figure1_300dpi.png",
        facecolor=PAPER,
        dpi=300,
        metadata={"Title": title},
    )
    figure.savefig(
        HERE / "wave_go_figure1_600dpi.png",
        facecolor=PAPER,
        dpi=600,
        metadata={"Title": title},
    )
    plt.close(figure)


if __name__ == "__main__":
    main()
