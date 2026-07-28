#!/usr/bin/env python3
"""Render the WAVE-Go paper overview as editable vector and print-ready raster."""

from __future__ import annotations

from pathlib import Path

import matplotlib as mpl

mpl.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle
from PIL import Image
import numpy as np


HERE = Path(__file__).resolve().parent
ASSETS = HERE / "assets"

SEARCH_IMAGE = ASSETS / "search_far.jpg"
VERIFY_IMAGE = ASSETS / "verify_accepted.jpg"
CHARGE_IMAGE = ASSETS / "charge_close_20260727.jpg"
ROBOT_IMAGE = ASSETS / "go2w.png"

INK = "#1F2933"
MUTED = "#5F6C76"
GRID = "#C9D1D6"
PAPER = "#FFFFFF"
PANEL = "#F7F9FA"

BLUE = "#0072B2"
BLUE_LIGHT = "#E8F3F9"
ORANGE = "#E69F00"
ORANGE_LIGHT = "#FCF3DF"
GREEN = "#009E73"
GREEN_LIGHT = "#E5F4EF"
VERMILLION = "#D55E00"
VERMILLION_LIGHT = "#F9ECE6"
PURPLE = "#8A5AA6"
PURPLE_LIGHT = "#F1EBF5"
GRAY = "#8B969E"
GRAY_LIGHT = "#EDF0F2"


def rounded_box(
    ax: plt.Axes,
    x: float,
    y: float,
    w: float,
    h: float,
    *,
    face: str = PAPER,
    edge: str = GRID,
    linewidth: float = 0.9,
    radius: float = 0.008,
    zorder: float = 1,
) -> FancyBboxPatch:
    patch = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle=f"round,pad=0,rounding_size={radius}",
        facecolor=face,
        edgecolor=edge,
        linewidth=linewidth,
        zorder=zorder,
    )
    ax.add_patch(patch)
    return patch


def label(
    ax: plt.Axes,
    x: float,
    y: float,
    value: str,
    *,
    size: float = 8.0,
    color: str = INK,
    weight: str = "normal",
    ha: str = "left",
    va: str = "center",
    rotation: float = 0.0,
    zorder: float = 10,
    linespacing: float = 1.15,
) -> None:
    ax.text(
        x,
        y,
        value,
        fontsize=size,
        color=color,
        fontweight=weight,
        ha=ha,
        va=va,
        rotation=rotation,
        zorder=zorder,
        linespacing=linespacing,
        family="Liberation Sans",
    )


def arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    color: str = MUTED,
    linewidth: float = 1.25,
    style: str = "-|>",
    mutation_scale: float = 10.0,
    connectionstyle: str = "arc3",
    linestyle: str = "-",
    zorder: float = 5,
) -> None:
    patch = FancyArrowPatch(
        start,
        end,
        arrowstyle=style,
        mutation_scale=mutation_scale,
        linewidth=linewidth,
        color=color,
        connectionstyle=connectionstyle,
        linestyle=linestyle,
        shrinkA=1.5,
        shrinkB=1.5,
        zorder=zorder,
    )
    ax.add_patch(patch)


def vector_check(
    ax: plt.Axes,
    x: float,
    y: float,
    *,
    scale: float,
    color: str,
    linewidth: float = 1.4,
    zorder: float = 10,
) -> None:
    ax.plot(
        [x - 0.48 * scale, x - 0.10 * scale, x + 0.58 * scale],
        [y - 0.02 * scale, y - 0.42 * scale, y + 0.48 * scale],
        color=color,
        linewidth=linewidth,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=zorder,
    )


def chip(
    ax: plt.Axes,
    x: float,
    y: float,
    w: float,
    h: float,
    value: str,
    *,
    face: str = PAPER,
    edge: str = GRID,
    color: str = INK,
    size: float = 6.5,
    weight: str = "normal",
) -> None:
    rounded_box(
        ax,
        x,
        y,
        w,
        h,
        face=face,
        edge=edge,
        linewidth=0.75,
        radius=min(0.007, h * 0.28),
        zorder=5,
    )
    label(
        ax,
        x + w / 2,
        y + h / 2,
        value,
        size=size,
        color=color,
        weight=weight,
        ha="center",
        zorder=7,
    )


def image_array(path: Path, crop: tuple[int, int, int, int] | None = None) -> np.ndarray:
    image = Image.open(path)
    image.load()
    if crop is not None:
        image = image.crop(crop)
    return np.asarray(image)


def add_image(
    ax: plt.Axes,
    path: Path,
    box: tuple[float, float, float, float],
    *,
    crop: tuple[int, int, int, int] | None = None,
    edge: str = GRID,
    linewidth: float = 0.9,
    radius: float = 0.009,
    zorder: float = 3,
) -> None:
    x, y, w, h = box
    array = image_array(path, crop=crop)
    clip = FancyBboxPatch(
        (x, y),
        w,
        h,
        boxstyle=f"round,pad=0,rounding_size={radius}",
        transform=ax.transData,
    )
    image = ax.imshow(
        array,
        extent=(x, x + w, y, y + h),
        interpolation="lanczos",
        aspect="auto",
        zorder=zorder,
    )
    image.set_clip_path(clip)
    rounded_box(
        ax,
        x,
        y,
        w,
        h,
        face="none",
        edge=edge,
        linewidth=linewidth,
        radius=radius,
        zorder=zorder + 1,
    )


def add_robot(
    ax: plt.Axes,
    box: tuple[float, float, float, float],
    *,
    alpha: float = 1.0,
) -> None:
    x, y, w, h = box
    # Exclude a small unrelated alpha fragment at the source image's lower edge.
    array = image_array(ROBOT_IMAGE, crop=(30, 55, 720, 675))
    ax.imshow(
        array,
        extent=(x, x + w, y, y + h),
        interpolation="lanczos",
        aspect="auto",
        alpha=alpha,
        zorder=8,
    )


def panel_heading(
    ax: plt.Axes,
    panel: str,
    x: float,
    y: float,
    title: str,
    subtitle: str | None = None,
) -> None:
    label(ax, x, y, panel, size=11.5, weight="bold")
    label(ax, x + 0.021, y, title, size=10.2, weight="bold")
    if subtitle:
        label(ax, x + 0.021, y - 0.027, subtitle, size=6.8, color=MUTED)


def draw_axes_icon(
    ax: plt.Axes,
    origin: tuple[float, float],
    *,
    scale: float,
    forward_label: str,
    lateral_label: str,
    vertical_label: str,
    color: str,
    vertical_direction: float = 1.0,
) -> None:
    x, y = origin
    arrow(ax, (x, y), (x + scale, y), color=color, linewidth=1.25, mutation_scale=8)
    arrow(
        ax,
        (x, y),
        (x - 0.56 * scale, y + 0.64 * scale),
        color=color,
        linewidth=1.25,
        mutation_scale=8,
    )
    arrow(
        ax,
        (x, y),
        (x, y + vertical_direction * 0.90 * scale),
        color=color,
        linewidth=1.25,
        mutation_scale=8,
    )
    label(ax, x + scale + 0.002, y, forward_label, size=5.5, color=color)
    label(
        ax,
        x - 0.59 * scale,
        y + 0.70 * scale,
        lateral_label,
        size=5.5,
        color=color,
        ha="right",
    )
    label(
        ax,
        x,
        y + vertical_direction * scale,
        vertical_label,
        size=5.5,
        color=color,
        ha="center",
    )


def draw_action_row(
    ax: plt.Axes,
    *,
    x: float,
    y: float,
    active: int,
    row_label: str,
    k_label: str,
    active_color: str,
) -> None:
    label(ax, x - 0.006, y + 0.009, row_label, size=5.8, color=MUTED, ha="right")
    width = 0.0093
    gap = 0.0017
    for index in range(16):
        face = active_color if index < active else GRAY_LIGHT
        edge = active_color if index < active else "#D8DEE2"
        ax.add_patch(
            FancyBboxPatch(
                (x + index * (width + gap), y),
                width,
                0.018,
                boxstyle="round,pad=0,rounding_size=0.002",
                facecolor=face,
                edgecolor=edge,
                linewidth=0.45,
                zorder=6,
            )
        )
    label(
        ax,
        x + 16 * (width + gap) + 0.002,
        y + 0.009,
        k_label,
        size=5.8,
        color=active_color,
        weight="bold",
    )


def draw_shield(ax: plt.Axes, center: tuple[float, float], scale: float) -> None:
    x, y = center
    points = np.asarray(
        [
            (x - 0.78 * scale, y + 0.62 * scale),
            (x, y + scale),
            (x + 0.78 * scale, y + 0.62 * scale),
            (x + 0.65 * scale, y - 0.34 * scale),
            (x, y - scale),
            (x - 0.65 * scale, y - 0.34 * scale),
        ]
    )
    ax.add_patch(
        Polygon(
            points,
            closed=True,
            facecolor=VERMILLION_LIGHT,
            edgecolor=VERMILLION,
            linewidth=1.2,
            zorder=6,
        )
    )
    label(ax, x, y + 0.008, "VETO", size=6.6, color=VERMILLION, weight="bold", ha="center")
    label(ax, x, y - 0.012, "ONLY", size=5.5, color=VERMILLION, weight="bold", ha="center")


def draw_authority_matrix(ax: plt.Axes) -> None:
    x0, y0 = 0.783, 0.596
    width, height = 0.174, 0.176
    columns = ["Reason.", "Track", "ID 560", "RGB-D", "LiDAR", "Odom"]
    rows = ["Approach", "Continuity", "Complete"]
    colors = [BLUE, GRAY, GREEN, ORANGE, VERMILLION, PURPLE]
    active = {
        (0, 0),
        (0, 2),
        (1, 1),
        (1, 2),
        (2, 2),
        (2, 3),
        (2, 4),
        (2, 5),
    }
    cols = len(columns)
    row_count = len(rows)
    cell_w = width / cols
    cell_h = height / row_count

    for index, value in enumerate(columns):
        label(
            ax,
            x0 + (index + 0.5) * cell_w,
            y0 + height + 0.018,
            value,
            size=5.0,
            color=colors[index],
            weight="bold",
            ha="center",
            rotation=42,
            va="bottom",
        )
    for row, value in enumerate(rows):
        label(
            ax,
            x0 - 0.009,
            y0 + height - (row + 0.5) * cell_h,
            value,
            size=5.5,
            color=MUTED,
            ha="right",
        )
    for row in range(row_count):
        for col in range(cols):
            is_active = (row, col) in active
            face = colors[col] if is_active else PAPER
            alpha = 0.82 if is_active else 1.0
            rect = Rectangle(
                (x0 + col * cell_w, y0 + height - (row + 1) * cell_h),
                cell_w,
                cell_h,
                facecolor=face,
                edgecolor=GRID,
                linewidth=0.55,
                alpha=alpha,
                zorder=4,
            )
            ax.add_patch(rect)
            if is_active:
                vector_check(
                    ax,
                    x0 + (col + 0.5) * cell_w,
                    y0 + height - (row + 0.5) * cell_h,
                    scale=0.008,
                    color=PAPER,
                    linewidth=1.0,
                )
    ax.add_patch(
        Rectangle(
            (x0, y0),
            width,
            height,
            facecolor="none",
            edgecolor=INK,
            linewidth=0.9,
            zorder=8,
        )
    )
    label(
        ax,
        x0 + 0.5 * cell_w,
        y0 + 0.5 * cell_h,
        "×",
        size=7.3,
        color=VERMILLION,
        weight="bold",
        ha="center",
    )
    label(
        ax,
        x0 + 1.5 * cell_w,
        y0 + 0.5 * cell_h,
        "×",
        size=7.3,
        color=VERMILLION,
        weight="bold",
        ha="center",
    )


def build_figure() -> plt.Figure:
    mpl.rcParams.update(
        {
            "font.family": "Liberation Sans",
            "font.size": 8.0,
            "axes.linewidth": 0.8,
            "svg.fonttype": "none",
            "svg.image_inline": True,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )

    figure = plt.figure(figsize=(15.6, 7.2), facecolor=PAPER)
    ax = figure.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    label(ax, 0.026, 0.965, "WAVE-Go", size=19.0, weight="bold")
    label(
        ax,
        0.127,
        0.965,
        "World-model action adaptation with verified execution for map-independent charging",
        size=10.2,
        color=MUTED,
    )
    chip(
        ax,
        0.778,
        0.943,
        0.195,
        0.040,
        "planner-free  •  no Go2-W fine-tuning",
        face=GRAY_LIGHT,
        edge="#D7DDE1",
        color=INK,
        size=6.6,
        weight="bold",
    )

    rounded_box(
        ax,
        0.025,
        0.365,
        0.288,
        0.545,
        face="#F5FAFD",
        edge="#BCD8E8",
        linewidth=0.8,
        radius=0.008,
    )
    rounded_box(
        ax,
        0.323,
        0.365,
        0.405,
        0.545,
        face="#FFFBF2",
        edge="#E7D5A8",
        linewidth=0.8,
        radius=0.008,
    )
    rounded_box(
        ax,
        0.738,
        0.365,
        0.237,
        0.545,
        face="#F6FAF8",
        edge="#BEDBCF",
        linewidth=0.8,
        radius=0.008,
    )
    rounded_box(
        ax,
        0.025,
        0.035,
        0.950,
        0.305,
        face=PANEL,
        edge="#D8DEE2",
        linewidth=0.8,
        radius=0.008,
    )

    # Panel a: perception and multimodal world-model outputs.
    panel_heading(
        ax,
        "a",
        0.041,
        0.878,
        "Perceive and generate",
        "A shared pretrained NWM provides separate semantic and action outputs.",
    )
    add_image(
        ax,
        SEARCH_IMAGE,
        (0.046, 0.704, 0.112, 0.115),
        edge=BLUE,
        linewidth=1.0,
    )
    chip(
        ax,
        0.051,
        0.688,
        0.102,
        0.026,
        "FIRST-PERSON RGB",
        face=BLUE_LIGHT,
        edge=BLUE,
        color=BLUE,
        size=5.6,
        weight="bold",
    )
    rounded_box(
        ax,
        0.176,
        0.704,
        0.109,
        0.115,
        face=PAPER,
        edge=PURPLE,
        linewidth=1.0,
        radius=0.007,
        zorder=4,
    )
    label(ax, 0.188, 0.799, "LANGUAGE TASK", size=5.8, color=PURPLE, weight="bold")
    label(
        ax,
        0.188,
        0.761,
        "Find the QR-marked\ncharging dock in an\nunknown environment.",
        size=6.6,
        color=INK,
        va="center",
    )
    chip(ax, 0.052, 0.642, 0.058, 0.030, "RGB-D", face=ORANGE_LIGHT, edge=ORANGE, color=ORANGE, size=5.6, weight="bold")
    chip(ax, 0.116, 0.642, 0.058, 0.030, "LiDAR", face=VERMILLION_LIGHT, edge=VERMILLION, color=VERMILLION, size=5.6, weight="bold")
    chip(ax, 0.180, 0.642, 0.052, 0.030, "odom", face=PURPLE_LIGHT, edge=PURPLE, color=PURPLE, size=5.6, weight="bold")
    chip(ax, 0.238, 0.642, 0.047, 0.030, "stage", face=GRAY_LIGHT, edge=GRAY, color=MUTED, size=5.6, weight="bold")

    arrow(ax, (0.102, 0.684), (0.132, 0.615), color=BLUE)
    arrow(ax, (0.230, 0.701), (0.208, 0.615), color=PURPLE)

    rounded_box(
        ax,
        0.051,
        0.493,
        0.234,
        0.128,
        face=PAPER,
        edge=INK,
        linewidth=1.05,
        radius=0.007,
        zorder=3,
    )
    label(
        ax,
        0.168,
        0.605,
        "PRETRAINED VISION–ACTION WORLD MODEL",
        size=6.2,
        color=INK,
        weight="bold",
        ha="center",
    )
    rounded_box(ax, 0.061, 0.512, 0.098, 0.072, face=BLUE_LIGHT, edge=BLUE, linewidth=0.9, radius=0.006, zorder=4)
    rounded_box(ax, 0.177, 0.512, 0.098, 0.072, face=ORANGE_LIGHT, edge=ORANGE, linewidth=0.9, radius=0.006, zorder=4)
    label(ax, 0.110, 0.561, "Reasoner", size=8.2, color=BLUE, weight="bold", ha="center")
    label(ax, 0.110, 0.534, "semantic identity", size=5.8, color=MUTED, ha="center")
    label(ax, 0.226, 0.561, "Generator", size=8.2, color=ORANGE, weight="bold", ha="center")
    label(ax, 0.226, 0.534, "egocentric motion", size=5.8, color=MUTED, ha="center")
    label(
        ax,
        0.168,
        0.501,
        "shared visual–language representation",
        size=5.4,
        color=MUTED,
        ha="center",
    )
    arrow(ax, (0.110, 0.511), (0.110, 0.468), color=BLUE)
    arrow(ax, (0.226, 0.511), (0.226, 0.468), color=ORANGE)

    rounded_box(ax, 0.053, 0.400, 0.114, 0.065, face=BLUE_LIGHT, edge=BLUE, linewidth=0.9, radius=0.005, zorder=3)
    rounded_box(ax, 0.177, 0.400, 0.108, 0.065, face=ORANGE_LIGHT, edge=ORANGE, linewidth=0.9, radius=0.005, zorder=3)
    label(ax, 0.110, 0.446, "STRICT 6-FIELD JSON", size=5.7, color=BLUE, weight="bold", ha="center")
    label(ax, 0.110, 0.421, "dock identity • confidence", size=5.5, color=INK, ha="center")
    label(ax, 0.231, 0.446, r"$A_t \in \mathbb{R}^{16 \times 9}$", size=8.0, color=ORANGE, weight="bold", ha="center")
    label(ax, 0.231, 0.419, r"$[\Delta p_C,\ r_{6D}]$ at 10 Hz", size=5.8, color=INK, ha="center")

    arrow(ax, (0.285, 0.432), (0.344, 0.764), color=ORANGE, linewidth=1.5, connectionstyle="arc3,rad=-0.10")

    # Panel b: cross-embodiment adapter and receding-horizon execution.
    panel_heading(
        ax,
        "b",
        0.339,
        0.878,
        "Adapt and execute",
        "Geometry-consistent transfer and stepwise risk-aware control.",
    )
    label(ax, 0.348, 0.826, "AV optical frame", size=6.3, color=ORANGE, weight="bold")
    draw_axes_icon(
        ax,
        (0.374, 0.745),
        scale=0.040,
        forward_label="+Z",
        lateral_label="+X",
        vertical_label="+Y",
        color=ORANGE,
        vertical_direction=-1.0,
    )
    rounded_box(ax, 0.425, 0.710, 0.126, 0.104, face=PAPER, edge=ORANGE, linewidth=1.0, radius=0.006, zorder=4)
    label(ax, 0.488, 0.789, r"$R_C = \Pi_{SO(3)}(r_{6D})$", size=7.0, color=INK, ha="center")
    label(ax, 0.488, 0.760, r"$\Delta p_B = T_{B\leftarrow C}\Delta p_C$", size=7.0, color=INK, ha="center")
    label(ax, 0.488, 0.731, r"$R_B = T_{B\leftarrow C}R_CT_{B\leftarrow C}^\top$", size=6.3, color=INK, ha="center")
    arrow(ax, (0.412, 0.759), (0.425, 0.759), color=ORANGE)
    arrow(ax, (0.551, 0.759), (0.574, 0.759), color=GREEN)
    label(ax, 0.488, 0.697, "training-free SE(3) adapter", size=5.8, color=MUTED, ha="center")
    label(ax, 0.584, 0.826, "Go2-W base frame", size=6.3, color=GREEN, weight="bold")
    draw_axes_icon(
        ax,
        (0.608, 0.745),
        scale=0.040,
        forward_label="+X",
        lateral_label="+Y",
        vertical_label="+Z",
        color=GREEN,
    )
    rounded_box(ax, 0.650, 0.710, 0.061, 0.104, face=GREEN_LIGHT, edge=GREEN, linewidth=1.0, radius=0.006, zorder=4)
    label(ax, 0.681, 0.783, r"$v_x$", size=9.0, color=GREEN, weight="bold", ha="center")
    label(ax, 0.681, 0.748, r"$\omega_z$", size=9.0, color=GREEN, weight="bold", ha="center")
    label(ax, 0.681, 0.720, "Twist", size=5.8, color=MUTED, ha="center")

    label(ax, 0.345, 0.658, "RISK-ADAPTIVE ACTION PREFIX", size=6.4, color=INK, weight="bold")
    draw_action_row(ax, x=0.392, y=0.635, active=16, row_label="open", k_label="K=16", active_color=GREEN)
    draw_action_row(ax, x=0.392, y=0.606, active=12, row_label="moderate", k_label="K=12", active_color=ORANGE)
    draw_action_row(ax, x=0.392, y=0.577, active=8, row_label="near hazard", k_label="K=8", active_color=VERMILLION)
    label(ax, 0.392, 0.555, "fresh perception + safety check before every 0.1-s step", size=5.5, color=MUTED)

    rounded_box(ax, 0.342, 0.421, 0.116, 0.102, face=PAPER, edge="#D9C58B", linewidth=0.9, radius=0.006, zorder=4)
    label(ax, 0.400, 0.504, "SEEDED CANDIDATES", size=5.8, color=INK, weight="bold", ha="center")
    for index, seed in enumerate((0, 2, 3, 5)):
        face = GREEN_LIGHT if index < 2 else GRAY_LIGHT
        edge = GREEN if index < 2 else GRAY
        chip(
            ax,
            0.351 + index * 0.0255,
            0.461,
            0.021,
            0.027,
            str(seed),
            face=face,
            edge=edge,
            color=edge,
            size=5.4,
            weight="bold",
        )
    label(ax, 0.400, 0.442, "search: 1  |  precision: 2", size=5.4, color=MUTED, ha="center")
    arrow(ax, (0.459, 0.472), (0.500, 0.472), color=ORANGE)

    draw_shield(ax, (0.531, 0.472), 0.037)
    chip(ax, 0.489, 0.525, 0.038, 0.026, "RGB-D", face=ORANGE_LIGHT, edge=ORANGE, color=ORANGE, size=5.1, weight="bold")
    chip(ax, 0.533, 0.525, 0.038, 0.026, "LiDAR", face=VERMILLION_LIGHT, edge=VERMILLION, color=VERMILLION, size=5.1, weight="bold")
    chip(ax, 0.577, 0.525, 0.038, 0.026, "odom", face=PURPLE_LIGHT, edge=PURPLE, color=PURPLE, size=5.1, weight="bold")
    arrow(ax, (0.508, 0.524), (0.518, 0.506), color=ORANGE, linewidth=0.9)
    arrow(ax, (0.552, 0.524), (0.541, 0.506), color=VERMILLION, linewidth=0.9)
    arrow(ax, (0.596, 0.524), (0.549, 0.501), color=PURPLE, linewidth=0.9)
    arrow(ax, (0.568, 0.472), (0.600, 0.472), color=GREEN, linewidth=1.4)

    rounded_box(ax, 0.603, 0.433, 0.058, 0.078, face=GREEN_LIGHT, edge=GREEN, linewidth=0.9, radius=0.006, zorder=4)
    label(ax, 0.632, 0.486, "DreamWaQ", size=6.4, color=GREEN, weight="bold", ha="center")
    label(ax, 0.632, 0.458, "low-level\ncontrol", size=5.5, color=MUTED, ha="center")
    arrow(ax, (0.661, 0.472), (0.674, 0.472), color=GREEN)
    add_robot(ax, (0.665, 0.400, 0.055, 0.100))
    label(ax, 0.694, 0.392, "Go2-W", size=6.0, color=INK, weight="bold", ha="center")
    label(ax, 0.450, 0.386, "command TTL 0.20 s", size=5.4, color=MUTED, ha="center")
    label(ax, 0.573, 0.386, r"deployed $v_x \leq 0.35$ m s$^{-1}$", size=5.4, color=MUTED, ha="center")
    arrow(
        ax,
        (0.695, 0.405),
        (0.347, 0.406),
        color=GREEN,
        linewidth=0.95,
        style="-|>",
        connectionstyle="arc3,rad=-0.16",
        linestyle="--",
    )
    label(ax, 0.523, 0.372, "receding-horizon sensory feedback", size=5.5, color=GREEN, ha="center")

    # Panel c: authority-separated evidence matrix.
    panel_heading(
        ax,
        "c",
        0.754,
        0.878,
        "Verify before completion",
        "Semantic evidence and physical completion have different authority.",
    )
    label(ax, 0.856, 0.823, "EVIDENCE AUTHORITY MATRIX", size=6.3, color=INK, weight="bold", ha="center")
    draw_authority_matrix(ax)
    rounded_box(ax, 0.759, 0.526, 0.196, 0.046, face=BLUE_LIGHT, edge=BLUE, linewidth=0.8, radius=0.005, zorder=4)
    label(ax, 0.857, 0.549, "Reasoner identifies the dock — it cannot trigger charge.", size=5.6, color=BLUE, weight="bold", ha="center")

    chip(ax, 0.759, 0.465, 0.092, 0.040, "exact ID 560", face=GREEN_LIGHT, edge=GREEN, color=GREEN, size=5.7, weight="bold")
    chip(ax, 0.861, 0.465, 0.094, 0.040, "RGB-D 0.20–0.40 m", face=ORANGE_LIGHT, edge=ORANGE, color=ORANGE, size=5.4, weight="bold")
    chip(ax, 0.759, 0.414, 0.092, 0.040, "front clearance ≥ 0.42 m", face=VERMILLION_LIGHT, edge=VERMILLION, color=VERMILLION, size=4.9, weight="bold")
    chip(ax, 0.861, 0.414, 0.094, 0.040, "stopped 3 s + upright", face=PURPLE_LIGHT, edge=PURPLE, color=PURPLE, size=5.3, weight="bold")
    arrow(ax, (0.857, 0.408), (0.857, 0.384), color=GREEN, linewidth=1.4)
    chip(ax, 0.800, 0.369, 0.114, 0.035, "ALL REQUIRED  →  CHARGE", face=GREEN, edge=GREEN, color=PAPER, size=5.8, weight="bold")

    # Panel d: real closed-loop demonstration.
    panel_heading(
        ax,
        "d",
        0.041,
        0.307,
        "Closed-loop demonstration",
        "Unmodified HouseWorld observations from the successful WAVE-Go run.",
    )

    add_image(
        ax,
        SEARCH_IMAGE,
        (0.046, 0.074, 0.181, 0.171),
        edge=BLUE,
        linewidth=1.0,
    )
    chip(ax, 0.055, 0.218, 0.059, 0.024, "1  SEARCH", face=BLUE, edge=BLUE, color=PAPER, size=5.6, weight="bold")
    label(ax, 0.136, 0.057, "world-model exploration • no map target", size=5.8, color=MUTED, ha="center")

    arrow(ax, (0.232, 0.160), (0.271, 0.160), color=BLUE, linewidth=1.6)
    add_image(
        ax,
        VERIFY_IMAGE,
        (0.278, 0.074, 0.120, 0.171),
        crop=(220, 170, 335, 340),
        edge=GREEN,
        linewidth=1.0,
    )
    chip(ax, 0.286, 0.218, 0.064, 0.024, "2  VERIFY", face=GREEN, edge=GREEN, color=PAPER, size=5.6, weight="bold")
    label(ax, 0.338, 0.057, "exact ArUco ID 560", size=5.8, color=MUTED, ha="center")

    arrow(ax, (0.404, 0.160), (0.438, 0.160), color=GREEN, linewidth=1.6)
    rounded_box(ax, 0.445, 0.084, 0.137, 0.151, face=PAPER, edge=BLUE, linewidth=1.0, radius=0.008, zorder=4)
    label(ax, 0.457, 0.215, "SEMANTIC GATE", size=5.9, color=BLUE, weight="bold")
    label(ax, 0.457, 0.185, '"target_kind":', size=5.7, color=MUTED)
    label(ax, 0.457, 0.161, '"robot_charging_dock"', size=5.8, color=BLUE, weight="bold")
    label(ax, 0.457, 0.131, '"confidence": 0.90', size=5.8, color=INK)
    label(ax, 0.457, 0.105, "identity only → approach", size=5.5, color=MUTED)
    label(ax, 0.514, 0.057, "strict six-field JSON", size=5.8, color=MUTED, ha="center")

    arrow(ax, (0.587, 0.160), (0.621, 0.160), color=ORANGE, linewidth=1.6)
    add_image(
        ax,
        CHARGE_IMAGE,
        (0.628, 0.074, 0.184, 0.171),
        crop=(0, 0, 960, 545),
        edge=ORANGE,
        linewidth=1.0,
    )
    chip(ax, 0.636, 0.218, 0.101, 0.024, "3  APPROACH + STOP", face=ORANGE, edge=ORANGE, color=PAPER, size=5.4, weight="bold")
    label(ax, 0.720, 0.057, "RGB-D 0.357 m • exact frames 3/3", size=5.8, color=MUTED, ha="center")

    arrow(ax, (0.817, 0.160), (0.847, 0.160), color=GREEN, linewidth=1.6)
    rounded_box(ax, 0.854, 0.074, 0.102, 0.171, face=GREEN_LIGHT, edge=GREEN, linewidth=1.1, radius=0.008, zorder=4)
    label(ax, 0.905, 0.218, "4  SUCCEEDED", size=6.3, color=GREEN, weight="bold", ha="center")
    vector_check(ax, 0.905, 0.175, scale=0.035, color=GREEN, linewidth=3.0)
    label(ax, 0.905, 0.132, "posture  charge", size=6.1, color=INK, weight="bold", ha="center")
    label(ax, 0.905, 0.105, "body height  0.222 m", size=5.8, color=MUTED, ha="center")
    label(ax, 0.905, 0.057, "stopped before crouching", size=5.8, color=MUTED, ha="center")

    label(
        ax,
        0.972,
        0.014,
        "WAVE-Go overview • vector labels and unmodified experimental image content",
        size=5.1,
        color="#8B959C",
        ha="right",
    )
    return figure


def main() -> None:
    for asset in (SEARCH_IMAGE, VERIFY_IMAGE, CHARGE_IMAGE, ROBOT_IMAGE):
        if not asset.is_file():
            raise FileNotFoundError(asset)

    figure = build_figure()
    metadata = {
        "Title": "WAVE-Go: World-model action adaptation with verified execution",
        "Author": "WAVE-Go project",
        "Subject": "Scientific overview figure for map-independent Go2-W charging",
        "Keywords": "world model, cross-embodiment, action chunk, verified execution",
    }
    figure.savefig(
        HERE / "wave_go_nature_overview.svg",
        facecolor=PAPER,
        metadata={
            "Title": metadata["Title"],
            "Description": metadata["Subject"],
            "Creator": metadata["Author"],
            "Keywords": metadata["Keywords"],
        },
    )
    figure.savefig(HERE / "wave_go_nature_overview.pdf", facecolor=PAPER, metadata=metadata)
    figure.savefig(
        HERE / "wave_go_nature_overview_300dpi.png",
        facecolor=PAPER,
        dpi=300,
        metadata={"Title": metadata["Title"]},
    )
    figure.savefig(
        HERE / "wave_go_nature_overview_600dpi.png",
        facecolor=PAPER,
        dpi=600,
        metadata={"Title": metadata["Title"]},
    )
    plt.close(figure)


if __name__ == "__main__":
    main()
