#!/usr/bin/python3
"""Render an archived native ikd-Tree ROS topic snapshot without making PCD."""
import argparse
from pathlib import Path

import numpy as np

from dense_capture import spectral_rgb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with np.load(args.source) as data:
        points = data['points']
    keep = ((points[:, 0] > -8) & (points[:, 0] < 22) &
            (points[:, 1] > -12) & (points[:, 1] < 16) &
            (points[:, 2] > -2) & (points[:, 2] < 9))
    xyz = points[keep]
    rng = np.random.default_rng(20261002)
    xyz = xyz[rng.choice(len(xyz), min(220000, len(xyz)), replace=False)]
    colors = spectral_rgb(xyz[:, 2], -.4) / 255.
    fig = plt.figure(figsize=(16, 9), facecolor='black')
    ax = fig.add_subplot(111, projection='3d', facecolor='black')
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=colors, s=.21,
               alpha=.80, depthshade=False, rasterized=True)
    ax.set(xlim=(-8, 22), ylim=(-12, 16), zlim=(-2, 9),
           xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)')
    ax.set_box_aspect((30, 28, 11), zoom=1.03)
    ax.view_init(elev=29, azim=-68)
    ax.set_title('Native LIO ikd-Tree map | original LiDAR + IMU bag | 2026-10-02',
                 color='white', pad=12, fontsize=16)
    ax.tick_params(colors='#c4d4e3')
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.label.set_color('#dce8f2')
        axis.pane.fill = False
    fig.text(.06, .035, f'{keep.sum():,} measured map points in view; no interpolation or synthetic graph.',
             color='#bccdde', fontsize=11)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
