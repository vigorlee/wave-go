#!/usr/bin/env python3
"""Independent point-preservation and feature-evidence checks for Z<=5 outputs."""
import argparse
import json
from pathlib import Path

import numpy as np

from process_roofless_import import mapped_pcd


def compare_filtered(source, result):
    target = 0
    for start in range(0, len(source), 800_000):
        piece = source[start:start + 800_000]
        selected = piece[np.isfinite(piece['z']) & (piece['z'] <= 5)]
        actual = result[target:target + len(selected)]
        if len(actual) != len(selected) or not np.array_equal(selected, actual):
            raise AssertionError(f'PCD differs from original at source offset {start}')
        target += len(selected)
    if target != len(result):
        raise AssertionError('Output has extra points')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', choices=['12', 'mt'], required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    parent = root / 'outputs' / f'imported_{args.name}_20260926'
    out = parent / 'z5_features'
    source = mapped_pcd(parent / ('12_rgb_intensity.pcd' if args.name == '12'
                                  else 'MT_rgb_intensity.pcd'))
    full = mapped_pcd(out / 'full_z5_rgb.pcd')
    display_source = mapped_pcd(parent / 'display_no_roof.pcd')
    display = mapped_pcd(out / 'display_z5_rgb.pcd')
    colored = mapped_pcd(out / 'display_z5_semantic.pcd')
    compare_filtered(source, full)
    compare_filtered(display_source, display)
    assert len(display) == len(colored)
    assert np.array_equal(display[['x', 'y', 'z', 'intensity']],
                          colored[['x', 'y', 'z', 'intensity']])
    assert float(full['z'].max()) <= 5 and float(display['z'].max()) <= 5
    labels = np.load(out / 'class_ids.npy')
    assert len(labels) == len(display) and set(np.unique(labels)) <= set(range(10))
    graph = json.loads((out / 'feature_graph.json').read_text())
    report = json.loads((out / 'feature_report.json').read_text())
    stairs = [n for n in graph['nodes'] if n['kind'] == 'stairs']
    ramps = [n for n in graph['nodes'] if n['kind'] == 'ramp']
    walls = [n for n in graph['nodes'] if n['kind'] == 'vertical_wall']
    landings = [n for n in graph['nodes'] if n['kind'] == 'stair_landing']
    upper = [n for n in stairs if n.get('level') == 'upper']
    assert graph['point_count'] == len(display) == report['display_z5_points']
    assert len(full) == report['full_z5_points']
    assert len(stairs) >= 4 and len(ramps) >= 2 and len(walls) == 1
    assert len(upper) >= (2 if args.name == '12' else 1)
    assert len(landings) >= 1 and report['compact_obstacles'] > 6
    for node in landings:
        e = node['evidence']
        assert e['support_points'] >= 140 and e['colored_points'] >= 140
        assert e['height_p10_p90_span_m'] <= .12
    for node in stairs:
        e = node['evidence']
        assert e['number_of_steps'] >= 4 and .12 <= e['median_step_height_m'] <= .21
        assert e['profile_support_count'] >= 5 and e['colored_points'] > 100
    for node in ramps:
        e = node['evidence']
        assert 12 <= e['slope_deg'] <= 28 and e['plane_rmse_m'] < .1
        assert e['support_points'] >= 700 and e['colored_points'] > 500
    wall = walls[0]['evidence']
    assert wall['fit_support_points'] > 5000 and wall['colored_points'] > 1000
    assert wall['plane_rmse_m'] < .1 and np.diff(wall['y_range_m'])[0] > 8
    pictures = [out / name for name in ['01_z5_rgb.png', '02_z5_feature_colors.png',
                                        '03_z5_feature_boxes.png', '04_z5_metric_graph.png',
                                        '05_z5_fusion.png', '06_z5_top.png',
                                        '07_z5_stair_ramp_detail.png',
                                        '08_upper_stairs_wall.png']]
    pictures.extend([out / '09_left_stairs_detail.png',
                     out / '10_left_stairs_front.png',
                     out / '11_left_stairs_color.png'])
    for picture in pictures:
        assert picture.exists() and picture.stat().st_size > 100_000, picture
    result = {'name': args.name, 'full_points_checked': len(full),
              'display_points_checked': len(display), 'max_z_m': float(full['z'].max()),
              'stairs': len(stairs), 'upper_stairs': len(upper),
              'ramps': len(ramps), 'vertical_walls': len(walls),
              'stair_landings': len(landings), 'other_obstacles': report['compact_obstacles'],
              'screenshots': len(pictures),
              'pointwise_original_match': True}
    (out / 'validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
