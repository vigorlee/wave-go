#!/usr/bin/env python3
"""Audit the locally stitched stair/floor visualization and its provenance."""
import json
from pathlib import Path

import numpy as np

from process_roofless_import import mapped_pcd
from process_z5_features import xyz_of
from stitch_z5_left_stairs import BASE, OUTPUT, SOURCE, left_pair, xyz_in_bounds


def main():
    base = mapped_pcd(BASE / 'display_z5_rgb.pcd')
    source = mapped_pcd(SOURCE / 'display_z5_rgb.pcd')
    stitched = mapped_pcd(OUTPUT / 'display_z5_rgb.pcd')
    semantic = mapped_pcd(OUTPUT / 'display_z5_semantic.pcd')
    labels = np.load(OUTPUT / 'class_ids.npy')
    source_labels = np.load(SOURCE / 'class_ids.npy')
    source_graph = json.loads((SOURCE / 'feature_graph.json').read_text())
    graph = json.loads((OUTPUT / 'feature_graph.json').read_text())
    report = json.loads((OUTPUT / 'feature_report.json').read_text())
    assert np.array_equal(stitched[:len(base)], base)
    assert len(stitched) == len(semantic) == len(labels) == graph['point_count']
    assert np.array_equal(stitched[['x', 'y', 'z', 'intensity']],
                          semantic[['x', 'y', 'z', 'intensity']])
    assert float(stitched['z'].max()) <= 5
    lower, landing, _ = left_pair(source_graph)
    source_xyz = xyz_of(source)
    selected = ((source_labels == 2) & xyz_in_bounds(source_xyz, lower, .06)) | (
        (source_labels == 9) & xyz_in_bounds(source_xyz, landing, .08))
    added = stitched[len(base):]
    assert len(added) == selected.sum() == report['registered_source_points']
    shift = np.asarray(graph['registration']['translation_xyz_m'])
    assert np.max(np.abs(xyz_of(added) - (source_xyz[selected] + shift))) < 3e-5
    assert np.array_equal(added[['rgb', 'intensity']], source[selected][['rgb', 'intensity']])
    assert np.array_equal(labels[len(base):], source_labels[selected])
    assert (graph['coordinate_transform_applied'] is True and
            graph['registration']['upper_nearest_median_m'] < .08 and
            graph['registration']['upper_fraction_within_20cm'] > .9)

    stairs = {n['id']: n for n in graph['nodes'] if n['kind'] == 'stairs'}
    landings = [n for n in graph['nodes'] if n['kind'] == 'stair_landing']
    floors = [n for n in graph['nodes'] if n['kind'] == 'upper_floor']
    assert len(floors) == 2 and len(landings) == 2
    assert len([n for n in landings if np.mean(n['bounds'], axis=0)[1] > 0]) == 1
    assert len([n for n in landings if np.mean(n['bounds'], axis=0)[1] < 0]) == 1
    corridors = [n for n in graph['nodes'] if n['kind'] == 'upper_corridor']
    assert len(corridors) == 1
    corridor = corridors[0]
    corridor_evidence = corridor['evidence']
    assert corridor['label_zh'] == '二楼长廊'
    assert corridor_evidence['source_scan'] == '12'
    assert corridor_evidence['measured_points'] >= 8_000
    assert corridor_evidence['longitudinal_span_m'] > 40
    assert corridor_evidence['observed_y_bins_50cm'] >= 80
    assert len(corridor_evidence['sections']) >= 15
    assert corridor_evidence['measured_points'] == report['upper_corridor_points']
    assert corridor_evidence['longitudinal_span_m'] == report['upper_corridor_span_m']
    assert int(np.count_nonzero(labels == 10)) >= corridor_evidence['measured_points']
    edges = {(e['source'], e['target'], e['relation']) for e in graph['edges']}
    for node in floors:
        e = node['evidence']
        assert e['source_scan'] == '12' and e['colored_points'] >= 350
        assert e['height_p10_p90_span_m'] < .09
        assert (e['adjacent_upper_stair_id'], node['id'],
                'measured_upper_floor_adjacency') in edges
        assert e['adjacent_upper_stair_id'] in stairs
        assert (node['id'], corridor['id'], 'measured_corridor_overlap') in edges

    pictures = ['01_z5_rgb', '02_z5_feature_colors', '03_z5_feature_boxes',
                '04_z5_metric_graph', '05_z5_fusion', '06_z5_top',
                '07_z5_stair_ramp_detail', '08_upper_stairs_wall',
                '12_z5_fastlio_cloud']
    for side, numbers in [('left', ('09', '10', '11', '13', '14')),
                          ('right', ('15', '16', '17', '18', '19'))]:
        pictures.extend(f'{number}_{side}_stairs_{kind}' for number, kind in zip(
            numbers, ('detail', 'front', 'color', 'semantic', 'graph')))
        audit = json.loads((OUTPUT / f'{numbers[0]}_{side}_stairs_detail.json').read_text())
        floor = next(n for n in floors if
                     ('left' if np.mean(n['bounds'], axis=0)[1] > 0 else 'right') == side)
        assert audit['topic_scope'] == f'{side}_'
        assert any(item['text'].startswith(floor['id'] + ' ') for item in audit['visible_labels'])
    pictures += ['20_upper_corridor', '21_upper_corridor_semantic',
                 '22_upper_corridor_graph', '23_upper_corridor_color']
    for name in pictures[-4:]:
        audit = json.loads((OUTPUT / f'{name}.json').read_text())
        assert audit['topic_scope'] == 'corridor_'
        assert audit['camera_preset'] == 'corridor'
        if name != '23_upper_corridor_color':
            assert any(item['text'].startswith('C1 ') for item in audit['visible_labels'])
    for name in pictures:
        picture = OUTPUT / (name + '.png')
        assert picture.is_file() and picture.stat().st_size > 100_000, picture
    result = {'output': str(OUTPUT), 'point_count': len(stitched),
              'source_12_points_unchanged': len(base), 'source_mt_registered_points': len(added),
              'left_and_right_flights_connected': True,
              'upper_floor_platforms': len(floors),
              'upper_corridor_measured_points': corridor_evidence['measured_points'],
              'upper_corridor_length_m': corridor_evidence['longitudinal_span_m'],
              'class_10_colored_points': int((labels == 10).sum()),
              'screenshots': len(pictures), 'coordinate_audit_passed': True}
    (OUTPUT / 'validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
