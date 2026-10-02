#!/usr/bin/env python3
"""Check the saved roofless point clouds, geometric graph and demo captures."""
import argparse
import hashlib
import json
import struct
from pathlib import Path

import numpy as np

from process_roofless_import import mapped_pcd

SHOTS = ['00_before_roof', '01_no_roof_rgb', '02_no_roof_height',
         '03_geometry_boxes', '04_graph', '05_combined',
         '06_top_rgb', '07_side_combined']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--raw', required=True, help='Source-derived RGB PCD filename')
    args = parser.parse_args()
    data = args.data
    report = json.loads((data / 'roof_report.json').read_text())
    graph = json.loads((data / 'geometry_graph.json').read_text())
    clouds = {name: mapped_pcd(data / file_name) for name, file_name in {
        'source': args.raw, 'full_no_roof': 'full_no_roof.pcd',
        'display_original': 'display_original.pcd',
        'display_no_roof': 'display_no_roof.pcd'}.items()}
    expected = {'source': report['source_points'],
                'full_no_roof': report['retained_points'],
                'display_original': report['display_original_points'],
                'display_no_roof': report['display_no_roof_points']}
    assert {key: len(value) for key, value in clouds.items()} == expected
    assert expected['source'] - expected['full_no_roof'] == report['removed_points']
    assert graph['point_count'] == expected['display_no_roof']
    assert graph['frame'] == report['frame']
    ids = np.load(data / 'cluster_ids.npy', mmap_mode='r')
    assert len(ids) == graph['point_count']
    assert int((ids > 0).sum()) == graph['clustered_point_count']
    assert int(ids.max()) == report['geometry_clusters']
    assert all(node['z_m'] < 8.1 for node in graph['nodes']
               if node['kind'] == 'horizontal_layer')

    source_z = clouds['source']['z']
    roofless_z = clouds['full_no_roof']['z']
    bands = {}
    for name, low, high in [('below_cut', -100, 8.1),
                            ('roof_underside', 8.1, 8.5),
                            ('roof_surface', 8.5, 9.1),
                            ('above_roof', 9.1, 100)]:
        before = int(((source_z >= low) & (source_z < high)).sum())
        after = int(((roofless_z >= low) & (roofless_z < high)).sum())
        bands[name] = {'source': before, 'roofless': after,
                       'removed_fraction': (before - after) / before if before else None}
    assert bands['below_cut']['source'] == bands['below_cut']['roofless']
    assert bands['roof_surface']['removed_fraction'] > .995

    def below_cut_hash(cloud):
        digest = hashlib.blake2b(digest_size=16)
        for start in range(0, len(cloud), 1_000_000):
            chunk = cloud[start:start + 1_000_000]
            digest.update(chunk[chunk['z'] < 8.1].tobytes())
        return digest.hexdigest()

    source_lower_hash = below_cut_hash(clouds['source'])
    roofless_lower_hash = below_cut_hash(clouds['full_no_roof'])
    assert source_lower_hash == roofless_lower_hash

    screenshots = {}
    for name in SHOTS:
        path = data / (name + '.png')
        assert path.is_file(), path
        with path.open('rb') as stream:
            header = stream.read(24)
        assert header[:8] == b'\x89PNG\r\n\x1a\n' and header[12:16] == b'IHDR', path
        width, height = struct.unpack('>II', header[16:24])
        assert width >= 1280 and height >= 720 and path.stat().st_size > 100_000, path
        audit = json.loads((data / (name + '.json')).read_text())
        assert audit['frame'] == report['frame']
        assert audit['layout'] == 'single_window'
        screenshots[name] = {'width': width, 'height': height,
                             'mode': audit['mode'],
                             'camera': audit['camera_preset']}
    result = {'name': report['name'], 'cloud_counts': expected,
              'roof_height_bands': bands, 'graph_nodes': len(graph['nodes']),
              'unchanged_points_below_8p1m_hash': source_lower_hash,
              'local_box_count': report['geometry_clusters'],
              'screenshots': screenshots, 'status': 'passed'}
    (data / 'validation.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps({'name': result['name'], 'status': 'passed',
                      'roof_surface_removed_fraction': bands['roof_surface']['removed_fraction'],
                      'screenshots': len(screenshots)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
