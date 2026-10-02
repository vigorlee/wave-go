#!/usr/bin/env python3
"""Build an evidence-based geometric graph from the imported 923 PCD.

No class names are inferred: LAZ classification is uniformly zero.  Connected
components here describe occupied geometry beneath the dominant horizontal
plane, in the original LAZ coordinate frame.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.ndimage import label
from imported_923_io import read_pcd


def build_graph(points, voxel_m=0.12, min_cluster_points=2000, max_clusters=6):
    xyz = np.stack([points[name] for name in 'xyz'], axis=1)
    if not np.isfinite(xyz).all():
        raise ValueError('Non-finite XYZ detected')
    z = xyz[:, 2]
    edges = np.arange(np.floor(z.min() * 100) / 100,
                      np.ceil(min(z.max(), 3) * 100) / 100 + .02, .01)
    counts, edges = np.histogram(z, bins=edges)
    dominant_z = float((edges[np.argmax(counts)] + edges[np.argmax(counts) + 1]) / 2)
    plane_mask = np.abs(z - dominant_z) <= .055
    lower_mask = z < dominant_z - .13
    lower = xyz[lower_mask]
    base = lower.min(axis=0)
    vox = np.floor((lower - base) / voxel_m).astype(np.int32)
    occupancy = np.zeros(tuple(vox.max(axis=0) + 1), dtype=np.uint8)
    occupancy[vox[:, 0], vox[:, 1], vox[:, 2]] = 1
    components, _ = label(occupancy, structure=np.ones((3, 3, 3), bool))
    lower_ids = components[vox[:, 0], vox[:, 1], vox[:, 2]]
    sizes = np.bincount(lower_ids)
    ranked = [int(i) for i in np.argsort(sizes[1:])[::-1] + 1
              if sizes[i] >= min_cluster_points][:max_clusters]
    cluster_ids = np.zeros(len(points), dtype=np.uint8)
    nodes, relations = [], []
    plane_xyz = xyz[plane_mask]
    plane_xy = np.quantile(plane_xyz[:, :2], [.02, .98], axis=0)
    plane_center = [*np.median(plane_xyz[:, :2], axis=0).tolist(), dominant_z]
    nodes.append({'id': 'P0', 'kind': 'dominant_horizontal_plane',
                  'label_zh': '主水平面', 'position': plane_center,
                  'bounds': [[*plane_xy[0].tolist(), dominant_z],
                             [*plane_xy[1].tolist(), dominant_z]],
                  'point_count': int(plane_mask.sum()),
                  'evidence': '10 mm Z histogram peak, ±55 mm band'})
    for rank, component_id in enumerate(ranked, start=1):
        local_mask = lower_ids == component_id
        original_indices = np.flatnonzero(lower_mask)[local_mask]
        cluster_ids[original_indices] = rank
        cloud = xyz[original_indices]
        bounds = np.quantile(cloud, [.02, .98], axis=0)
        center = np.median(cloud, axis=0)
        node_id = f'G{rank}'
        nodes.append({'id': node_id, 'kind': 'geometric_component',
                      'label_zh': f'几何簇 {rank}', 'position': center.tolist(),
                      'bounds': bounds.tolist(), 'point_count': int(len(cloud)),
                      'evidence': f'{voxel_m:.2f} m 26-neighbor occupied-voxel component; 2–98% bounds'})
        relations.append({'source': 'P0', 'target': node_id,
                          'relation': 'same_source_cloud', 'label_zh': '同源点云'})
    graph = {'frame': 'imported_923', 'coordinate_transform_applied': False,
             'semantic_classification': False, 'dominant_z_m': dominant_z,
             'component_voxel_m': voxel_m, 'point_count': len(points),
             'nodes': nodes, 'edges': relations,
             'clustered_point_count': int(np.count_nonzero(cluster_ids)),
             'notes': 'Graph and boxes are geometric evidence, not stairs/person/object detections.'}
    return graph, cluster_ids


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    points = read_pcd(args.input)
    graph, clusters = build_graph(points)
    (args.output / 'geometry_graph.json').write_text(json.dumps(graph, ensure_ascii=False, indent=2))
    np.save(args.output / 'cluster_ids.npy', clusters)
    print(json.dumps({'points': len(points), 'dominant_z_m': graph['dominant_z_m'],
                      'clusters': [(n['id'], n['point_count']) for n in graph['nodes'][1:]],
                      'output': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
