#!/usr/bin/env python3
"""Remove a measured roof surface and build a geometric graph for large LAZ maps.

The source PCD is read-only.  Both the complete no-roof cloud and a bounded
display sample are retained in the same, untransformed coordinate frame.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.signal import find_peaks

from imported_923_io import PCD_DTYPE


def mapped_pcd(path):
    with path.open('rb') as stream:
        header = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header ends before DATA binary')
            header.append(line.decode('ascii').strip())
            if line.strip() == b'DATA binary':
                break
        offset = stream.tell()
    count = int(next(s.split()[1] for s in header if s.startswith('POINTS ')))
    if path.stat().st_size != offset + count * PCD_DTYPE.itemsize:
        raise ValueError('PCD byte count does not match header')
    return np.memmap(path, mode='r', dtype=PCD_DTYPE, offset=offset, shape=(count,))


def write_pcd(path, source, indices=None, keep=None):
    count = (len(indices) if indices is not None else int(np.count_nonzero(keep)))
    header = (f'VERSION .7\nFIELDS x y z rgb intensity\nSIZE 4 4 4 4 2\n'
              f'TYPE F F F U U\nCOUNT 1 1 1 1 1\nWIDTH {count}\nHEIGHT 1\n'
              f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n').encode()
    with path.open('wb') as stream:
        stream.write(header)
        if indices is not None:
            for start in range(0, count, 1_000_000):
                stream.write(source[indices[start:start + 1_000_000]].tobytes())
        else:
            for start in range(0, len(source), 1_000_000):
                part = slice(start, start + 1_000_000)
                stream.write(source[part][keep[part]].tobytes())
    if path.stat().st_size != len(header) + count * PCD_DTYPE.itemsize:
        raise ValueError('Output PCD is incomplete')


def roof_mask(source, lower=8.5, upper=9.1, grid_m=.25, margin_m=.75):
    x, y, z = (source[k] for k in 'xyz')
    origin = np.array([float(x.min()), float(y.min())])
    shape = tuple((np.ceil((np.array([float(x.max()), float(y.max())]) - origin) /
                           grid_m).astype(int) + 2).tolist())
    candidate = (z >= lower) & (z <= upper)
    cx = np.floor((x[candidate] - origin[0]) / grid_m).astype(np.int32)
    cy = np.floor((y[candidate] - origin[1]) / grid_m).astype(np.int32)
    counts = np.zeros(shape, dtype=np.int32)
    np.add.at(counts, (cx, cy), 1)
    occupied = counts >= 2
    closed = ndimage.binary_closing(occupied, structure=np.ones((3, 3), bool))
    components, n_components = ndimage.label(closed, structure=np.ones((3, 3), bool))
    sizes = np.bincount(components.ravel())
    sizes[0] = 0
    largest = int(np.argmax(sizes))
    if not largest or sizes[largest] * grid_m**2 < 100:
        raise ValueError('No roof-sized horizontal connected component found')
    footprint = ndimage.binary_dilation(components == largest, iterations=1)
    high_halo = ndimage.binary_dilation(footprint, iterations=6)
    roof_sum = np.zeros(shape, dtype=np.float64)
    np.add.at(roof_sum, (cx, cy), z[candidate])
    measured = counts > 0
    local_z = np.divide(roof_sum, counts, out=np.zeros_like(roof_sum), where=measured)
    nearest = ndimage.distance_transform_edt(~measured, return_distances=False,
                                               return_indices=True)
    local_z = local_z[tuple(nearest)]
    remove = np.zeros(len(source), dtype=bool)
    for start in range(0, len(source), 1_000_000):
        stop = min(start + 1_000_000, len(source))
        ix = np.floor((x[start:stop] - origin[0]) / grid_m).astype(np.int32)
        iy = np.floor((y[start:stop] - origin[1]) / grid_m).astype(np.int32)
        ix = np.clip(ix, 0, shape[0] - 1)
        iy = np.clip(iy, 0, shape[1] - 1)
        cutoff = np.maximum(lower - .4, local_z[ix, iy] - margin_m)
        remove[start:stop] = ((footprint[ix, iy] & (z[start:stop] >= cutoff)) |
                              (high_halo[ix, iy] & (z[start:stop] > upper)))
    meta = {'roof_candidate_z_band_m': [lower, upper],
            'xy_grid_m': grid_m, 'local_roof_margin_m': margin_m,
            'minimum_removed_roof_z_m': lower - .4, 'high_halo_m': 1.5,
            'candidate_points': int(candidate.sum()),
            'largest_roof_component_cells': int(sizes[largest]),
            'roof_footprint_area_m2': float(footprint.sum() * grid_m**2),
            'roof_component_count': int(n_components),
            'removed_points': int(remove.sum()),
            'retained_points': int(len(source) - remove.sum()),
            'removed_z_quantiles_m': np.quantile(z[remove], [0, .01, .5, .99, 1]).tolist(),
            'retained_z_quantiles_m': np.quantile(z[~remove], [0, .01, .5, .99, 1]).tolist()}
    return remove, meta


def voxel_representatives(source, target=2_500_000):
    origin = np.array([float(source[k].min()) for k in 'xyz'])
    for voxel_m in (.04, .05, .06, .08):
        coords = [np.floor((source[k] - origin[i]) / voxel_m).astype(np.uint64)
                  for i, k in enumerate('xyz')]
        if any(c.max() >= 2**21 for c in coords):
            raise ValueError('Voxel index exceeds 21-bit encoding range')
        keys = (coords[0] << 42) | (coords[1] << 21) | coords[2]
        del coords
        _, indices = np.unique(keys, return_index=True)
        del keys
        indices.sort()
        if len(indices) <= target or voxel_m == .08:
            return indices, voxel_m
        del indices
    raise AssertionError('Voxel loop returned no result')


def layers_and_components(display, frame, max_components=8):
    xyz = np.stack([display[k] for k in 'xyz'], axis=1)
    z = xyz[:, 2]
    bins = np.arange(np.floor(z.min() * 20) / 20,
                     np.ceil(z.max() * 20) / 20 + .1, .05)
    histogram, edges = np.histogram(z, bins=bins)
    smooth = ndimage.gaussian_filter1d(histogram.astype(float), 1.2)
    peaks, props = find_peaks(smooth, distance=12,
                              prominence=max(500, .035 * float(smooth.max())))
    candidates = sorted(peaks, key=lambda i: smooth[i], reverse=True)
    heights = []
    for index in candidates:
        height = float((edges[index] + edges[index + 1]) / 2)
        support = int((np.abs(z - height) <= .10).sum())
        if support >= max(1500, .006 * len(display)):
            heights.append((height, support))
        if len(heights) >= 5:
            break
    heights.sort()
    if not heights:
        index = int(np.argmax(histogram))
        height = float((edges[index] + edges[index + 1]) / 2)
        heights = [(height, int((np.abs(z - height) <= .10).sum()))]

    floor_mask = np.zeros(len(display), dtype=bool)
    nodes, relations = [], []
    for index, (height, _) in enumerate(heights):
        band = np.abs(z - height) <= .10
        floor_mask |= band
        cloud = xyz[band]
        bounds = np.quantile(cloud, [.02, .98], axis=0)
        center = np.median(cloud, axis=0)
        nodes.append({'id': f'H{index + 1}', 'kind': 'horizontal_layer',
                      'label_zh': f'水平层 {index + 1}',
                      'position': center.tolist(), 'bounds': bounds.tolist(),
                      'z_m': height, 'point_count': int(len(cloud)),
                      'evidence': '5 cm Z histogram peak, ±10 cm support band'})
        if index:
            relations.append({'source': f'H{index}', 'target': f'H{index + 1}',
                              'relation': 'height_adjacent', 'label_zh': '高度相邻'})

    residual_idx = np.flatnonzero(~floor_mask)
    residual = xyz[residual_idx]
    voxel_m = .25
    base = residual.min(axis=0)
    vox = np.floor((residual - base) / voxel_m).astype(np.int32)
    occupied = np.zeros(tuple(vox.max(axis=0) + 1), dtype=np.uint8)
    occupied[vox[:, 0], vox[:, 1], vox[:, 2]] = 1
    components, _ = ndimage.label(occupied, structure=np.ones((3, 3, 3), bool))
    local_ids = components[vox[:, 0], vox[:, 1], vox[:, 2]]
    sizes = np.bincount(local_ids)
    ranked = [int(i) for i in np.argsort(sizes[1:])[::-1] + 1
              if sizes[i] >= max(1500, .003 * len(display))][:max_components]
    cluster_ids = np.zeros(len(display), dtype=np.uint8)
    if ranked and sizes[ranked[0]] > .35 * len(display):
        structural_component = ranked.pop(0)
        cloud = residual[local_ids == structural_component]
        bounds = np.quantile(cloud, [.02, .98], axis=0)
        center = np.median(cloud, axis=0)
        nodes.append({'id': 'S0', 'kind': 'structural_component',
                      'label_zh': '建筑主体', 'position': center.tolist(),
                      'bounds': bounds.tolist(), 'point_count': int(len(cloud)),
                      'evidence': 'Largest 25 cm connected component; excluded from local-object boxes'})
        for index, (height, _) in enumerate(heights, 1):
            if bounds[0, 2] <= height <= bounds[1, 2]:
                relations.append({'source': f'H{index}', 'target': 'S0',
                                  'relation': 'structural_overlap', 'label_zh': '高度重叠'})
    for index, component in enumerate(ranked, 1):
        member = residual_idx[local_ids == component]
        cluster_ids[member] = index
        cloud = xyz[member]
        bounds = np.quantile(cloud, [.02, .98], axis=0)
        center = np.median(cloud, axis=0)
        nodes.append({'id': f'G{index}', 'kind': 'geometric_component',
                      'label_zh': f'几何簇 {index}',
                      'position': center.tolist(), 'bounds': bounds.tolist(),
                      'point_count': int(len(cloud)),
                      'evidence': '25 cm 26-neighbor occupied-voxel component; 2–98% bounds'})
        nearest = int(np.argmin([abs(center[2] - h) for h, _ in heights])) + 1
        relations.append({'source': f'H{nearest}', 'target': f'G{index}',
                          'relation': 'near_height', 'label_zh': '高度邻近'})
    graph = {'frame': frame, 'coordinate_transform_applied': False,
             'semantic_classification': False, 'point_count': len(display),
             'horizontal_layers_z_m': [h for h, _ in heights],
             'nodes': nodes, 'edges': relations,
             'clustered_point_count': int(np.count_nonzero(cluster_ids)),
             'notes': 'Horizontal layers and connected geometry are measured candidates; edges are height relations, not traversability.'}
    return graph, cluster_ids


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--roof-band', nargs=2, type=float, default=(8.5, 9.1))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    source = mapped_pcd(args.input)
    remove, report = roof_mask(source, *args.roof_band)
    report.update({'name': args.name, 'source_pcd': str(args.input),
                   'source_points': len(source), 'frame': f'imported_{args.name}',
                   'roof_method': 'largest connected 25 cm XY surface in upper Z band; remove local roof and underside to at least 8.1 m within footprint, plus high parapet in 1.5 m halo'})
    write_pcd(args.output / 'full_no_roof.pcd', source, keep=~remove)
    indices, voxel_m = voxel_representatives(source)
    kept_indices = indices[~remove[indices]]
    write_pcd(args.output / 'display_original.pcd', source, indices=indices)
    write_pcd(args.output / 'display_no_roof.pcd', source, indices=kept_indices)
    report.update({'display_voxel_m': voxel_m, 'display_original_points': len(indices),
                   'display_no_roof_points': len(kept_indices),
                   'display_removed_points': len(indices) - len(kept_indices)})
    display = mapped_pcd(args.output / 'display_no_roof.pcd')
    graph, cluster_ids = layers_and_components(display, report['frame'])
    report.update({'horizontal_layers_z_m': graph['horizontal_layers_z_m'],
                   'geometry_clusters': len([n for n in graph['nodes'] if n['kind'] == 'geometric_component'])})
    (args.output / 'roof_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    (args.output / 'geometry_graph.json').write_text(json.dumps(graph, ensure_ascii=False, indent=2))
    np.save(args.output / 'cluster_ids.npy', cluster_ids)
    print(json.dumps({'name': args.name, 'source': len(source),
                      'removed': report['removed_points'],
                      'display': len(kept_indices),
                      'layers': graph['horizontal_layers_z_m'],
                      'nodes': len(graph['nodes']), 'edges': len(graph['edges'])},
                     ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
