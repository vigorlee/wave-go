#!/usr/bin/env python3
"""Locally register MT's measured left staircase into the 12 display map.

The two source maps are never modified.  This is a local translation fitted on
their overlapping upper flight, not a global registration of both buildings.
"""
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from process_roofless_import import mapped_pcd, write_pcd
from process_z5_features import CLASS_NAMES, PALETTE, local_geometry, xyz_of


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'outputs/imported_12_20260926/z5_features'
SOURCE = ROOT / 'outputs/imported_mt_20260926/z5_features'
OUTPUT = ROOT / 'outputs/imported_12_20260926/z5_stitched_12_mt'
COMPOSITE_PALETTE = {**PALETTE, 10: (206, 211, 126)}
COMPOSITE_NAMES = {**CLASS_NAMES, 10: '二楼楼层平台'}


def left_pair(graph):
    landings = [n for n in graph['nodes'] if n['kind'] == 'stair_landing'
                and np.mean(n['bounds'], axis=0)[1] > 0]
    if len(landings) != 1:
        raise ValueError(f'Expected one measured left landing; found {len(landings)}')
    landing = landings[0]
    nodes = {n['id']: n for n in graph['nodes']}
    e = landing['evidence']
    return nodes[e['lower_stair_id']], landing, nodes[e['upper_stair_id']]


def xyz_in_bounds(xyz, node, margin=0):
    lo, hi = np.asarray(node['bounds'], float)
    return np.all((xyz >= lo - margin) & (xyz <= hi + margin), axis=1)


def translation_from_upper(base_xyz, source_xyz, base_labels, source_labels,
                           base_upper, source_upper):
    target = base_xyz[(base_labels == 8) & xyz_in_bounds(base_xyz, base_upper, .15)]
    moving = source_xyz[(source_labels == 8) & xyz_in_bounds(source_xyz, source_upper, .15)]
    if min(len(target), len(moving)) < 4000:
        raise ValueError('Upper stair does not have enough overlapping measured points')
    rng = np.random.default_rng(42)
    target_sample = target[rng.choice(len(target), min(5000, len(target)), False)]
    moving_sample = moving[rng.choice(len(moving), min(5000, len(moving)), False)]
    tree = cKDTree(target_sample)
    delta = np.median(target_sample, axis=0) - np.median(moving_sample, axis=0)
    for _ in range(8):
        distance, near = tree.query(moving_sample + delta)
        supported = distance < np.quantile(distance, .55)
        update = np.median(target_sample[near[supported]] - moving_sample[supported], axis=0)
        delta = .6 * delta + .4 * update
    distance, _ = cKDTree(target).query(moving + delta)
    metrics = {'transform': 'local translation only; no rotation or global merge',
               'translation_xyz_m': delta.tolist(),
               'overlap_target_upper_points': len(target),
               'overlap_source_upper_points': len(moving),
               'upper_nearest_median_m': float(np.median(distance)),
               'upper_nearest_p90_m': float(np.quantile(distance, .9)),
               'upper_fraction_within_20cm': float(np.mean(distance < .2))}
    if (metrics['upper_nearest_median_m'] > .08 or
            metrics['upper_nearest_p90_m'] > .2 or
            metrics['upper_fraction_within_20cm'] < .90):
        raise ValueError(f'Left staircase registration is not accurate enough: {metrics}')
    return delta, metrics


def shifted_node(node, delta, new_id):
    result = copy.deepcopy(node)
    result['id'] = new_id
    result['bounds'] = (np.asarray(result['bounds'], float) + delta).tolist()
    result['evidence']['source_scan'] = 'MT'
    result['evidence']['source_bounds'] = node['bounds']
    result['evidence']['registration_translation_xyz_m'] = delta.tolist()
    return result


def verified_geometry(xyz):
    cache_path = ROOT / '.runtime/z5_probe/12_all_normals.npz'
    if cache_path.exists():
        geometry = np.load(cache_path)
        if not np.array_equal(xyz, geometry['xyz']):
            raise ValueError('12 geometry cache no longer matches its display cloud')
        normals, curvature, kth = (geometry[key] for key in ('normals', 'curvature', 'kth'))
    else:
        normals, curvature, kth = local_geometry(xyz)
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(cache_path, xyz=xyz, normals=normals,
                            curvature=curvature, kth=kth)
    return normals, curvature, kth


def upper_floor_platforms(xyz, labels, graph, geometry):
    """Find measured near-horizontal floor patches beyond both upper flights."""
    normals, curvature, kth = geometry
    nodes = []
    for stair in (n for n in graph['nodes'] if n['kind'] == 'stairs' and
                  n.get('level') == 'upper'):
        lo, hi = np.asarray(stair['bounds'], float)
        support = ((xyz[:, 0] > hi[0] + .05) & (xyz[:, 0] < hi[0] + 2.2) &
                   (xyz[:, 1] > lo[1] - .3) & (xyz[:, 1] < hi[1] + .3) &
                   (np.abs(xyz[:, 2] - hi[2]) < .10) &
                   (np.abs(normals[:, 2]) > .78) & (curvature < .10) & (kth < .35))
        if support.sum() < 400:
            raise ValueError(f'No measured upper floor platform beyond {stair["id"]}')
        center_z = float(np.median(xyz[support, 2]))
        planar = support & (np.abs(xyz[:, 2] - center_z) < .065)
        bounds_xy = np.quantile(xyz[planar, :2], [.02, .98], axis=0)
        if (bounds_xy[1, 0] - bounds_xy[0, 0] < .6 or
                bounds_xy[1, 1] - bounds_xy[0, 1] < 1.0):
            raise ValueError(f'Measured upper floor near {stair["id"]} is too small')
        selected = ((xyz[:, 0] >= bounds_xy[0, 0] - .03) &
                    (xyz[:, 0] <= bounds_xy[1, 0] + .03) &
                    (xyz[:, 1] >= bounds_xy[0, 1] - .03) &
                    (xyz[:, 1] <= bounds_xy[1, 1] + .03) &
                    (np.abs(xyz[:, 2] - center_z) < .08) &
                    (np.abs(normals[:, 2]) > .50) & (curvature < .13) &
                    ~np.isin(labels, [2, 3, 7, 8, 9]))
        if selected.sum() < 350:
            raise ValueError(f'Upper floor near {stair["id"]} has too few colored points')
        zq = np.quantile(xyz[planar, 2], [.1, .9])
        bounds = np.quantile(xyz[selected], [.02, .98], axis=0)
        if zq[1] - zq[0] > .09:
            raise ValueError(f'Upper floor near {stair["id"]} is not planar')
        labels[selected] = 10
        node = {'id': f'U{len(nodes) + 1}', 'kind': 'upper_floor',
                'label_zh': '二楼楼层平台', 'bounds': bounds.tolist(),
                'evidence': {'source_scan': '12', 'adjacent_upper_stair_id': stair['id'],
                             'horizontal_support_points': int(planar.sum()),
                             'colored_points': int(selected.sum()),
                             'median_z_m': center_z,
                             'height_p10_p90_span_m': float(zq[1] - zq[0]),
                             'evidence': 'measured horizontal patch beyond upper stair top'}}
        nodes.append(node)
    return nodes


def upper_corridor(xyz, labels, graph, floor_nodes, geometry):
    """Trace the observed long 2F floor strip beside the fitted corridor wall."""
    normals, curvature, kth = geometry
    wall = next(n for n in graph['nodes'] if n['kind'] == 'vertical_wall')['evidence']
    wall_x = wall['slope_x_per_y'] * xyz[:, 1] + wall['x_intercept_m']
    relative_x = xyz[:, 0] - wall_x
    ylo = min(node['bounds'][0][1] for node in floor_nodes) - .25
    yhi = max(node['bounds'][1][1] for node in floor_nodes) + .45
    region = ((xyz[:, 1] >= ylo) & (xyz[:, 1] <= yhi) &
              (relative_x >= -2.6) & (relative_x <= 0.0) &
              (xyz[:, 2] > 2.10) & (xyz[:, 2] < 2.58))
    horizontal = (region & (np.abs(normals[:, 2]) > .75) &
                  (curvature < .10) & (kth < .35))
    bins = np.floor((xyz[:, 1] - ylo) / .5).astype(np.int32)
    observed = np.zeros(len(xyz), bool)
    support_bins = []
    for number in np.unique(bins[horizontal]):
        in_bin = horizontal & (bins == number)
        if in_bin.sum() < 18:
            continue
        z = float(np.median(xyz[in_bin, 2]))
        measured = (region & (bins == number) &
                    (np.abs(xyz[:, 2] - z) < .065) &
                    (np.abs(normals[:, 2]) > .48) &
                    (curvature < .15) & (kth < .40) &
                    ~np.isin(labels, [2, 3, 7, 8, 9]))
        observed |= measured
        support_bins.append({'y_m': float(ylo + (number + .5) * .5),
                             'horizontal_support_points': int(in_bin.sum()),
                             'height_m': z,
                             'colored_points': int(measured.sum())})
    cloud = xyz[observed]
    if len(cloud) < 8000 or len(support_bins) < 80:
        raise ValueError('The measured upper floor does not support a long corridor')
    bounds = np.quantile(cloud, [.01, .99], axis=0)
    if bounds[1, 1] - bounds[0, 1] < 35:
        raise ValueError('Upper corridor has insufficient longitudinal span')
    # One long rectangular Box would falsely fill empty space.  Each short
    # outline encloses only the measured floor points in that Y slice.
    sections = []
    section_bin = np.floor((cloud[:, 1] - ylo) / 2.5).astype(np.int32)
    for number in np.unique(section_bin):
        points = cloud[section_bin == number]
        if len(points) < 80:
            continue
        lo, hi = np.quantile(points, [.02, .98], axis=0)
        sections.append({'bounds': [lo.tolist(), hi.tolist()],
                         'support_points': len(points)})
    labels[observed] = 10
    return {'id': 'C1', 'kind': 'upper_corridor', 'label_zh': '二楼长廊',
            'bounds': bounds.tolist(),
            'evidence': {'source_scan': '12', 'measured_points': len(cloud),
                         'observed_y_bins_50cm': len(support_bins),
                         'longitudinal_span_m': float(bounds[1, 1] - bounds[0, 1]),
                         'wall_relative_x_band_m': [-2.6, 0.0],
                         'height_profile': support_bins, 'sections': sections,
                         'evidence': 'measured near-horizontal points beside fitted wall; no gap filling'}}


def main():
    base_raw = mapped_pcd(BASE / 'display_z5_rgb.pcd')
    source_raw = mapped_pcd(SOURCE / 'display_z5_rgb.pcd')
    base_labels = np.load(BASE / 'class_ids.npy')
    source_labels = np.load(SOURCE / 'class_ids.npy')
    base_graph = json.loads((BASE / 'feature_graph.json').read_text())
    source_graph = json.loads((SOURCE / 'feature_graph.json').read_text())
    if len(base_raw) != len(base_labels) or len(source_raw) != len(source_labels):
        raise ValueError('Point labels and source PCD have different lengths')
    base_xyz, source_xyz = xyz_of(base_raw), xyz_of(source_raw)
    source_lower, source_landing, source_upper = left_pair(source_graph)
    base_upper = max((n for n in base_graph['nodes'] if n['kind'] == 'stairs' and
                      n.get('level') == 'upper'),
                     key=lambda n: np.mean(n['bounds'], axis=0)[1])
    if np.mean(base_upper['bounds'], axis=0)[1] < 0:
        raise ValueError('The 12 scan has no measured upper flight on the left')
    delta, registration = translation_from_upper(
        base_xyz, source_xyz, base_labels, source_labels, base_upper, source_upper)

    # Carry the measured MT lower treads and connector, without duplicating its
    # upper flight.  Recolor native 12 points only when within 10 cm of those
    # MT feature measurements; all geometry remains source-measured.
    lower = (source_labels == 2) & xyz_in_bounds(source_xyz, source_lower, .06)
    platform = (source_labels == 9) & xyz_in_bounds(source_xyz, source_landing, .08)
    selected = lower | platform
    if lower.sum() < 8000 or platform.sum() < 500:
        raise ValueError('MT left lower flight / landing point support is too small')
    added = np.array(source_raw[selected], copy=True)
    for key, shift in zip('xyz', delta):
        added[key] += np.float32(shift)
    added_xyz = xyz_of(added)
    area_lo = np.min(added_xyz, axis=0) - .15
    area_hi = np.max(added_xyz, axis=0) + .15
    native_roi = np.flatnonzero(np.all((base_xyz >= area_lo) & (base_xyz <= area_hi), axis=1))
    near_distance, near_index = cKDTree(added_xyz).query(base_xyz[native_roi])
    native_match = native_roi[near_distance < .10]
    if len(native_match) < 4000:
        raise ValueError('MT lower flight is not supported by enough native 12 points')
    combined_labels = np.concatenate([base_labels, source_labels[selected]]).copy()
    combined_labels[native_match] = source_labels[selected][near_index[near_distance < .10]]
    geometry = verified_geometry(base_xyz)
    floor_nodes = upper_floor_platforms(base_xyz, combined_labels[:len(base_xyz)],
                                        base_graph, geometry)
    corridor_node = upper_corridor(base_xyz, combined_labels[:len(base_xyz)],
                                   base_graph, floor_nodes, geometry)
    combined_rgb = np.concatenate([np.array(base_raw), added])
    semantic = combined_rgb.copy()
    palette = np.asarray([COMPOSITE_PALETTE[i] for i in range(max(COMPOSITE_PALETTE) + 1)],
                         np.uint32)
    colors = palette[combined_labels]
    semantic['rgb'] = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]

    OUTPUT.mkdir(parents=True, exist_ok=True)
    keep = np.ones(len(combined_rgb), bool)
    write_pcd(OUTPUT / 'display_z5_rgb.pcd', combined_rgb, keep=keep)
    write_pcd(OUTPUT / 'display_z5_semantic.pcd', semantic, keep=keep)
    np.save(OUTPUT / 'class_ids.npy', combined_labels)

    graph = copy.deepcopy(base_graph)
    new_lower_id = f'T{1 + max(int(n["id"][1:]) for n in graph["nodes"] if n["kind"] == "stairs")}'
    new_landing_id = f'L{1 + max(int(n["id"][1:]) for n in graph["nodes"] if n["kind"] == "stair_landing")}'
    new_lower = shifted_node(source_lower, delta, new_lower_id)
    new_landing = shifted_node(source_landing, delta, new_landing_id)
    new_landing['evidence']['lower_stair_id'] = new_lower_id
    new_landing['evidence']['upper_stair_id'] = base_upper['id']
    graph['nodes'] += [new_lower, new_landing] + floor_nodes + [corridor_node]
    graph['edges'] += [{'source': new_lower_id, 'target': new_landing_id,
                        'relation': 'registered_measured_stair_adjacency'},
                       {'source': new_landing_id, 'target': base_upper['id'],
                        'relation': 'registered_measured_stair_adjacency'}]
    graph['edges'] += [{'source': node['evidence']['adjacent_upper_stair_id'],
                        'target': node['id'], 'relation': 'measured_upper_floor_adjacency'}
                       for node in floor_nodes]
    graph['edges'] += [{'source': node['id'], 'target': corridor_node['id'],
                        'relation': 'measured_corridor_overlap'} for node in floor_nodes]
    graph['frame'] = 'z5_12_mt_stitched'
    graph['source_pcds'] = [str(BASE / 'display_z5_rgb.pcd'),
                            str(SOURCE / 'display_z5_rgb.pcd')]
    graph['coordinate_transform_applied'] = True
    graph['registration_scope'] = 'MT lower left flight and landing only; 12 upper and rest of map unchanged'
    graph['registration'] = registration
    graph['point_count'] = len(combined_rgb)
    graph['palette_rgb'] = {COMPOSITE_NAMES[k]: value
                            for k, value in COMPOSITE_PALETTE.items()}
    graph['semantic_type'] = 'geometric candidates and locally registered measured source points'
    (OUTPUT / 'feature_graph.json').write_text(json.dumps(graph, ensure_ascii=False, indent=2))

    report = json.loads((BASE / 'feature_report.json').read_text())
    report.update(name='12+MT', display_z5_points=len(combined_rgb),
                  display_z_max_m=float(max(combined_rgb['z'])),
                  stair_flights=report['stair_flights'] + 1,
                  stair_landings=report['stair_landings'] + 1,
                  upper_floor_platforms=len(floor_nodes),
                  upper_corridor_points=corridor_node['evidence']['measured_points'],
                  upper_corridor_span_m=corridor_node['evidence']['longitudinal_span_m'],
                  registered_source_points=len(added),
                  native_recolored_points=len(native_match),
                  registration=registration,
                  note='Local MT left staircase measurements fitted to 12 upper flight; original scans retained separately')
    report['class_counts'] = {COMPOSITE_NAMES[k]: int((combined_labels == k).sum())
                              for k in COMPOSITE_PALETTE}
    (OUTPUT / 'feature_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({'output': str(OUTPUT), 'left_lower_id': new_lower_id,
                      'left_platform_id': new_landing_id, 'left_upper_id': base_upper['id'],
                      'registered_points': len(added), 'native_recolored': len(native_match),
                      'upper_floor_platforms': [{k: node[k] for k in ('id', 'bounds', 'evidence')}
                                                for node in floor_nodes],
                      'upper_corridor': {'id': corridor_node['id'],
                                         'points': corridor_node['evidence']['measured_points'],
                                         'span_m': corridor_node['evidence']['longitudinal_span_m'],
                                         'sections': len(corridor_node['evidence']['sections'])},
                      'registration': registration}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
