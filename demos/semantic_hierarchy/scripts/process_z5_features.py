#!/usr/bin/env python3
"""Measured Z<=5 m cloud and conservative, geometry-backed feature candidates.

No coordinate transform, point synthesis, trained segmentation, or semantic
ground truth is implied. Input PCDs remain unchanged.
"""
import argparse
import json
import warnings
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.signal import find_peaks
from scipy.spatial import cKDTree

from process_roofless_import import mapped_pcd, write_pcd

PALETTE = {
    0: (108, 121, 138),  # unassigned
    1: (106, 176, 145),  # ground
    2: (74, 142, 222),   # lower stair treads and risers
    3: (177, 143, 212),  # ramps
    4: (204, 177, 115),  # horizontal structure
    5: (117, 153, 181),  # vertical structure
    6: (224, 153, 100),  # other compact obstacle
    7: (224, 119, 124),  # verified vertical wall plane
    8: (66, 191, 219),   # upper stair flight
    9: (230, 190, 97),   # measured landing between stair flights
}
CLASS_NAMES = {0: '未分类', 1: '地面', 2: '楼梯', 3: '坡道',
               4: '水平结构', 5: '立面结构', 6: '其他障碍',
               7: '垂直墙', 8: '二层楼梯', 9: '楼梯平台'}


def xyz_of(raw):
    return np.stack([raw[k] for k in 'xyz'], axis=1)


def local_geometry(xyz, k=32):
    tree = cKDTree(xyz)
    kth, neighbors = tree.query(xyz, k=k, workers=-1)
    normals = np.empty_like(xyz)
    curvature = np.empty(len(xyz), np.float32)
    for start in range(0, len(xyz), 90_000):
        stop = min(start + 90_000, len(xyz))
        local = xyz[neighbors[start:stop]].astype(np.float32)
        local -= local.mean(axis=1, keepdims=True)
        covariance = np.einsum('nki,nkj->nij', local, local, optimize=True) / k
        values, vectors = np.linalg.eigh(covariance)
        normals[start:stop] = vectors[:, :, 0]
        curvature[start:stop] = values[:, 0] / np.maximum(values.sum(axis=1), 1e-8)
    return normals, curvature, kth[:, -1].astype(np.float32)


def heightmap(xyz, normals, curvature, kth, resolution=.05,
              z_band=(-1.4, .8), horizontal=.92, max_curvature=.05,
              max_kth=.22, reduction='max'):
    origin = np.floor(xyz[:, :2].min(axis=0) / resolution) * resolution - resolution
    upper = np.ceil(xyz[:, :2].max(axis=0) / resolution) * resolution + resolution
    shape = tuple((np.rint((upper - origin) / resolution).astype(int) + 1).tolist())
    height = np.full(shape, np.inf if reduction == 'min' else -np.inf, np.float32)
    mask = ((xyz[:, 2] > z_band[0]) & (xyz[:, 2] < z_band[1]) &
            (np.abs(normals[:, 2]) > horizontal) &
            (curvature < max_curvature) & (kth < max_kth))
    ij = np.floor((xyz[mask, :2] - origin) / resolution).astype(np.int32)
    if reduction == 'min':
        np.minimum.at(height, (ij[:, 0], ij[:, 1]), xyz[mask, 2])
    else:
        np.maximum.at(height, (ij[:, 0], ij[:, 1]), xyz[mask, 2])
    height[~np.isfinite(height)] = np.nan
    return height, origin, resolution


def upper_level_height(xyz, normals, curvature, kth):
    horizontal = ((np.abs(normals[:, 2]) > .8) & (curvature < .08) &
                  (kth < .32) & (xyz[:, 2] > 1.5) & (xyz[:, 2] < 3.0))
    counts, edges = np.histogram(xyz[horizontal, 2], bins=np.arange(1.5, 3.05, .05))
    if counts.max() < 500:
        raise ValueError('No measured upper horizontal level supports a second stair pass')
    return float((edges[counts.argmax()] + edges[counts.argmax() + 1]) / 2)


def detect_stairs(height, origin, resolution):
    ycoords = origin[1] + (np.arange(height.shape[1]) + .5) * resolution
    rows = []
    for ycenter in np.arange(ycoords[0] + .5, ycoords[-1] - .5, .10):
        local = height[:, np.abs(ycoords - ycenter) <= .40]
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            profile = np.nanmedian(local, axis=1)
        good = (np.isfinite(local).sum(axis=1) >= 2) & np.isfinite(profile)
        if good.sum() < 30:
            continue
        indices = np.arange(len(profile))
        smooth = ndimage.median_filter(np.interp(indices, indices[good], profile[good]), size=3)
        for sign in (1, -1):
            response = sign * (smooth[4:] - smooth[:-4])
            peak, _ = find_peaks(response, height=(.10, .25), prominence=.05, distance=3)
            peak = peak[good[peak] & good[peak + 4]] + 2
            if len(peak) < 3:
                continue
            boundaries = np.r_[0, np.flatnonzero((np.diff(peak) < 3) |
                                               (np.diff(peak) > 12)) + 1, len(peak)]
            for a, b in zip(boundaries[:-1], boundaries[1:]):
                group = peak[a:b]
                if len(group) < 3:
                    continue
                steps = sign * (smooth[group + 2] - smooth[group - 2])
                spacing = np.diff(group) * resolution
                step = float(np.median(steps))
                if (.12 <= step <= .21 and .13 <= np.median(spacing) <= .48 and
                        float(steps.sum()) >= .38):
                    rows.append({'bounds_xy': [[float(origin[0] + group[0] * resolution - .15),
                                                float(ycenter - .4)],
                                               [float(origin[0] + group[-1] * resolution + .15),
                                                float(ycenter + .4)]],
                                 'steps': int(len(group)), 'step_height_m': step,
                                 'direction': int(sign)})
    if not rows:
        return [], []
    grid = .10
    shape = (int(np.ceil(height.shape[0] * resolution / grid)) + 1,
             int(np.ceil(height.shape[1] * resolution / grid)) + 1)
    votes = np.zeros(shape, np.uint16)
    for row in rows:
        lo, hi = np.asarray(row['bounds_xy'])
        a = np.clip(np.floor((lo - origin) / grid).astype(int), 0, np.array(shape) - 1)
        b = np.clip(np.ceil((hi - origin) / grid).astype(int), 0, np.array(shape))
        votes[a[0]:b[0], a[1]:b[1]] += 1
    component, count = ndimage.label(votes >= 2, structure=np.ones((3, 3), bool))
    boxes = []
    for cid in range(1, count + 1):
        coords = np.argwhere(component == cid)
        if len(coords) * grid * grid < 1.0:
            continue
        lo = origin + coords.min(axis=0) * grid
        hi = origin + (coords.max(axis=0) + 1) * grid
        members = [r for r in rows if np.all(np.asarray(r['bounds_xy'][0]) < hi)
                   and np.all(np.asarray(r['bounds_xy'][1]) > lo)]
        if len(members) < 5 or max(r['steps'] for r in members) < 4:
            continue
        boxes.append({'bounds_xy': [lo.tolist(), hi.tolist()],
                      'number_of_steps': int(max(r['steps'] for r in members)),
                      'median_step_height_m': float(np.median([r['step_height_m'] for r in members])),
                      'profile_support_count': len(members),
                      'area_m2': round(len(coords) * grid * grid, 2)})
    boxes.sort(key=lambda r: r['bounds_xy'][0][0])
    return boxes, rows


def detect_ramps(xyz, normals, curvature, kth):
    angle = np.rad2deg(np.arccos(np.clip(np.abs(normals[:, 2]), 0, 1)))
    candidate = ((angle >= 8) & (angle <= 32) & (curvature < .03) &
                 (kth < .22) & (xyz[:, 2] > -1.32) & (xyz[:, 2] < .2))
    xy = xyz[candidate, :2]
    if len(xy) == 0:
        return []
    resolution = .25
    origin = np.floor(xyz[:, :2].min(axis=0) / resolution) * resolution - resolution
    shape = tuple((np.ceil((xyz[:, :2].max(axis=0) - origin) / resolution).astype(int) + 2).tolist())
    ij = np.floor((xy - origin) / resolution).astype(int)
    votes = np.zeros(shape, np.int32)
    np.add.at(votes, (ij[:, 0], ij[:, 1]), 1)
    component, count = ndimage.label(votes >= 8, structure=np.ones((3, 3), bool))
    candidate_index = np.flatnonzero(candidate)
    point_cid = component[ij[:, 0], ij[:, 1]]
    ramps = []
    for cid in range(1, count + 1):
        if np.count_nonzero(component == cid) < 40:
            continue
        selected = candidate_index[point_cid == cid]
        cloud = xyz[selected]
        if len(cloud) < 700 or np.diff(np.quantile(cloud[:, 2], [.05, .95]))[0] < .55:
            continue
        matrix = np.column_stack([cloud[:, 0], cloud[:, 1], np.ones(len(cloud))])
        beta, *_ = np.linalg.lstsq(matrix, cloud[:, 2], rcond=None)
        slope = float(np.rad2deg(np.arctan(np.linalg.norm(beta[:2]))))
        residual = float(np.sqrt(np.mean((matrix @ beta - cloud[:, 2]) ** 2)))
        if not 12 <= slope <= 28 or residual > .10:
            continue
        footprint = ndimage.binary_dilation(component == cid, iterations=1)
        lo, hi = np.quantile(cloud, [.02, .98], axis=0)
        ramps.append({'bounds_xy': [lo[:2].tolist(), hi[:2].tolist()],
                      'bounds_z_m': [float(lo[2]), float(hi[2])],
                      'slope_deg': round(slope, 1), 'plane_rmse_m': round(residual, 3),
                      'support_points': len(cloud), 'plane': beta.tolist(),
                      'footprint_origin': origin.tolist(), 'footprint_resolution_m': resolution,
                      '_footprint': footprint})
    ramps.sort(key=lambda r: r['bounds_xy'][0][0])
    return ramps


def detect_vertical_wall(xyz, normals, curvature, kth, upper_stairs, upper_z):
    """Fit one dominant corridor wall from measured vertical surface points."""
    if not upper_stairs:
        return None
    # The upper stair landings localize the adjacent corridor. The wall line
    # itself is then estimated from points, with no supplied wall coordinates.
    rightmost = max(s['bounds_xy'][1][0] for s in upper_stairs)
    candidate = ((xyz[:, 0] > rightmost - 4) & (xyz[:, 0] < rightmost + 6) &
                 (xyz[:, 2] > .2) & (xyz[:, 2] < 4.9) &
                 (np.abs(normals[:, 2]) < .35) & (np.abs(normals[:, 0]) > .65) &
                 (curvature < .045) & (kth < .28))
    points = xyz[candidate]
    if len(points) < 3000:
        return None
    rng = np.random.default_rng(24)
    sample = points[rng.choice(len(points), min(len(points), 18000), replace=False)]
    best = (-np.inf, 0., 0.)
    for _ in range(600):
        i, j = rng.choice(len(sample), 2, replace=False)
        if abs(sample[i, 1] - sample[j, 1]) < 2:
            continue
        slope = (sample[i, 0] - sample[j, 0]) / (sample[i, 1] - sample[j, 1])
        if abs(slope) > .35:
            continue
        intercept = sample[i, 0] - slope * sample[i, 1]
        residual = np.abs(points[:, 0] - slope * points[:, 1] - intercept) / np.sqrt(1 + slope*slope)
        inlier = residual < .09
        if inlier.sum() < 3000:
            continue
        span = np.diff(np.quantile(points[inlier, 1], [.02, .98]))[0]
        score = int(inlier.sum()) + 100 * span
        if score > best[0]:
            best = (score, float(slope), float(intercept))
    if not np.isfinite(best[0]):
        return None
    _, slope, intercept = best
    for _ in range(2):
        residual = np.abs(points[:, 0] - slope * points[:, 1] - intercept) / np.sqrt(1 + slope*slope)
        inlier = residual < .09
        slope, intercept = np.linalg.lstsq(
            np.column_stack([points[inlier, 1], np.ones(inlier.sum())]),
            points[inlier, 0], rcond=None)[0]
    residual = np.abs(points[:, 0] - slope * points[:, 1] - intercept) / np.sqrt(1 + slope*slope)
    inlier = residual < .09
    cloud = points[inlier]
    ylo, yhi = np.quantile(cloud[:, 1], [.02, .98])
    if len(cloud) < 5000 or yhi - ylo < 8:
        return None
    plane_rmse = float(np.sqrt(np.mean(residual[inlier] ** 2)))
    zlo = float(max(xyz[:, 2].min(), -1.2))
    zhi = float(min(upper_z + .12, 5.0))
    x_end = np.asarray([slope * ylo + intercept, slope * yhi + intercept])
    wall = {'slope_x_per_y': float(slope), 'x_intercept_m': float(intercept),
            'y_range_m': [float(ylo), float(yhi)],
            'z_range_m': [zlo, zhi],
            'plane_rmse_m': round(plane_rmse, 3),
            'fit_support_points': int(len(cloud)),
            'bounds': [[float(x_end.min() - .13), float(ylo), zlo],
                       [float(x_end.max() + .13), float(yhi), zhi]],
            'evidence': 'RANSAC vertical XY plane; normal, planarity, span and residual checks'}
    return wall


def local_surface_heights(xyz, height, origin, resolution):
    ij = np.floor((xyz[:, :2] - origin) / resolution).astype(int)
    on_map = ((ij[:, 0] >= 0) & (ij[:, 0] < height.shape[0]) &
              (ij[:, 1] >= 0) & (ij[:, 1] < height.shape[1]))
    nearest = ndimage.distance_transform_edt(~np.isfinite(height),
                                             return_distances=False, return_indices=True)
    filled = height[tuple(nearest)]
    result = np.full(len(xyz), np.nan, np.float32)
    result[on_map] = filled[ij[on_map, 0], ij[on_map, 1]]
    return result


def assign_labels(xyz, normals, curvature, kth, lower_map, upper_map,
                  origin, resolution, stairs, ramps, wall):
    z = xyz[:, 2]
    labels = np.zeros(len(xyz), np.uint8)
    absnz = np.abs(normals[:, 2])
    floor_z = float(np.median(z[(absnz > .94) & (z > -1.35) & (z < -1.1)]))
    labels[(absnz > .85) & (curvature < .06) & (kth < .24) &
           (np.abs(z - floor_z) < .12)] = 1
    labels[(absnz > .85) & (curvature < .06) & (kth < .25) &
           (z > floor_z + .15)] = 4
    labels[(absnz < .35) & (curvature < .05) & (kth < .25) &
           (z > floor_z + .13)] = 5
    # Restrict ramp color to measured sloped surface close to its fitted plane.
    for ramp in ramps:
        fp = ramp.pop('_footprint')
        i = np.floor((xyz[:, :2] - ramp['footprint_origin']) /
                     ramp['footprint_resolution_m']).astype(np.int32)
        inside = ((i[:, 0] >= 0) & (i[:, 0] < fp.shape[0]) &
                  (i[:, 1] >= 0) & (i[:, 1] < fp.shape[1]))
        indices = np.flatnonzero(inside)
        valid = fp[i[indices, 0], i[indices, 1]]
        predicted = (xyz[indices, :2] @ np.asarray(ramp['plane'][:2]) + ramp['plane'][2])
        picked = indices[valid & (np.abs(z[indices] - predicted) < .11) &
                         (z[indices] > floor_z + .04)]
        labels[picked] = 3
        ramp['colored_points'] = len(picked)
    # Two height layers prevent the upper landing from hiding stair treads.
    local_by_level = {
        'lower': local_surface_heights(xyz, lower_map, origin, resolution),
        'upper': local_surface_heights(xyz, upper_map, origin, resolution)}
    for stair in stairs:
        lo, hi = np.asarray(stair['bounds_xy'])
        inside = np.all((xyz[:, :2] >= lo) & (xyz[:, :2] <= hi), axis=1)
        if stair['level'] == 'upper':
            z_lo, z_hi = stair['height_band_m']
            label = 8
        else:
            z_lo = floor_z + .07
            # A profile can miss one or two treads at its end.  Limiting the
            # color by detected step count left *measured* upper treads grey.
            # The lower height layer itself is capped at Z=.8 m, while the
            # local-surface and footprint gates below prevent distant floors
            # or objects from being called stairs.
            z_hi = .91
            label = 2
        local_height = local_by_level[stair['level']]
        surface = (inside & (z > z_lo) & (z < z_hi) &
                   (z >= local_height - .26) & (z <= local_height + .16))
        # Nearest measured tread height also includes vertical risers and sparse cells.
        labels[surface] = label
        cloud = xyz[surface]
        if len(cloud):
            stair['bounds_z_m'] = np.quantile(cloud[:, 2], [.02, .98]).tolist()
        stair['colored_points'] = int(surface.sum())
    if wall is not None:
        slope, intercept = wall['slope_x_per_y'], wall['x_intercept_m']
        ylo, yhi = wall['y_range_m']
        zlo, zhi = wall['z_range_m']
        residual = np.abs(xyz[:, 0] - slope * xyz[:, 1] - intercept) / np.sqrt(1 + slope*slope)
        selected = ((residual < .13) & (xyz[:, 1] >= ylo) & (xyz[:, 1] <= yhi) &
                    (z >= zlo) & (z <= zhi) & (np.abs(normals[:, 2]) < .58) &
                    (curvature < .10) & ~np.isin(labels, [2, 3, 8]))
        labels[selected] = 7
        wall['colored_points'] = int(selected.sum())
    return labels, floor_z


def label_stair_landings(xyz, normals, curvature, labels, stairs):
    """Color only observed, level tread-to-tread connectors in XY gaps."""
    landings = []
    lower = [(i + 1, s) for i, s in enumerate(stairs) if s['level'] == 'lower']
    upper = [(i + 1, s) for i, s in enumerate(stairs) if s['level'] == 'upper']
    for upper_id, high in upper:
        high_lo = np.asarray(high['bounds_xy'][0])
        candidates = []
        for lower_id, low in lower:
            low_hi = np.asarray(low['bounds_xy'][1])
            gap = high_lo[0] - low_hi[0]
            overlap = (min(low_hi[1], high['bounds_xy'][1][1]) -
                       max(low['bounds_xy'][0][1], high_lo[1]))
            if -.15 <= gap <= 1.2 and overlap >= .8:
                candidates.append((abs(gap), lower_id, low))
        if not candidates:
            continue
        _, lower_id, low = min(candidates)
        lo_xy = np.minimum(low['bounds_xy'][0], high['bounds_xy'][0])
        hi_xy = np.maximum(low['bounds_xy'][1], high['bounds_xy'][1])
        xlo = low['bounds_xy'][1][0] - .12
        xhi = high['bounds_xy'][0][0] + .12
        z_reference = high['bounds_z_m'][0]
        scan = ((xyz[:, 0] >= xlo) & (xyz[:, 0] <= xhi) &
                (xyz[:, 1] >= lo_xy[1]) & (xyz[:, 1] <= hi_xy[1]) &
                (np.abs(xyz[:, 2] - z_reference) < .17) &
                (np.abs(normals[:, 2]) > .72) & (curvature < .10))
        support = xyz[scan]
        if len(support) < 140:
            continue
        zq = np.quantile(support[:, 2], [.10, .50, .90])
        xyq = np.quantile(support[:, :2], [.02, .98], axis=0)
        if (zq[2] - zq[0] > .12 or
                xyq[1, 0] - xyq[0, 0] < .32 or
                xyq[1, 1] - xyq[0, 1] < .8):
            continue
        selected = ((xyz[:, 0] >= xlo) & (xyz[:, 0] <= xhi) &
                    (xyz[:, 1] >= xyq[0, 1] - .06) &
                    (xyz[:, 1] <= xyq[1, 1] + .06) &
                    (np.abs(xyz[:, 2] - zq[1]) < .13) &
                    (np.abs(normals[:, 2]) > .45) &
                    ~np.isin(labels, [2, 3, 7, 8]))
        labels[selected] = 9
        cloud = xyz[selected]
        if len(cloud) < 140:
            continue
        bounds = np.quantile(cloud, [.02, .98], axis=0)
        landings.append({'bounds': bounds.tolist(),
                         'lower_stair_id': f'T{lower_id}',
                         'upper_stair_id': f'T{upper_id}',
                         'support_points': int(len(support)),
                         'colored_points': int(len(cloud)),
                         'height_p10_p90_span_m': round(float(zq[2] - zq[0]), 3),
                         'evidence': 'measured horizontal points in adjacent stair gap'})
    return landings


def refine_stair_step_counts(xyz, labels, stairs):
    """Count additional measured tread-height modes missed by XY profiles."""
    revised = 0
    for stair in stairs:
        lo, hi = np.asarray(stair['bounds_xy'])
        stair_label = 8 if stair['level'] == 'upper' else 2
        mask = ((labels == stair_label) &
                np.all((xyz[:, :2] >= lo) & (xyz[:, :2] <= hi), axis=1))
        z = xyz[mask, 2]
        if len(z) < 500:
            continue
        counts, edges = np.histogram(z, bins=np.arange(z.min() - .04, z.max() + .05, .015))
        peaks, _ = find_peaks(counts, prominence=max(12, counts.max() * .10), distance=6)
        levels = (edges[peaks] + edges[peaks + 1]) / 2
        rise = np.diff(levels)
        if len(rise) < 3 or not np.all((rise >= .09) & (rise <= .26)):
            continue
        observed = len(rise)
        stair['tread_height_modes_m'] = np.round(levels, 3).tolist()
        stair['histogram_observed_rises'] = observed
        if observed > stair['number_of_steps']:
            stair['profile_detected_steps'] = stair['number_of_steps']
            stair['number_of_steps'] = observed
            revised += 1
    return revised


def compact_obstacles(xyz, labels, floor_z, stairs, ramps):
    # Residual compact 3-D clusters are generic obstacles, never guessed people.
    candidate = ((labels == 0) & (xyz[:, 2] > floor_z + .18) &
                 (xyz[:, 2] < floor_z + 2.4))
    for feature in stairs + ramps:
        lo, hi = np.asarray(feature['bounds_xy'])
        overlap = np.all((xyz[:, :2] >= lo - .4) & (xyz[:, :2] <= hi + .4), axis=1)
        candidate[overlap] = False
    points = xyz[candidate]
    if not len(points):
        return []
    voxel = .18
    origin = np.floor(points.min(axis=0) / voxel) * voxel
    ijk = np.floor((points - origin) / voxel).astype(np.int32)
    shape = tuple((ijk.max(axis=0) + 2).tolist())
    if np.prod(shape) > 20_000_000:
        return []
    filled = np.zeros(shape, bool)
    filled[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
    comp, count = ndimage.label(filled, structure=np.ones((3, 3, 3), bool))
    point_ids = comp[ijk[:, 0], ijk[:, 1], ijk[:, 2]]
    candidate_indices = np.flatnonzero(candidate)
    found = []
    for cid in range(1, count + 1):
        member = candidate_indices[point_ids == cid]
        if len(member) < 260:
            continue
        cloud = xyz[member]
        lo, hi = np.quantile(cloud, [.02, .98], axis=0)
        extent = hi - lo
        if (extent[0] > 2.2 or extent[1] > 2.2 or
                extent[2] < .36 or extent[2] > 2.5):
            continue
        found.append((len(member), member, lo, hi))
    found.sort(key=lambda x: -x[0])
    boxes = []
    for _, member, lo, hi in found[:18]:
        labels[member] = 6
        boxes.append({'bounds': [lo.tolist(), hi.tolist()],
                      'colored_points': int(len(member)),
                      'evidence': '18 cm connected residual voxels; compact 3-D extent'})
    return boxes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', choices=['12', 'mt'], required=True)
    parser.add_argument('--geometry-cache', type=Path,
                        help='Optional verified XYZ/normals cache for reruns')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    parent = root / 'outputs' / f'imported_{args.name}_20260926'
    source = parent / ('12_rgb_intensity.pcd' if args.name == '12' else 'MT_rgb_intensity.pcd')
    out = parent / 'z5_features'
    out.mkdir(parents=True, exist_ok=True)
    raw = mapped_pcd(source)
    display_source = mapped_pcd(parent / 'display_no_roof.pcd')
    full_keep = np.isfinite(raw['z']) & (raw['z'] <= 5.0)
    display_keep = np.isfinite(display_source['z']) & (display_source['z'] <= 5.0)
    write_pcd(out / 'full_z5_rgb.pcd', raw, keep=full_keep)
    write_pcd(out / 'display_z5_rgb.pcd', display_source, keep=display_keep)
    display = mapped_pcd(out / 'display_z5_rgb.pcd')
    xyz = xyz_of(display)
    print(args.name, 'local geometry', len(xyz), flush=True)
    cache = np.load(args.geometry_cache) if args.geometry_cache else None
    if cache is not None:
        if not np.array_equal(xyz, cache['xyz']):
            raise ValueError('Geometry cache XYZ differs from display PCD')
        normals, curvature, kth = (cache[k] for k in ('normals', 'curvature', 'kth'))
    else:
        normals, curvature, kth = local_geometry(xyz)
    print(args.name, 'heightmap and feature detection', flush=True)
    h, origin, resolution = heightmap(xyz, normals, curvature, kth)
    lower_stairs, lower_profiles = detect_stairs(h, origin, resolution)
    for stair in lower_stairs:
        stair['level'] = 'lower'
    upper_z = upper_level_height(xyz, normals, curvature, kth)
    upper_band = (upper_z - 1.925, upper_z + .025)
    upper_map, upper_origin, upper_resolution = heightmap(
        xyz, normals, curvature, kth, z_band=upper_band, horizontal=.8,
        max_curvature=.08, max_kth=.32, reduction='min')
    if not np.allclose(upper_origin, origin) or upper_resolution != resolution:
        raise ValueError('Upper and lower heightmap coordinates differ')
    upper_stairs, upper_profiles = detect_stairs(upper_map, origin, resolution)
    for stair in upper_stairs:
        stair['level'] = 'upper'
        stair['height_band_m'] = [float(upper_band[0]), float(upper_band[1])]
    stairs = lower_stairs + upper_stairs
    ramps = detect_ramps(xyz, normals, curvature, kth)
    wall = detect_vertical_wall(xyz, normals, curvature, kth, upper_stairs, upper_z)
    labels, floor_z = assign_labels(xyz, normals, curvature, kth, h, upper_map,
                                    origin, resolution, stairs, ramps, wall)
    landings = label_stair_landings(xyz, normals, curvature, labels, stairs)
    refined_steps = refine_stair_step_counts(xyz, labels, stairs)
    others = compact_obstacles(xyz, labels, floor_z, stairs, ramps)
    recolored = np.array(display)
    colors = np.asarray([PALETTE[i] for i in range(max(PALETTE) + 1)], np.uint32)
    rgb = colors[labels]
    recolored['rgb'] = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    # This format writes structured records in source order with the same XYZ.
    all_keep = np.ones(len(recolored), bool)
    write_pcd(out / 'display_z5_semantic.pcd', recolored, keep=all_keep)
    np.save(out / 'class_ids.npy', labels)
    np.savez_compressed(out / 'feature_heightmap.npz', height=h,
                        upper_height=upper_map, upper_level_z=upper_z,
                        upper_band_m=upper_band, origin=origin, resolution=resolution)
    nodes = []
    for kind, items, prefix in [('stairs', stairs, 'T'), ('ramp', ramps, 'R')]:
        for i, item in enumerate(items, 1):
            lo, hi = np.asarray(item['bounds_xy'])
            zlo, zhi = item.get('bounds_z_m', [floor_z, floor_z + .8])
            label = ((f'二层楼梯 {item["number_of_steps"]}级' if item['level'] == 'upper'
                      else f'楼梯 {item["number_of_steps"]}级') if kind == 'stairs'
                     else f'坡道 {item["slope_deg"]:.1f}°')
            nodes.append({'id': f'{prefix}{i}', 'kind': kind, 'label_zh': label,
                          'level': item.get('level'),
                          'bounds': [[float(lo[0]), float(lo[1]), float(zlo)],
                                     [float(hi[0]), float(hi[1]), float(zhi)]],
                          'evidence': item})
    if wall is not None:
        nodes.append({'id': 'W1', 'kind': 'vertical_wall', 'label_zh': '垂直墙',
                      'bounds': wall['bounds'], 'evidence': wall})
    for i, landing in enumerate(landings, 1):
        nodes.append({'id': f'L{i}', 'kind': 'stair_landing',
                      'label_zh': '楼梯平台', 'bounds': landing['bounds'],
                      'evidence': landing})
    for i, item in enumerate(others, 1):
        nodes.append({'id': f'O{i}', 'kind': 'other_obstacle',
                      'label_zh': '其他障碍', 'bounds': item['bounds'], 'evidence': item})
    ground_node = {'id': 'F0', 'kind': 'ground', 'label_zh': '地面',
                   'position': [float(np.median(xyz[:, 0])),
                                float(np.median(xyz[:, 1])), floor_z]}
    # A minimum-spanning metric graph keeps connections local. Edges describe
    # centroid proximity only; they are not traversability or navigation links.
    major = [n for n in nodes if n['kind'] in ('stairs', 'ramp',
                                             'vertical_wall', 'stair_landing')]
    centers = {n['id']: np.mean(n['bounds'], axis=0) for n in nodes}
    edges = []
    if major:
        base = min(major, key=lambda n: np.linalg.norm(centers[n['id']][:2] -
                                                      np.asarray(ground_node['position'])[:2]))
        edges.append({'source': 'F0', 'target': base['id'], 'relation': 'spatial_proximity'})
        visited = {base['id']}
        while len(visited) < len(major):
            distance, source_id, target_id = min(
                (float(np.linalg.norm(centers[a][:2] - centers[b['id']][:2])), a, b['id'])
                for a in visited for b in major if b['id'] not in visited)
            edges.append({'source': source_id, 'target': target_id,
                          'relation': 'spatial_proximity', 'distance_xy_m': round(distance, 2)})
            visited.add(target_id)
        for node in nodes:
            if node['kind'] != 'other_obstacle':
                continue
            nearest = min(major, key=lambda n: np.linalg.norm(
                centers[n['id']][:2] - centers[node['id']][:2]))
            distance = float(np.linalg.norm(centers[nearest['id']][:2] -
                                            centers[node['id']][:2]))
            if distance < 3.5:
                edges.append({'source': nearest['id'], 'target': node['id'],
                              'relation': 'spatial_proximity',
                              'distance_xy_m': round(distance, 2)})
    for i, landing in enumerate(landings, 1):
        for stair_id in (landing['lower_stair_id'], landing['upper_stair_id']):
            edges.append({'source': stair_id, 'target': f'L{i}',
                          'relation': 'measured_stair_adjacency'})
    graph = {'frame': f'z5_{args.name}', 'source_pcd': str(source),
             'coordinate_transform_applied': False, 'z_max_m': 5.0,
             'semantic_type': 'geometric candidates, not learned semantic ground truth',
             'point_count': len(xyz), 'nodes': [ground_node] + nodes, 'edges': edges,
             'palette_rgb': {CLASS_NAMES[k]: v for k, v in PALETTE.items()}}
    report = {'name': args.name, 'source_pcd': str(source),
              'source_points': len(raw), 'full_z5_points': int(full_keep.sum()),
              'removed_above_z5_points': int((~full_keep).sum()),
              'display_z5_points': len(xyz), 'display_z_max_m': float(xyz[:, 2].max()),
              'full_z_max_m': float(raw['z'][full_keep].max()),
              'ground_z_m': floor_z,
              'class_counts': {CLASS_NAMES[k]: int(np.count_nonzero(labels == k))
                               for k in PALETTE},
              'lower_stair_profile_runs': len(lower_profiles),
              'upper_stair_profile_runs': len(upper_profiles),
              'upper_horizontal_level_z_m': upper_z,
              'stair_flights': len(stairs),
              'stair_flights_with_refined_step_count': refined_steps,
              'upper_stair_flights': len(upper_stairs),
              'ramps': len(ramps), 'vertical_walls': int(wall is not None),
              'stair_landings': len(landings),
              'compact_obstacles': len(others),
              'note': 'Z crop uses the original PCD coordinate; slopes and steps are geometric candidates.'}
    (out / 'feature_graph.json').write_text(json.dumps(graph, ensure_ascii=False, indent=2))
    (out / 'feature_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
