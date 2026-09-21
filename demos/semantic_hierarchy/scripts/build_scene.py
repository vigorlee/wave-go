#!/usr/bin/env python3
"""Build an evidence-labelled scene graph from the recorded OfficeWorld map.

This is a geometric/semantic adapter, not S-Graphs optimization or Hydra inference.
All metric coordinates remain in the recorded LIO map frame.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
import spark_dsg as dsg
from semantic_labels_cn import KIND_ZH, RELATION_ZH, ONTOLOGY_ZH, translate_graph

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['unknown', 'stairs', 'ramp', 'cylinder', 'person', 'wall', 'floor']
COLORS = np.array([[75, 91, 111], [38, 170, 255], [180, 115, 250],
                   [255, 171, 48], [250, 70, 135], [137, 172, 200],
                   [62, 200, 159]], np.uint8)


def pcd_xyz(path):
    with open(path, 'rb') as stream:
        header = []
        while True:
            line = stream.readline().decode('ascii').strip()
            header.append(line)
            if line.startswith('DATA '):
                break
        assert 'FIELDS x y z' in header and 'DATA binary' in header, header
        return np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 3).copy()


def cluster_ids(points, eps, minimum, keep):
    if len(points) < minimum:
        return []
    ids = DBSCAN(eps=eps, min_samples=minimum, n_jobs=2).fit_predict(points)
    groups = [np.flatnonzero(ids == i) for i in set(ids) if i >= 0]
    return sorted([i for i in groups if len(i) >= keep], key=len, reverse=True)


def bounds(points):
    # Robust extents exclude isolated returns; the complete cloud is retained.
    return np.quantile(points, [0.02, 0.98], axis=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'outputs')
    parser.add_argument('--geometry', type=Path, default=ROOT.parent/'WavegoV2/data/office_map.pcd')
    parser.add_argument('--semantics', type=Path, default=ROOT/'data/semantic_map.npz')
    parser.add_argument('--trajectory', type=Path, default=ROOT/'data/lio_trajectory.npy')
    parser.add_argument('--source-recording', default='office_rosbag_20260916_01')
    parser.add_argument('--z-max', type=float, default=8.2)
    parser.add_argument('--floor-heights', type=float, nargs='+', help='Explicit known-height priors; require measured horizontal support')
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    source = args.geometry
    points = pcd_xyz(source)
    raw = np.load(args.semantics)
    trajectory = np.load(args.trajectory)
    assert len(trajectory) >= 80, 'Need at least 80 LIO observations'
    assert np.isfinite(points).all() and np.isfinite(raw['points']).all()
    assert len(raw['labels']) == len(raw['points'])
    distance, nearest = cKDTree(raw['points']).query(points, workers=2)
    labels = np.where(distance <= .16, raw['labels'][nearest], 0).astype(np.uint8)
    labels[labels >= len(NAMES)] = 0
    original_labels = labels.copy()
    # Evidence: 0 unlabelled, 1 recorded visual semantic prediction,
    # 2 geometric plane hypothesis. Never replace visual labels with geometry.
    evidence = np.where(labels > 0, 1, 0).astype(np.uint8)
    roi = ((points[:, 0] > -3) & (points[:, 0] < 21) &
           (points[:, 1] > -8) & (points[:, 1] < 13) &
           (points[:, 2] > -1.5) & (points[:, 2] < args.z_max))
    cloud = points[roi]
    dist, neighbors = cKDTree(cloud).query(cloud, k=20, workers=2)
    centered = cloud[neighbors] - cloud[neighbors].mean(axis=1, keepdims=True)
    eigen, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij', centered, centered) / 20)
    normals = vectors[:, :, 0]
    planar = eigen[:, 0] / np.maximum(eigen.sum(axis=1), 1e-9) < .06
    horizontal = (abs(normals[:, 2]) > .88) & planar
    vertical = (abs(normals[:, 2]) < .22) & planar
    indices = np.flatnonzero(roi)

    floor_nodes = []
    # Start/end plateaus provide observation anchors, not architectural truth.
    anchors = [np.median(trajectory[:80, 1:], axis=0),
               np.median(trajectory[-100:, 1:], axis=0)]
    if args.floor_heights:
        anchors=[np.array([0.,0.,z+.5]) for z in args.floor_heights]
    for ai,anchor in enumerate(anchors):
        local = horizontal & (np.linalg.norm(cloud[:, :2] - anchor[:2], axis=1) < 3)
        local &= (cloud[:, 2] > anchor[2] - 1.15) & (cloud[:, 2] < anchor[2] - .18)
        h, bins = np.histogram(cloud[local, 2], bins=np.arange(anchor[2] - 1.2, anchor[2], .06))
        if h.sum() < 50 and not args.floor_heights:
            continue
        peak = np.argmax(gaussian_filter1d(h.astype(float), 1))
        z = float((bins[peak] + bins[peak+1]) / 2)
        if args.floor_heights:z=args.floor_heights[ai]
        if any(abs(z - f['z']) < 1 for f in floor_nodes):
            continue
        support = horizontal & (abs(cloud[:, 2] - z) < .24)
        region = cloud[support]
        if len(region)<50:continue
        lo, hi = bounds(region)
        fid = len(floor_nodes)
        floor_nodes.append(dict(id=f'F{fid}', z=z, anchor=anchor.tolist(),
                                position=[*np.median(region[:, :2], axis=0).tolist(), z],
                                bounds=[lo.tolist(), hi.tolist()], count=int(support.sum()),
                                source='known floor-height prior + observed horizontal plane' if args.floor_heights else 'LIO start/end height plateau + local horizontal plane',
                                status='prior_assisted_floor_candidate' if args.floor_heights else 'candidate_floor; drift not corrected'))
        selected = indices[support & (labels[indices] == 0)]
        labels[selected] = 6
        evidence[selected] = 2
    floor_nodes.sort(key=lambda n: n['z'])
    for i, node in enumerate(floor_nodes):
        node['id'] = f'F{i}'
    # Reject visually predicted structural labels contradicted by local shape.
    # Raw predictions are saved separately; these rejections are auditable.
    assert len(floor_nodes) >= 1, 'No supported floor candidate'
    floor_near = np.min(abs(cloud[:, 2, None] - np.array([f['z'] for f in floor_nodes])), axis=1) < .35
    reject_floor = (original_labels[indices] == 6) & (~horizontal | ~floor_near)
    reject_wall = (original_labels[indices] == 5) & ~(planar & (abs(normals[:, 2]) < .45))
    rejected = indices[reject_floor | reject_wall]
    labels[rejected] = 0
    evidence[rejected] = 3
    assert len(floor_nodes) >= 1, 'No supported floor candidate'

    nodes = []
    edges = []
    def add_node(nid, kind, position, label, source, **kwargs):
        node = dict(id=nid, kind=kind, position=np.asarray(position).tolist(),
                    label=label, source=source, **kwargs)
        nodes.append(node)
        return node

    def edge(a, b, relation, **kwargs):
        edges.append(dict(source=a, target=b, relation=relation, **kwargs))

    add_node('B0', 'building', [8, 2, 9.5], 'OfficeWorld recording',
             'dataset identity; not inferred building extent', status='container')
    for f in floor_nodes:
        add_node(f['id'], 'floor', f['position'], f['id'] + ' floor candidate',
                 f['source'], bounds=f['bounds'], z=f['z'], count=f['count'], status=f['status'])
        edge('B0', f['id'], 'contains')

    # Connected observed horizontal patches are regions, not certified rooms.
    regions = []
    for f in floor_nodes:
        mask = horizontal & (abs(cloud[:, 2] - f['z']) < .24)
        fp = cloud[mask]
        groups = cluster_ids(fp[:, :2], .48, 8, 140)
        if not groups:
            groups = [np.arange(len(fp))]
        for group in groups[:4]:
            p = fp[group]
            lo, hi = bounds(p)
            if min(hi[:2] - lo[:2]) < .4:
                continue
            rid = f'R{len(regions)}'
            region = add_node(rid, 'region', [*np.median(p[:, :2], axis=0), f['z'] + .45],
                              rid + ' observed region', 'connected observed horizontal surface',
                              floor=f['id'], count=len(p), bounds=[lo.tolist(), hi.tolist()],
                              status='room_candidate_only; no closed-room validation')
            regions.append(region)
            edge(f['id'], rid, 'contains_region')

    def parent_for(position, base_z=None):
        z = position[2] if base_z is None else base_z
        f = min(floor_nodes, key=lambda f: abs(f['z'] - z))
        candidates = [r for r in regions if r['floor'] == f['id']]
        r = min(candidates, key=lambda r: np.linalg.norm(np.array(r['position'])[:2] - position[:2])) if candidates else f
        return f['id'], r['id']

    for label in [1, 2, 3, 4]:
        # Only original visual predictions can create these object classes.
        p = points[(labels == label) & roi & (evidence == 1)]
        minimum = 30 if label != 2 else 18
        for ids in cluster_ids(p, .38 if label != 4 else .30, 5, minimum):
            group = p[ids]
            lo, hi = bounds(group)
            if label == 3 and (hi[2]-lo[2] < .35 or min(hi[:2]-lo[:2]) < .12):
                continue
            if label == 1 and min(hi[:2]-lo[:2]) < .65:
                continue
            center = (lo + hi) / 2
            kind = NAMES[label]
            nid = f'O{sum(n["kind"] in NAMES[1:5] for n in nodes)}'
            fid, parent = parent_for(center, lo[2])
            add_node(nid, kind, center, f'{kind} {nid}',
                     'recorded Grounding DINO + SAM2 / metric depth / LIO association',
                     bounds=[lo.tolist(), hi.tolist()], count=len(group), floor=fid,
                     semantic_label=label, status='predicted_class; not ground truth')
            edge(parent, nid, 'near_region')

    # Supported vertical planar patches: local RANSAC, not S-Graphs factors.
    remaining = np.flatnonzero(vertical)
    rng = np.random.default_rng(12)
    for wi in range(14):
        if len(remaining) < 230:
            break
        p = cloud[remaining]
        normal = normals[remaining, :2]
        candidates = rng.choice(len(p), min(80, len(p)), replace=False)
        best = None
        for candidate in candidates:
            n = normal[candidate]
            n = n / np.linalg.norm(n)
            match = (abs((p[:, :2] - p[candidate, :2]) @ n) < .10)
            match &= abs(normal @ n) > .94
            if best is None or match.sum() > best.sum():
                best = match
        if best.sum() < 230:
            break
        inliers = remaining[best]
        remaining = remaining[~best]
        groups = cluster_ids(cloud[inliers], .48, 5, 180)
        if not groups:
            continue
        inliers = inliers[groups[0]]
        lo, hi = bounds(cloud[inliers])
        if hi[2] - lo[2] < 1.1:
            continue
        selected = indices[inliers]
        unknown = selected[labels[selected] == 0]
        labels[unknown] = 5
        evidence[unknown] = 2
        center = (lo + hi) / 2
        fid, parent = parent_for(center, lo[2])
        nid = f'W{sum(n["kind"] == "wall" for n in nodes)}'
        add_node(nid, 'wall', center, f'wall {nid}', 'local vertical plane RANSAC',
                 bounds=[lo.tolist(), hi.tolist()], count=len(inliers), floor=fid,
                 semantic_label=5, status='geometric_hypothesis')
        edge(parent, nid, 'bounded_by_candidate')

    if len(floor_nodes) > 1:
        zlo, zhi = floor_nodes[0]['z'], floor_nodes[-1]['z']
        segment = trajectory[(trajectory[:, 3] > zlo + 1.1) & (trajectory[:, 3] < zhi - .2), 1:]
        stairs = [n for n in nodes if n['kind'] == 'stairs']
        close = [n['id'] for n in stairs if len(segment) and
                 n['bounds'][1][2] - n['bounds'][0][2] > 1.0 and
                 cKDTree(segment).query(np.array(n['position']))[0] < 3.5]
        if close:
            edge(floor_nodes[0]['id'], floor_nodes[-1]['id'], 'stairs_candidate',
                 supporting_objects=close, evidence_source='recorded LIO ascent + nearby visual stairs',
                 status='observed_transition; Nav2 traversal not validated')

    # Keep unobserved cells unknown. No fabricated ray tracing or freespace fill.
    mapdir = out / 'floor_maps'
    mapdir.mkdir(exist_ok=True)
    resolution = .10
    origin = np.floor(cloud[:, :2].min(axis=0) / resolution) * resolution
    shape = np.ceil((cloud[:, :2].max(axis=0) - origin) / resolution).astype(int) + 1
    for f in floor_nodes:
        free = cloud[horizontal & (abs(cloud[:, 2] - f['z']) < .22)]
        obstacles = cloud[(cloud[:, 2] > f['z'] + .27) & (cloud[:, 2] < f['z'] + 1.65)]
        grid = np.full((shape[1], shape[0]), 205, np.uint8)
        for p, value in [(free, 254), (obstacles, 0)]:
            xy = np.floor((p[:, :2] - origin) / resolution).astype(int)
            grid[xy[:, 1], xy[:, 0]] = value
        Image.fromarray(np.flipud(grid)).save(mapdir / (f['id'] + '.pgm'))
        (mapdir / (f['id'] + '.yaml')).write_text(
            f'image: {f["id"]}.pgm\nmode: trinary\nresolution: {resolution}\n'
            f'origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n')

    graph = dict(frame='lio_map', units='metres', nodes=nodes, edges=edges,
                 ontology=NAMES, ontology_zh=ONTOLOGY_ZH, colors=COLORS.tolist(),
                 display_roi=[[-3, -8, -1.5], [21, 13, args.z_max]],
                 source_recording=args.source_recording,
                 limitations=['saved recorded map, not a live mapping run',
                              'predicted semantics; geometric floor/room/wall hypotheses',
                              'no elevator observation; person detections, if any, are historical observations',
                              'no S-Graphs optimization or full Hydra frontend/backend',
                              'no loop closure or measured map accuracy',
                              'Nav2-format map previews, not validated navigation maps'])
    if args.floor_heights:graph['limitations'].append('Known floor-height priors and structural-calibrated input; not unassisted SLAM')
    # Keep machine-readable English identifiers and add a Chinese display view.
    graph = translate_graph(graph)
    (out / 'scene_graph.json').write_text(json.dumps(graph, indent=2))
    np.savez_compressed(out / 'semantic_cloud.npz', points=points, labels=labels,
                        colors=COLORS[labels], evidence=evidence, roi=roi,
                        original_labels=original_labels)
    np.save(out / 'trajectory.npy', trajectory)
    packed = np.zeros(len(points), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                                          ('rgb','<u4'),('label','u1'),('evidence','u1')])
    for i, key in enumerate('xyz'):
        packed[key] = points[:, i]
    colors = COLORS[labels].astype(np.uint32)
    packed['rgb'] = (colors[:,0]<<16) | (colors[:,1]<<8) | colors[:,2]
    packed['label'], packed['evidence'] = labels, evidence
    header = ('VERSION .7\nFIELDS x y z rgb label evidence\nSIZE 4 4 4 4 1 1\n'
              'TYPE F F F U U U\nCOUNT 1 1 1 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n')
    (out/'semantic_map.pcd').write_bytes(header.encode()+packed.tobytes())

    # Real MIT-SPARK Spark-DSG serialization, with explicit extra floor layer.
    sg = dsg.DynamicSceneGraph(empty=True)
    for layer, name in [(2,'OBJECTS'),(3,'REGIONS'),(4,'FLOORS'),(5,'BUILDINGS')]:
        sg.add_layer(layer, name=name)
    symbol = {}
    for n in nodes:
        layer = {'building': 5, 'floor': 4, 'region': 3}.get(n['kind'], 2)
        attrs = dsg.SemanticNodeAttributes()
        attrs.name = n['label']
        attrs.is_predicted = n.get('status') != 'container'
        attrs.position = np.array(n['position'], np.float64)
        attrs.semantic_label = n.get('semantic_label', 0)
        attrs.color = COLORS[n.get('semantic_label', 0)]
        if 'bounds' in n:
            lo, hi = np.array(n['bounds'])
            attrs.bounding_box = dsg.BoundingBox((hi-lo).astype(np.float32), ((hi+lo)/2).astype(np.float32))
        attrs.metadata.set(n)
        symbol[n['id']] = dsg.NodeSymbol(n['id'][0], int(n['id'][1:]))
        assert sg.add_node(layer, symbol[n['id']], attrs)
    for e in edges:
        attrs = dsg.EdgeAttributes()
        attrs.metadata.set(e)
        assert sg.insert_edge(symbol[e['source']], symbol[e['target']], attrs)
    sg.metadata.set({'pipeline': 'Wavego semantic hierarchy adapter', 'limitations': graph['limitations']})
    sg.save(out / 'spark_dsg.json')
    loaded = dsg.DynamicSceneGraph.load(out / 'spark_dsg.json')
    assert loaded.num_nodes() == len(nodes) and loaded.num_edges() == len(edges)
    hashes = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [source, args.semantics, args.trajectory]}
    counts = {k: sum(n['kind'] == k for n in nodes) for k in set(n['kind'] for n in nodes)}
    report = dict(total_points=len(points), roi_points=int(roi.sum()),
                  visual_label_points=int((evidence == 1).sum()),
                  geometry_hypothesis_points=int((evidence == 2).sum()),
                  rejected_structural_predictions=int(len(rejected)),
                  class_counts={NAMES[i]: int((labels == i).sum()) for i in range(len(NAMES))},
                  graph_node_counts=counts, edges=len(edges), floor_candidates=floor_nodes,
                  dsg_roundtrip=True, resolution=resolution, source_hashes=hashes,
                  source_geometry=str(source), limitations=graph['limitations'])
    (out / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
