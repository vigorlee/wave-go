#!/usr/bin/env python3
"""Run the pinned, unmodified HOV-SG floor segmentation method on our measured map.

Only this geometry stage runs: no CLIP/SAM model or room/instance pipeline is
substituted. AST extraction bypasses unrelated model imports, not algorithm code.
"""
import ast
import hashlib
import importlib.util
import json
import os
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN
from scene_profile import DATA_ROOT

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / '.runtime/HOV-SG'
OUT = DATA_ROOT / 'outputs/three_representations/hovsg_floor_probe'
SHA = 'd6e65a53c8be6faec3f01f00d1644d967f89e605'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    actual = subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != SHA:
        raise RuntimeError(f'Expected reviewed HOV-SG {SHA}; found {actual}')
    filename = UPSTREAM / 'hovsg/graph/graph.py'
    source = filename.read_text()
    tree = ast.parse(source)
    graph = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Graph')
    method = next(n for n in graph.body if isinstance(n, ast.FunctionDef) and n.name == 'segment_floors')
    # Compile the exact upstream AST, without edits to constants, control flow,
    # or thresholds. The Floor data class is also imported from upstream.
    spec = importlib.util.spec_from_file_location('upstream_hovsg_floor', UPSTREAM / 'hovsg/graph/floor.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    scope = dict(os=os, np=np, o3d=o3d, plt=plt, gaussian_filter1d=gaussian_filter1d,
                 find_peaks=find_peaks, Rotation=Rotation, DBSCAN=DBSCAN, Floor=module.Floor)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(filename), 'exec'), scope)
    data = np.load(DATA_ROOT / 'outputs/reference_demo/mapping_replay.npz')
    xyz = data['points']
    # ROS Z-up -> HOV-SG Y-up: determinant +1, no scaling or geometry changes.
    transform = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz @ transform.T))
    state = SimpleNamespace(full_pcd=cloud, floors=[], graph_tmp_folder=str(OUT),
                            cfg=SimpleNamespace(pipeline=SimpleNamespace(save_intermediate_results=True)))
    start = time.perf_counter()
    result = dict(repository='https://github.com/hovsg/HOV-SG', commit=SHA,
                  scope='Graph.segment_floors only; NOT full HOV-SG',
                  source_sha256=hashlib.sha256(filename.read_bytes()).hexdigest(),
                  method_sha256=hashlib.sha256(ast.get_source_segment(source, method).encode()).hexdigest(),
                  source_lines=[method.lineno, method.end_lineno], input_points=len(xyz),
                  input_frame='lio_map', input_to_hovsg_rotation=transform.tolist(),
                  input_file=str(DATA_ROOT / 'outputs/reference_demo/mapping_replay.npz'))
    try:
        intervals = scope['segment_floors'](state, str(OUT), flip_zy=False)
        result.update(status='completed', floor_intervals_z_m=np.asarray(intervals).tolist(),
                      num_floors=len(state.floors),
                      points_per_floor=[len(f.pcd.points) for f in state.floors])
        floor_labels=np.zeros(len(xyz),np.uint8)
        for i,(lo,hi) in enumerate(intervals):
            floor_labels[(xyz[:,2]>=lo)&(xyz[:,2]<=hi)]=i+1
            assert np.count_nonzero(floor_labels==i+1)==len(state.floors[i].pcd.points)
        np.savez_compressed(OUT/'floor_labels.npz',labels=floor_labels)
        result['unassigned_points']=int(np.count_nonzero(floor_labels==0))
        for f in state.floors:
            f.save(str(OUT))
    except Exception as exc:
        result.update(status='failed', error=f'{type(exc).__name__}: {exc}')
    result['stage_wall_seconds'] = time.perf_counter() - start
    result['note'] = 'No floor ground truth; this is a compatibility probe, not an accuracy or full-pipeline speed benchmark.'
    (OUT / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
