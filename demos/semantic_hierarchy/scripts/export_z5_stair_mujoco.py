#!/usr/bin/env python3
"""Export a measured stair *proxy* for a MuJoCo import check, not a full PCD world.

The scanned map is never changed.  This exporter converts one supported stair
cross-section into solid collision boxes and records every interpolation and
quantization decision.  It does not assert that the robot can navigate it.
"""
import argparse
import csv
import json
from pathlib import Path
import re
import struct
import zlib

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def png_write(path, array):
    height, width = array.shape[:2]
    rows = b''.join(b'\0' + array[y].tobytes() for y in range(height))

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    path.write_bytes(b'\x89PNG\r\n\x1a\n' +
                     chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)) +
                     chunk(b'IDAT', zlib.compress(rows, 6)) + chunk(b'IEND', b''))


def profile_from_heightmap(heightmap, graph, floor_z, step_m=.15):
    stairs = {n['id']: n for n in graph['nodes'] if n['kind'] == 'stairs'}
    landing = next(n for n in graph['nodes'] if n['kind'] == 'stair_landing')
    low = stairs[landing['evidence']['lower_stair_id']]
    high = stairs[landing['evidence']['upper_stair_id']]
    lower = heightmap['height']
    upper = heightmap['upper_height']
    origin = heightmap['origin']
    resolution = float(heightmap['resolution'])
    xlo = float(low['bounds'][0][0] - .4)
    xhi = float(high['bounds'][1][0] + .35)
    ylo = max(low['bounds'][0][1], high['bounds'][0][1]) + .25
    yhi = min(low['bounds'][1][1], high['bounds'][1][1]) - .25
    if yhi - ylo < .8:
        raise ValueError('Stair flights have no supported shared width')
    # Use the lower height layer before the landing and the upper layer after
    # it; a single top-down layer cannot express both stairs and raised floor.
    switch_x = float(np.mean(landing['bounds'], axis=0)[0])
    bin_m = .15
    edges = np.arange(xlo, xhi + bin_m, bin_m)
    centers = (edges[:-1] + edges[1:]) / 2
    estimates = np.full(len(centers), np.nan)
    support_cells = np.zeros(len(centers), int)
    jlo = max(0, int(np.floor((ylo - origin[1]) / resolution)))
    jhi = min(lower.shape[1], int(np.ceil((yhi - origin[1]) / resolution)))
    for i, (a, b, x) in enumerate(zip(edges[:-1], edges[1:], centers)):
        ilo = max(0, int(np.floor((a - origin[0]) / resolution)))
        ihi = min(lower.shape[0], int(np.ceil((b - origin[0]) / resolution)))
        patch = (lower if x < switch_x else upper)[ilo:ihi, jlo:jhi]
        finite = patch[np.isfinite(patch)]
        if len(finite) >= 3:
            estimates[i] = float(np.median(finite))
            support_cells[i] = len(finite)
    valid = np.isfinite(estimates)
    if valid.sum() < .65 * len(estimates):
        raise ValueError('Too many unobserved stair bins for a collision proxy')
    interpolated = int((~valid).sum())
    estimates = np.interp(centers, centers[valid], estimates[valid])
    monotone = np.maximum.accumulate(estimates)
    # A proxy made from 15 cm vertical blocks.  These are collision solids,
    # deliberately distinct from the raw measured point cloud.
    steps = np.maximum(0, np.rint((monotone - floor_z) / step_m).astype(int))
    top_z = steps * step_m
    return edges, centers, estimates, top_z, support_cells, interpolated, landing


def export(name, matrix_root):
    data = ROOT / 'outputs' / f'imported_{name}_20260926' / 'z5_features'
    graph = json.loads((data / 'feature_graph.json').read_text())
    report = json.loads((data / 'feature_report.json').read_text())
    heightmap = np.load(data / 'feature_heightmap.npz')
    floor_z = report['ground_z_m']
    edges, centers, raw_heights, heights, support, filled, landing = profile_from_heightmap(
        heightmap, graph, floor_z)
    output = data / 'mujoco_stair_proxy'
    output.mkdir(exist_ok=True)
    origin_x = float(edges[0] - 1.2)
    y_center = float(np.mean(landing['bounds'], axis=0)[1])
    # Source X becomes simulator X; simulator Z is zeroed at the measured
    # dominant ground.  The selected corridor is a local proxy, not a map fit.
    half_width = 1.0
    robot_variant = f'go2w_z5_{name}_aligned.xml'
    parts = ['<mujoco model="z5 measured stair collision proxy">',
             f'  <include file="{robot_variant}"/>',
             '  <statistic center="4 0 1.1" extent="12"/>',
             '  <visual><global offwidth="1280" offheight="720"/></visual>',
             '  <worldbody>',
             '    <light pos="4 0 6" dir="0 0 -1" directional="true"/>',
             '    <geom name="measured_ground_proxy" type="plane" size="0 0 0.01"'
             ' rgba="0.30 0.36 0.42 1" friction="0.8 0.02 0.01"/>']
    for i, (a, b, top) in enumerate(zip(edges[:-1], edges[1:], heights)):
        if top < .03:
            continue
        center = (a + b) / 2 - origin_x
        kind = ('lower' if centers[i] < landing['bounds'][0][0] else
                'landing' if centers[i] <= landing['bounds'][1][0] else 'upper')
        color = {'lower': '0.29 0.56 0.87 1',
                 'landing': '0.90 0.74 0.38 1',
                 'upper': '0.26 0.75 0.86 1'}[kind]
        parts.append(f'    <geom name="pcd{name}_{kind}_{i:02d}" type="box"'
                     f' pos="{center:.4f} 0 {top/2:.4f}"'
                     f' size="{(b-a)/2:.4f} {half_width:.4f} {top/2:.4f}"'
                     f' rgba="{color}" friction="0.9 0.02 0.01"/>')
    parts.extend(['  </worldbody>', '</mujoco>'])
    xml = '\n'.join(parts) + '\n'
    portable = output / f'scene_z5_{name}_stair_proxy.xml'
    portable.write_text(xml)
    tiny_pcd = output / 'direct_import_probe.pcd'
    tiny_pcd.write_text('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\n'
                        'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        'WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n'
                        '0 0 0\n1 0 0\n0 1 0\n')
    direct_test = output / 'direct_import_probe.xml'
    direct_test.write_text('<mujoco><asset><mesh name="scan" file="direct_import_probe.pcd"/>'
                           '</asset><worldbody><geom type="mesh" mesh="scan"/>'
                           '</worldbody></mujoco>')
    try:
        mujoco.MjModel.from_xml_path(str(direct_test))
        direct_error = None
    except ValueError as exc:
        direct_error = str(exc)
    runtime_dir = matrix_root / 'matrix/src/robot_mujoco/zsibot_robots/go2w'
    runtime_path = runtime_dir / portable.name
    # The stock Go2-W starts facing +Y.  This isolated copy rotates only the
    # spawn pose toward the measured ascending +X axis; source go2w.xml stays
    # untouched and remains the baseline for every other MATRiX scene.
    robot_source = (runtime_dir / 'go2w.xml').read_text()
    robot_aligned, replacements = re.subn(
        r'(<body name="base_link" pos=")[^"]*(" quat=")[^"]*',
        r'\g<1>0.65 0 0.65\g<2>1 0 0 0', robot_source, count=1)
    if replacements != 1:
        raise ValueError('Could not isolate the Go2-W initial pose')
    (output / robot_variant).write_text(robot_aligned)
    (runtime_dir / robot_variant).write_text(robot_aligned)
    runtime_path.write_text(xml)
    with (output / 'measured_profile.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_x_m', 'source_height_z_m', 'proxy_top_above_floor_m',
                         'measured_heightmap_cells'])
        writer.writerows(zip(centers, raw_heights, heights, support))
    model = mujoco.MjModel.from_xml_path(str(runtime_path))
    state = mujoco.MjData(model)
    mujoco.mj_forward(model, state)
    geom_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                  for i in range(model.ngeom)]
    exported_count = sum(geom_name is not None and geom_name.startswith(f'pcd{name}_')
                         for geom_name in geom_names)
    if exported_count < .8 * len(heights):
        raise AssertionError('Collision geometry did not load')
    result = {'source_pcd': graph['source_pcd'], 'source_map_frame': graph['frame'],
              'direct_pcd_import': direct_error is None,
              'direct_pcd_error': direct_error, 'collision_proxy': True,
              'source_x_range_m': [float(edges[0]), float(edges[-1])],
              'source_y_corridor_m': [float(y_center - half_width),
                                     float(y_center + half_width)],
              'source_floor_z_m': floor_z, 'sim_x_origin_source_m': origin_x,
              'sim_y_center_source_m': y_center,
              'spawn_yaw_deg': 0,
              'heightmap_bins': len(heights), 'interpolated_empty_bins': filled,
              'height_quantization_m': .15,
              'height_error_median_m': float(np.median(np.abs(
                  heights - (raw_heights - floor_z)))),
              'height_error_max_m': float(np.max(np.abs(
                  heights - (raw_heights - floor_z)))),
              'mujoco_compiled': True, 'mujoco_ngeom': model.ngeom,
              'exported_stair_collision_geoms': exported_count,
              'mujoco_nq': model.nq, 'nav2_navigation_test': 'not_run',
              'local_rl_trial_report': (str(output / 'locomotion_trial.json')
                                        if (output / 'locomotion_trial.json').exists()
                                        else None),
              'runtime_scene': str(runtime_path),
              'limitations': ['one observed stair corridor only',
                              'no scanned walls or full-building collision mesh',
                              'no UE scene, localization or navigation map']}
    try:
        renderer = mujoco.Renderer(model, 720, 1280)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = [4.6, 0, 1.45]
        camera.distance = 13
        camera.azimuth = 140
        camera.elevation = -18
        renderer.update_scene(state, camera=camera)
        png_write(output / 'mujoco_proxy_preview.png', renderer.render())
        renderer.close()
        result['preview_png'] = str(output / 'mujoco_proxy_preview.png')
    except Exception as exc:
        result['preview_error'] = str(exc)
    (output / 'import_test.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', choices=['12', 'mt'], default='12')
    parser.add_argument('--matrix-root', type=Path,
                        default=Path('/home/unitree/Go2w+cosmos3Nav/matrix_go2w_lcm_demo'))
    args = parser.parse_args()
    export(args.name, args.matrix_root)
