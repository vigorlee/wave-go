#!/usr/bin/env python3
"""Native, single-window RViz feature views for an imported Z<=5 m LAZ scan."""
import argparse
import json
import os
import signal
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import rospy
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QCheckBox, QHBoxLayout, QLabel,
                             QPushButton, QVBoxLayout, QWidget)

from imported_923_demo import ImportedMap
from imported_923_io import read_pcd

ROOT = Path(__file__).resolve().parents[1]
MODES = [('graph', '01  Graph 关系图'),
         ('boxes', '02  语义候选 + Box'),
         ('zheight', '03  纯点云 / 高度'),
         ('combined', '04  融合叠加'),
         ('height', '分类点云'), ('rgb', '原始 RGB')]
CAPTIONS = {
    'rgb': '原始测量 RGB；仅保留原始坐标 Z ≤ 5.00 m',
    'zheight': '同一份原始测量点云，以 RViz FAST-LIO 风格按 Z 高度着色；颜色不表示类别',
    'height': '同一份点云的分层几何着色；二层踏步从平台下方分离，垂直墙由平面拟合验证',
    'boxes': '分类着色点云 + 一层／二层楼梯、二楼楼层平台、坡道、墙及障碍物的几何候选 Box 与中文标签',
    'graph': '几何候选关系图；节点和边位于测量坐标，背景为 Z 高度着色点云',
    'combined': '点云 + 特征着色 + Box + Metric Graph，均为同一原始坐标'}


def save_native_window(window, path):
    """Read the RViz X11 child surface, which QWidget.grab leaves black."""
    with tempfile.TemporaryDirectory(prefix='wavego_xwd_') as directory:
        capture = str(Path(directory) / 'window.xwd')
        subprocess.run(['xwd', '-silent', '-id', str(int(window.winId())),
                        '-out', capture], check=True, stdout=subprocess.DEVNULL)
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', capture,
                        '-frames:v', '1', str(path)], check=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, type=Path)
    parser.add_argument('--capture', action='store_true')
    parser.add_argument('--capture-only', choices=['overview_color', 'overview_fusion',
                                                  'corridor', 'corridor_semantic',
                                                  'corridor_graph', 'corridor_color'] +
                        [f'{side}_stairs_{view}' for side in ('left', 'right')
                         for view in ('side', 'front', 'color', 'semantic', 'graph')],
                        help='Save just one stair view for camera inspection')
    args = parser.parse_args()
    data = args.data
    graph = json.loads((data / 'feature_graph.json').read_text())
    report = json.loads((data / 'feature_report.json').read_text())
    stitched = bool(graph.get('coordinate_transform_applied'))
    raw = read_pcd(data / 'display_z5_rgb.pcd')
    xlo, xhi = np.quantile(raw['x'], [.01, .99])
    ylo, yhi = np.quantile(raw['y'], [.01, .99])
    mid = [float((xlo + xhi) / 2), float((ylo + yhi) / 2),
           float(np.median(raw['z']))]
    distance = float(max(22, .85 * max(xhi - xlo, yhi - ylo)))
    feature_nodes = [n for n in graph['nodes'] if n['kind'] != 'ground']
    major_nodes = [n for n in feature_nodes if n['kind'] in ('stairs', 'ramp')]
    upper_nodes = [n for n in major_nodes if n.get('level') == 'upper']
    landings = [n for n in feature_nodes if n['kind'] == 'stair_landing']
    upper_floors = {n['evidence']['adjacent_upper_stair_id']: n for n in feature_nodes
                    if n['kind'] == 'upper_floor'}
    corridors = [n for n in feature_nodes if n['kind'] == 'upper_corridor']
    clustered = [n for n in major_nodes if abs(np.mean(n['bounds'], axis=0)[0] -
                                               np.median([np.mean(m['bounds'], axis=0)[0]
                                                          for m in major_nodes])) < 6]
    focal_feature = np.median([np.mean(n['bounds'], axis=0) for n in clustered], axis=0).tolist()
    upper_centers = np.asarray([np.mean(n['bounds'], axis=0) for n in upper_nodes])
    upper_focus = np.median(upper_centers, axis=0).tolist() if len(upper_nodes) else mid
    if len(upper_nodes) > 1:
        upper_focus[1] = float((upper_centers[:, 1].min() + upper_centers[:, 1].max()) / 2)
    elif len(upper_nodes) == 1:
        walls = [n for n in graph['nodes'] if n['kind'] == 'vertical_wall']
        if walls:
            wall_center = np.mean(walls[0]['bounds'], axis=0)
            upper_focus = ((upper_centers[0] + wall_center) / 2).tolist()
    upper_distance = float(max(30., .82 * np.ptp(upper_centers[:, 1]))
                           if len(upper_nodes) > 1 else 23.)
    camera_values = {'overview': [distance * (1.18 if stitched else 1),
                                  .72 if stitched else .76,
                                  -.24 if stitched else .64, mid],
                     'top': [distance * (1.4 if stitched else 1.16), 1.55, .63, mid],
                     'side': [distance * .8, .25, 1.38, mid],
                     'feature': [13., .78, .74, focal_feature],
                     'upper': [upper_distance, .34, .05, upper_focus]}
    if corridors:
        corridor_center = np.mean(corridors[0]['bounds'], axis=0).tolist()
        camera_values['corridor'] = [36., 1.18, -.24, corridor_center]
    # In the requested overview orientation, positive Y is the left-hand
    # staircase.  The negative-Y landing in the 12 scan is the right-hand one.
    pair_landings = {}
    for landing in landings:
        side = 'left' if np.mean(landing['bounds'], axis=0)[1] > 0 else 'right'
        if side in pair_landings:
            raise ValueError(f'More than one {side} stair landing')
        pair_landings[side] = landing
        linked = [n for n in feature_nodes if n['id'] in
                  (landing['evidence']['lower_stair_id'],
                   landing['evidence']['upper_stair_id'])]
        floor = upper_floors.get(landing['evidence']['upper_stair_id'])
        focus_nodes = linked + [landing] + ([floor] if floor else [])
        focus = np.mean([np.mean(n['bounds'], axis=0) for n in focus_nodes], axis=0)
        camera_values[f'{side}_stairs_side'] = [9.2 if side == 'left' else 9.5,
                                                 .42, 1.57, focus.tolist()]
        camera_values[f'{side}_stairs_front'] = [11.0 if side == 'left' else 12.5,
                                                  .38, 3.14, focus.tolist()]
    frame = graph['frame']
    runtime = ROOT / '.runtime' / frame
    runtime.mkdir(parents=True, exist_ok=True)
    rospy.init_node('wavego_' + frame + '_viewer', disable_signals=True)
    app = QApplication([])
    window = QWidget()
    if args.capture:
        window.setWindowFlag(Qt.WindowStaysOnTopHint, True)
    window.setWindowTitle(f'Wavego | {report["name"].upper()} Z≤5m 特征地图')
    window.setStyleSheet('QWidget{background:#ffffff;color:#25354c;} QLabel{font-size:15px;} '
                         'QPushButton{padding:10px 14px;border:1px solid #cbd7e3;'
                         'border-radius:5px;background:#f5f7fa;} '
                         'QPushButton:checked{background:#233c5c;color:white;}')
    outer = QVBoxLayout(window)
    title = QLabel(f'<b style="font-size:24px">{report["name"].upper()} · Z ≤ 5 m 三维特征地图</b><br>'
                   f'原始 {report["source_points"]:,} 点 → 完整裁切 {report["full_z5_points"]:,} 点'
                   f' · 显示 {report["display_z5_points"]:,} 点'
                   f' · 楼梯 {report["stair_flights"]} 处（其中二层 {report["upper_stair_flights"]} 处）'
                   f' · 二楼楼层平台 {len(upper_floors)} 处'
                   + (f' · 二楼长廊 {corridors[0]["evidence"]["longitudinal_span_m"]:.1f} m'
                      if corridors else '') +
                   f' · 坡道 {report["ramps"]} 处 · 垂直墙 {report["vertical_walls"]} 面'
                   + (f' · MT 局部配准 {report["registered_source_points"]:,} 点' if stitched else ''))
    outer.addWidget(title)
    bar = QHBoxLayout()
    buttons = {}
    current = ['combined']
    labels = QCheckBox('中文标签')
    labels.setChecked(True)

    pair_descriptions, pair_notes = {}, {}

    def choose(name):
        current[0] = name
        for key, button in buttons.items():
            button.setChecked(key == name)
        side = name.removesuffix('_chain') if name.endswith('_chain') else None
        view_mode = ('colored' if name == 'height' else
                     'combined' if side or name == 'corridor' else name)
        view.choose(view_mode, labels.isChecked())
        subtitle.setText(pair_descriptions[side] if side else
                         corridor_description if name == 'corridor' else CAPTIONS[name])
        chain_note.setVisible(bool(side or name == 'corridor'))
        if side:
            chain_note.setText(pair_notes[side])
            set_camera(f'{side}_stairs_side')
        elif name == 'corridor':
            chain_note.setText(corridor_note)
            set_camera('corridor')

    mode_buttons = MODES.copy()
    for side, caption in [('right', '右侧'), ('left', '左侧')]:
        if side in pair_landings:
            mode_buttons.insert(4, (f'{side}_chain', f'{caption}一二楼连续台阶'))
    if corridors:
        mode_buttons.insert(6, ('corridor', '二楼长廊'))
    for key, caption in mode_buttons:
        button = QPushButton(caption)
        button.setCheckable(True)
        button.clicked.connect(lambda _, selected=key: choose(selected))
        buttons[key] = button
        bar.addWidget(button)
    bar.addStretch()
    outer.addLayout(bar)
    subtitle = QLabel()
    outer.addWidget(subtitle)
    chain_note = QLabel()
    corridor_description = ''
    corridor_note = ''
    if corridors:
        evidence = corridors[0]['evidence']
        corridor_description = (f'C1 二楼长廊：沿实测墙侧连续观察约 '
                                f'{evidence["longitudinal_span_m"]:.1f} m；'
                                f'只为实测近水平点着色，{len(evidence["sections"])} 个短段轮廓不填充缺测区')
        corridor_note = (f'<span style="color:#a2aa47;font-weight:700">'
                         f'C1 二楼长廊 · {evidence["measured_points"]:,} 个实测点</span>'
                         '　连接左右两处二楼楼梯口平台；高度随测量剖面变化')
    for side, landing in pair_landings.items():
        low_id = landing['evidence']['lower_stair_id']
        high_id = landing['evidence']['upper_stair_id']
        stair_by_id = {node['id']: node for node in feature_nodes}
        low_stair, high_stair = stair_by_id[low_id], stair_by_id[high_id]
        floor = upper_floors.get(high_id)
        pair_descriptions[side] = (f'{"左" if side == "left" else "右"}侧：{low_id} 一楼台阶'
                                   f' {low_stair["evidence"]["number_of_steps"]} 级 → '
                                   f'{landing["id"]} 实测平台 → {high_id} 二楼台阶 '
                                   f'{high_stair["evidence"]["number_of_steps"]} 级'
                                   + (f' → {floor["id"]} 二楼楼层平台；' if floor else '；') +
                                   '黄线为有实测支持的邻接关系')
        pair_notes[side] = (
            f'<span style="color:#4a8ede;font-weight:700">{low_id} 一楼楼梯</span>'
            '　→　'
            f'<span style="color:#b38b23;font-weight:700">{landing["id"]} 连接平台</span>'
            '　→　'
            f'<span style="color:#218ca4;font-weight:700">{high_id} 二楼楼梯</span>'
            + (f'　→　<span style="color:#a2aa47;font-weight:700">{floor["id"]} 二楼楼层平台</span>'
               if floor else '')
            + ('　　MT 左侧下段局部配准到 12 坐标；均为实测点'
               if stitched and side == 'left' else
               '　　同一份 12 实测点云 · 同一坐标' if stitched else
               '　　同一份实测点云 · 同一坐标'))
    chain_note.setStyleSheet('font-size:16px;padding:5px 8px;background:#f3f7fb;')
    chain_note.setVisible(False)
    outer.addWidget(chain_note)
    holder = QWidget()
    canvas = QVBoxLayout(holder)
    canvas.setContentsMargins(0, 0, 0, 0)
    view = ImportedMap(holder, canvas, runtime, frame, camera_values,
                       height_bounds=(min(-1.5, report['ground_z_m'] - .3), 5.0))
    outer.addWidget(holder, 1)

    def set_camera(preset):
        focus_ids = [n['id'] + ' ' for n in upper_nodes]
        focus_ids += [n['id'] + ' ' for n in graph['nodes']
                      if n['kind'] == 'vertical_wall']
        if preset == 'upper':
            view.label_prefixes = tuple(focus_ids)
        elif preset == 'corridor':
            ids = [n['id'] for n in corridors] + [n['id'] for n in upper_floors.values()]
            ids += [n['id'] for n in upper_nodes]
            view.label_prefixes = tuple(i + ' ' for i in ids)
        elif preset.endswith(('_stairs_side', '_stairs_front')):
            side = preset.split('_')[0]
            landing = pair_landings[side]
            ids = (landing['evidence']['lower_stair_id'],
                   landing['evidence']['upper_stair_id'], landing['id'])
            floor = upper_floors.get(landing['evidence']['upper_stair_id'])
            ids += (floor['id'],) if floor else ()
            view.label_prefixes = tuple(i + ' ' for i in ids)
        else:
            view.label_prefixes = None
        view.set_topic_scope('corridor' if preset == 'corridor' else
                             preset.split('_')[0] if preset.endswith(
                                 ('_stairs_side', '_stairs_front')) else None)
        view.camera(preset)
        view.draw_labels()
    swatches = [('地面', '#6ab091'), ('楼梯', '#4a8ede'), ('二层楼梯', '#42bfdb'),
                ('楼梯平台', '#e6be61')]
    if upper_floors:
        swatches.append(('二楼楼层平台', '#ced37e'))
    swatches += [
                ('坡道', '#b18fd4'), ('垂直墙', '#e0777c'),
                ('水平结构', '#ccb173'), ('立面结构', '#7599b5'),
                ('其他障碍', '#db9168'), ('未分类', '#6c798a')]
    legend = QLabel('　'.join(f'<span style="color:{color};font-size:21px">●</span> {name}'
                              for name, color in swatches))
    outer.addWidget(legend)
    note = QLabel('依据：局部法向量与高度剖面；标签均为几何候选，不代表训练模型识别或人工真值。'
                  + (' 左侧下段及平台为 MT 实测点经局部平移配准，非全局建图。' if stitched else ''))
    outer.addWidget(note)
    footer = QHBoxLayout()
    footer.addWidget(labels)
    labels.toggled.connect(lambda _: choose(current[0]))
    footer.addStretch()
    for caption, preset in ([('总览', 'overview'), ('俯视', 'top'),
                             ('侧视', 'side'), ('障碍细节', 'feature'),
                             ('二层楼梯／墙', 'upper')]
                             + ([('二楼长廊全景', 'corridor')] if corridors else [])
                             + [(f'{"左" if side == "left" else "右"}侧楼梯侧视',
                                 f'{side}_stairs_side') for side in ('left', 'right')
                                if side in pair_landings]
                             + [(f'{"左" if side == "left" else "右"}侧楼梯正视',
                                 f'{side}_stairs_front') for side in ('left', 'right')
                                if side in pair_landings]):
        button = QPushButton(caption)
        button.clicked.connect(lambda _, p=preset: set_camera(p))
        footer.addWidget(button)
    outer.addLayout(footer)
    window.resize(1850, 1060)
    window.show()
    choose('combined' if stitched else
           'left_chain' if 'left' in pair_landings else
           'right_chain' if 'right' in pair_landings else 'combined')
    if stitched or not pair_landings:
        set_camera('overview')
    (runtime / 'viewer_window.json').write_text(json.dumps({'window_id': int(window.winId())}))

    if args.capture:
        shots = [('01_z5_rgb', 'rgb', 'overview'),
                 ('02_z5_feature_colors', 'height', 'overview'),
                 ('03_z5_feature_boxes', 'boxes', 'overview'),
                 ('04_z5_metric_graph', 'graph', 'overview'),
                 ('05_z5_fusion', 'combined', 'overview'),
                 ('12_z5_fastlio_cloud', 'zheight', 'overview'),
                 ('06_z5_top', 'combined', 'top'),
                 ('07_z5_stair_ramp_detail', 'boxes', 'feature'),
                 ('08_upper_stairs_wall', 'combined', 'upper')]
        for side in ('left', 'right'):
            if side not in pair_landings:
                continue
            number = {'left': ('09', '10', '11', '13', '14'),
                      'right': ('15', '16', '17', '18', '19')}[side]
            shots.extend([(f'{number[0]}_{side}_stairs_detail', f'{side}_chain',
                           f'{side}_stairs_side'),
                          (f'{number[1]}_{side}_stairs_front', 'combined',
                           f'{side}_stairs_front'),
                          (f'{number[2]}_{side}_stairs_color', 'height',
                           f'{side}_stairs_side'),
                          (f'{number[3]}_{side}_stairs_semantic', 'boxes',
                           f'{side}_stairs_side'),
                          (f'{number[4]}_{side}_stairs_graph', 'graph',
                           f'{side}_stairs_side')])
        if corridors:
            shots.extend([('20_upper_corridor', 'corridor', 'corridor'),
                          ('21_upper_corridor_semantic', 'boxes', 'corridor'),
                          ('22_upper_corridor_graph', 'graph', 'corridor'),
                          ('23_upper_corridor_color', 'height', 'corridor')])
        if args.capture_only:
            if args.capture_only in ('overview_color', 'overview_fusion'):
                shots = ([('02_z5_feature_colors', 'height', 'overview')] if
                         args.capture_only == 'overview_color' else
                         [('05_z5_fusion', 'combined', 'overview')])
            elif args.capture_only.startswith('corridor'):
                if not corridors:
                    raise ValueError('No measured upper corridor in this map')
                kind = args.capture_only.removeprefix('corridor_')
                shot = {'corridor': ('20_upper_corridor', 'corridor', 'corridor'),
                        'semantic': ('21_upper_corridor_semantic', 'boxes', 'corridor'),
                        'graph': ('22_upper_corridor_graph', 'graph', 'corridor'),
                        'color': ('23_upper_corridor_color', 'height', 'corridor')}
                shots = [shot[kind]]
            else:
                side = args.capture_only.split('_')[0]
                if side not in pair_landings:
                    raise ValueError(f'No measured {side} landing in this map')
                suffix = args.capture_only.removeprefix(side + '_stairs_')
                shots = [next(shot for shot in shots
                              if shot[0].endswith(f'_{side}_stairs_' +
                                                  {'side': 'detail', 'front': 'front',
                                                   'color': 'color', 'semantic': 'semantic',
                                                   'graph': 'graph'}[suffix]))]

        def next_shot():
            if not shots:
                app.quit()
                return
            name, mode, preset = shots.pop(0)
            choose(mode)
            set_camera(preset)

            def save():
                window.raise_()
                window.activateWindow()
                save_native_window(window, data / (name + '.png'))
                audit = json.loads((runtime / 'imported_display_state.json').read_text())
                audit['ui_mode'] = mode
                audit['camera_preset'] = preset
                audit['visible_labels'] = json.loads((runtime / 'imported_visible_labels.json').read_text())
                (data / (name + '.json')).write_text(json.dumps(audit, ensure_ascii=False, indent=2))
                print('CAPTURE', report['name'], name, flush=True)
                QTimer.singleShot(120, next_shot)

            QTimer.singleShot(2300, save)

        QTimer.singleShot(4700, next_shot)

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    app.exec_()
    view.close()


if __name__ == '__main__':
    main()
