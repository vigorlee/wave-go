#!/usr/bin/env python3
"""Single-window native RViz demo for imported geometric maps."""
import argparse
import ctypes
import json
import os
import signal
from pathlib import Path

import numpy as np
import rospy
import yaml
from PyQt5 import sip
from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QCheckBox, QDockWidget, QHBoxLayout,
                             QLabel, QPushButton, QToolBar, QVBoxLayout, QWidget)
from rviz import bindings as rviz
from visualization_msgs.msg import Marker, MarkerArray
from imported_923_io import read_pcd

ROOT = Path(__file__).resolve().parents[1]


class ImportedMap(QObject):
    received = pyqtSignal(str, object)

    def __init__(self, parent, layout, runtime, frame='imported_923', camera_values=None,
                 height_bounds=None):
        super().__init__(parent)
        self.runtime = runtime
        self.frame_id = frame
        self.topic_prefix = '/' + frame
        self.camera_values = camera_values or {'overview': [20, .64, .6, [2, 4, .4]],
                                               'top': [22, 1.56, .6, [2, 4, .3]],
                                               'side': [20, .21, 1.4, [2, 4, .3]]}
        self.height_bounds = height_bounds
        self.markers = {'boxes': {}, 'graph': {}}
        self.mode = 'combined'
        self.topic_scope = ''
        self.show_labels = True
        self.label_prefixes = None
        self.frame = rviz.VisualizationFrame(parent)
        self.frame.setWindowFlags(Qt.Widget)
        layout.addWidget(self.frame, 1)
        self.frame.setSplashPath('')
        self.frame.initialize()
        config = yaml.safe_load((ROOT / 'config/three_representations.rviz').read_text())
        visual = config['Visualization Manager']
        visual['Global Options']['Fixed Frame'] = frame
        visual['Global Options']['Background Color'] = '5; 12; 24'
        distance, pitch, yaw, focal = self.camera_values['overview']
        visual['Views']['Current'].update({'Distance': distance, 'Pitch': pitch, 'Yaw': yaw,
                                           'Focal Point': dict(zip('XYZ', focal))})
        topics = {'Raw measured cloud': self.topic_prefix + '/rgb',
                  'Graph geometry context': self.topic_prefix + '/height',
                  'Model semantic points': self.topic_prefix + '/components',
                  'Model unknown context': self.topic_prefix + '/context',
                  'Model object outlines': self.topic_prefix + '/boxes',
                  'Metric graph': self.topic_prefix + '/graph'}
        names = {'Raw measured cloud': 'Measured cloud',
                 'Graph geometry context': 'Measured context',
                 'Model semantic points': 'Geometric components',
                 'Model unknown context': 'Unassigned context',
                 'Model object outlines': 'Geometric boxes',
                 'Metric graph': 'Geometric graph'}
        for display in visual['Displays']:
            name = display['Name']
            if name in topics:
                key = 'Marker Topic' if display['Class'] == 'rviz/MarkerArray' else 'Topic'
                display[key] = topics[name]
                display['Name'] = names[name]
            display['Enabled'] = False
        config_path = runtime / 'imported_map.rviz'
        config_path.write_text(yaml.safe_dump(config))
        settings = rviz.Config()
        rviz.YamlConfigReader().readFile(settings, str(config_path))
        self.frame.load(settings)
        self.frame.menuBar().hide()
        self.frame.statusBar().hide()
        self.frame.setHideButtonVisibility(False)
        self.frame.setParent(parent, Qt.Widget)
        self.frame.show()
        for widget in self.frame.findChildren(QDockWidget) + self.frame.findChildren(QToolBar):
            widget.hide()
        self.manager = self.frame.getManager()
        group = self.manager.getRootDisplayGroup()
        self.displays = {group.getDisplayAt(i).getName(): group.getDisplayAt(i)
                         for i in range(group.numDisplays())}
        self.lib = ctypes.CDLL(str(ROOT / '.runtime/native/libwavego_labels.so'))
        self.lib.wavego_labels_create.argtypes = [ctypes.c_void_p]
        self.lib.wavego_labels_create.restype = ctypes.c_void_p
        for name in ['clear', 'destroy', 'update']:
            method = getattr(self.lib, 'wavego_labels_' + name)
            method.argtypes = [ctypes.c_void_p]
            method.restype = None
        self.lib.wavego_labels_add.argtypes = [ctypes.c_void_p, ctypes.c_char_p] + [ctypes.c_float] * 7
        self.lib.wavego_labels_add.restype = None
        self.layer = self.lib.wavego_labels_create(sip.unwrapinstance(self.frame))
        self.received.connect(self.update_markers)
        self.subscribers = [
            rospy.Subscriber(self.topic_prefix + '/box_labels', MarkerArray,
                             lambda m: self.received.emit('boxes', m), queue_size=1),
            rospy.Subscriber(self.topic_prefix + '/graph_labels', MarkerArray,
                             lambda m: self.received.emit('graph', m), queue_size=1)]
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.layout_labels)
        self.timer.start(60)

    def update_markers(self, kind, message):
        for item in message.markers:
            key = (item.ns, item.id)
            if item.action == Marker.DELETEALL:
                self.markers[kind].clear()
            elif item.action == Marker.DELETE:
                self.markers[kind].pop(key, None)
            elif item.type == Marker.TEXT_VIEW_FACING and item.header.frame_id == self.frame_id:
                self.markers[kind][key] = item
        self.draw_labels()

    def draw_labels(self):
        self.lib.wavego_labels_clear(self.layer)
        if self.mode == 'graph':
            markers = list(self.markers['graph'].values())
        elif self.mode == 'boxes':
            markers = list(self.markers['boxes'].values())
        elif self.mode == 'combined':
            markers = list(self.markers['boxes'].values()) + [
                item for item in self.markers['graph'].values()
                if item.text.startswith(('P0 ', 'H1 ', 'H2 ', 'H3 ', 'H4 ', 'H5 ',
                                         'S0 ', 'F0 '))]
        else:
            markers = []
        if self.label_prefixes is not None:
            markers = [item for item in markers
                       if item.text.startswith(self.label_prefixes)]
        visible = []
        if self.show_labels:
            for item in markers:
                point, color = item.pose.position, item.color
                self.lib.wavego_labels_add(self.layer, item.text.encode('utf-8'),
                                           point.x, point.y, point.z,
                                           color.r, color.g, color.b, max(.46, item.scale.z * 1.4))
                visible.append({'text': item.text, 'position': [point.x, point.y, point.z]})
        (self.runtime / 'imported_visible_labels.json').write_text(json.dumps(visible, ensure_ascii=False))
        self.layout_labels()

    def layout_labels(self):
        if self.layer:
            self.lib.wavego_labels_update(self.layer)
            self.manager.queueRender()

    def choose(self, mode, labels=True):
        self.mode, self.show_labels = mode, labels
        def scoped(name):
            return self.topic_prefix + '/' + self.topic_scope + name
        groups = {'before': ['Measured cloud'],
                  'rgb': ['Measured cloud'], 'height': ['Measured cloud'],
                  'zheight': ['Measured cloud'],
                  'colored': ['Geometric components', 'Unassigned context'],
                  'boxes': ['Geometric components', 'Unassigned context', 'Geometric boxes'],
                  'graph': ['Measured context', 'Geometric graph'],
                  'combined': ['Geometric components', 'Unassigned context',
                               'Geometric boxes', 'Geometric graph']}
        enabled = groups[mode]
        for name, display in self.displays.items():
            display.setEnabled(name in enabled)
        raw = self.displays['Measured cloud']
        topic = 'before' if mode == 'before' else 'height' if mode == 'height' else 'rgb'
        raw.subProp('Topic').setValue(scoped(topic))
        raw.subProp('Color Transformer').setValue('AxisColor' if mode == 'zheight' else 'RGB8')
        if mode == 'zheight':
            raw.subProp('Axis').setValue('Z')
            if self.height_bounds is not None:
                bounds = raw.subProp('Autocompute Value Bounds')
                bounds.setValue(False)
                bounds.subProp('Min Value').setValue(self.height_bounds[0])
                bounds.subProp('Max Value').setValue(self.height_bounds[1])
        context = self.displays['Measured context']
        graph_height = mode == 'graph' and self.height_bounds is not None
        context.subProp('Topic').setValue(scoped('rgb' if graph_height else 'height'))
        context.subProp('Color Transformer').setValue('AxisColor' if graph_height else 'RGB8')
        if graph_height:
            context.subProp('Axis').setValue('Z')
            bounds = context.subProp('Autocompute Value Bounds')
            bounds.setValue(False)
            bounds.subProp('Min Value').setValue(self.height_bounds[0])
            bounds.subProp('Max Value').setValue(self.height_bounds[1])
        context.subProp('Alpha').setValue(.20 if graph_height else .13)
        for display_name, topic_name in [('Geometric components', 'components'),
                                         ('Unassigned context', 'context'),
                                         ('Geometric boxes', 'boxes'),
                                         ('Geometric graph', 'graph')]:
            self.displays[display_name].subProp(
                'Marker Topic' if topic_name in ('boxes', 'graph') else 'Topic').setValue(
                    scoped(topic_name))
        self.draw_labels()
        (self.runtime / 'imported_display_state.json').write_text(json.dumps({
            'mode': mode, 'enabled': enabled, 'labels': labels,
            'frame': self.frame_id, 'topic_scope': self.topic_scope,
            'layout': 'single_window'}))

    def set_topic_scope(self, scope):
        next_scope = f'{scope}_' if scope else ''
        if next_scope != self.topic_scope:
            self.topic_scope = next_scope
            self.choose(self.mode, self.show_labels)

    def camera(self, preset):
        values = self.camera_values[preset]
        current = self.manager.getViewManager().getCurrent()
        for name, value in zip(['Distance', 'Pitch', 'Yaw'], values[:3]):
            current.subProp(name).setValue(value)
        for name, value in zip('XYZ', values[3]):
            current.subProp('Focal Point').subProp(name).setValue(value)
        self.manager.queueRender()

    def close(self):
        self.timer.stop()
        for subscriber in self.subscribers:
            subscriber.unregister()
        if self.layer:
            self.lib.wavego_labels_destroy(self.layer)
            self.layer = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--capture', action='store_true')
    args = parser.parse_args()
    args.data.mkdir(parents=True, exist_ok=True)
    graph = json.loads((args.data / 'geometry_graph.json').read_text())
    frame_id = graph['frame']
    runtime = ROOT / '.runtime' / frame_id
    runtime.mkdir(parents=True, exist_ok=True)
    roof_file = args.data / 'roof_report.json'
    roof = json.loads(roof_file.read_text()) if roof_file.exists() else None
    if roof:
        raw = read_pcd(args.data / 'display_no_roof.pcd')
        xlo, xhi = np.quantile(raw['x'], [.02, .98])
        ylo, yhi = np.quantile(raw['y'], [.02, .98])
        focal = [float((xlo + xhi) / 2), float((ylo + yhi) / 2), float(np.median(raw['z']))]
        distance = float(max(20., .82 * max(xhi - xlo, yhi - ylo)))
        camera_values = {'overview': [distance, .68, .62, focal],
                         'top': [distance * 1.4, 1.56, .62, focal],
                         'side': [distance * 1.4, .24, 1.4, focal]}
    else:
        camera_values = None
    rospy.init_node('wavego_imported_923_viewer', disable_signals=True)
    app = QApplication([])
    window = QWidget()
    if args.capture:
        window.setWindowFlag(Qt.WindowStaysOnTopHint, True)
    display_name = roof['name'].upper() if roof else '923'
    window.setWindowTitle(f'Wavego | {display_name} 点云与几何 Graph')
    window.setStyleSheet('QWidget{background:#fff;color:#26364b;} QLabel{font-size:15px;} '
                         'QPushButton{padding:10px 17px;border:1px solid #ccd7e4;'
                         'border-radius:5px;background:#f5f7fa;} '
                         'QPushButton:checked{background:#233c5c;color:white;}')
    outer = QVBoxLayout(window)
    title = QLabel(f'<b style="font-size:24px">{display_name} LAZ  ·  点云与几何 Graph</b><br>'
                   + (f'{roof["source_points"]:,} 个原始点 → {roof["retained_points"]:,} 个去顶点 · '
                      if roof else f'{graph["point_count"]:,} 个原始点 · ')
                   + f'{len([n for n in graph["nodes"] if n["kind"] == "geometric_component"])} 个几何簇 · 原始坐标')
    outer.addWidget(title)
    bar = QHBoxLayout()
    choices = ([('before', '00  去顶前')] if roof else []) + [
               ('rgb', '01  RGB 点云'), ('height', '02  高度点云'),
               ('boxes', '03  几何簇 + Box'), ('graph', '04  Graph'),
               ('combined', '05  融合叠加')]
    buttons = {}
    mode = ['combined']
    labels = QCheckBox('中文标签')
    labels.setChecked(True)

    def choose(name):
        mode[0] = name
        for key, button in buttons.items():
            button.setChecked(key == name)
        view.choose(name, labels.isChecked())
        subtitle.setText({'before': '去顶前的显示点云；与去顶后使用同一坐标和视角',
                          'rgb': '去顶后 RGB 点云；无语义分类' if roof else '原始 RGB；无语义分类',
                          'height': '同一坐标，按 Z 高度着色',
                          'boxes': '几何体素连通簇；Box 为 2–98% 分位范围',
                          'graph': '水平层和几何簇的高度关系；不是物体级语义 Graph',
                          'combined': '去顶点云坐标下的几何簇、Box 与 Graph 叠加'}[name])

    for key, caption in choices:
        button = QPushButton(caption)
        button.setCheckable(True)
        button.clicked.connect(lambda _, selected=key: choose(selected))
        buttons[key] = button
        bar.addWidget(button)
    bar.addStretch()
    outer.addLayout(bar)
    subtitle = QLabel()
    outer.addWidget(subtitle)
    holder = QWidget()
    canvas = QVBoxLayout(holder)
    canvas.setContentsMargins(0, 0, 0, 0)
    view = ImportedMap(holder, canvas, runtime, frame_id, camera_values)
    outer.addWidget(holder, 1)
    info = QLabel('说明：LAZ 的 classification 全为 0；H 为水平层，S 为建筑主体，G 为局部几何簇。'
                  '这些不是楼梯、行人或其它模型识别结果。')
    outer.addWidget(info)
    footer = QHBoxLayout()
    footer.addWidget(labels)
    labels.toggled.connect(lambda _: choose(mode[0]))
    footer.addStretch()
    for caption, preset in [('总览', 'overview'), ('俯视', 'top'), ('侧视', 'side')]:
        button = QPushButton(caption)
        button.clicked.connect(lambda _, p=preset: view.camera(p))
        footer.addWidget(button)
    outer.addLayout(footer)
    window.resize(1900, 1080)
    window.show()
    choose('combined')
    view.camera('overview')
    (runtime / 'viewer_window.json').write_text(json.dumps({'window_id': int(window.winId())}))

    if args.capture:
        shots = ([('00_before_roof', 'before', 'overview')] if roof else []) + [
                 ('01_no_roof_rgb' if roof else '01_rgb', 'rgb', 'overview'),
                 ('02_no_roof_height' if roof else '02_height', 'height', 'overview'),
                 ('03_geometry_boxes', 'boxes', 'overview'),
                 ('04_graph', 'graph', 'overview'),
                 ('05_combined', 'combined', 'overview')]
        if roof:
            shots += [('06_top_rgb', 'rgb', 'top'),
                      ('07_side_combined', 'combined', 'side')]

        def next_shot():
            if not shots:
                app.quit()
                return
            name, selected, preset = shots.pop(0)
            choose(selected)
            view.camera(preset)

            def save():
                window.raise_()
                window.activateWindow()
                app.primaryScreen().grabWindow(int(window.winId())).save(str(args.data / (name + '.png')))
                audit = json.loads((runtime / 'imported_display_state.json').read_text())
                audit['camera_preset'] = preset
                audit['visible_labels'] = json.loads((runtime / 'imported_visible_labels.json').read_text())
                (args.data / (name + '.json')).write_text(json.dumps(audit, ensure_ascii=False, indent=2))
                print('CAPTURE', name, flush=True)
                QTimer.singleShot(150, next_shot)

            QTimer.singleShot(2200, save)

        QTimer.singleShot(4500, next_shot)

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    app.exec_()
    view.close()


if __name__ == '__main__':
    main()
