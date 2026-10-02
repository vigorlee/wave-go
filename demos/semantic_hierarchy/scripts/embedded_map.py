"""One RViz manager constructed inside the demo, plus Unicode marker labels.

The render window is created with its final Qt parent. No foreign-window
reparenting or second top-level RViz process is involved.
"""
import ctypes
import json
from pathlib import Path

import rospy
import yaml
from PyQt5 import sip
from PyQt5.QtCore import Qt, QObject, QTimer, pyqtSignal
from PyQt5.QtWidgets import QDockWidget, QToolBar
from rviz import bindings as rviz
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from academic_palette import HEIGHT_RANGE

ROOT = Path(__file__).resolve().parents[1]


class EmbeddedMap(QObject):
    camera_received = pyqtSignal(str)
    marker_received = pyqtSignal(str, object)

    def __init__(self, parent, layout, runtime, style):
        super().__init__(parent)
        self.runtime = runtime
        self.style = style
        self.mode = 'combined'
        self.show_labels = True
        self.markers = {'objects': {}, 'graph': {}}
        self.frame = rviz.VisualizationFrame(parent)
        self.frame.setWindowFlags(Qt.Widget)
        layout.addWidget(self.frame, 1)
        self.frame.setSplashPath('')
        self.frame.initialize()
        config_data = yaml.safe_load((ROOT/'config/three_representations.rviz').read_text())
        config_data['Visualization Manager']['Global Options']['Background Color'] = (
            '0; 0; 0' if style == 'fastlio' else '250; 251; 253')
        config_path = runtime/'embedded_style.rviz'
        config_path.write_text(yaml.safe_dump(config_data))
        config = rviz.Config()
        rviz.YamlConfigReader().readFile(config, str(config_path))
        self.frame.load(config)
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
        for display in self.displays.values():
            display.setEnabled(False)
        # Qt rasterizes the exact ROS text into an Ogre billboard texture. This
        # avoids RViz's Latin-only default font while retaining metric anchors.
        self.lib = ctypes.CDLL(str(ROOT/'.runtime/native/libwavego_labels.so'))
        self.lib.wavego_labels_create.argtypes = [ctypes.c_void_p]
        self.lib.wavego_labels_create.restype = ctypes.c_void_p
        self.lib.wavego_labels_clear.argtypes = [ctypes.c_void_p]
        self.lib.wavego_labels_destroy.argtypes = [ctypes.c_void_p]
        self.lib.wavego_labels_update.argtypes = [ctypes.c_void_p]
        self.lib.wavego_labels_add.argtypes = [ctypes.c_void_p, ctypes.c_char_p] + [ctypes.c_float]*7
        for name in ['clear', 'destroy', 'update', 'add']:
            getattr(self.lib, 'wavego_labels_' + name).restype = None
        self.layer = self.lib.wavego_labels_create(sip.unwrapinstance(self.frame))
        self.marker_received.connect(self.update_markers)
        self.camera_received.connect(self.apply_camera)
        self.subscribers = [
            rospy.Subscriber('/reference_map/model_labels', MarkerArray,
                             lambda m: self.marker_received.emit('objects', m), queue_size=1),
            rospy.Subscriber('/reference_map/metric_graph_labels', MarkerArray,
                             lambda m: self.marker_received.emit('graph', m), queue_size=1),
            rospy.Subscriber('/reference_map/camera', String,
                             lambda m: self.camera_received.emit(m.data), queue_size=1)]
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.save_camera)
        self.timer.start(150)
        self.label_timer = QTimer(self)
        self.label_timer.timeout.connect(self.update_layout)
        self.label_timer.start(50)

    def update_layout(self):
        if self.layer:
            self.lib.wavego_labels_update(self.layer)
            self.manager.queueRender()

    def camera_values(self):
        camera = self.manager.getViewManager().getCurrent()
        return [float(camera.subProp(k).getValue()) for k in ['Distance', 'Pitch', 'Yaw']] + [
            [float(camera.subProp('Focal Point').subProp(k).getValue()) for k in 'XYZ']]

    def save_camera(self):
        (self.runtime/'reference_compare_camera.json').write_text(json.dumps(self.camera_values()))

    def apply_camera(self, raw):
        distance, pitch, yaw, focal = json.loads(raw)['values']
        camera = self.manager.getViewManager().getCurrent()
        for name, value in [('Distance', distance), ('Pitch', pitch), ('Yaw', yaw)]:
            camera.subProp(name).setValue(value)
        for name, value in zip('XYZ', focal):
            camera.subProp('Focal Point').subProp(name).setValue(value)
        self.manager.queueRender()

    def update_markers(self, kind, message):
        for marker in message.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETEALL:
                self.markers[kind].clear()
            elif marker.action == Marker.DELETE:
                self.markers[kind].pop(key, None)
            elif marker.type == Marker.TEXT_VIEW_FACING and marker.header.frame_id == 'lio_map':
                self.markers[kind][key] = marker
        self.draw_labels()

    def draw_labels(self):
        if not self.layer:
            return
        self.lib.wavego_labels_clear(self.layer)
        # Combined mode keeps object labels, with graph labels available in the
        # graph tab. This avoids drawing two names over the same object centre.
        kind = ('objects' if self.mode in ('semantic', 'combined') else
                'graph' if self.mode == 'graph' else None)
        drawn = []
        if self.show_labels and kind:
            for m in self.markers[kind].values():
                p, c = m.pose.position, m.color
                self.lib.wavego_labels_add(self.layer, m.text.encode('utf-8'), p.x, p.y, p.z,
                                           c.r, c.g, c.b, max(.48, m.scale.z*1.4))
                drawn.append({'text': m.text, 'position': [p.x, p.y, p.z],
                              'source': m.ns, 'frame': m.header.frame_id})
        (self.runtime/'visible_labels.json').write_text(json.dumps(drawn, ensure_ascii=False, indent=2))
        self.update_layout()
        self.manager.queueRender()

    def choose(self, options):
        self.mode, self.show_labels = options['mode'], options.get('labels', True)
        groups = {
            'graph': ['Graph geometry context', 'Metric graph'],
            'semantic': ['Model semantic points', 'Model unknown context', 'Model object outlines'],
            'raw': ['Raw measured cloud'], 'hovsg': ['HOV-SG upstream floors'],
            'utonia': ['Utonia feature cloud'],
            'combined': ['Model semantic points', 'Model unknown context', 'Model object outlines', 'Metric graph']}
        enabled = groups[self.mode].copy()
        if options.get('trajectory', False):
            enabled.append('Recorded LIO trajectory')
        for name, display in self.displays.items():
            display.setEnabled(name in enabled)
        for name in ['Raw measured cloud', 'Graph geometry context']:
            display = self.displays[name]
            display.subProp('Topic').setValue('/reference_map/paper_height_points' if self.style == 'paper'
                                              else '/reference_map/height_points')
            display.subProp('Color Transformer').setValue('AxisColor' if self.style == 'fastlio' else 'RGB8')
            if self.style == 'fastlio':
                display.subProp('Axis').setValue('Z')
                bounds = display.subProp('Autocompute Value Bounds')
                bounds.setValue(False)
                bounds.subProp('Min Value').setValue(HEIGHT_RANGE[0])
                bounds.subProp('Max Value').setValue(HEIGHT_RANGE[1])
        if not options.get('height', True):
            self.displays['Raw measured cloud'].subProp('Color Transformer').setValue('FlatColor')
        self.draw_labels()
        (self.runtime/'comparison_display_state.json').write_text(json.dumps(
            dict(options, enabled=enabled, frame='lio_map', layout='integrated',
                 label_renderer='Qt Unicode textures anchored to ROS markers')))

    def close(self):
        self.timer.stop()
        self.label_timer.stop()
        for subscriber in self.subscribers:
            subscriber.unregister()
        if self.layer:
            self.lib.wavego_labels_destroy(self.layer)
            self.layer = None
