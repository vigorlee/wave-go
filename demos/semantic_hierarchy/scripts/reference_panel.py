#!/usr/bin/env python3
"""Isolated RViz renderer, embedded by reference_view via its native window ID.

ROS1 RViz's Ogre material registry cannot initialize two managers in one process.
Each panel therefore owns a process, with cameras synchronized through ROS.
"""
import argparse,json,os,yaml
from pathlib import Path
import rospy
from std_msgs.msg import String,Bool
from PyQt5.QtWidgets import QApplication,QWidget,QDockWidget,QToolBar
from PyQt5.QtCore import Qt,QTimer,QObject,pyqtSignal,QEvent
from PyQt5.QtGui import QColor
from rviz import bindings as rviz
from academic_palette import HEIGHT_RANGE

ROOT=Path(__file__).resolve().parents[1]
RUNTIME=Path(os.environ.get('SCENE_SESSION_DIR',str(ROOT/'.runtime')))
RUNTIME.mkdir(parents=True,exist_ok=True)
NATIVE_RVIZ=os.environ.get('SCENE_NATIVE_RVIZ','1').lower() not in ('0','false','no')
ap=argparse.ArgumentParser();ap.add_argument('--side',choices=['left','right','fused','compare'],required=True)
ap.add_argument('--style',choices=['fastlio','paper','batlow'],default='paper')
ap.add_argument('--automated-capture',action='store_true');args=ap.parse_args()
rospy.init_node('reference_rviz_'+args.side,disable_signals=True)
app=QApplication([])
class CaptureInputFilter(QObject):
    def eventFilter(self,obj,event):
        return event.type() in [QEvent.MouseMove,QEvent.MouseButtonPress,QEvent.MouseButtonRelease,
                                QEvent.MouseButtonDblClick,QEvent.Wheel,QEvent.TouchBegin,QEvent.TouchUpdate]
if args.automated_capture:
    capture_filter=CaptureInputFilter();app.installEventFilter(capture_filter)
frame=rviz.VisualizationFrame()
# Keep RViz as a normal native top-level window.  On the current X11/Qt stack,
# reparenting an Ogre/OpenGL RViz frame into ``createWindowContainer`` makes
# the embedded GL surface black even though the ROS topics are healthy.  The
# outer demo still provides the controls and synchronizes this window through
# the camera topic; leaving the native surface intact keeps the point cloud
# visible and interactive.
frame.setSplashPath('');frame.initialize()
config_path=ROOT/('config/three_representations.rviz' if args.side=='compare' else 'config/reference_demo.rviz')
if args.side=='compare':
    values_config=yaml.safe_load(config_path.read_text())
    values_config['Visualization Manager']['Global Options']['Background Color']='0; 0; 0' if args.style=='fastlio' else '250; 251; 253'
    config_path=RUNTIME/'comparison_style.rviz';config_path.write_text(yaml.safe_dump(values_config))
config=rviz.Config();rviz.YamlConfigReader().readFile(config,str(config_path));frame.load(config)
frame.menuBar().hide();frame.statusBar().hide()
for w in frame.findChildren(QDockWidget)+frame.findChildren(QToolBar):w.hide()
manager=frame.getManager();group=manager.getRootDisplayGroup()
displays={group.getDisplayAt(i).getName():group.getDisplayAt(i) for i in range(group.numDisplays())}
for name,display in displays.items():
    selected={'left':['Instance cuboids','Instance labels','Recorded LIO trajectory'],
              'right':['Height cloud','Recorded LIO trajectory'],
              'fused':['Model semantic points','Model unknown context','Model object outlines','Model object labels','Recorded LIO trajectory'],
              'compare':['Graph geometry context','Metric graph','Metric graph labels']}[args.side]
    display.setEnabled(name in selected)
if NATIVE_RVIZ:
    # A native RViz top-level window keeps Ogre's OpenGL surface alive on X11.
    # Put it beside the WAVEGO controls when the desktop is wide enough.
    screen=app.primaryScreen().availableGeometry()
    # Keep the Ogre viewport below the desktop's very large logical size;
    # oversized native GL windows can fail to repaint on some drivers.
    panel_width=max(900,min(1600,screen.width()//2))
    panel_height=max(700,min(1100,screen.height()-120))
    native_geometry=(screen.x()+screen.width()-panel_width,screen.y()+60,panel_width,panel_height)
    frame.setGeometry(*native_geometry)
else:
    frame.resize(850,900)
frame.show()
if NATIVE_RVIZ:
    # Some window managers restore the previous RViz placement on show();
    # apply the side-by-side placement once more after the native window is
    # mapped so it cannot cover the WAVEGO controls.
    QTimer.singleShot(250,lambda: frame.setGeometry(*native_geometry))
(RUNTIME/f'reference_{args.side}_window.json').write_text(json.dumps({'pid':os.getpid(),'window_id':int(frame.winId())}))
camera_pub=rospy.Publisher('/reference_map/camera',String,queue_size=1,latch=False)

def values():
    c=manager.getViewManager().getCurrent()
    return [float(c.subProp('Distance').getValue()),float(c.subProp('Pitch').getValue()),
            float(c.subProp('Yaw').getValue()),[float(c.subProp('Focal Point').subProp(k).getValue()) for k in ['X','Y','Z']]]

previous=[values()]
def apply(raw):
    obj=json.loads(raw)
    if obj['origin']==args.side:return
    distance,pitch,yaw,focal=obj['values'];c=manager.getViewManager().getCurrent()
    for key,value in [('Distance',distance),('Pitch',pitch),('Yaw',yaw)]:c.subProp(key).setValue(value)
    for key,value in zip(['X','Y','Z'],focal):c.subProp('Focal Point').subProp(key).setValue(value)
    manager.queueRender();previous[0]=values()

class Bridge(QObject):
    camera=pyqtSignal(str)
    labels=pyqtSignal(bool)
    cutaway=pyqtSignal(bool)
    mode=pyqtSignal(str)
bridge=Bridge();bridge.camera.connect(apply)
if args.side!='compare':
    bridge.labels.connect(lambda yes:displays['Model object labels' if args.side=='fused' else 'Instance labels'].setEnabled(yes) if args.side!='right' else None)
def change_mode(raw):
    if args.side!='compare':return
    obj=json.loads(raw)
    groups={'graph':['Graph geometry context','Metric graph','Metric graph labels'],
            'semantic':['Model semantic points','Model unknown context','Model object outlines','Model object labels'],
            'raw':['Raw measured cloud'],
            'hovsg':['HOV-SG upstream floors'],
            'utonia':['Utonia feature cloud'],
            'combined':['Model semantic points','Model unknown context','Model object outlines','Metric graph','Metric graph labels']}
    enabled=groups[obj['mode']].copy()
    if not obj.get('labels',True):enabled=[n for n in enabled if 'labels' not in n]
    if obj.get('trajectory',False):enabled.append('Recorded LIO trajectory')
    for name,display in displays.items():display.setEnabled(name in enabled)
    classic=obj.get('style')=='fastlio'
    for cloud in ['Raw measured cloud','Graph geometry context']:
        displays[cloud].subProp('Topic').setValue('/reference_map/paper_height_points' if obj.get('style')=='paper' else '/reference_map/height_points')
        displays[cloud].subProp('Color Transformer').setValue('AxisColor' if classic else 'RGB8')
        if classic:
            displays[cloud].subProp('Axis').setValue('Z')
            bounds=displays[cloud].subProp('Autocompute Value Bounds');bounds.setValue(False)
            bounds.subProp('Min Value').setValue(HEIGHT_RANGE[0]);bounds.subProp('Max Value').setValue(HEIGHT_RANGE[1])
    if not obj.get('height',True):displays['Raw measured cloud'].subProp('Color Transformer').setValue('FlatColor')
    manager.queueRender()
    (RUNTIME/'comparison_display_state.json').write_text(json.dumps(dict(**obj,enabled=enabled,frame='lio_map')))
bridge.mode.connect(change_mode)
rospy.Subscriber('/reference_map/representation',String,lambda m:bridge.mode.emit(m.data),queue_size=1)
def cutaway(yes):
    if args.side=='right':
        displays['Height cloud'].setEnabled(not yes)
        displays['Height cutaway'].setEnabled(yes)
bridge.cutaway.connect(cutaway)
rospy.Subscriber('/reference_map/camera',String,lambda m:bridge.camera.emit(m.data),queue_size=1)
rospy.Subscriber('/reference_map/show_labels',Bool,lambda m:bridge.labels.emit(m.data),queue_size=1)
rospy.Subscriber('/reference_map/cutaway',Bool,lambda m:bridge.cutaway.emit(m.data),queue_size=1)
def sync():
    v=values()
    (RUNTIME/f'reference_{args.side}_camera.json').write_text(json.dumps(v))
    if v!=previous[0]:
        previous[0]=v;camera_pub.publish(String(data=json.dumps({'origin':args.side,'values':v})))
timer=QTimer();timer.timeout.connect(sync);timer.start(100)
app.exec_()
