#!/usr/bin/env python3
"""Two actual RViz panels with synchronized views and a recorded scan timeline."""
import argparse
import json
import time
import sys
import subprocess
from pathlib import Path
import rospy
from std_msgs.msg import Float64,String,Bool
from PyQt5.QtWidgets import (QApplication,QWidget,QDockWidget,QHBoxLayout,QVBoxLayout,QLabel,
                            QPushButton,QSlider,QComboBox,QCheckBox,QListWidget,QSplitter)
from PyQt5.QtCore import QTimer,Qt,pyqtSignal,QObject,QProcess
from PyQt5.QtGui import QWindow

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/reference_demo'
parser=argparse.ArgumentParser()
parser.add_argument('--capture',action='store_true')
parser.add_argument('--auto-play',action='store_true')
parser.add_argument('--record',action='store_true')
parser.add_argument('--layout',choices=['single','dual'],default='single')
args=parser.parse_args()
rospy.init_node('wavego_reference_view',anonymous=True,disable_signals=True)
seek=rospy.Publisher('/reference_map/seek',Float64,queue_size=1,latch=True)
camera_pub=rospy.Publisher('/reference_map/camera',String,queue_size=1,latch=True)
labels_pub=rospy.Publisher('/reference_map/show_labels',Bool,queue_size=1,latch=True)
cutaway_pub=rospy.Publisher('/reference_map/cutaway',Bool,queue_size=1,latch=True)
report=json.loads((OUT/'replay_report.json').read_text())
annotations=json.loads((OUT/'semantic_annotations.json').read_text())
if args.layout=='single':
    annotations=[n for n in annotations if n['kind'] in ['stairs','ramp','cylinder','person']]
duration=report['duration_seconds']
app=QApplication([])
window=QWidget()
if args.capture or args.record:window.setWindowFlag(Qt.WindowStaysOnTopHint,True)
window.setWindowTitle('Wavego | 单地图语义融合 · 原始扫描回放' if args.layout=='single' else 'Wavego | 跨楼层语义建图 · 原始扫描回放')
window.setStyleSheet('QWidget {background:#ffffff;color:#243047;} QLabel {font-size:14px;} QPushButton {background:#f3f5f8;border:1px solid #ccd5df;border-radius:4px;padding:8px;} QPushButton:hover {background:#e7effa;} QListWidget {border:1px solid #d7dee8;font-size:12px;} QComboBox {padding:5px;}')
outer=QVBoxLayout(window)
header=QHBoxLayout()
heading=QLabel('<b style="font-size:23px">WAVEGO · 单地图语义融合</b><br>语义点云 + 物体标签 + LIO 轨迹  /  同一 lio_map 坐标系' if args.layout=='single' else '<b style="font-size:23px">WAVEGO · 跨楼层语义建图</b><br>OfficeWorld 仿真传感器记录  /  4 cm 累积  /  蓝线为 LIO 轨迹')
header.addWidget(heading)
header.addStretch()
note=QLabel('历史扫描回放 · 保留原有模型语义类别<br>Grounding DINO + SAM2 投影结果；非 Hydra / S-Graphs 原生建图' if args.layout=='single' else '历史扫描回放 · Grounding DINO + SAM2 目标框投影<br>中文名称仅为显示翻译，非智元原楼宇地图')
note.setStyleSheet('color:#687489;font-size:12px;')
header.addWidget(note)
outer.addLayout(header)
body=QHBoxLayout()
sidebar=QWidget()
sidebar.setFixedWidth(235)
side=QVBoxLayout(sidebar)
side.setContentsMargins(0,4,8,0)
side.addWidget(QLabel('<b>语义类别</b><br>按已有模型类别着色' if args.layout=='single' else '<b>语义实例与结构</b><br>? 视觉预测　* 几何候选'))
if args.layout=='single':
    for title,color in [('楼梯','#3b5ba5'),('坡道','#9d6db1'),('圆柱障碍物','#d9954a'),
                        ('行人（本记录未检出）','#c65d7b'),('墙面','#68adb7'),('地面','#63a68f'),('未分类','#709ab8')]:
        side.addWidget(QLabel(f'<span style="color:{color};font-size:20px">●</span> {title}'))
    side.addWidget(QLabel('<b>物体簇 · 点击定位</b>'))
items=QListWidget()
names={'stairs':'楼梯','ramp':'坡道','cylinder':'圆柱障碍物','wall':'墙面','floor':'楼层地面','region':'观测区域'}
for n in annotations:items.addItem(f'{n["id"]}  {names[n["kind"]]}')
side.addWidget(items,1)
details=QLabel('原模型类别不被几何规则重写。<br>物体框为语义点云聚类显示。' if args.layout=='single' else '点击实例可查看来源与定位。<br>区域名称尚未确认，保留为观测区域。')
details.setWordWrap(True)
details.setMinimumHeight(90)
side.addWidget(details)
status=QLabel('正在接收建图数据…')
status.setWordWrap(True)
side.addWidget(status)
body.addWidget(sidebar)
split=QSplitter(Qt.Horizontal)
panels=[]
workers=[]
layouts=[(True,'三维语义点云 · 类别颜色 / 实例标签 / 运动轨迹')]
if args.layout=='dual':layouts=[(True,'语义实例地图 · 彩色包围框'),(False,'LiDAR 点云地图 · 高度着色')]
for semantic,title in layouts:
    holder=QWidget()
    lay=QVBoxLayout(holder)
    lay.setContentsMargins(0,0,0,0)
    label=QLabel(title)
    label.setStyleSheet('font-size:17px;font-weight:600;padding:8px;background:#f6f8fb;')
    lay.addWidget(label)
    side_name='fused' if args.layout=='single' else ('left' if semantic else 'right')
    state_file=ROOT/f'.runtime/reference_{side_name}_window.json'
    state_file.unlink(missing_ok=True)
    process=QProcess(window)
    process.setProcessChannelMode(QProcess.MergedChannels)
    process.setStandardOutputFile(str(ROOT/f'.runtime/reference_{side_name}.log'))
    process.start(sys.executable,[str(ROOT/'scripts/reference_panel.py'),'--side',side_name])
    workers.append(process)
    split.addWidget(holder)
    panels.append(dict(layout=lay,state_file=state_file,process=process,embedded=False))
body.addWidget(split,1)
outer.addLayout(body,1)
scale=QLabel('类别颜色固定，未知点为浅蓝；几何楼层 / 房间候选未混入模型语义；语义精度尚未测量' if args.layout=='single' else 'Batlow 高度色标 −1.5 m ～ 8.2 m　｜　两个视图使用同一坐标与相机；未识别物体不会强行命名')
scale.setAlignment(Qt.AlignCenter)
scale.setStyleSheet('color:#657188;font-size:12px;')
outer.addWidget(scale)
controls=QHBoxLayout()
play=QPushButton('▶ 回放建图')
controls.addWidget(play)
slider=QSlider(Qt.Horizontal)
slider.setRange(0,int(duration*10))
slider.setValue(int(duration*10))
controls.addWidget(slider,1)
clock=QLabel()
clock.setMinimumWidth(140)
controls.addWidget(clock)
speed=QComboBox()
speed.addItems(['1×','2×','4×','8×'])
speed.setCurrentIndex(2)
controls.addWidget(speed)
for title,preset in [('总览','overview'),('楼梯侧视','side'),('俯视','top')]:
    button=QPushButton(title)
    controls.addWidget(button)
    button.clicked.connect(lambda checked=False,p=preset:camera(p))
showlabels=QCheckBox('全部标签')
showlabels.setChecked(True)
showlabels.toggled.connect(lambda yes:labels_pub.publish(Bool(data=yes)))
controls.addWidget(showlabels)
cutaway=QCheckBox('点云剖切')
cutaway.setChecked(True)
cutaway.toggled.connect(lambda yes:cutaway_pub.publish(Bool(data=yes)))
controls.addWidget(cutaway)
cutaway.setVisible(args.layout=='dual')
outer.addLayout(controls)

def set_camera(values):
    camera_pub.publish(String(data=json.dumps({'origin':'controls','values':values})))

def camera(preset):
    values={'overview':(35.,.67,.65,(9.,3.,2.)),
            'side':(30.,.20,1.57,(9.,3.,2.1)),
            'top':(38.,1.56,0.,(9.,3.,2.))}[preset]
    set_camera(values)

def embed_panels():
    for panel in panels:
        if panel['embedded'] or not panel['state_file'].exists():continue
        state=json.loads(panel['state_file'].read_text())
        if state['pid']!=panel['process'].processId():continue
        native=QWindow.fromWinId(state['window_id'])
        container=QWidget.createWindowContainer(native,window)
        panel['layout'].addWidget(container,1)
        panel.update(embedded=True,native=native,container=container)
    if all(p['embedded'] for p in panels):embed.stop()

def select(row):
    if row<0:return
    n=annotations[row]
    source='视觉预测' if n['kind'] in ['stairs','ramp','cylinder'] else '几何候选'
    details.setText(f'<b>{n["id"]} · {names[n["kind"]]}</b><br>来源：{source}<br>关联测量点：{n["associated_dense_points"]:,}<br>层级：{n.get("floor",n["id"])}<br>类别与边界尚未人工确认')
    set_camera((16.,.55,.65,tuple(n['position'])))
items.itemClicked.connect(lambda item:select(items.row(item)))

playing=[False]
lasttick=[time.monotonic()]
def update_time(v):
    t=duration if v==slider.maximum() else v/10
    clock.setText(f'{t:05.1f} / {duration:05.1f} s')
    seek.publish(Float64(data=t))
slider.valueChanged.connect(update_time)

def toggle():
    if not playing[0] and slider.value()>=slider.maximum():slider.setValue(0)
    playing[0]=not playing[0]
    lasttick[0]=time.monotonic()
    play.setText('Ⅱ 暂停' if playing[0] else '▶ 回放建图')
play.clicked.connect(toggle)

def tick():
    now=time.monotonic()
    if playing[0]:
        step=(now-lasttick[0])*(2**speed.currentIndex())*10
        slider.setValue(min(slider.maximum(),slider.value()+max(1,round(step))))
        if slider.value()==slider.maximum():playing[0]=False;play.setText('▶ 回放建图')
    lasttick[0]=now

class Bridge(QObject):
    status=pyqtSignal(str)
bridge=Bridge()
def update_status(raw):
    message=json.loads(raw)
    visible=set(message['instances']) & {n['id'] for n in annotations}
    status.setText(f'已累积点：{message["points"]:,}<br>当前可见标注：{len(visible)} / {len(annotations)}')
bridge.status.connect(update_status)
rospy.Subscriber('/reference_map/status',String,lambda m:bridge.status.emit(m.data),queue_size=1)
timer=QTimer();timer.timeout.connect(tick);timer.start(250)
embed=QTimer();embed.timeout.connect(embed_panels);embed.start(150)
window.resize(2040,1140)
window.move(15,20)
window.show()
(OUT/'viewer_window.json').write_text(json.dumps({'window_id':int(window.winId()),'width':window.width(),'height':window.height()}))
camera('overview')
update_time(slider.value())
QTimer.singleShot(2000,lambda:update_time(slider.value()))
cutaway_pub.publish(Bool(data=True))
if args.auto_play:QTimer.singleShot(5000,toggle)

if args.capture:
    shots=[('01_single_semantic.png','overview'),('02_single_stairs.png','side'),('03_single_top.png','top')] if args.layout=='single' else [('01_dual_map.png','overview'),('02_stairs_dual.png','side'),('03_top_dual.png','top')]
    def capture_next():
        if not shots:app.quit();return
        name,view=shots.pop(0)
        camera(view)
        window.raise_();window.activateWindow()
        def save():
            app.primaryScreen().grabWindow(int(window.winId())).save(str(OUT/name),'PNG')
            poses={side:json.loads((ROOT/f'.runtime/reference_{side}_camera.json').read_text()) for side in (['fused'] if args.layout=='single' else ['left','right'])}
            (OUT/(name+'.camera.json')).write_text(json.dumps(poses,indent=2))
            print('CAPTURED',name,flush=True)
            QTimer.singleShot(100,capture_next)
        QTimer.singleShot(3500,save)
    QTimer.singleShot(5000,capture_next)
if args.record:
    recorder=[None]
    def finish_record():
        recorder[0].stdin.write(b'q');recorder[0].stdin.flush()
        print('VIDEO_RECORDED',OUT/'semantic_mapping_demo.mp4',flush=True)
        QTimer.singleShot(2500,app.quit)
    def start_record():
        camera('overview');slider.setValue(0);window.raise_();window.activateWindow()
        command=['/usr/bin/ffmpeg','-y','-loglevel','warning','-f','x11grab','-framerate','12',
                 '-draw_mouse','0','-window_id',str(int(window.winId())),'-i',':0.0',
                 '-c:v','libx264','-preset','veryfast','-crf','20','-pix_fmt','yuv420p',
                 '-movflags','+faststart',str(OUT/'semantic_mapping_demo.mp4')]
        log=(ROOT/'.runtime/reference_video.log').open('wb')
        recorder[0]=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=log,stderr=log)
        QTimer.singleShot(1500,toggle)
        QTimer.singleShot(int((duration/4+6)*1000),finish_record)
    QTimer.singleShot(6000,start_record)
app.exec_()
for process in workers:process.terminate();process.waitForFinished(3000)
