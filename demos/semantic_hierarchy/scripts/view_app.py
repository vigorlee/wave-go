#!/usr/bin/env python3
"""Native Qt + actual RViz panel; saved-data demo, no browser required."""
import argparse
import json
from pathlib import Path
from PyQt5.QtWidgets import QApplication,QWidget,QDockWidget,QHBoxLayout,QVBoxLayout,QLabel,QPushButton
from PyQt5.QtCore import QTimer,Qt
from rviz import bindings as rviz

ROOT=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('--capture',action='store_true')
args=parser.parse_args()
report=json.loads((ROOT/'outputs/report.json').read_text())
app=QApplication([])
window=QWidget()
if args.capture: window.setWindowFlag(Qt.WindowStaysOnTopHint,True)
window.setWindowTitle('Wavego | 3D semantic mapping | OfficeWorld recorded-data demo')
window.setStyleSheet('QWidget {background:#0e1420;color:#e2eaf5;} QLabel {font-size:15px;} QPushButton {background:#25344b;border:1px solid #485e78;padding:10px;border-radius:6px;font-size:15px;} QPushButton:hover {background:#385678;}')
layout=QHBoxLayout(window)
side=QWidget()
side.setFixedWidth(285)
sidebar=QVBoxLayout(side)
title=QLabel('<span style="font-size:26px;font-weight:700">WAVEGO</span><br><span style="font-size:19px">立体语义地图</span>')
sidebar.addWidget(title)
subtitle=QLabel('OfficeWorld · Go2-W<br>已录制 LiDAR + IMU 地图')
sidebar.addWidget(subtitle)
sidebar.addSpacing(20)
for name,color in [('楼梯 · 视觉预测','#26aaff'),('坡道 · 视觉预测','#b473fa'),('圆柱 · 视觉预测','#ffab30'),('墙面 · 预测 / 几何候选','#89acc8'),('地面 · 预测 / 几何候选','#3ec89f'),('未标注测量点','#75869e'),('已记录 LIO 轨迹','#ffeb87')]:
    sidebar.addWidget(QLabel(f'<span style="color:{color}">●</span> {name}'))
sidebar.addSpacing(18)
counts=report['graph_node_counts']
stats=QLabel(f'测量点：{report["total_points"]:,}<br>楼层候选：{counts["floor"]}<br>观测区域：{counts["region"]}<br>语义 / 结构节点：{sum(counts.values())}<br>三维地图 + Spark-DSG')
sidebar.addWidget(stats)
sidebar.addStretch()
frame=rviz.VisualizationFrame()
frame.setSplashPath('')
frame.initialize()
config=rviz.Config()
rviz.YamlConfigReader().readFile(config,str(ROOT/'config/semantic_scene.rviz'))
frame.load(config)
frame.menuBar().hide()
frame.statusBar().hide()
for dock in frame.findChildren(QDockWidget): dock.hide()
manager=frame.getManager()
group=manager.getRootDisplayGroup()
displays={group.getDisplayAt(i).getName():group.getDisplayAt(i) for i in range(group.numDisplays())}

def view(mode):
    graph=mode=='graph'
    for title,display in displays.items():
        if title.startswith('Scene graph'): display.setEnabled(graph)
        elif title.startswith(('Floor and wall','Visual object','Recorded LIO','Unlabelled','Semantic points')): display.setEnabled(not graph)
    settings={'overview':(40.,.73,.82,(8.,4.,1.5)),
              'stairs':(25.,.25,1.57,(10.,3.,2.6)),
              'top':(34.,1.56,0.,(8.,3.,2.)),
              'graph':(39.,.08,4.71,(8.,-12.,6.))}
    distance,pitch,yaw,focal=settings[mode]
    current=manager.getViewManager().getCurrent()
    for key,value in [('Distance',distance),('Pitch',pitch),('Yaw',yaw)]: current.subProp(key).setValue(value)
    for key,value in zip(['X','Y','Z'],focal): current.subProp('Focal Point').subProp(key).setValue(value)
    manager.queueRender()

for name,mode in [('三维语义总览','overview'),('楼梯与跨层侧视','stairs'),('俯视语义地图','top'),('楼层—区域—物体图','graph')]:
    button=QPushButton(name)
    button.clicked.connect(lambda checked=False,m=mode:view(m))
    sidebar.addWidget(button)
note=QLabel('标签为候选，尚未测量精度。<br>此窗口显示已录制数据。<br>尚未运行完整 Hydra / S-Graphs<br>电梯与行人：无观测数据。')
note.setStyleSheet('font-size:12px;color:#9babbe;')
sidebar.addWidget(note)
layout.addWidget(side)
layout.addWidget(frame,1)
window.resize(1900,1120)
window.move(30,30)
window.show()
view('overview')

def capture():
    window.raise_()
    window.activateWindow()
    def save():
        p=ROOT/'outputs/screenshots/05_native_demo.png'
        app.primaryScreen().grabWindow(int(window.winId())).save(str(p),'PNG')
        print('CAPTURED',p,flush=True)
        app.quit()
    QTimer.singleShot(1500,save)

if args.capture: QTimer.singleShot(6000,capture)
app.exec_()
