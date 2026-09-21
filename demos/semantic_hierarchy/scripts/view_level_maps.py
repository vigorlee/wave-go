#!/usr/bin/env python3
"""Native RViz viewer for preserved and newly measured maps; display only."""
import argparse,json,os,sys,yaml
from pathlib import Path
import numpy as np
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2,PointField
from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QHBoxLayout,QLabel,QPushButton,QDockWidget,QToolBar
from PyQt5.QtCore import QTimer,QObject,QEvent,Qt
from rviz import bindings as rviz
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--new',type=Path,required=True);p.add_argument('--capture',type=Path);p.add_argument('--measured',type=Path);a=p.parse_args()
old=np.load(ROOT/'outputs/reference_demo/mapping_replay.npz')['points'];new=np.load(a.new)['xyzi'][:,:3]
datasets={'old':old,'new':new}
if a.measured:datasets['measured']=np.load(a.measured)['xyzi'][:,:3]
rospy.init_node('level_map_view',anonymous=True,disable_signals=True)
pub=rospy.Publisher('/level_comparison/points',PointCloud2,queue_size=1,latch=True)
app=QApplication([]);w=QWidget();w.setWindowTitle('WaveGo | 旧地图保留 · 新采集两层地图');w.resize(1500,950)
w.setStyleSheet('QWidget{background:#f7f8fa;color:#233447;font-size:15px} QPushButton{padding:10px 18px;background:white;border:1px solid #bdc9d5;border-radius:4px}')

class CaptureFilter(QObject):
 def eventFilter(self,obj,event):return event.type() in (QEvent.MouseMove,QEvent.MouseButtonPress,QEvent.MouseButtonRelease,QEvent.MouseButtonDblClick,QEvent.Wheel,QEvent.KeyPress,QEvent.KeyRelease)
if a.capture:
 capture_filter=CaptureFilter();app.installEventFilter(capture_filter);w.setWindowFlag(Qt.WindowStaysOnTopHint,True)
layout=QVBoxLayout(w);title=QLabel('');layout.addWidget(title);buttons=QHBoxLayout();layout.addLayout(buttons)
frame=rviz.VisualizationFrame();frame.setSplashPath('');frame.initialize()
conf=yaml.safe_load((ROOT/'config/three_representations.rviz').read_text());m=conf['Visualization Manager'];m['Global Options'].update({'Fixed Frame':'map','Background Color':'0; 0; 0'});m['Displays']=[{'Class':'rviz/PointCloud2','Name':'Measured map','Enabled':True,'Topic':'/level_comparison/points','Queue Size':1,'Style':'Points','Size (Pixels)':1,'Color Transformer':'AxisColor','Position Transformer':'XYZ','Axis':'Z','Autocompute Value Bounds':{'Value':False,'Min Value':-1.5,'Max Value':8.2},'Alpha':1.0,'Decay Time':0}];m['Views']['Current']['Target Frame']='map'
cp=ROOT/'.runtime/level_comparison.rviz';cp.write_text(yaml.safe_dump(conf));c=rviz.Config();rviz.YamlConfigReader().readFile(c,str(cp));frame.load(c)
frame.getManager().getRootDisplayGroup().getDisplayAt(0).setEnabled(True)
frame.menuBar().hide();frame.statusBar().hide()
for child in frame.findChildren(QDockWidget)+frame.findChildren(QToolBar):child.hide()
layout.addWidget(frame,1);state=['new'];view=['overview'];status=QLabel('鼠标拖动旋转 · 滚轮缩放 | 点云按真实 Z 高度着色 | 未压平楼层或移动点坐标');layout.addWidget(status)

def cloud(which):
 state[0]=which;x=datasets[which];total=len(x);x=x[((x>[-3,-8,-1.5])&(x<[21,13,9.2])).all(1)];x=x[(x[:,0]>10.8)&(x[:,0]<14.8)&(x[:,1]>-2.5)&(x[:,1]<7.5)] if view[0]=='stairs' else x;x=np.ascontiguousarray(x,dtype='<f4')
 msg=PointCloud2(header=Header(stamp=rospy.Time.now(),frame_id='map'),height=1,width=len(x),fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])],is_bigendian=False,point_step=12,row_step=len(x)*12,data=x.tobytes(),is_dense=True);pub.publish(msg)
 names={'new':'新采集 / 楼层结构校正 · 已知层高 5.51 m','old':'旧地图 / 原始结果保留','measured':'新采集 / 纯 LIO 估计（未做楼层校正）'};title.setText(names[which]+'　|　'+format(len(x),',')+' 点　|　4 cm 测量点积累')
 status.setText('鼠标旋转 / 滚轮缩放 | 显示场景范围；完整 PCD 保留 '+format(total,',')+' 点 | 校正版使用已知楼层先验，非独立 SLAM 精度结果')

def camera(name):
 view[0]=name;settings={'overview':(32,.58,.65,[8,2,2.3]),'side':(30,0,1.570796,[8,2,2.8]),'top':(35,1.5707,0,[8,2,2]),'stairs':(20,.15,0,[12.8,2.5,3.5])};d,pi,y,f=settings[name];v=frame.getManager().getViewManager().getCurrent()
 for k,z in [('Distance',d),('Pitch',pi),('Yaw',y)]:v.subProp(k).setValue(z)
 for k,z in zip(['X','Y','Z'],f):v.subProp('Focal Point').subProp(k).setValue(z)
 frame.getManager().queueRender();cloud(state[0])
for label,key in [('旧地图（保留）','old'),*([('新采集 · 纯LIO','measured')] if a.measured else []),('新采集 · 楼层校正','new')]:
 b=QPushButton(label);b.clicked.connect(lambda checked,k=key:cloud(k));buttons.addWidget(b)
for label,key in [('立体视图','overview'),('正侧视图','side'),('俯视图','top'),('楼梯剖面','stairs')]:
 b=QPushButton(label);b.clicked.connect(lambda checked,k=key:camera(k));buttons.addWidget(b)
buttons.addStretch();w.show();QTimer.singleShot(1800,lambda:(cloud('new'),camera('overview')))
if a.capture:
 a.capture.mkdir(parents=True,exist_ok=True);jobs=[(k,v) for v in ['overview','side','top','stairs'] for k in datasets];counter=[0]
 def step():
  i=counter[0]
  if i:
   k,v=jobs[i-1];shot=app.primaryScreen().grabWindow(int(w.winId()));shot.copy(frame.geometry()).save(str(a.capture/f'{k}_{v}.png'));shot.save(str(a.capture/f'{k}_{v}_window.png'))
   vc=frame.getManager().getViewManager().getCurrent();meta=dict(dataset=k,view=v,distance=float(vc.subProp('Distance').getValue()),pitch=float(vc.subProp('Pitch').getValue()),yaw=float(vc.subProp('Yaw').getValue()),focal=[float(vc.subProp('Focal Point').subProp(z).getValue()) for z in ['X','Y','Z']],source=str(a.new if k=='new' else (a.measured if k=='measured' else ROOT/'outputs/reference_demo/mapping_replay.npz')));(a.capture/f'{k}_{v}.json').write_text(json.dumps(meta,indent=2))
  if i==len(jobs):app.quit();return
  k,v=jobs[i];cloud(k);camera(v);counter[0]+=1;QTimer.singleShot(2500,step)
 QTimer.singleShot(4000,step)
try:app.exec_()
finally:rospy.signal_shutdown('viewer closed')
