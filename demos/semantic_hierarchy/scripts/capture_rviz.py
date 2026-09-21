#!/usr/bin/env python3
"""Capture actual RViz renders; no generated or retouched scene content."""
import datetime
import json
from pathlib import Path
from PyQt5.QtWidgets import QApplication, QWidget, QDockWidget
from PyQt5.QtCore import QTimer, Qt
from rviz import bindings as rviz

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/screenshots'
OUT.mkdir(exist_ok=True)
app=QApplication([])
frame=rviz.VisualizationFrame()
frame.setWindowFlag(Qt.WindowStaysOnTopHint,True)
frame.setSplashPath('')
frame.initialize()
frame.setWindowTitle('Wavego | 3D semantic scene | recorded OfficeWorld data')
config=rviz.Config()
rviz.YamlConfigReader().readFile(config,str(ROOT/'config/semantic_scene.rviz'))
frame.load(config)
for dock in frame.findChildren(QDockWidget): dock.hide()
panel=next(w for w in frame.findChildren(QWidget) if w.metaObject().className()=='rviz::RenderPanel')
panel.setFixedSize(1920,1080)
frame.resize(2000,1180)
frame.move(30,30)
frame.show()
manager=frame.getManager()
group=manager.getRootDisplayGroup()
displays={group.getDisplayAt(i).getName():group.getDisplayAt(i) for i in range(group.numDisplays())}
views=[
 ('01_semantic_overview',40.,.73,.82,(8.,4.,1.5),False),
 ('02_stairs_side',25.,.25,1.57,(10.,3.,2.6),False),
 ('03_semantic_top',33.,1.56,0.,(8.,2.,2.0),False),
 ('04_hierarchy',37.,.08,4.71,(8.,-12.,6.),True),
]
records=[]
index=0

def configure():
 global index
 if index==len(views):
  (OUT/'manifest.json').write_text(json.dumps(records,indent=2))
  print('CAPTURE_COMPLETE',flush=True)
  app.quit()
  return
 name,dist,pitch,yaw,focal,graph=views[index]
 for title,display in displays.items():
  if title.startswith('Scene graph'): display.setEnabled(graph)
  elif title.startswith('Floor and wall'): display.setEnabled(not graph)
  elif title.startswith('Visual object'): display.setEnabled(not graph)
  elif title.startswith('Recorded LIO'): display.setEnabled(not graph)
  elif title.startswith('Unlabelled'): display.setEnabled(not graph)
  elif title.startswith('Semantic points'): display.setEnabled(not graph)
 controller=manager.getViewManager().getCurrent()
 for n,value in [('Distance',dist),('Pitch',pitch),('Yaw',yaw)]: controller.subProp(n).setValue(value)
 for n,value in zip(['X','Y','Z'],focal): controller.subProp('Focal Point').subProp(n).setValue(value)
 manager.queueRender()
 frame.raise_()
 frame.activateWindow()
 QTimer.singleShot(4000,capture)

def capture():
 global index
 name=views[index][0]
 image=app.primaryScreen().grabWindow(int(panel.winId()))
 assert image.width()==1920 and image.height()==1080
 assert image.save(str(OUT/(name+'.png')),'PNG')
 records.append(dict(file=name+'.png',captured_at=datetime.datetime.now().astimezone().isoformat(),
                     source='actual RViz rendering of saved recorded map',camera=views[index][1:]))
 print('CAPTURED',name,flush=True)
 index+=1
 QTimer.singleShot(100,configure)

QTimer.singleShot(4500,configure)
app.exec_()
