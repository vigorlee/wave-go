#!/usr/bin/env python3
"""Three synchronized representations in one native RViz canvas, with replay."""
import argparse
import json
import os
import subprocess
import sys
import time
import signal
from pathlib import Path

import rospy
from std_msgs.msg import String, Float64
from PyQt5.QtCore import Qt, QTimer, QProcess, QObject, pyqtSignal, QEvent
from PyQt5.QtGui import QWindow,QPixmap
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QPushButton, QCheckBox,
                             QHBoxLayout, QVBoxLayout, QSlider, QComboBox, QButtonGroup,QDialog)
from academic_palette import CLASS_COLORS,HEIGHT_RANGE,HEIGHT_LUT,PAPER_LUT,hex_color
from scene_profile import DATA_ROOT,PROFILE,CHINESE

ROOT=Path(__file__).resolve().parents[1]
RUNTIME=Path(os.environ.get('SCENE_SESSION_DIR',str(ROOT/'.runtime')))
RUNTIME.mkdir(parents=True,exist_ok=True)
NATIVE_RVIZ=os.environ.get('SCENE_NATIVE_RVIZ','0').lower() not in ('0','false','no')
ap=argparse.ArgumentParser()
ap.add_argument('--capture',action='store_true')
ap.add_argument('--record',action='store_true')
ap.add_argument('--capture-hovsg',action='store_true')
ap.add_argument('--style',choices=['fastlio','paper','batlow'],default='fastlio')
ap.add_argument('--show-features',action='store_true')
ap.add_argument('--capture-compare',action='store_true')
ap.add_argument('--mode',choices=['graph','semantic','raw','combined','hovsg','utonia'],default='raw')
args=ap.parse_args()
OUT=Path(os.environ.get('SCENE_CAPTURE_DIR', str(DATA_ROOT/'outputs/fastlio_comparison'/(args.style+('_cn' if CHINESE else '')))))
OUT.mkdir(exist_ok=True,parents=True)
auto_capture=args.capture or args.record or args.capture_hovsg or args.capture_compare
rospy.init_node('wavego_three_view',anonymous=True,disable_signals=True)
mode_pub=rospy.Publisher('/reference_map/representation',String,queue_size=1,latch=True)
camera_pub=rospy.Publisher('/reference_map/camera',String,queue_size=1,latch=True)
seek_pub=rospy.Publisher('/reference_map/seek',Float64,queue_size=1,latch=True)
report=json.loads((DATA_ROOT/'outputs/reference_demo/replay_report.json').read_text())
scene=json.loads((DATA_ROOT/'outputs/scene_graph.json').read_text())
counts={k:sum(n['kind']==k for n in scene['nodes']) for k in ['building','floor','region','stairs','ramp','cylinder','person','wall']}
object_count=sum(counts[k] for k in ['stairs','ramp','cylinder','person'])
profile_title=PROFILE.get('title','一张地图 · 三种表示')
source_caption='来源：新 RGB-D 模型标注 + 校正后的 LIO 点云 + Spark-DSG 结构候选；包含已知层高先验' if PROFILE else '来源：LIO 测量几何 + 已保存 DINO / SAM2 语义 + Spark-DSG 结构候选；算法运行范围见对比说明'
def result_json(relative):
    p=DATA_ROOT/relative
    return json.loads(p.read_text()) if p.exists() else {}
floor_result=result_json('outputs/three_representations/hovsg_floor_probe/result.json')
features=result_json('outputs/academic_upgrade/utonia_report.json')
duration=report['duration_seconds']
app=QApplication([])
class AutomatedInputFilter(QObject):
    def eventFilter(self,obj,event):
        return event.type() in [QEvent.MouseMove,QEvent.MouseButtonPress,QEvent.MouseButtonRelease,
                                QEvent.MouseButtonDblClick,QEvent.Wheel,QEvent.KeyPress,QEvent.KeyRelease,
                                QEvent.TouchBegin,QEvent.TouchUpdate]
if auto_capture:
    input_filter=AutomatedInputFilter();app.installEventFilter(input_filter)
w=QWidget()
if auto_capture:w.setWindowFlag(Qt.WindowStaysOnTopHint,True)
w.setWindowTitle('Wavego | '+profile_title+' | 场景图 · 语义框 · 纯点云' if CHINESE else 'Wavego | '+profile_title+' | Graph · 语义 Box · 纯点云')
w.setStyleSheet('QWidget{background:#ffffff;color:#26364b;} QLabel{font-size:14px;} QPushButton{padding:10px 15px;border:1px solid #ccd7e4;border-radius:5px;background:#f5f7fa;} QPushButton:checked{background:#233c5c;color:white;border-color:#233c5c;} QComboBox{padding:5px;}')
outer=QVBoxLayout(w);outer.setContentsMargins(18,14,18,14)
head=QHBoxLayout()
head.addWidget(QLabel(f'<b style="font-size:25px">WAVEGO / {profile_title}</b><br>OfficeWorld · Go2-W · 4 cm 测量点累积 · 同一坐标与相机' if not CHINESE else f'<b style="font-size:25px">WAVEGO / {profile_title}</b><br>OfficeWorld · Go2-W · 4 cm 测量点累积 · 统一地图坐标与视角'))
head.addStretch()
head.addWidget(QLabel(f'数据：{duration:.1f} s 已录制扫描<br>交互：拖动旋转 / 滚轮缩放 / 时间轴回放'))
outer.addLayout(head)
tabs=QHBoxLayout();group=QButtonGroup(w);buttons={}
titles={'graph':'01  场景图','semantic':'02  语义颜色 + 目标框','raw':'03  纯点云','combined':'融合叠加'} if CHINESE else {'graph':'01  Graph 场景图','semantic':'02  语义颜色 + Box','raw':'03  纯点云','combined':'融合叠加'}
if (DATA_ROOT/'outputs/three_representations/hovsg_floor_probe/floor_labels.npz').exists():titles['hovsg']='原版楼层分割' if CHINESE else 'HOV-SG 分层'
if args.show_features and (DATA_ROOT/'outputs/academic_upgrade/utonia_features.npz').exists():titles['utonia']='Utonia 特征（实验）' if not CHINESE else 'Utonia 特征（实验）'
if args.mode not in titles:ap.error('Requested method result is not prepared')
mode=[args.mode]
for key,title in titles.items():
    b=QPushButton(title);b.setCheckable(True);group.addButton(b);tabs.addWidget(b);buttons[key]=b
model_panel_dir=DATA_ROOT/'outputs/rgbd_models'/('panels_cn' if CHINESE else 'panels')
if not model_panel_dir.exists():model_panel_dir=DATA_ROOT/'outputs/rgbd_models/panels'
model_panels=sorted(model_panel_dir.glob('frame_*.png'))
if model_panels:
    model_button=QPushButton('RGB-D 三模型');tabs.addWidget(model_button)
    def open_models():
        dialog=QDialog(w);dialog.setWindowTitle('新记录 | 深度估计 · 目标检测 · 分割');layout=QVBoxLayout(dialog)
        layout.addWidget(QLabel('同步 RGB-D 的实际模型输出；深度估计为相对深度，地图投影使用录制的米制深度。'))
        pic=QLabel();pic.setAlignment(Qt.AlignCenter);layout.addWidget(pic,1);select=QComboBox();layout.addWidget(select)
        for p in model_panels:
            frame=int(p.stem.split('_')[-1]);meta=json.loads((DATA_ROOT/f'outputs/rgbd_models/frames/{frame:04d}.json').read_text())
            select.addItem(f'{meta["frame"]["stamp"]-report["first_stamp"]:.1f} s  /  {len(meta["objects"])} 个候选目标',str(p))
        select.currentIndexChanged.connect(lambda _:pic.setPixmap(QPixmap(select.currentData()).scaled(1080,754,Qt.KeepAspectRatio,Qt.SmoothTransformation)))
        select.setCurrentIndex(min(2,len(model_panels)-1));select.currentIndexChanged.emit(select.currentIndex());dialog.resize(1140,870);dialog.show()
    model_button.clicked.connect(open_models)
tabs.addStretch();outer.addLayout(tabs)
body=QHBoxLayout();sidebar=QWidget();sidebar.setFixedWidth(258);left=QVBoxLayout(sidebar);left.setContentsMargins(0,10,14,0)
caption=QLabel();caption.setWordWrap(True);left.addWidget(caption)
left.addSpacing(14)
semantic_legend=QWidget();sl=QVBoxLayout(semantic_legend);sl.setContentsMargins(0,0,0,0)
sl.addWidget(QLabel('<b>语义 / 节点图例</b>'))
for name,key in ([('楼梯','stairs'),('坡道','ramp'),('圆柱障碍物','cylinder'),('行人','person'),('墙面','wall'),('地面','floor'),('未分类几何','unassigned')] if CHINESE else [('楼梯 stairs','stairs'),('坡道 ramp','ramp'),('圆柱 cylinder','cylinder'),('墙面 wall','wall'),('地面 floor','floor'),('未分类几何','unassigned')]):
    color=hex_color(CLASS_COLORS[key])
    sl.addWidget(QLabel(f'<span style="color:{color};font-size:22px">●</span> {name}'))
left.addWidget(semantic_legend)
obstacle_text = (f'<b>障碍物类别统计</b><br>楼梯：{counts["stairs"]} 个候选簇<br>坡道：{counts["ramp"]} 个候选簇<br>圆柱障碍物：{counts["cylinder"]} 个候选簇<br>行人：{counts["person"]} 个候选簇') if CHINESE else (f'<b>障碍物类别统计</b><br>楼梯：{counts["stairs"]}<br>坡道：{counts["ramp"]}<br>圆柱：{counts["cylinder"]}<br>行人：{counts["person"]}')
left.addWidget(QLabel(obstacle_text))
left.addSpacing(8)
height_legend=QWidget();hl=QVBoxLayout(height_legend);hl.setContentsMargins(0,0,0,0)
hl.addWidget(QLabel('<b>'+({'fastlio':'FAST-LIO 风格 / Z 高度','paper':'低饱和蓝金 / Z 高度','batlow':'上一版 Batlow / Z 高度'} if CHINESE else {'fastlio':'FAST-LIO / Z 高度','paper':'低饱和蓝金 / Z 高度','batlow':'上一版 Batlow / Z 高度'})[args.style]+'</b>'))
colorbar=QLabel();colorbar.setFixedHeight(16)
if args.style=='fastlio':
    import colorsys
    legend_colors=[tuple(round(v*255) for v in colorsys.hsv_to_rgb((i/8)*.833333,1.,1.)) for i in range(9)]
else:legend_colors=[(HEIGHT_LUT if args.style=='batlow' else PAPER_LUT)[round(i*255/8)] for i in range(9)]
stops=','.join(f'stop:{i/8:.3f} {hex_color(c)}' for i,c in enumerate(legend_colors))
colorbar.setStyleSheet('background:qlineargradient(x1:0,y1:0,x2:1,y2:0,'+stops+');border-radius:3px;')
hl.addWidget(colorbar)
hl.addWidget(QLabel(f'{HEIGHT_RANGE[0]}　　　　　　　　{HEIGHT_RANGE[1]}'))
left.addWidget(height_legend)
graph_legend=QLabel('<b>场景图结构</b><br><span style="color:#715697">●</span> 数据容器<br><span style="color:#63a68f">●</span> 楼层候选<br><span style="color:#ad8246">●</span> 观测区域<br>物体簇与墙面候选<br><br>灰边：层级关联<br>蓝边：楼梯连接候选' if CHINESE else '<b>Graph 结构</b><br><span style="color:#715697">●</span> B · 数据容器<br><span style="color:#63a68f">●</span> F · 楼层候选<br><span style="color:#ad8246">●</span> R · 观测区域<br>O / W · 物体簇 / 墙面<br><br>灰边：层级关联<br>蓝边：楼梯连接候选')
left.addWidget(graph_legend)
left.addStretch()
state=QLabel('正在载入记录…');state.setWordWrap(True);left.addWidget(state)
left.addWidget(QLabel(('模型预测与结构候选分开显示<br>'+('行人仅作历史观测，不进行动态追踪' if counts['person'] else '本记录没有可靠行人检测')) if CHINESE else ('类别来自模型预测；“*” 为结构候选<br>'+('行人：历史观测，非动态追踪' if counts['person'] else '本记录没有有效行人标签'))))
body.addWidget(sidebar)
holder=QWidget();canvas=QVBoxLayout(holder);canvas.setContentsMargins(0,0,0,0)
viewtitle=QLabel();viewtitle.setStyleSheet('padding:9px;background:#f4f7fa;font-size:16px;font-weight:600;');canvas.addWidget(viewtitle)
body.addWidget(holder,1);outer.addLayout(body,1)
map_view=None
if not NATIVE_RVIZ:
    from embedded_map import EmbeddedMap
    map_view=EmbeddedMap(holder,canvas,RUNTIME,args.style)
footer=QLabel(source_caption)
footer.setStyleSheet('font-size:12px;color:#63728a;');outer.addWidget(footer)
controls=QHBoxLayout();play=QPushButton('▶ 回放');controls.addWidget(play)
slider=QSlider(Qt.Horizontal);slider.setRange(0,int(duration*10));slider.setValue(slider.maximum());controls.addWidget(slider,1)
clock=QLabel();controls.addWidget(clock)
speed=QComboBox();speed.addItems(['1×','2×','4×','8×']);speed.setCurrentIndex(2);controls.addWidget(speed)
labels=QCheckBox('类别标签');labels.setChecked(True);controls.addWidget(labels)
path=QCheckBox('轨迹');controls.addWidget(path)
height=QCheckBox('按高度着色');height.setChecked(True);controls.addWidget(height)
outer.addLayout(controls)

def send_camera(name='overview'):
    v={'overview':[31.,.64,.65,[9.,3.,3.3]],'side':[30.,.22,1.57,[9.,3.,2.7]],'top':[34.,1.56,0.,[9.,3.,2.]]}[name]
    camera_pub.publish(String(data=json.dumps({'origin':'three_view','values':v})))

for title,preset in [('总览','overview'),('侧视','side'),('俯视','top')]:
    b=QPushButton(title);controls.addWidget(b);b.clicked.connect(lambda _,p=preset:send_camera(p))

descriptions={
 'graph':(f'<b>场景图 / 关系结构</b><br><br>节点位于同一地图坐标，背景为淡化测量点云。<br><br>{counts["building"]} 个数据容器 · {counts["floor"]} 个楼层候选<br>{counts["region"]} 个观测区域 · {object_count} 个物体簇<br>{counts["wall"]} 个墙面候选<br><br>不是完整 Hydra / S-Graphs 推理结果。' if CHINESE else f'<b>Graph / 关系结构</b><br><br>节点位于同一地图坐标。背景为淡化测量点云。<br><br>{counts["building"]} 容器 · {counts["floor"]} 楼层候选<br>{counts["region"]} 观测区域 · {object_count} 物体簇<br>{counts["wall"]} 墙面候选<br><br>不是完整 Hydra / S-Graphs 推理结果.'),
 'semantic':('<b>语义 / 类别与实例</b><br><br>类别、目标框和节点统一使用低饱和配色。<br><br>目标框来自 Grounding DINO + SAM2；中文名称仅为显示翻译。<br><br>未分类几何为浅蓝背景。' if CHINESE else '<b>Semantic / 类别与实例</b><br><br>类别、Box 和节点统一使用低饱和配色。<br><br>Grounding DINO + SAM2 目标框；中文名称仅为显示翻译。<br><br>未分类几何为浅蓝背景。'),
 'raw':('<b>几何 / 纯测量点云</b><br><br>'+{'fastlio':'FAST-LIO 风格 RViz AxisColor，按 Z 高度着色。','paper':'低饱和蓝金连续色，按 Z 高度着色。','batlow':'保留上一版 Batlow 连续高度色。'}[args.style]+'<br><br>4 cm 体素内保留首个原始点；固定高度色标不代表语义类别。' if CHINESE else '<b>Geometry / 纯测量点云</b><br><br>'+{'fastlio':'FAST-LIO 同款 RViz AxisColor，按 Z 高度着色。','paper':'低饱和蓝金连续色，按 Z 高度着色。','batlow':'保留上一版 Batlow 连续高度色。'}[args.style]+'<br><br>4 cm 体素内保留首个原始点；固定高度色标，不代表语义类别。'),
 'combined':('<b>融合 / 同图叠加</b><br><br>场景图、彩色语义和物体框叠加在同一坐标。<br><br>灰边表示包含关系，蓝边表示楼梯连接候选。' if CHINESE else '<b>Combined / 同图叠加</b><br><br>Graph、彩色语义与物体 Box 叠加在同一坐标。<br><br>灰边为包含关系，蓝边为楼梯连接候选。')}
descriptions['hovsg']=f'<b>HOV-SG / 原版模块实测</b><br><br>原版高度直方图 + 峰值聚类，得到 {floor_result.get("num_floors","—")} 个楼层区间。<br><br><span style="color:#4090ca">●</span> 层区间 0：蓝色<br><span style="color:#e4a63b">●</span> 层区间 1：金色<br>灰色：区间外点<br><br>仅运行楼层分割阶段。<br>这里是最终地图的静态结果；颜色表示层归属。'
descriptions['utonia']=f'<b>Utonia / 预训练特征</b><br><br>官方预训练编码器 + 官方特征 PCA 配色。<br><br>{features.get("encoder_input_points",0):,} 个输入点<br>{features.get("feature_dim",0):,} 维局部特征<br><br>相近颜色表示特征相似，不代表语义类别或置信度。<br><br>最终地图离线推理；颜色映射回全部实测点，坐标保持不变。'

def choose(key=None):
    if key:mode[0]=key
    key=mode[0];buttons[key].setChecked(True);caption.setText(descriptions[key]);viewtitle.setText(titles[key]+'  /  lio_map · metres')
    semantic_legend.setVisible(key in ['semantic','combined','graph'])
    height_legend.setVisible(key in ['raw','graph'])
    graph_legend.setVisible(key in ['graph','combined'])
    footer.setText('来源：官方 Utonia 编码器与 PCA；输入仅 XYZ，RGB / 法向量置零；特征可视化不是类别预测' if key=='utonia' else source_caption)
    height.setEnabled(key=='raw')
    static=key in ['hovsg','utonia']
    if static:
        playing[0]=False;play.setText('▶ 回放');slider.setValue(slider.maximum())
    play.setEnabled(not static);slider.setEnabled(not static)
    shown_title=titles[key]+'  /  统一地图坐标 · 米' if CHINESE else titles[key]+'  /  lio_map · metres'
    viewtitle.setText(shown_title)
    options=dict(mode=key,style=args.style,labels=labels.isChecked(),trajectory=path.isChecked(),height=height.isChecked())
    mode_pub.publish(String(data=json.dumps(options)))
    if map_view:map_view.choose(options)

for key,b in buttons.items():b.clicked.connect(lambda _,k=key:choose(k))
for control in [labels,path,height]:control.toggled.connect(lambda _:choose())

playing=[False];last=[time.monotonic()]
def seek(value):
    t=duration if value==slider.maximum() else value/10
    clock.setText(f'{t:.1f} / {duration:.1f} s');seek_pub.publish(Float64(data=t))
slider.valueChanged.connect(seek)
def toggle():
    if not playing[0] and slider.value()==slider.maximum():slider.setValue(0)
    playing[0]=not playing[0];last[0]=time.monotonic();play.setText('Ⅱ 暂停' if playing[0] else '▶ 回放')
play.clicked.connect(toggle)
def tick():
    now=time.monotonic()
    if playing[0]:
        slider.setValue(min(slider.maximum(),slider.value()+max(1,round((now-last[0])*10*2**speed.currentIndex()))))
        if slider.value()==slider.maximum():playing[0]=False;play.setText('▶ 回放')
    last[0]=now

class Bridge(QObject):status=pyqtSignal(str)
bridge=Bridge();latest=[{}]
def update_status(raw):
    s=json.loads(raw);latest[0]=s
    state.setText(f'<b>已累积 {s["points"]:,} 点</b><br>{len(scene["nodes"])} 节点 / {len(scene["edges"])} 条关系（完整图）')
bridge.status.connect(update_status)
rospy.Subscriber('/reference_map/status',String,lambda m:bridge.status.emit(m.data),queue_size=1)
worker=None
embedded={}
if NATIVE_RVIZ:
    worker=QProcess(w);worker.setStandardOutputFile(str(RUNTIME/'three_panel.log'));worker.setProcessChannelMode(QProcess.MergedChannels)
    statefile=RUNTIME/'reference_compare_window.json';statefile.unlink(missing_ok=True)
    worker_args=[str(ROOT/'scripts/reference_panel.py'),'--side','compare','--style',args.style]
    if auto_capture:worker_args.append('--automated-capture')
    worker.start(sys.executable,worker_args)
    def embed():
        if not statefile.exists():return
        s=json.loads(statefile.read_text())
        if s['pid']!=worker.processId():return
        native=QWindow.fromWinId(s['window_id'])
        # Optional diagnostic fallback; the default constructs EmbeddedMap above.
        notice=QLabel('RViz 已在独立窗口打开\n可在右侧窗口旋转、缩放和切换显示层')
        notice.setAlignment(Qt.AlignCenter)
        notice.setStyleSheet('font-size:20px;color:#607089;background:#f4f7fa;border:1px solid #d9e2ec;')
        canvas.addWidget(notice,1)
        native.show()
        embedded.update(native=native,container=None,window_id=s['window_id'])
        embedding.stop()
        send_camera();choose();seek(slider.value())
    embedding=QTimer();embedding.timeout.connect(embed);embedding.start(150)
else:
    embedded['window_id']=int(map_view.frame.winId())
timer=QTimer();timer.timeout.connect(tick);timer.start(200)
w.resize(2040,1140);w.move(15,20);w.show();choose();seek(slider.value());send_camera()
((RUNTIME if args.record else OUT)/'viewer_window.json').write_text(json.dumps({'window_id':int(w.winId())}))

def capture(name):
    w.raise_();w.activateWindow()
    screen=app.primaryScreen();screen.grabWindow(int(w.winId())).save(str(OUT/(name+'.png')))
    screen.grabWindow(embedded['window_id']).save(str(OUT/(name+'_map.png')))
    audit={'camera':json.loads((RUNTIME/'reference_compare_camera.json').read_text()),
           'display':json.loads((RUNTIME/'comparison_display_state.json').read_text()),'status':latest[0]}
    if map_view:
        audit['labels']=json.loads((RUNTIME/'visible_labels.json').read_text())
    (OUT/(name+'.json')).write_text(json.dumps(audit,indent=2))
    print('CAPTURE',name,flush=True)

if args.capture or args.capture_compare:
    shots=[('01_graph','graph','overview'),('02_semantic_boxes','semantic','overview'),('03_raw_cloud','raw','overview'),('04_combined','combined','overview'),('05_stairs_side','combined','side')]
    if 'hovsg' in titles:shots.append(('06_hovsg_upstream_floors','hovsg','overview'))
    if 'utonia' in titles:shots.append(('07_utonia_features','utonia','overview'))
    if args.capture_compare:
        shots=[(view+'_'+key,key,view) for view in ['overview','side','top'] for key in ['raw','semantic','graph']]
    def next_shot():
        if not shots:app.quit();return
        name,representation,view=shots.pop(0);choose(representation);send_camera(view)
        QTimer.singleShot(4000,lambda:(capture(name),QTimer.singleShot(100,next_shot)))
    QTimer.singleShot(7000,next_shot)
if args.capture_hovsg:
    def capture_hovsg():
        choose('hovsg');send_camera('overview')
        QTimer.singleShot(4500,lambda:(capture('06_hovsg_upstream_floors'),app.quit()))
    QTimer.singleShot(7500,capture_hovsg)

recording=[None]
if args.record:
    def finish():
        recording[0].stdin.write(b'q');recording[0].stdin.flush()
        QTimer.singleShot(2000,app.quit)
    def record():
        w.raise_();w.activateWindow();choose('raw');send_camera();slider.setValue(0)
        log=(RUNTIME/'three_video.log').open('wb')
        capture_window=int(embedded['window_id']) if NATIVE_RVIZ else int(w.winId())
        recording[0]=subprocess.Popen(['/usr/bin/ffmpeg','-y','-loglevel','warning','-f','x11grab','-framerate','12','-draw_mouse','0','-window_id',str(capture_window),'-i',os.environ.get('DISPLAY',':0'),'-c:v','libx264','-preset','veryfast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'three_representations_demo.mp4')],stdin=subprocess.PIPE,stdout=log,stderr=log)
        QTimer.singleShot(1500,toggle)
        QTimer.singleShot(15000,lambda:choose('semantic'))
        QTimer.singleShot(30000,lambda:choose('graph'))
        QTimer.singleShot(47000,lambda:choose('combined'))
        QTimer.singleShot(54000,lambda:send_camera('side'))
        QTimer.singleShot(61000,lambda:send_camera('overview'))
        if 'hovsg' in titles:
            QTimer.singleShot(68000,lambda:choose('hovsg'))
            QTimer.singleShot(75000,lambda:send_camera('side'))
            if 'utonia' in titles:
                QTimer.singleShot(81000,lambda:(choose('utonia'),send_camera('overview')))
                QTimer.singleShot(91000,lambda:send_camera('side'))
                QTimer.singleShot(101000,finish)
            else:QTimer.singleShot(82000,finish)
        else:QTimer.singleShot(68000,finish)
    QTimer.singleShot(7000,record)
signal.signal(signal.SIGINT,lambda *_:app.quit())
signal.signal(signal.SIGTERM,lambda *_:app.quit())
app.exec_()
if map_view:map_view.close()
if worker:
    worker.terminate();worker.waitForFinished(3000)
if recording[0] and recording[0].poll() is None:
    recording[0].communicate(b'q',timeout=10)
