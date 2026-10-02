#!/usr/bin/python3
"""Focused-window keyboard/button teleoperation with bridge-level deadman."""
import os,sys,time,subprocess,json,math,signal
from pathlib import Path
from PyQt5 import QtCore,QtWidgets
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.srv import CancelGoal
R=Path(__file__).resolve().parents[1]
class Keys:
 def __init__(self):self.held=set();self.latched=False
 def stop(self,latch=False):self.held.clear();self.latched=latch
 def command(self,speed):
  if self.latched:return 0.,0.
  return speed*(int('forward' in self.held)-int('back' in self.held)),.7*(int('left' in self.held)-int('right' in self.held))
class Panel(QtWidgets.QWidget):
 def __init__(self):
  super().__init__();self.setWindowTitle('Go2-W 手动驾驶 | WASD / 方向键');self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint);self.resize(460,360);self.keys=Keys();self.owned=False;self.node=rclpy.create_node('go2w_keyboard_control');self.pub=self.node.create_publisher(Twist,'/cmd_vel_nav',10);self.node.create_subscription(Odometry,'/odom/mujoco_odom',self.odom,10);self.pose=None;self.yaw=0.;self.recovering_until=0.
  layout=QtWidgets.QVBoxLayout(self);self.status=QtWidgets.QLabel('点击“接管手动”，再按住按键或按钮');self.status.setWordWrap(True);layout.addWidget(self.status)
  b=QtWidgets.QPushButton('接管手动 / 解除停止锁定');b.clicked.connect(self.take);layout.addWidget(b)
  modes=QtWidgets.QHBoxLayout();layout.addLayout(modes)
  for text,mode in [('1 普通行走','flat'),('2 爬楼 / 越障','up')]:
   b=QtWidgets.QPushButton(text);b.clicked.connect(lambda checked=False,m=mode:self.setmode(m));modes.addWidget(b)
  b=QtWidgets.QPushButton('R 恢复站立（重启仿真到起点）');b.clicked.connect(self.recover);layout.addWidget(b)
  grid=QtWidgets.QGridLayout();layout.addLayout(grid)
  for text,key,row,col in [('↑ W 前进','forward',0,1),('← A 左转','left',1,0),('↓ S 后退','back',1,1),('→ D 右转','right',1,2)]:
   b=QtWidgets.QPushButton(text);b.setMinimumHeight(58);b.setFocusPolicy(QtCore.Qt.NoFocus);b.pressed.connect(lambda k=key:self.keys.held.add(k));b.released.connect(lambda k=key:self.keys.held.discard(k));grid.addWidget(b,row,col)
  b=QtWidgets.QPushButton('停止并锁定（空格 / Esc）');b.setStyleSheet('background:#a62a32;color:white;padding:12px');b.clicked.connect(lambda:self.keys.stop(True));layout.addWidget(b)
  self.speed=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.speed.setRange(15,80);self.speed.setValue(45);layout.addWidget(QtWidgets.QLabel('速度 0.15–0.80 m/s；先用低速熟悉场景'));layout.addWidget(self.speed)
  layout.addWidget(QtWidgets.QLabel('W/↑ 前进 · S/↓ 后退 · A/← 左转 · D/→ 右转\nR 重启仿真恢复站立；空格/Esc 停止锁定。\n松键或失焦停止。此面板仅控制仿真。'))
  self.setStyleSheet(self.styleSheet()+'QWidget{font-size:15px;}');QtWidgets.QApplication.instance().installEventFilter(self);self.timer=QtCore.QTimer();self.timer.timeout.connect(self.tick);self.timer.start(50)
 def write(self,name,text):
  p=R/'.run/go2w'/name;p.parent.mkdir(parents=True,exist_ok=True);t=p.with_suffix('.teleop.tmp');t.write_text(text);t.replace(p)
 def odom(self,m):
  q=m.pose.pose.orientation;self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));self.pose=(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,time.monotonic())
 def take(self):
  self.owned=True;self.keys.stop(True);self.write('manual_control',f'{time.time()+.25} 0\n');self.pub.publish(Twist());subprocess.run(['systemctl','--user','stop','go2w-course.service'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
  self.clients=[]
  for name in ('navigate_through_poses','navigate_to_pose'):
   c=self.node.create_client(CancelGoal,f'/{name}/_action/cancel_goal');self.clients.append(c)
   if c.service_is_ready():c.call_async(CancelGoal.Request())
  self.write('stair_frame','manual 0 0 0 1 10\n');self.write('nav_mode','flat\n')
  try:result=subprocess.run(['/usr/bin/python3',str(R/'scripts/set_go2w_nav_mode.py'),'flat'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=15)
  except (subprocess.TimeoutExpired,OSError) as exc:self.status.setText('接管超时，保持停止：'+str(exc));return False
  if result.returncode:self.status.setText('接管失败，保持停止：'+result.stdout[-200:]);return False
  self.keys.stop(False);self.activateWindow();self.setFocus();return True
 def setmode(self,mode):
  self.keys.stop(True)
  if not self.owned and not self.take():return
  if mode=='up' and self.pose:
   self.write('stair_frame',f'manual_up_{time.time_ns()} {self.pose[0]} {self.pose[1]} {self.yaw} 8 {self.pose[2]+1}\n');self.speed.setValue(65)
  else:self.write('stair_frame','manual_flat 0 0 0 1 10\n');self.speed.setValue(45)
  self.write('nav_mode',mode+'\n');self.keys.stop(False);self.setFocus()
 def recover(self):
  self.keys.stop(True);self.owned=True;self.write('manual_control','0 0\n');self.pub.publish(Twist())
  result=subprocess.run(['systemd-run','--user','--quiet','--collect','--unit=go2w-recovery','--property=WorkingDirectory='+str(R),'/bin/bash',str(R/'scripts/recover_go2w_simulation.sh')],capture_output=True,text=True)
  self.status.setText('正在重启仿真并恢复站立；控制面板随后自动重新打开。' if result.returncode==0 else '恢复未启动：'+result.stderr[-120:])
 def eventFilter(self,obj,e):
  if e.type() in (QtCore.QEvent.WindowDeactivate,QtCore.QEvent.ApplicationDeactivate):self.keys.stop(self.keys.latched)
  if e.type() in (QtCore.QEvent.KeyPress,QtCore.QEvent.KeyRelease) and self.isActiveWindow():
   if e.isAutoRepeat():return True
   mapping={QtCore.Qt.Key_W:'forward',QtCore.Qt.Key_Up:'forward',QtCore.Qt.Key_S:'back',QtCore.Qt.Key_Down:'back',QtCore.Qt.Key_A:'left',QtCore.Qt.Key_Left:'left',QtCore.Qt.Key_D:'right',QtCore.Qt.Key_Right:'right'}
   if e.key() in (QtCore.Qt.Key_1,QtCore.Qt.Key_2):
    if e.type()==QtCore.QEvent.KeyPress:self.setmode('up' if e.key()==QtCore.Qt.Key_2 else 'flat')
    return True
   if e.key()==QtCore.Qt.Key_R:
    if e.type()==QtCore.QEvent.KeyPress:self.recover()
    return True
   if e.key() in (QtCore.Qt.Key_Space,QtCore.Qt.Key_Escape):self.keys.stop(True);return True
   if e.key() in mapping:
    if e.type()==QtCore.QEvent.KeyPress:self.keys.held.add(mapping[e.key()])
    else:self.keys.held.discard(mapping[e.key()])
    return True
  return super().eventFilter(obj,e)
 def tick(self):
  rclpy.spin_once(self.node,timeout_sec=0)
  if not self.owned:return
  if not self.isActiveWindow():self.keys.stop(self.keys.latched)
  v,w=self.keys.command(self.speed.value()/100)
  if time.monotonic()<self.recovering_until:v=w=0.;self.keys.stop(True)
  if self.pose is None or time.monotonic()-self.pose[3]>1:v=w=0.;self.keys.stop(True)
  self.write('manual_control',f'{time.time()+.25} {int(bool(v or w))}\n');m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)
  state='停止已锁定' if self.keys.latched else ('正在运动' if v or w else '手动待命：按住按键运动')
  self.status.setText(f'{state}  |  速度 {v:.2f} m/s / 转向 {w:.2f} rad/s'+(f'\n位置 x={self.pose[0]:.2f}, y={self.pose[1]:.2f}, z={self.pose[2]:.2f}' if self.pose else ''))
 def closeEvent(self,e):
  if self.owned:self.keys.stop(True);self.write('manual_control','0 0\n');self.pub.publish(Twist())
  e.accept()
if __name__=='__main__':
 rclpy.init();app=QtWidgets.QApplication(sys.argv);p=Panel();p.show();p.take();signal.signal(signal.SIGTERM,lambda *_:app.quit());signal.signal(signal.SIGINT,lambda *_:app.quit());app.exec_();p.node.destroy_node();rclpy.shutdown()
