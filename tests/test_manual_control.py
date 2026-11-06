"""Exercise fail-closed GUI paths, including stale odometry and focus loss."""
import os,sys,time
from pathlib import Path
os.environ['QT_QPA_PLATFORM']='offscreen'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from PyQt5 import QtWidgets
from go2w_manual_control import Panel

def test_stale_odometry_and_focus_loss_publish_float_zero(monkeypatch):
 monkeypatch.setattr(rclpy,"spin_once",lambda *args,**kwargs:None)
 rclpy.init();app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
 p=Panel();p.timer.stop();messages=[];writes={}
 class Publisher:
  def publish(self,m):messages.append(m)
 p.pub=Publisher();p.write=lambda k,v:writes.update({k:v});p.owned=True;p.isActiveWindow=lambda:True
 try:
  p.keys.held.add('forward');p.pose=None;p.tick()
  assert messages[-1].linear.x==0. and p.keys.latched
  p.keys.stop(False);p.keys.held.add('forward');p.pose=(8.,.7,.4,time.monotonic());p.isActiveWindow=lambda:False;p.tick()
  assert messages[-1].linear.x==0. and not p.keys.held
  p.isActiveWindow=lambda:True;p.keys.held.add('forward');p.keys.stop(True);p.keys.held.add('forward');p.tick()
  assert messages[-1].linear.x==0.
 finally:p.owned=False;p.close();p.node.destroy_node();rclpy.shutdown()
