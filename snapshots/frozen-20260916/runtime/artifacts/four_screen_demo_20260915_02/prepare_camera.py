import ctypes,time,subprocess,json,re
from pathlib import Path
from PyQt5.QtWidgets import QApplication
R=Path(__file__).resolve().parent
rows=subprocess.check_output(['wmctrl','-l'],text=True).splitlines()
pats=['MuJoCo : ','four_screen.rviz','Depth Anything V2 + Grounded SAM 2','zsibot_mujoco_ue (64-bit']
ids=[next(row.split()[0] for row in rows if p in row) for p in pats]
for wid,(a,b) in zip(ids,[(100,80),(2100,80),(100,1190),(2100,1190)]):
 subprocess.run(['wmctrl','-ir',wid,'-b','remove,maximized_vert,maximized_horz']);subprocess.run(['wmctrl','-ir',wid,'-e',f'0,{a},{b},1700,850']);subprocess.run(['wmctrl','-ir',wid,'-b','add,above'])
time.sleep(2)
x=ctypes.CDLL('libX11.so.6');t=ctypes.CDLL('libXtst.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;d=x.XOpenDisplay(None)
x.XFlush.argtypes=[ctypes.c_void_p];x.XKeysymToKeycode.argtypes=[ctypes.c_void_p,ctypes.c_ulong];x.XKeysymToKeycode.restype=ctypes.c_uint
for f,args in [('XTestFakeMotionEvent',[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_ulong]),('XTestFakeButtonEvent',[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong]),('XTestFakeKeyEvent',[ctypes.c_void_p,ctypes.c_uint,ctypes.c_int,ctypes.c_ulong])]:getattr(t,f).argtypes=args
def key(k,on):t.XTestFakeKeyEvent(d,x.XKeysymToKeycode(d,k),on,0);x.XFlush(d)
def b(k,on):t.XTestFakeButtonEvent(d,k,on,0);x.XFlush(d)
def move(a,b):t.XTestFakeMotionEvent(d,-1,a,b,0);x.XFlush(d)
move(800,500);b(1,1)
for i in range(1,31):move(800+18*i,500+4*i);time.sleep(.012)
b(1,0);time.sleep(.2)
move(841,738);key(0xffe3,1)
for _ in range(2):b(3,1);time.sleep(.05);b(3,0);time.sleep(.1)
key(0xffe3,0)
for _ in range(3):b(4,1);b(4,0);time.sleep(.1)
key(0xff09,1);key(0xff09,0)
time.sleep(.5);app=QApplication([]);app.primaryScreen().grabWindow(0).save(str(R/'prepared_screen.png'))
