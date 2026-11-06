import ctypes as C,time,sys,subprocess
x=C.CDLL('libX11.so.6');t=C.CDLL('libXtst.so.6')
x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=C.c_void_p
d=x.XOpenDisplay(b':0');assert d
x.XStringToKeysym.argtypes=[C.c_char_p];x.XStringToKeysym.restype=C.c_ulong
x.XKeysymToKeycode.argtypes=[C.c_void_p,C.c_ulong];x.XKeysymToKeycode.restype=C.c_uint
x.XFlush.argtypes=[C.c_void_p];t.XTestFakeKeyEvent.argtypes=[C.c_void_p,C.c_uint,C.c_int,C.c_ulong]
def key(name,on):t.XTestFakeKeyEvent(d,x.XKeysymToKeycode(d,x.XStringToKeysym(name.encode())),on,0)
def tap(name):key(name,1);key(name,0);x.XFlush(d);time.sleep(.015)
wid=next(l.split()[0] for l in subprocess.check_output(['wmctrl','-l'],text=True).splitlines() if 'zsibot_mujoco_ue (64-bit' in l)
subprocess.run(['wmctrl','-ia',wid],check=True);time.sleep(.3)
tap('Escape');tap('grave');time.sleep(.4)
for c in sys.argv[1]:
 shifted=c.isupper() or c=='_'
 if shifted:key('Shift_L',1)
 name={' ':'space','/':'slash','.':'period','_':'minus','-':'minus',':':'colon','=':'equal',"'":'apostrophe'}.get(c,c.lower())
 tap(name)
 if shifted:key('Shift_L',0)
tap('Return');time.sleep(.5);tap('Escape')
