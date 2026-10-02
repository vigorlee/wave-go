"""Send a console command only while the UE window owns keyboard focus."""
import ctypes as C
import subprocess
import sys
import time

x = C.CDLL('libX11.so.6')
t = C.CDLL('libXtst.so.6')
x.XOpenDisplay.argtypes = [C.c_char_p]
x.XOpenDisplay.restype = C.c_void_p
d = x.XOpenDisplay(b':0')
assert d
x.XStringToKeysym.argtypes = [C.c_char_p]
x.XStringToKeysym.restype = C.c_ulong
x.XKeysymToKeycode.argtypes = [C.c_void_p, C.c_ulong]
x.XKeysymToKeycode.restype = C.c_uint
x.XFlush.argtypes = [C.c_void_p]
x.XSetInputFocus.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_ulong]
x.XGetInputFocus.argtypes = [C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)]
t.XTestFakeKeyEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
wid = next(int(l.split()[0], 16) for l in subprocess.check_output(['wmctrl', '-l'], text=True).splitlines() if 'zsibot_mujoco_ue (64-bit' in l)

def check():
    focus, revert = C.c_ulong(), C.c_int()
    x.XGetInputFocus(d, C.byref(focus), C.byref(revert))
    if focus.value != wid:
        raise RuntimeError('UE lost keyboard focus; command canceled')

def key(name, on):
    check()
    code = x.XKeysymToKeycode(d, x.XStringToKeysym(name.encode()))
    if not code:
        raise ValueError(name)
    t.XTestFakeKeyEvent(d, code, on, 0)

def tap(name):
    key(name, 1)
    key(name, 0)
    x.XFlush(d)
    time.sleep(.025)

engine = subprocess.check_output(['ibus', 'engine'], text=True).strip()
subprocess.run(['wmctrl', '-ir', hex(wid), '-b', 'add,above'], check=True)
subprocess.run(['wmctrl', '-ia', hex(wid)], check=True)
time.sleep(.3)
x.XSetInputFocus(d, wid, 2, 0)
x.XFlush(d)
subprocess.run(['ibus', 'engine', 'xkb:us::eng'], check=True)
time.sleep(.25)
try:
    tap('Escape')
    tap('grave')
    time.sleep(.4)
    key('Control_L', 1)
    tap('a')
    key('Control_L', 0)
    tap('BackSpace')
    for c in sys.argv[1]:
        shifted = c.isupper() or c in '_():"|'
        if shifted:
            key('Shift_L', 1)
        name = {' ': 'space', '/': 'slash', '.': 'period', '_': 'minus', '-': 'minus', ':': 'semicolon', '=': 'equal', "'": 'apostrophe', '(': '9', ')': '0', ',': 'comma', '"': 'apostrophe', '|': 'backslash'}.get(c, c.lower())
        tap(name)
        if shifted:
            key('Shift_L', 0)
    tap('Return')
    time.sleep(.7)
    tap('Escape')
finally:
    for name in ('Control_L', 'Shift_L'):
        t.XTestFakeKeyEvent(d, x.XKeysymToKeycode(d, x.XStringToKeysym(name.encode())), 0, 0)
    x.XFlush(d)
    subprocess.run(['ibus', 'engine', engine])
