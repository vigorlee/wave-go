#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${DISPLAY:-}" ]] || ! command -v xwininfo >/dev/null 2>&1; then
  exit 0
fi

WINDOW_ID="$(xwininfo -root -tree 2>/dev/null | awk '/MuJoCo : Nezha2 YardWorld navigation scene/ && !found {print $1; found=1}')"
if [[ -z "${WINDOW_ID}" ]]; then
  echo "[WARN] Nezha2 MuJoCo window was not found; navigation remains active." >&2
  exit 0
fi

python3 - "${WINDOW_ID}" <<'PY'
import ctypes
import sys
import time

window = int(sys.argv[1], 16)
x11 = ctypes.CDLL("libX11.so.6")
xtst = ctypes.CDLL("libXtst.so.6")
x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XKeysymToKeycode.restype = ctypes.c_ubyte
x11.XFlush.argtypes = [ctypes.c_void_p]
xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]

display = x11.XOpenDisplay(None)
if not display:
    raise SystemExit("cannot open X display")

x11.XRaiseWindow(display, window)
x11.XSetInputFocus(display, window, 1, 0)
keycode = x11.XKeysymToKeycode(display, ord("]"))
xtst.XTestFakeKeyEvent(display, keycode, 1, 0)
xtst.XTestFakeKeyEvent(display, keycode, 0, 0)
x11.XFlush(display)
time.sleep(0.2)
PY

echo "[OK] MuJoCo navigation tracking view selected."
