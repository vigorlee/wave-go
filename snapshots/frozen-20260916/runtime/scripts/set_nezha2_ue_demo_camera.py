#!/usr/bin/env python3
"""Enter MATRiX free camera and frame Nezha2 from behind for capture."""

from __future__ import annotations

import ctypes
import subprocess
import time


def window_id() -> int:
    output = subprocess.check_output(["wmctrl", "-lp"], text=True)
    for line in output.splitlines():
        if "zsibot_mujoco_ue" in line:
            return int(line.split()[0], 16)
    raise RuntimeError("MATRiX UE window was not found")


def main() -> None:
    window = window_id()
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
        raise RuntimeError("cannot open X display")
    x11.XRaiseWindow(display, window)
    x11.XSetInputFocus(display, window, 1, 0)

    def key(name: str, hold_seconds: float = 0.0) -> None:
        keycode = x11.XKeysymToKeycode(display, ord(name))
        xtst.XTestFakeKeyEvent(display, keycode, 1, 0)
        x11.XFlush(display)
        time.sleep(hold_seconds)
        xtst.XTestFakeKeyEvent(display, keycode, 0, 0)
        x11.XFlush(display)

    key("v")
    time.sleep(0.25)
    # Free-camera reverse motion moves the viewpoint out of the Nezha2 body;
    # it does not send velocity commands to the ROS/MATRiX navigation stack.
    key("s", hold_seconds=0.85)
    time.sleep(0.4)


if __name__ == "__main__":
    main()
