#!/usr/bin/env python3
import argparse
import ctypes
import ctypes.util
import re
import subprocess
import time


class XWindowAttributes(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_int),
        ("y", ctypes.c_int),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("border_width", ctypes.c_int),
        ("depth", ctypes.c_int),
        ("visual", ctypes.c_void_p),
        ("root", ctypes.c_ulong),
        ("window_class", ctypes.c_int),
        ("bit_gravity", ctypes.c_int),
        ("win_gravity", ctypes.c_int),
        ("backing_store", ctypes.c_int),
        ("backing_planes", ctypes.c_ulong),
        ("backing_pixel", ctypes.c_ulong),
        ("save_under", ctypes.c_int),
        ("colormap", ctypes.c_ulong),
        ("map_installed", ctypes.c_int),
        ("map_state", ctypes.c_int),
        ("all_event_masks", ctypes.c_long),
        ("your_event_mask", ctypes.c_long),
        ("do_not_propagate_mask", ctypes.c_long),
        ("override_redirect", ctypes.c_int),
        ("screen", ctypes.c_void_p),
    ]


def find_window(ue=False):
    try:
        output = subprocess.check_output(
            ["xwininfo", "-root", "-tree"], text=True, stderr=subprocess.DEVNULL
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    title = "zsibot_mujoco_ue" if ue else "MuJoCo"
    match = re.search(
        rf'^\s*(0x[0-9a-fA-F]+)\s+"{title}[^\"]*"', output, re.MULTILINE
    )
    return int(match.group(1), 16) if match else None


def send_key(window, keysym):
    x11_path = ctypes.util.find_library("X11")
    xtst_path = ctypes.util.find_library("Xtst")
    if not x11_path or not xtst_path:
        raise RuntimeError("X11/XTest runtime libraries are unavailable")

    x11 = ctypes.CDLL(x11_path)
    xtst = ctypes.CDLL(xtst_path)
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
    x11.XStringToKeysym.restype = ctypes.c_ulong
    x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XKeysymToKeycode.restype = ctypes.c_uint
    x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    x11.XGetWindowAttributes.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(XWindowAttributes),
    ]
    x11.XGetWindowAttributes.restype = ctypes.c_int
    x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]

    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("cannot open the X11 display")
    try:
        attributes = XWindowAttributes()
        if not x11.XGetWindowAttributes(display, window, ctypes.byref(attributes)):
            raise RuntimeError("cannot inspect the MuJoCo window")
        if attributes.map_state != 2:
            raise RuntimeError("MuJoCo window is not viewable yet")
        symbol = x11.XStringToKeysym(keysym.encode("ascii"))
        keycode = x11.XKeysymToKeycode(display, symbol)
        if keycode == 0:
            raise RuntimeError(f"unknown X11 key: {keysym}")
        x11.XRaiseWindow(display, window)
        x11.XSetInputFocus(display, window, 2, 0)
        xtst.XTestFakeKeyEvent(display, keycode, 1, 0)
        xtst.XTestFakeKeyEvent(display, keycode, 0, 0)
        x11.XFlush(display)
    finally:
        x11.XCloseDisplay(display)


def raise_window(window):
    x11_path = ctypes.util.find_library("X11")
    if not x11_path:
        raise RuntimeError("X11 runtime library is unavailable")

    x11 = ctypes.CDLL(x11_path)
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XGetWindowAttributes.argtypes = [
        ctypes.c_void_p,
        ctypes.c_ulong,
        ctypes.POINTER(XWindowAttributes),
    ]
    x11.XGetWindowAttributes.restype = ctypes.c_int
    x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]

    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError("cannot open the X11 display")
    try:
        attributes = XWindowAttributes()
        if not x11.XGetWindowAttributes(display, window, ctypes.byref(attributes)):
            raise RuntimeError("cannot inspect the MuJoCo window")
        if attributes.map_state != 2:
            raise RuntimeError("MuJoCo window is not viewable yet")
        x11.XRaiseWindow(display, window)
        x11.XSetInputFocus(display, window, 2, 0)
        x11.XFlush(display)
    finally:
        x11.XCloseDisplay(display)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("key", nargs="?", default="space")
    parser.add_argument("--wait", type=float, default=0.0)
    parser.add_argument("--raise-only", action="store_true")
    parser.add_argument("--ue", action="store_true")
    args = parser.parse_args()

    deadline = time.monotonic() + max(0.0, args.wait)
    while True:
        window = find_window(ue=args.ue)
        if window is not None:
            try:
                if args.raise_only:
                    raise_window(window)
                else:
                    send_key(window, args.key)
            except RuntimeError:
                pass
            else:
                if args.raise_only:
                    window_name = "UE scene" if args.ue else "MuJoCo"
                    print(f"raised {window_name} window 0x{window:x}")
                else:
                    print(f"sent {args.key} to MuJoCo window 0x{window:x}")
                return
        if time.monotonic() >= deadline:
            window_name = "UE scene" if args.ue else "MuJoCo"
            raise SystemExit(f"{window_name} window was not found")
        time.sleep(0.02)


if __name__ == "__main__":
    main()
