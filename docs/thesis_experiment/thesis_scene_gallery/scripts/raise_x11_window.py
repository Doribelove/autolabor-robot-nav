#!/usr/bin/env python3
"""Raise an X11 window before a deterministic viewport capture."""

import ctypes
import sys


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: raise_x11_window.py WINDOW_ID")

    window_id = int(sys.argv[1], 0)
    x11 = ctypes.cdll.LoadLibrary("libX11.so.6")
    x11.XOpenDisplay.restype = ctypes.c_void_p
    display = x11.XOpenDisplay(None)
    if not display:
        raise SystemExit("cannot open X11 display")

    try:
        x11.XRaiseWindow(ctypes.c_void_p(display), ctypes.c_ulong(window_id))
        x11.XSetInputFocus(
            ctypes.c_void_p(display), ctypes.c_ulong(window_id), 1, 0
        )
        x11.XFlush(ctypes.c_void_p(display))
    finally:
        x11.XCloseDisplay(ctypes.c_void_p(display))


if __name__ == "__main__":
    main()
