#!/usr/bin/env python3
"""Exercise only the candidate's verified Qt window, without navigation input."""
import argparse
import ctypes as C
import subprocess


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('show', 'minimize'))
    parser.add_argument('window', type=lambda value: int(value, 0))
    parser.add_argument('pid', type=int)
    args = parser.parse_args()
    property_text = subprocess.check_output(['xprop', '-id', hex(args.window), '_NET_WM_PID'], text=True)
    if property_text.strip().split('=')[-1].strip() != str(args.pid):
        raise RuntimeError('Window process identity mismatch')
    command = subprocess.check_output(['ps', '-p', str(args.pid), '-o', 'args='], text=True)
    if not command.startswith('/home/slam/robot_j6m_ws_optimized_20260905/devel/lib/autolabor_operator_gui/autolabor_operator_gui_node '):
        raise RuntimeError('Not an owned candidate Qt process')
    x = C.CDLL('libX11.so.6')
    x.XOpenDisplay.restype = C.c_void_p
    x.XOpenDisplay.argtypes = [C.c_char_p]
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError('X display unavailable')
    x.XDefaultScreen.argtypes = [C.c_void_p]
    x.XDefaultRootWindow.argtypes = [C.c_void_p]
    x.XDefaultRootWindow.restype = C.c_ulong
    x.XIconifyWindow.argtypes = [C.c_void_p, C.c_ulong, C.c_int]
    x.XMapRaised.argtypes = [C.c_void_p, C.c_ulong]
    x.XInternAtom.argtypes = [C.c_void_p, C.c_char_p, C.c_int]
    x.XInternAtom.restype = C.c_ulong
    x.XSendEvent.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.c_void_p]
    x.XFlush.argtypes = [C.c_void_p]
    x.XCloseDisplay.argtypes = [C.c_void_p]
    if args.action == 'minimize':
        x.XIconifyWindow(display, args.window, x.XDefaultScreen(display))
    else:
        class Data(C.Union):
            _fields_ = [('b', C.c_char * 20), ('s', C.c_short * 10), ('l', C.c_long * 5)]
        class ClientMessage(C.Structure):
            _fields_ = [('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
                        ('display', C.c_void_p), ('window', C.c_ulong), ('message_type', C.c_ulong),
                        ('format', C.c_int), ('data', Data)]
        class Event(C.Union):
            _fields_ = [('client', ClientMessage), ('pad', C.c_long * 24)]
        event = Event()
        event.client.type, event.client.window, event.client.format = 33, args.window, 32
        event.client.display = display
        event.client.message_type = x.XInternAtom(display, b'_NET_ACTIVE_WINDOW', 0)
        event.client.data.l[0] = 1
        x.XMapRaised(display, args.window)
        x.XSendEvent(display, x.XDefaultRootWindow(display), 0, (1 << 19) | (1 << 20), C.byref(event))
    x.XFlush(display)
    x.XCloseDisplay(display)
    print(args.action, hex(args.window), args.pid)


if __name__ == '__main__':
    main()
