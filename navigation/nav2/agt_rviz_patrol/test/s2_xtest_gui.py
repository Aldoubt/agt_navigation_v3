"""X11-only interactive GUI acceptance helper for an isolated S2 RViz window.

Emits XTest pointer and keyboard events to the actual desktop, not ROS topics
or services. Run only with user permission, verified isolation and DISPLAY=:0.
Coordinates are root-screen pixels; no call touches chassis/motion APIs.
"""
import argparse
import ctypes
import time

x11 = ctypes.CDLL('libX11.so.6')
xtst = ctypes.CDLL('libXtst.so.6')
x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
x11.XStringToKeysym.restype = ctypes.c_ulong
x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
x11.XKeysymToKeycode.restype = ctypes.c_uint
x11.XFlush.argtypes = [ctypes.c_void_p]
x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
xtst.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
xtst.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]
xtst.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--window', type=lambda s: int(s, 0), required=True)
    p.add_argument('--display', default=':0')
    p.add_argument('event', choices=('focus', 'move', 'click', 'drag', 'key', 'scroll'))
    p.add_argument('args', nargs='*')
    a = p.parse_args()
    display = x11.XOpenDisplay(a.display.encode())
    if not display:
        raise SystemExit('no authorized X11 display')

    def flush():
        x11.XFlush(display)
        x11.XSync(display, False)

    def move(x, y):
        xtst.XTestFakeMotionEvent(display, -1, int(x), int(y), 0)
        flush()

    def button(number, press):
        xtst.XTestFakeButtonEvent(display, number, press, 0)
        flush()

    def key(name):
        sym = x11.XStringToKeysym(name.encode())
        if not sym:
            raise ValueError('no keysym: '+name)
        code = x11.XKeysymToKeycode(display, sym)
        if not code:
            raise ValueError('no keycode for '+name)
        xtst.XTestFakeKeyEvent(display, code, 1, 0)
        flush()
        time.sleep(.07)
        xtst.XTestFakeKeyEvent(display, code, 0, 0)
        flush()

    x11.XRaiseWindow(display, a.window)
    flush()
    time.sleep(.1)
    if a.event == 'focus':
        x11.XSetInputFocus(display, a.window, 1, 0)
        flush()
    elif a.event == 'move':
        move(*a.args)
    elif a.event == 'click':
        move(*a.args[:2]); time.sleep(.13); button(1, 1); time.sleep(.1); button(1, 0)
    elif a.event == 'drag':
        x, y, dx, dy = map(int, a.args)
        move(x, y); time.sleep(.12); button(1, 1)
        for n in range(1, 13):
            move(x + (dx-x)*n/12., y+(dy-y)*n/12.); time.sleep(.035)
        button(1, 0)
    elif a.event == 'key':
        key(a.args[0])
    elif a.event == 'scroll':
        move(*a.args[:2]); button(4 if int(a.args[2])>0 else 5, 1); button(4 if int(a.args[2])>0 else 5, 0)
    print('XTEST',a.event,*a.args,flush=True)

if __name__ == '__main__':
    main()
