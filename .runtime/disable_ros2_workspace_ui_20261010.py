"""Use the visible Alicia Code extension UI; keep receipts on SanStand."""
import argparse
import ctypes as C
import datetime
import importlib.util
import json
from pathlib import Path
import time

spec = importlib.util.spec_from_file_location('desktop_helper', Path(__file__).with_name('reload_vscode_window_20261009.py'))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)
parser = argparse.ArgumentParser()
parser.add_argument('action', choices=('type_query', 'click', 'capture'))
parser.add_argument('--x', type=int)
parser.add_argument('--y', type=int)
args = parser.parse_args()
out = helper.DEFAULT_OUTPUT / 'ros2_workspace_disable'
assert Path('/media/zhuyupei/SanStand').is_mount()
out.mkdir(exist_ok=True)
d = helper.Desktop()
targets = d.candidates()
assert len(targets) == 1, targets
target = targets[0]
wid = target['window_id']
stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
receipt = dict(action=args.action, window=target, started_ns=time.time_ns(), ros_calls=False)

def click_at(x, y):
    d.assert_focused(wid)
    d.x.XTranslateCoordinates.argtypes = [C.c_void_p, C.c_ulong, C.c_ulong, C.c_int, C.c_int, C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_ulong)]
    d.x.XTranslateCoordinates.restype = C.c_int
    rx, ry, child = C.c_int(), C.c_int(), C.c_ulong()
    assert d.x.XTranslateCoordinates(d.d, wid, d.root, x, y, C.byref(rx), C.byref(ry), C.byref(child))
    d.xt.XTestFakeMotionEvent.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int, C.c_ulong]
    d.xt.XTestFakeButtonEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
    d.xt.XTestFakeMotionEvent(d.d, -1, rx.value, ry.value, 0)
    d.xt.XTestFakeButtonEvent(d.d, 1, 1, 0)
    d.xt.XTestFakeButtonEvent(d.d, 1, 0, 0)
    d.x.XFlush(d.d)
    time.sleep(.2)

def toggle_source(settings):
    from gi.repository import GLib
    d.key('Super_L', True)
    try:
        d.tap('space')
    finally:
        d.key('Super_L', False)
    d.x.XFlush(d.d)
    time.sleep(.5)
    context = GLib.MainContext.default()
    while context.pending():
        context.iteration(False)

try:
    receipt['before'] = d.screenshot(wid, out / (stamp + '_before.png'))
    if args.action == 'type_query':
        assert args.x is not None and args.y is not None
        d.focus(wid)
        from gi.repository import Gio
        settings = Gio.Settings.new('org.gnome.desktop.input-sources')
        original = tuple(settings.get_value('mru-sources').unpack()[0])
        assert original in (('xkb', 'us'), ('ibus', 'libpinyin'))
        switched = original != ('xkb', 'us')
        try:
            if switched:
                toggle_source(settings)
            assert tuple(settings.get_value('mru-sources').unpack()[0]) == ('xkb', 'us')
            click_at(args.x, args.y)
            d.assert_focused(wid)
            d.key('Control_L', True)
            d.tap('a')
            d.key('Control_L', False)
            for ch in '@id:ranch-hand-robotics.rde-ros-2':
                shift = ch in '@:'
                name = {'@': '2', ':': 'semicolon', '-': 'minus', '.': 'period'}.get(ch, ch)
                if shift:
                    d.key('Shift_L', True)
                try:
                    d.tap(name)
                finally:
                    if shift:
                        d.key('Shift_L', False)
                d.x.XFlush(d.d)
                time.sleep(.035)
            d.x.XFlush(d.d)
            time.sleep(.7)
        finally:
            if switched:
                toggle_source(settings)
        assert tuple(settings.get_value('mru-sources').unpack()[0]) == original
        time.sleep(1.5)
    elif args.action == 'click':
        assert args.x is not None and args.y is not None
        d.focus(wid)
        click_at(args.x, args.y)
        receipt['clicked_window_xy'] = [args.x, args.y]
        time.sleep(1)
    receipt['after'] = d.screenshot(wid, out / (stamp + '_after.png'))
finally:
    (out / (stamp + '.json')).write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    d.x.XCloseDisplay(d.d)
print(json.dumps(receipt, ensure_ascii=False))
