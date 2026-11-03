#!/usr/bin/env python3
"""Prepare or explicitly execute a reload of the Alicia VS Code window only.

Default --inspect sends no GUI input. --prepare opens the command palette and
captures the selected reload command without pressing Enter. --execute requires
a recent preparation receipt; all screenshots and receipts stay on SanStand.
No process is killed and no ROS API is used.
"""
import argparse
import ctypes as C
import ctypes.util
import datetime
import hashlib
import json
import os
from pathlib import Path
import time


EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
DEFAULT_OUTPUT = EXTERNAL / 'ros_log/full_latest_restart_20261010T002347Z/codex_ui_analysis'
COMMAND = 'Developer: Reload Window'
COMMAND_ID = 'workbench.action.reloadWindow'


class XClientMessageEvent(C.Structure):
    class Data(C.Union):
        _fields_ = [('b', C.c_char * 20), ('s', C.c_short * 10), ('l', C.c_long * 5)]
    _fields_ = [('type', C.c_int), ('serial', C.c_ulong),
                ('send_event', C.c_int), ('display', C.c_void_p),
                ('window', C.c_ulong), ('message_type', C.c_ulong),
                ('format', C.c_int), ('data', Data)]


class XEvent(C.Union):
    _fields_ = [('client', XClientMessageEvent), ('pad', C.c_long * 24)]


class Desktop:
    def __init__(self):
        os.environ['DISPLAY'] = ':0'
        os.environ['XAUTHORITY'] = '/run/user/1000/gdm/Xauthority'
        self.x = C.CDLL(ctypes.util.find_library('X11'))
        self.xt = C.CDLL(ctypes.util.find_library('Xtst'))
        signatures = {
            'XOpenDisplay': ([C.c_char_p], C.c_void_p),
            'XDefaultRootWindow': ([C.c_void_p], C.c_ulong),
            'XInternAtom': ([C.c_void_p, C.c_char_p, C.c_int], C.c_ulong),
            'XGetWindowProperty': ([C.c_void_p, C.c_ulong, C.c_ulong, C.c_long,
                                   C.c_long, C.c_int, C.c_ulong,
                                   C.POINTER(C.c_ulong), C.POINTER(C.c_int),
                                   C.POINTER(C.c_ulong), C.POINTER(C.c_ulong),
                                   C.POINTER(C.POINTER(C.c_ubyte))], C.c_int),
            'XFree': ([C.c_void_p], C.c_int),
            'XSendEvent': ([C.c_void_p, C.c_ulong, C.c_int, C.c_long, C.POINTER(XEvent)], C.c_int),
            'XGetInputFocus': ([C.c_void_p, C.POINTER(C.c_ulong), C.POINTER(C.c_int)], C.c_int),
            'XQueryTree': ([C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong),
                            C.POINTER(C.c_ulong), C.POINTER(C.POINTER(C.c_ulong)),
                            C.POINTER(C.c_uint)], C.c_int),
            'XStringToKeysym': ([C.c_char_p], C.c_ulong),
            'XKeysymToKeycode': ([C.c_void_p, C.c_ulong], C.c_ubyte),
            'XFlush': ([C.c_void_p], C.c_int),
            'XCloseDisplay': ([C.c_void_p], C.c_int),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.x, name)
            fn.argtypes, fn.restype = args, result
        self.xt.XTestFakeKeyEvent.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_ulong]
        self.xt.XTestFakeKeyEvent.restype = C.c_int
        self.d = self.x.XOpenDisplay(b':0')
        if not self.d:
            raise RuntimeError('Cannot access the existing Ubuntu X11 desktop')
        self.root = self.x.XDefaultRootWindow(self.d)

    def atom(self, name):
        return self.x.XInternAtom(self.d, name.encode(), 0)

    def prop(self, window, name):
        actual_type, fmt, nitems, remaining = C.c_ulong(), C.c_int(), C.c_ulong(), C.c_ulong()
        data = C.POINTER(C.c_ubyte)()
        status = self.x.XGetWindowProperty(self.d, window, self.atom(name), 0, 16384,
                                          0, 0, C.byref(actual_type), C.byref(fmt),
                                          C.byref(nitems), C.byref(remaining), C.byref(data))
        if status != 0:
            raise RuntimeError('Could not read window property ' + name)
        try:
            if fmt.value == 32:
                return list(C.cast(data, C.POINTER(C.c_ulong))[:nitems.value])
            if fmt.value == 8:
                return C.string_at(data, nitems.value)
            return None
        finally:
            if data:
                self.x.XFree(data)

    def candidates(self):
        out = []
        for wid in self.prop(self.root, '_NET_CLIENT_LIST') or []:
            title = (self.prop(wid, '_NET_WM_NAME') or self.prop(wid, 'WM_NAME') or b'').decode('utf-8', 'replace')
            cls = (self.prop(wid, 'WM_CLASS') or b'').decode('utf-8', 'replace').split('\0')
            pids = self.prop(wid, '_NET_WM_PID') or []
            if 'code' not in [s.lower() for s in cls] or 'alicia_wa_full' not in title or len(pids) != 1:
                continue
            pid = pids[0]
            try:
                if Path('/proc/' + str(pid)).stat().st_uid != os.getuid():
                    continue
            except FileNotFoundError:
                continue
            out.append({'window_id': wid, 'window_id_hex': hex(wid), 'pid': pid,
                        'title': title, 'class': cls})
        return out

    def assert_focused(self, wid):
        active = self.prop(self.root, '_NET_ACTIVE_WINDOW') or []
        if active != [wid]:
            raise RuntimeError('Target Code window is not active; no keys sent')
        focused, revert = C.c_ulong(), C.c_int()
        self.x.XGetInputFocus(self.d, C.byref(focused), C.byref(revert))
        current = focused.value
        for _ in range(16):
            if current == wid:
                return
            if current in (0, 1, self.root):
                break
            root, parent, children, count = C.c_ulong(), C.c_ulong(), C.POINTER(C.c_ulong)(), C.c_uint()
            ok = self.x.XQueryTree(self.d, current, C.byref(root), C.byref(parent), C.byref(children), C.byref(count))
            if children:
                self.x.XFree(children)
            if not ok or parent.value == current:
                break
            current = parent.value
        raise RuntimeError('Keyboard focus does not belong to target Code window')

    def focus(self, wid):
        event = XEvent()
        event.client.type, event.client.send_event = 33, 1
        event.client.display, event.client.window = self.d, wid
        event.client.message_type, event.client.format = self.atom('_NET_ACTIVE_WINDOW'), 32
        event.client.data.l[0] = 2  # Existing desktop utility request.
        event.client.data.l[1] = 0
        self.x.XSendEvent(self.d, self.root, 0, (1 << 20) | (1 << 19), C.byref(event))
        self.x.XFlush(self.d)
        time.sleep(0.35)
        self.assert_focused(wid)

    def key(self, name, pressed):
        sym = self.x.XStringToKeysym(name.encode('ascii'))
        code = self.x.XKeysymToKeycode(self.d, sym)
        if not code:
            raise RuntimeError('Key is unavailable: ' + name)
        if not self.xt.XTestFakeKeyEvent(self.d, code, int(pressed), 0):
            raise RuntimeError('XTest key injection failed')

    def tap(self, name):
        self.key(name, True)
        self.key(name, False)

    def palette(self, wid):
        self.focus(wid)
        from gi.repository import Gio
        settings = Gio.Settings.new('org.gnome.desktop.input-sources')
        sources = [tuple(s) for s in settings.get_value('sources').unpack()]
        temporary_source = ('xkb', 'us')
        if set(sources) != {temporary_source, ('ibus', 'libpinyin')} or len(sources) != 2:
            raise RuntimeError('Expected the existing two GNOME English/Pinyin sources; no input sent')
        def current_source():
            recent = settings.get_value('mru-sources').unpack()
            if not recent or tuple(recent[0]) not in sources:
                raise RuntimeError('Cannot identify current GNOME input source')
            return tuple(recent[0])
        original_source = current_source()
        toggled = False
        def toggle():
            self.assert_focused(wid)
            self.key('Super_L', True)
            try:
                self.tap('space')
            finally:
                self.key('Super_L', False)
            self.x.XFlush(self.d)
            time.sleep(0.5)
            # GNOME writes the actual source order after the normal shortcut.
            from gi.repository import GLib
            context = GLib.MainContext.default()
            while context.pending():
                context.iteration(False)
        try:
            if original_source != temporary_source:
                toggle()
                toggled = True
            if current_source() != temporary_source:
                raise RuntimeError('English input method did not activate; no palette input sent')
            self.tap('Escape')
            self.tap('F1')
            self.x.XFlush(self.d)
            time.sleep(0.4)
            self.assert_focused(wid)
            self.key('Control_L', True)
            try:
                self.tap('a')
            finally:
                self.key('Control_L', False)
            # Include the prefix after Ctrl+A, since it may select the existing >.
            for ch in '>' + COMMAND:
                self.assert_focused(wid)
                shifted = ch.isupper() or ch in ':>'
                name = {' ': 'space', ':': 'semicolon', '>': 'period'}.get(ch, ch.lower())
                if shifted:
                    self.key('Shift_L', True)
                try:
                    self.tap(name)
                finally:
                    if shifted:
                        self.key('Shift_L', False)
                self.x.XFlush(self.d)
                time.sleep(0.025)
            self.x.XFlush(self.d)
            time.sleep(1.0)
        finally:
            if toggled:
                toggle()
            if current_source() != original_source:
                raise RuntimeError('Original input method could not be restored')
        return {'original': original_source, 'temporary': temporary_source,
                'restored': original_source, 'clipboard_untouched': True}

    def screenshot(self, wid, path):
        import gi
        gi.require_version('Gdk', '3.0')
        gi.require_version('GdkX11', '3.0')
        from gi.repository import Gdk, GdkX11
        display = Gdk.Display.open(':0')
        if display is None:
            raise RuntimeError('Cannot open GDK display for screenshot')
        win = GdkX11.X11Window.foreign_new_for_display(display, wid)
        if win is None:
            raise RuntimeError('Cannot capture target Code window')
        pixbuf = Gdk.pixbuf_get_from_window(win, 0, 0, win.get_width(), win.get_height())
        if pixbuf is None:
            raise RuntimeError('Code window screenshot is unavailable')
        pixbuf.savev(str(path), 'png', [], [])
        return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--inspect', action='store_true')
    mode.add_argument('--prepare', action='store_true')
    mode.add_argument('--execute', action='store_true')
    parser.add_argument('--window-id', type=lambda s: int(s, 0))
    parser.add_argument('--prepared-receipt', type=Path)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    if not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('SanStand is not mounted; refusing system-disk fallback')
    output = args.output_dir.resolve()
    if EXTERNAL.resolve() not in output.parents:
        raise RuntimeError('Output must remain under the SanStand Robot-Controller directory')
    output.mkdir(parents=True, exist_ok=True)
    desktop = Desktop()
    candidates = desktop.candidates()
    if args.window_id is not None:
        candidates = [c for c in candidates if c['window_id'] == args.window_id]
    if len(candidates) != 1:
        raise RuntimeError('Expected exactly one Alicia Code window: ' + json.dumps(candidates, ensure_ascii=False))
    target = candidates[0]
    wid = target['window_id']
    if args.execute:
        if args.prepared_receipt is None:
            raise RuntimeError('--execute requires a previous --prepared-receipt')
        prepared = json.loads(args.prepared_receipt.read_text())
        if (prepared.get('phase') != 'prepared_without_enter'
                or prepared.get('command_id') != COMMAND_ID
                or prepared.get('window') != target
                or not 0 <= time.time() - prepared['time_unix'] <= 600):
            raise RuntimeError('Preparation receipt does not match this window or is older than 10 minutes')
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    prefix = output / ('vscode_reload_' + stamp)
    receipt_path = Path(str(prefix) + '.json')
    receipt = {'time_unix': time.time(), 'time_utc': stamp, 'window': target,
               'command': COMMAND, 'command_id': COMMAND_ID,
               'phase': 'inspected_without_input', 'enter_sent': False,
               'screenshots': {}}
    def save():
        receipt_path.write_text(json.dumps(receipt, ensure_ascii=False, indent=2) + '\n')
    try:
        receipt['screenshots']['before'] = desktop.screenshot(wid, Path(str(prefix) + '_before.png'))
        save()
        if args.prepare or args.execute:
            receipt['input_method'] = desktop.palette(wid)
            receipt['screenshots']['prepared'] = desktop.screenshot(wid, Path(str(prefix) + '_prepared.png'))
            receipt['phase'] = 'prepared_without_enter'
            save()
        if args.execute:
            desktop.assert_focused(wid)
            # Persist the intent before the window reload can interrupt its tool host.
            receipt['phase'] = 'reload_submission_started'
            receipt['prepared_receipt'] = str(args.prepared_receipt)
            save()
            desktop.tap('Return')
            desktop.x.XFlush(desktop.d)
            receipt['enter_sent'], receipt['phase'] = True, 'reload_command_submitted'
            save()
            time.sleep(2.0)
            try:
                receipt['screenshots']['after'] = desktop.screenshot(wid, Path(str(prefix) + '_after.png'))
            except Exception as exc:
                receipt['after_screenshot_error'] = str(exc)
            save()
        print(json.dumps({'receipt': str(receipt_path), 'phase': receipt['phase'],
                          'window_id': hex(wid), 'enter_sent': receipt['enter_sent']}, ensure_ascii=False))
    except Exception as exc:
        receipt['error'] = str(exc)
        save()
        raise
    finally:
        desktop.x.XCloseDisplay(desktop.d)


if __name__ == '__main__':
    main()
