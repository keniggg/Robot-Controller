#!/usr/bin/env python3
"""Bounded, header-only D405 usbmon capture. Does not load modules or use ROS.

Linux's binary usbmon read(2) compatibility interface returns the first 48
bytes of a usbmon_packet, never a transfer payload. We request exactly 48
bytes and retain only six common fields, not setup bytes or ISO descriptors.
Reference: https://docs.kernel.org/usb/usbmon.html (Raw binary format and API).
No claim is made that HCD requests are an independent physical USB bus trace.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import stat
import struct
import time

MOUNT = Path('/media/zhuyupei/SanStand')
EXTERNAL = MOUNT/'实验记录/Robot-Controller'
SYSFS = Path('/sys/bus/usb/devices')
MAX_SECONDS = 45.0
MAX_DIRECTORY_BYTES = 64*1024*1024
SUMMARY_RESERVE_BYTES = 64*1024
MAX_HEADERS = 500000
HEADER_BYTES = 48
# Native byte order, fixed-width standard sizes, no implicit struct padding.
# Offsets: id0, type8, xfer9, ep10, dev11, bus12, flags14/15,
# seconds16, usec24, status28, length32, captured_length36, setup40..47.
HEADER = struct.Struct('=QBBBBHbbqiiII8s')
assert HEADER.size == HEADER_BYTES
DOC_URL = 'https://docs.kernel.org/usb/usbmon.html'
D405_ID = ('8086', '0b5b')  # librealsense v2.56.5 src/ds/d400/d400-private.h RS405_PID


def parse_header(raw):
    if not isinstance(raw, bytes) or len(raw) != HEADER_BYTES:
        raise ValueError('usbmon_read_did_not_return_exactly_48_header_bytes')
    tag, event, xfer, ep, dev, bus, _, _, seconds, usec, status, length, _, _ = HEADER.unpack(raw)
    if event not in (ord('S'), ord('C'), ord('E')):
        raise ValueError('unsupported_usbmon_event_header')
    if xfer not in (0, 1, 2, 3) or not 1 <= bus <= 65535 or not 1 <= dev <= 127:
        raise ValueError('invalid_usbmon_transfer_address')
    if ep & 0x70 or not 0 <= usec < 1000000 or seconds < 0:
        raise ValueError('invalid_usbmon_endpoint_or_timestamp')
    address = '%s%s:%d:%03d:%d' % ('ZICB'[xfer], 'i' if ep & 0x80 else 'o', bus, dev, ep & 0x0f)
    return dict(tag=tag, timestamp_us=seconds*1000000+usec, event=chr(event),
                address=address, status=status, length=length, bus=bus, device=dev)


def header_line(header):
    # Only these six fields are persisted. No payload/setup/descriptor text.
    return ('%x %d %s %s %d %d\n' % (header['tag'], header['timestamp_us'],
        header['event'], header['address'], header['status'], header['length'])).encode('ascii')


def identity_at(path):
    path = Path(path)
    values = {key: (path/key).read_text(encoding='ascii').strip()
              for key in ('idVendor', 'idProduct', 'product', 'busnum', 'devnum', 'speed')}
    if (values['idVendor'].lower(), values['idProduct'].lower()) != D405_ID:
        raise ValueError('not_an_intel_d405_sysfs_device')
    int(values['idProduct'], 16)
    bus, dev = int(values['busnum']), int(values['devnum'])
    if not 1 <= bus <= 65535 or not 1 <= dev <= 127:
        raise ValueError('d405_sysfs_address_out_of_range')
    serial_path = path/'serial'
    return dict(sysfs_path=str(path.resolve(strict=True)), sysfs_inode=path.stat().st_ino,
                vendor=values['idVendor'].lower(), product_id=values['idProduct'].lower(),
                product=values['product'], serial=(serial_path.read_text(encoding='ascii').strip()
                    if serial_path.exists() else None), bus=bus, device=dev, speed=values['speed'])


def discover_d405(sysfs_root=SYSFS):
    found = []
    for path in Path(sysfs_root).iterdir():
        try:
            vendor=(path/'idVendor').read_text(encoding='ascii').strip().lower()
            product_id=(path/'idProduct').read_text(encoding='ascii').strip().lower()
        except (OSError, UnicodeError):
            continue
        if (vendor, product_id) == D405_ID:
            found.append(identity_at(path))
    if len(found) != 1:
        raise ValueError('exactly_one_sysfs_intel_d405_required: found=%d' % len(found))
    return found[0]


def validate_seconds(value):
    if isinstance(value, bool) or not math.isfinite(float(value)) or not 0 < float(value) <= MAX_SECONDS:
        raise ValueError('seconds_must_be_finite_positive_and_at_most_45')
    return float(value)


def external_output(path):
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand_is_not_mounted_no_fallback')
    output = Path(path).resolve()
    root = EXTERNAL.resolve(strict=True)
    if root not in output.parents or output.exists():
        raise ValueError('output_must_be_a_new_directory_strictly_under_SanStand_Robot_Controller')
    # Existing case directory only: never silently construct a replacement tree.
    if not output.parent.is_dir():
        raise ValueError('output_parent_case_directory_must_exist')
    output.mkdir(mode=0o755)
    return output


class HeaderBudget:
    def __init__(self, stream, *, max_bytes=MAX_DIRECTORY_BYTES-SUMMARY_RESERVE_BYTES,
                 max_lines=MAX_HEADERS):
        if type(max_bytes) is not int or max_bytes < 0 or type(max_lines) is not int or max_lines < 0:
            raise ValueError('invalid_header_budget')
        self.stream, self.max_bytes, self.max_lines = stream, max_bytes, max_lines
        self.bytes = self.lines = 0
        self.sha256 = hashlib.sha256()

    def write(self, header):
        line = header_line(header)
        if self.lines >= self.max_lines: return 'line_limit'
        if self.bytes + len(line) > self.max_bytes: return 'byte_limit'
        written = self.stream.write(line)
        if written != len(line): raise OSError('incomplete_header_output_write')
        self.sha256.update(line); self.bytes += len(line); self.lines += 1
        return None


def capture(output, seconds):
    seconds=validate_seconds(seconds)
    report=dict(schema=1, diagnostic_only=True, payload_requested=False, payload_recorded=False,
        setup_bytes_recorded=False, iso_descriptors_recorded=False, readonly_monitor=True,
        device_operations_sent=False, ros_operations_sent=False, module_loading_performed=False,
        binary_api='read(2) 48-byte usbmon compatibility header', binary_header_bytes=HEADER_BYTES,
        binary_native_byte_order=True, documentation=DOC_URL,
        columns=['urb_tag_hex','usbmon_timestamp_us','event','address','status','transfer_length'],
        timestamp_semantics='usbmon header timestamp seconds/useconds; raw header clock, not image exposure',
        requested_seconds=seconds, maximum_seconds=MAX_SECONDS,
        maximum_directory_bytes=MAX_DIRECTORY_BYTES, maximum_headers_read=MAX_HEADERS,
        identity_check_interval_sec=1.0, host_started_wall_ns=time.time_ns(),
        completed=False, source_closed=False, output_closed=False, error=None,
        limitations=['Host-controller request headers are not a physical USB bus capture.',
                    'No dropped-event statistics or payload/setup bytes are requested.',
                    'Device identity is rechecked at least once per second and finally; any change invalidates the run.',
                    'Deadline is cooperative: scheduling and filesystem close can add wall time; actual elapsed is reported.',
                    'SDK/ROS image source matching requires independent existing logs.'])
    fd=None; stream=None; budget=None; before=None; started=None; previous_handlers={};stop={'signal':None}
    counters=Counter();addresses=Counter(); first_stamp=None;last_stamp=None; reason=None
    def request_stop(signum, _frame): stop['signal']=signum
    try:
        before=discover_d405();report['device']=before
        node=Path('/dev/usbmon%d'%before['bus']);report['monitor_node']=str(node)
        fd=os.open(str(node),os.O_RDONLY|os.O_NONBLOCK|os.O_CLOEXEC)
        if not stat.S_ISCHR(os.fstat(fd).st_mode):raise ValueError('usbmon_monitor_is_not_character_device')
        if identity_at(before['sysfs_path']) != before:raise ValueError('device_changed_before_capture')
        stream=(output/'headers.txt').open('xb',buffering=65536);budget=HeaderBudget(stream)
        for signum in (signal.SIGINT,signal.SIGTERM):
            previous_handlers[signum]=signal.getsignal(signum);signal.signal(signum,request_stop)
        started=time.monotonic();deadline=started+seconds;next_identity=started
        report.update(capture_started_monotonic=started,capture_started_wall_ns=time.time_ns())
        while True:
            now=time.monotonic()
            if stop['signal'] is not None:reason='signal';break
            if now >= deadline:reason='duration_limit';break
            if counters['headers_read'] >= MAX_HEADERS:reason='header_limit';break
            if now >= next_identity:
                if not MOUNT.is_mount():raise RuntimeError('SanStand_unmounted_during_capture')
                if identity_at(before['sysfs_path']) != before:raise ValueError('d405_identity_changed')
                counters['identity_checks']+=1;next_identity=now+1.0
            readable,_,_=select.select([fd],[],[],min(.1,deadline-now))
            if not readable:continue
            # Do not fetch transfer data: read's fixed 48-byte API is deliberate.
            if time.monotonic() >= deadline:reason='duration_limit';break
            try:raw=os.read(fd,HEADER_BYTES)
            except BlockingIOError:continue
            if not raw:raise OSError('usbmon_stream_eof')
            header=parse_header(raw);counters['headers_read']+=1
            if header['bus']!=before['bus'] or header['device']!=before['device']:
                counters['other_device_headers_discarded']+=1;continue
            if time.monotonic() >= deadline:reason='duration_limit';break
            reason=budget.write(header)
            if reason is not None:break
            counters['target_'+header['event']]+=1;addresses[header['address']]+=1
            if header['event']=='C' and header['status']!=0:counters['nonzero_completion_status']+=1
            if last_stamp is not None and header['timestamp_us']<last_stamp:counters['timestamp_regressions']+=1
            if first_stamp is None:first_stamp=header['timestamp_us']
            last_stamp=header['timestamp_us']
        report['stop_reason']=reason
    except Exception as exc:
        report['error']=type(exc).__name__+': '+str(exc)[:2048]
    finally:
        report['signal']=stop['signal']
        if started is not None:report['capture_loop_elapsed_sec']=time.monotonic()-started
        if fd is not None:
            try:os.close(fd);report['source_closed']=True
            except OSError as exc:report['error']=report['error'] or repr(exc)
        if stream is not None:
            try:stream.close();report['output_closed']=True
            except OSError as exc:report['error']=report['error'] or repr(exc)
        for signum,handler in previous_handlers.items():signal.signal(signum,handler)
        if before is not None:
            try:
                report['final_device']=identity_at(before['sysfs_path'])
                if report['final_device']!=before:raise ValueError('d405_identity_changed_at_close')
                report['device_identity_unchanged']=True
            except Exception as exc:
                report['device_identity_unchanged']=False;report['error']=report['error'] or str(exc)
        report.update(counts=dict(counters),address_counts=dict(addresses),first_timestamp_us=first_stamp,
                      last_timestamp_us=last_stamp,host_finished_wall_ns=time.time_ns())
        if budget is not None:report.update(header_bytes=budget.bytes,header_lines=budget.lines,header_sha256=budget.sha256.hexdigest())
        report['completed']=bool(report['error'] is None and reason=='duration_limit' and report['source_closed']
            and report['output_closed'] and report.get('device_identity_unchanged') and budget is not None and budget.lines>0)
        report['tool_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
        if len(raw)>SUMMARY_RESERVE_BYTES:raise RuntimeError('summary_exceeds_reserved_budget')
        if not MOUNT.is_mount():raise RuntimeError('SanStand_unmounted_cannot_save_summary_no_fallback')
        with (output/'summary.json').open('xb') as summary:summary.write(raw)
        print(json.dumps(dict(output=str(output),completed=report['completed'],stop_reason=reason,
                              error=report['error'],headers=report.get('header_lines',0)),ensure_ascii=False),flush=True)
    return 0 if report['completed'] else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path,help='New subdirectory below an existing SanStand case')
    parser.add_argument('--seconds',type=float,default=45.)
    args=parser.parse_args();seconds=validate_seconds(args.seconds)
    output=external_output(args.output)
    return capture(output,seconds)


if __name__=='__main__':raise SystemExit(main())
