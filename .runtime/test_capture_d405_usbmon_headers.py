"""Pure synthetic header/filesystem tests; never opens USB, ROS or a monitor."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import signal
import stat
import struct
from types import SimpleNamespace

import pytest

spec=importlib.util.spec_from_file_location('usbmon_header_tool',Path(__file__).with_name('capture_d405_usbmon_headers.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def packet(*,event=b'C',xfer=3,ep=0x81,dev=2,bus=4,seconds=1791376179,usec=687000,status=0,length=921600):
    # Explicit documented offsets, independent from the parser's format string.
    b=bytearray(48)
    struct.pack_into('=Q',b,0,0xfedcba0123456789)
    b[8:12]=event+bytes([xfer,ep,dev]);struct.pack_into('=H',b,12,bus)
    struct.pack_into('=qiiII',b,16,seconds,usec,status,length,0xffffffff)
    b[40:48]=b'SECRETXX'
    return bytes(b)


def device(root,name='4-1',pid='0b5b',bus='4',dev='2'):
    p=root/name;p.mkdir()
    values=dict(idVendor='8086',idProduct=pid,product='Intel(R) RealSense(TM) Depth Camera 405',
                busnum=bus,devnum=dev,speed='5000',serial='example-serial')
    for k,v in values.items():(p/k).write_text(v+'\n')
    return p


def test_binary48_offsets_and_six_fields_never_include_setup_or_capture_length():
    h=m.parse_header(packet(status=-71,length=4294967295))
    assert h==dict(tag=0xfedcba0123456789,timestamp_us=1791376179687000,event='C',
                   address='Bi:4:002:1',status=-71,length=4294967295,bus=4,device=2)
    line=m.header_line(h)
    assert len(line.split())==6 and b'SECRET' not in line and line.endswith(b'-71 4294967295\n')


@pytest.mark.parametrize('xfer,ep,address',[(0,0x82,'Zi:4:002:2'),(1,3,'Io:4:002:3'),(2,0,'Co:4:002:0'),(3,0x81,'Bi:4:002:1')])
def test_all_transfer_types(xfer,ep,address):
    assert m.parse_header(packet(event=b'S',xfer=xfer,ep=ep))['address']==address


@pytest.mark.parametrize('raw',[b'',packet()[:-1],packet()+b'payload'])
def test_only_exact_header_length_accepted(raw):
    with pytest.raises(ValueError):m.parse_header(raw)


@pytest.mark.parametrize('kwargs',[dict(event=b'X'),dict(xfer=4),dict(ep=0x31),dict(bus=0),dict(dev=128),dict(usec=1000000),dict(seconds=-1)])
def test_invalid_headers_fail_closed(kwargs):
    with pytest.raises(ValueError):m.parse_header(packet(**kwargs))


def test_discovery_uses_d405_usb_id_not_product_spelling_and_rejects_multiple(tmp_path):
    p=device(tmp_path);device(tmp_path,'4-2',pid='0b5c',dev='3')
    got=m.discover_d405(tmp_path);assert got['sysfs_path']==str(p) and got['bus']==4 and got['device']==2
    device(tmp_path,'4-3',dev='4')
    with pytest.raises(ValueError,match='exactly_one'):m.discover_d405(tmp_path)


def test_device_address_reuse_or_change_is_visible(tmp_path):
    p=device(tmp_path);before=m.identity_at(p);(p/'devnum').write_text('3')
    assert m.identity_at(p)!=before
    (p/'idProduct').write_text('0b5c')
    with pytest.raises(ValueError,match='not_an_intel_d405'):m.identity_at(p)


def test_line_and_byte_limits_are_checked_before_write():
    h=m.parse_header(packet());line=m.header_line(h);stream=io.BytesIO()
    budget=m.HeaderBudget(stream,max_bytes=len(line),max_lines=2)
    assert budget.write(h) is None and budget.write(h)=='byte_limit'
    assert budget.bytes==len(line) and budget.lines==1 and stream.getvalue()==line
    assert budget.sha256.hexdigest()==hashlib.sha256(line).hexdigest()
    budget=m.HeaderBudget(io.BytesIO(),max_bytes=10000,max_lines=1)
    assert budget.write(h) is None and budget.write(h)=='line_limit'


def test_partial_output_write_is_failure():
    sink=SimpleNamespace(write=lambda data:len(data)-1)
    with pytest.raises(OSError,match='incomplete'):m.HeaderBudget(sink).write(m.parse_header(packet()))


@pytest.mark.parametrize('seconds',[0,-1,45.000001,float('inf'),float('nan'),True])
def test_duration_cannot_exceed_bound(seconds):
    with pytest.raises(ValueError):m.validate_seconds(seconds)


def test_external_output_has_no_unmounted_or_symlink_fallback(tmp_path,monkeypatch):
    mount=tmp_path/'disk';root=mount/'records';root.mkdir(parents=True)
    monkeypatch.setattr(m,'MOUNT',mount);monkeypatch.setattr(m,'EXTERNAL',root)
    monkeypatch.setattr(Path,'is_mount',lambda self:False)
    with pytest.raises(RuntimeError,match='no_fallback'):m.external_output(root/'new')
    monkeypatch.setattr(Path,'is_mount',lambda self:True)
    (root/'escape').symlink_to(tmp_path,target_is_directory=True)
    with pytest.raises(ValueError):m.external_output(root/'escape'/'new')
    got=m.external_output(root/'new');assert got.is_dir()
    with pytest.raises(ValueError):m.external_output(root/'new')


@pytest.mark.parametrize('failure',[False,True])
def test_fake_capture_reads_only48_closes_output_and_restores_handlers(tmp_path,monkeypatch,failure):
    ident=dict(sysfs_path='/synthetic/usb',bus=4,device=2)
    monkeypatch.setattr(m,'discover_d405',lambda:ident.copy())
    monkeypatch.setattr(m,'identity_at',lambda p:ident.copy())
    monkeypatch.setattr(Path,'is_mount',lambda self:True)
    mono=[0.]
    def clock():mono[0]+=.01;return mono[0]
    monkeypatch.setattr(m.time,'monotonic',clock)
    opened=[];closed=[];reads=[]
    def op(path,flags):
        assert path=='/dev/usbmon4' and flags&(m.os.O_WRONLY|m.os.O_RDWR)==0
        opened.append(path);return 54321
    def read(fd,n):
        assert fd==54321 and n==48;reads.append(n)
        if failure:raise OSError('synthetic read failure')
        return packet(dev=3 if len(reads)==1 else 2)
    monkeypatch.setattr(m.os,'open',op);monkeypatch.setattr(m.os,'read',read)
    monkeypatch.setattr(m.os,'close',lambda fd:closed.append(fd))
    monkeypatch.setattr(m.os,'fstat',lambda fd:SimpleNamespace(st_mode=stat.S_IFCHR))
    monkeypatch.setattr(m.select,'select',lambda r,w,e,timeout:(r,[],[]))
    handlers={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    code=m.capture(tmp_path,.15)
    summary=json.loads((tmp_path/'summary.json').read_text())
    assert code==(2 if failure else 0)
    assert opened==['/dev/usbmon4'] and closed==[54321] and reads
    assert summary['source_closed'] and summary['output_closed'] and not summary['payload_requested']
    assert all(signal.getsignal(s)==h for s,h in handlers.items())
    lines=(tmp_path/'headers.txt').read_bytes()
    assert b'SECRET' not in lines and all(len(line.split())==6 for line in lines.splitlines())
    assert all(b':003:' not in line for line in lines.splitlines())
    assert hashlib.sha256(lines).hexdigest()==summary['header_sha256']
    if not failure:assert summary['counts']['other_device_headers_discarded']==1
