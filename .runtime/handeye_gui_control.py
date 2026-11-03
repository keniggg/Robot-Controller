"""Operate only observed handeye sampling buttons; no joint/gripper controls."""
import ctypes as C
import json,time,sys
from pathlib import Path
from PyQt5 import QtWidgets
D=C.c_void_p; U=C.c_ulong; I=C.c_int
x=C.CDLL('libX11.so.6')
x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=D
x.XDefaultRootWindow.argtypes=[D];x.XDefaultRootWindow.restype=U
x.XInternAtom.argtypes=[D,C.c_char_p,I];x.XInternAtom.restype=U
x.XSendEvent.argtypes=[D,U,I,C.c_long,D]
x.XFlush.argtypes=[D]
x.XTranslateCoordinates.argtypes=[D,U,U,I,I,C.POINTER(I),C.POINTER(I),C.POINTER(U)]
x.XFetchName.argtypes=[D,U,C.POINTER(C.c_char_p)]
x.XGetWindowProperty.argtypes=[D,U,U,C.c_long,C.c_long,I,U,C.POINTER(U),C.POINTER(I),C.POINTER(U),C.POINTER(U),C.POINTER(C.POINTER(C.c_ubyte))]
x.XFree.argtypes=[D]
class Data(C.Union):_fields_=[('b',C.c_char*20),('s',C.c_short*10),('l',C.c_long*5)]
class Client(C.Structure):_fields_=[('type',I),('serial',U),('send_event',I),('display',D),('window',U),('message_type',U),('format',I),('data',Data)]
class Event(C.Union):_fields_=[('client',Client),('pad',C.c_long*24)]
d=x.XOpenDisplay(None)
if not d:raise SystemExit('No X display')
wid=0x3400006;root=x.XDefaultRootWindow(d)
name=C.c_char_p();x.XFetchName(d,wid,C.byref(name))
if not name.value or 'Alicia-D' not in name.value.decode():raise SystemExit('Unexpected target window')
atom=x.XInternAtom(d,b'_NET_ACTIVE_WINDOW',0)
e=Event();e.client=Client(33,0,1,d,wid,atom,32,Data());e.client.data.l[0]=2
x.XSendEvent(d,root,0,(1<<20)|(1<<19),C.byref(e));x.XFlush(d);time.sleep(.5)
actual=U();fmt=I();count=U();remaining=U();prop=C.POINTER(C.c_ubyte)()
x.XGetWindowProperty(d,root,atom,0,1,0,0,C.byref(actual),C.byref(fmt),C.byref(count),C.byref(remaining),C.byref(prop))
active=C.cast(prop,C.POINTER(U))[0] if count.value else 0
if prop:x.XFree(prop)
if active!=wid:raise SystemExit('GUI did not become active: '+hex(active))
action=sys.argv[1] if len(sys.argv)>1 else 'view'
# Coordinates are in the visually inspected, 1320 x 860 client.
buttons={'refresh':(447,664),'algorithms':(484,617),'stable':(114,666),'sample':(281,666)}
if action in buttons:
 px,py=buttons[action];dx=I();dy=I();child=U()
 x.XTranslateCoordinates(d,wid,root,0,0,C.byref(dx),C.byref(dy),C.byref(child))
 xt=C.CDLL('libXtst.so.6')
 xt.XTestFakeMotionEvent.argtypes=[D,I,I,I,U]
 xt.XTestFakeButtonEvent.argtypes=[D,C.c_uint,I,U]
 xt.XTestFakeMotionEvent(d,-1,dx.value+px,dy.value+py,0)
 xt.XTestFakeButtonEvent(d,1,1,0);xt.XTestFakeButtonEvent(d,1,0,0);x.XFlush(d)
 time.sleep(2 if action=='stable' else .5)
app=QtWidgets.QApplication([])
out=Path(json.loads((Path(__file__).parent/'current_handeye_recollection.json').read_text())['session'])
file=out/('gui_'+action+'_'+str(time.time_ns())+'.png')
app.primaryScreen().grabWindow(wid).save(str(file))
print(file)
