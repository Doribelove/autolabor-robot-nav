#!/usr/bin/python3
"""One 16x16 GLX pbuffer, local display only; no ROS or Gazebo import."""
import ctypes as C
import hashlib
import json
import os
from pathlib import Path
import time

root=Path(__file__).resolve().parent
captures=[]
def maps(label):
    payload=Path('/proc/self/maps').read_bytes();p=root/(label+'.maps.txt')
    with p.open('xb') as f:f.write(payload);f.flush();os.fsync(f.fileno())
    captures.append({'label':label,'path':str(p),'sha256':hashlib.sha256(payload).hexdigest(),
      'monotonic_ns':time.monotonic_ns(),'deleted_rows':[r for r in payload.decode().splitlines() if '(deleted)' in r]})

report={'pid':os.getpid(),'DISPLAY':os.environ.get('DISPLAY'),'scope':'single offscreen16x16 GLX pbuffer; no ROS/Gazebo','captures':captures}
dpy=ctx=configs=pbuffer=None
try:
    maps('00_before_libGL')
    x=C.CDLL('libX11.so.6');g=C.CDLL('libGL.so.1')
    ptr=C.c_void_p;integer=C.c_int;ulong=C.c_ulong
    x.XOpenDisplay.argtypes=[C.c_char_p];x.XOpenDisplay.restype=ptr
    x.XDefaultScreen.argtypes=[ptr];x.XDefaultScreen.restype=integer
    x.XFree.argtypes=[ptr];x.XFree.restype=integer
    x.XCloseDisplay.argtypes=[ptr];x.XCloseDisplay.restype=integer
    g.glXChooseFBConfig.argtypes=[ptr,integer,C.POINTER(integer),C.POINTER(integer)];g.glXChooseFBConfig.restype=C.POINTER(ptr)
    g.glXCreateNewContext.argtypes=[ptr,ptr,integer,ptr,integer];g.glXCreateNewContext.restype=ptr
    g.glXCreatePbuffer.argtypes=[ptr,ptr,C.POINTER(integer)];g.glXCreatePbuffer.restype=ulong
    g.glXMakeContextCurrent.argtypes=[ptr,ulong,ulong,ptr];g.glXMakeContextCurrent.restype=integer
    g.glXDestroyContext.argtypes=[ptr,ptr];g.glXDestroyContext.restype=None
    g.glXDestroyPbuffer.argtypes=[ptr,ulong];g.glXDestroyPbuffer.restype=None
    g.glViewport.argtypes=[integer,integer,integer,integer];g.glViewport.restype=None
    g.glClearColor.argtypes=[C.c_float]*4;g.glClearColor.restype=None
    g.glClear.argtypes=[C.c_uint];g.glClear.restype=None
    g.glFinish.argtypes=[];g.glFinish.restype=None
    g.glGetString.argtypes=[C.c_uint];g.glGetString.restype=C.c_char_p
    maps('01_after_libGL_load')
    dpy=x.XOpenDisplay(None)
    if not dpy:raise RuntimeError('XOpenDisplay failed')
    maps('02_after_XOpenDisplay')
    attributes=(integer*15)(0x8012,1,0x8010,4,0x8011,1,8,8,9,8,10,8,12,0,0)
    count=integer();configs=g.glXChooseFBConfig(dpy,x.XDefaultScreen(dpy),attributes,C.byref(count))
    if not configs or count.value<=0:raise RuntimeError('no Pbuffer FBConfig')
    maps('03_after_choose_FBConfig')
    ctx=g.glXCreateNewContext(dpy,configs[0],0x8014,None,1)
    if not ctx:raise RuntimeError('GLX context unavailable')
    maps('04_after_create_context')
    dimensions=(integer*5)(0x8041,16,0x8040,16,0)
    pbuffer=g.glXCreatePbuffer(dpy,configs[0],dimensions)
    if not pbuffer or not g.glXMakeContextCurrent(dpy,pbuffer,pbuffer,ctx):raise RuntimeError('Pbuffer activation failed')
    maps('05_after_make_current')
    report['GL']={label:g.glGetString(code).decode() for label,code in [('vendor',0x1f00),('renderer',0x1f01),('version',0x1f02)]}
    g.glViewport(0,0,16,16);g.glClearColor(.1,.2,.3,1.);g.glClear(0x4000);g.glFinish()
    maps('06_after_clear_finish')
    report['success']=True
except BaseException as error:
    report.update(success=False,error=type(error).__name__+': '+str(error))
    raise
finally:
    if dpy:
        if ctx:g.glXMakeContextCurrent(dpy,0,0,None);g.glXDestroyContext(dpy,ctx)
        if pbuffer:g.glXDestroyPbuffer(dpy,pbuffer)
        if configs:x.XFree(configs)
        x.XCloseDisplay(dpy)
    maps('07_after_cleanup')
    (root/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
