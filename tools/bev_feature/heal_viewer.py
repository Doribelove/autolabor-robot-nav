#!/usr/bin/env python3
"""Separate diagnostic window; closing it ends only the HEAL preview worker."""
import json
from pathlib import Path
import sys
import time
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib

root = Path(sys.argv[1])
window = Gtk.Window(title='HEAL BEV-VTv2 · J6M BPU 实测预览（交通模型）')
window.set_default_size(1300, 820)
box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
label = Gtk.Label(label='正在加载BPU模型和本车四路图像……')
picture = Gtk.Image()
box.pack_start(label, False, False, 3); box.pack_start(picture, True, True, 0)
window.add(box); window.connect('destroy', Gtk.main_quit)
last = None


def refresh():
    global last
    try:
        status = json.loads((root/'latest.json').read_text())
        age = time.time()-status['updated_epoch_s']
        label.set_text('四路真实输入；两个空位已屏蔽；官方交通模型，尚未训练垃圾类别。'
                       ' BPU调用 %.1f ms | 更新 %.1f 秒前'%(
                           status['timing']['vendor_infer_ms'], age))
        path = root/'comparison.png'
        stamp = path.stat().st_mtime_ns
        size = window.get_size()
        key = stamp, size.width, size.height
        if key != last:
            pixbuf = GdkPixbuf.Pixbuf.new_from_file_at_scale(str(path), max(300, size.width-12),
                                                          max(200, size.height-55), True)
            picture.set_from_pixbuf(pixbuf); last = key
    except (FileNotFoundError, json.JSONDecodeError):
        pass
    return True


window.show_all(); GLib.timeout_add(500, refresh); Gtk.main()
