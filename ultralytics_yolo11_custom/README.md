# Project-local YOLO11-GAM runtime

This directory is the NVIDIA runtime copy of the trusted training library at
`/home/slam/yolo11/yolo11_GAM/ultralytics`.  It was synchronized from that
working tree on 2026-08-28, including the local GAM/channel-handling changes
on top of source commit `679c3c1` (Ultralytics version `8.4.7`).  Generated
`__pycache__` and `.pyc/.pyo` files are deliberately not part of the copy.

The production detector prepends this directory to its private `PYTHONPATH`
and rejects startup unless `ultralytics.__file__` resolves inside the nested
`ultralytics/` package.  A pip-installed official Ultralytics distribution is
therefore neither selected nor accepted as a fallback.

The vendored Ultralytics source retains its upstream AGPL-3.0 license notices.
