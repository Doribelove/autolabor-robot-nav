#!/usr/bin/env python3
"""Read HBRT/model metadata only. Never create a BPU instance or submit a task.

ctypes declarations follow the installed vendor hbrt4-c headers (Object,
Version, PreInit, HbmHeader). A header parse is NOT a model load/inference test.
"""
import argparse
import ctypes as c
import json
from pathlib import Path


class Object(c.Structure):
    _fields_ = [("ptr", c.c_size_t), ("opaque", c.c_size_t)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", required=True)
    parser.add_argument("--model")
    args = parser.parse_args()
    library = c.CDLL(str(Path(args.library).resolve()))
    output = {"library": str(Path(args.library).resolve()),
              "header_only": True, "inference_tested": False}

    def bind(name, types):
        fn = getattr(library, name)
        fn.argtypes = types
        fn.restype = c.c_int
        return fn

    def call(fn, *values):
        code = fn(*values)
        if code != 0:
            raise RuntimeError("{} returned {}".format(fn.__name__, code))

    version_get = bind("hbrt4GetToolkitVersion", [c.POINTER(Object)])
    version_text = bind("hbrt4VersionGetCString", [Object, c.POINTER(c.c_char_p)])

    def text_version(version):
        string = c.c_char_p()
        call(version_text, version, c.byref(string))
        return string.value.decode("utf-8")

    version = Object()
    call(version_get, c.byref(version))
    output["runtime_version"] = text_version(version)
    if not args.model:
        print(json.dumps(output, indent=2))
        return 0

    create_builder = bind("hbrt4PreInitBuilderCreate", [c.POINTER(Object)])
    into = bind("hbrt4PreInitBuilderInto", [c.POINTER(Object), c.POINTER(Object)])
    destroy_preinit = bind("hbrt4PreInitDestroy", [c.POINTER(Object)])
    create_header = bind("hbrt4HbmHeaderCreateByFilename2",
                         [Object, c.c_char_p, c.POINTER(Object)])
    destroy_header = bind("hbrt4HbmHeaderDestroy", [c.POINTER(Object)])
    get_version = bind("hbrt4HbmHeaderGetToolkitVersion", [Object, c.POINTER(Object)])
    get_march = bind("hbrt4HbmHeaderGetBpuMarch", [Object, c.POINTER(c.c_int)])
    builder, preinit, header = Object(), Object(), Object()
    try:
        call(create_builder, c.byref(builder))
        call(into, c.byref(builder), c.byref(preinit))
        call(create_header, preinit, str(Path(args.model).resolve()).encode(), c.byref(header))
        call(get_version, header, c.byref(version))
        output["model_toolkit_version"] = text_version(version)
        march = c.c_int()
        call(get_march, header, c.byref(march))
        output["model_march_enum"] = march.value
        output["model_is_nash_m"] = march.value == 5059394
    except Exception as error:
        output["error"] = str(error)
    finally:
        if header.ptr:
            call(destroy_header, c.byref(header))
        if preinit.ptr:
            call(destroy_preinit, c.byref(preinit))
    print(json.dumps(output, indent=2))
    return int("error" in output)


if __name__ == "__main__":
    raise SystemExit(main())
