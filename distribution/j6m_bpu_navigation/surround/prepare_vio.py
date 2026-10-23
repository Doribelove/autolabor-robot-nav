"""Derive a private RX4 -> PYM NV12 -> GDC candidate from supplied vendor JSON.

This writes files only. It never starts cameras or asserts hardware acceptance.
"""
import argparse
import copy
from pathlib import Path

from .contracts import read_json, atomic_json, sha256


def prepare(vendor_vpm, vendor_camera, calibration_file, output):
    vpm, camera, cal = map(read_json, (vendor_vpm, vendor_camera, calibration_file))
    candidates = {}
    for pipeline in vpm.values():
        if not isinstance(pipeline, dict) or "vin_node0" not in pipeline or "pym_node0" not in pipeline:
            continue
        mipi = pipeline["vin_node0"]["cim"]["input"]["mipi"]
        if mipi["rx_index"] == 4 and mipi["format"] == 30:
            candidates[mipi["vc_index"]] = pipeline
    if set(candidates) != {0, 1, 2, 3}:
        raise ValueError("Vendor input must contain four RX4 YUV422 -> PYM pipelines")
    sizes = {(tuple(c["raw_size"]), tuple(c["rectified_size"])) for c in cal["cameras"].values()}
    if len(sizes) != 1:
        raise ValueError("This candidate generator expects the same sensor resolution for four cameras")
    raw, rect = next(iter(sizes))
    if any(x % 16 for x in (raw[0], rect[0])) or any(x % 2 for x in (*raw, *rect)):
        raise ValueError("GDC widths must be 16-aligned and heights even")
    result = {}
    for vc in range(4):
        p = copy.deepcopy(candidates[vc])
        m = p["vin_node0"]["cim"]["input"]["mipi"]
        if (m["width"], m["height"]) != raw:
            raise ValueError("Raw calibration resolution differs from vendor sensor mode")
        # The generic 11V sample crops 1920x1536 to 1920x1280. Keep the full
        # calibrated input here so contact pixels and K do not silently shift.
        p["vin_node0"].pop("lpwm", None)
        pym = p["pym_node0"]
        pym["common"].update(output_buf_num=4, layer_num_trans_next=0, time_out=500)
        chn = pym["chn"]
        chn.update(src_in_width=raw[0], src_in_height=raw[1], src_in_stride_y=raw[0],
                   src_in_stride_uv=raw[0], ds_roi_en=1, ds_roi_uv_bypass=0)
        chn["ds_roi"][0].update(ds_roi_layer=0, ds_roi_sel=0, ds_roi_start_top=0,
                               ds_roi_start_left=0, ds_roi_region_width=raw[0],
                               ds_roi_region_height=raw[1], ds_roi_wstride_y=raw[0],
                               ds_roi_wstride_uv=raw[0], ds_roi_out_width=raw[0],
                               ds_roi_out_height=raw[1])
        p["gdc_node0"] = dict(in_buf_noclean=0, in_buf_noncached=0,
                              out_buf_noinvalid=0, out_buf_noncached=0,
                              input_width=raw[0], input_height=raw[1],
                              output_width=rect[0], output_height=rect[1],
                              buf_num=4, fb_buf_num=4)
        port = camera.get("port_%d" % vc, {})
        if port.get("sensor_name") != "isx031std" or port.get("deserial_port") != vc:
            raise ValueError("Unexpected supplied ISX031 port mapping")
        result["pipeline%d" % vc] = p
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output / "vpm_config.json", result)
    atomic_json(output / "hb_j6dev.json", camera)
    atomic_json(output / "candidate.json", dict(
        status="NOT_HARDWARE_VERIFIED", synchronization_enabled=False,
        source_sha256={str(p): sha256(p) for p in (vendor_vpm, vendor_camera, calibration_file)},
        corrections=["full 1920x1536 layer0 instead of inherited 1280-height crop",
                     "only layer0 enabled", "old absolute LPWM path removed",
                     "GDC perspective node added; calibrated bins still required"],
        note="Verify packed input, NV12 output, slot allocation, clock and port orientation on the selected image."))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--vendor-vpm", required=True)
    p.add_argument("--vendor-camera", required=True)
    p.add_argument("--calibration", required=True)
    p.add_argument("--output", required=True)
    a = p.parse_args()
    prepare(a.vendor_vpm, a.vendor_camera, a.calibration, a.output)
