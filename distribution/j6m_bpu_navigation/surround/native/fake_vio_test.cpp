// Test-only SDK double. Never linked into the capture service.
#include <stdint.h>
#include <string.h>
#include "hb_vio_interface.h"
#include "hb_mem_mgr.h"
namespace {
int releases = 0, stops = 0, deinit = 0, fail_start = 0;
bool cim_nv16 = false;
bool fail_gdc = false, bad_gdc_shape = false, gdc_metadata = false;
char packed[24], y[12], uv[12];
void meta(image_info_t* i) {
    *i = {};
    i->time_stamp = 123456789;
    i->frame_id = 42;
    i->tv.tv_sec = 1700000000;
    i->tv.tv_usec = 123;
    i->img_format = 30;
}
void addr(address_info_t* a, bool nv12) {
    *a = {};
    a->width = 4; a->height = 2; a->stride_size = nv12 ? 6 : 12;
    a->addr[0] = nv12 ? y : packed;
    a->addr[1] = nv12 ? uv : nullptr;
}
}
extern "C" {
void fake_reset() {
    releases = stops = deinit = fail_start = 0;
    cim_nv16 = false;
    fail_gdc = bad_gdc_shape = gdc_metadata = false;
    memset(packed, 99, sizeof(packed));
    memset(y, 99, sizeof(y)); memset(uv, 99, sizeof(uv));
    for (int i = 0; i < 8; ++i) { packed[i] = i; packed[12 + i] = 8 + i; }
    for (int i = 0; i < 4; ++i) { y[i] = i; y[6 + i] = i + 4; uv[i] = uv[6 + i] = 128; }
}
int fake_releases() { return releases; }
int fake_stops() { return stops; }
int fake_deinit() { return deinit; }
void fake_fail_start() { fail_start = 1; }
void fake_cim_nv16() { cim_nv16 = true; }
void fake_fail_gdc() { fail_gdc = true; }
void fake_bad_gdc_shape() { bad_gdc_shape = true; }
void fake_gdc_metadata() { gdc_metadata = true; }
int32_t hb_vio_init(const char*) { return 0; }
int32_t hb_vio_deinit() { ++deinit; return 0; }
int32_t hb_cam_init(uint32_t, const char*) { return 0; }
int32_t hb_cam_deinit(uint32_t) { ++deinit; return 0; }
int32_t hb_cam_start(uint32_t) { return fail_start ? -77 : 0; }
int32_t hb_cam_stop(uint32_t) { ++stops; return 0; }
int32_t hb_vio_start_pipeline(uint32_t) { return 0; }
int32_t hb_vio_stop_pipeline(uint32_t) { ++stops; return 0; }
int32_t hb_cam_get_data(uint32_t, CAM_DATA_TYPE_E, void* value) {
    auto* b = static_cast<hb_vio_buffer_t*>(value);
    meta(&b->img_info); addr(&b->img_addr, cim_nv16); return 0;
}
int32_t hb_cam_free_data(uint32_t, CAM_DATA_TYPE_E, void*) { ++releases; return 0; }
int32_t hb_vio_get_data(uint32_t, VIO_DATA_TYPE_E, void* value) {
    auto* b = static_cast<pym_buffer_v3_t*>(value);
    *b = {}; meta(&b->pym_img_info); addr(&b->ds_out[0], true); return 0;
}
int32_t hb_vio_free_pymbuf(uint32_t, VIO_DATA_TYPE_E, void*) { ++releases; return 0; }
int32_t hb_vio_set_gdc_cfg(uint32_t, uint32_t*, uint64_t) { return 0; }
int32_t hb_vio_run_gdc(uint32_t, hb_vio_buffer_t* src, hb_vio_buffer_t* dst, int32_t) {
    if (fail_gdc) return -88;
    *dst = *src;
    if (bad_gdc_shape) dst->img_addr.width = 4098;
    if (gdc_metadata) {
        dst->img_info.img_format = 31;
        dst->img_info.time_stamp = 999;
        dst->img_info.frame_id = 999;
        dst->img_addr.width = 2;
    }
    return 0;
}
int32_t hb_vio_free_gdcbuf(uint32_t, hb_vio_buffer_t*) { ++releases; return 0; }
int32_t hb_mem_invalidate_buf_with_vaddr(uint64_t, uint64_t) { return 0; }
}
