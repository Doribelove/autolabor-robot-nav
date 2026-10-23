// Private C ABI over the vendor VIO API. No shell scripts, remounts or killall.
// Compile against the headers belonging to the selected camera image.
#include <dlfcn.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include "hb_vio_interface.h"
#include "hb_mem_mgr.h"

namespace {
thread_local std::string error;
template<class T> T symbol(void* handle, const char* name) {
    auto p = reinterpret_cast<T>(dlsym(handle, name));
    if (!p) throw std::runtime_error(std::string("Missing vendor symbol: ") + name);
    return p;
}
struct Api {
    void* vio = nullptr;
    void* mem = nullptr;
#define API(name) decltype(&::name) name = nullptr
    API(hb_vio_init); API(hb_vio_deinit);
    API(hb_cam_init); API(hb_cam_deinit); API(hb_cam_start); API(hb_cam_stop);
    API(hb_vio_start_pipeline); API(hb_vio_stop_pipeline);
    API(hb_cam_get_data); API(hb_cam_free_data);
    API(hb_vio_get_data); API(hb_vio_free_pymbuf);
    API(hb_vio_set_gdc_cfg); API(hb_vio_run_gdc); API(hb_vio_free_gdcbuf);
    API(hb_mem_invalidate_buf_with_vaddr);
#undef API
    bool vio_ready = false, camera_ready = false;
    std::vector<unsigned> started, pipelines;
    bool gdc[64] = {};
    std::mutex gdc_mutex;
    Api(const char* vl, const char* ml) {
        vio = dlopen(vl, RTLD_NOW | RTLD_LOCAL);
        if (!vio) throw std::runtime_error(dlerror());
        mem = dlopen(ml, RTLD_NOW | RTLD_LOCAL);
        if (!mem) { std::string e = dlerror(); dlclose(vio); throw std::runtime_error(e); }
        try {
#define LOAD(name) name = symbol<decltype(name)>(vio, #name)
            LOAD(hb_vio_init); LOAD(hb_vio_deinit); LOAD(hb_cam_init); LOAD(hb_cam_deinit);
            LOAD(hb_cam_start); LOAD(hb_cam_stop); LOAD(hb_vio_start_pipeline);
            LOAD(hb_vio_stop_pipeline); LOAD(hb_cam_get_data); LOAD(hb_cam_free_data);
            LOAD(hb_vio_get_data); LOAD(hb_vio_free_pymbuf); LOAD(hb_vio_set_gdc_cfg);
            LOAD(hb_vio_run_gdc); LOAD(hb_vio_free_gdcbuf);
#undef LOAD
            hb_mem_invalidate_buf_with_vaddr =
                symbol<decltype(hb_mem_invalidate_buf_with_vaddr)>(mem, "hb_mem_invalidate_buf_with_vaddr");
        } catch (...) { dlclose(mem); dlclose(vio); mem = vio = nullptr; throw; }
    }
    ~Api() {
        for (auto i = started.rbegin(); i != started.rend(); ++i) hb_cam_stop(*i);
        for (auto i = pipelines.rbegin(); i != pipelines.rend(); ++i) hb_vio_stop_pipeline(*i);
        if (camera_ready) hb_cam_deinit(0);
        if (vio_ready) hb_vio_deinit();
        if (mem) dlclose(mem);
        if (vio) dlclose(vio);
    }
};
void checked(int ret, const char* operation) {
    if (ret < 0) throw std::runtime_error(std::string(operation) + ": " + std::to_string(ret));
}
// SDK buffers must be returned on every error path, including format mismatch.
struct Lease {
    Api* a; unsigned pipe; int kind; void* value;
    ~Lease() {
        if (kind == 0) a->hb_cam_free_data(pipe, HB_CAM_YUV_DATA, value);
        if (kind == 1) a->hb_vio_free_pymbuf(pipe, HB_VIO_PYM_DATA_V3, value);
        if (kind == 2) a->hb_vio_free_gdcbuf(pipe, static_cast<hb_vio_buffer_t*>(value));
    }
};
}

extern "C" {
struct sc_meta {
    uint64_t raw_stamp;
    uint64_t hal_received_ns;
    uint32_t sequence, width, height, bytes;
    int32_t vendor_format;
    uint32_t gdc_applied;
};
const char* sc_error() { return error.c_str(); }
unsigned sc_abi_version() { return 1; }
void* sc_open(const char* vpm, const char* camera, const char* vio_lib, const char* mem_lib) {
    try {
        auto a = std::unique_ptr<Api>(new Api(vio_lib, mem_lib));
        checked(a->hb_vio_init(vpm), "hb_vio_init"); a->vio_ready = true;
        checked(a->hb_cam_init(0, camera), "hb_cam_init"); a->camera_ready = true;
        return a.release();
    } catch (const std::exception& e) { error = e.what(); return nullptr; }
}
int sc_set_gdc(void* ctx, unsigned pipe, const void* bytes, size_t length) {
    try {
        if (!ctx || pipe >= 64 || !bytes || !length || length % 4)
            throw std::runtime_error("Invalid GDC buffer");
        auto* a = static_cast<Api*>(ctx);
        if (!a->started.empty()) throw std::runtime_error("Configure GDC before starting cameras");
        std::vector<uint32_t> aligned(length / 4);
        memcpy(aligned.data(), bytes, length);
        checked(a->hb_vio_set_gdc_cfg(pipe, aligned.data(), length), "hb_vio_set_gdc_cfg");
        a->gdc[pipe] = true;
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
int sc_start(void* ctx, unsigned pipe) {
    try {
        if (!ctx || pipe >= 64) throw std::runtime_error("Invalid camera pipeline");
        auto* a = static_cast<Api*>(ctx);
        if (std::find(a->pipelines.begin(), a->pipelines.end(), pipe) != a->pipelines.end())
            throw std::runtime_error("Duplicate camera pipeline");
        checked(a->hb_vio_start_pipeline(pipe), "hb_vio_start_pipeline");
        a->pipelines.push_back(pipe);
        checked(a->hb_cam_start(pipe), "hb_cam_start");
        a->started.push_back(pipe);
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
// kind=0: CIM packed YUV422, kind=1: PYM NV12, kind=2: CIM NV16.
// Optional GDC consumes PYM.
// Copy into caller memory before returning the SDK lease; no dangling pointers.
int sc_read(void* ctx, unsigned pipe, int kind, unsigned layer, uint8_t* out,
            size_t capacity, sc_meta* meta) {
    try {
        if (!ctx || pipe >= 64 || kind < 0 || kind > 2 || layer >= 6 || !out || !meta)
            throw std::runtime_error("Invalid read arguments");
        auto* a = static_cast<Api*>(ctx);
        hb_vio_buffer_t cim = {}, gdc = {};
        pym_buffer_v3_t pym = {};
        void* value = kind != 1 ? static_cast<void*>(&cim) : static_cast<void*>(&pym);
        int ret = kind != 1 ? a->hb_cam_get_data(pipe, HB_CAM_YUV_DATA, value) :
                              a->hb_vio_get_data(pipe, HB_VIO_PYM_DATA_V3, value);
        checked(ret, "get camera frame");
        Lease input{a, pipe, kind == 1 ? 1 : 0, value};
        image_info_t info = kind != 1 ? cim.img_info : pym.pym_img_info;
        address_info_t addr = kind != 1 ? cim.img_addr : pym.ds_out[layer];
        std::unique_lock<std::mutex> gdc_lock(a->gdc_mutex, std::defer_lock);
        std::unique_ptr<Lease> output;
        if (a->gdc[pipe]) {
            if (kind != 1) throw std::runtime_error("GDC requires verified NV12 PYM input");
            gdc_lock.lock(); // J6M has one GDC; serialize all four cameras.
            hb_vio_buffer_t source = {}; source.img_info = info; source.img_addr = addr;
            checked(a->hb_vio_run_gdc(pipe, &source, &gdc, 0), "hb_vio_run_gdc");
            output.reset(new Lease{a, pipe, 2, &gdc});
            addr = gdc.img_addr;
        }
        const size_t w = addr.width, h = addr.height, stride = addr.stride_size;
        const size_t row = kind == 0 ? w * 2 : w;
        const size_t size = kind == 1 ? w * h * 3 / 2 : w * h * 2;
        if (!w || !h || w > 4096 || h > 4096 || (w % 2) || (h % 2)
                || stride < row || capacity < size || !addr.addr[0]
                || (kind != 0 && !addr.addr[1]))
            throw std::runtime_error("Unexpected camera shape, stride, planes or capacity");
        checked(a->hb_mem_invalidate_buf_with_vaddr(
                    reinterpret_cast<uint64_t>(addr.addr[0]), stride * h), "invalidate Y");
        for (size_t y = 0; y < h; ++y)
            memcpy(out + y * row, addr.addr[0] + y * stride, row);
        if (kind != 0) {
            const size_t uv_rows = kind == 1 ? h / 2 : h;
            checked(a->hb_mem_invalidate_buf_with_vaddr(
                        reinterpret_cast<uint64_t>(addr.addr[1]), stride * uv_rows), "invalidate UV");
            for (size_t y = 0; y < uv_rows; ++y)
                memcpy(out + w * h + y * w, addr.addr[1] + y * stride, w);
        }
        // Preserve exposure identity from the input. Format belongs to the
        // returned image (GDC output when enabled), not the source CIM/PYM tag.
        const int32_t output_format = a->gdc[pipe] ? gdc.img_info.img_format : info.img_format;
        *meta = {info.time_stamp, uint64_t(info.tv.tv_sec) * 1000000000ULL
                   + uint64_t(info.tv.tv_usec) * 1000ULL, info.frame_id,
                 uint32_t(w), uint32_t(h), uint32_t(size), output_format,
                 uint32_t(a->gdc[pipe])};
        return 0;
    } catch (const std::exception& e) { error = e.what(); return -1; }
}
void sc_close(void* ctx) { delete static_cast<Api*>(ctx); }
}
