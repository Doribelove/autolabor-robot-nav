#include <videocapture.hpp>
#include <sensorcapture.hpp>
#include <hidapi.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
using Clock = std::chrono::steady_clock;
}

int main(int argc, char **argv) {
    if (argc == 2 && std::string(argv[1]) == "--version") {
        std::cout << "{\"open_capture\":\"" << ZED_OC_MAJOR_VERSION << '.'
                  << ZED_OC_MINOR_VERSION << '.' << ZED_OC_PATCH_VERSION
                  << "\",\"hidapi\":\"" << hid_version_str()
                  << "\",\"cuda_required\":false}" << std::endl;
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--list") {
        sl_oc::sensors::SensorCapture sensors(sl_oc::VERBOSITY::ERROR);
        const auto serials = sensors.getDeviceList();
        std::cout << "{\"sensor_serials\":[";
        for (size_t i = 0; i < serials.size(); ++i)
            std::cout << (i ? "," : "") << serials[i];
        std::cout << "],\"capture_started\":false}" << std::endl;
        return 0;
    }
    if (argc != 5 || std::string(argv[1]) != "--capture") {
        std::cerr << "Usage: zed_capture_probe --version | --list | --capture VIDEO_ID SERIAL SECONDS\n";
        return 2;
    }
    int device = -1, serial = 0, seconds = 0;
    try {
        device = std::stoi(argv[2]); serial = std::stoi(argv[3]); seconds = std::stoi(argv[4]);
    } catch (const std::exception &) { return 2; }
    if (device < 0 || device > 63 || serial <= 0 || seconds < 1 || seconds > 60) return 2;
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    sl_oc::video::VideoParams parameters;
    parameters.res = sl_oc::video::RESOLUTION::HD720;
    parameters.fps = sl_oc::video::FPS::FPS_15;
    parameters.verbose = sl_oc::VERBOSITY::ERROR;
    // Keep the sensor object alive until the video object releases its sync pointer.
    sl_oc::sensors::SensorCapture sensors(sl_oc::VERBOSITY::ERROR);
    sl_oc::video::VideoCapture video(parameters);
    if (!video.initializeVideo(device)) {
        std::cerr << "{\"error\":\"video_open_failed\"}\n"; return 21;
    }
    if (video.getSerialNumber() != serial) {
        std::cerr << "{\"error\":\"camera_serial_mismatch\"}\n"; return 22;
    }
    if (!sensors.initializeSensors(serial) || !video.enableSensorSync(&sensors)) {
        std::cerr << "{\"error\":\"sensor_open_or_sync_failed\"}\n"; return 23;
    }
    std::atomic<bool> running{true};
    uint64_t imu_count = 0, imu_regressions = 0, imu_nonfinite = 0, last_imu_ns = 0;
    std::thread imu_thread([&] {
        while (running && !interrupted) {
            const auto imu = sensors.getLastIMUData(1000);
            if (imu.valid != sl_oc::sensors::data::Imu::NEW_VAL) continue;
            if (imu.timestamp <= last_imu_ns && last_imu_ns) ++imu_regressions;
            last_imu_ns = imu.timestamp;
            if (!(std::isfinite(imu.aX) && std::isfinite(imu.aY) && std::isfinite(imu.aZ) &&
                  std::isfinite(imu.gX) && std::isfinite(imu.gY) && std::isfinite(imu.gZ))) ++imu_nonfinite;
            ++imu_count;
        }
    });
    uint64_t frames = 0, last_frame_ns = 0, regressions = 0, invalid_frames = 0;
    uint64_t first_frame_ns = 0, first_id = 0, last_id = 0;
    unsigned width = 0, height = 0;
    const auto begin = Clock::now();
    auto last_arrival = begin;
    double max_gap_ms = 0;
    while (!interrupted && Clock::now() - begin < std::chrono::seconds(seconds)) {
        const auto frame = video.getLastFrame(100);
        if (!frame.data || frame.timestamp == 0 || frame.timestamp == last_frame_ns) continue;
        const auto now = Clock::now();
        if (frames) {
            max_gap_ms = std::max(max_gap_ms, std::chrono::duration<double, std::milli>(now-last_arrival).count());
            if (frame.timestamp < last_frame_ns) ++regressions;
        } else { first_frame_ns = frame.timestamp; first_id = frame.frame_id; }
        if (frame.width != 2560 || frame.height != 720 || frame.channels != 2) ++invalid_frames;
        last_frame_ns = frame.timestamp; last_id = frame.frame_id;
        width = frame.width; height = frame.height; last_arrival = now; ++frames;
    }
    running = false; imu_thread.join();
    const double elapsed = std::chrono::duration<double>(Clock::now()-begin).count();
    const bool ok = !interrupted && frames > 0 && imu_count > 0 && !regressions &&
                    !imu_regressions && !imu_nonfinite && !invalid_frames;
    std::cout << "{\"capture_ok\":" << (ok ? "true" : "false")
              << ",\"serial\":" << serial << ",\"seconds\":" << elapsed
              << ",\"frames\":" << frames << ",\"frame_rate_hz\":" << frames/elapsed
              << ",\"width_side_by_side\":" << width << ",\"height\":" << height
              << ",\"first_frame_id\":" << first_id << ",\"last_frame_id\":" << last_id
              << ",\"first_frame_stamp_ns\":" << first_frame_ns
              << ",\"last_frame_stamp_ns\":" << last_frame_ns
              << ",\"max_arrival_gap_ms\":" << max_gap_ms
              << ",\"image_timestamp_regressions\":" << regressions
              << ",\"invalid_frames\":" << invalid_frames
              << ",\"imu_samples\":" << imu_count << ",\"imu_rate_hz\":" << imu_count/elapsed
              << ",\"imu_timestamp_regressions\":" << imu_regressions
              << ",\"imu_nonfinite\":" << imu_nonfinite
              << ",\"depth_computed\":false,\"ros_published\":false}" << std::endl;
    return ok ? 0 : 24;
}
