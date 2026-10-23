// AGX-only ZED SDK acquisition. No inference, resize, crop or normalization.
#include <sl/Camera.hpp>
#include "json.hpp"
#include <lz4.h>
#include <zlib.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
std::atomic<bool> running{true};
void stop(int) { running = false; }
uint64_t unix_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }
void atomic_json(const std::string &name, const json &j) {
    { std::ofstream f(name + ".tmp"); f << j.dump(2) << '\n'; }
    if (rename((name + ".tmp").c_str(), name.c_str())) throw std::runtime_error("status rename");
}
sockaddr_in address(const std::string &ip, int port) {
    sockaddr_in a{}; a.sin_family=AF_INET; a.sin_port=htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &a.sin_addr)!=1) throw std::runtime_error("IPv4 required");
    return a;
}
bool send_all(int fd, const void *p, size_t n) {
    auto deadline=Clock::now()+std::chrono::milliseconds(200);
    auto b=static_cast<const char*>(p);
    while (n && running && Clock::now()<deadline) {
        auto v=send(fd,b,n,MSG_NOSIGNAL);
        if (v<=0) return false;
        b+=v; n-=v;
    }
    return n==0;
}
int connect_to(const json &c, const char *port_name="image_port") {
    int fd=socket(AF_INET,SOCK_STREAM,0), one=1;
    timeval tv{0,200000};
    setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv));
    setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
    setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
    auto local=address(c.at("agx_ip"),0), peer=address(c.at("j6m_ip"),c.at(port_name));
    if (bind(fd,(sockaddr*)&local,sizeof(local)) || connect(fd,(sockaddr*)&peer,sizeof(peer))) { close(fd);return -1; }
    return fd;
}
template<class T> json vec3(const T &v) { return {v.x,v.y,v.z}; }
template<class T> json matrix(const T &v, int n) { json a=json::array(); for(int i=0;i<n;++i) a.push_back(v.m[i]); return a; }
json matrix(const sl::Matrix3f &v, int n) { json a=json::array(); for(int i=0;i<n;++i) a.push_back(v.r[i]); return a; }
json intrinsics(const sl::CameraParameters &p) {
    json d=json::array();for(int i=0;i<12;i++) d.push_back(p.disto[i]);
    return {{"fx",p.fx},{"fy",p.fy},{"cx",p.cx},{"cy",p.cy},{"distortion",d}};
}
struct Frame { json header; std::vector<char> rgba,depth; Clock::time_point acquired; };
int main(int argc,char **argv) {
    if(argc!=2) { std::cerr<<"usage: zed_sender config.json\n";return 2; }
    json c; {std::ifstream f(argv[1]);f>>c;}
    std::string state=c.at("agx_state"), token=c.at("token");
    int lock=open((state+"/camera.lock").c_str(),O_CREAT|O_RDWR,0600);
    if(lock<0 || flock(lock,LOCK_EX|LOCK_NB)) return 3;
    signal(SIGTERM,stop);signal(SIGINT,stop);signal(SIGPIPE,SIG_IGN);
    sl::Camera camera;sl::InitParameters init;
    init.input.setFromSerialNumber(c.at("serial").get<unsigned int>());
    init.camera_resolution=sl::RESOLUTION::HD720;init.camera_fps=15;
    init.depth_mode=sl::DEPTH_MODE::QUALITY;init.coordinate_units=sl::UNIT::METER;
    init.coordinate_system=sl::COORDINATE_SYSTEM::IMAGE;init.depth_stabilization=0;
    init.sensors_required=true;init.sdk_verbose=0;
    init.optional_settings_path=c.at("settings_dir").get<std::string>().c_str();
    auto code=camera.open(init);
    if(code!=sl::ERROR_CODE::SUCCESS) {
        atomic_json(state+"/sender.json",{{"state","camera_open_failed"},{"error",sl::toString(code).c_str()},{"at_ns",unix_ns()}});return 4;
    }
    auto info=camera.getCameraInformation(); auto config=info.camera_configuration;
    if(info.serial_number!=c.at("serial").get<unsigned int>() || config.resolution.width!=1280 || config.resolution.height!=720) return 5;
    auto session=std::to_string(unix_ns())+"-"+std::to_string(getpid());
    json calibration={{"sdk",sl::Camera::getSDKVersion().c_str()},{"serial",info.serial_number},
      {"width",1280},{"height",720},{"fps",config.fps},{"color_encoding","bgra8"},
      {"color_source","SDK VIEW::LEFT rectified at full HD720 acquisition resolution"},
      {"depth_encoding","32FC1"},{"depth_units","m"},{"depth_definition","optical Z aligned to left rectified image"},
      {"coordinate_system","IMAGE: x right, y down, z forward"},
      {"left_optical_frame","zed2_left_camera_optical_frame"},{"imu_frame","zed2_imu_sdk_image_frame"},
      {"imu_protocol",2},{"imu_axes","SDK IMAGE convention; not the legacy ROS Z_UP_X_FWD imu_link"},
      {"left_rectified",intrinsics(config.calibration_parameters.left_cam)},
      {"right_rectified",intrinsics(config.calibration_parameters.right_cam)},
      {"left_raw",intrinsics(config.calibration_parameters_raw.left_cam)},
      {"stereo_transform_row_major",matrix(config.calibration_parameters.stereo_transform,16)},
      {"T_left_from_imu_row_major",matrix(info.sensors_configuration.camera_imu_transform,16)},
      {"imu_native_rate_hz",info.sensors_configuration.gyroscope_parameters.sampling_rate},
      {"robot_mount",c.at("robot_mount")},{"positional_tracking",false},{"depth_stabilization",0},
      {"clock_domain","AGX UNIX nanoseconds from ZED SDK; original acquisition timestamps"}};
    atomic_json(state+"/calibration.json",calibration);
    std::mutex mutex; std::condition_variable changed; std::shared_ptr<Frame> latest;
    std::atomic<uint64_t> captured{0},sent{0},overwritten{0},network_errors{0},wire_bytes{0},imu_unique{0},imu_sent{0},imu_errors{0};
    std::atomic<uint64_t> last_sent_seq{0},last_imu_ns{0};
    std::atomic<int64_t> imu_sdk_host_skew_ns{0};
    std::thread transmit([&]{
      int fd=-1;
      std::vector<char> packed(LZ4_compressBound(1280*720*4)*2);
      while(running) {
        std::shared_ptr<Frame> f;
        {std::unique_lock<std::mutex> l(mutex);changed.wait_for(l,std::chrono::milliseconds(200),[&]{return latest || !running;});f=std::move(latest);}
        if(!f || Clock::now()-f->acquired>std::chrono::milliseconds(300)) continue;
        if(fd<0) { fd=connect_to(c);if(fd<0){network_errors++;std::this_thread::sleep_for(std::chrono::milliseconds(500));continue;} }
        int nc=LZ4_compress_default(f->rgba.data(),packed.data(),f->rgba.size(),LZ4_compressBound(f->rgba.size()));
        int nd=LZ4_compress_default(f->depth.data(),packed.data()+nc,f->depth.size(),LZ4_compressBound(f->depth.size()));
        if(nc<=0 || nd<=0) { running=false;break; }
        f->header["color_lz4_bytes"]=nc;f->header["depth_lz4_bytes"]=nd;
        f->header["color_crc32"]=crc32(0,(const Bytef*)f->rgba.data(),f->rgba.size());
        f->header["depth_crc32"]=crc32(0,(const Bytef*)f->depth.data(),f->depth.size());
        f->header["send_ns"]=unix_ns();
        std::string h=f->header.dump();uint32_t sizes[2]={htonl(h.size()),htonl(nc+nd)};
        if(!send_all(fd,"ZB01",4) || !send_all(fd,sizes,8) || !send_all(fd,h.data(),h.size()) || !send_all(fd,packed.data(),nc+nd)) {
            close(fd);fd=-1;network_errors++;continue;
        }
        sent++;wire_bytes+=12+h.size()+nc+nd;last_sent_seq=f->header.at("sequence").get<uint64_t>();
      }
      if(fd>=0) close(fd);
    });
    std::thread imu([&]{
      int fd=-1;uint64_t previous=0,next_due=0,sequence=0;auto retry=Clock::now();
      while(running) {
        sl::SensorsData d;
        auto read_begin=unix_ns();auto sensor_code=camera.getSensorsData(d,sl::TIME_REFERENCE::CURRENT);auto read_end=unix_ns();
        if(sensor_code==sl::ERROR_CODE::SUCCESS && d.imu.is_available) {
          uint64_t ts=d.imu.timestamp.getNanoseconds();
          if(ts>previous) {
            previous=ts;imu_unique++;imu_sdk_host_skew_ns=(int64_t)ts-(int64_t)read_end;
            if(!next_due) next_due=ts;
            if(ts>=next_due) {
              do {next_due+=5000000;} while(next_due<=ts);
              auto q=d.imu.pose.getOrientation();
              json j={{"protocol",2},{"token",token},{"session",session},{"serial",info.serial_number},{"sequence",++sequence},
                {"stamp_ns",ts},{"host_read_begin_ns",read_begin},{"host_read_end_ns",read_end},
                {"timestamp_policy","original SDK timestamp unchanged; host read stamps are separate diagnostics"},
                {"frame_id","zed2_imu_sdk_image_frame"},{"acceleration_m_s2",vec3(d.imu.linear_acceleration)},
                {"angular_velocity_deg_s",vec3(d.imu.angular_velocity)},
                {"acceleration_uncalibrated_m_s2",vec3(d.imu.linear_acceleration_uncalibrated)},
                {"angular_velocity_uncalibrated_deg_s",vec3(d.imu.angular_velocity_uncalibrated)},
                {"orientation_xyzw",{q.x,q.y,q.z,q.w}},
                {"orientation_covariance",matrix(d.imu.pose_covariance,9)},
                {"angular_velocity_covariance_deg_s",matrix(d.imu.angular_velocity_covariance,9)},
                {"acceleration_covariance",matrix(d.imu.linear_acceleration_covariance,9)},
                {"native_effective_rate_hz",d.imu.effective_rate},{"image_sync_trigger",d.image_sync_trigger},
                {"sampling","200 Hz timestamp-grid selection of unique native samples; no interpolation"}};
              auto b=j.dump();uint32_t size=htonl(b.size());
              if(fd<0 && Clock::now()>=retry) {fd=connect_to(c,"imu_port");retry=Clock::now()+std::chrono::milliseconds(500);}
              if(fd>=0 && send_all(fd,&size,4) && send_all(fd,b.data(),b.size())) {imu_sent++;last_imu_ns=ts;}
              else {imu_errors++;if(fd>=0)close(fd);fd=-1;}
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
      if(fd>=0)close(fd);
    });
    sl::Mat image,depth;sl::RuntimeParameters runtime;runtime.enable_depth=true;
    uint64_t errors=0,consecutive=0;auto begin=Clock::now(),last_status=begin;
    while(running) {
      if(camera.grab(runtime)!=sl::ERROR_CODE::SUCCESS) {
        errors++;if(++consecutive>=30){running=false;break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(30));continue;
      }
      consecutive=0;auto acquired=Clock::now();uint64_t seq=++captured,stamp=camera.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
      if(camera.retrieveImage(image,sl::VIEW::LEFT,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS ||
         camera.retrieveMeasure(depth,sl::MEASURE::DEPTH,sl::MEM::CPU)!=sl::ERROR_CODE::SUCCESS) { errors++;continue; }
      auto f=std::make_shared<Frame>();f->acquired=acquired;f->rgba.resize(1280*720*4);f->depth.resize(1280*720*4);
      for(int y=0;y<720;y++) {
        memcpy(f->rgba.data()+y*5120,(char*)image.getPtr<sl::uchar1>(sl::MEM::CPU)+y*image.getStepBytes(sl::MEM::CPU),5120);
        memcpy(f->depth.data()+y*5120,(char*)depth.getPtr<float>(sl::MEM::CPU)+y*depth.getStepBytes(sl::MEM::CPU),5120);
      }
      f->header={{"protocol",1},{"token",token},{"session",session},{"sequence",seq},{"serial",info.serial_number},
        {"color_stamp_ns",stamp},{"depth_stamp_ns",depth.timestamp.getNanoseconds()},{"frame_id","zed2_left_camera_optical_frame"},
        {"width",1280},{"height",720},{"color_encoding","bgra8"},{"depth_encoding","32FC1"},{"depth_units","m"},
        {"color_raw_bytes",f->rgba.size()},{"depth_raw_bytes",f->depth.size()},{"codec","lz4-block"},{"byte_order","little"},
        {"calibration",calibration},{"production_ready",false},{"motion_eligible",false}};
      {std::lock_guard<std::mutex> l(mutex);if(latest)overwritten++;latest=f;}changed.notify_one();
      if(Clock::now()-last_status>std::chrono::seconds(1)) {
        double elapsed=std::chrono::duration<double>(Clock::now()-begin).count();
        atomic_json(state+"/sender.json",{{"state","capturing"},{"session",session},{"pid",getpid()},{"at_ns",unix_ns()},
          {"elapsed_s",elapsed},{"captured",captured.load()},{"sent",sent.load()},{"capture_hz",captured/elapsed},
          {"sent_hz",sent/elapsed},{"latest_slot_overwritten",overwritten.load()},{"network_errors",network_errors.load()},
          {"wire_bytes",wire_bytes.load()},{"wire_mbit_s",wire_bytes*8/elapsed/1e6},{"grab_errors",errors},
          {"imu_native_unique",imu_unique.load()},{"imu_sent",imu_sent.load()},{"imu_hz",imu_sent/elapsed},
          {"imu_send_errors",imu_errors.load()},{"last_imu_stamp_ns",last_imu_ns.load()},
          {"imu_sdk_minus_host_read_ms",imu_sdk_host_skew_ns.load()/1e6},{"last_sent_sequence",last_sent_seq.load()}});
        last_status=Clock::now();
      }
    }
    running=false;changed.notify_all();transmit.join();imu.join();camera.close();
    return errors ? 6 : 0;
}
