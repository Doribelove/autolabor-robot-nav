#include <camera_info_manager/camera_info_manager.h>
#include <hikrobot_mvs_camera/GetImagingControls.h>
#include <hikrobot_mvs_camera/SetImagingControls.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <MvCameraControl.h>
#include <MvErrorDefine.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

std::string errorCode(int code)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex
         << static_cast<std::uint32_t>(code);
  return stream.str();
}

bool isError(int actual, unsigned int expected)
{
  return static_cast<unsigned int>(actual) == expected;
}

std::string boundedString(const unsigned char* data, std::size_t capacity)
{
  const char* text = reinterpret_cast<const char*>(data);
  return std::string(text, strnlen(text, capacity));
}

std::string lowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char character) {
                   return static_cast<char>(std::tolower(character));
                 });
  return value;
}

std::string normalizeAutoMode(const std::string& value,
                              const std::string& parameter_name)
{
  const std::string lowered = lowerCopy(value);
  if (lowered == "off")
  {
    return "Off";
  }
  if (lowered == "once")
  {
    return "Once";
  }
  if (lowered == "continuous" || lowered == "continues")
  {
    return "Continuous";
  }

  ROS_WARN_STREAM("Invalid ~" << parameter_name << "='" << value
                  << "'; using Continuous");
  return "Continuous";
}

void setDefaultEnvironment(const char* name, const std::string& value)
{
  if (std::getenv(name) == nullptr)
  {
    setenv(name, value.c_str(), 0);
  }
}

}  // namespace

class HikrobotMvsCamera
{
public:
  HikrobotMvsCamera(ros::NodeHandle nh, ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      private_nh_(std::move(private_nh)),
      image_transport_(nh_)
  {
    private_nh_.param<std::string>("mvs_root", mvs_root_, "/opt/MVS");
    private_nh_.param<std::string>("serial_number", serial_number_, "");
    private_nh_.param<std::string>("transport", transport_, "usb");
    private_nh_.param<std::string>("camera_name", camera_name_, "camera");
    private_nh_.param<std::string>("frame_id", frame_id_,
                                   "camera_optical_frame");
    private_nh_.param<std::string>("camera_info_url", camera_info_url_, "");
    private_nh_.param<std::string>("image_topic", image_topic_, "image_raw");
    private_nh_.param("publisher_queue_size", publisher_queue_size_, 1);
    private_nh_.param("image_width", image_width_, 0);
    private_nh_.param("image_height", image_height_, 0);
    private_nh_.param("offset_x", offset_x_, 0);
    private_nh_.param("offset_y", offset_y_, 0);

    private_nh_.param("trigger_mode", trigger_mode_, false);
    private_nh_.param<std::string>("trigger_source", trigger_source_, "Line0");
    private_nh_.param("acquisition_frame_rate", acquisition_frame_rate_, 20.0);

    private_nh_.param<std::string>("exposure_auto", exposure_auto_,
                                   "Continuous");
    private_nh_.param("exposure_time_us", exposure_time_us_, 5000.0);
    private_nh_.param<std::string>("gain_auto", gain_auto_, "Continuous");
    private_nh_.param("gain", gain_, 0.0);

    private_nh_.param("grab_timeout_ms", grab_timeout_ms_, 1000);
    private_nh_.param("max_consecutive_timeouts",
                      max_consecutive_timeouts_, 5);
    private_nh_.param("reconnect_delay_sec", reconnect_delay_sec_, 1.0);

    publisher_queue_size_ = std::max(1, publisher_queue_size_);
    image_width_ = std::max(0, image_width_);
    image_height_ = std::max(0, image_height_);
    offset_x_ = std::max(0, offset_x_);
    offset_y_ = std::max(0, offset_y_);
    grab_timeout_ms_ = std::max(50, grab_timeout_ms_);
    max_consecutive_timeouts_ = std::max(1, max_consecutive_timeouts_);
    reconnect_delay_sec_ = std::max(0.1, reconnect_delay_sec_);
    acquisition_frame_rate_ = std::max(0.0, acquisition_frame_rate_);
    exposure_time_us_ = std::max(1.0, exposure_time_us_);
    gain_ = std::max(0.0, gain_);
    exposure_auto_ = normalizeAutoMode(exposure_auto_, "exposure_auto");
    gain_auto_ = normalizeAutoMode(gain_auto_, "gain_auto");
    transport_ = lowerCopy(transport_);
    if (transport_ == "gige")
    {
      transport_mask_ = MV_GIGE_DEVICE;
    }
    else if (transport_ == "all")
    {
      transport_mask_ = MV_GIGE_DEVICE | MV_USB_DEVICE;
    }
    else
    {
      if (transport_ != "usb")
      {
        ROS_WARN_STREAM("Invalid ~transport='" << transport_
                        << "'; using usb");
      }
      transport_ = "usb";
      transport_mask_ = MV_USB_DEVICE;
    }

    camera_publisher_ =
      image_transport_.advertiseCamera(image_topic_, publisher_queue_size_);
    camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
      nh_, camera_name_, camera_info_url_));
    get_imaging_controls_service_ = private_nh_.advertiseService(
      "get_imaging_controls",
      &HikrobotMvsCamera::getImagingControls,
      this);
    set_imaging_controls_service_ = private_nh_.advertiseService(
      "set_imaging_controls",
      &HikrobotMvsCamera::setImagingControls,
      this);
  }

  ~HikrobotMvsCamera()
  {
    disconnect();
    if (sdk_initialized_)
    {
      const int result = MV_CC_Finalize();
      if (result != MV_OK)
      {
        ROS_WARN_STREAM("MV_CC_Finalize failed: " << errorCode(result));
      }
    }
  }

  int run()
  {
    setDefaultEnvironment("MVCAM_SDK_PATH", mvs_root_);
    setDefaultEnvironment("MVCAM_COMMON_RUNENV", mvs_root_ + "/lib");
    setDefaultEnvironment("MVCAM_GENICAM_CLPROTOCOL",
                          mvs_root_ + "/lib/CLProtocol");
    setDefaultEnvironment("ALLUSERSPROFILE", mvs_root_ + "/MVFG");

    const int initialize_result = MV_CC_Initialize();
    if (initialize_result != MV_OK)
    {
      ROS_FATAL_STREAM("MV_CC_Initialize failed: "
                       << errorCode(initialize_result));
      return 2;
    }
    sdk_initialized_ = true;

    ROS_INFO_STREAM("Hikrobot MVS SDK initialized (version "
                    << errorCode(static_cast<int>(MV_CC_GetSDKVersion()))
                    << "); waiting for camera serial '"
                    << (serial_number_.empty() ? "<any single device>"
                                               : serial_number_)
                    << "'");

    while (ros::ok())
    {
      ros::spinOnce();
      if (!connect())
      {
        ros::WallDuration(reconnect_delay_sec_).sleep();
        continue;
      }

      captureUntilDisconnected();
      disconnect();

      if (ros::ok())
      {
        ros::WallDuration(reconnect_delay_sec_).sleep();
      }
    }

    return 0;
  }

private:
  struct DeviceIdentity
  {
    std::string model;
    std::string serial;
    std::string transport;
  };

  struct FloatFeature
  {
    double current{0.0};
    double minimum{0.0};
    double maximum{0.0};
  };

  DeviceIdentity identify(const MV_CC_DEVICE_INFO& device) const
  {
    DeviceIdentity identity;
    if (device.nTLayerType == MV_USB_DEVICE)
    {
      identity.transport = "USB3";
      identity.model = boundedString(
        device.SpecialInfo.stUsb3VInfo.chModelName,
        sizeof(device.SpecialInfo.stUsb3VInfo.chModelName));
      identity.serial = boundedString(
        device.SpecialInfo.stUsb3VInfo.chSerialNumber,
        sizeof(device.SpecialInfo.stUsb3VInfo.chSerialNumber));
    }
    else if (device.nTLayerType == MV_GIGE_DEVICE)
    {
      identity.transport = "GigE";
      identity.model = boundedString(
        device.SpecialInfo.stGigEInfo.chModelName,
        sizeof(device.SpecialInfo.stGigEInfo.chModelName));
      identity.serial = boundedString(
        device.SpecialInfo.stGigEInfo.chSerialNumber,
        sizeof(device.SpecialInfo.stGigEInfo.chSerialNumber));
    }
    else
    {
      identity.transport = "unknown";
      identity.model = "unknown";
    }
    return identity;
  }

  bool connect()
  {
    MV_CC_DEVICE_INFO_LIST device_list = {};
    const int enumerate_result =
      MV_CC_EnumDevices(transport_mask_, &device_list);
    if (enumerate_result != MV_OK)
    {
      ROS_WARN_STREAM_THROTTLE(
        5.0, "MV_CC_EnumDevices failed: " << errorCode(enumerate_result));
      return false;
    }

    if (device_list.nDeviceNum == 0)
    {
      ROS_WARN_THROTTLE(
        5.0,
        "No Hikrobot camera detected. Connect the camera over USB3; "
        "the driver will retry.");
      return false;
    }

    MV_CC_DEVICE_INFO* selected = nullptr;
    DeviceIdentity selected_identity;
    std::ostringstream available;

    for (unsigned int index = 0; index < device_list.nDeviceNum; ++index)
    {
      MV_CC_DEVICE_INFO* candidate = device_list.pDeviceInfo[index];
      if (candidate == nullptr)
      {
        continue;
      }
      const DeviceIdentity identity = identify(*candidate);
      if (available.tellp() > 0)
      {
        available << ", ";
      }
      available << identity.model << "[" << identity.serial << "]";

      if (!serial_number_.empty() && identity.serial == serial_number_)
      {
        selected = candidate;
        selected_identity = identity;
        break;
      }
      if (serial_number_.empty() && device_list.nDeviceNum == 1)
      {
        selected = candidate;
        selected_identity = identity;
      }
    }

    if (selected == nullptr)
    {
      if (serial_number_.empty())
      {
        ROS_ERROR_STREAM_THROTTLE(
          5.0,
          "Multiple Hikrobot cameras are present; set ~serial_number. "
            << "Available: " << available.str());
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(
          5.0, "Camera serial '" << serial_number_
          << "' not found. Available: " << available.str());
      }
      return false;
    }

    int result = MV_CC_CreateHandle(&handle_, selected);
    if (result != MV_OK)
    {
      handle_ = nullptr;
      ROS_ERROR_STREAM("MV_CC_CreateHandle failed: " << errorCode(result));
      return false;
    }

    result = MV_CC_OpenDevice(handle_);
    if (result != MV_OK)
    {
      ROS_ERROR_STREAM(
        "Cannot open " << selected_identity.model << "["
        << selected_identity.serial << "]: " << errorCode(result)
        << ". Close the MVS GUI or any other process using the camera.");
      disconnect();
      return false;
    }

    const int strategy_result =
      MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly);
    if (strategy_result != MV_OK)
    {
      ROS_WARN_STREAM("Could not select latest-frame grab strategy: "
                      << errorCode(strategy_result));
    }

    if (!configureCamera())
    {
      disconnect();
      return false;
    }

    result = MV_CC_StartGrabbing(handle_);
    if (result != MV_OK)
    {
      ROS_ERROR_STREAM("MV_CC_StartGrabbing failed: " << errorCode(result));
      disconnect();
      return false;
    }
    grabbing_ = true;

    ROS_INFO_STREAM("Connected Hikrobot " << selected_identity.model << " ("
                    << selected_identity.transport << ", serial "
                    << selected_identity.serial << "), publishing "
                    << nh_.resolveName(image_topic_) << " and "
                    << nh_.resolveName("camera_info"));
    return true;
  }

  bool configureCamera()
  {
    if (image_width_ > 0 || image_height_ > 0)
    {
      if (image_width_ <= 0 || image_height_ <= 0)
      {
        ROS_ERROR("Both ~image_width and ~image_height must be positive "
                  "when a calibrated output size is requested");
        return false;
      }

      // Reset the ROI origin before expanding the image dimensions. This
      // ordering is required by GenICam cameras when the previous ROI was
      // smaller or offset from the sensor origin.
      if (!setInt("OffsetX", offset_x_, true) ||
          !setInt("OffsetY", offset_y_, true) ||
          !setInt("Width", image_width_, true) ||
          !setInt("Height", image_height_, true) ||
          !verifyInt("OffsetX", offset_x_) ||
          !verifyInt("OffsetY", offset_y_) ||
          !verifyInt("Width", image_width_) ||
          !verifyInt("Height", image_height_))
      {
        ROS_ERROR("Camera ROI does not match the requested calibrated output");
        return false;
      }
    }

    if (!setEnum("TriggerMode", trigger_mode_ ? "On" : "Off", true))
    {
      return false;
    }

    if (trigger_mode_)
    {
      if (!setEnum("TriggerSource", trigger_source_, true))
      {
        return false;
      }
      setBool("AcquisitionFrameRateEnable", false, false);
    }
    else if (acquisition_frame_rate_ > 0.0)
    {
      if (setBool("AcquisitionFrameRateEnable", true, false))
      {
        setFloat("AcquisitionFrameRate", acquisition_frame_rate_, false);
      }
    }

    if (setEnum("ExposureAuto", exposure_auto_, false) &&
        exposure_auto_ == "Off")
    {
      setFloat("ExposureTime", exposure_time_us_, false);
    }

    if (setEnum("GainAuto", gain_auto_, false) && gain_auto_ == "Off")
    {
      setFloat("Gain", gain_, false);
    }

    return true;
  }

  bool setEnum(const std::string& key, const std::string& value, bool required)
  {
    const int result =
      MV_CC_SetEnumValueByString(handle_, key.c_str(), value.c_str());
    if (result == MV_OK)
    {
      return true;
    }
    if (required)
    {
      ROS_ERROR_STREAM("Failed to set " << key << "=" << value << ": "
                       << errorCode(result));
    }
    else
    {
      ROS_WARN_STREAM("Camera does not accept optional " << key << "="
                      << value << ": " << errorCode(result));
    }
    return false;
  }

  bool setBool(const std::string& key, bool value, bool required)
  {
    const int result = MV_CC_SetBoolValue(handle_, key.c_str(), value);
    if (result == MV_OK)
    {
      return true;
    }
    if (required)
    {
      ROS_ERROR_STREAM("Failed to set " << key << "="
                       << (value ? "true" : "false") << ": "
                       << errorCode(result));
    }
    else
    {
      ROS_WARN_STREAM("Camera does not accept optional " << key << "="
                      << (value ? "true" : "false") << ": "
                      << errorCode(result));
    }
    return false;
  }

  bool setFloat(const std::string& key, double value, bool required)
  {
    const int result =
      MV_CC_SetFloatValue(handle_, key.c_str(), static_cast<float>(value));
    if (result == MV_OK)
    {
      return true;
    }
    if (required)
    {
      ROS_ERROR_STREAM("Failed to set " << key << "=" << value << ": "
                       << errorCode(result));
    }
    else
    {
      ROS_WARN_STREAM("Camera does not accept optional " << key << "="
                      << value << ": " << errorCode(result));
    }
    return false;
  }

  bool getFloat(const std::string& key,
                FloatFeature& value,
                std::string& message) const
  {
    MVCC_FLOATVALUE feature = {};
    const int result = MV_CC_GetFloatValue(
      handle_, key.c_str(), &feature);
    if (result != MV_OK)
    {
      std::ostringstream stream;
      stream << "Failed to read " << key << ": " << errorCode(result);
      message = stream.str();
      return false;
    }

    value.current = feature.fCurValue;
    value.minimum = feature.fMin;
    value.maximum = feature.fMax;
    return true;
  }

  bool getImagingControls(
    hikrobot_mvs_camera::GetImagingControls::Request&,
    hikrobot_mvs_camera::GetImagingControls::Response& response)
  {
    if (handle_ == nullptr)
    {
      response.success = false;
      response.message = "camera is not connected";
      return true;
    }

    FloatFeature exposure;
    FloatFeature gain;
    std::string message;
    if (!getFloat("ExposureTime", exposure, message) ||
        !getFloat("Gain", gain, message))
    {
      response.success = false;
      response.message = message;
      return true;
    }

    response.success = true;
    response.message = "camera imaging controls available";
    response.exposure_auto = exposure_auto_ != "Off";
    response.exposure_time_us = exposure.current;
    response.exposure_min_us = exposure.minimum;
    response.exposure_max_us = exposure.maximum;
    response.gain_auto = gain_auto_ != "Off";
    response.gain = gain.current;
    response.gain_min = gain.minimum;
    response.gain_max = gain.maximum;
    return true;
  }

  bool setImagingControls(
    hikrobot_mvs_camera::SetImagingControls::Request& request,
    hikrobot_mvs_camera::SetImagingControls::Response& response)
  {
    if (handle_ == nullptr)
    {
      response.success = false;
      response.message = "camera is not connected";
      return true;
    }
    if ((!request.exposure_auto &&
         (!std::isfinite(request.exposure_time_us) ||
          request.exposure_time_us <= 0.0)) ||
        (!request.gain_auto &&
         (!std::isfinite(request.gain) || request.gain < 0.0)))
    {
      response.success = false;
      response.message = "manual exposure and gain must be finite and valid";
      return true;
    }

    FloatFeature exposure;
    FloatFeature gain;
    std::string message;
    if (!getFloat("ExposureTime", exposure, message) ||
        !getFloat("Gain", gain, message))
    {
      response.success = false;
      response.message = message;
      return true;
    }

    const double requested_exposure = std::max(
      exposure.minimum,
      std::min(exposure.maximum, request.exposure_time_us));
    const double requested_gain = std::max(
      gain.minimum, std::min(gain.maximum, request.gain));
    const std::string previous_exposure_auto = exposure_auto_;
    const std::string previous_gain_auto = gain_auto_;
    const double previous_exposure_time_us = exposure.current;
    const double previous_gain = gain.current;

    const auto rollback = [&]() {
      bool restored = true;
      if (previous_exposure_auto == "Off")
      {
        restored =
          setEnum("ExposureAuto", "Off", false) && restored;
        restored =
          setFloat("ExposureTime", previous_exposure_time_us, false) &&
          restored;
      }
      else
      {
        restored =
          setEnum("ExposureAuto", previous_exposure_auto, false) && restored;
      }
      if (previous_gain_auto == "Off")
      {
        restored = setEnum("GainAuto", "Off", false) && restored;
        restored = setFloat("Gain", previous_gain, false) && restored;
      }
      else
      {
        restored =
          setEnum("GainAuto", previous_gain_auto, false) && restored;
      }
      exposure_auto_ = previous_exposure_auto;
      gain_auto_ = previous_gain_auto;
      return restored;
    };
    const auto fail = [&](const std::string& failure) {
      const bool restored = rollback();
      response.success = false;
      response.message =
        failure +
        (restored ? "; previous controls restored"
                  : "; WARNING: control rollback was incomplete");
      return true;
    };

    const std::string requested_exposure_auto =
      request.exposure_auto ? "Continuous" : "Off";
    if (!setEnum("ExposureAuto", requested_exposure_auto, true))
    {
      return fail("camera rejected ExposureAuto");
    }
    exposure_auto_ = requested_exposure_auto;
    if (!request.exposure_auto)
    {
      if (!setFloat("ExposureTime", requested_exposure, true))
      {
        return fail("camera rejected ExposureTime");
      }
      exposure_time_us_ = requested_exposure;
    }

    const std::string requested_gain_auto =
      request.gain_auto ? "Continuous" : "Off";
    if (!setEnum("GainAuto", requested_gain_auto, true))
    {
      return fail("camera rejected GainAuto");
    }
    gain_auto_ = requested_gain_auto;
    if (!request.gain_auto)
    {
      if (!setFloat("Gain", requested_gain, true))
      {
        return fail("camera rejected Gain");
      }
      gain_ = requested_gain;
    }

    if (!getFloat("ExposureTime", exposure, message) ||
        !getFloat("Gain", gain, message))
    {
      return fail(message);
    }

    response.success = true;
    response.message = "camera imaging controls applied";
    response.exposure_time_us = exposure.current;
    response.gain = gain.current;
    ROS_INFO_STREAM_THROTTLE(
      5.0, "Runtime imaging controls: exposure_auto="
      << (request.exposure_auto ? "true" : "false")
      << ", exposure_us=" << exposure.current
      << ", gain_auto=" << (request.gain_auto ? "true" : "false")
      << ", gain=" << gain.current);
    return true;
  }

  bool setInt(const std::string& key, std::int64_t value, bool required)
  {
    const int result = MV_CC_SetIntValueEx(handle_, key.c_str(), value);
    if (result == MV_OK)
    {
      return true;
    }
    if (required)
    {
      ROS_ERROR_STREAM("Failed to set " << key << "=" << value << ": "
                       << errorCode(result));
    }
    else
    {
      ROS_WARN_STREAM("Camera does not accept optional " << key << "="
                      << value << ": " << errorCode(result));
    }
    return false;
  }

  bool verifyInt(const std::string& key, std::int64_t expected)
  {
    MVCC_INTVALUE_EX value = {};
    const int result = MV_CC_GetIntValueEx(handle_, key.c_str(), &value);
    if (result != MV_OK)
    {
      ROS_ERROR_STREAM("Failed to read back " << key << ": "
                       << errorCode(result));
      return false;
    }
    if (value.nCurValue != expected)
    {
      ROS_ERROR_STREAM("Camera applied " << key << "=" << value.nCurValue
                       << ", expected " << expected);
      return false;
    }
    return true;
  }

  void captureUntilDisconnected()
  {
    int consecutive_timeouts = 0;

    while (ros::ok() && handle_ != nullptr)
    {
      MV_FRAME_OUT frame = {};
      const int result =
        MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);

      if (isError(result, MV_E_NODATA))
      {
        ros::spinOnce();
        if (trigger_mode_)
        {
          ROS_DEBUG_THROTTLE(
            5.0, "Waiting for an external camera trigger on %s",
            trigger_source_.c_str());
          continue;
        }

        ++consecutive_timeouts;
        if (consecutive_timeouts >= max_consecutive_timeouts_)
        {
          ROS_WARN("Camera produced no frames; reconnecting.");
          return;
        }
        continue;
      }

      if (result != MV_OK)
      {
        ROS_WARN_STREAM("MV_CC_GetImageBuffer failed: " << errorCode(result)
                        << "; reconnecting");
        return;
      }

      consecutive_timeouts = 0;
      const ros::Time stamp = ros::Time::now();
      const bool converted = publishFrame(frame, stamp);
      const int free_result = MV_CC_FreeImageBuffer(handle_, &frame);
      if (free_result != MV_OK)
      {
        ROS_WARN_STREAM("MV_CC_FreeImageBuffer failed: "
                        << errorCode(free_result) << "; reconnecting");
        return;
      }

      if (!converted)
      {
        ROS_WARN_THROTTLE(5.0, "Dropping a camera frame after conversion error");
      }
      ros::spinOnce();
    }
  }

  bool publishFrame(const MV_FRAME_OUT& frame, const ros::Time& stamp)
  {
    const unsigned int width =
      frame.stFrameInfo.nExtendWidth != 0
        ? frame.stFrameInfo.nExtendWidth
        : frame.stFrameInfo.nWidth;
    const unsigned int height =
      frame.stFrameInfo.nExtendHeight != 0
        ? frame.stFrameInfo.nExtendHeight
        : frame.stFrameInfo.nHeight;

    if (width == 0 || height == 0 || frame.pBufAddr == nullptr)
    {
      ROS_ERROR_THROTTLE(5.0, "MVS returned an invalid frame");
      return false;
    }

    const std::uint64_t output_size =
      static_cast<std::uint64_t>(width) * height * 3U;
    if (output_size > std::numeric_limits<unsigned int>::max() ||
        frame.stFrameInfo.nFrameLenEx >
          std::numeric_limits<unsigned int>::max())
    {
      ROS_ERROR_THROTTLE(5.0, "Camera frame exceeds MVS conversion limits");
      return false;
    }

    conversion_buffer_.resize(static_cast<std::size_t>(output_size));
    MV_CC_PIXEL_CONVERT_PARAM_EX conversion = {};
    conversion.nWidth = width;
    conversion.nHeight = height;
    conversion.enSrcPixelType = frame.stFrameInfo.enPixelType;
    conversion.pSrcData = frame.pBufAddr;
    conversion.nSrcDataLen =
      static_cast<unsigned int>(frame.stFrameInfo.nFrameLenEx);
    conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    conversion.pDstBuffer = conversion_buffer_.data();
    conversion.nDstBufferSize =
      static_cast<unsigned int>(conversion_buffer_.size());

    const int result = MV_CC_ConvertPixelTypeEx(handle_, &conversion);
    if (result != MV_OK)
    {
      ROS_ERROR_STREAM_THROTTLE(
        5.0, "MV_CC_ConvertPixelTypeEx failed: " << errorCode(result)
        << " (source pixel type "
        << errorCode(static_cast<int>(frame.stFrameInfo.enPixelType)) << ")");
      return false;
    }

    if (conversion.nDstLen < output_size)
    {
      ROS_ERROR_STREAM_THROTTLE(
        5.0, "MVS returned only " << conversion.nDstLen
        << " converted bytes; expected " << output_size);
      return false;
    }

    sensor_msgs::Image image;
    image.header.seq = frame.stFrameInfo.nFrameNum;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = height;
    image.width = width;
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = width * 3U;
    image.data.assign(conversion_buffer_.begin(),
                      conversion_buffer_.begin() + output_size);

    sensor_msgs::CameraInfo camera_info =
      camera_info_manager_->getCameraInfo();
    camera_info.header = image.header;
    if (camera_info.width == 0 || camera_info.height == 0)
    {
      camera_info.width = width;
      camera_info.height = height;
    }
    else if (camera_info.width != width || camera_info.height != height)
    {
      ROS_WARN_STREAM_THROTTLE(
        5.0, "Calibration resolution " << camera_info.width << "x"
        << camera_info.height << " differs from camera output "
        << width << "x" << height);
    }

    camera_publisher_.publish(image, camera_info);
    ++published_frames_;
    if (published_frames_ == 1)
    {
      ROS_INFO_STREAM("Published first " << width << "x" << height
                      << " bgr8 frame in '" << frame_id_ << "'");
    }
    return true;
  }

  void disconnect()
  {
    if (handle_ == nullptr)
    {
      return;
    }

    if (grabbing_)
    {
      const int stop_result = MV_CC_StopGrabbing(handle_);
      if (stop_result != MV_OK)
      {
        ROS_WARN_STREAM("MV_CC_StopGrabbing failed: "
                        << errorCode(stop_result));
      }
      grabbing_ = false;
    }

    const int close_result = MV_CC_CloseDevice(handle_);
    if (close_result != MV_OK)
    {
      ROS_WARN_STREAM("MV_CC_CloseDevice failed: "
                      << errorCode(close_result));
    }

    const int destroy_result = MV_CC_DestroyHandle(handle_);
    if (destroy_result != MV_OK)
    {
      ROS_WARN_STREAM("MV_CC_DestroyHandle failed: "
                      << errorCode(destroy_result));
    }
    handle_ = nullptr;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher camera_publisher_;
  std::unique_ptr<camera_info_manager::CameraInfoManager>
    camera_info_manager_;
  ros::ServiceServer get_imaging_controls_service_;
  ros::ServiceServer set_imaging_controls_service_;

  std::string mvs_root_;
  std::string serial_number_;
  std::string transport_;
  unsigned int transport_mask_{MV_USB_DEVICE};
  std::string camera_name_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string image_topic_;
  int publisher_queue_size_{1};
  int image_width_{0};
  int image_height_{0};
  int offset_x_{0};
  int offset_y_{0};

  bool trigger_mode_{false};
  std::string trigger_source_;
  double acquisition_frame_rate_{20.0};
  std::string exposure_auto_;
  double exposure_time_us_{5000.0};
  std::string gain_auto_;
  double gain_{0.0};

  int grab_timeout_ms_{1000};
  int max_consecutive_timeouts_{5};
  double reconnect_delay_sec_{1.0};

  void* handle_{nullptr};
  bool sdk_initialized_{false};
  bool grabbing_{false};
  std::uint64_t published_frames_{0};
  std::vector<unsigned char> conversion_buffer_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hikrobot_mvs_camera");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  HikrobotMvsCamera camera(nh, private_nh);
  return camera.run();
}
