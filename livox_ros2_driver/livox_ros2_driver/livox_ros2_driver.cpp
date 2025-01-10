//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <chrono>
#include <csignal>
#include <future>
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include "include/livox_ros2_driver.h"

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"


#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include "livox_interfaces/msg/custom_point.h"
#include "livox_interfaces/msg/custom_msg.h"

namespace
{
  const int32_t kSdkVersionMajorLimit = 2;

  inline void SignalHandler(int signum)
  {
    rclcpp::shutdown();
    exit(signum);
  }
}

namespace livox_ros
{
LivoxDriver::LivoxDriver(const std::string node_name, const rclcpp::NodeOptions &options)
: Node(node_name, options)
{
}

void LivoxDriver::init()
{
  RCLCPP_INFO(this->get_logger(), "Livox Ros Driver Version: %s",
    LIVOX_ROS_DRIVER_VERSION_STRING);

  signal(SIGINT, SignalHandler);

  /** Check sdk version */
  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    RCLCPP_INFO(this->get_logger(),
      "The SDK version[%d.%d.%d] is too low", _sdkversion.major,
      _sdkversion.minor, _sdkversion.patch);
    rclcpp::shutdown();
    return;
  }

  /** Init default system parameter */
  int xfer_format;
  int multi_topic;
  int data_src;
  double publish_freq; /* Hz */
  int output_data_type;
  std::string frame_id;
  std::string lvx_file_path;
  std::string user_config_path;
  std::string cmdline_bd_code;

  auto xfer_format_param = this->get_parameter("xfer_format");
  if (xfer_format_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    xfer_format = this->declare_parameter<int>("xfer_format", kPointCloud2Msg);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "xfer_format");
  } else {
    xfer_format = xfer_format_param.get_value<int>();
  }

  auto multi_topic_param = this->get_parameter("multi_topic");
  if (multi_topic_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    multi_topic = this->declare_parameter<int>("multi_topic", 0);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "multi_topic");
  } else {
    multi_topic = multi_topic_param.get_value<int>();
  }

  auto data_src_param = this->get_parameter("data_src");
  if (data_src_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    data_src = this->declare_parameter<int>("data_src", kSourceRawLidar);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "data_src");
  } else {
    data_src = data_src_param.get_value<int>();
  }

  auto publish_freq_param = this->get_parameter("publish_freq");
  if (publish_freq_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    publish_freq = this->declare_parameter<double>("publish_freq", 10.0);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "publish_freq");
  } else {
    publish_freq = publish_freq_param.get_value<double>();
  }

  auto output_data_type_param = this->get_parameter("output_data_type");
  if (output_data_type_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    output_data_type = this->declare_parameter<int>("output_data_type", kOutputToRos);
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "output_data_type");
  } else {
    output_data_type = output_data_type_param.get_value<int>();
  }

  auto frame_id_param = this->get_parameter("frame_id");
  if (frame_id_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    frame_id = this->declare_parameter<std::string>("frame_id", "frame_default");
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "frame_id");
  } else {
    frame_id = frame_id_param.get_value<std::string>();
  }

  auto lvx_file_path_param = this->get_parameter("lvx_file_path");
  if (lvx_file_path_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    lvx_file_path = this->declare_parameter<std::string>("lvx_file_path", "/home/livox/livox_test.lvx");
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "lvx_file_path");
  } else {
    lvx_file_path = lvx_file_path_param.get_value<std::string>();
  }
  auto user_config_path_param = this->get_parameter("user_config_path");
  if (user_config_path_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    user_config_path = this->declare_parameter<std::string>("user_config_path", "path_default");
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "user_config_path");
  } else {
    user_config_path = user_config_path_param.get_value<std::string>();
  }
  auto cmdline_input_bd_code_param = this->get_parameter("cmdline_input_bd_code");
  if (cmdline_input_bd_code_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    cmdline_bd_code = this->declare_parameter<std::string>("cmdline_input_bd_code", "000000000000001");
    RCLCPP_WARN(this->get_logger(),
                "Parameter %s not set, using default", "cmdline_input_bd_code");
  } else {
    cmdline_bd_code = cmdline_input_bd_code_param.get_value<std::string>();
  }

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.1) {
    publish_freq = 0.1;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ =
    std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_data_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this->weak_from_this());

  int ret = 0;
  if (data_src == kSourceRawLidar) {
    RCLCPP_INFO(this->get_logger(), "Data Source is raw lidar.");
    RCLCPP_INFO(this->get_logger(), "Config file : %s", user_config_path.c_str());

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);
    
    std::string bd_code_str;
    for (const auto &code : bd_code_list) {
    	bd_code_str += code + " ";
    }
    RCLCPP_INFO(this->get_logger(), "bd_code_list: %s", bd_code_str.c_str());
    RCLCPP_INFO(this->get_logger(), "publish freq: %f", publish_freq);

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq);
    auto x = lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
    RCLCPP_INFO(this->get_logger(), "RegisterLds return %d!", x);
    ret = read_lidar->InitLdsLidar(bd_code_list, user_config_path.c_str());
    if (!ret) {
      RCLCPP_INFO(this->get_logger(), "Init lds lidar success!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Init lds lidar fail!");
    }
  } else if (data_src == kSourceRawHub) {
    RCLCPP_INFO(this->get_logger(), "Data Source is hub.");

    RCLCPP_INFO(this->get_logger(), "Config file : %s",
        user_config_path.c_str());

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code.c_str(), bd_code_list);

    LdsHub *read_hub = LdsHub::GetInstance(1000 / publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_hub));
    ret = read_hub->InitLdsHub(bd_code_list, user_config_path.c_str());
    if (!ret) {
      RCLCPP_INFO(this->get_logger(), "Init lds hub success!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Init lds hub fail!");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Data Source is lvx file.");

    do {
      if (!IsFilePathValid(lvx_file_path.c_str())) {
        RCLCPP_INFO(this->get_logger(), "File path invalid : %s !", lvx_file_path.c_str());
        break;
      }

      std::string rosbag_file_path;
      int path_end_pos = lvx_file_path.find_last_of('.');
      rosbag_file_path = lvx_file_path.substr(0, path_end_pos);
      rosbag_file_path += ".bag";

      LdsLvx *read_lvx = LdsLvx::GetInstance(1000 / publish_freq);
      lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lvx));
      lddc_ptr_->CreateBagFile(rosbag_file_path);
      int ret = read_lvx->InitLdsLvx(lvx_file_path.c_str());
      if (!ret) {
        RCLCPP_INFO(this->get_logger(), "Init lds lvx file success!");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Init lds lvx file fail!");
      }
    } while (0);
  }

  poll_thread_ = std::make_shared<std::thread>(&LivoxDriver::pollThread, this);
}

LivoxDriver::~LivoxDriver()
{
  exit_signal_.set_value();
  poll_thread_->join();
}

void LivoxDriver::pollThread()
{
  std::future_status status;

  do {
    lddc_ptr_->DistributeLidarData();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}
}  // namespace livox_ros

// #include <rclcpp_components/register_node_macro.hpp>

// RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::LivoxDriver)
