#pragma once

#include <atomic>
#include <memory>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

namespace CameraTypes
{
enum CameraType
{
  Unknown = 0,
  Color = 1,
  Depth = 2,
};
}

namespace ros_kortex_vision
{
class Vision
{
public:
  Vision(const rclcpp::NodeOptions& options);
  ~Vision();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void run();
  void quit();

private:
  bool configure();
  bool initialize();
  bool start();
  bool loadCameraInfo();
  bool publish();
  void stop();
  bool changePipelineState(GstState state);

private:
  // ROS elements
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::shared_ptr<image_transport::CameraPublisher> camera_publisher_;

  // Node status booleans
  std::atomic<bool> is_started_;
  std::atomic<bool> stop_requested_;
  std::atomic<bool> quit_requested_;

  // Gstreamer elements
  GstElement* gst_pipeline_;
  GstElement* gst_sink_;

  // General gstreamer configuration
  std::string camera_config_;
  std::string camera_name_;
  std::string camera_info_;
  std::string frame_id_;
  std::string image_encoding_;
  std::string base_frame_id_;
  int retry_count_;
  int camera_type_;
  double time_offset_;
  int image_width_;
  int image_height_;
  int pixel_size_;
  bool use_gst_timestamps_;
  bool is_first_initialize_;

  // Maximum publication rate variables
  double max_pub_rate_hz_;
  std::shared_ptr<rclcpp::Rate> max_pub_rate_;
};
}  // namespace ros_kortex_vision
