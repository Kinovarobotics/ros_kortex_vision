#ifndef KINOVA_VISION_H
#define KINOVA_VISION_H

extern "C" {
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
}

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

namespace CameraTypes
{
enum CameraType
{
  Unknown = 0,
  Color = 1,
  Depth = 2,
};
}

class Vision
{
public:
  Vision(ros::NodeHandle nh_camera, ros::NodeHandle nh_private);
  ~Vision();

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
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  camera_info_manager::CameraInfoManager camera_info_manager_;
  image_transport::CameraPublisher camera_publisher_;
  image_transport::ImageTransport image_transport_;

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

  bool is_started_;
  bool stop_requested_;
  bool quit_requested_;
  int retry_count_;
  int camera_type_;
  double time_offset_;
  int image_width_;
  int image_height_;
  int pixel_size_;
  bool use_gst_timestamps_;
  bool is_first_initialize_;
};

#endif
