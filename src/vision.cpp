#include <ros_kortex_vision/vision.h>

#include <stdio.h>

namespace
{
constexpr auto NODE_NAME = "kinova_vision";
constexpr auto CAM_INFO_DEFAULT_URL_MAX_SIZE = 128;

constexpr auto DEFAULT_BASE_FRAME_ID = "camera_link";
constexpr auto DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
constexpr auto DEFAULT_COLOR_FRAME_ID = "camera_color_frame";

const int RETRY_INTERVAL = 3;
const unsigned int STATE_CHANGE_ASYNC_TIMEOUT = 15;
const unsigned int APP_SINK_BUFFER_COUNT = 5;

// ROS Parameter names
// TODO: Use generate_param_library and clean up configuration
const std::string STREAM_CONFIG_PARAM = "stream_config";
const std::string CAMERA_TYPE_PARAM = "camera_type";
const std::string CAMERA_NAME_PARAM = "camera_name";
const std::string FRAME_ID_PARAM = "frame_id";
const std::string CAMERA_INFO_URL_USER_PARAM = "camera_info_url_user";
const std::string CAMERA_INFO_URL_DEFAULT_PARAM = "camera_info_url_default";
}  // namespace

namespace ros_kortex_vision
{
Vision::Vision(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>(NODE_NAME, options) }
  , camera_info_manager_{ std::make_shared<camera_info_manager::CameraInfoManager>(node_.get()) }
  , camera_publisher_{ std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(node_.get(), "image_raw")) }
  , is_started_(false)
  , stop_requested_(false)
  , quit_requested_(false)
  , gst_pipeline_(NULL)
  , gst_sink_(NULL)
  , base_frame_id_(DEFAULT_BASE_FRAME_ID)
  , retry_count_(0)
  , camera_type_(CameraTypes::Unknown)
  , time_offset_(0)
  , image_width_(0)
  , image_height_(0)
  , pixel_size_(0)
  , use_gst_timestamps_(false)
  , is_first_initialize_(true)
{
}

Vision::~Vision()
{
  quit();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Vision::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool Vision::configure()
{
  node_->declare_parameter<std::string>(STREAM_CONFIG_PARAM);
  if (!node_->get_parameter<std::string>(STREAM_CONFIG_PARAM, camera_config_))
  {
    RCLCPP_FATAL(node_->get_logger(), "'%s' rosparam is not set. This is needed to set up a gstreamer pipeline.",
                 STREAM_CONFIG_PARAM.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Using gstreamer config from rosparam: \"" << camera_config_ << "\"");
  }

  std::string camera_type;
  node_->declare_parameter<std::string>(CAMERA_TYPE_PARAM);
  if (!node_->get_parameter<std::string>(CAMERA_TYPE_PARAM, camera_type))
  {
    RCLCPP_FATAL(node_->get_logger(), "'%s' rosparam is not set. This param is required for this node to run! Exiting.",
                 CAMERA_TYPE_PARAM.c_str());
    return false;
  }

  // Set encoding related to camera type;
  if (camera_type == "color")
  {
    camera_type_ = CameraTypes::Color;
    image_encoding_ = sensor_msgs::image_encodings::RGB8;
  }
  else if (camera_type == "depth")
  {
    camera_type_ = CameraTypes::Depth;
    image_encoding_ = sensor_msgs::image_encodings::TYPE_16UC1;
  }
  else
  {
    RCLCPP_FATAL(node_->get_logger(), "%s: '%s' is invalid! Must be 'color' or 'depth'", CAMERA_TYPE_PARAM.c_str(),
                 camera_type.c_str());
    return false;
  }

  pixel_size_ = sensor_msgs::image_encodings::numChannels(image_encoding_) *
                (sensor_msgs::image_encodings::bitDepth(image_encoding_) / 8);

  node_->declare_parameter<std::string>(CAMERA_NAME_PARAM);
  if (node_->get_parameter<std::string>(CAMERA_NAME_PARAM, camera_name_))
  {
    camera_info_manager_->setCameraName(camera_name_);
  }
  else
  {
    camera_name_ = "Camera";
    RCLCPP_WARN(node_->get_logger(), "%s param not found. Using default value: %s", CAMERA_NAME_PARAM.c_str(),
                camera_name_.c_str());
    camera_info_manager_->setCameraName(camera_name_);
  }

  node_->declare_parameter<std::string>(FRAME_ID_PARAM);
  if (!node_->get_parameter<std::string>(FRAME_ID_PARAM, frame_id_))
  {
    frame_id_ = "/camera_frame";
    RCLCPP_WARN(node_->get_logger(), "No camera frame_id set, using frame '%s'", frame_id_.c_str());
    node_->set_parameter(rclcpp::Parameter(FRAME_ID_PARAM, frame_id_));
  }

  return true;
}

bool Vision::initialize()
{
  if (!gst_is_initialized())
  {
    gst_init(0, 0);
  }

  GError* error = 0;  // Assignment to zero is a gst requirement

  gst_pipeline_ = gst_parse_launch(camera_config_.c_str(), &error);
  if (gst_pipeline_ == NULL)
  {
    RCLCPP_FATAL(node_->get_logger(), error->message);
    return false;
  }

  // Create sink
  gst_sink_ = gst_element_factory_make("appsink", NULL);
  GstCaps* caps = gst_app_sink_get_caps(GST_APP_SINK(gst_sink_));

  // Set image encoding related to camera/stream type
  std::string gst_encoding = "";

  if (image_encoding_ == sensor_msgs::image_encodings::TYPE_16UC1)
  {
    gst_encoding = "GRAY16_LE";
  }
  else if (image_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    gst_encoding = "RGB";
  }
  caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, gst_encoding.c_str(), NULL);

  gst_app_sink_set_caps(GST_APP_SINK(gst_sink_), caps);
  gst_caps_unref(caps);

  if (GST_IS_PIPELINE(gst_pipeline_))
  {
    GstPad* outpad = gst_bin_find_unlinked_pad(GST_BIN(gst_pipeline_), GST_PAD_SRC);
    g_assert(outpad);

    GstElement* outelement = gst_pad_get_parent_element(outpad);
    g_assert(outelement);
    gst_object_unref(outpad);

    if (!gst_bin_add(GST_BIN(gst_pipeline_), gst_sink_))
    {
      RCLCPP_FATAL(node_->get_logger(), "[%s]: gst_bin_add() failed", camera_name_.c_str());
      gst_object_unref(outelement);
      gst_object_unref(gst_pipeline_);
      gst_pipeline_ = NULL;
      return false;
    }

    if (!gst_element_link(outelement, gst_sink_))
    {
      RCLCPP_FATAL(node_->get_logger(), "[%s]: gstreamer: cannot link outelement(\"%s\") -> sink\n",
                   camera_name_.c_str(), gst_element_get_name(outelement));
      gst_object_unref(outelement);
      gst_object_unref(gst_pipeline_);
      gst_pipeline_ = NULL;
      return false;
    }

    gst_object_unref(outelement);
  }
  else
  {
    GstElement* launchpipe = gst_pipeline_;
    gst_pipeline_ = gst_pipeline_new(NULL);
    g_assert(gst_pipeline_);

    gst_object_unparent(GST_OBJECT(launchpipe));

    gst_bin_add_many(GST_BIN(gst_pipeline_), launchpipe, gst_sink_, NULL);

    if (!gst_element_link(launchpipe, gst_sink_))
    {
      RCLCPP_FATAL(node_->get_logger(), "[%s]: gstreamer: cannot link launchpipe -> sink", camera_name_.c_str());
      gst_object_unref(gst_pipeline_);
      gst_pipeline_ = NULL;
      return false;
    }
  }

  // Calibration between ros node time and gst timestamps
  GstClock* clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  auto now = node_->now();
  time_offset_ = now.seconds() - static_cast<double>(GST_TIME_AS_USECONDS(ct)) / 1e6;

  gst_element_set_state(gst_pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(gst_pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
  {
    RCLCPP_FATAL(node_->get_logger(), "[%s]: Failed to PAUSE stream, check your gstreamer configuration.",
                 camera_name_.c_str());
    return false;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Stream is PAUSED", camera_name_.c_str());
  }

  return true;
}

bool Vision::start()
{
  GstStateChangeReturn ret = gst_element_set_state(gst_pipeline_, GST_STATE_PLAYING);

  switch (ret)
  {
    case GST_STATE_CHANGE_FAILURE:
    case GST_STATE_CHANGE_NO_PREROLL:
      RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start stream", camera_name_.c_str());
      return false;

    case GST_STATE_CHANGE_ASYNC: {
      ret = gst_element_get_state(gst_pipeline_, NULL, NULL, STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);

      switch (ret)
      {
        case GST_STATE_CHANGE_FAILURE:
          RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start stream", camera_name_.c_str());
          return false;

        case GST_STATE_CHANGE_ASYNC:
          RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to start stream (timeout)", camera_name_.c_str());
          return false;
      }
    }

    case GST_STATE_CHANGE_SUCCESS:
      RCLCPP_INFO(node_->get_logger(), "[%s]: Stream started", camera_name_.c_str());
      break;

    default:
      return false;
  }

  // Get the frame width and height
  GstPad* pad = gst_element_get_static_pad(gst_sink_, "sink");
  const GstCaps* caps = gst_pad_get_current_caps(pad);

  GstStructure* structure = gst_caps_get_structure(caps, 0);
  gst_structure_get_int(structure, "width", &image_width_);
  gst_structure_get_int(structure, "height", &image_height_);

  gst_app_sink_set_max_buffers(GST_APP_SINK(gst_sink_), APP_SINK_BUFFER_COUNT);
  gst_app_sink_set_drop(GST_APP_SINK(gst_sink_), true);

  return true;
}

bool Vision::loadCameraInfo()
{
  std::string cam_info_default;
  char cam_info_default_resolved[CAM_INFO_DEFAULT_URL_MAX_SIZE];

  /*
   * The user can specify a custom camera information file when launching the nodelet.
   * Otherwise, a default information file is selected based on the sensor resolution.
   */
  node_->declare_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM);
  node_->get_parameter<std::string>(CAMERA_INFO_URL_USER_PARAM, camera_info_);
  if (camera_info_.empty())
  {
    RCLCPP_INFO(node_->get_logger(),
                "[%s]: Custom camera information file not set, using default one based on sensor resolution",
                camera_name_.c_str());

    node_->declare_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM);
    node_->get_parameter<std::string>(CAMERA_INFO_URL_DEFAULT_PARAM, cam_info_default);
    if (!cam_info_default.empty())
    {
      snprintf(cam_info_default_resolved, CAM_INFO_DEFAULT_URL_MAX_SIZE, cam_info_default.c_str(), image_width_,
               image_height_);
      camera_info_.assign(cam_info_default_resolved);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "[%s]: Parameter '[%s]' not found or empty.", camera_name_.c_str(),
                  CAMERA_INFO_URL_DEFAULT_PARAM.c_str());
    }
  }

  if (camera_info_manager_->validateURL(camera_info_))
  {
    if (camera_info_manager_->loadCameraInfo(camera_info_))
    {
      RCLCPP_INFO(node_->get_logger(), "[%s]: Loaded camera calibration from '%s'", camera_name_.c_str(),
                  camera_info_.c_str());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "[%s]: Camera info at '%s' not found. Using an uncalibrated config.",
                  camera_name_.c_str(), camera_info_.c_str());
    }
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "[%s]: Camera info url syntax not supported.", camera_name_.c_str());
  }

  return true;
}

bool Vision::publish()
{
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(gst_sink_));
  if (!sample)
  {
    return false;
  }

  GstBuffer* buf = gst_sample_get_buffer(sample);
  if (!buf)
  {
    gst_sample_unref(sample);
    return false;
  }

  GstMapInfo map;
  if (!gst_buffer_map(buf, &map, GST_MAP_READ))
  {
    gst_sample_unref(sample);
    return false;
  }

  gsize& buf_size = map.size;
  guint8*& buf_data = map.data;

  GstClockTime bt = gst_element_get_base_time(gst_pipeline_);

  // Update header information
  auto cur_cinfo = camera_info_manager_->getCameraInfo();

  if (cur_cinfo.height != image_height_ || cur_cinfo.width != image_width_)
  {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "[%s]: Calibration file sensor resolution (%dx%d pixels) doesn't match stream resolution (%dx%d "
                     "pixels)",
                     camera_name_.c_str(), cur_cinfo.height, cur_cinfo.width, image_height_, image_width_);
  }

  // Ensure camera intrinsics, etc are available in published messages
  auto cinfo = std::make_shared<sensor_msgs::msg::CameraInfo>(cur_cinfo);
  if (use_gst_timestamps_)
  {
    cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_USECONDS(buf->pts + bt) / 1e6 + time_offset_);
  }
  else
  {
    cinfo->header.stamp = node_->now();
  }

  cinfo->header.frame_id = frame_id_;

  unsigned int expected_frame_size = 0;

  expected_frame_size = image_width_ * image_height_ * pixel_size_;

  // Complain if the returned buffer is smaller than we expect
  if (buf_size < expected_frame_size)
  {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "[%s]: Image buffer underflow: expected frame to be %u bytes but got only %lu bytes. Make sure "
                     "frames are correctly encoded.",
                     camera_name_.c_str(), expected_frame_size, buf_size);
  }

  if (buf_size > expected_frame_size)
  {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "[%s]: Image buffer overflow: expected frame to be %u bytes but got %lu bytes. Make sure "
                     "frames are correctly encoded.",
                     camera_name_.c_str(), expected_frame_size, buf_size);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    return false;
  }

  // Construct Image message
  auto img = std::make_shared<sensor_msgs::msg::Image>();
  img->header = cinfo->header;

  // Image data and metadata
  img->width = image_width_;
  img->height = image_height_;
  img->encoding = image_encoding_;
  img->is_bigendian = false;
  img->data.resize(expected_frame_size);
  img->step = image_width_ * pixel_size_;

  // Copy only the data we received
  // Since we're publishing shared pointers, we need to copy the image so
  // we can free the buffer allocated by gstreamer
  std::copy(buf_data, (buf_data) + (buf_size), img->data.begin());

  // publish the image/info
  camera_publisher_->publish(img, cinfo);

  gst_buffer_unmap(buf, &map);
  gst_sample_unref(sample);

  return true;
}

void Vision::stop()
{
  stop_requested_ = true;
  is_started_ = false;

  if (gst_pipeline_)
  {
    changePipelineState(GST_STATE_PAUSED);
    changePipelineState(GST_STATE_READY);
    changePipelineState(GST_STATE_NULL);

    gst_object_unref(gst_pipeline_);
    gst_pipeline_ = NULL;
  }
}

bool Vision::changePipelineState(GstState state)
{
  GstStateChangeReturn ret;

  ret = gst_element_set_state(gst_pipeline_, state);
  if (GST_STATE_CHANGE_ASYNC == ret)
  {
    ret = gst_element_get_state(gst_pipeline_, NULL, NULL, STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);
  }

  switch (ret)
  {
    case GST_STATE_CHANGE_SUCCESS:
    case GST_STATE_CHANGE_NO_PREROLL:
      return true;

    case GST_STATE_CHANGE_FAILURE:
      RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to change pipeline state to %s", camera_name_.c_str(),
                   gst_element_state_get_name(state));
      return false;

    case GST_STATE_CHANGE_ASYNC: {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: Failed to change pipeline state to %s (timeout)", camera_name_.c_str(),
                   gst_element_state_get_name(state));
      return false;
    }

    default:
      RCLCPP_ERROR(node_->get_logger(),
                   "[%s]: Unknown state change return value when trying to change pipeline state to %s",
                   camera_name_.c_str(), gst_element_state_get_name(state));
      return false;
  }
}

void Vision::quit()
{
  quit_requested_ = true;

  if (gst_pipeline_)
  {
    // In case quit was requested during a communication loss, unblock gst_app_sink_pull_sample()
    // Note that the EOS event itself doesn't cause any state transitions of the pipeline
    GstEvent* event = NULL;
    event = gst_event_new_eos();
    gst_element_send_event(gst_pipeline_, event);
  }
}

void Vision::run()
{
  // Start the node, raise exceptions if there are any errors on initialization.
  if (!configure())
  {
    throw std::runtime_error("Failed to configure kinova vision node!");
  }

  while (rclcpp::ok() && !quit_requested_)
  {
    if (!is_started_)
    {
      if (!initialize())
      {
        RCLCPP_FATAL(node_->get_logger(), "[%s]: Failed to initialize stream!", camera_name_.c_str());
        break;
      }

      if (start())
      {
        loadCameraInfo();
        is_started_ = true;
        retry_count_ = 0;
      }
      else
      {
        stop();

        retry_count_++;

        RCLCPP_INFO(node_->get_logger(), "[%s]: Trying to connect... (attempt #%d)", camera_name_.c_str(),
                    retry_count_);

        rclcpp::sleep_for(std::chrono::seconds(RETRY_INTERVAL));
      }
    }
    else
    {
      if (!publish())
      {
        RCLCPP_WARN(node_->get_logger(), "[%s]: Could not get frame", camera_name_.c_str());
        stop();
      }
    }
    rclcpp::spin_some(node_);
  }
}
}  // namespace ros_kortex_vision
