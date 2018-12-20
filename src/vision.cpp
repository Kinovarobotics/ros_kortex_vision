#include "vision.h"
#include "constants.h"

#include <stdio.h>

const int CAM_INFO_DEFAULT_URL_MAX_SIZE = 128;

Vision::Vision(ros::NodeHandle nh_camera, ros::NodeHandle nh_private)
  : nh_(nh_camera)
  , nh_private_(nh_private)
  , camera_info_manager_(nh_camera)
  , image_transport_(nh_camera)
  , gst_pipeline_(NULL)
  , gst_sink_(NULL)
  , base_frame_id_(DEFAULT_BASE_FRAME_ID)
  , is_started_(false)
  , stop_requested_(false)
  , quit_requested_(false)
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
}

bool Vision::configure()
{
  std::string streamConfig_rosparam = "";
  bool bStreamConfigDefined = false;
  char* streamConfig_env = NULL;

  bStreamConfigDefined = nh_private_.getParam("stream_config", streamConfig_rosparam);

  if (!bStreamConfigDefined)
  {
    ROS_FATAL("'stream_config' rosparam is not set. This is needed to set up a gstreamer pipeline.");
    return false;
  }
  else
  {
    camera_config_ = streamConfig_rosparam;
    ROS_INFO_STREAM("Using gstreamer config from rosparam: \"" << camera_config_ << "\"");
  }

  std::string camera_type;
  nh_private_.getParam("camera_type", camera_type);

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
    ROS_FATAL("camera_type param not set! This param is required for this node to run! Exiting..");
    return false;
  }

  pixel_size_ = sensor_msgs::image_encodings::numChannels(image_encoding_) *
                (sensor_msgs::image_encodings::bitDepth(image_encoding_) / 8);

  if (nh_private_.getParam("camera_name", camera_name_))
  {
    camera_info_manager_.setCameraName(camera_name_);
  }
  else
  {
    camera_name_ = "Camera";
    ROS_WARN_STREAM("camera_name param not found. Using default value: " << camera_name_);
    camera_info_manager_.setCameraName(camera_name_);
  }

  if (!nh_private_.getParam("frame_id", frame_id_))
  {
    frame_id_ = "/camera_frame";
    ROS_WARN_STREAM("No camera frame_id set, using frame \"" << frame_id_ << "\".");
    nh_private_.setParam("frame_id", frame_id_);
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
    ROS_FATAL_STREAM(error->message);
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
      ROS_FATAL("[%s]: gst_bin_add() failed", camera_name_.c_str());
      gst_object_unref(outelement);
      gst_object_unref(gst_pipeline_);
      gst_pipeline_ = NULL;
      return false;
    }

    if (!gst_element_link(outelement, gst_sink_))
    {
      ROS_FATAL("[%s]: gstreamer: cannot link outelement(\"%s\") -> sink\n", camera_name_.c_str(),
                gst_element_get_name(outelement));
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
      ROS_FATAL("[%s]: gstreamer: cannot link launchpipe -> sink", camera_name_.c_str());
      gst_object_unref(gst_pipeline_);
      gst_pipeline_ = NULL;
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock* clock = gst_system_clock_obtain();
  ros::Time now = ros::Time::now();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now.toSec() - GST_TIME_AS_USECONDS(ct) / 1e6;

  gst_element_set_state(gst_pipeline_, GST_STATE_PAUSED);

  if (gst_element_get_state(gst_pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE)
  {
    ROS_FATAL("[%s]: Failed to PAUSE stream, check your gstreamer configuration.", camera_name_.c_str());
    return false;
  }
  else
  {
    ROS_DEBUG("[%s]: Stream is PAUSED", camera_name_.c_str());
  }

  if (is_first_initialize_)
  {
    // Create ROS camera interface
    is_first_initialize_ = false;
    camera_publisher_ = image_transport_.advertiseCamera("image_raw", 1);
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
      ROS_ERROR("[%s]: Failed to start stream", camera_name_.c_str());
      return false;

    case GST_STATE_CHANGE_ASYNC:
    {
      ret = gst_element_get_state(gst_pipeline_, NULL, NULL, STATE_CHANGE_ASYNC_TIMEOUT * GST_SECOND);

      switch (ret)
      {
        case GST_STATE_CHANGE_FAILURE:
          ROS_ERROR("[%s]: Failed to start stream", camera_name_.c_str());
          return false;

        case GST_STATE_CHANGE_ASYNC:
          ROS_ERROR("[%s]: Failed to start stream (timeout)", camera_name_.c_str());
          return false;
      }
    }

    case GST_STATE_CHANGE_SUCCESS:
      ROS_INFO("[%s]: Stream started", camera_name_.c_str());
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
  nh_private_.getParam("camera_info_url_user", camera_info_);
  if (camera_info_.empty())
  {
    ROS_INFO("[%s]: Custom camera information file not set, using default one based on sensor resolution",
             camera_name_.c_str());

    nh_private_.getParam("camera_info_url_default", cam_info_default);
    if (!cam_info_default.empty())
    {
      snprintf(cam_info_default_resolved, CAM_INFO_DEFAULT_URL_MAX_SIZE, cam_info_default.c_str(),
               image_width_, image_height_);
      camera_info_.assign(cam_info_default_resolved);
    }
    else
    {
      ROS_WARN("[%s]: Parameter 'camera_info_url_default' not found or empty.", camera_name_.c_str());
    }
  }

  if (camera_info_manager_.validateURL(camera_info_))
  {
    if (camera_info_manager_.loadCameraInfo(camera_info_))
    {
      ROS_INFO("[%s]: Loaded camera calibration from '%s'", camera_name_.c_str(), camera_info_.c_str());
    }
    else
    {
      ROS_WARN("[%s]: Camera info at '%s' not found. Using an uncalibrated config.", camera_name_.c_str(),
               camera_info_.c_str());
    }
  }
  else
  {
    ROS_WARN("[%s]: Camera info url syntax not supported.", camera_name_.c_str());
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
  sensor_msgs::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
  sensor_msgs::CameraInfoPtr cinfo;

  if (cur_cinfo.height != image_height_ || cur_cinfo.width != image_width_)
  {
    ROS_WARN_ONCE("[%s]: Calibration file sensor resolution (%dx%d pixels) doesn't match stream resolution (%dx%d "
                  "pixels)",
                  camera_name_.c_str(), cur_cinfo.height, cur_cinfo.width, image_height_, image_width_);
  }

  cinfo.reset(new sensor_msgs::CameraInfo(cur_cinfo));
  cinfo->height = image_height_;
  cinfo->width = image_width_;

  if (use_gst_timestamps_)
  {
    cinfo->header.stamp = ros::Time(GST_TIME_AS_USECONDS(buf->pts + bt) / 1e6 + time_offset_);
  }
  else
  {
    cinfo->header.stamp = ros::Time::now();
  }

  cinfo->header.frame_id = frame_id_;

  unsigned int expected_frame_size = 0;

  expected_frame_size = image_width_ * image_height_ * pixel_size_;

  // Complain if the returned buffer is smaller than we expect
  if (buf_size < expected_frame_size)
  {
    ROS_WARN_ONCE("[%s]: Image buffer underflow: expected frame to be %u bytes but got only %lu bytes. Make sure "
                  "frames are correctly encoded.",
                  camera_name_.c_str(), expected_frame_size, buf_size);
  }

  if (buf_size > expected_frame_size)
  {
    ROS_WARN_ONCE("[%s]: Image buffer overflow: expected frame to be %u bytes but got %lu bytes. Make sure "
                  "frames are correctly encoded.",
                  camera_name_.c_str(), expected_frame_size, buf_size);

    gst_buffer_unmap(buf, &map);
    gst_sample_unref(sample);

    return false;
  }

  // Construct Image message
  sensor_msgs::ImagePtr img(new sensor_msgs::Image());
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
  camera_publisher_.publish(img, cinfo);

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
      ROS_ERROR("[%s]: Failed to change pipeline state to %s",
                camera_name_.c_str(), gst_element_state_get_name(state));
      return false;

    case GST_STATE_CHANGE_ASYNC:
    {
      ROS_ERROR("[%s]: Failed to change pipeline state to %s (timeout)",
                camera_name_.c_str(), gst_element_state_get_name(state));
      return false;
    }

    default:
      ROS_ERROR("[%s]: Unknown state change return value when trying to change pipeline state to %s",
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
    GstEvent *event = NULL;
    event = gst_event_new_eos();
    gst_element_send_event(gst_pipeline_, event);
  }
}

void Vision::run()
{
  if (!configure())
  {
    ROS_FATAL("Failed to configure kinova vision node!");
    return;
  }

  while (ros::ok() && !quit_requested_)
  {
    if (!is_started_)
    {
      if (!initialize())
      {
        ROS_FATAL("[%s]: Failed to initialize stream!", camera_name_.c_str());
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

        ROS_INFO("[%s]: Trying to connect... (attempt #%d)", camera_name_.c_str(), retry_count_);

        ros::Duration(RETRY_INTERVAL).sleep();
      }
    }
    else
    {
      if (!publish())
      {
        ROS_WARN("[%s]: Could not get frame", camera_name_.c_str());
        stop();
      }
    }

    ros::spinOnce();
  }

  stop();
}