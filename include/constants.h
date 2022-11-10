#ifndef KINOVA_VISION_CONSTANTS_H
#define KINOVA_VISION_CONSTANTS_H

#include <string>

const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
const std::string DEFAULT_COLOR_FRAME_ID = "camera_color_frame";

const float RETRY_INTERVAL = 3.0;
const unsigned int STATE_CHANGE_ASYNC_TIMEOUT = 15;
const unsigned int APP_SINK_BUFFER_COUNT = 5;

#endif
