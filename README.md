# Kinova Vision module package

## Overview
This ROS 2 package provides helper methods and launch scripts to access the Kinova Vision module depth and color streams.


## Installation (using colcon)
The following instructions are for ROS 2, tested on Humble on Ubuntu 22.02.

### Building from Source

#### Dependencies

##### GStreamer packages
* gstreamer1.0-tools
* libgstreamer1.0-libav
* libgstreamer1.0-dev 
* libgstreamer-plugins-base1.0-dev 
* libgstreamer-plugins-good1.0-dev
* gstreamer1.0-plugins-good
* gstreamer1.0-plugins-base

```sh
sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base
```

##### ROS package
* rgbd_launch

```bash
sudo apt-get install ros-kinetic-rgbd-launch
```

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package.

1. Create a workspace
```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src/
```
2. Clone this git repo into `~/colcon_ws/src`
```bash
git clone -b ros2 https://github.com/PickNikRobotics/ros2_kortex_vision.git
```
```bash
cd ~/colcon_ws/
rosdep install --from-paths . --ignore-src -r -y
colcon build
```

## Usage

### Start kinova_vision node
```bash
source ~/colcon_ws/setup.bash
ros2 launch kinova_vision kinova_vision.launch.py 
```
The launch file provides arguments for launching depth, color, or registered depth images, as well as overriding other parameters.
For example, to only launch the color node,

```bash
source ~/colcon_ws/setup.bash
ros2 launch kinova_vision kinova_vision.launch.py launch_depth:=false
```

Additional information is available below.

### Start rviz to view both cameras

*TODO* Add `rviz2` save configurations and instructions for viewing camera streams.

### Specifying launch options
It's possible to override the default argument values when launching the **kinova_vision** node.

Arguments are set using the following syntax: `<argument>:=<value>`.

For instance, the default value of the `device` argument can be overridden to specify another IP address.
```bash
ros2 launch kinova_vision kinova_vision.launch.py device:=10.20.0.100
```

#### Additional information on arguments `color_camera_info_url` and `depth_camera_info_url`

These arguments specify the custom camera information file to use instead of the default camera information file.

The file is specified via a specific URL syntax, using either of these two formats:

`package://<package_name>/relative/path/to/file`

`file:///absolute/path/to/file`

For example:
```bash
ros2 launch kinova_vision kinova_vision.launch.py color_camera_info_url:=file:///home/user/custom_color_calib_1280x720.ini depth_camera_info_url:=file:///home/user/custom_depth_calib_480x270.ini
```

A custom camera information file is typically created from a default information file (refer to *launch/calibration/default_\*.ini*). Then, one simply needs to adjust the proper matrices.

The following matrices need to be adjusted with the proper values for `FX`, `FY`, `PPX`, `PPY`:
```
camera matrix
FX 0.00000 PPX
0.00000 FY PPY
0.00000 0.00000 1.00000

projection
FX 0.00000 PPX 0.00000 
0.00000 FY PPY 0.00000 
0.00000 0.00000 1.00000 0.00000 
```

The values for `FX`, `FY`, `PPX`, `PPY` can be obtained via the Vision module API. They represent the _focal length_ and the _principal point offset_ in both the _x_ and _y_ coordinates.

<a name="nodes"></a>
## Nodes

### kinova_vision_color

This node publishes the raw stream and the meta information of the color camera.

#### Subscribed Topics

None

#### Published Topics

* **`/camera/color/camera_info`** ([sensor_msgs/CameraInfo])

    Color camera calibration and meta information.

* **`/camera/color/image_raw`** ([sensor_msgs/Image])

    Color camera raw image (RGB8 encoding).

### kinova_vision_depth

This node publishes the raw stream and the meta information of the depth camera.

#### Subscribed Topics

None

#### Published Topics

* **`/camera/depth/camera_info`** ([sensor_msgs/CameraInfo])

    Depth camera calibration and meta information

* **`/camera/depth/image_raw`** ([sensor_msgs/Image])

    Depth camera raw image (millimeters - 16UC1 encoding).

### camera_color_tf_publisher

This node publishes the static coordinate transformation between the color camera frame (*camera_color_frame*) and the camera link frame (*camera_link*).

#### Subscribed Topics

None

#### Published Topics

* **`/tf_static`** ([tf2_msgs/TFMessage])

    Color camera frame static transformation

### camera_depth_tf_publisher

This node publishes the static coordinate transformation between the depth camera frame (*camera_depth_frame*) and the camera link frame (*camera_link*).

#### Subscribed Topics

None

#### Published Topics

* **`/tf_static`** ([tf2_msgs/TFMessage])

    Depth camera frame static transformation

### camera_nodelet_manager

This node uses the [image_proc] package to create a nodelet graph, transforming raw data from the device driver into point clouds, rectified images, and other products suitable for processing and visualization.
To include these,

```bash
ros2 launch kinova_vision kinova_vision.launch.py depth_registration:=true
```

#### Subscribed Topics

* **`/camera/color/camera_info`** ([sensor_msgs/CameraInfo])

    Color camera calibration and meta information

* **`/camera/color/image_raw`** ([sensor_msgs/Image])

    Color camera raw image

* **`/camera/depth/camera_info`** ([sensor_msgs/CameraInfo])

    Depth camera calibration and meta information

* **`/camera/depth/image_raw`** ([sensor_msgs/Image])

    Depth camera raw image

#### Published Topics

* **`/camera/depth_registered/camera_info`** ([sensor_msgs/CameraInfo])

    Color camera calibration and meta information

* **`/camera/depth_registered/image_rect`** ([sensor_msgs/Image])

    Depth rectified image (millimeters - 16UC1 encoding)

* **`/camera/depth/color/points`** ([sensor_msgs/PointCloud2])

    Depth camera point cloud data with color information (RGBD)


[image_view]: http://wiki.ros.org/image_view
[camera_info_manager]: http://wiki.ros.org/camera_info_manager
[sensor_msgs/CameraInfo]: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[tf2_msgs/TFMessage]: http://docs.ros.org/api/tf2_msgs/html/msg/TFMessage.html
[image_proc]: http://wiki.ros.org/image_proc
