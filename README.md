# FAST-LIVO2 Construction Sites

This FAST-LIVO2 variant adds dynamic object filtering and adaptive LiDAR–visual weighting for construction environments.

**Work in progress.** The implementation and configuration may change as development continues.

## Requirements

The build has been verified on Ubuntu 20.04 with ROS Noetic, GCC 9, CMake 3.16, Eigen 3.3.7, PCL 1.10, and OpenCV 4.2. The project uses C++17 and OpenMP. Install ROS Noetic and configure its package repository before installing the dependencies below. [ROS Noetic installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu)

```bash
sudo apt update
sudo apt install build-essential cmake git libeigen3-dev libpcl-dev \
  libopencv-dev libboost-thread-dev ros-noetic-ros-base ros-noetic-pcl-ros \
  ros-noetic-cv-bridge ros-noetic-eigen-conversions ros-noetic-image-transport \
  ros-noetic-rosbag ros-noetic-rviz
```

Install a compatible version of Sophus before building.

## Installation

Use a catkin workspace containing a single ROS package named `fast_livo` and clone this private repository.

```bash
source /opt/ros/noetic/setup.bash
WORKSPACE="$HOME/fast_livo2_ws"
mkdir -p "$WORKSPACE/src"
cd "$WORKSPACE/src"
git clone https://github.com/yanyujie0824/FAST-LIVO2-Costruction-Sites.git
git clone https://github.com/xuankuzcr/rpg_vikit.git

cd "$WORKSPACE"
catkin_make -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

Sensor calibration and topic settings are under [config/](config/), with ROS launch files under [launch/](launch/). The `dynamic_filter/enable` and `adaptive_weight/enable` parameters can be set independently. 

Based on [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) by Chunran Zheng and contributors. See the [GPL v2 license](LICENSE).
