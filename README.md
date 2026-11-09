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

## Installation

Use a catkin workspace containing a single ROS package named `fast_livo`. Run the steps below in the same shell. You can change the workspace path to suit your setup.

### 1. Build Sophus

`a621ff` is the short Git commit ID for the legacy, non-templated Sophus API used by FAST-LIVO2 and Vikit. The checkout pins that compatible version. The two source edits below make its `std::complex` assignments compatible with GCC 9. [Sophus commit](https://github.com/strasdat/Sophus/commit/a621ff2e) · [Upstream build instructions](https://github.com/hku-mars/FAST-LIVO2#23-sophus)

```bash
WORKSPACE="$HOME/fast_livo2_ws"
SOPHUS_SOURCE="$WORKSPACE/deps/Sophus"
SOPHUS_DIR="$SOPHUS_SOURCE/build"

mkdir -p "$WORKSPACE/src" "$WORKSPACE/deps"
git clone https://github.com/strasdat/Sophus.git "$SOPHUS_SOURCE"
git -C "$SOPHUS_SOURCE" checkout --detach a621ff
sed -i -e 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' \
  -e 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' \
  "$SOPHUS_SOURCE/sophus/so2.cpp"
cmake -S "$SOPHUS_SOURCE" -B "$SOPHUS_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SOPHUS_DIR" --target Sophus --parallel 4
```

If you already have a compatible Sophus build, set `SOPHUS_DIR` to the directory containing its `SophusConfig.cmake` and use that build in the next step.

### 2. Build the ROS workspace

Use your authenticated GitHub account to clone this private repository.

```bash
source /opt/ros/noetic/setup.bash
cd "$WORKSPACE/src"
git clone https://github.com/yanyujie0824/FAST-LIVO2-Costruction-Sites.git
git clone https://github.com/xuankuzcr/rpg_vikit.git
git -C rpg_vikit checkout --detach 6c886c8e5d83997806e00294826d528cea3581dd

cd "$WORKSPACE"
catkin_make -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3 -DSophus_DIR="$SOPHUS_DIR"
source devel/setup.bash
```

Sensor calibration and topic settings are under [config/](config/), with ROS launch files under [launch/](launch/). The `dynamic_filter/enable` and `adaptive_weight/enable` parameters can be set independently. The node creates its runtime output directories under `Log/` as needed.

Based on [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) by Chunran Zheng and contributors. See the [GPL v2 license](LICENSE).
