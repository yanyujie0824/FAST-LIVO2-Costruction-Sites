# FAST-LIVO2 Construction Sites

`rev0_wkb` 是面向施工场景的 FAST-LIVO2 版本，增加了基于体素射线历史与聚类跟踪的动态点滤除，以及基于 LiDAR 几何和视觉跟踪质量的自适应权重调节。

## 环境

已验证 Ubuntu 20.04、ROS Noetic、GCC 9、CMake 3.16、Eigen 3.3.7、PCL 1.10 和 OpenCV 4.2。项目使用 C++17 和 OpenMP。先安装 [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)，然后安装依赖。

```bash
sudo apt update
sudo apt install build-essential cmake git libeigen3-dev libpcl-dev \
  libopencv-dev libboost-thread-dev ros-noetic-ros-base ros-noetic-pcl-ros \
  ros-noetic-cv-bridge ros-noetic-eigen-conversions ros-noetic-image-transport \
  ros-noetic-rosbag ros-noetic-rviz
```

## 安装与编译

先编译 FAST-LIVO2 所用的旧版、非模板 Sophus。下面两处赋值修正用于兼容 Ubuntu 20.04 的 GCC。

```bash
mkdir -p ~/fast_livo2_deps
cd ~/fast_livo2_deps
git clone https://github.com/strasdat/Sophus.git
git -C Sophus checkout a621ff
sed -i -e 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' \
  -e 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' Sophus/sophus/so2.cpp
cmake -S Sophus -B Sophus/build -DCMAKE_BUILD_TYPE=Release
cmake --build Sophus/build --target Sophus --parallel 4
```

在单独的 catkin 工作区中编译本仓库和 Vikit，工作区内只放一份名为 `fast_livo` 的 ROS 包。克隆私有仓库需要 GitHub 访问权限。

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/fast_livo2_ws/src
cd ~/fast_livo2_ws/src
git clone https://github.com/yanyujie0824/FAST-LIVO2-Costruction-Sites.git
git clone https://github.com/xuankuzcr/rpg_vikit.git
git -C rpg_vikit checkout 6c886c8e5d83997806e00294826d528cea3581dd
cd ..
catkin_make -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DSophus_DIR="$HOME/fast_livo2_deps/Sophus/build"
source devel/setup.bash
```

## 运行 HILTI 2022

```bash
roslaunch fast_livo mapping_hesaixt32_hilti22.launch rviz:=false
```

在另一终端加载工作区后回放数据。将路径替换为本地 HILTI 2022 的 ROS bag。

```bash
source ~/fast_livo2_ws/devel/setup.bash
rosbag play --rate 0.65 /path/to/hilti.bag \
  --topics /hesai/pandar /alphasense/imu /alphasense/cam0/image_raw
```

[HILTI 配置](config/HILTI22.yaml)默认开启 `dynamic_filter/enable` 和 `adaptive_weight/enable`，可分别关闭。换用其他传感器时，使用相应的 `launch/` 和 `config/` 文件配置话题和标定。

从工作区根目录运行模块单测。

```bash
catkin_make run_tests_fast_livo_gtest_rev0_wkb_tests
catkin_test_results build/test_results
```

上游项目由 Chunran Zheng 等开发。参见 [FAST-LIVO2 上游仓库](https://github.com/hku-mars/FAST-LIVO2)和 [GPL v2 许可证](LICENSE)。
