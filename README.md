# GLIO_SLAM: （Beta version）

## 概述

&emsp;&emsp;GLIO_SLAM是一种基于优化的多传感器紧耦合SLAM算法，融合GNSS、激光雷达（LiDAR）和惯性测量单元（IMU）数据，实现了高精度的位姿估计与三维点云地图构建。


# 算法安装

## 系统要求
- **操作系统**: Ubuntu 18.04 (ROS Melodic) / 20.04 (ROS Noetic)
- **编译工具**: CMake ≥ 3.10, GCC ≥ 7.5

---

## 快速安装

### 1. 安装基础依赖
bash
sudo apt-get update \
sudo apt-get install -y \
libgoogle-glog-dev \
libgflags-dev \
libprotobuf-dev \
protobuf-compiler 
### 2. 安装库依赖
&emsp;&emsp;PCL (Point Cloud Library)、
g2o (通用图优化库)、
Sophus (李群/李代数库)、
GeographicLib (地理坐标库)
### 3. 安装ROS
- **Ubuntu 18.04 (ROS Melodic)**
- **Ubuntu 20.04 (ROS Noetic)**
bash
sudo apt install ros-melodic-desktop-full \
sudo apt install ros-noetic-desktop-full \
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
source ~/.bashrc
### 4. 克隆项目并编译
bash
mkdir -p catkin_ws/src && cd catkin_ws/src \
git clone https://github.com/yourusername/algorithm-system.git \
cd .. \
catkin_make -j$(nproc)

&emsp;&emsp;本次开源的GLIO_SLAM （Beta version）主要包括卫星导航与惯性导航和LiDAR融合部分，后端优化使用了g20优化库，具体开源的功能包括：
1. 惯性预积分精化处理

2. NDT前端匹配算法

3. 非线性后端优化

4. 传感器数据格式接口

5. RVIZ节点显示

&emsp;&emsp;此为实验内测版，我们将持续完善。后面会陆续开源更多传感器融合，包括RGB/红外相机、固态激光雷达、高精地图、轮速等传感器。
算法整理仓促，如有不足之处请各位指正！




