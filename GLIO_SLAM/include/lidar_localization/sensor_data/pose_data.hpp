/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 */
#ifndef GLIO_SLAM_SENSOR_DATA_POSE_DATA_HPP_
#define GLIO_SLAM_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

#include "GLIO_SLAM/sensor_data/velocity_data.hpp"

namespace GLIO_SLAM {

class PoseData {
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    struct {
      Eigen::Vector3f v = Eigen::Vector3f::Zero();
      Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;
    
  public:
    Eigen::Quaternionf GetQuaternion();

    void GetVelocityData(VelocityData &velocity_data) const;
};

}

#endif