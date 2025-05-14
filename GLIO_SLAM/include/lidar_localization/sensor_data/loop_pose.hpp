/*
 * @Description: 关键帧之间的相对位姿，用于闭环检测
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef GLIO_SLAM_SENSOR_DATA_LOOP_POSE_HPP_
#define GLIO_SLAM_SENSOR_DATA_LOOP_POSE_HPP_

#include <Eigen/Dense>

namespace GLIO_SLAM {
class LoopPose {
  public:
    double time = 0.0;
    unsigned int index0 = 0;
    unsigned int index1 = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif