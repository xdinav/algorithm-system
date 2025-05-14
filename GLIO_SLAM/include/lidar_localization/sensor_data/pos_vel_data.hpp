/*
 * @Description: synced GNSS-odo measurements as PosVelData
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#ifndef GLIO_SLAM_SENSOR_DATA_POS_VEL_DATA_HPP_
#define GLIO_SLAM_SENSOR_DATA_POS_VEL_DATA_HPP_

#include <string>

#include <Eigen/Dense>

namespace GLIO_SLAM {

class PosVelData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_SENSOR_DATA_POS_VEL_DATA_HPP_