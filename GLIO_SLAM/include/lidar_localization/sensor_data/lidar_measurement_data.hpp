/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:17:49
 */
#ifndef GLIO_SLAM_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_
#define GLIO_SLAM_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_

#include "GLIO_SLAM/sensor_data/cloud_data.hpp"
#include "GLIO_SLAM/sensor_data/imu_data.hpp"
#include "GLIO_SLAM/sensor_data/pose_data.hpp"

namespace GLIO_SLAM {

class LidarMeasurementData {
  public:
    double time = 0.0;

    CloudData point_cloud;
    IMUData imu;
    PoseData gnss_odometry;
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_SENSOR_DATA_LIDAR_MEASUREMENT_DATA_HPP_