/*
 * @Description: velocity 数据
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef GLIO_SLAM_SENSOR_DATA_VELOCITY_DATA_HPP_
#define GLIO_SLAM_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace GLIO_SLAM {
class VelocityData {
  public:
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
  
  public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    void NED2ENU(void);
};
}
#endif