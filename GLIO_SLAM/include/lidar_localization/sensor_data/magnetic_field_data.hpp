/*
 * @Description: magnetic field data
 * @Author: Ge Yao
 * @Date: 2020-11-25 23:07:14
 */
#ifndef GLIO_SLAM_SENSOR_DATA_MAGNETIC_FIELD_DATA_HPP_
#define GLIO_SLAM_SENSOR_DATA_MAGNETIC_FIELD_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace GLIO_SLAM {

class MagneticFieldData {
  public:
    struct MagneticField {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    MagneticField magnetic_field;
  
  public:
    static bool SyncData(
      std::deque<MagneticFieldData>& UnsyncedData, 
      std::deque<MagneticFieldData>& SyncedData, 
      double sync_time
    );
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    void NED2ENU(void);
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_SENSOR_DATA_MAGNETIC_FIELD_DATA_HPP_