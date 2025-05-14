/*
 * @Description: synced PosVelMagData publisher
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
#ifndef GLIO_SLAM_PUBLISHER_POS_VEL_MAG_PUBLISHER_HPP_
#define GLIO_SLAM_PUBLISHER_POS_VEL_MAG_PUBLISHER_HPP_

#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "GLIO_SLAM/sensor_data/pos_vel_mag_data.hpp"

#include "GLIO_SLAM/PosVelMag.h"

namespace GLIO_SLAM {

class PosVelMagPublisher {
  public:
    PosVelMagPublisher(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      std::string base_frame_id,
      std::string child_frame_id,
      int buff_size
    );
    PosVelMagPublisher() = default;

    void Publish(const PosVelMagData &pos_vel_mag_data, const double &time);
    void Publish(const PosVelMagData &pos_vel_mag_data);

    bool HasSubscribers();

  private:
    void PublishData(
      const PosVelMagData &pos_vel_mag_data, 
      ros::Time time
    );

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    PosVelMag pos_vel_mag_msg_;
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_PUBLISHER_POS_VEL_MAG_PUBLISHER_HPP_