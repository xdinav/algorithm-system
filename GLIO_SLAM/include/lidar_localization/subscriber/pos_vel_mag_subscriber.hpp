/*
 * @Description: Subscribe to PosVelMag messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef GLIO_SLAM_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_
#define GLIO_SLAM_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "GLIO_SLAM/PosVelMag.h"
#include "GLIO_SLAM/sensor_data/pos_vel_mag_data.hpp"

namespace GLIO_SLAM {

class PosVelMagSubscriber {
  public:
    PosVelMagSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    PosVelMagSubscriber() = default;
    void ParseData(std::deque<PosVelMagData>& pos_vel_mag_data_buff);

  private:
    void msg_callback(const PosVelMagConstPtr& pos_vel_mag_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelMagData> new_pos_vel_mag_data_;

    std::mutex buff_mutex_; 
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_SUBSCRIBER_POS_VEL_MAG_SUBSCRIBER_HPP_