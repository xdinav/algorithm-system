/*
 * @Description: Subscribe to PosVel messages
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */
#ifndef GLIO_SLAM_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_
#define GLIO_SLAM_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "GLIO_SLAM/PosVel.h"
#include "GLIO_SLAM/sensor_data/pos_vel_data.hpp"

namespace GLIO_SLAM {

class PosVelSubscriber {
  public:
    PosVelSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    PosVelSubscriber() = default;
    void ParseData(std::deque<PosVelData>& pos_vel_data_buff);

  private:
    void msg_callback(const PosVelConstPtr& pos_vel_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelData> new_pos_vel_data_;

    std::mutex buff_mutex_; 
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_SUBSCRIBER_POS_VEL_SUBSCRIBER_HPP_