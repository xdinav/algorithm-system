/*
 * @Description: 订阅velocity数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef GLIO_SLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define GLIO_SLAM_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "GLIO_SLAM/sensor_data/velocity_data.hpp"

namespace GLIO_SLAM {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};
}
#endif