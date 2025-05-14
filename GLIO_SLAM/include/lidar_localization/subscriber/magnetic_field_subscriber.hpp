/*
 * @Description: Subscribe to magnetic field measurement
 * @Author: Ge Yao
 * @Date: 2020-11-25 23:07:14
 */

#ifndef GLIO_SLAM_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_
#define GLIO_SLAM_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/MagneticField.h"

#include "GLIO_SLAM/sensor_data/magnetic_field_data.hpp"

namespace GLIO_SLAM {

class MagneticFieldSubscriber {
  public:
    MagneticFieldSubscriber(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      size_t buff_size
    );
    MagneticFieldSubscriber() = default;
    void ParseData(std::deque<MagneticFieldData>& mag_field_data_buff);

  private:
    void msg_callback(const sensor_msgs::MagneticFieldConstPtr& mag_field_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<MagneticFieldData> new_mag_field_data_;

    std::mutex buff_mutex_; 
};

} // namespace 

#endif // GLIO_SLAM_SUBSCRIBER_MAGNETIC_FIELD_SUBSCRIBER_HPP_