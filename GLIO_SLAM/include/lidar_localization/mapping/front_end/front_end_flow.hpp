/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef GLIO_SLAM_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define GLIO_SLAM_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "GLIO_SLAM/subscriber/cloud_subscriber.hpp"
#include "GLIO_SLAM/publisher/odometry_publisher.hpp"
#include "GLIO_SLAM/mapping/front_end/front_end.hpp"

namespace GLIO_SLAM {
class FrontEndFlow {
  public:
    FrontEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;

    CloudData current_cloud_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif
