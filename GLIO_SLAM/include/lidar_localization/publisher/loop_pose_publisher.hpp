/*
 * @Description: 发送闭环检测的相对位姿
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 */
#ifndef GLIO_SLAM_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_
#define GLIO_SLAM_PUBLISHER_LOOP_POSE_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "GLIO_SLAM/sensor_data/loop_pose.hpp"

namespace GLIO_SLAM {
class LoopPosePublisher {
  public:
    LoopPosePublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    LoopPosePublisher() = default;

    void Publish(LoopPose& loop_pose);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif