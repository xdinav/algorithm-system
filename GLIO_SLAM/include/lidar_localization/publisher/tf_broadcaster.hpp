/*
 * @Description: 发布tf的类
 * @Author: Ren Qian
 * @Date: 2020-03-05 15:23:26
 */

#ifndef GLIO_SLAM_PUBLISHER_TF_BROADCASTER_HPP_
#define GLIO_SLAM_PUBLISHER_TF_BROADCASTER_HPP_

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace GLIO_SLAM {
class TFBroadCaster {
  public:
    TFBroadCaster(std::string frame_id, std::string child_frame_id);
    TFBroadCaster() = default;
    void SendTransform(Eigen::Matrix4f pose, double time);
  protected:
    tf::StampedTransform transform_;
    tf::TransformBroadcaster broadcaster_;
};
}
#endif