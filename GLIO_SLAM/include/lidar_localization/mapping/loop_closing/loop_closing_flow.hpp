/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-29 03:32:14
 */
#ifndef GLIO_SLAM_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define GLIO_SLAM_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "GLIO_SLAM/subscriber/cloud_subscriber.hpp"
#include "GLIO_SLAM/subscriber/key_frame_subscriber.hpp"
// publisher
#include "GLIO_SLAM/publisher/loop_pose_publisher.hpp"
// loop closing
#include "GLIO_SLAM/mapping/loop_closing/loop_closing.hpp"

namespace GLIO_SLAM {
class LoopClosingFlow {
  public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();
    bool Save();
    
  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> key_scan_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<CloudData> key_scan_buff_;
    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> key_gnss_buff_;

    CloudData current_key_scan_;
    KeyFrame current_key_frame_;
    KeyFrame current_key_gnss_;
};
}

#endif