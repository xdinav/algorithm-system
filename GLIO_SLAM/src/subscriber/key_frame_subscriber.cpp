/*
 * @Description: 订阅 key frame 数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "GLIO_SLAM/subscriber/key_frame_subscriber.hpp"
#include "glog/logging.h"

namespace GLIO_SLAM{
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback, this);
}

void KeyFrameSubscriber::msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr) {
    buff_mutex_.lock();
    KeyFrame key_frame;
    key_frame.time = key_frame_msg_ptr->header.stamp.toSec();
    key_frame.index = (unsigned int)key_frame_msg_ptr->pose.covariance[0];

    key_frame.pose(0,3) = key_frame_msg_ptr->pose.pose.position.x;
    key_frame.pose(1,3) = key_frame_msg_ptr->pose.pose.position.y;
    key_frame.pose(2,3) = key_frame_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;

    Eigen::Matrix3f tmp;
    tmp << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Quaternionf q_tmp(tmp.transpose());
    q = q_tmp * q;

    q.x() = key_frame_msg_ptr->pose.pose.orientation.x;
    q.y() = key_frame_msg_ptr->pose.pose.orientation.y;
    q.z() = key_frame_msg_ptr->pose.pose.orientation.z;
    q.w() = key_frame_msg_ptr->pose.pose.orientation.w;
    key_frame.pose.block<3,3>(0,0) = q.matrix();

    new_key_frame_.push_back(key_frame);
    buff_mutex_.unlock();
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrame>& key_frame_buff) {
    buff_mutex_.lock();
    if (new_key_frame_.size() > 0) {
        key_frame_buff.insert(key_frame_buff.end(), new_key_frame_.begin(), new_key_frame_.end());
        new_key_frame_.clear();
    }
    buff_mutex_.unlock();
}
}
