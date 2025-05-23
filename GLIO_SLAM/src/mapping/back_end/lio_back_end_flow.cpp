/*
 * @Description: LIO mapping backend workflow, implementation
 * @Author: Ge Yao
 * @Date: 2020-11-29 15:47:49
 */

#include "GLIO_SLAM/mapping/back_end/lio_back_end_flow.hpp"

#include "glog/logging.h"

#include "GLIO_SLAM/tools/file_manager.hpp"
#include "GLIO_SLAM/global_defination/global_defination.h"

namespace GLIO_SLAM {

LIOBackEndFlow::LIOBackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    //
    // subscribers:
    //
    // a. lidar scan, key frame measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    // b. lidar odometry:
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, odom_topic, 100000);
    // c. GNSS position:
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    // d. loop closure detection:
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 100000);

    // e. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
    // // f. odometer measurement, for pre-integration:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/imu/vel", 1000000);
    // f. lidar to imu tf:
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");
    //addnew g.gnss
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);
    key_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/key_scan", "/velo_link", 100);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "/map", 100);

    back_end_ptr_ = std::make_shared<LIOBackEnd>();
}

bool LIOBackEndFlow::Run() {
    if (!InitCalibration())
    {
        return false;
    }
    // load messages into buffer:
    if (!ReadData())
        return false;

    // add loop poses for graph optimization:
    InsertLoopClosurePose();

    while(HasData()) {
        // make sure undistorted Velodyne measurement -- lidar pose in map frame -- lidar odometry are synced:
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool LIOBackEndFlow::InitCalibration()
{
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;

    if (!calibration_received)
    {
        if (lidar_to_imu_ptr_->LookupData1(lidar_to_imu_))
        {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool LIOBackEndFlow::ForceOptimize() {
    static std::deque<KeyFrame> optimized_key_frames;

    back_end_ptr_->ForceOptimize();

    if ( back_end_ptr_->HasNewOptimized() ) {
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool LIOBackEndFlow::SaveOptimizedOdometry() {
    back_end_ptr_ -> SaveOptimizedPose();

    return true;
}

bool LIOBackEndFlow::ReadData() {
    // a. lidar scan, key frame measurement:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // b. lidar odometry:
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    // c. GNSS position:
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    // d. loop closure detection:
    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);
    // e. IMU measurement, for pre-integration:
    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
    // f. odometer measurement, for pre-integration:
    velocity_sub_ptr_->ParseData(velocity_data_buff_);
    //addnew
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

/**
 * @brief  add loop closure for backend optimization
 * @param  void
 * @return true if success false otherwise
 */
bool LIOBackEndFlow::InsertLoopClosurePose() {
    while (loop_pose_data_buff_.size() > 0) {
        back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
        loop_pose_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::HasData() {
    if (
        cloud_data_buff_.empty() ||
        laser_odom_data_buff_.empty() ||
        gnss_pose_data_buff_.empty() ||
        imu_synced_data_buff_.empty()
    ) {
        return false;
    }

    return true;
}

bool LIOBackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();


    current_imu_data_ = imu_synced_data_buff_.front();

    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;

    if ( diff_laser_time < -0.05 || diff_gnss_time < -0.05 || diff_imu_time < -0.05 ) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if ( diff_laser_time > 0.05 ) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    if ( diff_gnss_time > 0.05 ) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if ( diff_imu_time > 0.05 ) {
        imu_synced_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    imu_synced_data_buff_.pop_front();

    return true;
}

bool LIOBackEndFlow::UpdateIMUPreIntegration(void) {
    while (
        !imu_raw_data_buff_.empty() &&
        imu_raw_data_buff_.front().time < current_imu_data_.time && velocity_data_buff_.front().time < current_imu_data_.time && gnss_data_buff_.front().time < current_imu_data_.time)
    {
        // new add
        //newadd
        VelocityData current_velocity_raw_data_=velocity_data_buff_.front();
        IMUData current_imu_raw_data_ = imu_raw_data_buff_.front();
        GNSSData current_gnss_raw_data_=gnss_data_buff_.front();
        Eigen::Quaterniond q(current_imu_raw_data_.orientation.w, current_imu_raw_data_.orientation.x, current_imu_raw_data_.orientation.y, current_imu_raw_data_.orientation.z);
        Eigen::Quaterniond q_tmp(lidar_to_imu_.block<3, 3>(0, 0).cast<double>());
        Eigen::Quaterniond res = q_tmp * q;
        current_imu_raw_data_.orientation.w = res.w();
        current_imu_raw_data_.orientation.x = res.x();
        current_imu_raw_data_.orientation.y = res.y();
        current_imu_raw_data_.orientation.z = res.z();
        current_imu_raw_data_.orientation.Normlize();
        if (!back_end_ptr_->UpdateIMUPreIntegration(current_imu_raw_data_,current_velocity_raw_data_,current_gnss_raw_data_))
        {
            break;
        }
        imu_raw_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::UpdateOdoPreIntegration(void) {
    while (
        !velocity_data_buff_.empty() &&
        velocity_data_buff_.front().time < current_imu_data_.time &&
        back_end_ptr_->UpdateOdoPreIntegration(velocity_data_buff_.front())
    ) {
        velocity_data_buff_.pop_front();
    }

    return true;
}

bool LIOBackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if (!odometry_inited) {
        // the origin of lidar odometry frame in map frame as init pose:
        //odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();
        odom_init_pose = current_laser_odom_data_.pose.inverse();

        odometry_inited = true;
    }

    // update IMU pre-integration:
    UpdateIMUPreIntegration();

    // // update odo pre-integration:
    // UpdateOdoPreIntegration();

    // current lidar odometry in map frame:
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    // optimization is carried out in map frame:
    return back_end_ptr_->Update(
        current_cloud_data_,
        current_laser_odom_data_,
        current_gnss_pose_data_,
        current_imu_data_
    );
}

bool LIOBackEndFlow::PublishData() {
    Eigen::Matrix3f tmp;
    tmp << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    current_laser_odom_data_.pose.block<3, 3>(0, 0) = tmp * current_laser_odom_data_.pose.block<3, 3>(0, 0);
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        CloudData key_scan;

        back_end_ptr_->GetLatestKeyScan(key_scan);
        key_scan_pub_ptr_->Publish(key_scan.cloud_ptr, key_scan.time);

        KeyFrame key_frame;

        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}

} // namespace GLIO_SLAM
