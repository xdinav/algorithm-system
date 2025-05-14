/*
 * @Description: Publish synced Lidar-IMU-GNSS measurement
 * @Author: Ge Yao
 * @Date: 2020-11-12 15:14:07
 */

#ifndef GLIO_SLAM_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_HPP_
#define GLIO_SLAM_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_HPP_

#include <GLIO_SLAM/LidarMeasurement.h>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "GLIO_SLAM/sensor_data/cloud_data.hpp"
#include "GLIO_SLAM/sensor_data/imu_data.hpp"
#include "GLIO_SLAM/sensor_data/velocity_data.hpp"

namespace GLIO_SLAM {

class LidarMeasurementPublisher {
  public:
    LidarMeasurementPublisher(
      ros::NodeHandle& nh,
      std::string topic_name,
      std::string lidar_frame_id,
      std::string imu_frame_id,
      std::string gnss_odometry_pos_frame_id,
      std::string gnss_odometry_vel_frame_id,
      size_t buff_size
    );
    LidarMeasurementPublisher() = default;

    void Publish(
      double time,
      const CloudData::CLOUD_PTR& cloud_ptr_input,
      const IMUData &imu_data,
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data 
    );
    void Publish(
      const CloudData::CLOUD_PTR& cloud_ptr_input,
      const IMUData &imu_data,
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data
    );

    bool HasSubscribers();
  
  private:
    void SetPointCloud(
        const ros::Time &time,
        const CloudData::CLOUD_PTR &cloud_data_ptr,
        sensor_msgs::PointCloud2 &point_cloud
    );
    void SetIMU(
        const ros::Time &time,
        const IMUData &imu_data,
        sensor_msgs::Imu &imu
    );
    void SetGNSSOdometry(
        const ros::Time &time,
        const Eigen::Matrix4f& transform_matrix,
        const VelocityData &velocity_data, 
        nav_msgs::Odometry &gnss_odometry 
    );
    void PublishData(
      const ros::Time &time,
      const CloudData::CLOUD_PTR& cloud_ptr_input,
      const IMUData &imu_data, 
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data
    );

  private:
    ros::NodeHandle nh_;

    std::string lidar_frame_id_;
    std::string imu_frame_id_;
    std::string gnss_odometry_pos_frame_id_;
    std::string gnss_odometry_vel_frame_id_;

    LidarMeasurement lidar_measurement_;
    
    ros::Publisher publisher_;
};

} // namespace GLIO_SLAM

#endif // GLIO_SLAM_PUBLISHER_LIDAR_MEASUREMENT_PUBLISHER_HPP_