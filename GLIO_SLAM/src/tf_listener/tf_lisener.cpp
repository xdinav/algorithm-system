/*
 * @Description: tf监听模块
 * @Author: Ren Qian
 * @Date: 2020-02-06 16:10:31
 */
#include "GLIO_SLAM/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace GLIO_SLAM {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id) 
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {
}
bool TFListener::LookupData1(Eigen::Matrix4f& transform_matrix) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    //  transform<< 1, 0, 0, 0,
    //        0, 1, 0, 0,
    //        0, 0, 1, 0, 
    //        0, 0, 0, 1;
    // transform<< 0, -1,0,0.0125,
    //                          1, 0,0,0.205,
    //                         0, 0, 1,-0.45,
    //                         0,0,0,1;
        // transform<< 1, 0,0,0.27255,
        //                      0, 1,0,-0.00053,
        //                     0, 0, 1,0.17954,
        //                     0,0,0,1;
        transform<<0.0727502  ,-0.99734674 ,-0.00262441,-0.2685153,
                                0.99722447 , 0.07278256, -0.01568628,0.65342808,
                                0.01583567 ,-0.00147594  ,0.99987352,0.49409979,
                                0,0,0, 1;
    transform_matrix=transform;
}
bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try {
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id_, child_frame_id_, ros::Time(0), transform);
        TransformToMatrix(transform, transform_matrix);
        return true;
    } catch (tf::TransformException &ex) {
        return false;
    }
}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 此矩阵为 child_frame_id 到 base_frame_id 的转换矩阵
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    return true;
}
}