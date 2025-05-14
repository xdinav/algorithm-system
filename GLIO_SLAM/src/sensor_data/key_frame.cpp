/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:17:00
 */
#include "GLIO_SLAM/sensor_data/key_frame.hpp"

namespace GLIO_SLAM {

Eigen::Quaternionf KeyFrame::GetQuaternion() const {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}

Eigen::Vector3f KeyFrame::GetTranslation() const {
    Eigen::Vector3f t = pose.block<3,1>(0,3);

    return t;
}

}