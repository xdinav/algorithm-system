/*
 * @Description: 打印信息
 * @Author: Ren Qian
 * @Date: 2020-03-02 23:25:26
 */
#ifndef GLIO_SLAM_TOOLS_PRINT_INFO_HPP_
#define GLIO_SLAM_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace GLIO_SLAM {
class PrintInfo {
  public:
    static void PrintPose(std::string head, Eigen::Matrix4f pose);
};
}
#endif