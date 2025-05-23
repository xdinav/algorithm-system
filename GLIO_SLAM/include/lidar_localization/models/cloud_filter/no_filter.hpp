/*
 * @Description: 不滤波
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef GLIO_SLAM_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define GLIO_SLAM_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "GLIO_SLAM/models/cloud_filter/cloud_filter_interface.hpp"

namespace GLIO_SLAM {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif