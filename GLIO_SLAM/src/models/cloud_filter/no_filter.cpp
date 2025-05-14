/*
 * @Description: 不滤波
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "GLIO_SLAM/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace GLIO_SLAM {
NoFilter::NoFilter() {
}

bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
    return true;
}
} 