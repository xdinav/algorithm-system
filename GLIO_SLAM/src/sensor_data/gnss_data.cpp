/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "GLIO_SLAM/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <ostream>

//静态成员变量必须在类外初始化
double GLIO_SLAM::GNSSData::origin_latitude = 0.0;
double GLIO_SLAM::GNSSData::origin_longitude = 0.0;
double GLIO_SLAM::GNSSData::origin_altitude = 0.0;
bool GLIO_SLAM::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian GLIO_SLAM::GNSSData::geo_converter;

namespace GLIO_SLAM {
void GNSSData::InitOriginPosition() {
    // latitude = 22.3304807923;
    // longitude = 114.179526047;
    // altitude = 11.7615366792;
    geo_converter.Reset(latitude, longitude, altitude);

    origin_latitude = latitude;
    origin_longitude = longitude;
    origin_altitude = altitude;

    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }

    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

void GNSSData::Reverse(
    const double &local_E, const double &local_N, const double &local_U,
    double &lat, double &lon, double &alt
) {
    if (!origin_position_inited) {
        LOG(WARNING) << "WARNING: GeoConverter is NOT initialized.";
    }

    geo_converter.Reverse(local_E, local_N, local_U, lat, lon, alt);
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.01) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.01) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}

}
