#include "pointcloud_preprocess.h"

#include <algorithm>
#include <cmath>

#include <glog/logging.h>
#include <pcl/point_types.h>

namespace lightning {

namespace {

bool HasPointField(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string &name) {
    return std::any_of(msg->fields.begin(), msg->fields.end(), [&](const sensor_msgs::PointField &field) {
        return field.name == name;
    });
}

}  // namespace

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const sensor_msgs::PointCloud2 ::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;

        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;

        case LidarType::ROBOSENSE:
            RoboSenseHandler(msg);
            break;

        case LidarType::MERGED:
            MergedCloudHandler(msg);
            break;

        default:
            LOG(ERROR) << "Error LiDAR Type";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                                   PointCloudType::Ptr &pcl_out) {
    cloud_out_.clear();
    cloud_full_.clear();

    const int plsize = static_cast<int>(msg->point_num);
    if (plsize <= 1) {
        *pcl_out = cloud_out_;
        return;
    }

    cloud_out_.reserve(plsize);
    const int filter_step = std::max(1, point_filter_num_);
    const double blind_sq = blind_ * blind_;

    for (int i = 1; i < plsize; ++i) {
        const auto &pt = msg->points[i];
        if (pt.line >= num_scans_) {
            continue;
        }
        if (!((pt.tag & 0x30) == 0x10 || (pt.tag & 0x30) == 0x00)) {
            continue;
        }
        if (i % filter_step != 0) {
            continue;
        }

        const double range_sq = static_cast<double>(pt.x) * static_cast<double>(pt.x) +
                                static_cast<double>(pt.y) * static_cast<double>(pt.y) +
                                static_cast<double>(pt.z) * static_cast<double>(pt.z);
        if (range_sq <= blind_sq) {
            continue;
        }

        const auto &prev_pt = msg->points[i - 1];
        if (std::fabs(pt.x - prev_pt.x) <= 1e-7 && std::fabs(pt.y - prev_pt.y) <= 1e-7 &&
            std::fabs(pt.z - prev_pt.z) <= 1e-7) {
            continue;
        }

        if (pt.z < height_min_ || pt.z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pt.x;
        added_pt.y = pt.y;
        added_pt.z = pt.z;
        added_pt.intensity = pt.reflectivity;
        // use curvature as time of each laser points, curvature unit: ms
        added_pt.time = pt.offset_time / double(1000000);
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;

        if (range < (blind_ * blind_)) {
            continue;
        }

        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].t / 1e6;  // curvature unit: ms

        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::RoboSenseHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<PointRobotSense> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.size());
    cloud_out_.reserve(plsize);

    const double head_time = msg->header.stamp.sec + msg->header.stamp.nsec / 1e9;

    for (int i = 0; i < plsize; ++i) {
        if (i % point_filter_num_ != 0) {
            continue;
        }

        const auto &src = pl_orig.points[i];
        const double range_sq = static_cast<double>(src.x) * static_cast<double>(src.x) +
                                static_cast<double>(src.y) * static_cast<double>(src.y) +
                                static_cast<double>(src.z) * static_cast<double>(src.z);
        if (range_sq <= blind_ * blind_) {
            continue;
        }

        if (src.z < height_min_ || src.z > height_max_) {
            continue;
        }

        PointType added_pt;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;
        added_pt.time = (src.timestamp - head_time) * 1e3;
        cloud_out_.points.push_back(added_pt);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 3.61;  // scan angular velocity
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
    std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
    std::vector<float> time_last(num_scans_, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;

        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }

            // compute offset time
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.time < time_last[layer]) {
                added_pt.time += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
                if (added_pt.z < height_min_ || added_pt.z > height_max_) {
                    continue;
                }
                cloud_out_.points.push_back(added_pt);
            }
        }
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::MergedCloudHandler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    const bool has_merged_fields = HasPointField(msg, "ring") && HasPointField(msg, "azimuth") &&
                                   HasPointField(msg, "feature");
    if (!has_merged_fields) {
        pcl::PointCloud<pcl::PointXYZI> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        const int plsize = static_cast<int>(pl_orig.points.size());
        if (plsize <= 0) {
            LOG_EVERY_N(WARNING, 100) << "merged cloud has no points";
            return;
        }

        cloud_out_.reserve(plsize);
        const int filter_step = std::max(1, point_filter_num_);
        for (int i = 0; i < plsize; ++i) {
            if (i % filter_step != 0) {
                continue;
            }

            const auto &src = pl_orig.points[i];
            PointType added_pt;
            added_pt.x = src.x;
            added_pt.y = src.y;
            added_pt.z = src.z;
            added_pt.intensity = src.intensity;

            if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
                continue;
            }

            const double range_sq = static_cast<double>(added_pt.x) * static_cast<double>(added_pt.x) +
                                    static_cast<double>(added_pt.y) * static_cast<double>(added_pt.y) +
                                    static_cast<double>(added_pt.z) * static_cast<double>(added_pt.z);
            if (range_sq <= blind_ * blind_) {
                continue;
            }

            added_pt.time = plsize > 1 ? (static_cast<double>(i) / static_cast<double>(plsize - 1)) *
                                             static_cast<double>(merged_scan_period_ms_)
                                       : 0.0;
            cloud_out_.points.push_back(added_pt);
        }

        cloud_out_.width = cloud_out_.size();
        cloud_out_.height = 1;
        cloud_out_.is_dense = false;
        return;
    }

    pcl::PointCloud<merged_cloud_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    const int plsize = static_cast<int>(pl_orig.points.size());
    if (plsize <= 0) {
        LOG_EVERY_N(WARNING, 100) << "merged cloud has no points";
        return;
    }

    cloud_out_.reserve(plsize);
    const int filter_step = std::max(1, point_filter_num_);

    std::uint16_t max_azimuth = 0;
    for (const auto &pt : pl_orig.points) {
        max_azimuth = std::max(max_azimuth, pt.azimuth);
    }

    for (int i = 0; i < plsize; ++i) {
        if (i % filter_step != 0) {
            continue;
        }

        const auto &src = pl_orig.points[i];
        PointType added_pt;
        added_pt.x = src.x;
        added_pt.y = src.y;
        added_pt.z = src.z;
        added_pt.intensity = src.intensity;

        if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) {
            continue;
        }

        const double range_sq = static_cast<double>(added_pt.x) * static_cast<double>(added_pt.x) +
                                static_cast<double>(added_pt.y) * static_cast<double>(added_pt.y) +
                                static_cast<double>(added_pt.z) * static_cast<double>(added_pt.z);
        if (range_sq <= blind_ * blind_) {
            continue;
        }

        if (added_pt.z < height_min_ || added_pt.z > height_max_) {
            continue;
        }

        if (max_azimuth > 0) {
            added_pt.time = (static_cast<double>(src.azimuth) / static_cast<double>(max_azimuth)) *
                            static_cast<double>(merged_scan_period_ms_);
        } else if (plsize > 1) {
            added_pt.time = (static_cast<double>(i) / static_cast<double>(plsize - 1)) *
                            static_cast<double>(merged_scan_period_ms_);
        } else {
            added_pt.time = 0.0;
        }

        cloud_out_.points.push_back(added_pt);
    }

    if (cloud_out_.size() > 1) {
        std::sort(cloud_out_.points.begin(), cloud_out_.points.end(), [](const PointType &a, const PointType &b) {
            return a.time < b.time;
        });
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace lightning
