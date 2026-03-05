#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/measure_group.h"
#include "common/point_def.h"
#include "livox_ros_driver/CustomMsg.h"

namespace lightning {

enum class LidarType { AVIA = 1, VELO32, OUST64, MERGED = 6 };

/**
 * point cloud preprocess
 * just unify the point format from livox/velodyne to PCL
 *
 * 预处理程序
 * 主要是对各种不同的雷达处理时间戳差异
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    /// processors
    void Process(const sensor_msgs::PointCloud2 ::ConstPtr &msg, PointCloudType::Ptr &pcl_out);

    void Process(const livox_ros_driver::CustomMsg::ConstPtr &cloud, PointCloudType::Ptr &pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    // accessors
    double &Blind() { return blind_; }
    int &NumScans() { return num_scans_; }
    int &PointFilterNum() { return point_filter_num_; }
    float &TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

   private:
    void Oust64Handler(const sensor_msgs::PointCloud2 ::ConstPtr &msg);
    void VelodyneHandler(const sensor_msgs::PointCloud2 ::ConstPtr &msg);
    void MergedCloudHandler(const sensor_msgs::PointCloud2 ::ConstPtr &msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;
    float merged_scan_period_ms_ = 100.0f;  // merged cloud default scan period is 0.1s
    bool given_offset_time_ = false;
};
}  // namespace lightning

#endif
