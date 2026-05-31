#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>

#include "common/imu.h"
#include "common/sensor_cloud_input.h"
#include "common/std_types.h"
#include "interfaces/localization_runtime.h"

/// 预声明
namespace lightning {
namespace application::system {
class LegacyRuntimeBridge;
}

namespace domain::contracts {
class ISystemRoot;
}

namespace loc {

/**
 * 实时定位接口实现
 */
class Localization : public ILocalizationRuntime {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = false;  // 在线模式还是离线模式
        bool with_ui_ = false;      // 是否带ui

        /// 参数
        SE3 T_body_lidar_;

        bool enable_lidar_odom_skip_ = false;  // 是否允许激光里程计跳帧
        int lidar_odom_skip_num_ = 1;          // 如果允许跳帧，跳多少帧
        bool enable_lidar_loc_skip_ = true;    // 是否允许激光定位跳帧
        bool enable_lidar_loc_rviz_ = false;   // 是否允许调试用rviz
        int lidar_loc_skip_num_ = 4;           // 如果允许跳帧，跳多少帧
        bool loc_on_kf_ = false;
    };

    Localization(Options options = Options());
    ~Localization() override;

    using TFCallback = ILocalizationRuntime::TFCallback;
    using LocStateCallback = ILocalizationRuntime::LocStateCallback;

    /**
     * 初始化，读配置参数
     * @param yaml_path
     * @param global_map_path
     * @param init_reloc_pose
     */
    bool Init(const std::string& yaml_path, const std::string& global_map_path) override;

    void FeedImu(IMUPtr imu) override { ProcessIMUMsg(imu); }
    void FeedCloud(const SensorCloudInput& cloud) override { ProcessCloud(cloud); }
    void SetInitialPose(const SE3& pose) override { SetExternalPose(pose.unit_quaternion(), pose.translation()); }
    void Finish() override;
    void SetTFCallback(TFCallback callback) override;
    void SetLocStateCallback(LocStateCallback callback) override;

    /// 处理IMU消息
    void ProcessIMUMsg(IMUPtr imu);
    void ProcessCloud(const SensorCloudInput& cloud);

    // void ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) override;

    /// 由外部设置pose，适用于手动重定位
    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    /// TODO: 其他初始化逻辑

    /// TODO: 处理odom消息

    // void SetPathCallback(std::function<void(const nav_msgs::msg::Path& path)>&& callback);
    // void SetPointcloudWorldCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetPointcloudBodyCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetHealthDiagNormalCallback(interface::health_diag_normal_callback&& callback);

   private:
    /// 模块  ========================================================================================================
    std::mutex global_mutex_;  // 防止处理过程中被重复init
    Options options_;

    std::shared_ptr<domain::contracts::ISystemRoot> system_root_ = nullptr;
    std::unique_ptr<application::system::LegacyRuntimeBridge> legacy_bridge_;

    /// 框架相关
    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;
};
}  // namespace loc

}  // namespace lightning
