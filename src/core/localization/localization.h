#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>

#include "common/imu.h"
#include "common/std_types.h"
#include "core/localization/localization_result.h"
#include "core/system/async_message_process.h"
#include "interfaces/fusion_engine.h"
#include "interfaces/localizer.h"
#include "interfaces/localization_runtime.h"
#include "interfaces/sensor_pipeline.h"
#include "livox_ros_driver/CustomMsg.h"

/// 预声明
namespace lightning {
namespace ui {
class PangolinWindow;
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
    };

    Localization(Options options = Options());
    ~Localization() = default;

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
    void FeedPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& cloud) override { ProcessLidarMsg(cloud); }
    void FeedLivoxCloud(const livox_ros_driver::CustomMsg::ConstPtr& cloud) override {
        ProcessLivoxLidarMsg(cloud);
    }
    void SetInitialPose(const SE3& pose) override { SetExternalPose(pose.unit_quaternion(), pose.translation()); }
    void Finish() override;
    void SetTFCallback(TFCallback callback) override;
    void SetLocStateCallback(LocStateCallback callback) override;

    /// 处理lidar消息
    void ProcessLidarMsg(const sensor_msgs::PointCloud2::ConstPtr laser_msg);
    void ProcessLivoxLidarMsg(const livox_ros_driver::CustomMsg::ConstPtr laser_msg);

    /// 处理IMU消息
    void ProcessIMUMsg(IMUPtr imu);

    // void ProcessOdomMsg(const nav_msgs::msg::Odometry::SharedPtr odom_msg) override;

    /// 由外部设置pose，适用于手动重定位
    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    /// TODO: 其他初始化逻辑

    /// TODO: 处理odom消息

    /// 异步处理函数
    void LidarLocProcCloud(CloudPtr);

    using PointcloudBodyCallback = std::function<void(const sensor_msgs::PointCloud2& pointcloud)>;
    using PointcloudWorldCallback = std::function<void(const sensor_msgs::PointCloud2& pointcloud)>;

    // void SetPathCallback(std::function<void(const nav_msgs::msg::Path& path)>&& callback);
    // void SetPointcloudWorldCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetPointcloudBodyCallback(std::function<void(const sensor_msgs::msg::PointCloud2& pointcloud)>&& callback);
    // void SetHealthDiagNormalCallback(interface::health_diag_normal_callback&& callback);

   private:
    /// 模块  ========================================================================================================
    std::mutex global_mutex_;  // 防止处理过程中被重复init
    Options options_;

    // ui
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;

    // pose graph
    std::shared_ptr<IFusionEngine> fusion_engine_ = nullptr;

    // lidar localization
    std::shared_ptr<ILocalizer> localizer_;
    std::shared_ptr<ISensorPipeline> sensor_pipeline_ = nullptr;

    /// TODO async 处理
    sys::AsyncMessageProcess<CloudPtr> lidar_loc_proc_cloud_;   // lidar loc 处理点云

    /// 结果数据 =====================================================================================================
    LocalizationResult loc_result_;

    /// 框架相关
    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;
    PointcloudBodyCallback pointcloud_body_callback_;
    PointcloudWorldCallback pointcloud_world_callback_;

    /// 输入检查
    double last_imu_time_ = 0;
    double last_odom_time_ = 0;
    double last_cloud_time_ = 0;
};
}  // namespace loc

}  // namespace lightning
