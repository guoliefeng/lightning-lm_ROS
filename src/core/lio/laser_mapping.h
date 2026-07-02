#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <condition_variable>
#include <list>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.hpp"
#include "pointcloud_preprocess.h"

#include "livox_ros_driver/CustomMsg.h"

namespace lightning {

namespace ui {
class PangolinWindow;
}

class LaserMapping {
   public:
    struct Options {
        Options()
            : is_in_slam_mode_(true),
              enable_icp_part_(true),
              plane_icp_weight_(1.0),
              icp_weight_(100.0),
              min_pts(300),
              kf_dis_th_(2.0),
              kf_angle_th_(15 * M_PI / 180.0),
              proj_kfs_(false),
              max_proj_kfs_(5) {}

        bool is_in_slam_mode_;
        bool enable_icp_part_;
        double plane_icp_weight_;
        double icp_weight_;
        int min_pts;
        double kf_dis_th_;
        double kf_angle_th_;
        bool proj_kfs_;
        int max_proj_kfs_;
        int min_effect_feat_surf_ = 80;  // 极低有效特征时拒绝本帧（与过大步长联合判断）
        double max_lidar_frame_trans_m_ = 1.5;
        double max_lidar_frame_rot_deg_ = 8.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;

    LaserMapping(Options options = Options());
    ~LaserMapping() {
        scan_down_body_ = nullptr;
        scan_undistort_ = nullptr;
        scan_down_world_ = nullptr;
        LOG(INFO) << "laser mapping deconstruct";
    }

    bool Init(const std::string& config_yaml);
    bool Run();

    void ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void ProcessPointCloud2(const livox_ros_driver::CustomMsg::ConstPtr& msg);
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr& msg_in);

    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    Keyframe::Ptr GetKeyframe() const { return last_kf_; }
    NavState GetState() const { return state_point_; }

    NavState GetIMUState() const {
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    CloudPtr GetScanUndist() const { return scan_undistort_; }
    CloudPtr GetProjCloud();
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

   private:
    bool SyncPackages();
    void ObsModel(NavState& s, ESKF::CustomObservationModel& obs);

    inline void PointBodyToWorld(const PointType& pi, PointType& po) {
        Vec3d p_global(state_point_.rot_ *
                           (offset_R_lidar_fixed_ * pi.getVector3fMap().cast<double>() + offset_t_lidar_fixed_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();
    bool LoadParamsFromYAML(const std::string& yaml);
    void MakeKF();
    void ProjectKFs(CloudPtr cloud, int size_limit = 1000);

   private:
    Options options_;

    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;

    double filter_size_map_min_ = 0;

    std::vector<double> extrinT_{3, 0.0};
    std::vector<double> extrinR_{9, 0.0};
    Mat3d offset_R_lidar_fixed_ = Mat3d::Identity();
    Vec3d offset_t_lidar_fixed_ = Vec3d::Zero();
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    CloudPtr scan_undistort_{new PointCloudType()};
    CloudPtr scan_down_body_{new PointCloudType()};
    CloudPtr scan_down_world_{new PointCloudType()};
    pcl::VoxelGrid<PointType> voxel_scan_;

    std::vector<PointVector> nearest_points_;
    std::vector<Vec4f> corr_pts_;
    std::vector<Vec4f> corr_norm_;
    std::vector<float> residuals_;
    std::vector<char> point_selected_surf_;
    std::vector<Vec4f> plane_coef_;

    std::vector<char> point_selected_icp_;

    std::mutex mtx_buffer_;
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    bool keep_first_imu_estimation_ = false;
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = true;
    int skip_lidar_num_ = 5;
    int skip_lidar_cnt_ = 0;

    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_surf_ = 0, frame_num_ = 0, effect_feat_icp_ = 0;

    double last_lidar_time_ = 0;

    MeasureGroup measures_;

    ESKF kf_;
    ESKF kf_imu_;

    NavState state_point_;

    bool use_aa_ = false;

    std::list<Keyframe::Ptr> proj_kfs_;

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H
