#include "laser_mapping.h"

#include <execution>
#include <fstream>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

#include "common/options.h"
#include "core/lightning_math.hpp"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

namespace lightning {

LaserMapping::LaserMapping(Options options) : options_(options) {
    preprocess_.reset(new PointCloudPreprocess());
    p_imu_.reset(new ImuProcess());
}

bool LaserMapping::Init(const std::string& config_yaml) {
    LOG(INFO) << "init laser mapping from " << config_yaml;
    if (!LoadParamsFromYAML(config_yaml)) {
        return false;
    }

    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    ESKF::Options eskf_options;
    eskf_options.max_iterations_ = fasterlio::NUM_MAX_ITERATIONS;
    eskf_options.epsi_ = 1e-3 * Eigen::Matrix<double, ESKF::state_dim_, 1>::Ones();
    eskf_options.lidar_obs_func_ = [this](NavState& s, ESKF::CustomObservationModel& obs) { ObsModel(s, obs); };
    eskf_options.use_aa_ = use_aa_;
    kf_.Init(eskf_options);

    return true;
}

bool LaserMapping::LoadParamsFromYAML(const std::string& yaml_file) {
    int lidar_type = 0, ivox_nearby_type = 18;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 1e-4, b_acc_cov = 1e-4;
    double filter_size_scan = 0.5;

    auto yaml = YAML::LoadFile(yaml_file);
    const auto fasterlio = yaml["fasterlio"];
    try {
        fasterlio::NUM_MAX_ITERATIONS = fasterlio["max_iteration"].as<int>();
        fasterlio::ESTI_PLANE_THRESHOLD = fasterlio["esti_plane_threshold"].as<float>();

        filter_size_scan = fasterlio["filter_size_scan"].as<float>();
        filter_size_map_min_ = fasterlio["filter_size_map"].as<float>();
        keep_first_imu_estimation_ = fasterlio["keep_first_imu_estimation"].as<bool>();
        gyr_cov = fasterlio["gyr_cov"].as<float>();
        acc_cov = fasterlio["acc_cov"].as<float>();
        b_gyr_cov = fasterlio["b_gyr_cov"].as<float>();
        b_acc_cov = fasterlio["b_acc_cov"].as<float>();
        preprocess_->Blind() = fasterlio["blind"].as<double>();
        preprocess_->TimeScale() = fasterlio["time_scale"].as<double>();
        lidar_type = fasterlio["lidar_type"].as<int>();
        preprocess_->NumScans() = fasterlio["scan_line"].as<int>();
        preprocess_->PointFilterNum() = fasterlio["point_filter_num"].as<int>();
        extrinT_ = fasterlio["extrinsic_T"].as<std::vector<double>>();
        extrinR_ = fasterlio["extrinsic_R"].as<std::vector<double>>();

        ivox_options_.resolution_ = fasterlio["ivox_grid_resolution"].as<float>();
        ivox_nearby_type = fasterlio["ivox_nearby_type"].as<int>();
        use_aa_ = fasterlio["use_aa"].as<bool>();

        skip_lidar_num_ = fasterlio["skip_lidar_num"].as<int>();
        enable_skip_lidar_ = skip_lidar_num_ > 0;

        options_.kf_dis_th_ = fasterlio["kf_dis_th"].as<double>();
        options_.kf_angle_th_ = fasterlio["kf_angle_th"].as<double>() * M_PI / 180.0;

        if (fasterlio["enable_icp_part"]) {
            options_.enable_icp_part_ = fasterlio["enable_icp_part"].as<bool>();
        }
        if (fasterlio["min_pts"]) {
            options_.min_pts = fasterlio["min_pts"].as<int>();
        }
        if (fasterlio["plane_icp_weight"]) {
            options_.plane_icp_weight_ = fasterlio["plane_icp_weight"].as<float>();
        }
        if (fasterlio["icp_weight"]) {
            options_.icp_weight_ = fasterlio["icp_weight"].as<float>();
        }
        if (fasterlio["imu_filter"]) {
            p_imu_->SetUseIMUFilter(fasterlio["imu_filter"].as<bool>());
        }
        if (fasterlio["proj_kfs"]) {
            options_.proj_kfs_ = fasterlio["proj_kfs"].as<bool>();
        }
        if (fasterlio["max_proj_kfs"]) {
            options_.max_proj_kfs_ = fasterlio["max_proj_kfs"].as<int>();
        }

        if (yaml["roi"] && yaml["roi"]["height_max"] && yaml["roi"]["height_min"]) {
            preprocess_->SetHeightROI(yaml["roi"]["height_max"].as<float>(), yaml["roi"]["height_min"].as<float>());
        }
    } catch (...) {
        LOG(ERROR) << "bad conversion";
        return false;
    }

    LOG(INFO) << "lidar_type " << lidar_type;
    if (lidar_type == 1) {
        preprocess_->SetLidarType(LidarType::AVIA);
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        preprocess_->SetLidarType(LidarType::VELO32);
        LOG(INFO) << "Using Velodyne 32 Lidar";
    } else if (lidar_type == 3) {
        preprocess_->SetLidarType(LidarType::OUST64);
        LOG(INFO) << "Using OUST 64 Lidar";
    } else if (lidar_type == 4) {
        preprocess_->SetLidarType(LidarType::ROBOSENSE);
        LOG(INFO) << "Using RoboSense Lidar";
    } else if (lidar_type == 6) {
        preprocess_->SetLidarType(LidarType::MERGED);
        LOG(INFO) << "Using merged PointCloud2 (meta_cloud)";
    } else {
        LOG(WARNING) << "unknown lidar_type";
        return false;
    }

    if (ivox_nearby_type == 0) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
    } else if (ivox_nearby_type == 6) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
    } else if (ivox_nearby_type == 18) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    } else if (ivox_nearby_type == 26) {
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
    } else {
        LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
        ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
    }

    voxel_scan_.setLeafSize(filter_size_scan, filter_size_scan, filter_size_scan);

    offset_t_lidar_fixed_ = math::VecFromArray<double>(extrinT_);
    offset_R_lidar_fixed_ = math::MatFromArray<double>(extrinR_);

    p_imu_->SetExtrinsic(offset_t_lidar_fixed_, offset_R_lidar_fixed_);
    p_imu_->SetGyrCov(Vec3d(gyr_cov, gyr_cov, gyr_cov));
    p_imu_->SetAccCov(Vec3d(acc_cov, acc_cov, acc_cov));
    p_imu_->SetGyrBiasCov(Vec3d(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu_->SetAccBiasCov(Vec3d(b_acc_cov, b_acc_cov, b_acc_cov));
    return true;
}

void LaserMapping::ProcessIMU(const lightning::IMUPtr& imu) {
    publish_count_++;

    const double timestamp = imu->timestamp;

    UL lock(mtx_buffer_);
    if (timestamp < last_timestamp_imu_) {
        LOG(WARNING) << "imu loop back, clear buffer";
        imu_buffer_.clear();
    }

    if (p_imu_->IsIMUInited()) {
        kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);

        if (ui_) {
            ui_->UpdateNavState(kf_imu_.GetX());
        }
    }

    last_timestamp_imu_ = timestamp;
    imu_buffer_.emplace_back(imu);
}

bool LaserMapping::Run() {
    if (!SyncPackages()) {
        return false;
    }

    p_imu_->Process(measures_, kf_, scan_undistort_);

    if (scan_undistort_ == nullptr || scan_undistort_->empty()) {
        LOG(WARNING) << "No point, skip this scan!";
        return false;
    }

    if (flg_first_scan_) {
        LOG(INFO) << "first scan pts: " << scan_undistort_->size();

        state_point_ = kf_.GetX();
        scan_down_world_->resize(scan_undistort_->size());
        for (int i = 0; i < scan_undistort_->size(); i++) {
            PointBodyToWorld(scan_undistort_->points[i], scan_down_world_->points[i]);
        }
        ivox_->AddPoints(scan_down_world_->points);

        first_lidar_time_ = measures_.lidar_end_time_;
        state_point_.timestamp_ = lidar_end_time_;
        flg_first_scan_ = false;
        return true;
    }

    if (enable_skip_lidar_) {
        skip_lidar_cnt_++;
        skip_lidar_cnt_ = skip_lidar_cnt_ % skip_lidar_num_;

        if (skip_lidar_cnt_ != 0) {
            if (ui_) {
                ui_->UpdateNavState(kf_.GetX());
                ui_->UpdateScan(scan_undistort_, kf_.GetX().GetPose());
            }
            return false;
        }
    }

    if (last_lidar_time_ > 0 && (measures_.lidar_begin_time_ - last_lidar_time_) > 0.5) {
        LOG(ERROR) << "检测到雷达断流，时长：" << (measures_.lidar_begin_time_ - last_lidar_time_);
    }

    last_lidar_time_ = measures_.lidar_begin_time_;
    flg_EKF_inited_ = (measures_.lidar_begin_time_ - first_lidar_time_) >= fasterlio::INIT_TIME;

    voxel_scan_.setInputCloud(scan_undistort_);
    voxel_scan_.filter(*scan_down_body_);

    int cur_pts = static_cast<int>(scan_down_body_->size());
    if (cur_pts < static_cast<int>(scan_undistort_->size() * 0.1) || cur_pts < options_.min_pts) {
        auto v = voxel_scan_;
        v.setLeafSize(0.1, 0.1, 0.1);
        v.setInputCloud(scan_undistort_);
        v.filter(*scan_down_body_);
        cur_pts = static_cast<int>(scan_down_body_->size());
    }

    if (cur_pts < 5) {
        LOG(WARNING) << "Too few points, skip this scan!" << scan_undistort_->size() << ", " << scan_down_body_->size();
        return false;
    }

    scan_down_world_->resize(cur_pts);
    nearest_points_.resize(cur_pts);
    residuals_.resize(cur_pts, 0);
    point_selected_surf_.resize(cur_pts, 1);
    point_selected_icp_.resize(cur_pts, 1);
    plane_coef_.resize(cur_pts, Vec4f::Zero());

    auto pred_state = kf_.GetX();
    kf_.Update(ESKF::ObsType::LIDAR, 1.0);

    state_point_ = kf_.GetX();
    state_point_.timestamp_ = measures_.lidar_end_time_;

    const double delta_rotation_deg = (pred_state.rot_.inverse() * state_point_.rot_).log().norm() * 180.0 / M_PI;

    LOG(INFO) << "[ mapping ]: In num: " << scan_undistort_->points.size() << " down " << cur_pts
              << " Map grid num: " << ivox_->NumValidGrids() << " effect num : " << effect_feat_surf_ << ", "
              << effect_feat_icp_;
    LOG(INFO) << "delta trans: " << (pred_state.pos_ - state_point_.pos_).transpose()
              << ", ang: " << delta_rotation_deg;

    if (last_kf_ == nullptr) {
        MakeKF();
    } else {
        SE3 last_pose = last_kf_->GetLIOPose();
        SE3 cur_pose = state_point_.GetPose();
        if ((last_pose.translation() - cur_pose.translation()).norm() > options_.kf_dis_th_ ||
            (last_pose.so3().inverse() * cur_pose.so3()).log().norm() > options_.kf_angle_th_) {
            MakeKF();
        } else if (!options_.is_in_slam_mode_ && (state_point_.timestamp_ - last_kf_->GetState().timestamp_) > 2.0) {
            MakeKF();
        }
    }

    kf_imu_ = kf_;
    if (!measures_.imu_.empty()) {
        std::deque<lightning::IMUPtr> imu_buffer_snapshot;
        {
            UL lock(mtx_buffer_);
            imu_buffer_snapshot = imu_buffer_;
        }

        double t = measures_.imu_.back()->timestamp;
        for (auto& imu : imu_buffer_snapshot) {
            double dt = imu->timestamp - t;
            kf_imu_.Predict(dt, p_imu_->Q_, imu->angular_velocity, imu->linear_acceleration);
            t = imu->timestamp;
        }
    }

    if (ui_) {
        ui_->UpdateScan(scan_down_body_, state_point_.GetPose());
    }

    LOG(INFO) << "LIO state: " << state_point_.pos_.transpose() << ", yaw "
              << state_point_.rot_.angleZ<double>() * 180 / M_PI << ", vel: " << state_point_.vel_.transpose()
              << ", grav: " << state_point_.grav_.transpose() << ", grav norm: " << state_point_.grav_.norm();

    return true;
}

void LaserMapping::ProjectKFs(CloudPtr cloud, int size_limit) {
    auto state = kf_.GetX();
    SE3 pose_cur(state.rot_, state.pos_);
    pose_cur = pose_cur.inverse();

    for (auto kf : proj_kfs_) {
        SE3 pose = pose_cur * kf->GetLIOPose();

        int cnt = 0;
        for (auto& pt : kf->GetCloud()->points) {
            Vec3d p = pose * ToVec3d(pt);
            PointType pcl_pt;
            pcl_pt.x = p.x();
            pcl_pt.y = p.y();
            pcl_pt.z = p.z();
            pcl_pt.intensity = pt.intensity;
            cloud->push_back(pcl_pt);
            cnt++;

            if (cnt > size_limit) {
                break;
            }
        }
    }
}

void LaserMapping::MakeKF() {
    Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_);

    if (last_kf_) {
        SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
        kf->SetOptPose(last_kf_->GetOptPose() * delta);
    } else {
        kf->SetOptPose(kf->GetLIOPose());
    }

    kf->SetState(state_point_);

    LOG(INFO) << "LIO: create kf " << kf->GetID() << ", state: " << state_point_.pos_.transpose()
              << ", kf opt pose: " << kf->GetOptPose().translation().transpose()
              << ", lio pose: " << kf->GetLIOPose().translation().transpose() << ", time: " << std::setprecision(14)
              << state_point_.timestamp_;

    if (options_.is_in_slam_mode_) {
        all_keyframes_.emplace_back(kf);
    }

    last_kf_ = kf;

    Timer::Evaluate([&, this]() { MapIncremental(); }, "    Incremental Mapping");

    if (proj_kfs_.size() >= static_cast<size_t>(options_.max_proj_kfs_)) {
        auto last = proj_kfs_.back();
        SE3 delta = last->GetLIOPose().inverse() * kf->GetLIOPose();

        if (delta.translation().norm() >= 3 && delta.so3().log().norm() >= 20.0 / 180.0 * M_PI) {
            proj_kfs_.pop_front();
            proj_kfs_.emplace_back(kf);
        }
    } else {
        proj_kfs_.emplace_back(kf);
    }
}

void LaserMapping::ProcessPointCloud2(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, dt: " << timestamp - last_timestamp_lidar_;
                return;
            }

            LOG(INFO) << "get cloud at " << std::setprecision(14) << timestamp
                      << ", latest imu: " << last_timestamp_imu_;

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;
            double timestamp = ToSec(msg->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            CloudPtr cloud(new PointCloudType());
            preprocess_->Process(msg, cloud);

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

void LaserMapping::ProcessPointCloud2(CloudPtr cloud) {
    UL lock(mtx_buffer_);
    Timer::Evaluate(
        [&, this]() {
            scan_count_++;

            double timestamp = math::ToSec(cloud->header.stamp);
            if (timestamp < last_timestamp_lidar_) {
                LOG(ERROR) << "lidar loop back, clear buffer";
                lidar_buffer_.clear();
            }

            lidar_buffer_.push_back(cloud);
            time_buffer_.push_back(timestamp);
            last_timestamp_lidar_ = timestamp;
        },
        "Preprocess (Standard)");
}

bool LaserMapping::SyncPackages() {
    UL lock(mtx_buffer_);
    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    if (!lidar_pushed_) {
        measures_.scan_ = lidar_buffer_.front();
        measures_.lidar_begin_time_ = time_buffer_.front();

        if (measures_.scan_->points.size() <= 1) {
            LOG(WARNING) << "Too few input point cloud!";
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else if (measures_.scan_->points.back().time / double(1000) < 0.5 * lidar_mean_scantime_) {
            lidar_end_time_ = measures_.lidar_begin_time_ + lidar_mean_scantime_;
        } else {
            scan_num_++;
            lidar_end_time_ = measures_.lidar_begin_time_ + measures_.scan_->points.back().time / double(1000);
            lidar_mean_scantime_ +=
                (measures_.scan_->points.back().time / double(1000) - lidar_mean_scantime_) / scan_num_;

            if ((lidar_end_time_ - measures_.lidar_begin_time_) > 5 * lo::lidar_time_interval) {
                lidar_end_time_ = measures_.lidar_begin_time_ + lo::lidar_time_interval;
                lidar_mean_scantime_ = lo::lidar_time_interval;
            }
        }

        lo::lidar_time_interval = lidar_mean_scantime_;
        measures_.lidar_end_time_ = lidar_end_time_;
        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    double imu_time = imu_buffer_.front()->timestamp;
    measures_.imu_.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = imu_buffer_.front()->timestamp;
        if (imu_time > lidar_end_time_) {
            break;
        }

        measures_.imu_.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;

    return true;
}

void LaserMapping::MapIncremental() {
    PointVector points_to_add;
    PointVector point_no_need_downsample;

    size_t cur_pts = scan_down_body_->size();
    points_to_add.reserve(cur_pts);
    point_no_need_downsample.reserve(cur_pts);

    std::vector<size_t> index(cur_pts);
    for (size_t i = 0; i < cur_pts; ++i) {
        index[i] = i;
    }

    std::for_each(index.begin(), index.end(), [&](const size_t& i) {
        PointBodyToWorld(scan_down_body_->points[i], scan_down_world_->points[i]);

        PointType& point_world = scan_down_world_->points[i];
        if (!nearest_points_[i].empty() && flg_EKF_inited_) {
            const PointVector& points_near = nearest_points_[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min_).array().floor() + 0.5f) * filter_size_map_min_;

            Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

            if (fabs(dis_2_center.x()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.y()) > 0.5 * filter_size_map_min_ &&
                fabs(dis_2_center.z()) > 0.5 * filter_size_map_min_) {
                point_no_need_downsample.emplace_back(point_world);
                return;
            }

            bool need_add = true;
            float dist = math::calc_dist(point_world.getVector3fMap(), center);
            if (points_near.size() >= fasterlio::NUM_MATCH_POINTS) {
                for (int readd_i = 0; readd_i < fasterlio::NUM_MATCH_POINTS; readd_i++) {
                    if (math::calc_dist(points_near[readd_i].getVector3fMap(), center) < dist + 1e-6) {
                        need_add = false;
                        break;
                    }
                }
            }

            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    });

    Timer::Evaluate(
        [&, this]() {
            ivox_->AddPoints(points_to_add);
            ivox_->AddPoints(point_no_need_downsample);
        },
        "    IVox Add Points");
}

void LaserMapping::ObsModel(NavState& s, ESKF::CustomObservationModel& obs) {
    int cnt_pts = static_cast<int>(scan_down_body_->size());

    std::vector<size_t> index(cnt_pts);
    for (size_t i = 0; i < index.size(); ++i) {
        index[i] = i;
    }

    Timer::Evaluate(
        [&, this]() {
            Mat3f R_wl = (s.rot_.matrix() * offset_R_lidar_fixed_).cast<float>();
            Vec3f t_wl = (s.rot_ * offset_t_lidar_fixed_ + s.pos_).cast<float>();

            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
                PointType& point_body = scan_down_body_->points[i];
                PointType& point_world = scan_down_world_->points[i];

                Vec3f p_body = point_body.getVector3fMap();
                point_world.getVector3fMap() = R_wl * p_body + t_wl;
                point_world.intensity = point_body.intensity;

                auto& points_near = nearest_points_[i];
                points_near.clear();

                ivox_->GetClosestPoint(point_world, points_near, fasterlio::NUM_MATCH_POINTS);
                point_selected_surf_[i] = points_near.size() >= fasterlio::MIN_NUM_MATCH_POINTS;
                point_selected_icp_[i] = point_selected_surf_[i];

                if (point_selected_surf_[i]) {
                    point_selected_surf_[i] =
                        math::esti_plane(plane_coef_[i], points_near, fasterlio::ESTI_PLANE_THRESHOLD);
                }

                if (point_selected_surf_[i]) {
                    auto temp = point_world.getVector4fMap();
                    temp[3] = 1.0;
                    float pd2 = plane_coef_[i].dot(temp);

                    if (p_body.norm() > 81 * pd2 * pd2) {
                        residuals_[i] = pd2;
                    } else {
                        point_selected_surf_[i] = false;
                    }
                }
            });
        },
        "    ObsModel (Lidar Match)");

    effect_feat_surf_ = 0;
    effect_feat_icp_ = 0;

    corr_pts_.resize(cnt_pts);
    corr_norm_.resize(cnt_pts);
    for (int i = 0; i < cnt_pts; i++) {
        if (point_selected_surf_[i]) {
            corr_norm_[effect_feat_surf_] = plane_coef_[i];
            corr_pts_[effect_feat_surf_] = scan_down_body_->points[i].getVector4fMap();
            corr_pts_[effect_feat_surf_][3] = residuals_[i];
            effect_feat_surf_++;
        }

        if (point_selected_icp_[i]) {
            effect_feat_icp_++;
        }
    }

    corr_pts_.resize(effect_feat_surf_);
    corr_norm_.resize(effect_feat_surf_);

    if (effect_feat_surf_ < 20) {
        obs.valid_ = false;
        LOG(WARNING) << "No enough effective surface points: " << effect_feat_surf_ << ", icp: " << effect_feat_icp_
                     << ", required: 20";
        return;
    }

    index.resize(effect_feat_surf_);
    const Mat3f off_R = offset_R_lidar_fixed_.cast<float>();
    const Vec3f off_t = offset_t_lidar_fixed_.cast<float>();
    const Mat3f Rt = s.rot_.matrix().transpose().cast<float>();

    obs.HTH_.setZero();
    obs.HTr_.setZero();

    std::vector<Mat6d> JTJ(effect_feat_surf_);
    std::vector<Vec6d> JTr(effect_feat_surf_);
    std::vector<double> res_sq(index.size());

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t& i) {
        Vec3f point_this_be = corr_pts_[i].head<3>();
        Vec3f point_this = off_R * point_this_be + off_t;
        Mat3f point_crossmat = math::SKEW_SYM_MATRIX(point_this);

        Vec3f norm_vec = corr_norm_[i].head<3>();
        Vec3f C(Rt * norm_vec);
        Vec3f A(point_crossmat * C);

        Eigen::Matrix<double, 1, ESKF::pose_obs_dim_> J;
        J.setZero();
        J << norm_vec[0], norm_vec[1], norm_vec[2], A[0], A[1], A[2];

        float res = -corr_pts_[i][3];
        double w = 1.0;

        JTJ[i] = (J.transpose() * J).eval() * w;
        JTr[i] = J.transpose() * res * w;
        res_sq[i] = res * res;
    });

    for (int i = 0; i < static_cast<int>(index.size()); ++i) {
        obs.HTH_ += JTJ[i] * options_.plane_icp_weight_;
        obs.HTr_ += JTr[i] * options_.plane_icp_weight_;
    }

    if (!res_sq.empty()) {
        std::sort(res_sq.begin(), res_sq.end());
        obs.lidar_residual_mean_ = res_sq[res_sq.size() / 2];
        obs.lidar_residual_max_ = res_sq[res_sq.size() - 1];
    }

    if (options_.enable_icp_part_) {
        JTJ.resize(cnt_pts);
        JTr.resize(cnt_pts);

        std::vector<size_t> icp_index(cnt_pts);
        for (size_t i = 0; i < icp_index.size(); ++i) {
            icp_index[i] = i;
        }

        std::for_each(std::execution::par_unseq, icp_index.begin(), icp_index.end(), [&](const size_t& i) {
            if (point_selected_icp_[i] == false) {
                return;
            }

            Vec3d q = scan_down_body_->points[i].getVector3fMap().cast<double>();
            Vec3d qs = scan_down_world_->points[i].getVector3fMap().cast<double>();

            Eigen::Matrix<double, 3, ESKF::pose_obs_dim_> J;
            J.setZero();
            J.block<3, 3>(0, 0) = Mat3d::Identity();
            J.block<3, 3>(0, 3) = -(s.rot_.matrix() * offset_R_lidar_fixed_) * SO3::hat(q);

            Vec3d e = qs - nearest_points_[i][0].getVector3fMap().cast<double>();
            if (e.norm() > 0.5) {
                point_selected_icp_[i] = false;
                return;
            }

            JTJ[i] = J.transpose() * J;
            JTr[i] = -J.transpose() * e;
        });

        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_icp_[i] == false) {
                continue;
            }
            obs.HTH_ += JTJ[i] * options_.icp_weight_;
            obs.HTr_ += JTr[i] * options_.icp_weight_;
        }
    }
}

CloudPtr LaserMapping::GetGlobalMap(bool use_lio_pose, bool use_voxel, float res) {
    CloudPtr global_map(new PointCloudType);

    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(res, res, res);

    for (auto& kf : all_keyframes_) {
        CloudPtr cloud = kf->GetCloud();
        CloudPtr cloud_filter(new PointCloudType);

        if (use_voxel) {
            voxel.setInputCloud(cloud);
            voxel.filter(*cloud_filter);
        } else {
            cloud_filter = cloud;
        }

        CloudPtr cloud_trans(new PointCloudType);

        if (use_lio_pose) {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetLIOPose().matrix());
        } else {
            pcl::transformPointCloud(*cloud_filter, *cloud_trans, kf->GetOptPose().matrix());
        }

        *global_map += *cloud_trans;
        LOG(INFO) << "kf " << kf->GetID() << ", pose: " << kf->GetOptPose().translation().transpose();
    }

    CloudPtr global_map_filtered(new PointCloudType);
    if (use_voxel) {
        voxel.setInputCloud(global_map);
        voxel.filter(*global_map_filtered);
    } else {
        global_map_filtered = global_map;
    }

    global_map_filtered->is_dense = false;
    global_map_filtered->height = 1;
    global_map_filtered->width = global_map_filtered->size();

    LOG(INFO) << "global map: " << global_map_filtered->size();
    return global_map_filtered;
}

void LaserMapping::SaveMap() {
    auto global_map = GetGlobalMap(true);
    pcl::io::savePCDFileBinaryCompressed("./data/lio.pcd", *global_map);
    LOG(INFO) << "lio map is saved to ./data/lio.pcd";
}

CloudPtr LaserMapping::GetRecentCloud() {
    if (lidar_buffer_.empty()) {
        return nullptr;
    }

    return lidar_buffer_.front();
}

CloudPtr LaserMapping::GetProjCloud() {
    CloudPtr cloud(new PointCloudType(*scan_undistort_));
    if (options_.proj_kfs_) {
        ProjectKFs(cloud);
    }
    return cloud;
}

}  // namespace lightning
