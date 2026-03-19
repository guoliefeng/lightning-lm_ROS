#include "map_runtime/dynamic_map_manager.h"

#include <cmath>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <glog/logging.h>

#include "ui/pangolin_window.h"

namespace lightning::loc {

bool DynamicMapManager::Init(const DynamicMapManagerOptions& options, std::shared_ptr<ui::PangolinWindow> ui) {
    options_.map_option_.map_path_ = options.map_option_.map_path_;
    options_.map_option_.chunk_size_ = options.map_option_.chunk_size_;
    options_.map_option_.inv_chunk_size_ = options.map_option_.inv_chunk_size_;
    options_.map_option_.voxel_size_in_chunk_ = options.map_option_.voxel_size_in_chunk_;
    options_.map_option_.save_dynamic_layer_ = options.map_option_.save_dynamic_layer_;
    options_.map_option_.load_map_size_ = options.map_option_.load_map_size_;
    options_.map_option_.unload_map_size_ = options.map_option_.unload_map_size_;
    options_.map_option_.enable_dynamic_polygon_ = options.map_option_.enable_dynamic_polygon_;
    options_.map_option_.max_pts_in_dyn_chunk_ = options.map_option_.max_pts_in_dyn_chunk_;
    options_.map_option_.policy_ = options.map_option_.policy_;
    options_.map_option_.delete_when_unload_ = options.map_option_.delete_when_unload_;
    options_.map_option_.load_dyn_cloud_ = options.map_option_.load_dyn_cloud_;
    options_.map_option_.save_dyn_when_quit_ = options.map_option_.save_dyn_when_quit_;
    options_.map_option_.save_dyn_when_unload_ = options.map_option_.save_dyn_when_unload_;

    options_.update_dynamic_cloud_ = options.update_dynamic_cloud_;
    options_.update_kf_dis_ = options.update_kf_dis_;
    options_.update_kf_time_ = options.update_kf_time_;
    options_.update_lidar_loc_score_ = options.update_lidar_loc_score_;
    options_.dynamic_layer_filter_z_min_ = options.dynamic_layer_filter_z_min_;
    options_.dynamic_layer_filter_z_max_ = options.dynamic_layer_filter_z_max_;
    ui_ = std::move(ui);

    map_ = std::make_shared<TiledMap>(options_.map_option_);
    return map_->LoadMapIndex();
}

void DynamicMapManager::SetTargetRebuildCallback(TargetRebuildCallback cb) {
    target_rebuild_callback_ = std::move(cb);
}

std::vector<FunctionalPoint> DynamicMapManager::GetAllFunctionalPoints() const {
    if (map_ == nullptr) {
        return {};
    }
    return map_->GetAllFP();
}

void DynamicMapManager::AddFunctionalPoint(const FunctionalPoint& fp) {
    if (map_) {
        map_->AddFP(fp);
    }
}

void DynamicMapManager::LoadOnPose(const SE3& pose) {
    if (map_) {
        map_->LoadOnPose(pose);
    }
}

bool DynamicMapManager::MaybeUpdateDynamicLayer(const CloudPtr& scan_body,
                                                const SE3& current_abs_pose,
                                                double current_score,
                                                double timestamp) {
    if (map_ == nullptr || !options_.update_dynamic_cloud_ || scan_body == nullptr || scan_body->empty()) {
        return false;
    }

    if (current_score <= options_.update_lidar_loc_score_) {
        return false;
    }

    if ((current_abs_pose.translation() - last_dyn_upd_pose_.pose_.translation()).norm() <= options_.update_kf_dis_ &&
        std::fabs(timestamp - last_dyn_upd_pose_.timestamp_) <= options_.update_kf_time_) {
        return false;
    }

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(scan_body);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(options_.dynamic_layer_filter_z_min_, options_.dynamic_layer_filter_z_max_);

    CloudPtr input_z_filter(new PointCloudType());
    pass.filter(*input_z_filter);
    if (input_z_filter->empty()) {
        return false;
    }

    CloudPtr cloud_world(new PointCloudType());
    pcl::transformPointCloud(*input_z_filter, *cloud_world, current_abs_pose.matrix());
    map_->UpdateDynamicCloud(cloud_world, true);

    last_dyn_upd_pose_.timestamp_ = timestamp;
    last_dyn_upd_pose_.pose_ = current_abs_pose;
    return true;
}

bool DynamicMapManager::RebuildTargetIfNeeded() {
    if (map_ == nullptr || (!map_->MapUpdated() && !map_->DynamicMapUpdated())) {
        return false;
    }

    if (target_rebuild_callback_) {
        target_rebuild_callback_(map_);
    }

    if (ui_) {
        ui_->UpdatePointCloudGlobal(map_->GetStaticCloud());
        ui_->UpdatePointCloudDynamic(map_->GetDynamicCloud());
    }

    map_->CleanMapUpdate();
    return true;
}

void DynamicMapManager::ResetDynamicLayer() {
    if (map_) {
        map_->ResetDynamicCloud();
    }
    last_dyn_upd_pose_ = TimedPose();
}

void DynamicMapManager::Finish() {
    if (map_ == nullptr) {
        return;
    }

    if (options_.map_option_.policy_ == TiledMap::DynamicCloudPolicy::PERSISTENT &&
        options_.map_option_.save_dyn_when_quit_ && !has_set_pose_) {
        map_->SaveToBin(true);
        LOG(INFO) << "dynamic maps saved";
    }
}

}  // namespace lightning::loc
