#include "core/localization/localization.h"

#include <memory>
#include <utility>

#include "application/system/legacy_runtime_bridge.h"
#include "application/system/system_root_impl.h"

namespace lightning::loc {

Localization::Localization(Options options) { options_ = options; }

Localization::~Localization() = default;

bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (legacy_bridge_ || system_root_) {
        Finish();
    }

    application::system::SystemRootImpl::Options root_options;
    root_options.config_path = yaml_path;
    root_options.global_map_path = global_map_path;
    root_options.online_mode = options_.online_mode_;
    root_options.with_ui = options_.with_ui_;
    root_options.default_trajectory_id = "default";

    system_root_ = std::make_shared<application::system::SystemRootImpl>(std::move(root_options));
    legacy_bridge_ = std::make_unique<application::system::LegacyRuntimeBridge>(system_root_, "default");
    legacy_bridge_->SetLegacyOutputHandlers(tf_callback_, loc_state_callback_);

    if (!legacy_bridge_->Init(yaml_path)) {
        legacy_bridge_.reset();
        system_root_.reset();
        return false;
    }
    return true;
}

void Localization::ProcessCloud(const SensorCloudInput& cloud) {
    UL lock(global_mutex_);
    if (legacy_bridge_) {
        legacy_bridge_->FeedLegacyCloud(cloud);
    }
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);
    if (legacy_bridge_) {
        legacy_bridge_->FeedLegacyImu(imu);
    }
}

void Localization::FeedGnss(const domain::sensor::GnssData& gnss) {
    UL lock(global_mutex_);
    if (legacy_bridge_) {
        legacy_bridge_->FeedGnss(gnss);
    }
}

void Localization::FeedOdometry(const domain::sensor::OdometryData& odom) {
    UL lock(global_mutex_);
    if (legacy_bridge_) {
        legacy_bridge_->FeedOdometry(odom);
    }
}

void Localization::Finish() {
    if (legacy_bridge_) {
        legacy_bridge_->Finish();
        legacy_bridge_.reset();
    }
    system_root_.reset();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    if (legacy_bridge_) {
        legacy_bridge_->SetInitialPose(SE3(q, t));
    }
}

void Localization::SetTFCallback(Localization::TFCallback callback) {
    tf_callback_ = std::move(callback);
    if (legacy_bridge_) {
        legacy_bridge_->SetLegacyOutputHandlers(tf_callback_, loc_state_callback_);
    }
}

void Localization::SetLocStateCallback(Localization::LocStateCallback callback) {
    loc_state_callback_ = std::move(callback);
    if (legacy_bridge_) {
        legacy_bridge_->SetLegacyOutputHandlers(tf_callback_, loc_state_callback_);
    }
}

}  // namespace lightning::loc
