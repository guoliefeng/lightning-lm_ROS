#include "adapters/lidar_loc_adapter.h"

namespace lightning::loc {

LidarLocAdapter::LidarLocAdapter(LidarLoc::Options options) : impl_(std::make_shared<LidarLoc>(options)) {}

LidarLocAdapter::LidarLocAdapter(std::shared_ptr<LidarLoc> impl) : impl_(std::move(impl)) {}

bool LidarLocAdapter::Init(const std::string& yaml_path) { return impl_->Init(yaml_path); }

void LidarLocAdapter::FeedLidarOdom(const NavState& state) { impl_->ProcessLO(state); }

void LidarLocAdapter::FeedDeadReckoning(const NavState& state) { impl_->ProcessDR(state); }

bool LidarLocAdapter::ProcessKeyframeScan(CloudPtr cloud) { return impl_->ProcessCloud(cloud); }

void LidarLocAdapter::SetInitialPose(const SE3& pose) { impl_->SetInitialPose(pose); }

LocalizationResult LidarLocAdapter::GetLocalizationResult() const { return impl_->GetLocalizationResult(); }

void LidarLocAdapter::Finish() { impl_->Finish(); }

void LidarLocAdapter::SetUI(std::shared_ptr<ui::PangolinWindow> ui) { impl_->SetUI(std::move(ui)); }

}  // namespace lightning::loc
