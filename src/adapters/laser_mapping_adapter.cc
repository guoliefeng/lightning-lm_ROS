#include "adapters/laser_mapping_adapter.h"

namespace lightning::loc {

LaserMappingAdapter::LaserMappingAdapter(LaserMapping::Options options)
    : impl_(std::make_shared<LaserMapping>(options)) {}

LaserMappingAdapter::LaserMappingAdapter(std::shared_ptr<LaserMapping> impl) : impl_(std::move(impl)) {}

bool LaserMappingAdapter::Init(const std::string& yaml_path) { return impl_->Init(yaml_path); }

void LaserMappingAdapter::ProcessIMU(const IMUPtr& imu) { impl_->ProcessIMU(imu); }

void LaserMappingAdapter::ProcessCloud(CloudPtr cloud) { impl_->ProcessPointCloud2(cloud); }

bool LaserMappingAdapter::Run() { return impl_->Run(); }

NavState LaserMappingAdapter::GetLidarOdomState() const { return impl_->GetState(); }

NavState LaserMappingAdapter::GetDeadReckoningState() const { return impl_->GetIMUState(); }

Keyframe::Ptr LaserMappingAdapter::GetKeyframe() const { return impl_->GetKeyframe(); }

CloudPtr LaserMappingAdapter::GetUndistortedScan() const { return impl_->GetScanUndist(); }

}  // namespace lightning::loc
