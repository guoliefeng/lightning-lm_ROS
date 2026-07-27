#pragma once

#include <memory>

#include "core/lio/laser_mapping.h"
#include "interfaces/motion_estimator.h"
#include "interfaces/ui_attachable.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

class LaserMappingAdapter : public IMotionEstimator, public IUiAttachable {
   public:
    explicit LaserMappingAdapter(LaserMapping::Options options = LaserMapping::Options());
    explicit LaserMappingAdapter(std::shared_ptr<LaserMapping> impl);

    bool Init(const std::string& yaml_path) override;
    void ProcessIMU(const IMUPtr& imu) override;
    void ProcessCloud(CloudPtr cloud) override;
    bool Run() override;
    NavState GetLidarOdomState() const override;
    NavState GetDeadReckoningState() const override;
    Keyframe::Ptr GetKeyframe() const override;
    CloudPtr GetUndistortedScan() const override;
    CloudPtr GetProjectedCloud() const override;
    void AttachUi(const std::shared_ptr<ui::PangolinWindow>& ui) override;

   private:
    std::shared_ptr<LaserMapping> impl_ = nullptr;
};

}  // namespace lightning::loc
