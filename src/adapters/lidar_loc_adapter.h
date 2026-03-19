#pragma once

#include <memory>

#include "core/localization/lidar_loc/lidar_loc.h"
#include "interfaces/localizer.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

class LidarLocAdapter : public ILocalizer {
   public:
    explicit LidarLocAdapter(LidarLoc::Options options = LidarLoc::Options());
    explicit LidarLocAdapter(std::shared_ptr<LidarLoc> impl);

    bool Init(const std::string& yaml_path) override;
    void FeedLidarOdom(const NavState& state) override;
    void FeedDeadReckoning(const NavState& state) override;
    bool ProcessKeyframeScan(CloudPtr cloud) override;
    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) override;
    void SetInitialPose(const SE3& pose) override;
    LocalizationResult GetLocalizationResult() const override;
    void Finish() override;

   private:
    std::shared_ptr<LidarLoc> impl_ = nullptr;
};

}  // namespace lightning::loc
