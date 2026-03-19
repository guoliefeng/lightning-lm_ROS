#pragma once

#include <memory>

#include "common/timed_pose.h"
#include "interfaces/dynamic_map_manager.h"

namespace lightning::loc {

class DynamicMapManager : public IDynamicMapManager {
   public:
    DynamicMapManager() = default;
    ~DynamicMapManager() override = default;

    bool Init(const DynamicMapManagerOptions& options, std::shared_ptr<ui::PangolinWindow> ui) override;
    void SetTargetRebuildCallback(TargetRebuildCallback cb) override;

    std::vector<FunctionalPoint> GetAllFunctionalPoints() const override;
    void AddFunctionalPoint(const FunctionalPoint& fp) override;

    void LoadOnPose(const SE3& pose) override;
    bool MaybeUpdateDynamicLayer(const CloudPtr& scan_body,
                                 const SE3& current_abs_pose,
                                 double current_score,
                                 double timestamp) override;
    bool RebuildTargetIfNeeded() override;

    void ResetDynamicLayer() override;
    void Finish() override;

   private:
    DynamicMapManagerOptions options_;
    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
    std::shared_ptr<TiledMap> map_ = nullptr;
    TargetRebuildCallback target_rebuild_callback_;
    TimedPose last_dyn_upd_pose_;
    bool has_set_pose_ = false;
};

}  // namespace lightning::loc
