#pragma once

#include <functional>
#include <map>
#include <memory>
#include <vector>

#include "common/functional_points.h"
#include "common/point_def.h"
#include "core/maps/tiled_map.h"

namespace lightning::ui {
class PangolinWindow;
}

namespace lightning::loc {

struct DynamicMapManagerOptions {
    TiledMap::Options map_option_ = TiledMap::Options();
    bool update_dynamic_cloud_ = true;
    double update_kf_dis_ = 5.0;
    double update_kf_time_ = 10.0;
    double update_lidar_loc_score_ = 2.2;
    double dynamic_layer_filter_z_min_ = 0.5;
    double dynamic_layer_filter_z_max_ = 30.0;
};

class IDynamicMapManager {
   public:
    using TargetRebuildCallback = std::function<void(const std::shared_ptr<TiledMap>& map)>;

    virtual ~IDynamicMapManager() = default;

    virtual bool Init(const DynamicMapManagerOptions& options, std::shared_ptr<ui::PangolinWindow> ui) = 0;
    virtual void SetTargetRebuildCallback(TargetRebuildCallback cb) = 0;

    virtual std::vector<FunctionalPoint> GetAllFunctionalPoints() const = 0;
    virtual void AddFunctionalPoint(const FunctionalPoint& fp) = 0;

    virtual void LoadOnPose(const SE3& pose) = 0;
    virtual bool MaybeUpdateDynamicLayer(const CloudPtr& scan_body,
                                         const SE3& current_abs_pose,
                                         double current_score,
                                         double timestamp) = 0;
    virtual bool RebuildTargetIfNeeded() = 0;

    virtual void ResetDynamicLayer() = 0;
    virtual void Finish() = 0;
};

}  // namespace lightning::loc
