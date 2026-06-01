#pragma once

#include <memory>

#include "common/point_def.h"
#include "domain/contracts/global_initializer.h"
#include "domain/contracts/local_tracker.h"
#include "domain/contracts/localizer.h"

namespace lightning::adapters {

class LegacyLocalizerRelocalizationAdapter : public domain::contracts::IGlobalInitializer,
                                             public domain::contracts::ILocalTracker {
   public:
    explicit LegacyLocalizerRelocalizationAdapter(std::shared_ptr<domain::contracts::ILocalizer> localizer);

    bool IsReady(const domain::sensor::ScanSnapshot& snapshot) const override;
    domain::result::AlignmentResult Initialize(const domain::sensor::ScanSnapshot& snapshot,
                                               const domain::result::MapState* prior_map_state) override;
    domain::result::AlignmentResult Track(const domain::sensor::ScanSnapshot& snapshot,
                                          const domain::geometry::Pose3& initial_pose) override;
    void Reset() override;

   private:
    CloudPtr BuildLegacyCloud(const domain::sensor::ScanSnapshot& snapshot) const;
    domain::result::AlignmentResult ProcessWithLocalizer(const domain::sensor::ScanSnapshot& snapshot);

    std::shared_ptr<domain::contracts::ILocalizer> localizer_ = nullptr;
};

}  // namespace lightning::adapters
