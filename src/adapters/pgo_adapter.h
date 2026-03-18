#pragma once

#include <memory>

#include "core/localization/pose_graph/pgo.h"
#include "interfaces/fusion_engine.h"

namespace lightning::loc {

class PGOAdapter : public IFusionEngine {
   public:
    PGOAdapter();
    explicit PGOAdapter(std::shared_ptr<PGO> impl);

    void FeedDeadReckoning(const NavState& state) override;
    void FeedLidarOdom(const NavState& state) override;
    void FeedLocalization(const LocalizationResult& result) override;
    void SetHighFrequencyOutputCallback(OutputCallback cb) override;

    void SetDebug(bool debug = true);

   private:
    std::shared_ptr<PGO> impl_ = nullptr;
};

}  // namespace lightning::loc
