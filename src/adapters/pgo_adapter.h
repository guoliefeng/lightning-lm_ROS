#pragma once

#include <memory>

#include "domain/contracts/pose_graph_backend.h"
#include "core/localization/pose_graph/pgo.h"
#include "interfaces/fusion_engine.h"

namespace lightning::loc {

class PGOAdapter : public IFusionEngine, public domain::contracts::IPoseGraphBackend {
   public:
    PGOAdapter();
    explicit PGOAdapter(std::shared_ptr<PGO> impl);

    void FeedDeadReckoning(const NavState& state) override;
    void FeedLidarOdom(const NavState& state) override;
    void FeedLocalization(const LocalizationResult& result) override;
    void SetHighFrequencyOutputCallback(IFusionEngine::OutputCallback cb) override;

    void FeedMotionEstimate(const domain::result::MotionEstimate& motion) override;
    void FeedLocalizationResult(const domain::result::LocalizationResult& localization) override;
    void SetOutputCallback(domain::contracts::IPoseGraphBackend::OutputCallback callback) override;
    domain::result::LocalizationResult GetLatestResult() const override;
    void Reset() override;

    void SetDebug(bool debug = true);

   private:
    std::shared_ptr<PGO> impl_ = nullptr;
    domain::result::LocalizationResult latest_result_;
    domain::contracts::IPoseGraphBackend::OutputCallback backend_output_callback_;
};

}  // namespace lightning::loc
