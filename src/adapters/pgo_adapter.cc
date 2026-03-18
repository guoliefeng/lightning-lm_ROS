#include "adapters/pgo_adapter.h"

namespace lightning::loc {

PGOAdapter::PGOAdapter() : impl_(std::make_shared<PGO>()) {}

PGOAdapter::PGOAdapter(std::shared_ptr<PGO> impl) : impl_(std::move(impl)) {}

void PGOAdapter::FeedDeadReckoning(const NavState& state) { impl_->ProcessDR(state); }

void PGOAdapter::FeedLidarOdom(const NavState& state) { impl_->ProcessLidarOdom(state); }

void PGOAdapter::FeedLocalization(const LocalizationResult& result) { impl_->ProcessLidarLoc(result); }

void PGOAdapter::SetHighFrequencyOutputCallback(OutputCallback cb) {
    impl_->SetHighFrequencyGlobalOutputHandleFunction(std::move(cb));
}

void PGOAdapter::SetDebug(bool debug) { impl_->SetDebug(debug); }

}  // namespace lightning::loc
