#pragma once

#include "domain/contracts/local_tracker.h"

namespace lightning::adapters {

class PassthroughLocalTracker : public domain::contracts::ILocalTracker {
   public:
    domain::result::AlignmentResult Track(const domain::sensor::ScanSnapshot& snapshot,
                                          const domain::geometry::Pose3& initial_pose) override;
    void Reset() override;
};

}  // namespace lightning::adapters
