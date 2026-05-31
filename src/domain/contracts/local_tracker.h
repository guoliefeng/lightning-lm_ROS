#pragma once

#include "domain/geometry/pose3.h"
#include "domain/result/alignment_result.h"
#include "domain/sensor/scan_snapshot.h"

namespace lightning::domain::contracts {

class ILocalTracker {
   public:
    virtual ~ILocalTracker() = default;

    virtual result::AlignmentResult Track(const sensor::ScanSnapshot& snapshot,
                                          const geometry::Pose3& initial_pose) = 0;
    virtual void Reset() = 0;
};

}  // namespace lightning::domain::contracts
