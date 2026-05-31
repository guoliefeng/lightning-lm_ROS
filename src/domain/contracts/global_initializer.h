#pragma once

#include "domain/result/alignment_result.h"
#include "domain/result/map_state.h"
#include "domain/sensor/scan_snapshot.h"

namespace lightning::domain::contracts {

class IGlobalInitializer {
   public:
    virtual ~IGlobalInitializer() = default;

    virtual bool IsReady(const sensor::ScanSnapshot& snapshot) const = 0;
    virtual result::AlignmentResult Initialize(const sensor::ScanSnapshot& snapshot,
                                               const result::MapState* prior_map_state) = 0;
    virtual void Reset() = 0;
};

}  // namespace lightning::domain::contracts
