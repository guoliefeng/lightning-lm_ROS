#pragma once

#include <cstddef>

#include "domain/contracts/global_initializer.h"

namespace lightning::adapters {

class PassthroughGlobalInitializer : public domain::contracts::IGlobalInitializer {
   public:
    struct Options {
        std::size_t min_points = 500;
    };

    PassthroughGlobalInitializer();
    explicit PassthroughGlobalInitializer(Options options);

    bool IsReady(const domain::sensor::ScanSnapshot& snapshot) const override;
    domain::result::AlignmentResult Initialize(const domain::sensor::ScanSnapshot& snapshot,
                                               const domain::result::MapState* prior_map_state) override;
    void Reset() override;

   private:
    Options options_;
};

}  // namespace lightning::adapters
