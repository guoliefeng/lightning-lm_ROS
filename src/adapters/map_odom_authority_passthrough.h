#pragma once

#include <mutex>

#include "domain/contracts/map_odom_authority.h"

namespace lightning::adapters {

class PassthroughMapOdomAuthority : public domain::contracts::IMapOdomAuthority {
   public:
    bool UpdateFromLocalization(const domain::result::LocalizationResult& localization,
                                const domain::geometry::Pose3& odom_pose) override;
    domain::geometry::Pose3 GetMapToOdom() const override;
    void Reset() override;
    void Freeze() override;
    void Unfreeze() override;
    bool IsFrozen() const override;

   private:
    mutable std::mutex mutex_;
    domain::geometry::Pose3 map_to_odom_ = domain::geometry::Pose3::Identity();
    bool frozen_ = false;
};

}  // namespace lightning::adapters
