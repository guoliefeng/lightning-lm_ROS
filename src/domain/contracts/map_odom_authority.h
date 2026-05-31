#pragma once

#include "domain/geometry/pose3.h"
#include "domain/result/localization_result.h"

namespace lightning::domain::contracts {

class IMapOdomAuthority {
   public:
    virtual ~IMapOdomAuthority() = default;

    virtual bool UpdateFromLocalization(const result::LocalizationResult& localization,
                                        const geometry::Pose3& odom_pose) = 0;
    virtual geometry::Pose3 GetMapToOdom() const = 0;
    virtual void Reset() = 0;
    virtual void Freeze() = 0;
    virtual void Unfreeze() = 0;
    virtual bool IsFrozen() const = 0;
};

}  // namespace lightning::domain::contracts
