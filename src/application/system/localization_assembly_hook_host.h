#pragma once

#include <functional>
#include <string>

namespace lightning::application::system {

struct LocalizationAssembly;

class ILocalizationAssemblyHookHost {
   public:
    using TrajectoryAssemblyHook = std::function<void(const std::string& trajectory_id, LocalizationAssembly& assembly)>;

    virtual ~ILocalizationAssemblyHookHost() = default;
    virtual void SetTrajectoryAssemblyHook(TrajectoryAssemblyHook hook) = 0;
};

}  // namespace lightning::application::system
