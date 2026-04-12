#pragma once

#include <memory>
#include <string>
#include <vector>

#include "domain/contracts/trajectory_context.h"

namespace lightning::domain::contracts {

class ITrajectoryManager {
   public:
    virtual ~ITrajectoryManager() = default;

    virtual std::shared_ptr<ITrajectoryContext> GetOrCreateTrajectory(const std::string& trajectory_id) = 0;
    virtual std::shared_ptr<ITrajectoryContext> FindTrajectory(const std::string& trajectory_id) const = 0;
    virtual std::vector<std::string> ListTrajectoryIds() const = 0;
    virtual bool RemoveTrajectory(const std::string& trajectory_id) = 0;
};

}  // namespace lightning::domain::contracts
