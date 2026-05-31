#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "domain/contracts/trajectory_manager.h"

namespace lightning::application::trajectory {

class TrajectoryManagerImpl : public domain::contracts::ITrajectoryManager {
   public:
    using CreateTrajectoryFn =
        std::function<std::shared_ptr<domain::contracts::ITrajectoryContext>(const std::string&)>;

    explicit TrajectoryManagerImpl(CreateTrajectoryFn create_trajectory);
    ~TrajectoryManagerImpl() override = default;

    std::shared_ptr<domain::contracts::ITrajectoryContext> GetOrCreateTrajectory(
        const std::string& trajectory_id) override;
    std::shared_ptr<domain::contracts::ITrajectoryContext> FindTrajectory(const std::string& trajectory_id) const override;
    std::vector<std::string> ListTrajectoryIds() const override;
    bool RemoveTrajectory(const std::string& trajectory_id) override;

   private:
    CreateTrajectoryFn create_trajectory_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<domain::contracts::ITrajectoryContext>> trajectories_;
};

}  // namespace lightning::application::trajectory
