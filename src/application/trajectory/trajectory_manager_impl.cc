#include "application/trajectory/trajectory_manager_impl.h"

#include <utility>

namespace lightning::application::trajectory {

TrajectoryManagerImpl::TrajectoryManagerImpl(CreateTrajectoryFn create_trajectory)
    : create_trajectory_(std::move(create_trajectory)) {}

std::shared_ptr<domain::contracts::ITrajectoryContext> TrajectoryManagerImpl::GetOrCreateTrajectory(
    const std::string& trajectory_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = trajectories_.find(trajectory_id);
    if (iter != trajectories_.end()) {
        return iter->second;
    }

    if (!create_trajectory_) {
        return nullptr;
    }

    auto trajectory = create_trajectory_(trajectory_id);
    if (!trajectory) {
        return nullptr;
    }

    trajectories_[trajectory_id] = trajectory;
    return trajectory;
}

std::shared_ptr<domain::contracts::ITrajectoryContext> TrajectoryManagerImpl::FindTrajectory(
    const std::string& trajectory_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto iter = trajectories_.find(trajectory_id);
    return iter == trajectories_.end() ? nullptr : iter->second;
}

std::vector<std::string> TrajectoryManagerImpl::ListTrajectoryIds() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    ids.reserve(trajectories_.size());
    for (const auto& [id, _] : trajectories_) {
        ids.push_back(id);
    }
    return ids;
}

bool TrajectoryManagerImpl::RemoveTrajectory(const std::string& trajectory_id) {
    std::shared_ptr<TrajectoryContextImpl> removed;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto iter = trajectories_.find(trajectory_id);
        if (iter == trajectories_.end()) {
            return false;
        }
        removed = iter->second;
        trajectories_.erase(iter);
    }

    if (removed) {
        removed->Stop();
    }
    return true;
}

}  // namespace lightning::application::trajectory
