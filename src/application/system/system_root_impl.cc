#include "application/system/system_root_impl.h"

#include <utility>

#include <glog/logging.h>

#include "application/system/system_assembler.h"
#include "application/trajectory/trajectory_context_impl.h"
#include "application/trajectory/trajectory_manager_impl.h"

namespace lightning::application::system {

SystemRootImpl::SystemRootImpl() = default;

SystemRootImpl::SystemRootImpl(Options options) : options_(std::move(options)) {}

SystemRootImpl::~SystemRootImpl() { Shutdown(); }

bool SystemRootImpl::Init(const std::string& config_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) {
        return true;
    }

    if (!config_path.empty()) {
        options_.config_path = config_path;
    }
    if (options_.config_path.empty()) {
        LOG(ERROR) << "SystemRootImpl requires a config path";
        return false;
    }

    plugin_registry_ = options_.plugin_registry;

    auto create_trajectory = [this](const std::string& trajectory_id) {
        return CreateTrajectoryContext(trajectory_id);
    };
    trajectory_manager_ = std::make_shared<trajectory::TrajectoryManagerImpl>(std::move(create_trajectory));

    initialized_ = true;
    return true;
}

void SystemRootImpl::Shutdown() {
    Stop();

    std::shared_ptr<domain::contracts::ITrajectoryManager> manager;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager = trajectory_manager_;
        trajectory_manager_.reset();
        map_state_repository_.reset();
        event_sinks_.clear();
        initialized_ = false;
    }

    if (manager) {
        for (const auto& id : manager->ListTrajectoryIds()) {
            manager->RemoveTrajectory(id);
        }
    }
}

void SystemRootImpl::SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) {
    SetEventSink(options_.default_trajectory_id, std::move(sink));
}

std::shared_ptr<domain::contracts::ITrajectoryManager> SystemRootImpl::GetTrajectoryManager() {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_manager_;
}

bool SystemRootImpl::Start() {
    auto trajectory = GetOrCreateDefaultTrajectory();
    if (!trajectory) {
        return false;
    }

    trajectory->Start();
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = true;
    return true;
}

void SystemRootImpl::Stop() {
    std::shared_ptr<domain::contracts::ITrajectoryManager> manager;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager = trajectory_manager_;
        started_ = false;
    }

    if (!manager) {
        return;
    }

    for (const auto& id : manager->ListTrajectoryIds()) {
        auto trajectory = manager->FindTrajectory(id);
        if (trajectory) {
            trajectory->Stop();
        }
    }
}

std::shared_ptr<domain::contracts::ITrajectoryContext> SystemRootImpl::GetOrCreateDefaultTrajectory() {
    return GetOrCreateTrajectory(options_.default_trajectory_id);
}

std::shared_ptr<domain::contracts::ITrajectoryContext> SystemRootImpl::GetOrCreateTrajectory(
    const std::string& trajectory_id) {
    std::shared_ptr<domain::contracts::ITrajectoryManager> manager;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manager = trajectory_manager_;
    }
    return manager ? manager->GetOrCreateTrajectory(trajectory_id) : nullptr;
}

bool SystemRootImpl::FeedImuToDefaultTrajectory(const domain::sensor::ImuData& imu) {
    return FeedImu(options_.default_trajectory_id, imu);
}

bool SystemRootImpl::FeedCloudToDefaultTrajectory(const domain::sensor::CloudData& cloud) {
    return FeedCloud(options_.default_trajectory_id, cloud);
}

bool SystemRootImpl::SetInitialPoseForDefaultTrajectory(const domain::geometry::Pose3& pose) {
    return SetInitialPose(options_.default_trajectory_id, pose);
}

void SystemRootImpl::SetEventSink(const std::string& trajectory_id,
                                  std::shared_ptr<domain::contracts::IEventSink> sink) {
    std::shared_ptr<domain::contracts::ITrajectoryContext> trajectory;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        event_sinks_[trajectory_id] = sink;
        trajectory = FindTrajectoryLocked(trajectory_id);
    }

    if (trajectory) {
        trajectory->SetEventSink(std::move(sink));
    }
}

bool SystemRootImpl::FeedImu(const std::string& trajectory_id, const domain::sensor::ImuData& imu) {
    auto trajectory = GetOrCreateTrajectory(trajectory_id);
    if (!trajectory) {
        return false;
    }
    trajectory->FeedImu(imu);
    return true;
}

bool SystemRootImpl::FeedCloud(const std::string& trajectory_id, const domain::sensor::CloudData& cloud) {
    auto trajectory = GetOrCreateTrajectory(trajectory_id);
    if (!trajectory) {
        return false;
    }
    trajectory->FeedCloud(cloud);
    return true;
}

bool SystemRootImpl::SetInitialPose(const std::string& trajectory_id, const domain::geometry::Pose3& pose) {
    auto trajectory = GetOrCreateTrajectory(trajectory_id);
    if (!trajectory) {
        return false;
    }
    trajectory->SetInitialPose(pose);
    return true;
}

std::shared_ptr<domain::contracts::ITrajectoryContext> SystemRootImpl::FindTrajectoryLocked(
    const std::string& trajectory_id) const {
    return trajectory_manager_ ? trajectory_manager_->FindTrajectory(trajectory_id) : nullptr;
}

std::shared_ptr<domain::contracts::ITrajectoryContext> SystemRootImpl::CreateTrajectoryContext(
    const std::string& trajectory_id) {
    LocalizationAssemblyOptions assembly_options;
    assembly_options.yaml_path = options_.config_path;
    assembly_options.global_map_path = options_.global_map_path;
    assembly_options.online_mode = options_.online_mode;
    assembly_options.with_ui = options_.with_ui;
    assembly_options.plugin_registry = plugin_registry_;

    auto assembly = SystemAssembler::AssembleLocalization(assembly_options);
    if (!assembly.localizer || !assembly.sensor_pipeline || !assembly.motion_estimator || !assembly.state_estimator) {
        LOG(ERROR) << "failed to assemble trajectory: " << trajectory_id;
        return nullptr;
    }

    if (!plugin_registry_) {
        plugin_registry_ = assembly.plugin_registry;
    }
    if (!map_state_repository_) {
        map_state_repository_ = assembly.map_state_repository;
    }
    trajectory::TrajectoryContextImpl::Options context_options;
    context_options.id = trajectory_id;
    context_options.config_path = options_.config_path;
    context_options.online_mode = options_.online_mode;

    auto context = std::make_shared<trajectory::TrajectoryContextImpl>(context_options, std::move(assembly));
    auto sink_iter = event_sinks_.find(trajectory_id);
    if (sink_iter != event_sinks_.end()) {
        context->SetEventSink(sink_iter->second);
    }
    if (!context->Initialize()) {
        LOG(ERROR) << "failed to initialize trajectory: " << trajectory_id;
        return nullptr;
    }
    return context;
}

}  // namespace lightning::application::system
