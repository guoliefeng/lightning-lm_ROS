#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "domain/contracts/plugin_registry.h"
#include "domain/contracts/map_state_repository.h"
#include "domain/contracts/system_root.h"
#include "domain/geometry/pose3.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::application::system {

struct LocalizationAssembly;

class SystemRootImpl : public domain::contracts::ISystemRoot {
   public:
    struct Options {
        std::string config_path;
        std::string global_map_path;
        std::string default_trajectory_id = "default";
        bool online_mode = false;
        bool with_ui = false;
        std::shared_ptr<domain::contracts::IPluginRegistry> plugin_registry = nullptr;
    };

    using TrajectoryAssemblyHook = std::function<void(const std::string& trajectory_id, LocalizationAssembly& assembly)>;

    SystemRootImpl();
    explicit SystemRootImpl(Options options);
    ~SystemRootImpl() override;

    bool Init(const std::string& config_path) override;
    void Shutdown() override;
    void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) override;
    std::shared_ptr<domain::contracts::ITrajectoryManager> GetTrajectoryManager() override;

    bool Start();
    void Stop();

    std::shared_ptr<domain::contracts::ITrajectoryContext> GetOrCreateDefaultTrajectory();
    std::shared_ptr<domain::contracts::ITrajectoryContext> GetOrCreateTrajectory(const std::string& trajectory_id);

    bool FeedImuToDefaultTrajectory(const domain::sensor::ImuData& imu);
    bool FeedCloudToDefaultTrajectory(const domain::sensor::CloudData& cloud);
    bool SetInitialPoseForDefaultTrajectory(const domain::geometry::Pose3& pose);

    void SetEventSink(const std::string& trajectory_id, std::shared_ptr<domain::contracts::IEventSink> sink);
    bool FeedImu(const std::string& trajectory_id, const domain::sensor::ImuData& imu);
    bool FeedCloud(const std::string& trajectory_id, const domain::sensor::CloudData& cloud);
    bool SetInitialPose(const std::string& trajectory_id, const domain::geometry::Pose3& pose);

    void SetTrajectoryAssemblyHook(TrajectoryAssemblyHook hook);

   private:
    std::shared_ptr<domain::contracts::ITrajectoryContext> FindTrajectoryLocked(const std::string& trajectory_id) const;
    std::shared_ptr<domain::contracts::ITrajectoryContext> CreateTrajectoryContext(const std::string& trajectory_id);

    Options options_;
    mutable std::mutex mutex_;
    bool initialized_ = false;
    bool started_ = false;

    std::shared_ptr<domain::contracts::ITrajectoryManager> trajectory_manager_ = nullptr;
    std::shared_ptr<domain::contracts::IPluginRegistry> plugin_registry_ = nullptr;
    std::shared_ptr<domain::contracts::IMapStateRepository> map_state_repository_ = nullptr;
    std::unordered_map<std::string, std::shared_ptr<domain::contracts::IEventSink>> event_sinks_;
    TrajectoryAssemblyHook trajectory_assembly_hook_;
};

}  // namespace lightning::application::system
