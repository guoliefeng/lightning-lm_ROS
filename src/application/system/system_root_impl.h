#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "domain/contracts/map_odom_authority.h"
#include "domain/contracts/plugin_registry.h"
#include "domain/contracts/map_state_repository.h"
#include "domain/contracts/system_root.h"
#include "domain/geometry/pose3.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::application::system {

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

    SystemRootImpl();
    explicit SystemRootImpl(Options options);
    ~SystemRootImpl() override;

    bool Init(const std::string& config_path) override;
    bool Start() override;
    void Stop() override;
    void Shutdown() override;
    void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) override;
    void SetEventSink(const std::string& trajectory_id,
                      std::shared_ptr<domain::contracts::IEventSink> sink) override;
    std::shared_ptr<domain::contracts::ITrajectoryManager> GetTrajectoryManager() override;

    std::shared_ptr<domain::contracts::ITrajectoryContext> GetOrCreateDefaultTrajectory();
    std::shared_ptr<domain::contracts::ITrajectoryContext> GetOrCreateTrajectory(
        const std::string& trajectory_id) override;

    bool FeedImuToDefaultTrajectory(const domain::sensor::ImuData& imu);
    bool FeedCloudToDefaultTrajectory(const domain::sensor::CloudData& cloud);
    bool SetInitialPoseForDefaultTrajectory(const domain::geometry::Pose3& pose);

    bool FeedImu(const std::string& trajectory_id, const domain::sensor::ImuData& imu) override;
    bool FeedCloud(const std::string& trajectory_id, const domain::sensor::CloudData& cloud) override;
    bool SetInitialPose(const std::string& trajectory_id, const domain::geometry::Pose3& pose) override;

    std::shared_ptr<domain::contracts::IMapOdomAuthority> GetMapOdomAuthority() const;

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
    std::shared_ptr<domain::contracts::IMapOdomAuthority> map_odom_authority_ = nullptr;
    std::unordered_map<std::string, std::shared_ptr<domain::contracts::IEventSink>> event_sinks_;
};

}  // namespace lightning::application::system
