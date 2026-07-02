#pragma once

#include <memory>
#include <string>

#include "domain/contracts/event_sink.h"
#include "domain/contracts/trajectory_context.h"
#include "domain/contracts/trajectory_manager.h"
#include "domain/geometry/pose3.h"
#include "domain/sensor/cloud_data.h"
#include "domain/sensor/imu_data.h"

namespace lightning::domain::contracts {

class ISystemRoot {
   public:
    virtual ~ISystemRoot() = default;

    virtual bool Init(const std::string& config_path) = 0;
    virtual bool Start() = 0;
    virtual void Stop() = 0;
    virtual void Shutdown() = 0;
    virtual void SetEventSink(std::shared_ptr<IEventSink> sink) = 0;
    virtual void SetEventSink(const std::string& trajectory_id, std::shared_ptr<IEventSink> sink) = 0;
    virtual std::shared_ptr<ITrajectoryManager> GetTrajectoryManager() = 0;
    virtual std::shared_ptr<ITrajectoryContext> GetOrCreateTrajectory(const std::string& trajectory_id) = 0;
    virtual bool FeedImu(const std::string& trajectory_id, const sensor::ImuData& imu) = 0;
    virtual bool FeedCloud(const std::string& trajectory_id, const sensor::CloudData& cloud) = 0;
    virtual bool FeedGnss(const std::string&, const sensor::GnssData&) { return false; }
    virtual bool FeedOdometry(const std::string&, const sensor::OdometryData&) { return false; }
    virtual bool SetInitialPose(const std::string& trajectory_id, const geometry::Pose3& pose) = 0;
};

}  // namespace lightning::domain::contracts
