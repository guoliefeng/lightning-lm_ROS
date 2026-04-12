#pragma once

#include <memory>
#include <string>

#include "domain/contracts/event_sink.h"
#include "domain/contracts/trajectory_manager.h"

namespace lightning::domain::contracts {

class ISystemRoot {
   public:
    virtual ~ISystemRoot() = default;

    virtual bool Init(const std::string& config_path) = 0;
    virtual void Shutdown() = 0;
    virtual void SetEventSink(std::shared_ptr<IEventSink> sink) = 0;
    virtual std::shared_ptr<ITrajectoryManager> GetTrajectoryManager() = 0;
};

}  // namespace lightning::domain::contracts
