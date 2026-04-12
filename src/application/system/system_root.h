#pragma once

#include <memory>
#include <string>

#include "domain/contracts/event_sink.h"

namespace lightning::application::trajectory {
class ITrajectoryManager;
}

namespace lightning::application::system {

class ISystemRoot {
   public:
    virtual ~ISystemRoot() = default;

    virtual bool Init(const std::string& config_path) = 0;
    virtual void Shutdown() = 0;
    virtual void SetEventSink(std::shared_ptr<domain::contracts::IEventSink> sink) = 0;
    virtual std::shared_ptr<trajectory::ITrajectoryManager> GetTrajectoryManager() = 0;
};

}  // namespace lightning::application::system
