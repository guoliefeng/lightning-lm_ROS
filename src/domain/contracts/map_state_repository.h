#pragma once

#include <string>
#include <vector>

#include "domain/result/map_state.h"

namespace lightning::domain::contracts {

class IMapStateRepository {
   public:
    virtual ~IMapStateRepository() = default;

    virtual bool Load(const std::string& map_id, result::MapState& map_state) = 0;
    virtual void Save(const result::MapState& map_state) = 0;
    virtual bool Remove(const std::string& map_id) = 0;
    virtual std::vector<std::string> ListMapIds() const = 0;
};

}  // namespace lightning::domain::contracts
