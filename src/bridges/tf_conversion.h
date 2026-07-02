#pragma once

#include <string>

#include <geometry_msgs/TransformStamped.h>

#include "domain/geometry/pose3.h"

namespace lightning::loc::bridges {

geometry_msgs::TransformStamped ToTransformStamped(const domain::geometry::Pose3& pose,
                                                   const std::string& parent_frame,
                                                   const std::string& child_frame,
                                                   double timestamp_s);

}  // namespace lightning::loc::bridges
