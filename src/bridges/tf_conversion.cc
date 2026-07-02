#include "bridges/tf_conversion.h"

#include "core/lightning_math.hpp"

namespace lightning::loc::bridges {

geometry_msgs::TransformStamped ToTransformStamped(const domain::geometry::Pose3& pose,
                                                   const std::string& parent_frame,
                                                   const std::string& child_frame,
                                                   double timestamp_s) {
    geometry_msgs::TransformStamped msg;
    msg.header.frame_id = parent_frame;
    msg.header.stamp = math::FromSec(timestamp_s);
    msg.child_frame_id = child_frame;

    msg.transform.translation.x = pose.translation.x();
    msg.transform.translation.y = pose.translation.y();
    msg.transform.translation.z = pose.translation.z();

    msg.transform.rotation.x = pose.rotation.x();
    msg.transform.rotation.y = pose.rotation.y();
    msg.transform.rotation.z = pose.rotation.z();
    msg.transform.rotation.w = pose.rotation.w();

    return msg;
}

}  // namespace lightning::loc::bridges
