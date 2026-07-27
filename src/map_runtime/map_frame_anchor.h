#pragma once

#include <string>

#include "common/eigen_types.h"
#include "domain/geometry/pose3.h"

namespace lightning {

class YAML_IO;

/// INS / 外部里程计坐标系到定位地图坐标系的固定锚定变换。
/// p_map = T_ins_to_map * p_ins
class MapFrameAnchor {
   public:
    MapFrameAnchor() = default;

    bool LoadFromYaml(const std::string& yaml_path);
    bool LoadFromYaml(const YAML_IO& yaml);

    bool enabled() const { return enabled_; }
    const SE3& T_ins_to_map() const { return T_ins_to_map_; }

    SE3 TransformInsToMap(const SE3& pose_ins) const;
    domain::geometry::Pose3 TransformPoseInsToMap(const domain::geometry::Pose3& pose_ins) const;
    Vec3d TransformPointInsToMap(const Vec3d& point_ins) const;

   private:
    bool enabled_ = false;
    SE3 T_ins_to_map_ = SE3();

    static SE3 PoseFromYamlNode(const YAML_IO& yaml, const std::string& node, const std::string& subnode);
    static SE3 PoseFromRpyTranslation(const Vec3d& translation, const Vec3d& rpy_deg);
};

}  // namespace lightning
