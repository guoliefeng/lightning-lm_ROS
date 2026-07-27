#!/usr/bin/env python3
"""从评测 bag 快速导出 TUM 轨迹（流式解析 /tf，避免全量缓存）。"""
import argparse
import math
import sys

import numpy as np
import rosbag

try:
    import yaml
except ImportError:
    yaml = None


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tf_to_mat(transform):
    T = np.eye(4)
    t = transform.translation
    q = transform.rotation
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def pose_from_rpy_translation(translation, rpy_deg):
    roll, pitch, yaw = [math.radians(v) for v in rpy_deg]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = translation
    return T


def load_ins_to_map(config_path):
    if not config_path or yaml is None:
        return np.eye(4)
    with open(config_path, "r", encoding="utf-8") as fin:
        cfg = yaml.safe_load(fin).get("map_frame", {})
    if not cfg.get("enabled", False):
        return np.eye(4)
    if cfg.get("mode", "fixed") == "anchor_pair":
        def node_pose(node):
            n = cfg.get(node, {})
            if all(k in n for k in ("x", "y", "z", "qx", "qy", "qz", "qw")):
                T = np.eye(4)
                T[:3, :3] = quat_to_mat(n["qx"], n["qy"], n["qz"], n["qw"])
                T[:3, 3] = [n["x"], n["y"], n["z"]]
                return T
            return pose_from_rpy_translation(n.get("translation", [0, 0, 0]), n.get("rpy_deg", [0, 0, 0]))
        return node_pose("map_anchor") @ np.linalg.inv(node_pose("ins_anchor"))
    translation = cfg.get("ins_to_map_translation", [0.0, 0.0, 0.0])
    rpy_deg = cfg.get("ins_to_map_rotation_rpy_deg", [0.0, 0.0, 0.0])
    return pose_from_rpy_translation(translation, rpy_deg)


def write_tum(path, poses):
    with open(path, "w", encoding="utf-8") as fout:
        for stamp, T in poses:
            x, y, z, w = mat_to_quat(T[:3, :3])
            t = T[:3, 3]
            fout.write("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (
                stamp, t[0], t[1], t[2], x, y, z, w))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--est-out", required=True)
    parser.add_argument("--ref-out", required=True)
    parser.add_argument("--ref-topic", default="/localization/ins")
    parser.add_argument("--config", default="")
    parser.add_argument("--sample-dt", type=float, default=0.05,
                        help="输出位姿最小时间间隔 (s)")
    args = parser.parse_args()

    T_ins_to_map = load_ins_to_map(args.config)
    T_mo_latest = None
    T_ob_latest = None
    t_mo = 0.0
    t_ob = 0.0
    est = []
    ref = []
    last_est_t = -1e9
    last_ref_t = -1e9

    def maybe_append_est(stamp):
        nonlocal last_est_t
        if T_mo_latest is None or T_ob_latest is None:
            return
        if stamp < last_est_t - 0.1:
            last_est_t = -1e9
        if stamp - last_est_t < args.sample_dt:
            return
        est.append((stamp, T_mo_latest @ T_ob_latest))
        last_est_t = stamp

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, _ in bag.read_messages(topics=["/tf", args.ref_topic]):
            if topic == "/tf":
                for tr in msg.transforms:
                    stamp = tr.header.stamp.to_sec()
                    if tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                        T_ob_latest = tf_to_mat(tr.transform)
                        t_ob = stamp
                        maybe_append_est(stamp)
                    elif tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                        T_mo_latest = tf_to_mat(tr.transform)
                        t_mo = stamp
                        maybe_append_est(stamp)
            else:
                stamp = msg.header.stamp.to_sec()
                if stamp < last_ref_t - 0.1:
                    last_ref_t = -1e9
                if stamp - last_ref_t < args.sample_dt:
                    continue
                T = np.eye(4)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
                T[:3, 3] = [p.x, p.y, p.z]
                if not np.allclose(T_ins_to_map, np.eye(4)):
                    T = T_ins_to_map @ T
                ref.append((stamp, T))
                last_ref_t = stamp

    if not est:
        print("no estimated poses from TF", file=sys.stderr)
        return 1
    if not ref:
        print("no reference poses on", args.ref_topic, file=sys.stderr)
        return 1

    write_tum(args.est_out, est)
    write_tum(args.ref_out, ref)
    print("est: %d poses -> %s" % (len(est), args.est_out))
    print("ref: %d poses -> %s" % (len(ref), args.ref_out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
