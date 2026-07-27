#!/usr/bin/env python3
"""在线定位精度评估：对比 TF(map->odom * odom->base_link) 与 INS 参考轨迹。

用法:
    python3 eval_loc_accuracy.py <eval.bag> [--ref-topic /localization/ins] [--csv out.csv]
    python3 eval_loc_accuracy.py eval.bag --config config/yangpu_qc.yaml

评估指标:
    - 平面位置误差 (m): mean / rmse / max / p95
    - 高程误差 (m):    mean / rmse / max
    - 航向角误差 (deg): mean / rmse / max
"""
import argparse
import math

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


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


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


def pose_from_yaml_node(cfg, subnode):
    node = cfg.get(subnode, {})
    if all(k in node for k in ("x", "y", "z", "qx", "qy", "qz", "qw")):
        T = np.eye(4)
        qx, qy, qz, qw = node["qx"], node["qy"], node["qz"], node["qw"]
        T[:3, :3] = quat_to_mat(qx, qy, qz, qw)
        T[:3, 3] = [node["x"], node["y"], node["z"]]
        return T
    translation = node.get("translation", [0.0, 0.0, 0.0])
    rpy_deg = node.get("rpy_deg", [0.0, 0.0, 0.0])
    return pose_from_rpy_translation(translation, rpy_deg)


def load_ins_to_map_transform(config_path):
    if not config_path:
        return np.eye(4), False
    if yaml is None:
        raise RuntimeError("PyYAML required for --config; pip install pyyaml")
    with open(config_path, "r", encoding="utf-8") as fin:
        cfg = yaml.safe_load(fin).get("map_frame", {})
    if not cfg.get("enabled", False):
        return np.eye(4), False

    mode = cfg.get("mode", "fixed")
    if mode == "anchor_pair":
        T_ins = pose_from_yaml_node(cfg, "ins_anchor")
        T_map = pose_from_yaml_node(cfg, "map_anchor")
        T_ins_to_map = T_map @ np.linalg.inv(T_ins)
    else:
        translation = cfg.get("ins_to_map_translation", [0.0, 0.0, 0.0])
        rpy_deg = cfg.get("ins_to_map_rotation_rpy_deg", [0.0, 0.0, 0.0])
        T_ins_to_map = pose_from_rpy_translation(translation, rpy_deg)

    yaw_deg = math.degrees(math.atan2(T_ins_to_map[1, 0], T_ins_to_map[0, 0]))
    print("map_frame anchor: trans [%.3f, %.3f, %.3f] m, yaw %.2f deg" % (
        T_ins_to_map[0, 3], T_ins_to_map[1, 3], T_ins_to_map[2, 3], yaw_deg))
    return T_ins_to_map, True


def transform_pose(T_ins_to_map, T_pose):
    return T_ins_to_map @ T_pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--ref-topic", default="/localization/ins")
    parser.add_argument("--csv", default="")
    parser.add_argument("--min-move", type=float, default=0.0,
                        help="只统计参考位移超过该值(m)后的样本")
    parser.add_argument("--align", action="store_true",
                        help="先做 2D 刚体对齐(Umeyama)再统计 ATE，消除地图系与参考系的常值偏移")
    parser.add_argument("--window", type=float, default=0.0,
                        help="只统计前 N 秒（0 表示全程）")
    parser.add_argument("--config", default="",
                        help="从 yaml 读取 map_frame.T_ins_to_map 并变换参考轨迹")
    parser.add_argument("--ins-to-map-trans", default="",
                        help="手动指定平移 dx,dy,dz (m)，覆盖 config 中的平移")
    parser.add_argument("--ins-to-map-yaw", type=float, default=None,
                        help="手动指定 yaw 偏移 (deg)，覆盖 config 中的旋转")
    args = parser.parse_args()

    T_ins_to_map, anchor_enabled = load_ins_to_map_transform(args.config)
    if args.ins_to_map_trans:
        vals = [float(v) for v in args.ins_to_map_trans.split(",")]
        T_ins_to_map[:3, 3] = vals[:3]
        anchor_enabled = True
    if args.ins_to_map_yaw is not None:
        yaw = math.radians(args.ins_to_map_yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        T_ins_to_map[:3, :3] = R @ T_ins_to_map[:3, :3]
        anchor_enabled = True

    map_to_odom = []   # (stamp, 4x4)
    odom_to_base = []  # (stamp, 4x4)
    ref = []           # (stamp, 4x4)

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, _ in bag.read_messages(topics=["/tf", args.ref_topic]):
            if topic == "/tf":
                for tr in msg.transforms:
                    stamp = tr.header.stamp.to_sec()
                    if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                        map_to_odom.append((stamp, tf_to_mat(tr.transform)))
                    elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                        odom_to_base.append((stamp, tf_to_mat(tr.transform)))
            else:
                stamp = msg.header.stamp.to_sec()
                T = np.eye(4)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
                T[:3, 3] = [p.x, p.y, p.z]
                if anchor_enabled:
                    T = transform_pose(T_ins_to_map, T)
                ref.append((stamp, T))

    if anchor_enabled:
        print("reference trajectory transformed by T_ins_to_map")

    if not ref:
        print("no reference messages on", args.ref_topic)
        return
    if not map_to_odom or not odom_to_base:
        print("missing tf chains: map->odom=%d odom->base=%d" % (len(map_to_odom), len(odom_to_base)))
        return

    map_to_odom.sort(key=lambda x: x[0])
    odom_to_base.sort(key=lambda x: x[0])

    def find_nearest_tf(tfs, stamp, max_dt=0.05):
        ts = np.array([t for t, _ in tfs])
        idx = int(np.searchsorted(ts, stamp))
        cand = []
        if 0 <= idx < len(tfs):
            cand.append(tfs[idx])
        if idx - 1 >= 0:
            cand.append(tfs[idx - 1])
        if not cand:
            return None
        best = min(cand, key=lambda x: abs(x[0] - stamp))
        if abs(best[0] - stamp) > max_dt:
            return None
        return best[1]

    # 估计位姿：同一时间戳的两条 TF 组合
    est = []
    for stamp, T_mo in map_to_odom:
        T_ob = find_nearest_tf(odom_to_base, stamp)
        if T_ob is not None:
            est.append((stamp, T_mo @ T_ob))
    print("est poses: %d, ref poses: %d" % (len(est), len(ref)))
    if not est:
        print("no matched tf pairs")
        return

    ref_t = np.array([r[0] for r in ref])
    ref_start_xy = ref[0][1][:2, 3]

    rows = []
    for stamp, T_est in est:
        idx = np.searchsorted(ref_t, stamp)
        if idx == 0 or idx >= len(ref_t):
            continue
        t0, T0 = ref[idx - 1]
        t1, T1 = ref[idx]
        if t1 - t0 > 0.5:
            continue
        a = (stamp - t0) / (t1 - t0) if t1 > t0 else 0.0
        p_ref = (1 - a) * T0[:3, 3] + a * T1[:3, 3]
        yaw_ref = yaw_of(T0) + a * ((yaw_of(T1) - yaw_of(T0) + math.pi) % (2 * math.pi) - math.pi)

        if args.min_move > 0 and np.linalg.norm(p_ref[:2] - ref_start_xy) < args.min_move:
            continue

        p_est = T_est[:3, 3]
        e_xy = float(np.linalg.norm(p_est[:2] - p_ref[:2]))
        e_z = float(abs(p_est[2] - p_ref[2]))
        e_yaw = math.degrees(abs((yaw_of(T_est) - yaw_ref + math.pi) % (2 * math.pi) - math.pi))
        rows.append((stamp, e_xy, e_z, e_yaw, p_est[0], p_est[1], p_ref[0], p_ref[1]))

    if not rows:
        print("no matched samples between est and ref")
        return

    arr = np.array(rows)

    if args.window > 0:
        arr = arr[arr[:, 0] - arr[0, 0] <= args.window]
        print("windowed to first %.1fs: %d samples" % (args.window, len(arr)))

    if args.align and len(arr) >= 3:
        # 2D Umeyama（无尺度）：est -> ref
        P = arr[:, 4:6]  # est xy
        Q = arr[:, 6:8]  # ref xy
        mp, mq = P.mean(0), Q.mean(0)
        H = (P - mp).T @ (Q - mq)
        U, _, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, d]) @ U.T
        t = mq - R @ mp
        theta = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        print("alignment: rot %.2f deg, trans [%.2f, %.2f] m" % (theta, t[0], t[1]))
        P_aligned = (R @ P.T).T + t
        arr[:, 1] = np.linalg.norm(P_aligned - Q, axis=1)

    exy, ez, eyaw = arr[:, 1], arr[:, 2], arr[:, 3]

    def stats(v):
        return "mean %.3f  rmse %.3f  max %.3f  p95 %.3f" % (
            v.mean(), math.sqrt((v ** 2).mean()), v.max(), np.percentile(v, 95))

    print("samples: %d, duration: %.1fs" % (len(rows), arr[-1, 0] - arr[0, 0]))
    print("planar position error (m):", stats(exy))
    print("height error (m):         ", stats(ez))
    print("yaw error (deg):          ", stats(eyaw))

    if args.csv:
        np.savetxt(args.csv, arr, delimiter=",",
                   header="stamp,e_xy,e_z,e_yaw_deg,est_x,est_y,ref_x,ref_y", comments="")
        print("per-sample errors written to", args.csv)


if __name__ == "__main__":
    main()
