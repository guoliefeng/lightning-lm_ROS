#!/usr/bin/env python3
"""在线定位精度评估：对比 TF(map->odom * odom->base_link) 与 INS 参考轨迹。

用法:
    python3 eval_loc_accuracy.py <eval.bag> [--ref-topic /localization/ins] [--csv out.csv]

评估指标:
    - 平面位置误差 (m): mean / rmse / max / p95
    - 高程误差 (m):    mean / rmse / max
    - 航向角误差 (deg): mean / rmse / max
"""
import argparse
import math

import numpy as np
import rosbag


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
    args = parser.parse_args()

    map_to_odom = {}   # stamp -> 4x4
    odom_to_base = {}  # stamp -> 4x4
    ref = []           # (stamp, 4x4)

    with rosbag.Bag(args.bag) as bag:
        for topic, msg, _ in bag.read_messages(topics=["/tf", args.ref_topic]):
            if topic == "/tf":
                for tr in msg.transforms:
                    stamp = tr.header.stamp.to_sec()
                    if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                        map_to_odom[stamp] = tf_to_mat(tr.transform)
                    elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_link":
                        odom_to_base[stamp] = tf_to_mat(tr.transform)
            else:
                stamp = msg.header.stamp.to_sec()
                T = np.eye(4)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
                T[:3, 3] = [p.x, p.y, p.z]
                ref.append((stamp, T))

    if not ref:
        print("no reference messages on", args.ref_topic)
        return
    if not map_to_odom or not odom_to_base:
        print("missing tf chains: map->odom=%d odom->base=%d" % (len(map_to_odom), len(odom_to_base)))
        return

    # 估计位姿：同一时间戳的两条 TF 组合
    est = []
    for stamp, T_mo in sorted(map_to_odom.items()):
        T_ob = odom_to_base.get(stamp)
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
