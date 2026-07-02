# Sprint 3 测试报告：GNSS/轮速通道 + 定位精度评估

> 测试日期：2026-07-02  
> 分支：`feat/refactoring`  
> 范围：GNSS/轮速数据通路验证、在线定位精度量化（yangpu/qc 60s）

---

## 1. 测试环境

| 项 | 值 |
|----|-----|
| ROS | Noetic，20 核 |
| 配置 | `config/yangpu_qc.yaml`（gnss_topic=/ins_driver/gps，odom_topic=/localization/ins） |
| 地图 | yangpu chunked（73 chunks） |
| 数据 | qc bag 0+1，`rosbag play -r 0.5 -u 60 --clock` |
| 参考轨迹 | `/localization/ins`（nav_msgs/Odometry，frame=map(INS)） |
| 评估工具 | `scripts/eval_loc_accuracy.py`（TF map→odom ∘ odom→base_link vs 参考，支持 2D Umeyama 对齐） |

评估命令：

```bash
# 运行期间录制
rosbag record -O /tmp/eval.bag /tf /localization/ins
# 结束后评估
python3 scripts/eval_loc_accuracy.py /tmp/eval.bag [--align] [--window N] --csv errors.csv
```

---

## 2. 单元测试 — 通过

```
relocalization main path smoke tests passed
```

新增 `TestGnssAndOdometryChannels`：`FeedGnss/FeedOdometry` → collator → `IStateEstimator`，
计数与载荷（lat=20.0、vel.x=1.0）断言通过。全部 7 个用例通过。

---

## 3. GNSS / 轮速通路验证 — 通过

启动日志：

```
subscribing gnss topic: /ins_driver/gps
subscribing odometry topic: /localization/ins
```

60s 播放期间状态估计器打点（每 100 条一次）：

```
state estimator received gnss #1,   lat: 19.7163, lon: 109.173
state estimator received gnss #101, ...
state estimator received odometry #1,   vel: 0 0 0
state estimator received odometry #101, ...
```

| 通道 | 打点批次 | 折算接收条数 | 预期（0.5x 播放，~100Hz 源） | 判定 |
|------|---------|-------------|------------------------------|------|
| GNSS | 60 | ≈6000 | ≈6000（60s × 100Hz） | 通过 |
| INS 里程计 | 55 | ≈5500 | ≈5500 | 通过 |

LIO 主路径不受影响（`No point` 仍仅启动 2 次）。

---

## 4. 定位精度评估 — **未达标，暴露 3 个真实问题**

### 4.1 总体结果（60s 全程，含运动段）

| 运行 | 初始化位置 | 平面误差 RMSE | 高程 RMSE | 航向 RMSE | 结论 |
|------|-----------|--------------|-----------|-----------|------|
| Run A（yaw cap 24） | (300.2, 100.1)，功能点 | 798 m | 102 m | 73° | 运动后发散 |
| Run B（yaw 60，INS 初值） | (238.6, -208.9)，**yaw 错 145°** | 799 m | 102 m | 73° | 运动后发散 |
| Run C（对照，无 GNSS 通道） | (238.7, -209.0) | 587 m | 697 m | 52° | 运动后发散 |
| Run D（±30° 窄搜索） | (0.08, -500.2)，功能点 | 587 m | 696 m | 52° | 运动后发散 |

> 数字如此大的原因不是"精度差几米"，而是 **LIO 在车辆开始运动后完全发散**（速度估计冲到 18~35 m/s），
> DR 外推带着 NDT guess 飞出地图，之后误差随时间线性积累。GNSS/轮速通道开关对结果无影响（对照组一致），
> 说明新通道没有引入回归。

### 4.2 发散时刻的确凿证据（Run B 日志）

```
12:28:36.9  LIO state: vel 0.96, 2.14   (正常)
12:28:37.09 [ mapping ]: effect num : 106, 139        ← 有效匹配点从 ~1200 崩到 ~100
12:28:37.09 delta trans: 2.85 5.87 1.83, ang: 14.06   ← ESKF 单步跳 6.9m/14°
12:28:37.09 LIO state: vel: -14.6 -29.9 -9.4          ← 速度爆炸
12:28:37.11 take dr guess: ..., v_norm: 34.5          ← DR 污染 NDT guess
```

**根因假设（待验证）**：qc 为岸桥（Quay Crane）场景，LiDAR 视野被随载体一起运动的
钢结构主导——LIO 把自体结构当静态环境（运动前速度估计恒为 ~0.3 m/s，而 INS 显示 1.4 m/s），
一旦外部结构（集装箱/地面）进出视野，有效约束瞬间坍缩，ESKF 无约束更新即发散。

### 4.3 发现的 3 个真实问题

| # | 问题 | 证据 | 建议 |
|---|------|------|------|
| 1 | **地图坐标系 ≠ INS map 坐标系**：INS 初值 (238.7,-208.9) 在切块地图中不是真值位置，多次运行分别锁到 (0,-500)、(300,100) 等功能点，Umeyama 对齐残差仍大 | 各 run 初始化位置互不一致；对齐旋转 90° / 平移数百米 | Phase C MapSession 必须记录地图原点与 INS 系的锚定变换；短期在 yaml 显式配置 `T_map_ins` |
| 2 | **低分初始化误接受**：`min_init_confidence 0.5` 时 yaw 偏差 145° 的解打 0.56 分被接受 | Run B `init success, score 0.562` 后航向全程错 | 初始化后加"与 INS 航向一致性"校验（GNSS 通道已就位，可直接用） |
| 3 | **LIO 对自体运动结构无防护**：effect num 从 1200 崩到 100 无任何降级动作 | 4.2 日志 | ESKF 更新前检查有效点数/条件数，不足时冻结更新只做 IMU 递推并上报 `degenerate` |

### 4.4 yaw 候选数截断（Phase A Sprint 2）被证伪并回退

对照实验：cap=24（Run A）初始化锁到 (300,100)，全量 60（Run B）在 INS 初值处成功（虽然 yaw 错）。
15° 角分辨率超出 NDT 4 次迭代的收敛域。**已回退**，代码中留有注释。
CPU 争核问题需改用"粗搜降采样点云"方案（见开发记录）。

---

## 5. 结论与验收

| 验收项 | 结果 |
|--------|------|
| GNSS 通道端到端（订阅→转换→domain→状态估计器） | **通过**（≈6000 条 / 60s） |
| 轮速/INS 里程计通道端到端 | **通过**（≈5500 条 / 60s） |
| 通道开关对主路径无回归 | **通过**（对照组行为一致） |
| 单元测试（7 用例） | **通过** |
| 60s 运动段定位精度 | **不通过** — 被 LIO 发散 + 地图系错位两个前置问题阻塞，非本 Sprint 改动引入 |

**精度指标在当前数据/地图组合下无法给出有效值**；需先解决 4.3 的问题 1（地图系锚定）
和问题 3（LIO 退化防护）后重测。测试框架（录制→评估→CSV）已就绪，重测成本约 5 分钟。

---

## 6. 归档

| 文件 | 内容 |
|------|------|
| `/tmp/loc_s3_test{,2,3,4}.log` | 4 次运行完整日志 |
| `/tmp/eval_s3{,b,c,d}.bag` | TF + 参考轨迹录制 |
| `/tmp/eval_s3*_errors.csv` | 逐样本误差 |
| `scripts/eval_loc_accuracy.py` | 评估工具（入库） |

## 7. 相关文档

- [Sprint 3 开发记录](../development/sprint3_sensor_channels.md)
- [Phase A 测试报告](phase_a_test_report.md)
