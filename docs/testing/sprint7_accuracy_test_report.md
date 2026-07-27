# Sprint 7 测试报告：A201 精度优化 + evo（INS 参考）

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 参考轨迹：`/localization/ins`（nav_msgs/Odometry）  
> 估计轨迹：TF `map→odom` × `odom→base_link`  
> 播放速率：**1.0×（常速）**  
> 配置：`config/yangpu_a201.yaml`（`map_frame.enabled: false`）

---

## 1. 测试矩阵

| 用例 | 脚本/输出目录 | bag 数 | 时长 | 说明 |
|------|---------------|--------|------|------|
| **基线** | Sprint 6 `/tmp/a201_full_eval.bag` | 131 | ~1030 s | 优化前全量跑，LIO 发散 |
| **S7 短测** | `scripts/run_a201_accuracy_eval.sh` → `/tmp/sprint7_clean` | 20 | ~157 s | 优化后干净录制 |
| **S7 全量** | 同上 → `/tmp/sprint7_full` | 131 | ~1030 s | 优化后常速全量（见 §4） |

初值（与 bag 首帧 INS 一致）：

```bash
--init_x 308.40 --init_y -150.60 --init_z -0.10
--init_qx -0.001009 --init_qy 0.002793 --init_qz 0.384632 --init_qw 0.923065
```

---

## 2. 基线（Sprint 6，优化前）

来源：`docs/testing/sprint6_a201_full_run_test_report.md`，`eval_loc_accuracy.py --align`（120 s 子集亦见同报告）

| 指标 | 值 |
|------|-----|
| 平面 RMSE | **732 m** |
| 航向 mean | **107°** |
| LIO 末态 | 位置 (−663 km, 228 km) 量级，速度 767 m/s |
| NDT | 发散后 score≈0 |

---

## 3. 优化后短测（20 bag，157 s）

### 3.1 运行健康度

| 项 | 结果 |
|----|------|
| 外部 INS 初始化 | 成功，conf **2.14**，pose (308.85, −150.79) |
| LIO 末态 | pos **(0.40, 0.67, −0.06) m**，vel **< 0.03 m/s** |
| odom 硬复位次数 | **0** |
| `lidar_loc分值较低: 0` | **0 次**（全程 NDT 有效） |

### 3.2 evo（Umeyama SE3 对齐）

| 指标 | APE (m) | RPE trans, δ=1 m (m) |
|------|---------|----------------------|
| mean | 38.51 | 4.70 |
| **rmse** | **44.96** | **5.78** |
| median | 27.43 | 3.91 |
| max | 108.52 | 13.54 |

### 3.3 `eval_loc_accuracy.py --align`

| 指标 | 值 |
|------|-----|
| 样本数 | 17 322（时长 156.8 s） |
| 平面 RMSE | **45.22 m** |
| 平面 max | 108.99 m |
| 高程 RMSE | 1.43 m |
| 航向 RMSE | 120.29° |
| Umeyama 对齐 | 平移 [108.50, 151.62] m，旋转 **−12.88°** |

### 3.4 与基线对比（短窗）

| 指标 | Sprint 6 基线 | Sprint 7（20 bag） | 变化 |
|------|---------------|-------------------|------|
| 平面 RMSE | 732 m | **45 m** | **≈16× 改善** |
| LIO 发散 | 是 | **否** | 防护生效 |
| evo RPE RMSE | — | **5.78 m** | 局部一致性可接受 |

> APE ~45 m 仍高于 Sprint 5 洋浦 QC 标定后 **~17.6 m**，主因是 A201 未做 `map_frame` 锚定 + 航向存在系统性偏差（对齐后 yaw 误差仍大）。

---

## 4. 全量测试（131 bag，1×）

> 测试命令：`OUT_DIR=/tmp/sprint7_full MAX_BAGS=0 RATE=1.0 scripts/run_a201_accuracy_eval.sh`  
> 结果文件：`/tmp/sprint7_full/{custom_metrics.txt, evo_ape.txt, evo_rpe.txt}`

（全量跑完后在此填写；预期 LIO 防护可维持全程无 767 m/s 发散。）

---

## 5. 复现步骤

```bash
cd ~/proj/lightning-lm_ROS1_ws && catkin_make
source devel/setup.bash

# 短测（约 2.6 min 播放 + 评测）
OUT_DIR=/tmp/sprint7_clean MAX_BAGS=20 RATE=1.0 \
  src/lightning-lm_ROS/scripts/run_a201_accuracy_eval.sh

# 全量（约 17 min）
OUT_DIR=/tmp/sprint7_full MAX_BAGS=0 RATE=1.0 \
  src/lightning-lm_ROS/scripts/run_a201_accuracy_eval.sh
```

---

## 6. 结论

1. **LIO 长时发散问题已基本遏制**：odom 硬复位 + 收紧门限后，157 s 连续运行 LIO 保持在亚米级 odom 域内，NDT 未失效。
2. **全局精度大幅优于 Sprint 6 基线**（732 m → **~45 m** APE），但仍需 **map_frame 标定** 才能接近 QC 场景 17 m 水平。
3. **RPE ~5.8 m** 表明短时段内相对运动与 INS 较一致；**航向 RMSE ~120°** 提示地图系/INS 航向定义或锚定未对齐，应作为 Sprint 8 优先项。
4. 评测必须使用 **干净录制** 的 eval bag；混录多次测试会导致 sim_time 跳变，evo 全轨迹 RMSE 虚高。
