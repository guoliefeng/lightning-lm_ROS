# Sprint 6 全量测试报告：yangpu/A201（131 bag × ~17min）

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 数据：`/home/glf/dataDisk/hainan/yangpu/A201/`（131 个 bag，总时长 **1030s ≈ 17.2min**）  
> 配置：`config/yangpu_a201.yaml`（headless，`map_frame.enabled: false`）  
> 脚本：`scripts/run_a201_full_test.sh`  
> 日志：`/tmp/a201_full_loc.log`（547k 行）  
> 评估 bag：`/tmp/a201_full_eval.bag`（101MB）

---

## 1. 测试概要

| 项 | 结果 |
|----|------|
| bag 播放 | **131/131 完成**，`play_exit=0` |
| 进程稳定性 | **无 crash / segfault / ERROR** |
| 运行时长 | 15:23:54 → 15:41:18（约 **17min 24s**） |
| 外部 INS 初始化 | **1 次成功**，未回退功能点 |
| GNSS / Odom 通道 | ~103k / ~93.5k 条 |
| loc fps（采样） | avg **7.6 Hz**（LOG_EVERY_N 共 19 条，峰值 ~11 Hz） |

---

## 2. 发现的问题

### 2.1 【严重】LIO 在 ~2.5min 发散（修复前全量跑）

| 时间点 | 现象 |
|--------|------|
| 15:24:03 | 外部 INS 初始化成功 `(309.06, -150.68)`，conf **0.88** |
| 15:26:43 | LIO 位姿跳至 `(3788, 4763, 538)`，速度 **767 m/s** |
| 15:26:43 | ivox 栅格 **763,286**（正常 ~11k） |
| 15:26:43 | NDT 失败 `score: 0`，`lidar loc target is null` |
| 15:41:12 | 末态 LIO `(-663000, 228000, 9493)`，速度仍 **767 m/s** |

**根因**：单帧 `delta_trans` 仍 < 1.5m，Sprint 4 退化防护**未触发**；ivox 无界膨胀 + 速度爆炸未被检测。

**修复**（见开发报告）：
- `max_lidar_velocity_mps: 25`
- `max_ivox_grids: 30000`
- 超阈值回滚 ESKF 状态

**修复后 180s 复测**（2x 播放）：末态 LIO `(-0.37, 0.03, -0.07)`，速度 **~0.01 m/s**，ivox **12808** ✅

### 2.2 【中等】PGO LidarOdom 时间戳回退刷屏

- 全量日志 **10,409 条** `LidarOdom定位时间戳回退`
- 首条 `-1.78e9 s`（epoch/sim 混用）；其余多为 **~−0.12 s** 轻微乱序

**修复**：`pgo.cc` 对大跳变清空队列；对 |Δt|<0.25s 乱序帧**静默丢弃**，不再刷屏。

### 2.3 【中等】长时精度 vs INS 不可用

120s 评估子集（`a201_eval_120.bag`）：

| 指标 | `--align` |
|------|-----------|
| 平面 RMSE | **732 m** |
| 航向 mean | **107°** |

原因：LIO 发散后 `odom→base_link` 错误；A201 未启用 `map_frame` 锚定；评估混合了发散段。

### 2.4 【低】其他告警

| 告警 | 次数/说明 |
|------|-----------|
| `No point, skip` | 2（启动 IMU 未就绪） |
| `Failed to find match for field 'ring'` | meta_cloud 缺字段，可忽略 |
| `lidar_loc分值较低: 0` | 发散后 NDT 持续失败（6900+ 次） |
| `smoother motion is too large` | PGO 平滑在 loc 失败时频繁告警 |
| `XmlRpcClient Connection refused` | roscore 启动瞬间，非致命 |

---

## 3. 通过项

- [x] 131 bag 连续播放无中断
- [x] 地图加载 73 chunks
- [x] UI 地图通路（短测已验证 `UI map refresh: 15 chunks`）
- [x] 外部 INS 初值初始化（未误落功能点）
- [x] Sprint 4 粗搜保留逻辑在 A201 有效
- [x] 单元测试 8 cases 通过

---

## 4. 复现命令

```bash
# 全量（~17min）
scripts/run_a201_full_test.sh

# 180s 快速回归（修复后）
rosparam set use_sim_time true
./devel/lib/lightning/run_loc_online --config config/yangpu_a201.yaml \
  --init_x 308.40 --init_y -150.60 --init_z -0.10 \
  --init_qx -0.001009 --init_qy 0.002793 --init_qz 0.384632 --init_qw -0.41581 &
sleep 12
rosbag play --clock -r 2.0 /home/glf/dataDisk/hainan/yangpu/A201/*.bag -u 180
```

---

## 5. 后续建议

1. **重跑全量 17min**（应用 LIO 速度/ivox 防护后）确认无 767 m/s 发散
2. 为 A201 标定 `map_frame`（或启用 INS 锚定）以评估真实 RMSE
3. PGO `ProcessDR` 同样处理轻微乱序（当前仅 LO 已修）
4. LIO ivox **滑动窗口 / 容量上限**（根治地图膨胀，非仅拒绝帧）

---

## 6. 相关文档

- [Sprint 6 开发记录（UI + 全量问题修复）](../development/sprint6_ui_map_display.md)
- [Sprint 6 UI 短测报告](sprint6_ui_and_a201_test_report.md)
