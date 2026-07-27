# Sprint 7：A201 定位精度优化与 evo 评估

> 日期：2026-07-02  
> 分支：`feat/refactoring`  
> 数据：`yangpu/A201`（131 bag，总时长约 1030 s）  
> 配置：`config/yangpu_a201.yaml`

---

## 1. 背景与问题

Sprint 6 全量 A201 测试（17 min @ 1×）暴露：

| 问题 | 现象 |
|------|------|
| LIO 长时发散 | ~2.5 min 后速度飙至 767 m/s，ivox 栅格 76 万+ |
| NDT 失效 | `lidar_loc` 分值持续为 0 |
| 精度崩溃 | 对 INS 平面 RMSE **732 m**（`--align`） |
| 初值脚本错误 | `run_a201_full_test.sh` 中 `init_qw=-0.41581`（非单位四元数；正确值 **0.923065**） |

Sprint 4 的 `delta_trans` 单帧门限无法拦截“速度爆炸但单帧位移小”的发散；回滚 `pred_state` 也无法阻止 odom 积分在帧间累积错误位移。

---

## 2. 优化项

### 2.1 初值修正

A201 首帧 INS（`2026-06-09-14-07-02_0.bag`）：

```
pos: (308.399, -150.603, -0.104)
quat: (-0.001009, 0.002793, 0.384632, 0.923065)
```

所有评测脚本统一使用上述 `qw=0.923065`。

### 2.2 LIO 退化防护增强（`laser_mapping.cc`）

| 机制 | 说明 |
|------|------|
| 退化帧跳过 KF | `degenerate_frame==true` 时不调用 `MakeKF()`，避免坏帧向 ivox 增量建图 |
| ivox LRU 容量 | `ivox_capacity` 默认对齐 `max_ivox_grids`（A201：**12000**） |
| ivox 溢出重置 | `ivox_overflow` 时 `ResetIvoxLocalMap()`，用当前 scan 重播种 |
| 速度门限收紧 | `max_lidar_velocity_mps: 8.0`（港内 IGv 典型 5~7 m/s） |
| 单帧步长收紧 | `max_lidar_frame_trans_m: 0.8`，`max_lidar_frame_rot_deg: 6.0` |
| odom 漂移检测 | 定位模式 `!is_in_slam_mode_` 下 `\|pos\| > max_odom_translation_m`（25 m） |
| **odom 硬复位** | odom 漂移时不只回滚一帧，而是 `ResetOdomState()` 将 pos/vel 置零并重建局部 ivox |

关键代码路径：

- `IVox::Clear()` — `src/core/ivox3d/ivox3d.h`
- `ResetOdomState()` / `ResetIvoxLocalMap()` — `src/core/lio/laser_mapping.cc`
- 配置项 — `config/yangpu_a201.yaml` → `fasterlio.*`

### 2.3 评测工具链

| 脚本 | 用途 |
|------|------|
| `scripts/run_a201_accuracy_eval.sh` | 常速播 bag → 录 `/tf`+`/localization/ins` → TUM → evo |
| `scripts/export_bag_to_tum.py` | bag 流式导出 TUM；处理多 bag 播放时 sim_time 回跳 |
| `scripts/eval_loc_accuracy.py` | 既有平面/航向指标（与 Sprint 5 一致） |

evo 用法（INS 为参考）：

```bash
evo_ape tum ref_ins.tum est.tum -a          # SE3 Umeyama 对齐后 ATE
evo_rpe tum ref_ins.tum est.tum -a --delta 1 --delta_unit m
```

### 2.4 配置变更摘要（`yangpu_a201.yaml`）

```yaml
fasterlio:
  max_lidar_frame_trans_m: 0.8
  max_lidar_frame_rot_deg: 6.0
  max_lidar_velocity_mps: 8.0
  max_ivox_grids: 12000
  ivox_capacity: 12000
  max_odom_translation_m: 25.0

system:
  with_ui: false   # 精度评测 headless
```

---

## 3. 已知限制与后续方向

1. **地图系与 INS 常值偏差**：A201 未启用 `map_frame`；20 bag 评测 Umeyama 对齐仍出现约 **(108, 152) m** 平移、**−13°** yaw，说明除 LIO 发散外仍有坐标系/标定误差，可参考 Sprint 5 为 A201 标定 `T_ins_to_map`。
2. **多 bag 时间轴**：`rosbag play` 多文件时 `/tf` 时间戳可能回跳；`export_bag_to_tum.py` 在 `stamp < last − 0.1 s` 时重置采样窗口。
3. **odom 硬复位与 TF 连续性**：复位将 `odom→base_link` 置零，全局位姿依赖 `map→odom`（NDT/PGO）补偿；需保证 `lidar_loc` 持续有效。
4. **RPE vs APE**：优化后 RPE（~5 m）明显优于 APE（~45 m），说明局部形状尚可、全局锚定/标定仍是主误差源。

---

## 4. 文件清单

| 文件 | 变更 |
|------|------|
| `src/core/ivox3d/ivox3d.h` | `Clear()` |
| `src/core/lio/laser_mapping.{h,cc}` | 退化逻辑、odom/ivox 复位 |
| `config/yangpu_a201.yaml` | LIO 门限、headless |
| `scripts/run_a201_full_test.sh` | 修正 `init_qw` |
| `scripts/run_a201_accuracy_eval.sh` | 新建 |
| `scripts/export_bag_to_tum.py` | 新建 |
