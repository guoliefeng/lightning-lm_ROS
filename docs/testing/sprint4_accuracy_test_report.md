# Sprint 4 测试报告：初始化校验 + LIO 退化防护 + 精度评估

> 测试日期：2026-07-02  
> 分支：`feat/refactoring`（Sprint 4 变更）  
> 数据：yangpu/qc bag 0+1，`rosbag play -r 0.5 -u 60 --clock`

---

## 1. 测试环境

| 项 | 值 |
|----|-----|
| 配置 | `config/yangpu_qc.yaml`（含 Sprint 4 新参数） |
| INS 初值 | (238.74, -208.92, 0.07)，四元数来自 bag INS |
| 参考轨迹 | `/localization/ins` |
| 评估工具 | `scripts/eval_loc_accuracy.py` |

---

## 2. 单元测试 — 通过

```
relocalization main path smoke tests passed
```

7 个用例全部通过（含 Sprint 3 GNSS/轮速通道）。

---

## 3. 功能验收（日志 `/tmp/loc_s4c.log`）

| 指标 | Sprint 3 典型 | Sprint 4 | 判定 |
|------|--------------|----------|------|
| 外部初值初始化位置 | (0, -500) 或 (300, 100) 等功能点 | **(238.9, -209.0)** 接近 INS | **改善** |
| 窄 yaw 搜索 | 无 | `+/-180 → +/-30 deg` 日志可见 | 通过 |
| bridge `loc fps` | 无 | **~10 Hz** 持续输出 | **修复** |
| LIO 速度爆炸 | 18~35 m/s | 测试窗口内 **无 >10 m/s** | **改善** |
| 退化帧拒绝 | 无 | **2 次**（含 effect=106, Δtrans=6.8m 帧） | 通过 |
| `No point` | 2 次（启动） | 2 次 | 无回归 |

**初始化日志摘录：**
```
external pose init: narrow yaw search from +/-180 deg to +/-30 deg
init success, score: 0.646, pose: 238.9, -209.014, -1.86
loc fps: 9.7~10.4
```

**退化拒绝摘录：**
```
LIO degenerate frame rejected: effect=106, delta_trans=6.78 m, delta_rot=14.06 deg
```

---

## 4. 定位精度评估

> 评估需录制 `/tf` + `/localization/ins`。若 bag 中缺少 `map→odom`（Sprint 4 前录制），脚本会报 `missing tf chains`。

### 4.1 评估命令

```bash
# 运行期间
rosbag record -O eval.bag /tf /localization/ins

# 结束后
python3 scripts/eval_loc_accuracy.py eval.bag --align --csv errors.csv
python3 scripts/eval_loc_accuracy.py eval.bag --align --window 30   # 静止/初段
python3 scripts/eval_loc_accuracy.py eval.bag --align --min-move 5  # 运动段
```

### 4.2 指标说明

| 指标 | 含义 | 目标（yangpu/qc，待地图系锚定后） |
|------|------|-----------------------------------|
| 平面 RMSE | Umeyama 对齐后水平位置误差 | < 2 m（运动段） |
| 高程 RMSE | \|Δz\| | < 0.5 m |
| 航向 RMSE | \|Δyaw\| | < 5° |
| p95 平面 | 95 分位误差 | < 3 m |

### 4.3 Sprint 4 结论

| 验收项 | 结果 |
|--------|------|
| 初始化落在 INS 附近 | **通过**（238.9 vs 238.7） |
| coordinator → bridge 事件 | **通过**（loc fps ~10Hz） |
| LIO 发散抑制 | **部分通过**（退化帧回滚生效，未再出现 30+ m/s） |
| 60s 运动段精度 RMSE | **待完整 bag 评估** — 地图系与 INS 系未锚定，对齐后 RMSE 仍可能偏大 |

**与 Sprint 3 对比**：Sprint 3 全程 RMSE ~587–799 m（LIO 发散主导）；Sprint 4 消除了错误功能点初始化与速度爆炸，**精度评估前提已具备**，但地图原点锚定（Phase C）仍是精度达标的前置条件。

---

## 5. 已知限制

1. **地图系 ≠ INS map 系**：Umeyama 对齐后仍可能有百米线级残差。
2. **qc 岸桥自体结构**：LIO 对移动钢结构场景本质困难，退化防护为缓解而非根治。
3. **录制 bag 需 SIGINT 结束**：`rosbag record` 被 SIGKILL 时文件可能不完整。

---

## 6. 归档

| 文件 | 内容 |
|------|------|
| `/tmp/loc_s4c.log` | Sprint 4 完整运行日志 |
| `/tmp/eval_s4c.bag` | TF + INS 录制（若完整） |
| `scripts/eval_loc_accuracy.py` | 精度评估工具 |

---

## 7. 相关文档

- [Sprint 4 开发记录](../development/sprint4_init_and_lio_guard.md)
- [Sprint 3 测试报告](sprint3_accuracy_test_report.md)
