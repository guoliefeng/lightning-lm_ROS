# Lightning-LM 文档索引

本目录记录 `lightning-lm_ROS` 在 `feat/refactoring` 分支上的架构设计与项目现状。

## 文档列表

| 文档 | 说明 |
|------|------|
| [architecture/project_status.md](architecture/project_status.md) | **项目现状总览**（分层、运行时、缺口、演进路线、验证结果、思维导图） |
| [architecture/relocalization_main_path.md](architecture/relocalization_main_path.md) | 重定位主路径：旧路径 vs 新路径 |
| [architecture/backend_decoupling.md](architecture/backend_decoupling.md) | Legacy fusion 与 pose graph backend 解耦边界 |
| [development/phase_a_tf_authority.md](development/phase_a_tf_authority.md) | Phase A：TF 接 authority、PGO 去重 |
| [development/sprint3_sensor_channels.md](development/sprint3_sensor_channels.md) | Sprint 3：GNSS/轮速 domain 通道 |
| [development/sprint4_init_and_lio_guard.md](development/sprint4_init_and_lio_guard.md) | Sprint 4：INS 初值校验、LIO 退化防护、事件链修复 |
| [development/sprint5_map_frame_anchor.md](development/sprint5_map_frame_anchor.md) | Sprint 5：地图系锚定 `T_map_ins`、评估脚本、TF 时间戳统一 |
| [development/sprint6_ui_map_display.md](development/sprint6_ui_map_display.md) | Sprint 6：定位 UI 地图显示修复、A201 外部初值优化 |
| [testing/phase_a_test_report.md](testing/phase_a_test_report.md) | Phase A 测试报告 |
| [testing/sprint3_accuracy_test_report.md](testing/sprint3_accuracy_test_report.md) | Sprint 3 测试报告（通道 + 精度，暴露地图系错位） |
| [testing/sprint4_accuracy_test_report.md](testing/sprint4_accuracy_test_report.md) | Sprint 4 测试报告（精度指标与 Sprint 3 对比） |
| [testing/sprint5_accuracy_test_report.md](testing/sprint5_accuracy_test_report.md) | Sprint 5 测试报告（`T_ins_to_map` 标定后精度复测） |
| [testing/sprint6_ui_and_a201_test_report.md](testing/sprint6_ui_and_a201_test_report.md) | Sprint 6 测试报告（UI 地图显示 + A201 短测） |
| [testing/sprint6_a201_full_run_test_report.md](testing/sprint6_a201_full_run_test_report.md) | Sprint 6 全量测试（A201 131 bag × 17min） |

## 分支与仓库

- 仓库：`guoliefeng/lightning-lm_ROS`
- 重点分支：`feat/refactoring`
- 工作区：`lightning-lm_ROS1_ws`

## 快速入口

- 在线定位：`run_loc_online` + `config/*.yaml`
- 在线建图：`run_slam_online`
- 地图切块：`run_map_chunker`
- 重定位主路径单测：`test_relocalization_main_path`
- 定位精度评估：`scripts/eval_loc_accuracy.py`
