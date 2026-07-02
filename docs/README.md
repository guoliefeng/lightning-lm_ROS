# Lightning-LM 文档索引

本目录记录 `lightning-lm_ROS` 在 `feat/refactoring` 分支上的架构设计与项目现状。

## 文档列表

| 文档 | 说明 |
|------|------|
| [architecture/project_status.md](architecture/project_status.md) | **项目现状总览**（分层、运行时、缺口、演进路线、验证结果、思维导图） |
| [development/phase_a_tf_authority.md](development/phase_a_tf_authority.md) | **Phase A 开发记录**（TF 接 authority、PGO 去重、NDT 限流、验证步骤） |
| [testing/phase_a_test_report.md](testing/phase_a_test_report.md) | **Phase A 测试报告**（单测、yangpu 50s 集成、TF 采样结果） |
| [architecture/relocalization_main_path.md](architecture/relocalization_main_path.md) | 重定位主路径：旧路径 vs 新路径 |
| [architecture/backend_decoupling.md](architecture/backend_decoupling.md) | Legacy fusion 与 pose graph backend 解耦边界 |

## 分支与仓库

- 仓库：`guoliefeng/lightning-lm_ROS`
- 重点分支：`feat/refactoring`
- 工作区：`lightning-lm_ROS1_ws`

## 快速入口

- 在线定位：`run_loc_online` + `config/*.yaml`
- 在线建图：`run_slam_online`
- 地图切块：`run_map_chunker`
- 重定位主路径单测：`test_relocalization_main_path`
