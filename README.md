# Partial Public Release

本仓库对应论文 *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping* 的部分公开材料，仅用于论文相关方法、结果与公开代码的学术查阅。

This repository contains a partial public release associated with the paper *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping*. It is intended for academic inspection of selected methods, released code and released experimental materials.

## Scope

当前仓库仅公开以下内容：

- 论文相关的部分代码
- 部分实验原始数据、统计结果与图表
- 用于误差计算和绘图的辅助脚本
- 部分实验视频

以下内容不在当前公开范围内：

- 完整工程代码与全部部署链路
- 全量实验原始数据
- 完整设备配置、场地注册信息与私有接口
- 全部实验视频与项目内部文档

This repository is not the complete project release. Only selected code, selected datasets, selected figures, selected videos and auxiliary scripts required for academic inspection are included.

## Repository Layout

- `code/sensor_fusion/`
  Public ROS1 sensor-fusion package used for method inspection. This directory preserves the released fusion implementation, interfaces and configuration template only.
- `scripts/`
  Auxiliary scripts for error calculation and figure generation from the released spreadsheets.
- `data/experiment_01/raw/`
  Released raw workbook for Experiment I.
- `data/experiment_01/processed/`
  Processed error tables derived from the released workbook.
- `data/experiment_01/figures/`
  Released figure files generated from the processed statistics.
- `data/experiment_02/bags/`
  Documentation for Experiment II sensor bag data that were collected but are not stored in the repository because of file size.
- `data/experiment_02/metadata/`
  Minimal metadata accompanying the released Experiment II data.
- `media/videos/`
  Released video materials, including acquisition-module camera footage and autonomous inter-row switching tests.

## Released Code

公开代码位于 `code/sensor_fusion/`，仅代表论文方法中可公开的融合算法实现子集，不对应完整工程。

The code in `code/sensor_fusion/` is a released subset rather than the complete project codebase. Deployment-specific modules, private integration logic and unreleased engineering components are not included.

## Released Data

### Experiment I

- Raw workbook: `data/experiment_01/raw/Test_Log（1.22）.xlsx`
- Point-wise error table: `data/experiment_01/processed/calculated_point_errors.xlsx`
- Summary statistics: `data/experiment_01/processed/error_statistics.xlsx`
- Figures: `data/experiment_01/figures/Figure_generated.png`, `data/experiment_01/figures/Figure_generated.tif`

实验一中的横向误差与航向误差由 `scripts/calculate_error.py` 从原始工作簿计算得到，统计图由 `scripts/speed_figures.py` 根据统计结果生成。

### Experiment II

- Sensor bag data were collected for Experiment II but are not included in this repository because the original rosbag files are too large for the current public release.
- When necessary for academic verification or method comparison, the Experiment II bag data may be provided separately on request.
- Minimal metadata: `data/experiment_02/metadata/`
- Partial videos: `media/videos/`

## Released Videos

- `media/videos/Test_Video_1.mp4`: camera-view footage captured by the data acquisition module.
- `media/videos/Test_Video_2.mp4`: test video showing autonomous navigation during switching between adjacent tomato rows.

## Notes

- 仓库中的公开材料用于论文审阅、方法理解与结果说明，不应视为完整可部署工程。
- 实验二原始传感器 bag 数据集因文件体积限制未纳入仓库；如有学术核验或方法复现实验需要，可通过正式沟通渠道按需提供。
