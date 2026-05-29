# 部分公开材料

[English README](README.md)

本仓库对应论文 *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping* 的部分公开材料，仅用于论文相关方法、公开代码与公开实验材料的学术查阅。

## 公开范围

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

本仓库不是完整项目发布，仅包含学术查阅所需的部分代码、部分数据集、部分图表、部分视频与辅助脚本。

## 仓库结构

- `code/sensor_fusion/`
  用于方法查阅的公开 ROS1 传感器融合功能包。该目录仅保留公开的融合算法实现、接口与配置模板。
- `scripts/`
  用于根据公开表格进行误差计算和图表生成的辅助脚本。
- `data/experiment_01/raw/`
  实验一公开的原始工作簿。
- `data/experiment_01/processed/`
  由公开工作簿生成的误差处理表格。
- `data/experiment_01/figures/`
  根据统计结果生成的公开图表文件。
- `data/experiment_02/bags/`
  实验二传感器 bag 数据的说明文档；相关数据已采集，但因文件体积原因未存储在本仓库中。
- `data/experiment_02/metadata/`
  实验二公开数据附带的最小元数据。
- `media/videos/`
  公开的视频材料，包括采集模块相机画面和自主相邻垄间切换测试视频。

## 公开代码

`code/sensor_fusion/` 中的代码是公开子集，并非完整项目代码。与部署相关的模块、私有集成逻辑和未公开工程组件不包含在内。

## 公开数据

### 实验一

- 原始工作簿：`data/experiment_01/raw/Test_Log（1.22）.xlsx`
- 逐点误差表：`data/experiment_01/processed/calculated_point_errors.xlsx`
- 统计汇总表：`data/experiment_01/processed/error_statistics.xlsx`
- 图表：`data/experiment_01/figures/Figure_generated.png`、`data/experiment_01/figures/Figure_generated.tif`

实验一中的横向误差与航向误差由 `scripts/calculate_error.py` 从原始工作簿计算得到，统计图由 `scripts/speed_figures.py` 根据统计结果生成。

### 实验二

- 实验二传感器 bag 数据已经采集，但原始 rosbag 文件过大，因此未纳入当前公开仓库。
- 如学术核验或方法比较需要，实验二 bag 数据可通过正式沟通渠道另行提供。
- 最小元数据：`data/experiment_02/metadata/`
- 部分视频：`media/videos/`

## 公开视频

- `media/videos/Test_Video_1.mp4`：采集模块采集到的相机拍摄画面。
- `media/videos/Test_Video_2.mp4`：自主导航进行相邻番茄垄间切换时的测试视频。

## 说明

- 仓库中的公开材料用于论文审阅、方法理解与结果说明，不应视为完整可部署工程。
- 实验二原始传感器 bag 数据集因文件体积限制未纳入仓库；如有学术核验或方法复现实验需要，可通过正式沟通渠道按需提供。
