# 数据说明 / Data Description

本目录包含与论文 *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping* 相关的部分公开数据、统计结果及辅助脚本。

This directory contains partially released data, statistical results, and auxiliary scripts associated with the paper *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping*.

## 1. 文件构成 / File Contents

### 1.1 误差计算与绘图脚本 / Error Calculation and Figure-Drawing Scripts

- `calculate_error.py`  
  用于实验一闭环测试中原始采集数据的误差计算。  
  Used for error calculation based on the raw data collected in Experiment I (closed-loop test).

- `speed_figures.py`  
  用于实验一闭环测试中速度误差统计图的绘制。  
  Used for plotting the speed-related error statistics for Experiment I (closed-loop test).

### 1.2 实验一闭环测试数据与统计结果 / Data and Statistical Results for Experiment I

- `Test_Log（1.22）.xlsx`  
  实验一闭环测试的原始采集数据表。  
  Raw data table collected in Experiment I (closed-loop test).

- `calculated_point_errors.xlsx`  
  基于原始数据表，并依据论文公式（14）与公式（15）计算得到的逐点误差结果表。  
  Point-wise error table calculated from the raw data according to Equations (14) and (15) in the paper.

- `error_statistics.xlsx`  
  基于逐点误差结果进一步整理得到的点位误差统计表与分速度误差统计表。  
  Statistical summary table derived from the point-wise error results, including point-based error statistics and speed-wise error statistics.

### 1.3 实验一闭环测试图表文件 / Figure Files for Experiment I

- `Figure_generated.png`  
  实验一闭环测试统计图的 PNG 格式版本。  
  PNG version of the statistical figure for Experiment I.

- `Figure_generated.tif`  
  实验一闭环测试统计图的 TIFF 格式版本。  
  TIFF version of the statistical figure for Experiment I.

### 1.4 实验二消融测试原始数据 / Raw Data for Experiment II (Ablation Test)

- `rtk_2026_01_21_0.2ms_01.bag`  
  实验二消融测试原始数据集 `B1`。  
  Raw dataset `B1` for Experiment II (ablation test).

- `rtk_2026_01_21_0.2ms_02.bag`  
  实验二消融测试原始数据集 `B2`。  
  Raw dataset `B2` for Experiment II (ablation test).

- `origin_location`  
  对应测试过程中 RTK 基站的部署位置记录。  
  Record of the RTK base-station deployment location during the experiment.

### 1.5 实验视频资料 / Experimental Video Materials

- `video1.mp4`  
  平台在温室环境中的部分真实测试视频。  
  Partial real-world test video of the platform in the greenhouse environment.

- `video2.mp4`  
  平台在温室环境中的部分真实测试视频。  
  Partial real-world test video of the platform in the greenhouse environment.

## 2. 计算说明 / Calculation Notes

实验一闭环测试中的误差计算依据论文中给出的相关公式进行处理，其中前端偏差与后端偏差用于计算横向误差与航向误差。对应结果文件如下：

The error calculation in Experiment I was conducted according to the relevant equations presented in the paper, where front-end deviation and rear-end deviation were used to compute the lateral error and heading error. The corresponding result files are listed below:

- 原始数据表 / Raw data table: `Test_Log（1.22）.xlsx`
- 逐点误差结果表 / Point-wise error table: `calculated_point_errors.xlsx`
- 统计汇总表 / Statistical summary table: `error_statistics.xlsx`

统计图文件 `Figure_generated.png` 与 `Figure_generated.tif` 均由相应绘图脚本基于统计结果生成。

The figure files `Figure_generated.png` and `Figure_generated.tif` were generated from the statistical results using the corresponding plotting script.

## 3. 公开范围说明 / Statement on Data Availability

受论文所属项目数据管理与保密要求限制，当前目录仅公开部分可用于说明研究过程与实验结果的数据、图表及辅助脚本。完整原始数据、全部算法代码及全部测试视频暂不对外公开。

Due to data management and confidentiality requirements of the project associated with the paper, only part of the data, figures, and auxiliary scripts that are necessary for illustrating the research process and experimental results are publicly released in this directory. The complete raw data, full algorithmic code, and all test videos are not publicly available at present.

如因学术研究需要申请获取更多相关数据，请通过正式邮件方式联系作者。

If additional data are required for academic research purposes, please contact the authors through formal email communication.

## 4. 真实性声明 / Statement of Authenticity

作者对本目录中已公开数据与说明内容的真实性负责。

The authors are responsible for the authenticity of the released data and the accompanying descriptions provided in this directory.
