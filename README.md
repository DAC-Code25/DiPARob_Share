# Partial Public Release

[中文说明](README.zh-CN.md)

This repository contains a partial public release associated with the paper *A narrow inter-row mobile platform with robust multi-sensor fusion navigation for greenhouse tomato phenotyping*. It is intended for academic inspection of selected methods, released code and released experimental materials.

## Scope

This repository currently releases only the following materials:

- Selected code related to the paper
- Selected raw experimental data, statistical results and figures
- Auxiliary scripts for error calculation and figure generation
- Selected experimental videos

The following materials are outside the current public release:

- Complete engineering code and the full deployment pipeline
- Full raw experimental datasets
- Complete device configuration, site registration information and private interfaces
- All experimental videos and internal project documents

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

The code in `code/sensor_fusion/` is a released subset rather than the complete project codebase. Deployment-specific modules, private integration logic and unreleased engineering components are not included.

## Released Data

### Experiment I

- Raw workbook: `data/experiment_01/raw/Test_Log（1.22）.xlsx`
- Point-wise error table: `data/experiment_01/processed/calculated_point_errors.xlsx`
- Summary statistics: `data/experiment_01/processed/error_statistics.xlsx`
- Figures: `data/experiment_01/figures/Figure_generated.png`, `data/experiment_01/figures/Figure_generated.tif`

The lateral and heading errors in Experiment I are calculated from the raw workbook by `scripts/calculate_error.py`, and the statistical figure is generated from the statistical results by `scripts/speed_figures.py`.

### Experiment II

- Sensor bag data were collected for Experiment II but are not included in this repository because the original rosbag files are too large for the current public release.
- When necessary for academic verification or method comparison, the Experiment II bag data may be provided separately on request.
- Minimal metadata: `data/experiment_02/metadata/`
- Partial videos: `media/videos/`

## Released Videos

- `media/videos/Test_Video_1.mp4`: camera-view footage captured by the data acquisition module.
- `media/videos/Test_Video_2.mp4`: test video showing autonomous navigation during switching between adjacent tomato rows.

## Notes

- The released materials in this repository are intended for paper review, method understanding and result explanation, and should not be regarded as a complete deployable project.
- The original Experiment II sensor bag dataset is not included because of file size limitations. It may be provided separately through formal communication channels when academic verification or method reproduction is required.
