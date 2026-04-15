# sensor_fusion

`sensor_fusion` is a ROS1 package for planar localization and heading estimation using dual RTK, IMU and wheel encoder measurements. The package contains the fusion node, message definitions, parser/driver interfaces and the EKF/UKF implementations used in the study.

## Scope of the public release

This directory is a released code subset from the larger project. It is prepared for code inspection and method review only. The released subset keeps the fusion logic, message interfaces and launch/configuration structure required to understand the implementation. Deployment-specific information has been removed or replaced by public-release values, including:

- site coordinates and map registration parameters
- serial device names, network endpoints and device identifiers
- replay files, field data examples and project-specific launch chains
- downstream deployment protocols and private integration details

This package should not be interpreted as the complete software stack used in the full project.

The default parameter file [`config/params.yaml`](config/params.yaml) is a public template. It preserves parameter names and representative value ranges, but device paths, network endpoints and similar deployment settings are sanitized public-release values rather than the original experimental configuration.

## Package contents

- [`src/fusion_node.cpp`](src/fusion_node.cpp): main ROS node, parameter loading, sensor synchronization and output publication
- [`src/ukf_fusion.cpp`](src/ukf_fusion.cpp): UKF implementation
- [`src/ekf_fusion.cpp`](src/ekf_fusion.cpp): EKF implementation
- [`src/rtk_parser.cpp`](src/rtk_parser.cpp): serial RTK parser
- [`src/encoder_handler.cpp`](src/encoder_handler.cpp): WebSocket encoder client
- [`src/network_sender.cpp`](src/network_sender.cpp): optional generic HTTP sender
- [`msg/Rtk.msg`](msg/Rtk.msg), [`msg/Encoder.msg`](msg/Encoder.msg), [`msg/VisionMeasurement.msg`](msg/VisionMeasurement.msg): message interfaces

## Build

Target environment for the released package: ROS1 Noetic on Ubuntu 20.04.

Typical dependencies:

- `roscpp`
- `sensor_msgs`
- `geometry_msgs`
- `nav_msgs`
- `tf2`, `tf2_ros`
- `message_filters`
- `yaml-cpp`
- `jsoncpp`
- `Eigen3`
- `GeographicLib`
- `websocketpp`
- `libcurl`

Build with catkin:

```bash
catkin_make --pkg sensor_fusion
source devel/setup.bash
```

## Run

The public launch entry is [`launch/fusion.launch`](launch/fusion.launch):

```bash
roslaunch sensor_fusion fusion.launch
```

By default, the sanitized configuration disables hardware bring-up and HTTP output:

- `fusion.use_hardware_sources: false`
- `fusion.enable_network_sender: false`

To run against real devices, the user must provide their own reviewed configuration by editing [`config/params.yaml`](config/params.yaml) or overriding `param_file`.

When HTTP output is enabled, the sender posts a generic JSON document containing timestamp, pose, twist and acceleration fields. Any deployment-specific downstream protocol should be implemented outside this public package.

## Main topics

Inputs:

- front RTK: `/fusion_node/front_rtk`
- rear RTK: `/fusion_node/rear_rtk`
- IMU: configurable, default `/imu/data`
- encoder: `encoder`
- optional vision constraint: configurable, default `/sensor_fusion/vision_measurement`

Outputs:

- `/sensor_fusion/odom_raw`
- `/sensor_fusion/odom_centered`
- `/sensor_fusion/odom_ukf_raw`
- `/sensor_fusion/odom_ukf_centered_input`
- `/sensor_fusion/odom_predict`
- `/sensor_fusion/rtk_debug`
- optional `/sensor_fusion/vision_stats`

## Notes for reviewers

- This package is only one released component of the repository and only one released subset of the full project.
- The public parameter file is a template, not an exact experimental deployment snapshot.
- Device paths, network endpoints and deployment glue code are intentionally replaced by sanitized public-release values.
- The fusion implementation itself is preserved for algorithmic inspection.
