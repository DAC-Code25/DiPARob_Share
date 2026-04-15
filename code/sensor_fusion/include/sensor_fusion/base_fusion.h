#ifndef SENSOR_FUSION_BASE_FUSION_H
#define SENSOR_FUSION_BASE_FUSION_H

#include <vector>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <ros/time.h>

#include <Eigen/Dense>

#include "sensor_fusion/rtk_parser.h"
#include "sensor_fusion/encoder_handler.h"

struct RTKData {
    ros::Time timestamp;
    double latitude;
    double longitude;
    int fix_mode;
    int satellite_count;
    bool raw_valid;
    bool valid;
};

struct EncoderData {
    ros::Time timestamp;
    double left_encoder;
    double right_encoder;
    double left_velocity;
    double right_velocity;
    bool valid;
};

struct IMUData {
    ros::Time timestamp;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
    bool valid;
};

struct FusionResult {
    double timestamp;
    double x;
    double y;
    double theta;
    double vx;
    double vy;
    double vtheta;
    double ax;
    double ay;
    bool valid;
};

struct VisionPoseMeasurement {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double position_variance = 0.0;
    double position_variance_along = 0.0;
    double heading_variance = 0.0;
    ros::Time stamp;
    bool valid = false;
};

class BaseFusion {
public:
    BaseFusion() {}

    virtual ~BaseFusion() {}

    virtual void init() = 0;

    virtual void setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise) = 0;

    virtual void setMeasurementNoise(double rtk_position_noise, double rtk_orientation_noise,
                            double imu_angular_velocity_noise, double imu_linear_acceleration_noise,
                            double encoder_velocity_noise) = 0;

    virtual void setOrigin(double latitude, double longitude) = 0;

    virtual void setAntennaOffset(double x, double y, double theta) = 0;

    virtual void setRTKBaseline(double distance, double tolerance) = 0;

    virtual void setIMUCalibration(const Eigen::Vector3d& lever_arm, const IMUData& bias) = 0;

    virtual void setWheelBase(double wheel_base) = 0;

    virtual void setMinSatellites(int min_satellites) = 0;

    virtual void predict(const IMUData& imu) = 0;

    virtual void update(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder) = 0;

    virtual void applyVisionPoseMeasurement(const VisionPoseMeasurement& measurement) {}

    virtual FusionResult getFusionResult() = 0;

    virtual bool initializeFromRTK(const RTKData& front_rtk, const RTKData& rear_rtk) = 0;

    virtual bool isInitialized() const = 0;

    virtual void reset() = 0;

    virtual void setInitialTime(const ros::Time& time) = 0;
};

#endif
