#ifndef SENSOR_FUSION_EKF_FUSION_H
#define SENSOR_FUSION_EKF_FUSION_H

#include "sensor_fusion/base_fusion.h"

#include <vector>
#include <mutex>
#include <condition_variable>
#include <cmath>

#include <Eigen/Dense>

#include "sensor_fusion/rtk_parser.h"
#include "sensor_fusion/encoder_handler.h"
#include <GeographicLib/LocalCartesian.hpp>

#define STATE_SIZE 8

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_THETA 2
#define INDEX_VX 3
#define INDEX_VY 4
#define INDEX_VTHETA 5
#define INDEX_AX 6
#define INDEX_AY 7

class EKFFusion : public BaseFusion {
public:
    EKFFusion();

    ~EKFFusion();

    void init() override;

    void setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise) override;

    void setMeasurementNoise(double rtk_position_noise, double rtk_orientation_noise,
                            double imu_angular_velocity_noise, double imu_linear_acceleration_noise,
                            double encoder_velocity_noise) override;

    void setOrigin(double latitude, double longitude) override;

    void setAntennaOffset(double x, double y, double theta) override;

    void setRTKBaseline(double distance, double tolerance) override;

    void setIMUCalibration(const Eigen::Vector3d& lever_arm, const IMUData& bias) override;

    void setWheelBase(double wheel_base) override;

    void setMinSatellites(int min_satellites) override;

    void predict(const IMUData& imu) override;

    void update(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder) override;

    void applyVisionPoseMeasurement(const VisionPoseMeasurement& measurement) override;

    FusionResult getFusionResult() override;

    bool initializeFromRTK(const RTKData& front_rtk, const RTKData& rear_rtk) override;

    bool isInitialized() const override;

    void reset() override;

    void zeroVelocityUpdate();

    void setInitialTime(const ros::Time& time) override;

private:
    Eigen::VectorXd x_;

    Eigen::MatrixXd P_;

    Eigen::MatrixXd Q_;

    Eigen::MatrixXd R_;

    Eigen::MatrixXd F_;

    Eigen::MatrixXd B_;

    Eigen::MatrixXd H_;

    FusionResult latest_result_;

    mutable std::mutex data_mutex_;

    std::condition_variable data_cv_;

    double origin_latitude_;

    double origin_longitude_;

    GeographicLib::LocalCartesian geo_converter_;

    Eigen::Vector3d antenna_offset_;

    double rtk_baseline_distance_;
    double rtk_baseline_tolerance_;

    double wheel_base_;
    double half_wheel_base_;

    int min_satellites_;

    Eigen::Vector3d imu_lever_arm_;
    IMUData imu_bias_;

    ros::Time last_update_time_;

    bool initialized_;

    double rtk_weight_;
    double imu_weight_;
    double encoder_weight_;

    void calculateStateTransitionMatrix(double dt);

    void calculateRTKHMatrix();

    void calculateIMUAndEncoderHMatrix();

    void WGS84ToLocal(double lat, double lon, double& x, double& y);

    void localToWGS84(double x, double y, double& lat, double& lon);

    double normalizeAngle(double angle);

    double calculateDistance(double x1, double y1, double x2, double y2);

    void buildMeasurement(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder,
                          Eigen::VectorXd& z, Eigen::MatrixXd& H, Eigen::MatrixXd& R);
    void addRTKMeasurement(const RTKData& front_rtk, const RTKData& rear_rtk,
                           std::vector<double>& z_vec, Eigen::MatrixXd& H_dynamic, Eigen::MatrixXd& R_dynamic);
    void addIMUAndEncoderMeasurement(const IMUData& imu, const EncoderData& encoder,
                                     std::vector<double>& z_vec, Eigen::MatrixXd& H_dynamic, Eigen::MatrixXd& R_dynamic);

    double alpha_;
    void updateQ(const Eigen::VectorXd& y, const Eigen::MatrixXd& S);
    void updateR(const Eigen::VectorXd& y, const Eigen::MatrixXd& H);

    void adaptWeights(const RTKData& front_rtk, const RTKData& rear_rtk,
                      const IMUData& imu, const EncoderData& encoder);
    bool initializeFromRTKUnlocked(const RTKData& front_rtk, const RTKData& rear_rtk);
};

#endif
