#include "sensor_fusion/ekf_fusion.h"

#include <iostream>
#include <cmath>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Cholesky>
#include <Eigen/LU>

EKFFusion::EKFFusion()
    : x_(Eigen::VectorXd::Zero(STATE_SIZE))
    , P_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
    , Q_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
    , R_(Eigen::MatrixXd::Identity(6, 6))
    , F_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
    , B_(Eigen::MatrixXd::Zero(STATE_SIZE, 0))
    , H_(Eigen::MatrixXd::Zero(6, STATE_SIZE))
    , origin_latitude_(0.0)
    , origin_longitude_(0.0)
    , initialized_(false)
    , rtk_weight_(1.0)
    , imu_weight_(1.0)
    , encoder_weight_(1.0)
    , alpha_(0.1)
    , wheel_base_(0.6)
    , half_wheel_base_(0.3)
    , min_satellites_(0)
{
    last_update_time_ = ros::Time(0);

    setProcessNoise(0.1, 0.5, 1.0);

    setMeasurementNoise(0.05, 0.01, 0.01, 0.1, 0.02);
}

EKFFusion::~EKFFusion() {
}

void EKFFusion::init() {
    x_.setZero();

    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    initialized_ = true;
}

void EKFFusion::setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    Q_(INDEX_X, INDEX_X) = position_noise * position_noise;
    Q_(INDEX_Y, INDEX_Y) = position_noise * position_noise;

    Q_(INDEX_THETA, INDEX_THETA) = 0.01;

    Q_(INDEX_VX, INDEX_VX) = velocity_noise * velocity_noise;
    Q_(INDEX_VY, INDEX_VY) = velocity_noise * velocity_noise;
    Q_(INDEX_VTHETA, INDEX_VTHETA) = 0.01;

    Q_(INDEX_AX, INDEX_AX) = acceleration_noise * acceleration_noise;
    Q_(INDEX_AY, INDEX_AY) = acceleration_noise * acceleration_noise;
}

void EKFFusion::setMeasurementNoise(double rtk_position_noise, double rtk_orientation_noise,
                                   double imu_angular_velocity_noise, double imu_linear_acceleration_noise,
                                   double encoder_velocity_noise) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    R_(0, 0) = rtk_position_noise * rtk_position_noise;
    R_(1, 1) = rtk_position_noise * rtk_position_noise;

    R_(2, 2) = rtk_orientation_noise * rtk_orientation_noise;

    R_(3, 3) = imu_angular_velocity_noise * imu_angular_velocity_noise;

    R_(4, 4) = imu_linear_acceleration_noise * imu_linear_acceleration_noise;
    R_(5, 5) = imu_linear_acceleration_noise * imu_linear_acceleration_noise;

}

void EKFFusion::setOrigin(double latitude, double longitude) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    origin_latitude_ = latitude;

    origin_longitude_ = longitude;

    geo_converter_.Reset(latitude, longitude, 0);
}

void EKFFusion::setAntennaOffset(double x, double y, double theta) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    antenna_offset_ << x, y, theta;
}

void EKFFusion::setRTKBaseline(double distance, double tolerance) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rtk_baseline_distance_ = distance;
    rtk_baseline_tolerance_ = tolerance;
}

void EKFFusion::setIMUCalibration(const Eigen::Vector3d& lever_arm, const IMUData& bias) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_lever_arm_ = lever_arm;
    imu_bias_ = bias;
}

void EKFFusion::setWheelBase(double wheel_base) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    wheel_base_ = wheel_base;
    half_wheel_base_ = wheel_base_ * 0.5;
}

void EKFFusion::setMinSatellites(int min_satellites) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    min_satellites_ = min_satellites;
}

void EKFFusion::predict(const IMUData& imu) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!initialized_ || imu.timestamp < last_update_time_) {
        return;
    }

    double dt = (imu.timestamp - last_update_time_).toSec();
    if (dt <= 0) {
        return;
    }

    calculateStateTransitionMatrix(dt);

    double corrected_ax = imu.linear_acceleration_x - imu_bias_.linear_acceleration_x;
    double corrected_ay = imu.linear_acceleration_y - imu_bias_.linear_acceleration_y;
    double corrected_gz = imu.angular_velocity_z - imu_bias_.angular_velocity_z;

    double omega = x_(INDEX_VTHETA);
    double lx = imu_lever_arm_(0);
    double ly = imu_lever_arm_(1);
    corrected_ax -= -omega * omega * lx;
    corrected_ay -= -omega * omega * ly;

    Eigen::VectorXd u(2);
    u(0) = corrected_ax;
    u(1) = corrected_gz;

    double x = x_(INDEX_X);
    double y = x_(INDEX_Y);
    double theta = x_(INDEX_THETA);
    double vx = x_(INDEX_VX);
    double vy = x_(INDEX_VY);
    double vtheta = x_(INDEX_VTHETA);
    double ax = x_(INDEX_AX);
    double ay = x_(INDEX_AY);

    double dt2 = dt * dt / 2.0;
    x_(INDEX_X) = x + vx * dt + ax * dt2;
    x_(INDEX_Y) = y + vy * dt + ay * dt2;
    x_(INDEX_THETA) = normalizeAngle(theta + vtheta * dt);
    x_(INDEX_VX) = vx + ax * dt;
    x_(INDEX_VY) = vy + ay * dt;
    x_(INDEX_VTHETA) = u(1);

    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    x_(INDEX_AX) = u(0) * cos_theta;
    x_(INDEX_AY) = u(0) * sin_theta;

    P_ = F_ * P_ * F_.transpose() + Q_;

    last_update_time_ = imu.timestamp;
}

void EKFFusion::update(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!initialized_) {
        initializeFromRTKUnlocked(front_rtk, rear_rtk);
        return;
    }

    adaptWeights(front_rtk, rear_rtk, imu, encoder);

    Eigen::VectorXd z;
    Eigen::MatrixXd H, R;
    buildMeasurement(front_rtk, rear_rtk, imu, encoder, z, H, R);

    if (z.size() == 0) {
        return;
    }

    Eigen::VectorXd z_pred = H * x_;
    Eigen::VectorXd y = z - z_pred;

    Eigen::MatrixXd S = H * P_ * H.transpose() + R;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() == Eigen::Success) {
        Eigen::MatrixXd K = P_ * H.transpose() * ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * P_;
    } else {
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE) - K * H) * P_;
    }

    updateQ(y, S);
    updateR(y, H);

    x_(INDEX_THETA) = normalizeAngle(x_(INDEX_THETA));
}

FusionResult EKFFusion::getFusionResult() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    FusionResult result;

    result.valid = false;

    if (!initialized_) {
        return result;
    }

    result.timestamp = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;

    result.x = x_(INDEX_X);
    result.y = x_(INDEX_Y);
    result.theta = x_(INDEX_THETA);
    result.vx = x_(INDEX_VX);
    result.vy = x_(INDEX_VY);
    result.vtheta = x_(INDEX_VTHETA);
    result.ax = x_(INDEX_AX);
    result.ay = x_(INDEX_AY);

    result.valid = true;

    return result;
}

bool EKFFusion::initializeFromRTK(const RTKData& front_rtk, const RTKData& rear_rtk) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return initializeFromRTKUnlocked(front_rtk, rear_rtk);
}

bool EKFFusion::isInitialized() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return initialized_;
}

bool EKFFusion::initializeFromRTKUnlocked(const RTKData& front_rtk, const RTKData& rear_rtk) {
    if (initialized_) {
        return true;
    }
    if (!front_rtk.raw_valid || !rear_rtk.raw_valid) {
        return false;
    }

    double front_x = 0.0, front_y = 0.0;
    double rear_x = 0.0, rear_y = 0.0;
    WGS84ToLocal(front_rtk.latitude, front_rtk.longitude, front_x, front_y);
    WGS84ToLocal(rear_rtk.latitude, rear_rtk.longitude, rear_x, rear_y);

    x_(INDEX_X) = front_x;
    x_(INDEX_Y) = front_y;
    x_(INDEX_THETA) = atan2(front_y - rear_y, front_x - rear_x);
    x_(INDEX_VX) = 0.0;
    x_(INDEX_VY) = 0.0;
    x_(INDEX_VTHETA) = 0.0;
    x_(INDEX_AX) = 0.0;
    x_(INDEX_AY) = 0.0;
    last_update_time_ = !front_rtk.timestamp.isZero() ? front_rtk.timestamp : rear_rtk.timestamp;
    initialized_ = true;
    latest_result_.valid = false;
    return true;
}

void EKFFusion::reset() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    x_.setZero();

    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    last_update_time_ = ros::Time(0);

    initialized_ = false;
}

void EKFFusion::zeroVelocityUpdate() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!initialized_) return;

    double speed = sqrt(x_(INDEX_VX) * x_(INDEX_VX) + x_(INDEX_VY) * x_(INDEX_VY));
    if (speed > 0.1) return;

    x_(INDEX_VX) = 0.0;
    x_(INDEX_VY) = 0.0;

    const double small_var = 1e-4;
    for (int i = 0; i < STATE_SIZE; ++i) {
        P_(INDEX_VX, i) = 0.0;
        P_(INDEX_VY, i) = 0.0;
        P_(i, INDEX_VX) = 0.0;
        P_(i, INDEX_VY) = 0.0;
    }
    P_(INDEX_VX, INDEX_VX) = small_var;
    P_(INDEX_VY, INDEX_VY) = small_var;
}

void EKFFusion::applyVisionPoseMeasurement(const VisionPoseMeasurement& measurement) {
    if (!measurement.valid) {
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!initialized_) {
        return;
    }

    Eigen::Matrix<double, 3, STATE_SIZE> H = Eigen::Matrix<double, 3, STATE_SIZE>::Zero();
    H(0, INDEX_X) = 1.0;
    H(1, INDEX_Y) = 1.0;
    H(2, INDEX_THETA) = 1.0;

    Eigen::Vector3d z;
    z << measurement.x, measurement.y, measurement.theta;

    Eigen::Vector3d z_pred;
    z_pred << x_(INDEX_X), x_(INDEX_Y), x_(INDEX_THETA);

    Eigen::Vector3d innovation = z - z_pred;
    innovation(2) = normalizeAngle(innovation(2));

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    double pos_var_lat = std::max(measurement.position_variance, 1e-6);
    double pos_var_along = pos_var_lat;
    if (std::isfinite(measurement.position_variance_along) && measurement.position_variance_along > 0.0) {
        pos_var_along = std::max(measurement.position_variance_along, pos_var_lat);
    }
    double heading_var = std::max(measurement.heading_variance, 1e-6);

    if (pos_var_along != pos_var_lat) {
        double theta = x_(INDEX_THETA);
        double c = std::cos(theta);
        double s = std::sin(theta);
        Eigen::Matrix2d R_heading;
        R_heading << c, -s,
                     s,  c;
        Eigen::Matrix2d D = Eigen::Matrix2d::Zero();
        D(0, 0) = pos_var_along;  // along-track
        D(1, 1) = pos_var_lat;    // lateral
        Eigen::Matrix2d R_pos = R_heading * D * R_heading.transpose();
        R.block<2, 2>(0, 0) = R_pos;
    } else {
        R(0, 0) = pos_var_lat;
        R(1, 1) = pos_var_lat;
    }
    R(2, 2) = heading_var;

    Eigen::Matrix<double, STATE_SIZE, 3> Ht = H.transpose();
    Eigen::Matrix3d S = H * P_ * Ht + R;
    Eigen::Matrix<double, STATE_SIZE, 3> K = P_ * Ht * S.inverse();

    x_ = x_ + K * innovation;
    x_(INDEX_THETA) = normalizeAngle(x_(INDEX_THETA));

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    P_ = (I - K * H) * P_;
    P_ = 0.5 * (P_ + P_.transpose());
}

void EKFFusion::calculateStateTransitionMatrix(double dt) {
    F_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    double theta = x_(INDEX_THETA);
    double vx = x_(INDEX_VX);
    double vy = x_(INDEX_VY);

    static std::unordered_map<double, std::pair<double, double>> trig_cache;
    static const double CACHE_PRECISION = 0.001;

    double theta_key = std::round(theta / CACHE_PRECISION) * CACHE_PRECISION;
    auto it = trig_cache.find(theta_key);

    double cos_theta, sin_theta;
    if (it != trig_cache.end()) {
        cos_theta = it->second.first;
        sin_theta = it->second.second;
    } else {
        cos_theta = cos(theta);
        sin_theta = sin(theta);
        trig_cache[theta_key] = {cos_theta, sin_theta};

        if (trig_cache.size() > 1000) {
            trig_cache.clear();
        }
    }

    double dt_cos = dt * cos_theta;
    double dt_sin = dt * sin_theta;

    F_(INDEX_X, INDEX_VX) = dt_cos;
    F_(INDEX_X, INDEX_VY) = -dt_sin;
    F_(INDEX_X, INDEX_THETA) = (-vx * sin_theta - vy * cos_theta) * dt;
    F_(INDEX_Y, INDEX_VX) = dt_sin;
    F_(INDEX_Y, INDEX_VY) = dt_cos;
    F_(INDEX_Y, INDEX_THETA) = (vx * cos_theta - vy * sin_theta) * dt;

    F_(INDEX_THETA, INDEX_VTHETA) = dt;

    F_(INDEX_VX, INDEX_AX) = dt;
    F_(INDEX_VY, INDEX_AY) = dt;
}

void EKFFusion::calculateRTKHMatrix() {
    H_ = Eigen::MatrixXd::Zero(3, STATE_SIZE);

    double theta = x_(INDEX_THETA);
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    H_(0, INDEX_X) = 1.0;
    H_(0, INDEX_THETA) = -antenna_offset_(0) * sin_theta - antenna_offset_(1) * cos_theta;

    H_(1, INDEX_Y) = 1.0;
    H_(1, INDEX_THETA) = antenna_offset_(0) * cos_theta - antenna_offset_(1) * sin_theta;

    H_(2, INDEX_THETA) = 1.0;
}

void EKFFusion::calculateIMUAndEncoderHMatrix() {
    H_ = Eigen::MatrixXd::Zero(5, STATE_SIZE);

    H_(0, INDEX_VTHETA) = 1.0;
    H_(1, INDEX_AX) = 1.0;
    H_(2, INDEX_AY) = 1.0;

    H_(3, INDEX_VX) = 1.0;
    H_(3, INDEX_VTHETA) = -half_wheel_base_;
    H_(4, INDEX_VX) = 1.0;
    H_(4, INDEX_VTHETA) = half_wheel_base_;
}

void EKFFusion::WGS84ToLocal(double lat, double lon, double& x, double& y) {
    double h;
    geo_converter_.Forward(lat, lon, 0, x, y, h);
}

void EKFFusion::localToWGS84(double x, double y, double& lat, double& lon) {
    double h;
    geo_converter_.Reverse(x, y, 0, lat, lon, h);
}

double EKFFusion::normalizeAngle(double angle) {
    angle = fmod(angle, 2.0 * M_PI);
    if (angle > M_PI) angle -= 2.0 * M_PI;
    if (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double EKFFusion::calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

void EKFFusion::updateQ(const Eigen::VectorXd& y, const Eigen::MatrixXd& S) {
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXd P_new = (I - K * H_) * P_;
    Q_ = (1 - alpha_) * Q_ + alpha_ * (K * y * y.transpose() * K.transpose() + P_new - F_ * P_ * F_.transpose());

    for (int i = 0; i < STATE_SIZE; ++i) {
        for (int j = 0; j < STATE_SIZE; ++j) {
            if (std::isnan(Q_(i, j)) || std::isinf(Q_(i, j))) {
                Q_(i, j) = 0.0;
            }
        }
    }
}

void EKFFusion::setInitialTime(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_update_time_.isZero()) {
        last_update_time_ = time;
    }
}

void EKFFusion::updateR(const Eigen::VectorXd& y, const Eigen::MatrixXd& H) {
    R_ = (1 - alpha_) * R_ + alpha_ * (y * y.transpose() - H * P_ * H.transpose());

    for (int i = 0; i < R_.rows(); ++i) {
        for (int j = 0; j < R_.cols(); ++j) {
            if (std::isnan(R_(i, j)) || std::isinf(R_(i, j))) {
                R_(i, j) = 0.0;
            }
        }
    }
}

void EKFFusion::adaptWeights(const RTKData& front_rtk, const RTKData& rear_rtk,
                             const IMUData& imu, const EncoderData& encoder) {
    bool front_fix = front_rtk.valid && front_rtk.fix_mode == 4;
    bool rear_fix = rear_rtk.valid && rear_rtk.fix_mode == 4;

    if (front_fix && rear_fix) {
        double satellite_factor = 1.0;
        if (min_satellites_ > 0) {
            int front_delta = front_rtk.satellite_count - min_satellites_;
            int rear_delta = rear_rtk.satellite_count - min_satellites_;
            int delta = std::min(front_delta, rear_delta);

            if (delta <= 0) {
                satellite_factor = 0.2;
            } else if (delta <= 4) {
                satellite_factor = 0.6;
            } else if (delta <= 8) {
                satellite_factor = 0.8;
            } else if (delta <= 10) {
                satellite_factor = 0.9;
            } else {
                satellite_factor = 1.0;
            }
        }

        rtk_weight_ = std::max(0.05, std::min(1.0, satellite_factor));
    } else {
        rtk_weight_ = 0.0;
    }

    if (imu.valid) {
        imu_weight_ = 1.0;
    } else {
        imu_weight_ = 0.1;
    }

    if (encoder.valid) {
        encoder_weight_ = 1.0;
    } else {
        encoder_weight_ = 0.1;
    }
}

void EKFFusion::buildMeasurement(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder,
                               Eigen::VectorXd& z, Eigen::MatrixXd& H, Eigen::MatrixXd& R) {
    std::vector<double> z_vec;
    H.resize(0, STATE_SIZE);
    R.resize(0, 0);

    addRTKMeasurement(front_rtk, rear_rtk, z_vec, H, R);
    addIMUAndEncoderMeasurement(imu, encoder, z_vec, H, R);

    if (!z_vec.empty()) {
        z = Eigen::Map<Eigen::VectorXd>(z_vec.data(), z_vec.size());
    }
}

void EKFFusion::addRTKMeasurement(const RTKData& front_rtk, const RTKData& rear_rtk,
                                std::vector<double>& z_vec, Eigen::MatrixXd& H_dynamic, Eigen::MatrixXd& R_dynamic) {
    if (!front_rtk.valid || !rear_rtk.valid) return;
    if (!std::isfinite(rtk_weight_) || rtk_weight_ <= 1e-6) return;

    double front_x, front_y, rear_x, rear_y;
    WGS84ToLocal(front_rtk.latitude, front_rtk.longitude, front_x, front_y);
    WGS84ToLocal(rear_rtk.latitude, rear_rtk.longitude, rear_x, rear_y);

    double measured_distance = calculateDistance(front_x, front_y, rear_x, rear_y);
    if (std::abs(measured_distance - rtk_baseline_distance_) > rtk_baseline_tolerance_) {
        std::cerr << "RTK baseline outlier detected! Measured: " << measured_distance
                  << ", Expected: " << rtk_baseline_distance_ << std::endl;
        return;
    }

    size_t prev_rows = H_dynamic.rows();
    H_dynamic.conservativeResize(prev_rows + 3, STATE_SIZE);
    calculateRTKHMatrix();
    H_dynamic.bottomRows(3) = H_;

    z_vec.push_back((front_x + rear_x) / 2.0);
    z_vec.push_back((front_y + rear_y) / 2.0);
    z_vec.push_back(atan2(front_y - rear_y, front_x - rear_x));

    R_dynamic.conservativeResize(prev_rows + 3, prev_rows + 3);
    Eigen::Matrix3d R_rtk_block = R_.block<3,3>(0,0);
    R_rtk_block /= rtk_weight_;
    R_dynamic.bottomRightCorner(3, 3) = R_rtk_block;
}

void EKFFusion::addIMUAndEncoderMeasurement(const IMUData& imu, const EncoderData& encoder,
                                          std::vector<double>& z_vec, Eigen::MatrixXd& H_dynamic, Eigen::MatrixXd& R_dynamic) {
    if (!imu.valid || !encoder.valid) return;

    size_t prev_rows = H_dynamic.rows();
    H_dynamic.conservativeResize(prev_rows + 5, STATE_SIZE);
    calculateIMUAndEncoderHMatrix();
    H_dynamic.bottomRows(5) = H_;

    z_vec.push_back(imu.angular_velocity_z);
    z_vec.push_back(imu.linear_acceleration_x);
    z_vec.push_back(imu.linear_acceleration_y);
    z_vec.push_back(encoder.left_velocity);
    z_vec.push_back(encoder.right_velocity);

    R_dynamic.conservativeResize(prev_rows + 5, prev_rows + 5);
    R_dynamic.bottomRightCorner(5, 5).setIdentity();
    R_dynamic.block<3, 3>(prev_rows, prev_rows) = R_.block<3,3>(3,3) / imu_weight_;
    R_dynamic(prev_rows + 3, prev_rows + 3) = (0.02 * 0.02) / encoder_weight_;
    R_dynamic(prev_rows + 4, prev_rows + 4) = (0.02 * 0.02) / encoder_weight_;
}
