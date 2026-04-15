#include "sensor_fusion/ukf_fusion.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/LU>

UKFFusion::UKFFusion()
    : x_(Eigen::VectorXd::Zero(STATE_SIZE))
    , P_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
    , Q_(Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
    , R_(Eigen::MatrixXd::Identity(6, 6))
    , X_sig_(Eigen::MatrixXd::Zero(STATE_SIZE, 2 * STATE_SIZE + 1))
    , origin_latitude_(0.0)
    , origin_longitude_(0.0)
    , initialized_(false)
    , cache_size_(100)
    , rtk_weight_(1.0)
    , imu_weight_(1.0)
    , encoder_weight_(1.0)
    , rtk_heading_weight_(1.0)
    , rear_offset_initialized_(false)
    , rtk_baseline_distance_(0.0)
    , rtk_baseline_tolerance_(0.1)
    , rtk_baseline_soft_tolerance_(0.15)
    , rtk_baseline_hard_tolerance_(0.25)
    , baseline_soft_tolerance_cap_(0.0)
    , baseline_hard_tolerance_cap_(0.0)
    , baseline_heading_consistency_limit_(0.9)
    , baseline_heading_relax_speed_(0.05)
    , baseline_sum_(0.0)
    , baseline_window_size_(15)
    , baseline_window_average_(0.0)
    , residual_threshold_(0.3)
    , encoder_mismatch_threshold_(0.2)
    , last_rtk_residual_(std::numeric_limits<double>::quiet_NaN())
    , last_encoder_mismatch_(std::numeric_limits<double>::quiet_NaN())
    , ukf_alpha_(1e-3)
    , ukf_beta_(2.0)
    , ukf_kappa_(0.0)
    , ukf_lambda_(0.0)
    , ukf_gamma_(std::sqrt(static_cast<double>(STATE_SIZE)))
    , wheel_base_(0.6)
    , half_wheel_base_(0.3)
    , min_satellites_(0)
    , imu_angular_velocity_noise_std_(0.01)
    , encoder_velocity_noise_std_(0.02)
    , imu_measurement_active_(false)
    , encoder_measurement_active_(false)
    , fallback_heading_used_(false)
    , fallback_heading_active_(false)
    , fallback_heading_initialized_(false)
    , fallback_heading_estimate_(0.0)
    , fallback_heading_variance_(0.04)
    , fallback_heading_measurement_(0.0)
    , fallback_heading_measurement_variance_(0.04)
    , fallback_heading_stamp_(0)
    , fallback_heading_process_noise_(0.04)
    , fallback_heading_min_variance_(0.04)
    , fallback_heading_max_variance_(1.0)
    , fallback_heading_bias_alpha_(0.02)
    , fallback_velocity_heading_noise_(0.1)
    , fallback_velocity_speed_threshold_(0.05)
    , fallback_gyro_bias_estimate_(0.0)
    , fallback_gyro_bias_initialized_(false)
    , rtk_residual_meter_score_(0.0)
    , encoder_mismatch_score_(0.0)
    , rtk_recent_residual_(std::numeric_limits<double>::quiet_NaN())
    , rtk_health_initialized_(false)
    , encoder_health_initialized_(false)
    , residual_ema_alpha_(computeEmaAlpha(50))
    , encoder_ema_alpha_(computeEmaAlpha(50))
    , residual_window_size_(50)
    , encoder_window_size_(50)
    , rtk_good_streak_(0)
    , encoder_good_streak_(0)
    , rtk_bad_streak_(0)
    , encoder_bad_streak_(0)
    , recovery_streak_required_(3)
    , recovery_release_ratio_(0.6)
    , residual_surge_count_(0)
    , residual_surge_limit_(3)
    , residual_soft_reset_multiplier_(3.0)
    , rtk_nis_gate_2d_(9.21)
    , rtk_nis_gate_3d_(11.34)
    , rtk_selector_hold_frames_(5)
    , rtk_selector_switch_penalty_(0.5)
    , rtk_selected_mode_(RtkMeasurementData::Mode::NONE)
    , rtk_selected_mode_dim_(0)
    , rtk_selected_mode_hold_counter_(0)
{
    last_update_time_ = ros::Time(0);
    coordinate_cache_.resize(cache_size_);
    front_antenna_offset_.setZero();
    rear_antenna_offset_.setZero();
    baseline_history_.clear();
    baseline_window_average_ = rtk_baseline_distance_;
    fallback_heading_process_noise_base_ = fallback_heading_process_noise_;
    resetHealthMonitorUnlocked();

    updateUkfWeights();

    setProcessNoise(0.1, 0.5, 1.0);

    setMeasurementNoise(0.05, 0.01, 0.01, 0.1, 0.02);
}

UKFFusion::~UKFFusion() {
}

void UKFFusion::updateUkfWeights() {
    const double n = static_cast<double>(STATE_SIZE);

    if (!std::isfinite(ukf_alpha_) || ukf_alpha_ <= 0.0) {
        ukf_alpha_ = 1e-3;
    }
    if (!std::isfinite(ukf_beta_)) {
        ukf_beta_ = 2.0;
    }
    if (!std::isfinite(ukf_kappa_)) {
        ukf_kappa_ = 0.0;
    }

    const double alpha_sq = ukf_alpha_ * ukf_alpha_;
    ukf_lambda_ = alpha_sq * (n + ukf_kappa_) - n;
    double denom = n + ukf_lambda_;

    if (!std::isfinite(denom) || denom <= 1e-9) {
        ukf_lambda_ = 0.0;
        denom = n;
    }

    ukf_gamma_ = std::sqrt(denom);

    w0_ = ukf_lambda_ / denom;
    wi_ = 1.0 / (2.0 * denom);
    w0_c_ = w0_ + (1.0 - alpha_sq + ukf_beta_);
    wi_c_ = wi_;
}

double UKFFusion::computeEmaAlpha(std::size_t window_size) const {
    if (window_size <= 1) {
        return 1.0;
    }
    double alpha = 2.0 / (static_cast<double>(window_size) + 1.0);
    return std::clamp(alpha, 0.01, 1.0);
}

void UKFFusion::resetHealthMonitorUnlocked() {
    residual_window_size_ = std::max<std::size_t>(1, residual_window_size_);
    encoder_window_size_ = std::max<std::size_t>(1, encoder_window_size_);
    residual_ema_alpha_ = computeEmaAlpha(residual_window_size_);
    encoder_ema_alpha_ = computeEmaAlpha(encoder_window_size_);
    recovery_streak_required_ = std::clamp(static_cast<int>(residual_window_size_ / 4), 2, 8);
    rtk_residual_meter_score_ = 0.0;
    encoder_mismatch_score_ = 0.0;
    rtk_recent_residual_ = std::numeric_limits<double>::quiet_NaN();
    last_rtk_residual_ = std::numeric_limits<double>::quiet_NaN();
    last_encoder_mismatch_ = std::numeric_limits<double>::quiet_NaN();
    rtk_health_initialized_ = false;
    encoder_health_initialized_ = false;
    rtk_good_streak_ = 0;
    encoder_good_streak_ = 0;
    rtk_bad_streak_ = 0;
    encoder_bad_streak_ = 0;
    health_status_ = {false, false, 0.0, 0.0, 0.0};
    health_status_.rtk_recent_residual = std::numeric_limits<double>::quiet_NaN();
    residual_surge_count_ = 0;
    fallback_heading_active_ = false;
    fallback_heading_used_ = false;
    fallback_heading_process_noise_ = fallback_heading_process_noise_base_;
    fallback_gyro_bias_estimate_ = 0.0;
    fallback_gyro_bias_initialized_ = false;

    last_front_rtk_xy_valid_ = false;
    last_rear_rtk_xy_valid_ = false;
    last_front_step_m_ = std::numeric_limits<double>::quiet_NaN();
    last_rear_step_m_ = std::numeric_limits<double>::quiet_NaN();
    front_jump_until_ = ros::Time(0);
    rear_jump_until_ = ros::Time(0);
    front_jump_active_ = false;
    rear_jump_active_ = false;

    front_bad_score_ = 0.0;
    rear_bad_score_ = 0.0;

    wheel_slip_score_ = 0.0;
    wheel_slip_detected_ = false;
    wheel_slip_nonholonomic_noise_scale_ = 1.0;
}

void UKFFusion::updateWheelSlipStatus(const IMUData& imu, const EncoderData& encoder) {
    wheel_slip_detected_ = false;
    wheel_slip_nonholonomic_noise_scale_ = 1.0;

    if (!wheel_slip_enabled_) {
        wheel_slip_score_ = 0.0;
        return;
    }

    const double a = std::clamp(wheel_slip_ema_alpha_, 0.02, 1.0);

    auto decay_only = [&]() {
        wheel_slip_score_ = std::clamp((1.0 - a) * wheel_slip_score_, 0.0, 1.0);
        wheel_slip_detected_ = wheel_slip_score_ >= 0.5;
        wheel_slip_nonholonomic_noise_scale_ =
            1.0 + wheel_slip_score_ * (std::max(wheel_slip_nonholonomic_noise_scale_max_, 1.0) - 1.0);
    };

    if (!imu.valid || !encoder.valid) {
        decay_only();
        return;
    }
    if (!std::isfinite(imu.angular_velocity_z) ||
        !std::isfinite(encoder.left_velocity) ||
        !std::isfinite(encoder.right_velocity) ||
        wheel_base_ <= 1e-6) {
        decay_only();
        return;
    }

    const double v_lin = 0.5 * (encoder.left_velocity + encoder.right_velocity);
    const double v_abs = std::abs(v_lin);
    const double w_enc = (encoder.right_velocity - encoder.left_velocity) / wheel_base_;
    const double w_imu = imu.angular_velocity_z - imu_bias_.angular_velocity_z;
    const double diff = std::abs(w_enc - w_imu);

    const double yaw_gate = 0.2;
    const bool gate = (v_abs >= wheel_slip_min_speed_mps_) ||
                      (std::abs(w_enc) >= yaw_gate) ||
                      (std::abs(w_imu) >= yaw_gate);
    if (!gate) {
        decay_only();
        return;
    }

    const double denom = std::max({std::abs(w_enc), std::abs(w_imu), yaw_gate});
    const double rel = diff / std::max(denom, 1e-6);

    double sample = 0.0;
    if (diff > wheel_slip_yaw_rate_diff_threshold_ &&
        rel > wheel_slip_yaw_rate_rel_threshold_) {
        const double diff_excess =
            (diff - wheel_slip_yaw_rate_diff_threshold_) /
            std::max(wheel_slip_yaw_rate_diff_threshold_, 1e-6);
        const double rel_excess =
            (rel - wheel_slip_yaw_rate_rel_threshold_) /
            std::max(1.0 - wheel_slip_yaw_rate_rel_threshold_, 0.05);
        sample = std::clamp(std::max(diff_excess, rel_excess), 0.0, 1.0);
    }

    wheel_slip_score_ = std::clamp(a * sample + (1.0 - a) * wheel_slip_score_, 0.0, 1.0);
    wheel_slip_detected_ = wheel_slip_score_ >= 0.5;
    wheel_slip_nonholonomic_noise_scale_ =
        1.0 + wheel_slip_score_ * (std::max(wheel_slip_nonholonomic_noise_scale_max_, 1.0) - 1.0);
}

void UKFFusion::init() {
    x_.setZero();

    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    resetHealthMonitorUnlocked();

    initialized_ = true;
}

void UKFFusion::setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise) {
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

void UKFFusion::setMeasurementNoise(double rtk_position_noise, double rtk_orientation_noise,
                                   double imu_angular_velocity_noise, double imu_linear_acceleration_noise,
                                   double encoder_velocity_noise) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    R_(0, 0) = rtk_position_noise * rtk_position_noise;
    R_(1, 1) = rtk_position_noise * rtk_position_noise;

    R_(2, 2) = rtk_orientation_noise * rtk_orientation_noise;

    R_(3, 3) = imu_angular_velocity_noise * imu_angular_velocity_noise;

    R_(4, 4) = imu_linear_acceleration_noise * imu_linear_acceleration_noise;
    R_(5, 5) = imu_linear_acceleration_noise * imu_linear_acceleration_noise;

    imu_angular_velocity_noise_std_ = std::max(imu_angular_velocity_noise, 1e-6);
    encoder_velocity_noise_std_ = std::max(encoder_velocity_noise, 1e-6);
}

void UKFFusion::setOrigin(double latitude, double longitude) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    origin_latitude_ = latitude;

    origin_longitude_ = longitude;

    geo_converter_.Reset(latitude, longitude, 0);

    for (auto& cache : coordinate_cache_) {
        cache.valid = false;
    }
}

void UKFFusion::setAntennaOffset(double x, double y, double theta) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    antenna_offset_ << x, y, theta;
    front_antenna_offset_ << x, y;
    recalcRearAntennaOffset();
}

void UKFFusion::setRTKBaseline(double distance, double tolerance) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rtk_baseline_distance_ = distance;
    rtk_baseline_tolerance_ = tolerance;
    rtk_baseline_soft_tolerance_ = std::max(tolerance, 1e-6);
    double default_hard = std::max(rtk_baseline_soft_tolerance_ * 1.8, rtk_baseline_soft_tolerance_ + 0.05);
    rtk_baseline_hard_tolerance_ = std::max(default_hard, rtk_baseline_soft_tolerance_);
    if (distance > 0.0) {
        baseline_soft_tolerance_cap_ = std::max({rtk_baseline_soft_tolerance_, distance * 0.4, 0.2});
        baseline_hard_tolerance_cap_ = std::max({rtk_baseline_hard_tolerance_, distance * 0.6, baseline_soft_tolerance_cap_});
    } else {
        baseline_soft_tolerance_cap_ = std::max(rtk_baseline_soft_tolerance_, 0.2);
        baseline_hard_tolerance_cap_ = std::max(rtk_baseline_hard_tolerance_, baseline_soft_tolerance_cap_);
    }
    baseline_history_.clear();
    baseline_sum_ = 0.0;
    baseline_window_average_ = rtk_baseline_distance_;
    recalcRearAntennaOffset();
}

void UKFFusion::setBaselineOutlierPolicy(double soft_tolerance, double hard_tolerance,
                                         double soft_cap, double hard_cap) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (soft_tolerance > 0.0) {
        rtk_baseline_soft_tolerance_ = soft_tolerance;
    }
    if (hard_tolerance > 0.0) {
        rtk_baseline_hard_tolerance_ = std::max(hard_tolerance, rtk_baseline_soft_tolerance_);
    } else {
        rtk_baseline_hard_tolerance_ = std::max(rtk_baseline_soft_tolerance_ * 1.8,
                                                rtk_baseline_soft_tolerance_ + 0.05);
    }
    if (soft_cap > 0.0) {
        baseline_soft_tolerance_cap_ = std::max(soft_cap, rtk_baseline_soft_tolerance_);
    }
    if (hard_cap > 0.0) {
        baseline_hard_tolerance_cap_ = std::max({hard_cap, baseline_soft_tolerance_cap_, rtk_baseline_hard_tolerance_});
    }
    baseline_history_.clear();
    baseline_sum_ = 0.0;
    baseline_window_average_ = rtk_baseline_distance_;
}

void UKFFusion::setBaselineConsistencyChecks(double heading_limit_rad, double relax_speed) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    baseline_heading_consistency_limit_ = heading_limit_rad;
    baseline_heading_relax_speed_ = std::max(relax_speed, 0.0);
}

void UKFFusion::setIMUCalibration(const Eigen::Vector3d& lever_arm, const IMUData& bias) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_lever_arm_ = lever_arm;
    imu_bias_ = bias;
}

void UKFFusion::setWheelBase(double wheel_base) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    wheel_base_ = wheel_base;
    half_wheel_base_ = wheel_base_ * 0.5;
}

void UKFFusion::setMinSatellites(int min_satellites) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    min_satellites_ = min_satellites;
}

void UKFFusion::setRtkNisGate(double gate_2d, double gate_3d) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(gate_2d) && gate_2d > 0.0) {
        rtk_nis_gate_2d_ = gate_2d;
    }
    if (std::isfinite(gate_3d) && gate_3d > 0.0) {
        rtk_nis_gate_3d_ = gate_3d;
    }
}

void UKFFusion::setRtkSelectorConfig(int hold_frames, double switch_penalty) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rtk_selector_hold_frames_ = std::clamp(hold_frames, 0, 50);
    if (std::isfinite(switch_penalty)) {
        rtk_selector_switch_penalty_ = std::clamp(switch_penalty, 0.0, 20.0);
    }
}

void UKFFusion::setRtkSingleStepPolicy(double jump_threshold_m,
                                       double jump_max_dt_s,
                                       double jump_hold_s,
                                       double jump_score_penalty) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(jump_threshold_m) && jump_threshold_m > 0.0) {
        rtk_single_step_jump_threshold_m_ = std::clamp(jump_threshold_m, 0.05, 5.0);
    }
    if (std::isfinite(jump_max_dt_s) && jump_max_dt_s > 0.0) {
        rtk_single_step_jump_max_dt_s_ = std::clamp(jump_max_dt_s, 0.05, 5.0);
    }
    if (std::isfinite(jump_hold_s) && jump_hold_s >= 0.0) {
        rtk_single_step_hold_s_ = std::clamp(jump_hold_s, 0.0, 30.0);
    }
    if (std::isfinite(jump_score_penalty) && jump_score_penalty >= 0.0) {
        rtk_single_step_score_penalty_ = std::clamp(jump_score_penalty, 0.0, 10.0);
    }
}

void UKFFusion::setRtkNoHeadingPositionNoiseScale(double min_scale, double max_scale) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(min_scale)) {
        rtk_no_heading_position_noise_scale_min_ = std::clamp(min_scale, 1.0, 500.0);
    }
    if (std::isfinite(max_scale)) {
        if (max_scale <= 0.0) {
            rtk_no_heading_position_noise_scale_max_ = 0.0;
        } else {
            rtk_no_heading_position_noise_scale_max_ = std::clamp(max_scale, 1.0, 5000.0);
        }
    }
    if (rtk_no_heading_position_noise_scale_max_ > 0.0 &&
        rtk_no_heading_position_noise_scale_max_ < rtk_no_heading_position_noise_scale_min_) {
        rtk_no_heading_position_noise_scale_max_ = rtk_no_heading_position_noise_scale_min_;
    }
}

void UKFFusion::setRtkStationaryPositionNoiseScale(double speed_threshold,
                                                   double position_noise_scale,
                                                   double release_time_s) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(speed_threshold)) {
        rtk_stationary_speed_threshold_ = std::max(speed_threshold, 0.0);
    }
    if (std::isfinite(position_noise_scale)) {
        rtk_stationary_position_noise_scale_ = std::clamp(position_noise_scale, 1.0, 5000.0);
    }
    if (std::isfinite(release_time_s)) {
        rtk_stationary_release_time_s_ = std::max(release_time_s, 0.0);
    }
    rtk_stationary_active_ = false;
    rtk_stationary_release_start_ = ros::Time(0);
}

void UKFFusion::setRtkTurnInPlacePositionNoiseScale(double speed_threshold,
                                                    double yaw_rate_threshold,
                                                    double position_noise_scale) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(speed_threshold)) {
        rtk_turn_speed_threshold_ = std::max(speed_threshold, 0.0);
    }
    if (std::isfinite(yaw_rate_threshold)) {
        rtk_turn_yaw_rate_threshold_ = std::max(yaw_rate_threshold, 0.0);
    }
    if (std::isfinite(position_noise_scale)) {
        rtk_turn_position_noise_scale_ = std::clamp(position_noise_scale, 1.0, 5000.0);
    }
}

void UKFFusion::setBaselineCourseConsistency(double consistency_limit_rad) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(consistency_limit_rad) && consistency_limit_rad > 0.0) {
        baseline_course_consistency_limit_rad_ = std::clamp(consistency_limit_rad, 0.01, 1.57);
    } else {
        baseline_course_consistency_limit_rad_ = 0.0;
    }
    baseline_course_history_.clear();
}

void UKFFusion::setBaselineDegradedFrontPenalty(double penalty) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(penalty) && penalty > 0.0) {
        baseline_degraded_front_score_penalty_ = std::clamp(penalty, 0.0, 10.0);
    } else {
        baseline_degraded_front_score_penalty_ = 0.0;
    }
}

void UKFFusion::setRtkAntennaPreferenceConfig(bool enabled,
                                              double ema_alpha,
                                              double penalty_scale,
                                              double residual_good_m,
                                              double residual_bad_m,
                                              double max_penalty) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    rtk_preference_enabled_ = enabled;

    if (std::isfinite(ema_alpha) && ema_alpha > 0.0) {
        rtk_preference_ema_alpha_ = std::clamp(ema_alpha, 0.01, 1.0);
    }
    if (std::isfinite(penalty_scale) && penalty_scale >= 0.0) {
        rtk_preference_penalty_scale_ = std::clamp(penalty_scale, 0.0, 10.0);
    }
    if (std::isfinite(residual_good_m) && residual_good_m >= 0.0) {
        rtk_preference_residual_good_m_ = std::clamp(residual_good_m, 0.0, 5.0);
    }
    if (std::isfinite(residual_bad_m) && residual_bad_m >= 0.0) {
        rtk_preference_residual_bad_m_ = std::clamp(residual_bad_m, 0.01, 20.0);
    }
    if (rtk_preference_residual_bad_m_ < rtk_preference_residual_good_m_ + 1e-3) {
        rtk_preference_residual_bad_m_ = rtk_preference_residual_good_m_ + 0.05;
    }
    if (std::isfinite(max_penalty) && max_penalty >= 0.0) {
        rtk_preference_max_penalty_ = std::clamp(max_penalty, 0.0, 20.0);
    }
}

UKFFusion::RtkDebugInfo UKFFusion::getRtkDebugInfo() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_rtk_debug_;
}

void UKFFusion::setVisionNisGate(double gate_3d) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (std::isfinite(gate_3d)) {
        vision_nis_gate_3d_ = gate_3d;
    }
}

UKFFusion::VisionDebugInfo UKFFusion::getVisionDebugInfo() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return last_vision_debug_;
}

void UKFFusion::setResidualThreshold(double threshold) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    residual_threshold_ = std::max(0.0, threshold);
}

void UKFFusion::setEncoderMismatchThreshold(double threshold) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    encoder_mismatch_threshold_ = std::max(0.0, threshold);
}

void UKFFusion::setResidualWindowSize(std::size_t window_size) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    residual_window_size_ = std::max<std::size_t>(1, window_size);
    residual_ema_alpha_ = computeEmaAlpha(residual_window_size_);
    recovery_streak_required_ = std::clamp(static_cast<int>(residual_window_size_ / 4), 2, 8);
}

void UKFFusion::setEncoderWindowSize(std::size_t window_size) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    encoder_window_size_ = std::max<std::size_t>(1, window_size);
    encoder_ema_alpha_ = computeEmaAlpha(encoder_window_size_);
}

void UKFFusion::setResidualSoftResetMultiplier(double multiplier) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    residual_soft_reset_multiplier_ = std::clamp(multiplier, 1.0, 10.0);
}

void UKFFusion::setResidualSurgeLimit(int limit) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    residual_surge_limit_ = std::max(1, limit);
}

void UKFFusion::setRecoveryReleaseRatio(double ratio) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    recovery_release_ratio_ = std::clamp(ratio, 0.1, 0.95);
}

void UKFFusion::setNonholonomicConstraint(bool enabled,
                                          double lateral_velocity_noise,
                                          double speed_threshold) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    nonholonomic_enabled_ = enabled;
    nonholonomic_lateral_velocity_noise_std_ = std::clamp(lateral_velocity_noise, 1e-4, 2.0);
    nonholonomic_speed_threshold_ = std::clamp(speed_threshold, 0.0, 1.0);
}

void UKFFusion::setWheelSlipConfig(bool enabled,
                                   double min_speed_mps,
                                   double yaw_rate_diff_threshold,
                                   double yaw_rate_rel_threshold,
                                   double ema_alpha,
                                   double encoder_weight_floor_scale,
                                   double nonholonomic_noise_scale_max) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    wheel_slip_enabled_ = enabled;

    if (!std::isfinite(min_speed_mps) || min_speed_mps <= 0.0) {
        wheel_slip_min_speed_mps_ = 0.0;
    } else {
        wheel_slip_min_speed_mps_ = std::clamp(min_speed_mps, 0.01, 2.0);
    }

    if (!std::isfinite(yaw_rate_diff_threshold) || yaw_rate_diff_threshold <= 0.0) {
        wheel_slip_yaw_rate_diff_threshold_ = 0.35;
    } else {
        wheel_slip_yaw_rate_diff_threshold_ = std::clamp(yaw_rate_diff_threshold, 0.05, 5.0);
    }

    if (!std::isfinite(yaw_rate_rel_threshold)) {
        wheel_slip_yaw_rate_rel_threshold_ = 0.7;
    } else {
        wheel_slip_yaw_rate_rel_threshold_ = std::clamp(yaw_rate_rel_threshold, 0.0, 0.95);
    }

    if (!std::isfinite(ema_alpha)) {
        wheel_slip_ema_alpha_ = 0.2;
    } else {
        wheel_slip_ema_alpha_ = std::clamp(ema_alpha, 0.02, 1.0);
    }

    if (!std::isfinite(encoder_weight_floor_scale)) {
        wheel_slip_encoder_weight_floor_scale_ = 0.25;
    } else {
        wheel_slip_encoder_weight_floor_scale_ = std::clamp(encoder_weight_floor_scale, 0.0, 1.0);
    }

    if (!std::isfinite(nonholonomic_noise_scale_max) || nonholonomic_noise_scale_max < 1.0) {
        wheel_slip_nonholonomic_noise_scale_max_ = 1.0;
    } else {
        wheel_slip_nonholonomic_noise_scale_max_ = std::clamp(nonholonomic_noise_scale_max, 1.0, 200.0);
    }

    if (!wheel_slip_enabled_) {
        wheel_slip_score_ = 0.0;
        wheel_slip_detected_ = false;
        wheel_slip_nonholonomic_noise_scale_ = 1.0;
    }
}

void UKFFusion::setFallbackHeadingConfig(double process_noise,
                                         double noise_floor,
                                         double max_variance,
                                         double bias_alpha,
                                         double velocity_heading_noise,
                                         double velocity_speed_threshold,
                                         double course_window_s,
                                         double course_min_distance_m,
                                         double course_gain) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double process_std = std::max(process_noise, 0.0);
    fallback_heading_process_noise_ = std::max(1e-6, process_std * process_std);
    fallback_heading_process_noise_base_ = fallback_heading_process_noise_;

    double min_std = std::max(noise_floor, 0.0);
    fallback_heading_min_variance_ = std::max(1e-6, min_std * min_std);

    double max_std = std::max(max_variance, min_std);
    fallback_heading_max_variance_ = std::max(fallback_heading_min_variance_, max_std * max_std);

    fallback_heading_variance_ = std::clamp(fallback_heading_variance_,
                                            fallback_heading_min_variance_,
                                            fallback_heading_max_variance_);
    fallback_heading_measurement_variance_ = std::clamp(fallback_heading_measurement_variance_,
                                                        fallback_heading_min_variance_,
                                                        fallback_heading_max_variance_);
    fallback_heading_bias_alpha_ = std::clamp(bias_alpha, 1e-4, 1.0);
    fallback_velocity_heading_noise_ = std::max(velocity_heading_noise, 1e-6);
    fallback_velocity_speed_threshold_ = std::max(velocity_speed_threshold, 0.0);
    fallback_course_window_s_ = std::clamp(course_window_s, 0.4, 5.0);
    fallback_course_min_distance_m_ = std::clamp(course_min_distance_m, 0.02, 0.5);
    fallback_course_gain_ = std::clamp(course_gain, 0.0, 1.0);
    fallback_course_history_.clear();
    fallback_gyro_bias_estimate_ = 0.0;
    fallback_gyro_bias_initialized_ = false;
}

void UKFFusion::resetHealthMonitor() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    resetHealthMonitorUnlocked();
}

void UKFFusion::generateSigmaPoints() {
    Eigen::MatrixXd P_sym = 0.5 * (P_ + P_.transpose());
    Eigen::LLT<Eigen::MatrixXd> llt(P_sym);
    if (llt.info() != Eigen::Success) {
        Eigen::MatrixXd P_jitter = P_sym;
        double jitter = 1e-9;
        for (int k = 0; k < 8 && llt.info() != Eigen::Success; ++k) {
            P_jitter.diagonal().array() += jitter;
            llt.compute(P_jitter);
            jitter *= 10.0;
        }
    }
    if (llt.info() != Eigen::Success) {
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            X_sig_.col(i) = x_;
        }
        return;
    }

    Eigen::MatrixXd P_sqrt = llt.matrixL();

    const double scale = ukf_gamma_;

    X_sig_.col(0) = x_;
    for (int i = 0; i < STATE_SIZE; ++i) {
        Eigen::VectorXd delta = scale * P_sqrt.col(i);
        X_sig_.col(i + 1) = x_ + delta;
        X_sig_.col(i + 1 + STATE_SIZE) = x_ - delta;
    }
}

void UKFFusion::predictSigmaPoints(double dt, const IMUData& imu) {
    double corrected_ax = imu.linear_acceleration_x - imu_bias_.linear_acceleration_x;
    double corrected_ay = imu.linear_acceleration_y - imu_bias_.linear_acceleration_y;
    double corrected_gz = imu.angular_velocity_z - imu_bias_.angular_velocity_z;

    if (fallback_gyro_bias_initialized_ && std::isfinite(fallback_gyro_bias_estimate_)) {
        corrected_gz -= fallback_gyro_bias_estimate_;
    }

    if (!std::isfinite(corrected_ax)) {
        corrected_ax = 0.0;
    }
    if (!std::isfinite(corrected_ay)) {
        corrected_ay = 0.0;
    }
    if (!std::isfinite(corrected_gz)) {
        corrected_gz = 0.0;
    }

    double omega = corrected_gz;
    double lx = imu_lever_arm_(0);
    double ly = imu_lever_arm_(1);

    double omega_sq = omega * omega;
    if (std::isfinite(lx) && std::isfinite(ly)) {
        corrected_ax += omega_sq * lx;
        corrected_ay += omega_sq * ly;
    }

    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        Eigen::VectorXd x_sig = X_sig_.col(i);

        double x = x_sig(INDEX_X);
        double y = x_sig(INDEX_Y);
        double theta = x_sig(INDEX_THETA);
        double vx = x_sig(INDEX_VX);
        double vy = x_sig(INDEX_VY);

        double dt2 = 0.5 * dt * dt;

        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        double ax_world = corrected_ax * cos_theta - corrected_ay * sin_theta;
        double ay_world = corrected_ax * sin_theta + corrected_ay * cos_theta;

        x_sig(INDEX_X) = x + vx * dt + ax_world * dt2;
        x_sig(INDEX_Y) = y + vy * dt + ay_world * dt2;
        x_sig(INDEX_THETA) = normalizeAngle(theta + corrected_gz * dt);
        x_sig(INDEX_VX) = vx + ax_world * dt;
        x_sig(INDEX_VY) = vy + ay_world * dt;
        x_sig(INDEX_VTHETA) = corrected_gz;
        x_sig(INDEX_AX) = ax_world;
        x_sig(INDEX_AY) = ay_world;
    }
}

void UKFFusion::updateStateFromSigmaPoints() {
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(STATE_SIZE);
    double theta_sin = 0.0;
    double theta_cos = 0.0;

    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        const double weight = (i == 0) ? w0_ : wi_;
        const Eigen::VectorXd& sig = X_sig_.col(i);

        for (int k = 0; k < STATE_SIZE; ++k) {
            if (k == INDEX_THETA) {
                continue;
            }
            mean(k) += weight * sig(k);
        }
        theta_sin += weight * std::sin(sig(INDEX_THETA));
        theta_cos += weight * std::cos(sig(INDEX_THETA));
    }

    mean(INDEX_THETA) = std::atan2(theta_sin, theta_cos);

    for (int i = 0; i < STATE_SIZE; ++i) {
        if (!std::isfinite(mean(i))) {
            mean(i) = 0.0;
        }
    }

    x_ = mean;

    P_.setZero();
    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        Eigen::VectorXd dx = X_sig_.col(i) - x_;
        dx(INDEX_THETA) = normalizeAngle(dx(INDEX_THETA));
        const double weight = (i == 0) ? w0_c_ : wi_c_;
        P_ += weight * dx * dx.transpose();
    }

    P_ += Q_;
    P_ = 0.5 * (P_ + P_.transpose());
}

void UKFFusion::predict(const IMUData& imu) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (!initialized_ || imu.timestamp < last_update_time_) {
        return;
    }

    double dt = (imu.timestamp - last_update_time_).toSec();
    if (dt <= 0) {
        return;
    }

    generateSigmaPoints();

    predictSigmaPoints(dt, imu);

    updateStateFromSigmaPoints();

    last_update_time_ = imu.timestamp;
}

void UKFFusion::updateFallbackHeading(const RtkMeasurementData& rtk_measurement,
                                      const RTKData& front_rtk,
                                      const RTKData& rear_rtk,
                                      const IMUData& imu,
                                      const EncoderData& encoder) {
    bool rtk_heading_reliable = (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL &&
                                 rtk_measurement.values.size() >= 3 &&
                                 rtk_measurement.heading_weight_scale >= 0.2);

    if (rtk_heading_reliable) {
        fallback_course_history_.clear();
        fallback_heading_active_ = false;
        fallback_heading_initialized_ = true;
        fallback_heading_estimate_ = normalizeAngle(rtk_measurement.values(2));
        fallback_heading_variance_ = fallback_heading_min_variance_;
        fallback_heading_measurement_variance_ = fallback_heading_min_variance_;
        fallback_heading_measurement_ = fallback_heading_estimate_;
        fallback_heading_process_noise_ = fallback_heading_process_noise_base_;
        fallback_heading_stamp_ = !imu.timestamp.isZero() ? imu.timestamp : ros::Time::now();
        return;
    }

    if (!imu.valid) {
        fallback_heading_active_ = false;
        return;
    }

    ros::Time stamp = imu.timestamp;
    if (stamp.isZero()) {
        stamp = fallback_heading_stamp_.isZero() ? ros::Time::now() : fallback_heading_stamp_;
    }

    if (!fallback_heading_initialized_) {
        fallback_heading_estimate_ = normalizeAngle(x_(INDEX_THETA));
        fallback_heading_variance_ = fallback_heading_min_variance_;
        fallback_heading_measurement_variance_ = fallback_heading_min_variance_;
        fallback_heading_stamp_ = stamp;
        fallback_heading_initialized_ = true;
    }

    double dt = (stamp - fallback_heading_stamp_).toSec();
    if (dt < 0.0) {
        dt = 0.0;
    }
    dt = std::min(dt, 0.5);

    double imu_rate_raw = imu.angular_velocity_z - imu_bias_.angular_velocity_z;
    double encoder_rate = 0.0;
    bool encoder_rate_valid = false;
    double encoder_linear_speed = 0.0;
    if (encoder.valid && wheel_base_ > 1e-6) {
        encoder_rate = (encoder.right_velocity - encoder.left_velocity) / wheel_base_;
        encoder_rate_valid = std::isfinite(encoder_rate);
        encoder_linear_speed = 0.5 * (encoder.left_velocity + encoder.right_velocity);
    }

    double abs_encoder_speed = std::abs(encoder_linear_speed);

    const double straight_rate_limit = 0.15;
    const double course_rate_limit = 0.25;
    bool lock_straight = false;
    bool allow_course = false;
    if (encoder_rate_valid && abs_encoder_speed > fallback_velocity_speed_threshold_) {
        const double abs_rate = std::abs(encoder_rate);
        lock_straight = abs_rate <= straight_rate_limit;
        allow_course = abs_rate <= course_rate_limit;
    }

    bool course_available = false;
    double course_heading = std::numeric_limits<double>::quiet_NaN();
    if (allow_course && fallback_course_gain_ > 1e-6) {
        Eigen::Vector2d rtk_xy;
        bool rtk_ok = false;
        ros::Time rtk_stamp;

        if (rear_rtk.valid) {
            double rx = 0.0, ry = 0.0;
            WGS84ToLocal(rear_rtk.latitude, rear_rtk.longitude, rx, ry);
            if (std::isfinite(rx) && std::isfinite(ry)) {
                rtk_xy << rx, ry;
                rtk_ok = true;
                rtk_stamp = rear_rtk.timestamp;
            }
        }
        if (!rtk_ok && front_rtk.valid) {
            double fx = 0.0, fy = 0.0;
            WGS84ToLocal(front_rtk.latitude, front_rtk.longitude, fx, fy);
            if (std::isfinite(fx) && std::isfinite(fy)) {
                rtk_xy << fx, fy;
                rtk_ok = true;
                rtk_stamp = front_rtk.timestamp;
            }
        }
        if (rtk_ok) {
            if (rtk_stamp.isZero()) {
                rtk_stamp = stamp;
            }
            const double now_s = rtk_stamp.toSec();
            if (std::isfinite(now_s)) {
                fallback_course_history_.emplace_back(now_s, rtk_xy);
                while (!fallback_course_history_.empty() &&
                       (now_s - fallback_course_history_.front().first) > fallback_course_window_s_) {
                    fallback_course_history_.pop_front();
                }

                if (fallback_course_history_.size() >= 2) {
                    const auto& oldest = fallback_course_history_.front();
                    const auto& newest = fallback_course_history_.back();
                    const double dt_course = newest.first - oldest.first;
                    if (dt_course >= 0.2 && dt_course <= 2.0) {
                        const Eigen::Vector2d dxy = newest.second - oldest.second;
                        const double dist = dxy.norm();
                        const double speed_course = dist / std::max(dt_course, 1e-6);
                        if (dist >= fallback_course_min_distance_m_ &&
                            speed_course >= std::max(fallback_velocity_speed_threshold_, 0.02)) {
                            course_heading = std::atan2(dxy.y(), dxy.x());
                            if (encoder_rate_valid && encoder_linear_speed < -fallback_velocity_speed_threshold_) {
                                course_heading = normalizeAngle(course_heading + 3.14159265358979323846);
                            }
                            course_available = std::isfinite(course_heading);
                        }
                    }
                }
            }
        }
    } else if (!allow_course) {
        fallback_course_history_.clear();
    }

    bool allow_bias_update = encoder_rate_valid &&
        abs_encoder_speed > fallback_velocity_speed_threshold_ &&
        std::isfinite(imu_rate_raw) &&
        !(wheel_slip_enabled_ && wheel_slip_detected_);

    if (allow_bias_update) {
        if (!fallback_gyro_bias_initialized_) {
            double diff = imu_rate_raw - encoder_rate;
            if (lock_straight) {
                diff = imu_rate_raw;
            }
            fallback_gyro_bias_estimate_ = diff;
            fallback_gyro_bias_initialized_ = true;
        } else {
            double diff = imu_rate_raw - encoder_rate;
            double alpha = fallback_heading_bias_alpha_;
            if (lock_straight) {
                diff = imu_rate_raw;
                alpha = std::clamp(fallback_heading_bias_alpha_ * 5.0, 1e-4, 1.0);
            }
            fallback_gyro_bias_estimate_ += alpha * (diff - fallback_gyro_bias_estimate_);
        }
        fallback_gyro_bias_estimate_ = std::clamp(fallback_gyro_bias_estimate_, -0.5, 0.5);
    }

    double imu_rate = imu_rate_raw - (fallback_gyro_bias_initialized_ ? fallback_gyro_bias_estimate_ : 0.0);

    double weighted_rate = 0.0;
    double weight_sum = 0.0;

    if (std::isfinite(imu_rate)) {
        double var = std::max(imu_angular_velocity_noise_std_ * imu_angular_velocity_noise_std_, 1e-6);
        double weight = 1.0 / var;
        weighted_rate += imu_rate * weight;
        weight_sum += weight;
    }

    if (encoder_rate_valid) {
        double base_var = std::max(2.0 * encoder_velocity_noise_std_ * encoder_velocity_noise_std_ /
                                   std::max(wheel_base_ * wheel_base_, 1e-6), 1e-6);
        if (health_status_.consistency_warning) {
            base_var *= 4.0;
        }
        if (wheel_slip_enabled_ && wheel_slip_score_ > 0.01) {
            const double slip_scale = 1.0 + 8.0 * std::clamp(wheel_slip_score_, 0.0, 1.0);
            base_var *= slip_scale;
        }
        double weight = 1.0 / base_var;
        weighted_rate += encoder_rate * weight;
        weight_sum += weight;
    }

    double combined_rate = imu_rate;
    if (weight_sum > 0.0) {
        combined_rate = weighted_rate / weight_sum;
    } else if (!std::isfinite(combined_rate)) {
        combined_rate = 0.0;
    }

    if (lock_straight) {
        combined_rate = 0.0;
    }

    fallback_heading_estimate_ = normalizeAngle(fallback_heading_estimate_ + combined_rate * dt);

    if (course_available) {
        const double err = normalizeAngle(course_heading - fallback_heading_estimate_);
        const double gain = std::clamp(fallback_course_gain_, 0.0, 1.0);
        fallback_heading_estimate_ = normalizeAngle(fallback_heading_estimate_ + gain * err);

        if (std::abs(err) > 0.6) {
            fallback_heading_estimate_ = normalizeAngle(course_heading);
        }
    }

    double process_scale = std::clamp(rtk_measurement.fallback_process_scale, 0.1, 5.0);
    double target_process_noise = fallback_heading_process_noise_base_ * process_scale;
    if (health_status_.consistency_warning) {
        target_process_noise *= 4.0;
    }
    if (health_status_.drift_detected) {
        target_process_noise *= 2.5;
    }
    if (encoder_mismatch_threshold_ > 1e-6 && std::isfinite(last_encoder_mismatch_)) {
        double mismatch_ratio = last_encoder_mismatch_ / encoder_mismatch_threshold_;
        if (mismatch_ratio > 1.0) {
            double boost = std::clamp(mismatch_ratio - 1.0, 0.0, 4.0);
            target_process_noise *= (1.0 + 0.45 * boost);
        }
    }
    if (std::isfinite(health_status_.rtk_residual_avg) && residual_threshold_ > 1e-6) {
        double residual_ratio = health_status_.rtk_residual_avg / residual_threshold_;
        if (residual_ratio > 1.0) {
            double boost = std::clamp(residual_ratio - 1.0, 0.0, 4.0);
            target_process_noise *= (1.0 + 0.3 * boost);
        }
    }

    double process_noise_cap = std::max(fallback_heading_max_variance_,
                                        fallback_heading_process_noise_base_ * 16.0);
    target_process_noise = std::clamp(target_process_noise,
                                      fallback_heading_process_noise_base_,
                                      process_noise_cap);

    double process_diff = target_process_noise - fallback_heading_process_noise_;
    double adapt_alpha = process_diff > 0.0 ? 0.6 : 0.12;
    fallback_heading_process_noise_ += process_diff * adapt_alpha;
    fallback_heading_process_noise_ = std::clamp(fallback_heading_process_noise_,
                                                 fallback_heading_process_noise_base_,
                                                 process_noise_cap);

    double integration_noise = fallback_heading_process_noise_;
    if (weight_sum > 0.0) {
        integration_noise += 1.0 / weight_sum;
    }
    double variance_increment = integration_noise * std::max(dt, 1e-3);
    fallback_heading_variance_ = std::clamp(fallback_heading_variance_ + variance_increment,
                                            fallback_heading_min_variance_,
                                            fallback_heading_max_variance_);

    if (health_status_.consistency_warning) {
        fallback_heading_variance_ = std::min(fallback_heading_max_variance_,
                                              fallback_heading_variance_ * 1.5);
    }

    double dynamic_variance = fallback_heading_variance_;
    if (abs_encoder_speed > fallback_velocity_speed_threshold_) {
        double velocity_scale = std::clamp(
            abs_encoder_speed / (fallback_velocity_speed_threshold_ + 1e-3), 1.0, 5.0);
        double velocity_based_var = fallback_velocity_heading_noise_ * fallback_velocity_heading_noise_ / velocity_scale;
        dynamic_variance = std::min(dynamic_variance, velocity_based_var);
    }
    if (lock_straight) {
        double lock_std = std::clamp(0.25 * fallback_velocity_heading_noise_, 0.006, 0.03);
        dynamic_variance = std::min(dynamic_variance, lock_std * lock_std);
    }

    fallback_heading_measurement_ = fallback_heading_estimate_;
    fallback_heading_measurement_variance_ = std::clamp(dynamic_variance,
                                                        fallback_heading_min_variance_,
                                                        fallback_heading_max_variance_);

    fallback_heading_stamp_ = stamp;
    fallback_heading_active_ = true;
}

Eigen::VectorXd UKFFusion::calculateIMUAndEncoderMeasurement(const IMUData& imu, const EncoderData& encoder) {
    imu_measurement_active_ = false;
    encoder_measurement_active_ = false;
    fallback_heading_used_ = false;
    nonholonomic_active_ = false;

    std::vector<double> values;
    values.reserve(6);

    (void)imu;

    if (encoder.valid) {
        encoder_measurement_active_ = true;
        values.push_back(encoder.left_velocity);
        values.push_back(encoder.right_velocity);

        if (nonholonomic_enabled_) {
            values.push_back(0.0);
            nonholonomic_active_ = true;
        }
    }

    if (fallback_heading_active_ && fallback_heading_initialized_) {
        fallback_heading_used_ = true;
        values.push_back(fallback_heading_measurement_);
    }

    if (values.empty()) {
        return Eigen::VectorXd();
    }

    Eigen::VectorXd z_imu_encoder(static_cast<int>(values.size()));
    for (std::size_t i = 0; i < values.size(); ++i) {
        z_imu_encoder(static_cast<int>(i)) = values[i];
    }
    return z_imu_encoder;
}

void UKFFusion::adaptWeights(const RTKData& front_rtk, const RTKData& rear_rtk,
                             const IMUData& imu, const EncoderData& encoder) {
    updateWheelSlipStatus(imu, encoder);

    double front_quality = computeRTKQuality(front_rtk);
    double rear_quality = computeRTKQuality(rear_rtk);

    if (front_rtk.valid || rear_rtk.valid) {
        double combined_quality = 0.0;
        if (front_rtk.valid && rear_rtk.valid) {
            combined_quality = std::min(front_quality, rear_quality);
        } else {
            combined_quality = front_rtk.valid ? front_quality : rear_quality;
        }

        double adaptive_min = 0.2;
        if (std::isfinite(last_rtk_residual_) && residual_threshold_ > 1e-6 &&
            last_rtk_residual_ > residual_threshold_) {
            double ratio = residual_threshold_ / std::max(last_rtk_residual_, residual_threshold_);
            double scale = std::clamp(ratio, 0.25, 1.0);
            combined_quality *= scale;
        }

        double severity = 0.0;
        if (std::isfinite(health_status_.rtk_residual_avg) && residual_threshold_ > 1e-6) {
            severity = health_status_.rtk_residual_avg / residual_threshold_;
        }

        if (severity > 1.0) {
            double penalty = 1.0 / (1.0 + (severity - 1.0));
            penalty = std::clamp(penalty, 0.35, 1.0);
            combined_quality *= penalty;
            adaptive_min = std::max(adaptive_min, std::min(0.45, 0.2 + 0.1 * (severity - 1.0)));
        }

        if (health_status_.drift_detected) {
            combined_quality = std::max(combined_quality * 0.7, adaptive_min);
        }

        if (severity >= residual_soft_reset_multiplier_ && !health_status_.drift_detected) {
            combined_quality = std::max(combined_quality, 0.6);
        }

        rtk_weight_ = std::clamp(combined_quality, adaptive_min, 1.0);
    } else {
        rtk_weight_ = 0.0;
        rtk_heading_weight_ = 0.0;
    }

    imu_weight_ = imu.valid ? 1.0 : 0.1;

    encoder_weight_ = encoder.valid ? 0.8 : 0.1;
    if (encoder.valid && std::isfinite(last_encoder_mismatch_) &&
        encoder_mismatch_threshold_ > 1e-6 &&
        last_encoder_mismatch_ > encoder_mismatch_threshold_) {
        double ratio = encoder_mismatch_threshold_ /
                       std::max(last_encoder_mismatch_, encoder_mismatch_threshold_);
        double scale = std::clamp(ratio, 0.2, 1.0);
        encoder_weight_ *= scale;
    }

    if (health_status_.consistency_warning) {
        encoder_weight_ = std::max(0.05, encoder_weight_ * 0.6);
    }

    if (wheel_slip_enabled_ && encoder.valid) {
        const double floor_scale = std::clamp(wheel_slip_encoder_weight_floor_scale_, 0.0, 1.0);
        const double scale = 1.0 - wheel_slip_score_ * (1.0 - floor_scale);
        encoder_weight_ *= std::clamp(scale, floor_scale, 1.0);
    }

    if (health_status_.drift_detected) {
        imu_weight_ = std::max(imu_weight_, 1.0);
    }

    rtk_heading_weight_ = rtk_weight_;
    double heading_penalty = 1.0;

    if (!front_rtk.valid || !rear_rtk.valid) {
        heading_penalty = std::min(heading_penalty, 0.1);
    }

    if (std::isfinite(last_rtk_residual_) && residual_threshold_ > 1e-6 &&
        last_rtk_residual_ > residual_threshold_) {
        double ratio = residual_threshold_ / std::max(last_rtk_residual_, residual_threshold_);
        heading_penalty = std::min(heading_penalty, std::clamp(ratio, 0.1, 1.0));
    }

    if (std::isfinite(health_status_.rtk_residual_avg) && residual_threshold_ > 1e-6) {
        double severity = health_status_.rtk_residual_avg / residual_threshold_;
        if (severity > 1.0) {
            heading_penalty = std::min(heading_penalty, 1.0 / (1.0 + (severity - 1.0) * 1.5));
        }
    }

    if (health_status_.drift_detected) {
        heading_penalty = std::min(heading_penalty, 0.25);
    }

    rtk_heading_weight_ = std::clamp(rtk_heading_weight_ * heading_penalty, 0.05, rtk_weight_);

    double total_weight = rtk_weight_ + imu_weight_ + encoder_weight_;
    if (total_weight <= 0.0) {
        rtk_weight_ = 0.4;
        imu_weight_ = 0.4;
        encoder_weight_ = 0.2;
        total_weight = rtk_weight_ + imu_weight_ + encoder_weight_;
    }

    rtk_weight_ /= total_weight;
    imu_weight_ /= total_weight;
    encoder_weight_ /= total_weight;
}

void UKFFusion::update(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    ROS_DEBUG_STREAM("[UKF] Update start. front_valid=" << front_rtk.valid
                     << " rear_valid=" << rear_rtk.valid
                     << " imu_valid=" << imu.valid
                     << " encoder_valid=" << encoder.valid);

    if (!initialized_) {
        initializeFromRTKUnlocked(front_rtk, rear_rtk);
        return;
    }

    adaptWeights(front_rtk, rear_rtk, imu, encoder);

    std::vector<RtkMeasurementData> rtk_candidates = buildRTKCandidateMeasurements(front_rtk, rear_rtk);

    double stationary_pos_scale = 1.0;
    if (rtk_stationary_speed_threshold_ > 0.0 &&
        rtk_stationary_position_noise_scale_ > 1.0) {
        const double thr = rtk_stationary_speed_threshold_;
        bool have_v_trans = false;
        double v_trans = 0.0;
        if (encoder.valid &&
            std::isfinite(encoder.left_velocity) &&
            std::isfinite(encoder.right_velocity)) {
            const double v_lin = 0.5 * (encoder.left_velocity + encoder.right_velocity);
            v_trans = std::abs(v_lin);
            have_v_trans = true;
        } else {
            const double speed_est = std::hypot(x_(INDEX_VX), x_(INDEX_VY));
            if (std::isfinite(speed_est)) {
                v_trans = speed_est;
                have_v_trans = true;
            }
        }

        const bool stationary = have_v_trans && (v_trans <= thr);

        ros::Time stamp = encoder.timestamp;
        if (stamp.isZero()) {
            stamp = !front_rtk.timestamp.isZero() ? front_rtk.timestamp : ros::Time::now();
        }

        if (stationary) {
            stationary_pos_scale = rtk_stationary_position_noise_scale_;
            rtk_stationary_active_ = true;
            rtk_stationary_release_start_ = ros::Time(0);
        } else {
            if (rtk_stationary_active_) {
                rtk_stationary_active_ = false;
                rtk_stationary_release_start_ = stamp;
            }
            if (!rtk_stationary_release_start_.isZero() &&
                rtk_stationary_release_time_s_ > 1e-3 &&
                !stamp.isZero()) {
                double dt = (stamp - rtk_stationary_release_start_).toSec();
                if (dt < 0.0) {
                    dt = 0.0;
                }
                if (dt < rtk_stationary_release_time_s_) {
                    const double ratio = 1.0 - dt / rtk_stationary_release_time_s_;
                    stationary_pos_scale = 1.0 + (rtk_stationary_position_noise_scale_ - 1.0) * ratio;
                } else {
                    rtk_stationary_release_start_ = ros::Time(0);
                }
            }
        }
    }
    double turn_pos_scale = 1.0;
    if (rtk_turn_position_noise_scale_ > 1.0 &&
        rtk_turn_yaw_rate_threshold_ > 1e-6) {
        double speed_thr = rtk_turn_speed_threshold_;
        if (!(std::isfinite(speed_thr) && speed_thr > 0.0)) {
            speed_thr = rtk_stationary_speed_threshold_;
        }

        bool have_speed = false;
        double v_trans = 0.0;
        if (encoder.valid &&
            std::isfinite(encoder.left_velocity) &&
            std::isfinite(encoder.right_velocity)) {
            v_trans = std::abs(0.5 * (encoder.left_velocity + encoder.right_velocity));
            have_speed = true;
        } else {
            const double speed_est = std::hypot(x_(INDEX_VX), x_(INDEX_VY));
            if (std::isfinite(speed_est)) {
                v_trans = speed_est;
                have_speed = true;
            }
        }

        bool have_yaw_rate = false;
        double yaw_rate = 0.0;
        bool opposite_sign = false;
        if (encoder.valid &&
            std::isfinite(encoder.left_velocity) &&
            std::isfinite(encoder.right_velocity) &&
            wheel_base_ > 1e-6) {
            yaw_rate = (encoder.right_velocity - encoder.left_velocity) / wheel_base_;
            have_yaw_rate = std::isfinite(yaw_rate);
            const double product = encoder.left_velocity * encoder.right_velocity;
            opposite_sign = std::isfinite(product) && (product < 0.0);
        } else if (imu.valid && std::isfinite(imu.angular_velocity_z)) {
            yaw_rate = imu.angular_velocity_z;
            have_yaw_rate = true;
        }

        bool turning_in_place = false;
        if ((speed_thr > 0.0) && have_speed && (v_trans <= speed_thr)) {
            if (opposite_sign) {
                turning_in_place = true;
            } else if (have_yaw_rate && (std::abs(yaw_rate) >= rtk_turn_yaw_rate_threshold_)) {
                turning_in_place = true;
            }
        }
        if (turning_in_place) {
            turn_pos_scale = rtk_turn_position_noise_scale_;
        }
    }

    double combined_pos_scale = stationary_pos_scale * turn_pos_scale;
    if (combined_pos_scale > 1.0 && std::isfinite(combined_pos_scale)) {
        for (auto& cand : rtk_candidates) {
            const double base = std::isfinite(cand.position_noise_scale) ? cand.position_noise_scale : 1.0;
            cand.position_noise_scale = std::clamp(base * combined_pos_scale, 1.0, 5000.0);
        }
    }

    generateSigmaPoints();

    RtkMeasurementData rtk_measurement;
    if (!rtk_candidates.empty()) {
        rtk_measurement = selectRtkMeasurement(rtk_candidates);
    } else {
        last_rtk_debug_ = RtkDebugInfo();
        last_rtk_debug_.selected_mode = RtkMeasurementData::Mode::NONE;
        last_rtk_debug_.selected_dim = 0;
        last_rtk_debug_.gate_passed = false;
        last_rtk_debug_.baseline_ok = false;
        last_rtk_debug_.candidate_count = 0;
        last_rtk_debug_.rtk_weight = rtk_weight_;
        last_rtk_debug_.imu_weight = imu_weight_;
        last_rtk_debug_.encoder_weight = encoder_weight_;
        last_rtk_debug_.rtk_heading_weight = rtk_heading_weight_;
    }
    ROS_DEBUG_STREAM("[UKF] RTK measurement selected mode=" << static_cast<int>(rtk_measurement.mode)
                     << " dim=" << rtk_measurement.values.size());
    updateFallbackHeading(rtk_measurement, front_rtk, rear_rtk, imu, encoder);
    Eigen::VectorXd z_imu_encoder = calculateIMUAndEncoderMeasurement(imu, encoder);
    ROS_DEBUG_STREAM("[UKF] IMU/Encoder dim=" << z_imu_encoder.size());

    const int rtk_dim = static_cast<int>(rtk_measurement.values.size());
    const int imu_encoder_dim = static_cast<int>(z_imu_encoder.size());
    const int total_dim = rtk_dim + imu_encoder_dim;

    const bool rtk_used = rtk_dim > 0;
    const bool encoder_used = encoder_measurement_active_;
    const bool imu_used = imu_measurement_active_;

    if (total_dim == 0) {
        last_rtk_debug_ = RtkDebugInfo();
        last_rtk_debug_.selected_mode = RtkMeasurementData::Mode::NONE;
        last_rtk_debug_.candidate_count = static_cast<int>(rtk_candidates.size());
        last_rtk_debug_.rtk_weight = rtk_weight_;
        last_rtk_debug_.imu_weight = imu_weight_;
        last_rtk_debug_.encoder_weight = encoder_weight_;
        last_rtk_debug_.rtk_heading_weight = rtk_heading_weight_;

        updateHealthStatus(std::numeric_limits<double>::quiet_NaN(), false,
                           std::numeric_limits<double>::quiet_NaN(), false);
        return;
    }

    Eigen::MatrixXd Z_pred_all(total_dim, 2 * STATE_SIZE + 1);
    int row_offset = 0;

    if (rtk_used) {
        ROS_DEBUG_STREAM("[UKF] Predicting RTK measurement, dim=" << rtk_dim);
        Eigen::MatrixXd Z_rtk_pred(rtk_dim, 2 * STATE_SIZE + 1);
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            const Eigen::VectorXd& x_sig = X_sig_.col(i);
            double theta = x_sig(INDEX_THETA);

            if (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_dim == 3) {
                Eigen::Vector2d front_rot = rotateOffset(front_antenna_offset_, theta);
                Eigen::Vector2d rear_rot;
                if (rear_offset_initialized_) {
                    rear_rot = rotateOffset(rear_antenna_offset_, theta);
                } else {
                    Eigen::Vector2d estimated_rear = front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
                    rear_rot = rotateOffset(estimated_rear, theta);
                }

                double front_x = x_sig(INDEX_X) + front_rot.x();
                double front_y = x_sig(INDEX_Y) + front_rot.y();
                double rear_x = x_sig(INDEX_X) + rear_rot.x();
                double rear_y = x_sig(INDEX_Y) + rear_rot.y();

                Z_rtk_pred(0, i) = (front_x + rear_x) * 0.5;
                Z_rtk_pred(1, i) = (front_y + rear_y) * 0.5;
                Z_rtk_pred(2, i) = normalizeAngle(atan2(front_y - rear_y, front_x - rear_x));
            } else if (rtk_dim == 2) {
                Eigen::Vector2d front_rot = rotateOffset(front_antenna_offset_, theta);
                Eigen::Vector2d rear_rot;
                if (rear_offset_initialized_) {
                    rear_rot = rotateOffset(rear_antenna_offset_, theta);
                } else {
                    Eigen::Vector2d estimated_rear =
                        front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
                    rear_rot = rotateOffset(estimated_rear, theta);
                }

                if (rtk_measurement.mode == RtkMeasurementData::Mode::FRONT_ONLY) {
                    Z_rtk_pred(0, i) = x_sig(INDEX_X) + front_rot.x();
                    Z_rtk_pred(1, i) = x_sig(INDEX_Y) + front_rot.y();
                } else if (rtk_measurement.mode == RtkMeasurementData::Mode::REAR_ONLY) {
                    Z_rtk_pred(0, i) = x_sig(INDEX_X) + rear_rot.x();
                    Z_rtk_pred(1, i) = x_sig(INDEX_Y) + rear_rot.y();
                } else if (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL) {
                    Z_rtk_pred(0, i) = x_sig(INDEX_X) + 0.5 * (front_rot.x() + rear_rot.x());
                    Z_rtk_pred(1, i) = x_sig(INDEX_Y) + 0.5 * (front_rot.y() + rear_rot.y());
                } else {
                    Z_rtk_pred(0, i) = x_sig(INDEX_X);
                    Z_rtk_pred(1, i) = x_sig(INDEX_Y);
                }
            } else {
                Z_rtk_pred(0, i) = x_sig(INDEX_X);
                Z_rtk_pred(1, i) = x_sig(INDEX_Y);
            }
        }

        Z_pred_all.block(row_offset, 0, rtk_dim, 2 * STATE_SIZE + 1) = Z_rtk_pred;
        row_offset += rtk_dim;
    }

    if (imu_encoder_dim > 0) {
        ROS_DEBUG_STREAM("[UKF] Predicting IMU/Encoder measurement");
        Eigen::MatrixXd Z_imu_encoder_pred(imu_encoder_dim, 2 * STATE_SIZE + 1);
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            const Eigen::VectorXd& x_sig = X_sig_.col(i);
            int row = 0;
            if (imu_used) {
                Z_imu_encoder_pred(row++, i) = x_sig(INDEX_VTHETA);
                Z_imu_encoder_pred(row++, i) = x_sig(INDEX_AX);
                Z_imu_encoder_pred(row++, i) = x_sig(INDEX_AY);
            }
            if (encoder_used) {
                double cos_theta = std::cos(x_sig(INDEX_THETA));
                double sin_theta = std::sin(x_sig(INDEX_THETA));
                double v_body = cos_theta * x_sig(INDEX_VX) + sin_theta * x_sig(INDEX_VY);
                Z_imu_encoder_pred(row++, i) = v_body - x_sig(INDEX_VTHETA) * half_wheel_base_;
                Z_imu_encoder_pred(row++, i) = v_body + x_sig(INDEX_VTHETA) * half_wheel_base_;
                if (nonholonomic_active_) {
                    double v_lat = -sin_theta * x_sig(INDEX_VX) + cos_theta * x_sig(INDEX_VY);
                    Z_imu_encoder_pred(row++, i) = v_lat;
                }
            }
            if (fallback_heading_used_) {
                Z_imu_encoder_pred(row++, i) = x_sig(INDEX_THETA);
            }
        }
        Z_pred_all.block(row_offset, 0, imu_encoder_dim, 2 * STATE_SIZE + 1) = Z_imu_encoder_pred;
    }

    Eigen::VectorXd z_pred = w0_ * Z_pred_all.col(0);
    for (int i = 1; i < 2 * STATE_SIZE + 1; ++i) {
        z_pred += wi_ * Z_pred_all.col(i);
    }

    if (rtk_used && rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_dim == 3) {
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            const double weight = (i == 0) ? w0_ : wi_;
            sin_sum += weight * std::sin(Z_pred_all(2, i));
            cos_sum += weight * std::cos(Z_pred_all(2, i));
        }
        z_pred(2) = std::atan2(sin_sum, cos_sum);
    }
    if (fallback_heading_used_) {
        int fallback_idx = total_dim - 1;
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            const double weight = (i == 0) ? w0_ : wi_;
            sin_sum += weight * std::sin(Z_pred_all(fallback_idx, i));
            cos_sum += weight * std::cos(Z_pred_all(fallback_idx, i));
        }
        z_pred(fallback_idx) = std::atan2(sin_sum, cos_sum);
    }

    Eigen::MatrixXd Pzz = Eigen::MatrixXd::Zero(total_dim, total_dim);
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(STATE_SIZE, total_dim);
    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        Eigen::VectorXd dz = Z_pred_all.col(i) - z_pred;
        if (rtk_used && rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_dim == 3) {
            dz(2) = normalizeAngle(dz(2));
        }
        if (fallback_heading_used_) {
            int fallback_idx = total_dim - 1;
            dz(fallback_idx) = normalizeAngle(dz(fallback_idx));
        }

        const double weight = (i == 0) ? w0_c_ : wi_c_;
        Pzz += weight * dz * dz.transpose();

        Eigen::VectorXd dx = X_sig_.col(i) - x_;
        dx(INDEX_THETA) = normalizeAngle(dx(INDEX_THETA));
        Pxz += weight * dx * dz.transpose();
    }

    Eigen::MatrixXd R_adjusted = Eigen::MatrixXd::Zero(total_dim, total_dim);
    int noise_row = 0;
    if (rtk_used) {
        double pos_weight = std::max(rtk_weight_, 1e-6);
        if (rtk_dim == 3) {
            Eigen::Matrix3d R_rtk = R_.block(0, 0, 3, 3);
            double position_scale = std::max(rtk_measurement.position_noise_scale, 1.0);
            double heading_scale = std::max(rtk_measurement.heading_noise_scale, 1.0);
            double heading_weight = std::max(rtk_heading_weight_, 1e-6) *
                std::max(rtk_measurement.heading_weight_scale, 0.05);

            R_rtk(0, 0) *= position_scale;
            R_rtk(1, 1) *= position_scale;
            R_rtk(0, 0) /= pos_weight;
            R_rtk(1, 1) /= pos_weight;

            R_rtk(2, 2) *= heading_scale;
            R_rtk(2, 2) /= std::max(heading_weight, 1e-6);
            R_adjusted.block(noise_row, noise_row, 3, 3) = R_rtk;
            noise_row += 3;
        } else {
            Eigen::Matrix2d R_rtk = R_.block(0, 0, 2, 2);
            double position_scale = std::max(rtk_measurement.position_noise_scale, 1.0);
            R_rtk(0, 0) *= position_scale;
            R_rtk(1, 1) *= position_scale;
            R_rtk(0, 0) /= pos_weight;
            R_rtk(1, 1) /= pos_weight;
            R_adjusted.block(noise_row, noise_row, 2, 2) = R_rtk;
            noise_row += 2;
        }
    }
    if (imu_encoder_dim > 0) {
        if (imu_used) {
            double imu_weight = std::max(imu_weight_, 1e-6);
            R_adjusted.block(noise_row, noise_row, 3, 3) = R_.block(3, 3, 3, 3) / imu_weight;
            noise_row += 3;
        }
        if (encoder_used) {
            double base_encoder_var = encoder_velocity_noise_std_ * encoder_velocity_noise_std_;
            double encoder_var = base_encoder_var / std::max(encoder_weight_, 1e-6);
            R_adjusted(noise_row, noise_row) = encoder_var;
            R_adjusted(noise_row + 1, noise_row + 1) = encoder_var;
            noise_row += 2;

            if (nonholonomic_active_) {
                double lat_var = nonholonomic_lateral_velocity_noise_std_ * nonholonomic_lateral_velocity_noise_std_;
                lat_var /= std::max(encoder_weight_, 1e-6);
                if (wheel_slip_enabled_ && wheel_slip_score_ > 0.01) {
                    lat_var *= std::max(wheel_slip_nonholonomic_noise_scale_, 1.0);
                }
                R_adjusted(noise_row, noise_row) = lat_var;
                noise_row += 1;
            }
        }
        if (fallback_heading_used_) {
            double combined_weight = 0.5 * (imu_weight_ + encoder_weight_);
            double fallback_weight = std::max(combined_weight, 1e-6);
            double fallback_var = fallback_heading_measurement_variance_ / fallback_weight;
            R_adjusted(noise_row, noise_row) = fallback_var;
            noise_row += 1;
        }
    }

    Pzz += R_adjusted;

    Eigen::FullPivLU<Eigen::MatrixXd> lu(Pzz);
    if (!lu.isInvertible()) {
        updateHealthStatus(std::numeric_limits<double>::quiet_NaN(), rtk_used,
                           std::numeric_limits<double>::quiet_NaN(), encoder_measurement_active_);
        ROS_WARN_STREAM_THROTTLE(1.0, "[UKF] Pzz not invertible, skipping update");
        return;
    }

    Eigen::MatrixXd K = Pxz * lu.inverse();
    ROS_DEBUG_STREAM("[UKF] Kalman gain computed");

    Eigen::VectorXd z_actual(total_dim);
    int actual_row = 0;
    if (rtk_used) {
        z_actual.segment(actual_row, rtk_dim) = rtk_measurement.values;
        actual_row += rtk_dim;
    }
    if (imu_encoder_dim > 0) {
        z_actual.segment(actual_row, imu_encoder_dim) = z_imu_encoder;
    }
    ROS_DEBUG_STREAM("[UKF] z_actual assigned. total_dim=" << total_dim << " actual_row=" << actual_row);

    Eigen::VectorXd innovation = z_actual - z_pred;
    if (rtk_used && rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_dim == 3) {
        innovation(2) = normalizeAngle(innovation(2));
    }
    if (fallback_heading_used_) {
        int fallback_idx = total_dim - 1;
        innovation(fallback_idx) = normalizeAngle(innovation(fallback_idx));
    }
    ROS_DEBUG_STREAM("[UKF] Innovation computed");

    x_ = x_ + K * innovation;
    x_(INDEX_THETA) = normalizeAngle(x_(INDEX_THETA));
    ROS_DEBUG_STREAM("[UKF] State updated");

    Eigen::MatrixXd update_term = K * Pzz * K.transpose();
    if (update_term.array().isFinite().all()) {
        P_ = P_ - update_term;
    }
    ROS_DEBUG_STREAM("[UKF] Covariance updated");

    double rtk_residual = std::numeric_limits<double>::quiet_NaN();
    if (rtk_used) {
        Eigen::VectorXd residual_segment = z_actual.segment(0, rtk_dim) - z_pred.segment(0, rtk_dim);
        if (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_dim == 3) {
            residual_segment(2) = normalizeAngle(residual_segment(2));
        }
        if (rtk_dim >= 2) {
            rtk_residual = residual_segment.head(2).norm();
        } else if (rtk_dim == 1) {
            rtk_residual = std::fabs(residual_segment(0));
        }
    }
    ROS_DEBUG_STREAM("[UKF] RTK residual computed: " << rtk_residual);

    bool soft_reset_applied = false;
    const bool rtk_gate_passed = rtk_used && last_rtk_debug_.gate_passed;
    if (rtk_gate_passed && std::isfinite(rtk_residual) && residual_threshold_ > 1e-6) {
        double soft_threshold = residual_threshold_ * residual_soft_reset_multiplier_;
        if (rtk_residual > soft_threshold) {
            residual_surge_count_ = std::min(residual_surge_count_ + 1, 1000);
        } else {
            residual_surge_count_ = std::max(residual_surge_count_ - 1, 0);
        }

        if (residual_surge_count_ >= residual_surge_limit_) {
            Eigen::Vector2d reset_pos = Eigen::Vector2d::Zero();
            bool has_theta = false;
            double reset_theta = 0.0;

            if (rtk_measurement.values.size() >= 2) {
                Eigen::Vector2d meas_xy(rtk_measurement.values(0), rtk_measurement.values(1));
                double theta_ref = x_(INDEX_THETA);

                Eigen::Vector2d front_rot = rotateOffset(front_antenna_offset_, theta_ref);
                Eigen::Vector2d rear_offset = rear_antenna_offset_;
                if (!rear_offset_initialized_) {
                    rear_offset = front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
                }
                Eigen::Vector2d rear_rot = rotateOffset(rear_offset, theta_ref);

                if (rtk_measurement.mode == RtkMeasurementData::Mode::FRONT_ONLY) {
                    reset_pos = meas_xy - front_rot;
                } else if (rtk_measurement.mode == RtkMeasurementData::Mode::REAR_ONLY) {
                    reset_pos = meas_xy - rear_rot;
                } else if (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL) {
                    reset_pos = meas_xy - 0.5 * (front_rot + rear_rot);
                } else {
                    reset_pos = meas_xy;
                }
            }

            if (rtk_measurement.mode == RtkMeasurementData::Mode::DUAL && rtk_measurement.values.size() >= 3) {
                has_theta = true;
                double baseline_theta = antenna_offset_.z();
                if (!std::isfinite(baseline_theta)) {
                    baseline_theta = 0.0;
                }
                reset_theta = normalizeAngle(rtk_measurement.values(2) - baseline_theta);
            }

            ros::Time reset_stamp = !front_rtk.timestamp.isZero() ? front_rtk.timestamp : ros::Time::now();
            softResetState(reset_pos, has_theta, reset_theta, reset_stamp);
            soft_reset_applied = true;
            rtk_residual = std::numeric_limits<double>::quiet_NaN();
        }
    } else {
        residual_surge_count_ = std::max(residual_surge_count_ - 1, 0);
    }

    double encoder_mismatch = std::numeric_limits<double>::quiet_NaN();
    if (encoder.valid) {
        double cos_theta = std::cos(x_(INDEX_THETA));
        double sin_theta = std::sin(x_(INDEX_THETA));
        double v_body = cos_theta * x_(INDEX_VX) + sin_theta * x_(INDEX_VY);
        double pred_left = v_body - x_(INDEX_VTHETA) * half_wheel_base_;
        double pred_right = v_body + x_(INDEX_VTHETA) * half_wheel_base_;
        encoder_mismatch = 0.5 * (std::fabs(pred_left - encoder.left_velocity) + std::fabs(pred_right - encoder.right_velocity));
    }
    ROS_DEBUG_STREAM("[UKF] Encoder mismatch computed: " << encoder_mismatch);

    if (std::isfinite(rtk_residual)) {
        last_rtk_residual_ = rtk_residual;
    } else {
        last_rtk_residual_ = std::numeric_limits<double>::quiet_NaN();
    }
    if (std::isfinite(encoder_mismatch)) {
        last_encoder_mismatch_ = encoder_mismatch;
    } else {
        last_encoder_mismatch_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (!soft_reset_applied) {
        updateHealthStatus(rtk_residual, rtk_gate_passed, encoder_mismatch,
                           encoder_measurement_active_ && encoder.valid);
    } else {
        updateHealthStatus(std::numeric_limits<double>::quiet_NaN(), false,
                           encoder_mismatch, encoder_measurement_active_ && encoder.valid);
    }
    ROS_DEBUG_STREAM("[UKF] Update finish. residual=" << rtk_residual
                     << " encoder_mismatch=" << encoder_mismatch);
}

std::vector<UKFFusion::RtkMeasurementData> UKFFusion::buildRTKCandidateMeasurements(const RTKData& front_rtk,
                                                                                    const RTKData& rear_rtk) {
    std::vector<RtkMeasurementData> candidates;
    candidates.reserve(4);

    RtkMeasurementData meta;
    meta.front_valid = front_rtk.valid;
    meta.rear_valid = rear_rtk.valid;

    Eigen::Vector2d front_xy;
    Eigen::Vector2d rear_xy;
    bool front_ok = false;
    bool rear_ok = false;

    if (front_rtk.valid) {
        double fx = 0.0, fy = 0.0;
        WGS84ToLocal(front_rtk.latitude, front_rtk.longitude, fx, fy);
        if (std::isfinite(fx) && std::isfinite(fy)) {
            front_xy << fx, fy;
            front_ok = true;
        }
    }

    if (rear_rtk.valid) {
        double rx = 0.0, ry = 0.0;
        WGS84ToLocal(rear_rtk.latitude, rear_rtk.longitude, rx, ry);
        if (std::isfinite(rx) && std::isfinite(ry)) {
            rear_xy << rx, ry;
            rear_ok = true;
        }
    }

    if (front_ok) {
        ros::Time stamp = front_rtk.timestamp;
        if (stamp.isZero()) {
            stamp = ros::Time::now();
        }
        if (last_front_rtk_xy_valid_) {
            const double dt = (stamp - last_front_rtk_stamp_).toSec();
            if (dt > 1e-6 && dt <= rtk_single_step_jump_max_dt_s_) {
                const double step = (front_xy - last_front_rtk_xy_).norm();
                last_front_step_m_ = step;
                if (step >= rtk_single_step_jump_threshold_m_) {
                    front_jump_until_ = stamp + ros::Duration(rtk_single_step_hold_s_);
                }
            }
        }
        last_front_rtk_xy_ = front_xy;
        last_front_rtk_stamp_ = stamp;
        last_front_rtk_xy_valid_ = true;
        front_jump_active_ = (!front_jump_until_.isZero() && stamp < front_jump_until_);
    } else {
        front_jump_active_ = false;
        last_front_step_m_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (rear_ok) {
        ros::Time stamp = rear_rtk.timestamp;
        if (stamp.isZero()) {
            stamp = ros::Time::now();
        }
        if (last_rear_rtk_xy_valid_) {
            const double dt = (stamp - last_rear_rtk_stamp_).toSec();
            if (dt > 1e-6 && dt <= rtk_single_step_jump_max_dt_s_) {
                const double step = (rear_xy - last_rear_rtk_xy_).norm();
                last_rear_step_m_ = step;
                if (step >= rtk_single_step_jump_threshold_m_) {
                    rear_jump_until_ = stamp + ros::Duration(rtk_single_step_hold_s_);
                }
            }
        }
        last_rear_rtk_xy_ = rear_xy;
        last_rear_rtk_stamp_ = stamp;
        last_rear_rtk_xy_valid_ = true;
        rear_jump_active_ = (!rear_jump_until_.isZero() && stamp < rear_jump_until_);
    } else {
        rear_jump_active_ = false;
        last_rear_step_m_ = std::numeric_limits<double>::quiet_NaN();
    }

    if (baseline_course_consistency_limit_rad_ > 1e-6) {
        Eigen::Vector2d course_xy;
        ros::Time course_stamp;
        bool have_course_sample = false;
        if (rear_ok) {
            course_xy = rear_xy;
            course_stamp = rear_rtk.timestamp;
            have_course_sample = true;
        } else if (front_ok) {
            course_xy = front_xy;
            course_stamp = front_rtk.timestamp;
            have_course_sample = true;
        }
        if (have_course_sample && !course_stamp.isZero()) {
            const double now_s = course_stamp.toSec();
            if (std::isfinite(now_s)) {
                if (!baseline_course_history_.empty() &&
                    now_s + 1e-6 < baseline_course_history_.back().first) {
                    baseline_course_history_.clear();
                }
                baseline_course_history_.emplace_back(now_s, course_xy);
                const double window_s = std::clamp(fallback_course_window_s_, 0.2, 6.0);
                while (!baseline_course_history_.empty() &&
                       (now_s - baseline_course_history_.front().first) > window_s) {
                    baseline_course_history_.pop_front();
                }
            }
        }
    }


    bool baseline_available = false;
    bool within_soft = false;
    bool within_hard = false;
    bool heading_consistent = false;
    bool gross_length_failure = false;
    bool drop_heading = true;
    double heading = std::numeric_limits<double>::quiet_NaN();

    if (front_ok && rear_ok && rtk_baseline_distance_ > 0.0) {
        baseline_available = true;
        const double measured_distance = calculateDistance(front_xy.x(), front_xy.y(),
                                                           rear_xy.x(), rear_xy.y());
        const double deviation = std::abs(measured_distance - rtk_baseline_distance_);
        meta.measured_baseline = measured_distance;
        meta.baseline_deviation = deviation;

        double base_soft = rtk_baseline_soft_tolerance_ > 0.0
            ? rtk_baseline_soft_tolerance_
            : std::max(rtk_baseline_tolerance_, 1e-6);
        double base_hard = rtk_baseline_hard_tolerance_ > 0.0
            ? std::max(rtk_baseline_hard_tolerance_, base_soft)
            : std::max(base_soft * 1.8, base_soft + 0.05);

        double dynamic_soft = base_soft;
        double dynamic_hard = std::max(base_hard, base_soft);

        heading = atan2(front_xy.y() - rear_xy.y(), front_xy.x() - rear_xy.x());
        const double state_speed = std::hypot(x_(INDEX_VX), x_(INDEX_VY));

	        heading_consistent = true;
	        if (baseline_heading_consistency_limit_ > 0.0) {
	            double predicted_heading = normalizeAngle(x_(INDEX_THETA));
	            double baseline_theta = antenna_offset_.z();
	            if (std::isfinite(baseline_theta)) {
	                predicted_heading = normalizeAngle(predicted_heading + baseline_theta);
	            }
	            double heading_diff = normalizeAngle(heading - predicted_heading);
	            double limit = baseline_heading_consistency_limit_;
	            if (baseline_heading_relax_speed_ > 1e-6 && state_speed < baseline_heading_relax_speed_) {
	                double ratio =
	                    (baseline_heading_relax_speed_ - state_speed) / std::max(baseline_heading_relax_speed_, 1e-6);
	                ratio = std::clamp(ratio, 0.0, 1.0);
	                const double relaxed = M_PI_2;
	                limit = std::clamp(limit + (relaxed - limit) * ratio, limit, relaxed);
	            }
	            heading_consistent = std::abs(heading_diff) <= limit;
	        }

	        if (heading_consistent &&
	            baseline_course_consistency_limit_rad_ > 1e-6 &&
	            baseline_course_history_.size() >= 2) {
	            const auto& oldest = baseline_course_history_.front();
	            const auto& newest = baseline_course_history_.back();
	            const double dt_course = newest.first - oldest.first;
	            if (dt_course >= 0.2 && dt_course <= 2.0) {
	                const Eigen::Vector2d dxy = newest.second - oldest.second;
	                const double dist = dxy.norm();
	                const double speed_course = dist / std::max(dt_course, 1e-6);
	                const double speed_thr = std::max(baseline_heading_relax_speed_, 0.02);
	                if (dist >= std::max(fallback_course_min_distance_m_, 0.02) &&
	                    speed_course >= speed_thr) {
	                    const double course_heading = std::atan2(dxy.y(), dxy.x());
	                    double axis_diff = std::abs(normalizeAngle(heading - course_heading));
	                    if (axis_diff > M_PI_2) {
	                        axis_diff = M_PI - axis_diff;
	                    }
	                    if (axis_diff > baseline_course_consistency_limit_rad_) {
	                        heading_consistent = false;
	                    }
	                }
	            }
	        }

	        const double ratio = (rtk_baseline_distance_ > 1e-6)
	            ? measured_distance / rtk_baseline_distance_
	            : 1.0;
        gross_length_failure = ratio < 0.5 || ratio > 1.6;

        within_soft = deviation <= dynamic_soft;
        within_hard = deviation <= dynamic_hard;

        if (within_hard && heading_consistent && !gross_length_failure) {
            baseline_history_.push_back(measured_distance);
            baseline_sum_ += measured_distance;
            if (baseline_history_.size() > baseline_window_size_) {
                baseline_sum_ -= baseline_history_.front();
                baseline_history_.pop_front();
            }
            if (!baseline_history_.empty()) {
                baseline_window_average_ = baseline_sum_ / static_cast<double>(baseline_history_.size());
            }
            if (baseline_history_.size() == baseline_window_size_) {
                double avg_deviation = std::abs(baseline_window_average_ - rtk_baseline_distance_);
                if (avg_deviation > base_soft * 0.5) {
                    dynamic_soft = std::max(base_soft, avg_deviation + 0.02);
                    dynamic_hard = std::max(dynamic_soft * 1.5, dynamic_soft + 0.05);
                }
            }
        }

        if (baseline_soft_tolerance_cap_ > 0.0) {
            dynamic_soft = std::min(dynamic_soft, baseline_soft_tolerance_cap_);
        }
        if (baseline_hard_tolerance_cap_ > 0.0) {
            dynamic_hard = std::min(dynamic_hard, baseline_hard_tolerance_cap_);
        }

        within_soft = deviation <= dynamic_soft;
        within_hard = deviation <= dynamic_hard;

        meta.baseline_soft_limit = dynamic_soft;
        meta.baseline_hard_limit = dynamic_hard;

	        drop_heading = !within_hard || !heading_consistent || gross_length_failure;

        double position_noise_scale = 1.0;
        double heading_noise_scale = 1.0;
        double heading_weight_scale = 1.0;
        double fallback_process_scale = 1.0;

        if (!within_soft) {
            double ratio_dev = deviation / std::max(dynamic_soft, 1e-6);
            position_noise_scale = std::clamp(1.0 + ratio_dev, 1.0, 4.0);
            heading_noise_scale = std::clamp(1.0 + ratio_dev * 3.0, 1.0, 10.0);
            heading_weight_scale = std::clamp(1.0 / heading_noise_scale, 0.12, 1.0);
            fallback_process_scale = std::clamp(1.0 / (1.0 + ratio_dev), 0.35, 1.0);
        }

        meta.baseline_ok = within_soft && !drop_heading;
        if (drop_heading) {
            meta.baseline_ok = false;
            double min_scale = std::max(rtk_no_heading_position_noise_scale_min_, 1.0);
            double max_scale = rtk_no_heading_position_noise_scale_max_;
            double scale = std::max(position_noise_scale, min_scale);
            if (std::isfinite(max_scale) && max_scale > 0.0) {
                scale = std::min(scale, max_scale);
            }
            meta.position_noise_scale = scale;
            meta.heading_noise_scale = 1.0;
            meta.heading_weight_scale = 0.05;
            meta.fallback_process_scale = std::min(0.35, fallback_process_scale);
        } else {
            meta.position_noise_scale = position_noise_scale;
            meta.heading_noise_scale = heading_noise_scale;
            meta.heading_weight_scale = heading_weight_scale;
            meta.fallback_process_scale = fallback_process_scale;
        }
    } else {
        meta.baseline_ok = false;
        meta.baseline_deviation = std::numeric_limits<double>::quiet_NaN();
        meta.measured_baseline = std::numeric_limits<double>::quiet_NaN();
        meta.baseline_soft_limit = std::numeric_limits<double>::quiet_NaN();
        meta.baseline_hard_limit = std::numeric_limits<double>::quiet_NaN();
        meta.position_noise_scale = 1.0;
        meta.heading_noise_scale = 1.0;
        meta.heading_weight_scale = 1.0;
        meta.fallback_process_scale = 1.0;
    }

    if (std::isfinite(health_status_.rtk_residual_avg) && residual_threshold_ > 1e-6) {
        const double severity = health_status_.rtk_residual_avg / residual_threshold_;
        if (std::isfinite(severity) && severity > 1.0) {
            const double infl_pos = std::clamp(1.0 + 0.35 * (severity - 1.0), 1.0, 3.0);
            const double infl_head = std::clamp(1.0 + 0.55 * (severity - 1.0), 1.0, 5.0);
            if (std::isfinite(meta.position_noise_scale)) {
                meta.position_noise_scale = std::clamp(meta.position_noise_scale * infl_pos, 1.0, 5000.0);
            }
            if (std::isfinite(meta.heading_noise_scale)) {
                meta.heading_noise_scale = std::clamp(meta.heading_noise_scale * infl_head, 1.0, 5000.0);
            }
            meta.heading_weight_scale = std::clamp(meta.heading_weight_scale / infl_head, 0.01, 1.0);
        }
    }

    auto quality_inflation = [&](const RTKData& rtk) -> double {
        const double q = computeRTKQuality(rtk);
        if (!std::isfinite(q) || q <= 1e-6) {
            return 5.0;
        }
        const double q_clip = std::clamp(q, 0.2, 1.0);
        return std::clamp(1.0 / q_clip, 1.0, 5.0);
    };
    const double front_quality_infl = front_ok ? quality_inflation(front_rtk) : 1.0;
    const double rear_quality_infl = rear_ok ? quality_inflation(rear_rtk) : 1.0;
    const double dual_quality_infl = std::max(front_quality_infl, rear_quality_infl);

    if (front_ok) {
        RtkMeasurementData cand = meta;
        cand.mode = RtkMeasurementData::Mode::FRONT_ONLY;
        cand.values = Eigen::VectorXd(2);
        cand.values << front_xy.x(), front_xy.y();
        cand.position_noise_scale = std::clamp(cand.position_noise_scale * front_quality_infl, 1.0, 5000.0);
        cand.heading_noise_scale = std::clamp(cand.heading_noise_scale * front_quality_infl, 1.0, 5000.0);
        cand.heading_weight_scale = std::clamp(cand.heading_weight_scale / front_quality_infl, 0.01, 1.0);
        candidates.push_back(cand);
    }
    if (rear_ok) {
        RtkMeasurementData cand = meta;
        cand.mode = RtkMeasurementData::Mode::REAR_ONLY;
        cand.values = Eigen::VectorXd(2);
        cand.values << rear_xy.x(), rear_xy.y();
        cand.position_noise_scale = std::clamp(cand.position_noise_scale * rear_quality_infl, 1.0, 5000.0);
        cand.heading_noise_scale = std::clamp(cand.heading_noise_scale * rear_quality_infl, 1.0, 5000.0);
        cand.heading_weight_scale = std::clamp(cand.heading_weight_scale / rear_quality_infl, 0.01, 1.0);
        candidates.push_back(cand);
    }

    if (front_ok && rear_ok) {
        bool allow_dual_pos = true;
        if (baseline_available) {
            allow_dual_pos = !gross_length_failure;
        }
        if (allow_dual_pos) {
            RtkMeasurementData cand = meta;
            cand.mode = RtkMeasurementData::Mode::DUAL;
            cand.values = Eigen::VectorXd(2);
            Eigen::Vector2d antenna_avg = 0.5 * (front_xy + rear_xy);
            cand.values << antenna_avg.x(), antenna_avg.y();
            cand.position_noise_scale = std::clamp(cand.position_noise_scale * dual_quality_infl, 1.0, 5000.0);
            cand.heading_noise_scale = std::clamp(cand.heading_noise_scale * dual_quality_infl, 1.0, 5000.0);
            cand.heading_weight_scale = std::clamp(cand.heading_weight_scale / dual_quality_infl, 0.01, 1.0);
            candidates.push_back(cand);
        }
    }

    if (baseline_available && !drop_heading && std::isfinite(heading)) {
        RtkMeasurementData cand = meta;
        cand.mode = RtkMeasurementData::Mode::DUAL;
        cand.values = Eigen::VectorXd(3);
        cand.values << (front_xy.x() + rear_xy.x()) * 0.5,
                        (front_xy.y() + rear_xy.y()) * 0.5,
                        heading;
        cand.position_noise_scale = std::clamp(cand.position_noise_scale * dual_quality_infl, 1.0, 5000.0);
        cand.heading_noise_scale = std::clamp(cand.heading_noise_scale * dual_quality_infl, 1.0, 5000.0);
        cand.heading_weight_scale = std::clamp(cand.heading_weight_scale / dual_quality_infl, 0.01, 1.0);
        candidates.push_back(cand);
    }

    return candidates;
}

bool UKFFusion::computeRtkCandidateNis(const RtkMeasurementData& candidate, double& nis, double& pos_residual) {
    const int dim = static_cast<int>(candidate.values.size());
    if (dim <= 0) {
        return false;
    }

    Eigen::MatrixXd Z_pred(dim, 2 * STATE_SIZE + 1);
    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        const Eigen::VectorXd& x_sig = X_sig_.col(i);
        double theta = x_sig(INDEX_THETA);

        if (candidate.mode == RtkMeasurementData::Mode::DUAL && dim == 3) {
            Eigen::Vector2d front_rot = rotateOffset(front_antenna_offset_, theta);
            Eigen::Vector2d rear_rot;
            if (rear_offset_initialized_) {
                rear_rot = rotateOffset(rear_antenna_offset_, theta);
            } else {
                Eigen::Vector2d estimated_rear =
                    front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
                rear_rot = rotateOffset(estimated_rear, theta);
            }

            double front_x = x_sig(INDEX_X) + front_rot.x();
            double front_y = x_sig(INDEX_Y) + front_rot.y();
            double rear_x = x_sig(INDEX_X) + rear_rot.x();
            double rear_y = x_sig(INDEX_Y) + rear_rot.y();

            Z_pred(0, i) = (front_x + rear_x) * 0.5;
            Z_pred(1, i) = (front_y + rear_y) * 0.5;
            Z_pred(2, i) = normalizeAngle(atan2(front_y - rear_y, front_x - rear_x));
        } else if (dim == 2) {
            Eigen::Vector2d front_rot = rotateOffset(front_antenna_offset_, theta);
            Eigen::Vector2d rear_rot;
            if (rear_offset_initialized_) {
                rear_rot = rotateOffset(rear_antenna_offset_, theta);
            } else {
                Eigen::Vector2d estimated_rear =
                    front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
                rear_rot = rotateOffset(estimated_rear, theta);
            }

            if (candidate.mode == RtkMeasurementData::Mode::FRONT_ONLY) {
                Z_pred(0, i) = x_sig(INDEX_X) + front_rot.x();
                Z_pred(1, i) = x_sig(INDEX_Y) + front_rot.y();
            } else if (candidate.mode == RtkMeasurementData::Mode::REAR_ONLY) {
                Z_pred(0, i) = x_sig(INDEX_X) + rear_rot.x();
                Z_pred(1, i) = x_sig(INDEX_Y) + rear_rot.y();
            } else if (candidate.mode == RtkMeasurementData::Mode::DUAL) {
                Z_pred(0, i) = x_sig(INDEX_X) + 0.5 * (front_rot.x() + rear_rot.x());
                Z_pred(1, i) = x_sig(INDEX_Y) + 0.5 * (front_rot.y() + rear_rot.y());
            } else {
                Z_pred(0, i) = x_sig(INDEX_X);
                Z_pred(1, i) = x_sig(INDEX_Y);
            }
        } else {
            Z_pred(0, i) = x_sig(INDEX_X);
            Z_pred(1, i) = x_sig(INDEX_Y);
        }
    }

    Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(dim);
    if (candidate.mode == RtkMeasurementData::Mode::DUAL && dim == 3) {
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
            const double weight = (i == 0) ? w0_ : wi_;
            z_pred(0) += weight * Z_pred(0, i);
            z_pred(1) += weight * Z_pred(1, i);
            sin_sum += weight * std::sin(Z_pred(2, i));
            cos_sum += weight * std::cos(Z_pred(2, i));
        }
        z_pred(2) = std::atan2(sin_sum, cos_sum);
    } else {
        z_pred = w0_ * Z_pred.col(0);
        for (int i = 1; i < 2 * STATE_SIZE + 1; ++i) {
            z_pred += wi_ * Z_pred.col(i);
        }
    }

    Eigen::MatrixXd Pzz = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < 2 * STATE_SIZE + 1; ++i) {
        Eigen::VectorXd dz = Z_pred.col(i) - z_pred;
        if (candidate.mode == RtkMeasurementData::Mode::DUAL && dim == 3) {
            dz(2) = normalizeAngle(dz(2));
        }
        const double weight = (i == 0) ? w0_c_ : wi_c_;
        Pzz += weight * dz * dz.transpose();
    }

    Eigen::MatrixXd R_adj = Eigen::MatrixXd::Zero(dim, dim);
    double pos_weight = std::max(rtk_weight_, 1e-6);
    if (dim == 3) {
        Eigen::Matrix3d R_rtk = R_.block(0, 0, 3, 3);
        double position_scale = std::max(candidate.position_noise_scale, 1.0);
        double heading_scale = std::max(candidate.heading_noise_scale, 1.0);
        double heading_weight = std::max(rtk_heading_weight_, 1e-6) *
            std::max(candidate.heading_weight_scale, 0.05);

        R_rtk(0, 0) *= position_scale;
        R_rtk(1, 1) *= position_scale;
        R_rtk(0, 0) /= pos_weight;
        R_rtk(1, 1) /= pos_weight;

        R_rtk(2, 2) *= heading_scale;
        R_rtk(2, 2) /= std::max(heading_weight, 1e-6);
        R_adj = R_rtk;
    } else if (dim == 2) {
        Eigen::Matrix2d R_rtk = R_.block(0, 0, 2, 2);
        double position_scale = std::max(candidate.position_noise_scale, 1.0);
        R_rtk(0, 0) *= position_scale;
        R_rtk(1, 1) *= position_scale;
        R_rtk(0, 0) /= pos_weight;
        R_rtk(1, 1) /= pos_weight;
        R_adj = R_rtk;
    } else {
        return false;
    }

    Pzz += R_adj;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(Pzz);
    if (!lu.isInvertible()) {
        return false;
    }

    Eigen::VectorXd innovation = candidate.values - z_pred;
    if (candidate.mode == RtkMeasurementData::Mode::DUAL && dim == 3) {
        innovation(2) = normalizeAngle(innovation(2));
    }

    pos_residual = (dim >= 2) ? innovation.head(2).norm() : std::fabs(innovation(0));
    nis = innovation.transpose() * lu.inverse() * innovation;
    return std::isfinite(nis);
}

UKFFusion::RtkMeasurementData UKFFusion::selectRtkMeasurement(const std::vector<RtkMeasurementData>& candidates) {
    last_rtk_debug_ = RtkDebugInfo();
    last_rtk_debug_.candidate_count = static_cast<int>(candidates.size());
    last_rtk_debug_.rtk_weight = rtk_weight_;
    last_rtk_debug_.imu_weight = imu_weight_;
    last_rtk_debug_.encoder_weight = encoder_weight_;
    last_rtk_debug_.rtk_heading_weight = rtk_heading_weight_;
    last_rtk_debug_.front_bad_score = front_bad_score_;
    last_rtk_debug_.rear_bad_score = rear_bad_score_;

    if (!candidates.empty()) {
        last_rtk_debug_.baseline_ok = candidates.front().baseline_ok;
        last_rtk_debug_.measured_baseline_m = candidates.front().measured_baseline;
        last_rtk_debug_.baseline_deviation_m = candidates.front().baseline_deviation;
        last_rtk_debug_.baseline_soft_limit_m = candidates.front().baseline_soft_limit;
        last_rtk_debug_.baseline_hard_limit_m = candidates.front().baseline_hard_limit;
    }

    const bool have_last = (rtk_selected_mode_ != RtkMeasurementData::Mode::NONE && rtk_selected_mode_dim_ > 0);

    struct AntennaStat {
        bool available = false;
        bool pass = false;
        double gate = std::numeric_limits<double>::quiet_NaN();
        double nis = std::numeric_limits<double>::quiet_NaN();
        double residual_m = std::numeric_limits<double>::quiet_NaN();
    };

    AntennaStat front_stat;
    AntennaStat rear_stat;

    const bool have_both_antennas =
        (!candidates.empty() && candidates.front().front_valid && candidates.front().rear_valid);
    bool baseline_degraded = false;
    double baseline_severity = 0.0;
    if (have_both_antennas && !candidates.empty()) {
        const auto& meta = candidates.front();
        bool gross = false;
        if (std::isfinite(meta.measured_baseline) && rtk_baseline_distance_ > 1e-6) {
            const double ratio = meta.measured_baseline / rtk_baseline_distance_;
            gross = (ratio < 0.5) || (ratio > 1.6);
        }
        bool hard = false;
        if (std::isfinite(meta.baseline_deviation) &&
            std::isfinite(meta.baseline_hard_limit) &&
            meta.baseline_hard_limit > 1e-6) {
            hard = meta.baseline_deviation > meta.baseline_hard_limit;
            baseline_severity = std::clamp(meta.baseline_deviation / meta.baseline_hard_limit, 0.0, 10.0);
        }
        baseline_degraded = gross || hard;
    }

    int best_pass_idx = -1;
    double best_pass_score = std::numeric_limits<double>::infinity();
    double best_pass_nis = std::numeric_limits<double>::quiet_NaN();
    double best_pass_residual = std::numeric_limits<double>::quiet_NaN();

    int best_any_2d_idx = -1;
    double best_any_2d_score = std::numeric_limits<double>::infinity();
    double best_any_2d_nis = std::numeric_limits<double>::quiet_NaN();
    double best_any_2d_residual = std::numeric_limits<double>::quiet_NaN();

    int best_any_idx = -1;
    double best_any_score = std::numeric_limits<double>::infinity();
    double best_any_nis = std::numeric_limits<double>::quiet_NaN();
    double best_any_residual = std::numeric_limits<double>::quiet_NaN();

    int last_idx = -1;
    double last_nis = std::numeric_limits<double>::quiet_NaN();
    double last_residual = std::numeric_limits<double>::quiet_NaN();
    bool last_pass = false;

    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
        const auto& cand = candidates[static_cast<std::size_t>(idx)];
        const int dim = static_cast<int>(cand.values.size());
        if (dim <= 0) {
            continue;
        }

        double nis = std::numeric_limits<double>::quiet_NaN();
        double pos_residual = std::numeric_limits<double>::quiet_NaN();
        if (!computeRtkCandidateNis(cand, nis, pos_residual)) {
            continue;
        }

        const double gate = (dim == 2) ? rtk_nis_gate_2d_ : rtk_nis_gate_3d_;
        const bool pass = std::isfinite(gate) && gate > 0.0 && nis <= gate;

        double score = nis / static_cast<double>(dim);

        if (cand.mode == RtkMeasurementData::Mode::FRONT_ONLY && dim == 2) {
            front_stat.available = true;
            front_stat.pass = pass;
            front_stat.gate = gate;
            front_stat.nis = nis;
            front_stat.residual_m = pos_residual;
        } else if (cand.mode == RtkMeasurementData::Mode::REAR_ONLY && dim == 2) {
            rear_stat.available = true;
            rear_stat.pass = pass;
            rear_stat.gate = gate;
            rear_stat.nis = nis;
            rear_stat.residual_m = pos_residual;
        }
        if (have_last && (cand.mode != rtk_selected_mode_ || dim != rtk_selected_mode_dim_)) {
            double switch_penalty = rtk_selector_switch_penalty_;
            if (cand.baseline_ok && cand.mode == RtkMeasurementData::Mode::DUAL) {
                switch_penalty *= 0.15;
            }
            score += switch_penalty;
        }
        if (cand.baseline_ok) {
            if (cand.mode == RtkMeasurementData::Mode::FRONT_ONLY ||
                cand.mode == RtkMeasurementData::Mode::REAR_ONLY) {
                score += 0.25;
            } else if (cand.mode == RtkMeasurementData::Mode::DUAL && dim == 2) {
                score += 0.08;
            }
        }

        if (rtk_single_step_score_penalty_ > 0.0) {
            if (cand.mode == RtkMeasurementData::Mode::FRONT_ONLY && front_jump_active_) {
                score += rtk_single_step_score_penalty_;
            } else if (cand.mode == RtkMeasurementData::Mode::REAR_ONLY && rear_jump_active_) {
                score += rtk_single_step_score_penalty_;
            } else if (cand.mode == RtkMeasurementData::Mode::DUAL &&
                       (front_jump_active_ || rear_jump_active_)) {
                score += 0.6 * rtk_single_step_score_penalty_;
            }
        }

        if (have_both_antennas && dim == 2) {
            double pref_scale = std::clamp(rtk_preference_penalty_scale_, 0.0, 10.0);
            if (baseline_degraded) {
                pref_scale *= (1.0 + 0.6 * std::clamp(baseline_severity, 0.0, 3.0));
            } else {
                pref_scale *= 0.25;
            }

            if (rtk_preference_enabled_ && pref_scale > 1e-6) {
                if (cand.mode == RtkMeasurementData::Mode::FRONT_ONLY) {
                    score += std::clamp(pref_scale * front_bad_score_, 0.0, rtk_preference_max_penalty_);
                } else if (cand.mode == RtkMeasurementData::Mode::REAR_ONLY) {
                    score += std::clamp(pref_scale * rear_bad_score_, 0.0, rtk_preference_max_penalty_);
                } else if (cand.mode == RtkMeasurementData::Mode::DUAL) {
                    const double avg_bad = 0.5 * (front_bad_score_ + rear_bad_score_);
                    score += std::clamp(0.6 * pref_scale * avg_bad, 0.0, rtk_preference_max_penalty_);
                }
            } else if (!rtk_preference_enabled_ && baseline_degraded &&
                       baseline_degraded_front_score_penalty_ > 1e-6 &&
                       cand.mode == RtkMeasurementData::Mode::FRONT_ONLY) {
                score += baseline_degraded_front_score_penalty_;
            }
        }

        if (score < best_any_score) {
            best_any_score = score;
            best_any_idx = idx;
            best_any_nis = nis;
            best_any_residual = pos_residual;
        }
        if (dim == 2 && score < best_any_2d_score) {
            best_any_2d_score = score;
            best_any_2d_idx = idx;
            best_any_2d_nis = nis;
            best_any_2d_residual = pos_residual;
        }

        if (have_last && cand.mode == rtk_selected_mode_ && dim == rtk_selected_mode_dim_) {
            last_idx = idx;
            last_nis = nis;
            last_residual = pos_residual;
            last_pass = pass;
        }

        if (pass && score < best_pass_score) {
            best_pass_score = score;
            best_pass_idx = idx;
            best_pass_nis = nis;
            best_pass_residual = pos_residual;
        }
    }

    int selected_idx = -1;
    double selected_nis = std::numeric_limits<double>::quiet_NaN();
    double selected_residual = std::numeric_limits<double>::quiet_NaN();
    bool gate_passed = false;

    if (best_pass_idx >= 0) {
        selected_idx = best_pass_idx;
        selected_nis = best_pass_nis;
        selected_residual = best_pass_residual;
        gate_passed = true;
    } else if (best_any_2d_idx >= 0) {
        selected_idx = best_any_2d_idx;
        selected_nis = best_any_2d_nis;
        selected_residual = best_any_2d_residual;
        gate_passed = false;
    } else if (best_any_idx >= 0) {
        selected_idx = best_any_idx;
        selected_nis = best_any_nis;
        selected_residual = best_any_residual;
        gate_passed = false;
    }

    bool suppress_hold = false;
    if (rtk_selected_mode_ == RtkMeasurementData::Mode::FRONT_ONLY && front_jump_active_) {
        suppress_hold = true;
    } else if (rtk_selected_mode_ == RtkMeasurementData::Mode::REAR_ONLY && rear_jump_active_) {
        suppress_hold = true;
    }

    if (!suppress_hold && have_last && rtk_selector_hold_frames_ > 0 &&
        rtk_selected_mode_hold_counter_ < rtk_selector_hold_frames_ &&
        last_idx >= 0 && last_pass) {
        selected_idx = last_idx;
        selected_nis = last_nis;
        selected_residual = last_residual;
        gate_passed = true;
    }

    if (selected_idx < 0) {
        rtk_selected_mode_ = RtkMeasurementData::Mode::NONE;
        rtk_selected_mode_dim_ = 0;
        rtk_selected_mode_hold_counter_ = 0;

        last_rtk_debug_.selected_mode = RtkMeasurementData::Mode::NONE;
        last_rtk_debug_.selected_dim = 0;
        last_rtk_debug_.gate_passed = false;
        last_rtk_debug_.nis = std::numeric_limits<double>::quiet_NaN();
        last_rtk_debug_.residual_pos_m = std::numeric_limits<double>::quiet_NaN();
        return RtkMeasurementData();
    }

    if (have_both_antennas && rtk_preference_enabled_ && rtk_preference_ema_alpha_ > 1e-6) {
        auto compute_badness = [&](const AntennaStat& st, bool jump_active) -> double {
            if (jump_active) {
                return 1.0;
            }
            if (!st.available || !std::isfinite(st.residual_m)) {
                return 0.0;
            }
            const double good = std::max(rtk_preference_residual_good_m_, 0.0);
            const double bad = std::max(rtk_preference_residual_bad_m_, good + 1e-3);
            double b = 0.0;
            if (st.residual_m <= good) {
                b = 0.0;
            } else if (st.residual_m >= bad) {
                b = 1.0;
            } else {
                b = (st.residual_m - good) / std::max(bad - good, 1e-6);
            }
            if (std::isfinite(st.gate) && st.gate > 1e-6 && std::isfinite(st.nis)) {
                const double ratio = st.nis / st.gate;
                if (ratio > 1.0) {
                    b = std::max(b, std::clamp((ratio - 1.0) / 3.0, 0.0, 1.0));
                }
            }
            return std::clamp(b, 0.0, 1.0);
        };

        const double alpha = std::clamp(rtk_preference_ema_alpha_, 0.01, 1.0);
        const double bad_front = compute_badness(front_stat, front_jump_active_);
        const double bad_rear = compute_badness(rear_stat, rear_jump_active_);
        const double alpha_front = (bad_front > 0.8) ? std::min(1.0, alpha * 2.5) : alpha;
        const double alpha_rear = (bad_rear > 0.8) ? std::min(1.0, alpha * 2.5) : alpha;
        front_bad_score_ = std::clamp((1.0 - alpha_front) * front_bad_score_ + alpha_front * bad_front, 0.0, 1.0);
        rear_bad_score_ = std::clamp((1.0 - alpha_rear) * rear_bad_score_ + alpha_rear * bad_rear, 0.0, 1.0);
        last_rtk_debug_.front_bad_score = front_bad_score_;
        last_rtk_debug_.rear_bad_score = rear_bad_score_;
    }

    RtkMeasurementData selected = candidates[static_cast<std::size_t>(selected_idx)];
    const int selected_dim = static_cast<int>(selected.values.size());

    if (!gate_passed) {
        const double gate = (selected_dim == 2) ? rtk_nis_gate_2d_ : rtk_nis_gate_3d_;
        double nis_ratio = 1.0;
        if (std::isfinite(gate) && gate > 0.0 && std::isfinite(selected_nis)) {
            nis_ratio = std::max(1.0, selected_nis / gate);
        }
        double inflation = std::clamp(nis_ratio, 1.0, 25.0);
        selected.position_noise_scale = std::clamp(selected.position_noise_scale * inflation, 1.0, 200.0);
        selected.heading_noise_scale = std::clamp(selected.heading_noise_scale * inflation, 1.0, 200.0);
        selected.heading_weight_scale = std::clamp(selected.heading_weight_scale / inflation, 0.01, 1.0);
        selected.fallback_process_scale = std::clamp(selected.fallback_process_scale / inflation, 0.1, 1.0);
    }

    if (selected.mode == rtk_selected_mode_ && selected_dim == rtk_selected_mode_dim_) {
        rtk_selected_mode_hold_counter_ = std::min(rtk_selected_mode_hold_counter_ + 1, 1000);
    } else {
        rtk_selected_mode_ = selected.mode;
        rtk_selected_mode_dim_ = selected_dim;
        rtk_selected_mode_hold_counter_ = 0;
    }

    last_rtk_debug_.selected_mode = selected.mode;
    last_rtk_debug_.selected_dim = selected_dim;
    last_rtk_debug_.gate_passed = gate_passed;
    last_rtk_debug_.nis = selected_nis;
    last_rtk_debug_.residual_pos_m = selected_residual;

    return selected;
}

Eigen::Vector2d UKFFusion::antennaMeasurementToBase(const Eigen::Vector2d& antenna_xy, bool is_front) {
    Eigen::Vector2d offset = is_front ? front_antenna_offset_ : rear_antenna_offset_;
    if (!is_front && !rear_offset_initialized_) {
        offset = front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
    }

    Eigen::Vector2d rotated = rotateOffset(offset, x_(INDEX_THETA));
    return antenna_xy - rotated;
}

Eigen::Vector2d UKFFusion::rotateOffset(const Eigen::Vector2d& offset, double theta) const {
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    return Eigen::Vector2d(offset.x() * cos_theta - offset.y() * sin_theta,
                           offset.x() * sin_theta + offset.y() * cos_theta);
}

void UKFFusion::updateHealthStatus(double rtk_residual, bool rtk_used,
                                   double encoder_mismatch, bool encoder_used) {
    const double residual_threshold_safe = std::max(residual_threshold_, 1e-6);
    const double encoder_threshold_safe = std::max(encoder_mismatch_threshold_, 1e-6);
    const double release_factor = std::clamp(recovery_release_ratio_, 0.2, 0.95);

    if (rtk_used && std::isfinite(rtk_residual)) {
        rtk_recent_residual_ = rtk_residual;

        if (!rtk_health_initialized_) {
            rtk_residual_meter_score_ = rtk_residual;
            rtk_health_initialized_ = true;
        } else {
            rtk_residual_meter_score_ =
                residual_ema_alpha_ * rtk_residual +
                (1.0 - residual_ema_alpha_) * rtk_residual_meter_score_;
        }

        const bool above_threshold = rtk_residual > residual_threshold_safe;
        const bool below_release = rtk_residual <= residual_threshold_safe * release_factor;

        if (above_threshold) {
            rtk_bad_streak_ = std::min(rtk_bad_streak_ + 1, 1000);
            rtk_good_streak_ = 0;
            health_status_.drift_detected = true;
        } else {
            rtk_bad_streak_ = 0;
            if (below_release) {
                rtk_good_streak_ = std::min(rtk_good_streak_ + 1, 1000);
            } else {
                rtk_good_streak_ = 0;
            }
        }

        if (health_status_.drift_detected && rtk_good_streak_ >= recovery_streak_required_) {
            health_status_.drift_detected = false;
            rtk_good_streak_ = 0;
        }
    } else {
        rtk_residual_meter_score_ *= (1.0 - residual_ema_alpha_);
    }

    if (encoder_used && std::isfinite(encoder_mismatch)) {
        if (!encoder_health_initialized_) {
            encoder_mismatch_score_ = encoder_mismatch;
            encoder_health_initialized_ = true;
        } else {
            encoder_mismatch_score_ =
                encoder_ema_alpha_ * encoder_mismatch +
                (1.0 - encoder_ema_alpha_) * encoder_mismatch_score_;
        }

        const bool above_threshold = encoder_mismatch > encoder_threshold_safe;
        const bool below_release = encoder_mismatch <= encoder_threshold_safe * release_factor;

        if (above_threshold) {
            encoder_bad_streak_ = std::min(encoder_bad_streak_ + 1, 1000);
            encoder_good_streak_ = 0;
            health_status_.consistency_warning = true;
        } else {
            encoder_bad_streak_ = 0;
            if (below_release) {
                encoder_good_streak_ = std::min(encoder_good_streak_ + 1, 1000);
            } else {
                encoder_good_streak_ = 0;
            }
        }

        if (health_status_.consistency_warning && encoder_good_streak_ >= recovery_streak_required_) {
            health_status_.consistency_warning = false;
            encoder_good_streak_ = 0;
        }
    } else {
        encoder_mismatch_score_ *= (1.0 - encoder_ema_alpha_);
    }

    health_status_.rtk_residual_avg = rtk_residual_meter_score_;
    health_status_.encoder_mismatch_avg = encoder_mismatch_score_;
    health_status_.rtk_recent_residual = std::isfinite(rtk_recent_residual_)
        ? rtk_recent_residual_
        : std::numeric_limits<double>::quiet_NaN();
    health_status_.wheel_slip_detected = wheel_slip_detected_;
    health_status_.wheel_slip_score = wheel_slip_enabled_
        ? wheel_slip_score_
        : std::numeric_limits<double>::quiet_NaN();
}

void UKFFusion::softResetState(const Eigen::Vector2d& position, bool has_theta, double theta,
                               const ros::Time& stamp) {
    x_(INDEX_X) = position.x();
    x_(INDEX_Y) = position.y();
    if (has_theta) {
        x_(INDEX_THETA) = normalizeAngle(theta);
    }

    x_(INDEX_VX) = 0.0;
    x_(INDEX_VY) = 0.0;
    x_(INDEX_VTHETA) = 0.0;
    x_(INDEX_AX) = 0.0;
    x_(INDEX_AY) = 0.0;

    P_.setZero();
    P_(INDEX_X, INDEX_X) = 0.25;
    P_(INDEX_Y, INDEX_Y) = 0.25;
    P_(INDEX_THETA, INDEX_THETA) = 0.05;
    P_(INDEX_VX, INDEX_VX) = 0.04;
    P_(INDEX_VY, INDEX_VY) = 0.04;
    P_(INDEX_VTHETA, INDEX_VTHETA) = 0.04;
    P_(INDEX_AX, INDEX_AX) = 0.09;
    P_(INDEX_AY, INDEX_AY) = 0.09;

    last_update_time_ = stamp;
    initialized_ = true;

    residual_surge_count_ = 0;
    rtk_residual_meter_score_ = 0.0;
    rtk_health_initialized_ = false;
    health_status_.drift_detected = false;
    health_status_.rtk_residual_avg = 0.0;
    health_status_.rtk_recent_residual = 0.0;
    last_rtk_residual_ = std::numeric_limits<double>::quiet_NaN();

    latest_result_.valid = false;
}

void UKFFusion::recalcRearAntennaOffset() {
    if (rtk_baseline_distance_ <= 0.0) {
        rear_offset_initialized_ = false;
        rear_antenna_offset_.setZero();
        return;
    }
    double baseline_theta = antenna_offset_.z();
    if (!std::isfinite(baseline_theta)) {
        baseline_theta = 0.0;
    }

    Eigen::Vector2d baseline_vec(rtk_baseline_distance_ * std::cos(baseline_theta),
                                 rtk_baseline_distance_ * std::sin(baseline_theta));
    rear_antenna_offset_ = front_antenna_offset_ - baseline_vec;
    rear_offset_initialized_ = true;
}

double UKFFusion::computeRTKQuality(const RTKData& rtk) const {
    if (!rtk.valid) {
        return 0.0;
    }

    double fix_quality = 0.4;
    if (rtk.fix_mode == 4) {
        fix_quality = 1.0;
    } else if (rtk.fix_mode == 5) {
        fix_quality = 0.7;
    } else if (rtk.fix_mode <= 0) {
        fix_quality = 0.2;
    }

    double satellite_factor = 1.0;
    if (min_satellites_ > 0) {
        int delta = rtk.satellite_count - min_satellites_;
        if (delta <= 0) {
            satellite_factor = 0.3;
        } else if (delta <= 4) {
            satellite_factor = 0.6;
        } else if (delta <= 8) {
            satellite_factor = 0.8;
        } else {
            satellite_factor = 1.0;
        }
    }

    return std::clamp(fix_quality * satellite_factor, 0.05, 1.0);
}

FusionResult UKFFusion::getFusionResult() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    FusionResult result;

    result.valid = false;

    if (!initialized_) {
        return result;
    }

    result.timestamp = last_update_time_.isZero()
        ? ros::Time::now().toSec()
        : last_update_time_.toSec();

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

bool UKFFusion::initializeFromRTK(const RTKData& front_rtk, const RTKData& rear_rtk) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return initializeFromRTKUnlocked(front_rtk, rear_rtk);
}

bool UKFFusion::isInitialized() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return initialized_;
}

bool UKFFusion::initializeFromRTKUnlocked(const RTKData& front_rtk, const RTKData& rear_rtk) {
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

    Eigen::Vector2d antenna_avg((front_x + rear_x) * 0.5, (front_y + rear_y) * 0.5);

    double baseline_heading = std::atan2(front_y - rear_y, front_x - rear_x);
    double baseline_theta = antenna_offset_.z();
    if (!std::isfinite(baseline_theta)) {
        baseline_theta = 0.0;
    }
    double theta = normalizeAngle(baseline_heading - baseline_theta);

    Eigen::Vector2d rear_offset = rear_antenna_offset_;
    if (!rear_offset_initialized_) {
        rear_offset = front_antenna_offset_ - Eigen::Vector2d(rtk_baseline_distance_, 0.0);
    }
    Eigen::Vector2d avg_offset = 0.5 * (front_antenna_offset_ + rear_offset);
    Eigen::Vector2d avg_offset_world = rotateOffset(avg_offset, theta);
    Eigen::Vector2d position = antenna_avg - avg_offset_world;
    ros::Time stamp = !front_rtk.timestamp.isZero() ? front_rtk.timestamp : rear_rtk.timestamp;

    softResetState(position, true, theta, stamp);
    return true;
}

UKFFusion::HealthStatus UKFFusion::getHealthStatus() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return health_status_;
}

void UKFFusion::reset() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    x_.setZero();

    P_ = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);

    last_update_time_ = ros::Time(0);

    initialized_ = false;

    rtk_weight_ = 1.0;
    imu_weight_ = 1.0;
    encoder_weight_ = 1.0;

    imu_measurement_active_ = false;
    encoder_measurement_active_ = false;
    fallback_heading_used_ = false;
    fallback_heading_active_ = false;
    fallback_heading_initialized_ = false;
    fallback_heading_variance_ = fallback_heading_min_variance_;
    fallback_heading_measurement_variance_ = fallback_heading_min_variance_;
    fallback_heading_estimate_ = 0.0;
    fallback_heading_measurement_ = 0.0;
    fallback_heading_stamp_ = ros::Time(0);
    fallback_gyro_bias_estimate_ = 0.0;
    fallback_gyro_bias_initialized_ = false;
    fallback_course_history_.clear();
    nonholonomic_active_ = false;
    baseline_history_.clear();
    baseline_sum_ = 0.0;
    baseline_window_average_ = rtk_baseline_distance_;

    resetHealthMonitorUnlocked();
}

void UKFFusion::zeroVelocityUpdate() {
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

void UKFFusion::applyVisionPoseMeasurement(const VisionPoseMeasurement& measurement) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_vision_debug_ = VisionDebugInfo();
    last_vision_debug_.attempted = measurement.valid && initialized_;
    last_vision_debug_.dim = 3;

    if (!measurement.valid || !initialized_) {
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

    last_vision_debug_.pos_var_lateral = pos_var_lat;
    last_vision_debug_.pos_var_along = pos_var_along;
    last_vision_debug_.heading_var = heading_var;

    if (pos_var_along != pos_var_lat) {
        double theta_axis = x_(INDEX_THETA);
        double c = std::cos(theta_axis);
        double s = std::sin(theta_axis);
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

    Eigen::Matrix3d S_inv = Eigen::Matrix3d::Zero();
    bool invertible = false;
    Eigen::LLT<Eigen::Matrix3d> llt(S);
    if (llt.info() == Eigen::Success) {
        S_inv = llt.solve(Eigen::Matrix3d::Identity());
        invertible = true;
    } else {
        Eigen::FullPivLU<Eigen::Matrix3d> lu(S);
        if (lu.isInvertible()) {
            S_inv = lu.inverse();
            invertible = true;
        }
    }
    if (!invertible) {
        last_vision_debug_.gate_passed = false;
        return;
    }

    double nis = innovation.transpose() * S_inv * innovation;
    if (!std::isfinite(nis)) {
        nis = std::numeric_limits<double>::quiet_NaN();
    }
    last_vision_debug_.nis = nis;

    double dx = innovation(0);
    double dy = innovation(1);
    last_vision_debug_.residual_pos_m = std::hypot(dx, dy);
    double theta_axis = x_(INDEX_THETA);
    double c = std::cos(theta_axis);
    double s = std::sin(theta_axis);
    last_vision_debug_.residual_along_m = c * dx + s * dy;
    last_vision_debug_.residual_lateral_m = -s * dx + c * dy;
    last_vision_debug_.residual_heading_rad = innovation(2);

    bool gate_enabled = std::isfinite(vision_nis_gate_3d_) && vision_nis_gate_3d_ > 0.0;
    bool pass = !gate_enabled || (std::isfinite(nis) && nis <= vision_nis_gate_3d_);
    last_vision_debug_.gate_passed = pass;
    if (!pass) {
        return;
    }

    Eigen::Matrix<double, STATE_SIZE, 3> K = P_ * Ht * S_inv;

    x_ = x_ + K * innovation;
    x_(INDEX_THETA) = normalizeAngle(x_(INDEX_THETA));

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
    Eigen::MatrixXd KH = K * H;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
}

void UKFFusion::setInitialTime(const ros::Time& time) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (last_update_time_.isZero()) {
        last_update_time_ = time;
    }
}

double UKFFusion::normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle));
}

double UKFFusion::calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

void UKFFusion::WGS84ToLocal(double lat, double lon, double& x, double& y) {
    static std::hash<double> hasher;
    size_t hash_key = hasher(lat) ^ (hasher(lon) << 1);
    size_t cache_index = hash_key % cache_size_;

    CoordinateCache& cache = coordinate_cache_[cache_index];
    if (cache.valid &&
        std::abs(cache.lat - lat) < 1e-9 &&
        std::abs(cache.lon - lon) < 1e-9) {
        x = cache.x;
        y = cache.y;
        return;
    }

    double h;
    geo_converter_.Forward(lat, lon, 0, x, y, h);

    cache.lat = lat;
    cache.lon = lon;
    cache.x = x;
    cache.y = y;
    cache.valid = true;
}

void UKFFusion::localToWGS84(double x, double y, double& lat, double& lon) {
    double h;
    geo_converter_.Reverse(x, y, 0, lat, lon, h);
}

void UKFFusion::setAlpha(double alpha) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ukf_alpha_ = alpha;
    updateUkfWeights();
}

void UKFFusion::setBeta(double beta) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ukf_beta_ = beta;
    updateUkfWeights();
}

void UKFFusion::setKappa(double kappa) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    ukf_kappa_ = kappa;
    updateUkfWeights();
}
