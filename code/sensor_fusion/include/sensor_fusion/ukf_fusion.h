#ifndef SENSOR_FUSION_UKF_FUSION_H
#define SENSOR_FUSION_UKF_FUSION_H

#include "sensor_fusion/base_fusion.h"

#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <memory>
#include <cstddef>
#include <limits>

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

struct CoordinateCache {
    double lat, lon, x, y;
    bool valid;
};

	class UKFFusion : public BaseFusion {
	public:
	    struct RtkMeasurementData {
	        enum class Mode { NONE, DUAL, FRONT_ONLY, REAR_ONLY } mode;
	        Eigen::VectorXd values;
	        bool baseline_ok;
	        bool front_valid;
	        bool rear_valid;
	        double baseline_deviation;
	        double measured_baseline;
	        double baseline_soft_limit;
	        double baseline_hard_limit;
	        double position_noise_scale;
	        double heading_noise_scale;
	        double heading_weight_scale;
	        double fallback_process_scale;
	        RtkMeasurementData()
	            : mode(Mode::NONE)
	            , baseline_ok(false)
	            , front_valid(false)
	            , rear_valid(false)
	            , baseline_deviation(0.0)
	            , measured_baseline(0.0)
	            , baseline_soft_limit(0.0)
	            , baseline_hard_limit(0.0)
	            , position_noise_scale(1.0)
	            , heading_noise_scale(1.0)
	            , heading_weight_scale(1.0)
	            , fallback_process_scale(1.0) {}
	    };

    struct RtkDebugInfo {
        RtkMeasurementData::Mode selected_mode = RtkMeasurementData::Mode::NONE;
        int selected_dim = 0;
        bool gate_passed = false;
        bool baseline_ok = false;
        double measured_baseline_m = std::numeric_limits<double>::quiet_NaN();
        double baseline_deviation_m = std::numeric_limits<double>::quiet_NaN();
        double baseline_soft_limit_m = std::numeric_limits<double>::quiet_NaN();
        double baseline_hard_limit_m = std::numeric_limits<double>::quiet_NaN();
        double nis = std::numeric_limits<double>::quiet_NaN();
        double residual_pos_m = std::numeric_limits<double>::quiet_NaN();
        double rtk_weight = std::numeric_limits<double>::quiet_NaN();
        double imu_weight = std::numeric_limits<double>::quiet_NaN();
        double encoder_weight = std::numeric_limits<double>::quiet_NaN();
        double rtk_heading_weight = std::numeric_limits<double>::quiet_NaN();
        int candidate_count = 0;

        double front_bad_score = std::numeric_limits<double>::quiet_NaN();
        double rear_bad_score = std::numeric_limits<double>::quiet_NaN();
    };

    struct VisionDebugInfo {
        bool attempted = false;
        bool gate_passed = false;
        int dim = 0;
        double nis = std::numeric_limits<double>::quiet_NaN();
        double residual_pos_m = std::numeric_limits<double>::quiet_NaN();
        double residual_along_m = std::numeric_limits<double>::quiet_NaN();
        double residual_lateral_m = std::numeric_limits<double>::quiet_NaN();
        double residual_heading_rad = std::numeric_limits<double>::quiet_NaN();
        double pos_var_lateral = std::numeric_limits<double>::quiet_NaN();
        double pos_var_along = std::numeric_limits<double>::quiet_NaN();
        double heading_var = std::numeric_limits<double>::quiet_NaN();
    };

    UKFFusion();

    ~UKFFusion();

    void init() override;

    void setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise) override;

    void setMeasurementNoise(double rtk_position_noise, double rtk_orientation_noise,
                            double imu_angular_velocity_noise, double imu_linear_acceleration_noise,
                            double encoder_velocity_noise) override;

    void setOrigin(double latitude, double longitude) override;

    void setAntennaOffset(double x, double y, double theta) override;

    void setRTKBaseline(double distance, double tolerance) override;

    void setIMUCalibration(const Eigen::Vector3d& lever_arm, const IMUData& bias) override;

    void predict(const IMUData& imu) override;

    void update(const RTKData& front_rtk, const RTKData& rear_rtk, const IMUData& imu, const EncoderData& encoder) override;

    void applyVisionPoseMeasurement(const VisionPoseMeasurement& measurement) override;

    FusionResult getFusionResult() override;

    bool initializeFromRTK(const RTKData& front_rtk, const RTKData& rear_rtk) override;

    bool isInitialized() const override;

    struct HealthStatus {
        bool drift_detected;
        bool consistency_warning;
        double rtk_residual_avg;
        double encoder_mismatch_avg;
        double rtk_recent_residual;
        bool wheel_slip_detected = false;
        double wheel_slip_score = std::numeric_limits<double>::quiet_NaN();
    };

    HealthStatus getHealthStatus() const;

    void reset() override;

    void zeroVelocityUpdate();

    void setAlpha(double alpha);
    void setBeta(double beta);
    void setKappa(double kappa);

    void setWheelBase(double wheel_base) override;

    void setMinSatellites(int min_satellites) override;

    void setBaselineOutlierPolicy(double soft_tolerance, double hard_tolerance,
                                  double soft_cap = -1.0, double hard_cap = -1.0);
    void setBaselineConsistencyChecks(double heading_limit_rad, double relax_speed);

    void setInitialTime(const ros::Time& time) override;

    void setResidualThreshold(double threshold);
    void setEncoderMismatchThreshold(double threshold);
    void setResidualWindowSize(std::size_t window_size);
    void setEncoderWindowSize(std::size_t window_size);
    void setResidualSoftResetMultiplier(double multiplier);
    void setResidualSurgeLimit(int limit);
    void setRecoveryReleaseRatio(double ratio);
    void setNonholonomicConstraint(bool enabled,
                                   double lateral_velocity_noise = 0.05,
                                   double speed_threshold = 0.05);
    void setWheelSlipConfig(bool enabled,
                            double min_speed_mps = 0.05,
                            double yaw_rate_diff_threshold = 0.35,
                            double yaw_rate_rel_threshold = 0.7,
                            double ema_alpha = 0.2,
                            double encoder_weight_floor_scale = 0.25,
                            double nonholonomic_noise_scale_max = 15.0);
    void setFallbackHeadingConfig(double process_noise, double noise_floor, double max_variance,
                                  double bias_alpha = 0.02,
                                  double velocity_heading_noise = 0.1,
                                  double velocity_speed_threshold = 0.05,
                                  double course_window_s = 1.5,
                                  double course_min_distance_m = 0.08,
                                  double course_gain = 0.35);
    void resetHealthMonitor();

	    void setRtkNisGate(double gate_2d, double gate_3d);
	    void setRtkSelectorConfig(int hold_frames, double switch_penalty);
		    void setRtkSingleStepPolicy(double jump_threshold_m,
		                                double jump_max_dt_s,
		                                double jump_hold_s,
		                                double jump_score_penalty);
		    void setRtkNoHeadingPositionNoiseScale(double min_scale, double max_scale);
		    void setRtkStationaryPositionNoiseScale(double speed_threshold,
		                                           double position_noise_scale,
		                                           double release_time_s);
			    void setRtkTurnInPlacePositionNoiseScale(double speed_threshold,
			                                             double yaw_rate_threshold,
			                                             double position_noise_scale);
			    void setBaselineCourseConsistency(double consistency_limit_rad);

			    void setBaselineDegradedFrontPenalty(double penalty);

			    void setRtkAntennaPreferenceConfig(bool enabled,
			                                       double ema_alpha,
			                                       double penalty_scale,
			                                       double residual_good_m,
			                                       double residual_bad_m,
			                                       double max_penalty);
			    RtkDebugInfo getRtkDebugInfo() const;

    void setVisionNisGate(double gate_3d);
    VisionDebugInfo getVisionDebugInfo() const;

    bool initializeFromRTKUnlocked(const RTKData& front_rtk, const RTKData& rear_rtk);

private:
    void updateWheelSlipStatus(const IMUData& imu, const EncoderData& encoder);

    Eigen::VectorXd x_;

    Eigen::MatrixXd P_;

    Eigen::MatrixXd Q_;

    Eigen::MatrixXd R_;

    double w0_, wi_;
    double w0_c_, wi_c_;

    Eigen::MatrixXd X_sig_;

    FusionResult latest_result_;

    mutable std::mutex data_mutex_;

    std::condition_variable data_cv_;

    double origin_latitude_;

    double origin_longitude_;

    GeographicLib::LocalCartesian geo_converter_;

    Eigen::Vector3d antenna_offset_;
    Eigen::Vector2d front_antenna_offset_;
    Eigen::Vector2d rear_antenna_offset_;
    bool rear_offset_initialized_;

    double rtk_baseline_distance_;
    double rtk_baseline_tolerance_;
    double rtk_baseline_soft_tolerance_;
    double rtk_baseline_hard_tolerance_;
    double baseline_soft_tolerance_cap_;
    double baseline_hard_tolerance_cap_;
    double baseline_heading_consistency_limit_;
    double baseline_heading_relax_speed_;
	    std::deque<double> baseline_history_;
	    double baseline_sum_;
	    std::size_t baseline_window_size_;
	    double baseline_window_average_;

	    double baseline_course_consistency_limit_rad_ = 0.0;
	    std::deque<std::pair<double, Eigen::Vector2d>> baseline_course_history_;

	    double baseline_degraded_front_score_penalty_ = 0.0;

	    bool rtk_preference_enabled_ = true;
	    double rtk_preference_ema_alpha_ = 0.05;
	    double rtk_preference_penalty_scale_ = 0.8;
	    double rtk_preference_residual_good_m_ = 0.08;
	    double rtk_preference_residual_bad_m_ = 0.30;
	    double rtk_preference_max_penalty_ = 2.0;
	    double front_bad_score_ = 0.0;
	    double rear_bad_score_ = 0.0;

    Eigen::Vector3d imu_lever_arm_;
    IMUData imu_bias_;

    ros::Time last_update_time_;

    bool initialized_;

    std::vector<CoordinateCache> coordinate_cache_;
    size_t cache_size_;

    double rtk_weight_;
    double imu_weight_;
    double encoder_weight_;
    double rtk_heading_weight_;

    double residual_threshold_;
    double encoder_mismatch_threshold_;
    double last_rtk_residual_;
    double last_encoder_mismatch_;
    HealthStatus health_status_;
    double rtk_residual_meter_score_;
    double encoder_mismatch_score_;
    double rtk_recent_residual_;
    bool rtk_health_initialized_;
    bool encoder_health_initialized_;
    double residual_ema_alpha_;
    double encoder_ema_alpha_;
    std::size_t residual_window_size_;
    std::size_t encoder_window_size_;
    int rtk_good_streak_;
    int encoder_good_streak_;
    int rtk_bad_streak_;
    int encoder_bad_streak_;
    int recovery_streak_required_;
    double recovery_release_ratio_;
    int residual_surge_count_;
    int residual_surge_limit_;
    double residual_soft_reset_multiplier_;

    double ukf_alpha_;
    double ukf_beta_;
    double ukf_kappa_;
    double ukf_lambda_;
    double ukf_gamma_;

    double wheel_base_;
    double half_wheel_base_;

    int min_satellites_;

    double imu_angular_velocity_noise_std_;
    double encoder_velocity_noise_std_;

    bool imu_measurement_active_;
    bool encoder_measurement_active_;
    bool fallback_heading_used_;

    bool fallback_heading_active_;
    bool fallback_heading_initialized_;
    double fallback_heading_estimate_;
    double fallback_heading_variance_;
    double fallback_heading_measurement_;
    double fallback_heading_measurement_variance_;
    ros::Time fallback_heading_stamp_;
    double fallback_heading_process_noise_;
    double fallback_heading_process_noise_base_;
    double fallback_heading_min_variance_;
    double fallback_heading_max_variance_;
    double fallback_heading_bias_alpha_;
    double fallback_velocity_heading_noise_;
    double fallback_velocity_speed_threshold_;
    double fallback_gyro_bias_estimate_;
    bool fallback_gyro_bias_initialized_;
    std::deque<std::pair<double, Eigen::Vector2d>> fallback_course_history_;
    double fallback_course_window_s_ = 1.5;
    double fallback_course_min_distance_m_ = 0.08;
    double fallback_course_gain_ = 0.35;

    bool nonholonomic_enabled_ = true;
    bool nonholonomic_active_ = false;
    double nonholonomic_lateral_velocity_noise_std_ = 0.05;
    double nonholonomic_speed_threshold_ = 0.05;

    bool wheel_slip_enabled_ = false;
    double wheel_slip_min_speed_mps_ = 0.05;
    double wheel_slip_yaw_rate_diff_threshold_ = 0.35;
    double wheel_slip_yaw_rate_rel_threshold_ = 0.7;
    double wheel_slip_ema_alpha_ = 0.2;
    double wheel_slip_encoder_weight_floor_scale_ = 0.25;
    double wheel_slip_nonholonomic_noise_scale_max_ = 15.0;
    double wheel_slip_score_ = 0.0;
    bool wheel_slip_detected_ = false;
    double wheel_slip_nonholonomic_noise_scale_ = 1.0;

	    double rtk_nis_gate_2d_;
	    double rtk_nis_gate_3d_;
	    int rtk_selector_hold_frames_;
	    double rtk_selector_switch_penalty_;
	    RtkMeasurementData::Mode rtk_selected_mode_;
	    int rtk_selected_mode_dim_;
	    int rtk_selected_mode_hold_counter_;
	    RtkDebugInfo last_rtk_debug_;

	    double rtk_single_step_jump_threshold_m_ = 0.3;
	    double rtk_single_step_jump_max_dt_s_ = 0.5;
	    double rtk_single_step_hold_s_ = 2.0;
	    double rtk_single_step_score_penalty_ = 0.8;
	    Eigen::Vector2d last_front_rtk_xy_ = Eigen::Vector2d::Zero();
	    Eigen::Vector2d last_rear_rtk_xy_ = Eigen::Vector2d::Zero();
	    ros::Time last_front_rtk_stamp_;
	    ros::Time last_rear_rtk_stamp_;
	    bool last_front_rtk_xy_valid_ = false;
	    bool last_rear_rtk_xy_valid_ = false;
	    double last_front_step_m_ = std::numeric_limits<double>::quiet_NaN();
	    double last_rear_step_m_ = std::numeric_limits<double>::quiet_NaN();
	    ros::Time front_jump_until_;
	    ros::Time rear_jump_until_;
		    bool front_jump_active_ = false;
		    bool rear_jump_active_ = false;

		    double rtk_no_heading_position_noise_scale_min_ = 2.5;
		    double rtk_no_heading_position_noise_scale_max_ = 200.0;

		    double rtk_stationary_speed_threshold_ = 0.0;
		    double rtk_stationary_position_noise_scale_ = 1.0;
		    double rtk_stationary_release_time_s_ = 0.0;
		    bool rtk_stationary_active_ = false;
		    ros::Time rtk_stationary_release_start_;

		    double rtk_turn_speed_threshold_ = 0.0;
		    double rtk_turn_yaw_rate_threshold_ = 0.0;
		    double rtk_turn_position_noise_scale_ = 1.0;

    double vision_nis_gate_3d_ = 11.34;
    VisionDebugInfo last_vision_debug_;

    double normalizeAngle(double angle);

    double calculateDistance(double x1, double y1, double x2, double y2);

    void generateSigmaPoints();

    void updateUkfWeights();

    void predictSigmaPoints(double dt, const IMUData& imu);

    void updateStateFromSigmaPoints();

    void WGS84ToLocal(double lat, double lon, double& x, double& y);

    void localToWGS84(double x, double y, double& lat, double& lon);

    Eigen::VectorXd calculateIMUAndEncoderMeasurement(const IMUData& imu, const EncoderData& encoder);
    void updateFallbackHeading(const RtkMeasurementData& rtk_measurement,
                               const RTKData& front_rtk,
                               const RTKData& rear_rtk,
                               const IMUData& imu,
                               const EncoderData& encoder);

    void adaptWeights(const RTKData& front_rtk, const RTKData& rear_rtk,
                      const IMUData& imu, const EncoderData& encoder);

    std::vector<RtkMeasurementData> buildRTKCandidateMeasurements(const RTKData& front_rtk, const RTKData& rear_rtk);
    bool computeRtkCandidateNis(const RtkMeasurementData& candidate, double& nis, double& pos_residual);
    RtkMeasurementData selectRtkMeasurement(const std::vector<RtkMeasurementData>& candidates);
    Eigen::Vector2d antennaMeasurementToBase(const Eigen::Vector2d& antenna_xy, bool is_front);
    Eigen::Vector2d rotateOffset(const Eigen::Vector2d& offset, double theta) const;
    void updateHealthStatus(double rtk_residual, bool rtk_used,
                            double encoder_mismatch, bool encoder_used);
    void recalcRearAntennaOffset();
    double computeRTKQuality(const RTKData& rtk) const;
    double computeEmaAlpha(std::size_t window_size) const;
    void resetHealthMonitorUnlocked();
    void softResetState(const Eigen::Vector2d& position, bool has_theta, double theta,
                        const ros::Time& stamp);
};

#endif
