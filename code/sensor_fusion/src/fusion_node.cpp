#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>
#include <cctype>
#include <boost/bind.hpp>
#include <cstddef>
#include <limits>
#include <atomic>

#include <json/json.h>
#include <yaml-cpp/yaml.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <curl/curl.h>
#include <Eigen/Dense>
#include <GeographicLib/LocalCartesian.hpp>
#include <nav_msgs/Odometry.h>
#include "sensor_fusion/VisionMeasurement.h"
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>

#include "sensor_fusion/rtk_parser.h"
#include "sensor_fusion/encoder_handler.h"
#include "sensor_fusion/ekf_fusion.h"
#include "sensor_fusion/ukf_fusion.h"
#include "sensor_fusion/network_sender.h"

#include <sensor_msgs/Imu.h>

std::unique_ptr<RTKParser> front_rtk_parser;
std::unique_ptr<RTKParser> rear_rtk_parser;
std::unique_ptr<EncoderHandler> encoder_handler;
std::unique_ptr<EKFFusion> ekf_fusion;
std::unique_ptr<UKFFusion> ukf_fusion;
std::unique_ptr<BaseFusion> fusion;
std::unique_ptr<NetworkSender> network_sender;
ros::Subscriber imu_subscriber;

bool use_hardware_sources = true;
bool use_network_sender = true;

int accuracy_threshold;
int min_satellites;
bool rtk_soft_quality_gate = false;
Json::Value global_params;

bool running = true;

std::mutex latest_data_mutex;
IMUData latest_imu_data;
bool latest_imu_valid = false;
EncoderData latest_encoder_data;
bool latest_encoder_valid = false;
bool fusion_time_initialized = false;

double zero_velocity_speed_threshold = 0.0;
double max_position_jump = 0.2;
double output_smoothing_alpha = 0.3;
double hold_speed_threshold = 0.05;
ros::Duration zero_velocity_hold_duration(0.5);
bool output_continuous_yaw = false;
bool output_stationary_hold_enabled = true;
double output_stationary_speed_threshold = 0.01;
double output_stationary_hold_time_s = 0.2;
bool output_pivot_hold_enabled = true;
double output_pivot_linear_threshold = 0.02;
double output_pivot_yaw_rate_threshold = 0.08;

bool straight_yaw_smoothing_enabled = true;
double straight_yaw_speed_threshold = 0.07;
double straight_yaw_rate_threshold = 0.08;
double straight_yaw_prev_weight = 0.85;
double straight_yaw_jump_limit_rad = 0.07;
double straight_yaw_severity_max = 1.0;
bool straight_yaw_course_pull_enabled = true;
double straight_yaw_course_gain = 0.65;
double straight_yaw_course_max_error_rad = 1.0;

bool straight_pos_smoothing_enabled = false;
double straight_pos_lateral_step_scale = 1.0;
double straight_pos_lateral_step_scale_degraded = 0.6;
double straight_pos_lateral_decay_degraded = 0.0;
double straight_pos_lateral_abs_limit_m = 0.0;
double straight_pos_dir_update_alpha = 0.06;
double straight_pos_dir_min_step_m = 0.01;
double straight_pos_dir_lock_distance_m = 0.0;
double straight_pos_frame_keep_s = 60.0;

struct OutputSmoothingState {
    FusionResult last_smoothed_result;
    FusionResult last_reliable_result;
    bool has_smoothed_result = false;
    bool has_reliable_result = false;
    double last_raw_x = 0.0;
    double last_raw_y = 0.0;
    bool has_last_raw_xy = false;
    double last_unwrapped_theta = 0.0;
    bool has_unwrapped_theta = false;
    double straight_dir_x = 1.0;
    double straight_dir_y = 0.0;
    double straight_origin_x = 0.0;
    double straight_origin_y = 0.0;
    bool has_straight_frame = false;
    double straight_travel_m = 0.0;
    bool straight_dir_locked = false;
    double last_straight_time = 0.0;
    bool has_last_straight_time = false;
    double stationary_start_time = 0.0;
    bool has_stationary_start_time = false;
    double stationary_anchor_x = 0.0;
    double stationary_anchor_y = 0.0;
    bool has_stationary_anchor = false;
    double stationary_anchor_sum_x = 0.0;
    double stationary_anchor_sum_y = 0.0;
    int stationary_anchor_count = 0;
    bool stationary_anchor_accumulating = false;
};

OutputSmoothingState raw_output_state;
OutputSmoothingState centered_output_state;
bool bootstrap_pending = true;
int bootstrap_frame_count = 0;
int bootstrap_required_frames = 5;
double bootstrap_max_residual = 0.5;
int initialization_required_samples = 3;
ros::Duration initialization_timeout(1.5);
std::vector<std::pair<RTKData, RTKData>> initialization_buffer;
ros::Time initialization_start_time;
bool initialization_active = false;
double origin_latitude = 0.0;
double origin_longitude = 0.0;
double rtk_baseline_distance_config = 0.0;
double rtk_baseline_soft_tolerance_config = 0.0;
double rtk_baseline_tolerance_config = 0.0;
double rtk_baseline_hard_tolerance_config = 0.0;
GeographicLib::LocalCartesian initialization_converter;
bool initialization_converter_ready = false;
double encoder_max_velocity_limit = 0.0;
double encoder_max_velocity_diff = 0.0;
double encoder_wheel_base_m = 0.0;
double encoder_ticks_per_rev = 65536.0;
double encoder_wheel_diameter_m = 0.0;
double encoder_ticks_velocity_min_dt_s = 0.0;
double encoder_ticks_velocity_max_dt_s = 0.5;

struct EncoderTicksVelocityState {
    bool initialized = false;
    double last_left_ticks = 0.0;
    double last_right_ticks = 0.0;
    ros::Time last_stamp;
};

EncoderTicksVelocityState encoder_ticks_velocity_state;

ros::Duration startup_warmup_duration(5.0);
bool startup_warmup_enabled = true;
bool startup_warmup_complete = false;
bool startup_reference_set = false;
ros::Time startup_reference_time;
std::atomic<bool> startup_imu_ready(false);
std::vector<std::pair<RTKData, RTKData>> startup_alignment_buffer;
int startup_alignment_max_samples = 20;

	struct VisionConstraintConfig {
	    bool enabled = false;
	    double confidence_threshold = 0.25;
	    double measurement_timeout = 0.3;
	    double lateral_gain = 0.8;
	    double heading_gain = 0.6;
	    double max_lateral_correction = 0.25;
	    double max_heading_correction = 0.1;
	    std::string topic = "/sensor_fusion/vision_measurement";
	    double position_noise = 0.04;
	    double position_noise_along = 2.0;
	    double heading_noise = 2.0 * M_PI / 180.0;
	    bool dynamic_noise = true;
	    double noise_scale_min = 0.8;
	    double noise_scale_max = 3.0;
	    double max_lateral_error = 0.35;
	    double max_heading_error = 15.0 * M_PI / 180.0;
	    double min_lookahead_for_heading = 0.25;
	    double nis_gate_3d = 11.34;
	    bool boost_when_rtk_degraded = true;
	    double degraded_noise_scale = 0.7;
	    double degraded_max_lateral_correction = 0.15;
	    double degraded_max_heading_correction = 6.0 * M_PI / 180.0;
	    bool invert_lateral = false;
	    bool invert_heading = false;
	    bool apply_camera_extrinsics = true;
	    bool apply_post_correction = false;
	    bool suppress_turning = true;
	    double turn_linear_thresh = 0.05;
	    double turn_angular_thresh = 0.3;
	    struct {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    } camera;
};

struct VisionMeasurementCache {
    sensor_fusion::VisionMeasurement measurement;
    ros::Time stamp;
    bool has_value = false;
};

VisionConstraintConfig vision_config;
VisionMeasurementCache vision_cache;
std::mutex vision_mutex;
ros::Subscriber vision_subscriber;
ros::Publisher odom_raw_publisher;
ros::Publisher odom_centered_publisher;
ros::Publisher odom_ukf_raw_publisher;
ros::Publisher odom_ukf_centered_input_publisher;
ros::Publisher odom_predict_publisher;
ros::Publisher rtk_debug_publisher;
ros::Publisher vision_stats_publisher;
ros::Timer vision_stats_timer;
std::string odom_frame_id = "odom";
std::string base_frame_id = "base_link";
std::string imu_topic = "/imu/data";
std::string predict_odom_topic = "/sensor_fusion/odom_predict";
bool publish_predict_odom = true;
double predict_odom_max_rate_hz = 100.0;

	struct VisionStatsCounters {
	    std::uint64_t received_total = 0;
	    std::uint64_t used_total = 0;
	    std::uint64_t drop_no_msg_total = 0;
	    std::uint64_t drop_timeout_total = 0;
	    std::uint64_t drop_low_conf_total = 0;
	    std::uint64_t drop_turning_total = 0;
	    std::uint64_t drop_amplitude_total = 0;
	    std::uint64_t drop_nis_total = 0;

	    std::uint64_t received = 0;
	    std::uint64_t used = 0;
	    std::uint64_t drop_no_msg = 0;
	    std::uint64_t drop_timeout = 0;
	    std::uint64_t drop_low_conf = 0;
	    std::uint64_t drop_turning = 0;
	    std::uint64_t drop_amplitude = 0;
	    std::uint64_t drop_nis = 0;

	    double last_age_s = -1.0;
	    double last_confidence = -1.0;
	    double last_nis = std::numeric_limits<double>::quiet_NaN();
	    bool last_gate_passed = false;
	    ros::Time last_stamp;
	};

VisionStatsCounters vision_stats;

namespace {

double normalizeAngleLocal(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

std::string jsonNumberOrNull(double value) {
    if (!std::isfinite(value)) {
        return "null";
    }
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

bool computeWheelVelocityFromEncoderTicks(double left_ticks,
                                         double right_ticks,
                                         const ros::Time& stamp,
                                         double ticks_per_rev,
                                         double wheel_diameter_m,
                                         double min_dt_s,
                                         double max_dt_s,
                                         double abs_speed_limit_mps,
                                         EncoderTicksVelocityState& state,
                                         double& out_left_mps,
                                         double& out_right_mps,
                                         double& out_dt_s) {
    out_left_mps = std::numeric_limits<double>::quiet_NaN();
    out_right_mps = std::numeric_limits<double>::quiet_NaN();
    out_dt_s = std::numeric_limits<double>::quiet_NaN();

    if (!std::isfinite(left_ticks) || !std::isfinite(right_ticks)) {
        return false;
    }
    if (!std::isfinite(ticks_per_rev) || ticks_per_rev <= 1e-6) {
        return false;
    }
    if (!std::isfinite(wheel_diameter_m) || wheel_diameter_m <= 1e-6) {
        return false;
    }
    if (stamp.isZero()) {
        return false;
    }

    if (!state.initialized) {
        state.initialized = true;
        state.last_left_ticks = left_ticks;
        state.last_right_ticks = right_ticks;
        state.last_stamp = stamp;
        return false;
    }

    double dt = (stamp - state.last_stamp).toSec();
    if (!std::isfinite(dt) || dt <= 0.0) {
        state.last_left_ticks = left_ticks;
        state.last_right_ticks = right_ticks;
        state.last_stamp = stamp;
        return false;
    }

    if (min_dt_s > 0.0 && dt < min_dt_s) {
        return false;
    }

    if (max_dt_s > 0.0 && dt > max_dt_s) {
        state.last_left_ticks = left_ticks;
        state.last_right_ticks = right_ticks;
        state.last_stamp = stamp;
        return false;
    }

    double d_left = left_ticks - state.last_left_ticks;
    double d_right = right_ticks - state.last_right_ticks;
    if (!std::isfinite(d_left) || !std::isfinite(d_right)) {
        state.last_left_ticks = left_ticks;
        state.last_right_ticks = right_ticks;
        state.last_stamp = stamp;
        return false;
    }

    const double wheel_circumference = wheel_diameter_m * std::acos(-1.0);
    const double left_dist_m = (d_left / ticks_per_rev) * wheel_circumference;
    const double right_dist_m = (d_right / ticks_per_rev) * wheel_circumference;
    const double left_mps = left_dist_m / dt;
    const double right_mps = right_dist_m / dt;

    if (!std::isfinite(left_mps) || !std::isfinite(right_mps)) {
        state.last_left_ticks = left_ticks;
        state.last_right_ticks = right_ticks;
        state.last_stamp = stamp;
        return false;
    }

    if (abs_speed_limit_mps > 0.0) {
        if (std::abs(left_mps) > abs_speed_limit_mps || std::abs(right_mps) > abs_speed_limit_mps) {
            state.last_left_ticks = left_ticks;
            state.last_right_ticks = right_ticks;
            state.last_stamp = stamp;
            return false;
        }
    }

    state.last_left_ticks = left_ticks;
    state.last_right_ticks = right_ticks;
    state.last_stamp = stamp;

    out_left_mps = left_mps;
    out_right_mps = right_mps;
    out_dt_s = dt;
    return true;
}

const char* rtkModeToString(UKFFusion::RtkMeasurementData::Mode mode) {
    switch (mode) {
        case UKFFusion::RtkMeasurementData::Mode::NONE:
            return "NONE";
        case UKFFusion::RtkMeasurementData::Mode::DUAL:
            return "DUAL";
        case UKFFusion::RtkMeasurementData::Mode::FRONT_ONLY:
            return "FRONT_ONLY";
        case UKFFusion::RtkMeasurementData::Mode::REAR_ONLY:
            return "REAR_ONLY";
        default:
            return "UNKNOWN";
    }
}

double unwrapAngle(double angle, double reference, bool has_reference) {
    if (!has_reference) {
        return angle;
    }
    double diff = normalizeAngleLocal(angle - reference);
    return reference + diff;
}

double sanitizedResidual(double value) {
    if (std::isfinite(value)) {
        return std::abs(value);
    }
    return 0.0;
}

bool baselineWithinTolerance(const RTKData& front, const RTKData& rear, double& out_distance) {
    if (!initialization_converter_ready) {
        out_distance = 0.0;
        return true;
    }
    double fx = 0.0, fy = 0.0, fz = 0.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    initialization_converter.Forward(front.latitude, front.longitude, 0.0, fx, fy, fz);
    initialization_converter.Forward(rear.latitude, rear.longitude, 0.0, rx, ry, rz);
    out_distance = std::hypot(fx - rx, fy - ry);
    double tolerance = rtk_baseline_soft_tolerance_config > 0.0
        ? rtk_baseline_soft_tolerance_config
        : std::max(rtk_baseline_tolerance_config, 0.2);
    return rtk_baseline_distance_config <= 0.0 ||
           std::abs(out_distance - rtk_baseline_distance_config) <= tolerance;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

void publishVisionStats(const ros::TimerEvent&) {
    if (!vision_stats_publisher) {
        return;
    }

    std_msgs::String msg;
    std::ostringstream oss;
    oss << "{"
        << "\"received\":" << vision_stats.received << ","
        << "\"used\":" << vision_stats.used << ","
        << "\"drop_no_msg\":" << vision_stats.drop_no_msg << ","
        << "\"drop_timeout\":" << vision_stats.drop_timeout << ","
        << "\"drop_low_conf\":" << vision_stats.drop_low_conf << ","
        << "\"drop_turning\":" << vision_stats.drop_turning << ","
        << "\"drop_amplitude\":" << vision_stats.drop_amplitude << ","
        << "\"drop_nis\":" << vision_stats.drop_nis << ","
        << "\"last_age_s\":" << vision_stats.last_age_s << ","
        << "\"last_confidence\":" << vision_stats.last_confidence << ","
        << "\"last_nis\":" << jsonNumberOrNull(vision_stats.last_nis) << ","
        << "\"last_gate_passed\":" << (vision_stats.last_gate_passed ? "true" : "false") << ","
        << "\"received_total\":" << vision_stats.received_total << ","
        << "\"used_total\":" << vision_stats.used_total << ","
        << "\"drop_no_msg_total\":" << vision_stats.drop_no_msg_total << ","
        << "\"drop_timeout_total\":" << vision_stats.drop_timeout_total << ","
        << "\"drop_low_conf_total\":" << vision_stats.drop_low_conf_total << ","
        << "\"drop_turning_total\":" << vision_stats.drop_turning_total << ","
        << "\"drop_amplitude_total\":" << vision_stats.drop_amplitude_total << ","
        << "\"drop_nis_total\":" << vision_stats.drop_nis_total
        << "}";
    msg.data = oss.str();
    vision_stats_publisher.publish(msg);

    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "Vision stats: recv=" << vision_stats.received
                              << " used=" << vision_stats.used
                              << " drop(no_msg/timeout/low_conf/turn/amp/nis)="
                              << vision_stats.drop_no_msg << "/"
                              << vision_stats.drop_timeout << "/"
                              << vision_stats.drop_low_conf << "/"
                              << vision_stats.drop_turning << "/"
                              << vision_stats.drop_amplitude << "/"
                              << vision_stats.drop_nis
                              << " last_age=" << vision_stats.last_age_s
                              << "s last_conf=" << vision_stats.last_confidence
                              << " last_nis=" << vision_stats.last_nis
                              << " gate=" << (vision_stats.last_gate_passed ? "pass" : "fail"));

    vision_stats.received = 0;
    vision_stats.used = 0;
    vision_stats.drop_no_msg = 0;
    vision_stats.drop_timeout = 0;
    vision_stats.drop_low_conf = 0;
    vision_stats.drop_turning = 0;
    vision_stats.drop_amplitude = 0;
    vision_stats.drop_nis = 0;
}

bool computeAverageRTKSamples(const std::vector<std::pair<RTKData, RTKData>>& samples,
                              const ros::Time& current_time,
                              RTKData& front_avg,
                              RTKData& rear_avg) {
    if (samples.empty()) {
        return false;
    }

    double front_x_sum = 0.0;
    double front_y_sum = 0.0;
    double rear_x_sum = 0.0;
    double rear_y_sum = 0.0;

    for (const auto& sample : samples) {
        double fx = 0.0, fy = 0.0, fz = 0.0;
        double rx = 0.0, ry = 0.0, rz = 0.0;
        if (initialization_converter_ready) {
            initialization_converter.Forward(sample.first.latitude, sample.first.longitude, 0.0, fx, fy, fz);
            initialization_converter.Forward(sample.second.latitude, sample.second.longitude, 0.0, rx, ry, rz);
        } else {
            fx = sample.first.latitude;
            fy = sample.first.longitude;
            rx = sample.second.latitude;
            ry = sample.second.longitude;
        }
        front_x_sum += fx;
        front_y_sum += fy;
        rear_x_sum += rx;
        rear_y_sum += ry;
    }

    const size_t count = samples.size();
    const double avg_front_x = front_x_sum / static_cast<double>(count);
    const double avg_front_y = front_y_sum / static_cast<double>(count);
    const double avg_rear_x = rear_x_sum / static_cast<double>(count);
    const double avg_rear_y = rear_y_sum / static_cast<double>(count);

    front_avg = samples.back().first;
    rear_avg = samples.back().second;
    front_avg.timestamp = current_time;
    rear_avg.timestamp = current_time;

    if (initialization_converter_ready) {
        double lat = 0.0, lon = 0.0, h = 0.0;
        initialization_converter.Reverse(avg_front_x, avg_front_y, 0.0, lat, lon, h);
        front_avg.latitude = lat;
        front_avg.longitude = lon;
        initialization_converter.Reverse(avg_rear_x, avg_rear_y, 0.0, lat, lon, h);
        rear_avg.latitude = lat;
        rear_avg.longitude = lon;
    } else {
        front_avg.latitude = avg_front_x;
        front_avg.longitude = avg_front_y;
        rear_avg.latitude = avg_rear_x;
        rear_avg.longitude = avg_rear_y;
    }

    front_avg.raw_valid = true;
    front_avg.valid = true;
    rear_avg.raw_valid = true;
    rear_avg.valid = true;
    return true;
}

bool finalizeInitializationFromBuffer(const ros::Time& current_time) {
    RTKData front_avg;
    RTKData rear_avg;
    if (!computeAverageRTKSamples(initialization_buffer, current_time, front_avg, rear_avg)) {
        return false;
    }

    bool success = fusion->initializeFromRTK(front_avg, rear_avg);
    if (!success) {
        ROS_WARN_STREAM("Initialization failed: fusion rejected the initial RTK samples.");
    } else {
        std::cout << "Initialized using " << initialization_buffer.size() << " RTK samples." << std::endl;
    }
    initialization_buffer.clear();
    initialization_start_time = ros::Time();
    initialization_active = false;
    return success;
}

FusionResult stabilizeOutput(const FusionResult& raw_result,
                             bool rtk_reliable,
                             bool yaw_reliable,
                             const UKFFusion::HealthStatus* health_status,
                             double encoder_speed,
                             bool encoder_valid,
                             bool encoder_stationary,
                             bool force_position_hold,
                             double baseline_severity,
                             OutputSmoothingState& state) {
    FusionResult stabilized = raw_result;

    double raw_dx_step = 0.0;
    double raw_dy_step = 0.0;
    double raw_step_dist = 0.0;
    bool raw_step_ok = false;
    if (state.has_last_raw_xy) {
        raw_dx_step = raw_result.x - state.last_raw_x;
        raw_dy_step = raw_result.y - state.last_raw_y;
        raw_step_dist = std::hypot(raw_dx_step, raw_dy_step);
        raw_step_ok = std::isfinite(raw_step_dist);
    }
    state.last_raw_x = raw_result.x;
    state.last_raw_y = raw_result.y;
    state.has_last_raw_xy = std::isfinite(raw_result.x) && std::isfinite(raw_result.y);

    double planar_speed = std::sqrt(raw_result.vx * raw_result.vx + raw_result.vy * raw_result.vy);
    double effective_encoder_speed = encoder_valid
        ? std::abs(encoder_speed)
        : planar_speed;
    bool encoder_slow = effective_encoder_speed <= hold_speed_threshold;
    bool filter_slow = planar_speed <= hold_speed_threshold;

    bool output_stationary_hold = false;
    if (output_stationary_hold_enabled && state.has_smoothed_result &&
        std::isfinite(raw_result.timestamp)) {
        if (force_position_hold) {
            output_stationary_hold = true;
            if (!state.has_stationary_start_time) {
                state.stationary_start_time = raw_result.timestamp;
                state.has_stationary_start_time = true;
            }
        } else if (encoder_stationary) {
            if (!state.has_stationary_start_time) {
                state.stationary_start_time = raw_result.timestamp;
                state.has_stationary_start_time = true;
            }
            double hold_s = output_stationary_hold_time_s;
            if (!std::isfinite(hold_s) || hold_s < 0.0) {
                hold_s = 0.0;
            }
            if (hold_s <= 1e-6 || (raw_result.timestamp - state.stationary_start_time) >= hold_s) {
                output_stationary_hold = true;
            }
        } else {
            state.has_stationary_start_time = false;
        }
    } else {
        state.has_stationary_start_time = false;
    }

    if (encoder_stationary && state.has_smoothed_result &&
        std::isfinite(state.last_smoothed_result.x) && std::isfinite(state.last_smoothed_result.y)) {
        if (!state.stationary_anchor_accumulating) {
            state.stationary_anchor_sum_x = 0.0;
            state.stationary_anchor_sum_y = 0.0;
            state.stationary_anchor_count = 0;
            state.stationary_anchor_accumulating = true;
        }
        state.stationary_anchor_sum_x += state.last_smoothed_result.x;
        state.stationary_anchor_sum_y += state.last_smoothed_result.y;
        state.stationary_anchor_count += 1;

        if (state.stationary_anchor_count > 0) {
            state.stationary_anchor_x = state.stationary_anchor_sum_x /
                                        static_cast<double>(state.stationary_anchor_count);
            state.stationary_anchor_y = state.stationary_anchor_sum_y /
                                        static_cast<double>(state.stationary_anchor_count);
            state.has_stationary_anchor = true;
        }
    } else {
        state.stationary_anchor_accumulating = false;
        state.stationary_anchor_sum_x = 0.0;
        state.stationary_anchor_sum_y = 0.0;
        state.stationary_anchor_count = 0;
    }

    double raw_unwrapped_theta = unwrapAngle(raw_result.theta, state.last_unwrapped_theta, state.has_unwrapped_theta);
    double angular_rate = std::abs(raw_result.vtheta);

    double residual_metric = 0.0;
    if (health_status) {
        residual_metric = std::max(sanitizedResidual(health_status->rtk_residual_avg),
                                   sanitizedResidual(health_status->rtk_recent_residual));
    }
    double residual_reference = std::max(bootstrap_max_residual, 0.1);
    double severity = residual_reference > 1e-6
        ? std::clamp(residual_metric / residual_reference, 0.0, 10.0)
        : 0.0;
    if (std::isfinite(baseline_severity)) {
        severity = std::max(severity, std::clamp(baseline_severity, 0.0, 10.0));
    }

	    bool straight_motion = false;
	    if ((straight_yaw_smoothing_enabled || straight_pos_smoothing_enabled) && state.has_smoothed_result) {
	        if (std::isfinite(effective_encoder_speed) && std::isfinite(angular_rate)) {
	            straight_motion = (effective_encoder_speed >= straight_yaw_speed_threshold) &&
	                              (angular_rate <= straight_yaw_rate_threshold);
	        }
	    }
	    const bool straight_pos_hold_low_speed =
	        straight_pos_smoothing_enabled &&
	        state.has_smoothed_result &&
	        state.has_straight_frame &&
	        std::isfinite(angular_rate) &&
	        (angular_rate <= straight_yaw_rate_threshold);
	    const bool straight_pos_active = straight_pos_smoothing_enabled && (straight_motion || straight_pos_hold_low_speed);
	    bool straight_yaw_motion = straight_yaw_smoothing_enabled && straight_motion;
	    if (straight_yaw_motion && std::isfinite(severity)) {
	        if (severity > straight_yaw_severity_max && yaw_reliable) {
            straight_yaw_motion = false;
        }
    }

    if (state.has_smoothed_result) {
        double dx_step = raw_result.x - state.last_smoothed_result.x;
        double dy_step = raw_result.y - state.last_smoothed_result.y;

        double step_dist = std::hypot(dx_step, dy_step);
        if (max_position_jump > 0.0 && std::isfinite(step_dist) && step_dist > max_position_jump) {
            const double scale = max_position_jump / std::max(step_dist, 1e-12);
            dx_step *= scale;
            dy_step *= scale;
        }

	        if (straight_pos_active) {
	            if (!state.has_straight_frame) {
                if (state.has_stationary_anchor &&
                    std::isfinite(state.stationary_anchor_x) &&
                    std::isfinite(state.stationary_anchor_y)) {
                    state.straight_origin_x = state.stationary_anchor_x;
                    state.straight_origin_y = state.stationary_anchor_y;
                } else {
                    state.straight_origin_x = state.last_smoothed_result.x;
                    state.straight_origin_y = state.last_smoothed_result.y;
                }
                state.straight_travel_m = 0.0;
                state.straight_dir_locked = false;
                if (raw_step_ok && raw_step_dist >= std::max(straight_pos_dir_min_step_m, 0.0)) {
                    const double inv = 1.0 / std::max(raw_step_dist, 1e-9);
                    state.straight_dir_x = raw_dx_step * inv;
                    state.straight_dir_y = raw_dy_step * inv;
                } else {
                    state.straight_dir_x = std::cos(state.last_smoothed_result.theta);
                    state.straight_dir_y = std::sin(state.last_smoothed_result.theta);
                }
                double n = std::hypot(state.straight_dir_x, state.straight_dir_y);
                if (!std::isfinite(n) || n < 1e-6) {
                    state.straight_dir_x = 1.0;
                    state.straight_dir_y = 0.0;
                } else {
                    state.straight_dir_x /= n;
                    state.straight_dir_y /= n;
                }
                state.has_straight_frame = true;
            }
            state.last_straight_time = raw_result.timestamp;
            state.has_last_straight_time = true;

	            const bool lock_enabled =
	                std::isfinite(straight_pos_dir_lock_distance_m) && straight_pos_dir_lock_distance_m > 0.0;
	            if (straight_motion) {
	                const bool dir_locked = lock_enabled && state.straight_dir_locked;
	                if (!dir_locked && raw_step_ok && raw_step_dist >= std::max(straight_pos_dir_min_step_m, 0.0)) {
	                    double ux = raw_dx_step / std::max(raw_step_dist, 1e-9);
	                    double uy = raw_dy_step / std::max(raw_step_dist, 1e-9);
	                    if (ux * state.straight_dir_x + uy * state.straight_dir_y < 0.0) {
	                        ux = -ux;
	                        uy = -uy;
	                    }
	                    const double alpha_raw = std::clamp(straight_pos_dir_update_alpha, 0.0, 1.0);
	                    const double alpha = alpha_raw;
	                    double nx = (1.0 - alpha) * state.straight_dir_x + alpha * ux;
	                    double ny = (1.0 - alpha) * state.straight_dir_y + alpha * uy;
	                    double nn = std::hypot(nx, ny);
	                    if (std::isfinite(nn) && nn > 1e-6) {
	                        state.straight_dir_x = nx / nn;
	                        state.straight_dir_y = ny / nn;
	                    }
	                }
	                if (raw_step_ok && std::isfinite(raw_step_dist) && raw_step_dist > 0.0) {
	                    state.straight_travel_m += raw_step_dist;
	                }
	                if (lock_enabled && !state.straight_dir_locked &&
	                    std::isfinite(state.straight_travel_m) &&
	                    state.straight_travel_m >= straight_pos_dir_lock_distance_m) {
	                    const double dx_dir = raw_result.x - state.straight_origin_x;
	                    const double dy_dir = raw_result.y - state.straight_origin_y;
	                    const double nn = std::hypot(dx_dir, dy_dir);
	                    if (std::isfinite(nn) && nn > 1e-6) {
	                        double ux = dx_dir / nn;
	                        double uy = dy_dir / nn;
	                        if (ux * state.straight_dir_x + uy * state.straight_dir_y < 0.0) {
	                            ux = -ux;
	                            uy = -uy;
	                        }
	                        state.straight_dir_x = ux;
	                        state.straight_dir_y = uy;
	                    }
	                    state.straight_dir_locked = true;
	                }
	            }

            const double vx_lat = -state.straight_dir_y;
            const double vy_lat = state.straight_dir_x;
            const double dalong = dx_step * state.straight_dir_x + dy_step * state.straight_dir_y;
            double dlat = dx_step * vx_lat + dy_step * vy_lat;
            const bool pos_reliable = rtk_reliable;
            const double lateral_scale = pos_reliable ? straight_pos_lateral_step_scale
                                                      : straight_pos_lateral_step_scale_degraded;
            dlat *= std::clamp(lateral_scale, 0.0, 1.0);
            if (straight_pos_lateral_abs_limit_m > 0.0) {
                const double lim = std::max(straight_pos_lateral_abs_limit_m, 0.0);
                const double rel_prev_x = state.last_smoothed_result.x - state.straight_origin_x;
                const double rel_prev_y = state.last_smoothed_result.y - state.straight_origin_y;
                const double lat_prev = rel_prev_x * vx_lat + rel_prev_y * vy_lat;
                const double lat_next = std::clamp(lat_prev + dlat, -lim, lim);
                dlat = lat_next - lat_prev;
            }
            dx_step = dalong * state.straight_dir_x + dlat * vx_lat;
            dy_step = dalong * state.straight_dir_y + dlat * vy_lat;

            if (!pos_reliable && straight_pos_lateral_decay_degraded > 0.0) {
                const double decay = std::clamp(straight_pos_lateral_decay_degraded, 0.0, 1.0);
                const double rel_x = (state.last_smoothed_result.x + dx_step) - state.straight_origin_x;
                const double rel_y = (state.last_smoothed_result.y + dy_step) - state.straight_origin_y;
                const double lat_next = rel_x * vx_lat + rel_y * vy_lat;
                const double dlat_correction = -decay * lat_next;
                dx_step += dlat_correction * vx_lat;
                dy_step += dlat_correction * vy_lat;
            }

        } else if (straight_pos_smoothing_enabled) {
            if (state.has_straight_frame && state.has_last_straight_time &&
                std::isfinite(raw_result.timestamp) && std::isfinite(state.last_straight_time)) {
                const double dt = raw_result.timestamp - state.last_straight_time;
                if (dt > std::max(straight_pos_frame_keep_s, 0.0)) {
                    state.has_straight_frame = false;
                    state.has_last_straight_time = false;
                }
            } else {
                state.has_straight_frame = false;
                state.has_last_straight_time = false;
            }
        }

        step_dist = std::hypot(dx_step, dy_step);
        if (max_position_jump > 0.0 && std::isfinite(step_dist) && step_dist > max_position_jump) {
            const double scale = max_position_jump / std::max(step_dist, 1e-12);
            dx_step *= scale;
            dy_step *= scale;
        }

        const double next_x = state.last_smoothed_result.x + dx_step;
        const double next_y = state.last_smoothed_result.y + dy_step;

        stabilized.x = next_x;
        stabilized.y = next_y;
    }

    if (!rtk_reliable && state.has_smoothed_result) {
        double alpha = std::clamp(output_smoothing_alpha, 0.0, 1.0);
        if (severity > 1.0) {
            double damp = 1.0 / (1.0 + 0.45 * (severity - 1.0));
            alpha = std::clamp(alpha * damp, 0.04, 0.32);
        } else {
            alpha = std::clamp(alpha * (0.75 + 0.25 * severity), 0.05, 0.4);
        }
        if (angular_rate > 0.25) {
            double relief = std::clamp((angular_rate - 0.25) / 0.5, 0.0, 1.0);
            alpha = std::clamp(alpha + 0.15 * relief, 0.06, 0.55);
        }
        stabilized.x = alpha * stabilized.x + (1.0 - alpha) * state.last_smoothed_result.x;
        stabilized.y = alpha * stabilized.y + (1.0 - alpha) * state.last_smoothed_result.y;
    }

    bool allow_hold = !rtk_reliable && state.has_smoothed_result && encoder_slow && filter_slow;
    if (allow_hold && state.has_smoothed_result) {
        double gain = 1.0 / (1.0 + severity);
        gain = std::clamp(gain, 0.05, 0.6);

        double dx = stabilized.x - state.last_smoothed_result.x;
        double dy = stabilized.y - state.last_smoothed_result.y;
        stabilized.x = state.last_smoothed_result.x + gain * dx;
        stabilized.y = state.last_smoothed_result.y + gain * dy;

        stabilized.vx *= gain;
        stabilized.vy *= gain;
        stabilized.vtheta *= gain;
    }

    double previous_unwrapped_theta = state.has_smoothed_result
        ? unwrapAngle(state.last_smoothed_result.theta, raw_unwrapped_theta, true)
        : raw_unwrapped_theta;

    double corrected_raw_unwrapped_theta = raw_unwrapped_theta;
    if (straight_motion && straight_yaw_course_pull_enabled && !yaw_reliable) {
        double dir_x = std::numeric_limits<double>::quiet_NaN();
        double dir_y = std::numeric_limits<double>::quiet_NaN();
        bool have_dir = false;

        if (state.has_straight_frame &&
            std::isfinite(state.straight_dir_x) &&
            std::isfinite(state.straight_dir_y)) {
            const double norm = std::hypot(state.straight_dir_x, state.straight_dir_y);
            if (std::isfinite(norm) && norm > 1e-6) {
                dir_x = state.straight_dir_x / norm;
                dir_y = state.straight_dir_y / norm;
                have_dir = true;
            }
        }

        if (!have_dir && raw_step_ok &&
            std::isfinite(raw_dx_step) && std::isfinite(raw_dy_step) &&
            raw_step_dist >= std::max(straight_pos_dir_min_step_m, 1e-3)) {
            dir_x = raw_dx_step / std::max(raw_step_dist, 1e-6);
            dir_y = raw_dy_step / std::max(raw_step_dist, 1e-6);
            have_dir = true;
        }

        if (have_dir) {
            const double course = std::atan2(dir_y, dir_x);
            double err = normalizeAngleLocal(course - corrected_raw_unwrapped_theta);
            if (err > M_PI_2) {
                err -= M_PI;
            } else if (err < -M_PI_2) {
                err += M_PI;
            }
            if (std::isfinite(err) && std::abs(err) <= std::max(straight_yaw_course_max_error_rad, 0.0)) {
                const double gain = std::clamp(straight_yaw_course_gain, 0.0, 1.0);
                corrected_raw_unwrapped_theta = corrected_raw_unwrapped_theta + gain * err;
            }
        }
    }

    double yaw_jump_base = 0.12;
    double yaw_jump_rate_gain = 0.45;
    double yaw_jump_max = 0.5;
    double yaw_jump_limit = std::clamp(yaw_jump_base + yaw_jump_rate_gain * angular_rate,
                                       yaw_jump_base, yaw_jump_max);
    if (yaw_reliable && straight_yaw_motion && straight_yaw_jump_limit_rad > 0.0) {
        yaw_jump_limit = std::min(yaw_jump_limit, straight_yaw_jump_limit_rad);
    }
    double limited_raw_unwrapped_theta = corrected_raw_unwrapped_theta;
    if (state.has_smoothed_result) {
        double delta = corrected_raw_unwrapped_theta - previous_unwrapped_theta;
        delta = std::clamp(delta, -yaw_jump_limit, yaw_jump_limit);
        limited_raw_unwrapped_theta = previous_unwrapped_theta + delta;
    }

    double theta_prev_weight = 0.0;
    if (state.has_smoothed_result) {
        if (!yaw_reliable || allow_hold) {
            theta_prev_weight = std::clamp(0.55 + 0.18 * severity, 0.55, 0.95);
        } else {
            theta_prev_weight = std::clamp(0.2 + output_smoothing_alpha * 0.5, 0.12, 0.55);
        }
        if (yaw_reliable && !allow_hold && straight_yaw_motion) {
            theta_prev_weight = std::max(theta_prev_weight, std::clamp(straight_yaw_prev_weight, 0.12, 0.995));
        }
        if (angular_rate > 0.25) {
            double relief = std::clamp((angular_rate - 0.25) / 0.5, 0.0, 1.0);
            theta_prev_weight *= std::clamp(1.0 - 0.6 * relief, 0.25, 1.0);
        }
        double prev_weight_max = 0.95;
        if (straight_yaw_motion) {
            prev_weight_max = 0.995;
        }
        theta_prev_weight = std::clamp(theta_prev_weight, 0.12, prev_weight_max);
    }

    double blended_unwrapped_theta = theta_prev_weight * previous_unwrapped_theta +
                                     (1.0 - theta_prev_weight) * limited_raw_unwrapped_theta;

    double stabilized_theta = output_continuous_yaw
        ? blended_unwrapped_theta
        : normalizeAngleLocal(blended_unwrapped_theta);
    stabilized.theta = stabilized_theta;

    state.last_unwrapped_theta = blended_unwrapped_theta;
    state.has_unwrapped_theta = true;

    if (output_stationary_hold && state.has_smoothed_result) {
        stabilized.x = state.last_smoothed_result.x;
        stabilized.y = state.last_smoothed_result.y;
        stabilized.vx = 0.0;
        stabilized.vy = 0.0;
    }

    state.last_smoothed_result = stabilized;
    state.has_smoothed_result = true;

    if (rtk_reliable) {
        state.last_reliable_result = stabilized;
        state.has_reliable_result = true;
    }

    return stabilized;
}

FusionResult stabilizeOutput(const FusionResult& raw_result,
                             bool rtk_reliable,
                             bool yaw_reliable,
                             const UKFFusion::HealthStatus* health_status,
                             double encoder_speed,
                             bool encoder_valid,
                             bool encoder_stationary,
                             bool force_position_hold,
                             double baseline_severity) {
    return stabilizeOutput(raw_result,
                           rtk_reliable,
                           yaw_reliable,
                           health_status,
                           encoder_speed,
                           encoder_valid,
                           encoder_stationary,
                           force_position_hold,
                           baseline_severity,
                           raw_output_state);
}

FusionResult stabilizeCenteredOutput(const FusionResult& raw_result,
                                     bool rtk_reliable,
                                     bool yaw_reliable,
                                     const UKFFusion::HealthStatus* health_status,
                                     double encoder_speed,
                                     bool encoder_valid,
                                     bool encoder_stationary,
                                     bool force_position_hold,
                                     double baseline_severity) {
    return stabilizeOutput(raw_result,
                           rtk_reliable,
                           yaw_reliable,
                           health_status,
                           encoder_speed,
                           encoder_valid,
                           encoder_stationary,
                           force_position_hold,
                           baseline_severity,
                           centered_output_state);
}

void visionMeasurementCallback(const sensor_fusion::VisionMeasurement::ConstPtr& msg) {
    if (!vision_config.enabled) {
        return;
    }
    std::lock_guard<std::mutex> lock(vision_mutex);
    vision_cache.measurement = *msg;
    vision_cache.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    vision_cache.has_value = true;

    vision_stats.received_total += 1;
    vision_stats.received += 1;
    vision_stats.last_confidence = msg->confidence;
    vision_stats.last_stamp = vision_cache.stamp;
}

bool fetchVisionMeasurement(sensor_fusion::VisionMeasurement& measurement_out) {
    if (!vision_config.enabled) {
        return false;
    }
    std::lock_guard<std::mutex> lock(vision_mutex);
    if (!vision_cache.has_value) {
        vision_stats.drop_no_msg_total += 1;
        vision_stats.drop_no_msg += 1;
        return false;
    }
    ros::Time reference_stamp = vision_cache.stamp.isZero() ? ros::Time::now() : vision_cache.stamp;
    double age = (ros::Time::now() - reference_stamp).toSec();
    if (vision_config.measurement_timeout > 0.0 && age > vision_config.measurement_timeout) {
        vision_stats.drop_timeout_total += 1;
        vision_stats.drop_timeout += 1;
        vision_stats.last_age_s = age;
        vision_stats.last_confidence = vision_cache.measurement.confidence;
        vision_stats.last_stamp = reference_stamp;
        return false;
    }
    if (vision_cache.measurement.confidence < vision_config.confidence_threshold) {
        vision_stats.drop_low_conf_total += 1;
        vision_stats.drop_low_conf += 1;
        vision_stats.last_age_s = age;
        vision_stats.last_confidence = vision_cache.measurement.confidence;
        vision_stats.last_stamp = reference_stamp;
        return false;
    }
    measurement_out = vision_cache.measurement;
    vision_stats.last_age_s = age;
    vision_stats.last_confidence = vision_cache.measurement.confidence;
    vision_stats.last_stamp = reference_stamp;
    return true;
}

geometry_msgs::Quaternion yawToQuaternion(double yaw) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

void publishOdometryMessage(const FusionResult& src, ros::Publisher& pub, const ros::Time& stamp) {
    if (!pub) {
        return;
    }
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = base_frame_id;
    odom.pose.pose.position.x = src.x;
    odom.pose.pose.position.y = src.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yawToQuaternion(src.theta);
    odom.twist.twist.linear.x = src.vx;
    odom.twist.twist.linear.y = src.vy;
    odom.twist.twist.angular.z = src.vtheta;
    pub.publish(odom);
}

FusionResult applyVisionConstraint(const FusionResult& stabilized_result) {
    if (!vision_config.enabled) {
        return stabilized_result;
    }
    sensor_fusion::VisionMeasurement measurement;
    if (!fetchVisionMeasurement(measurement)) {
        return stabilized_result;
    }

    FusionResult constrained = stabilized_result;
    vision_stats.used_total += 1;
    vision_stats.used += 1;
    double lateral_correction = -vision_config.lateral_gain * measurement.lateral_error;
    lateral_correction = std::clamp(lateral_correction,
                                    -vision_config.max_lateral_correction,
                                    vision_config.max_lateral_correction);

    double heading_correction = -vision_config.heading_gain * measurement.heading_error;
    heading_correction = std::clamp(heading_correction,
                                    -vision_config.max_heading_correction,
                                    vision_config.max_heading_correction);

    double sin_theta = std::sin(constrained.theta);
    double cos_theta = std::cos(constrained.theta);
    constrained.x += lateral_correction * (-sin_theta);
    constrained.y += lateral_correction * cos_theta;
    constrained.theta = normalizeAngleLocal(constrained.theta + heading_correction);
    return constrained;
}

bool computeVisionPoseMeasurement(const FusionResult& reference,
                                  bool rtk_degraded,
                                  VisionPoseMeasurement& measurement_out,
                                  double& lateral_correction_out,
                                  double& heading_correction_out) {
    if (!vision_config.enabled) {
        return false;
    }
    double speed = std::hypot(reference.vx, reference.vy);
    if (vision_config.suppress_turning &&
        speed < vision_config.turn_linear_thresh &&
        std::abs(reference.vtheta) > vision_config.turn_angular_thresh) {
        vision_stats.drop_turning_total += 1;
        vision_stats.drop_turning += 1;
        return false;
    }
    sensor_fusion::VisionMeasurement measurement;
    if (!fetchVisionMeasurement(measurement)) {
        return false;
    }

    double adjusted_lateral = static_cast<double>(measurement.lateral_error);
    double adjusted_heading = static_cast<double>(measurement.heading_error);
    if (vision_config.invert_lateral) {
        adjusted_lateral = -adjusted_lateral;
    }
    if (vision_config.invert_heading) {
        adjusted_heading = -adjusted_heading;
    }
    if (vision_config.apply_camera_extrinsics) {
        adjusted_lateral += vision_config.camera.y;
        adjusted_heading += vision_config.camera.yaw;
    }

    bool heading_applicable = true;
    if (vision_config.min_lookahead_for_heading > 0.0 &&
        static_cast<double>(measurement.lookahead_distance) < vision_config.min_lookahead_for_heading) {
        heading_applicable = false;
        adjusted_heading = 0.0;
    }

    if (vision_config.max_lateral_error > 0.0 &&
        std::abs(adjusted_lateral) > vision_config.max_lateral_error) {
        vision_stats.drop_amplitude_total += 1;
        vision_stats.drop_amplitude += 1;
        return false;
    }
    if (heading_applicable && vision_config.max_heading_error > 0.0 &&
        std::abs(adjusted_heading) > vision_config.max_heading_error) {
        vision_stats.drop_amplitude_total += 1;
        vision_stats.drop_amplitude += 1;
        return false;
    }

    double max_lateral_correction = vision_config.max_lateral_correction;
    double max_heading_correction = vision_config.max_heading_correction;
    if (rtk_degraded && vision_config.boost_when_rtk_degraded) {
        if (vision_config.degraded_max_lateral_correction > 0.0) {
            max_lateral_correction = vision_config.degraded_max_lateral_correction;
        }
        if (vision_config.degraded_max_heading_correction > 0.0) {
            max_heading_correction = vision_config.degraded_max_heading_correction;
        }
    }

    double lateral_correction = -vision_config.lateral_gain * adjusted_lateral;
    lateral_correction = std::clamp(lateral_correction,
                                    -max_lateral_correction,
                                    max_lateral_correction);

    double heading_correction = -vision_config.heading_gain * adjusted_heading;
    if (!heading_applicable) {
        heading_correction = 0.0;
    }
    heading_correction = std::clamp(heading_correction,
                                    -max_heading_correction,
                                    max_heading_correction);

    double sin_theta = std::sin(reference.theta);
    double cos_theta = std::cos(reference.theta);

    measurement_out.x = reference.x + lateral_correction * (-sin_theta);
    measurement_out.y = reference.y + lateral_correction * cos_theta;
    measurement_out.theta = normalizeAngleLocal(reference.theta + heading_correction);
    double scale = 1.0;
    if (vision_config.dynamic_noise) {
        double conf = std::max(static_cast<double>(measurement.confidence), 1e-3);
        scale = 1.0 / conf;
        scale = std::clamp(scale, vision_config.noise_scale_min, vision_config.noise_scale_max);
    }
    if (rtk_degraded && vision_config.boost_when_rtk_degraded) {
        if (std::isfinite(vision_config.degraded_noise_scale) && vision_config.degraded_noise_scale > 0.0) {
            scale = std::max(0.1, scale * vision_config.degraded_noise_scale);
        }
    }

    const double pos_std_lat = std::max(vision_config.position_noise, 1e-6) * scale;
    const double pos_std_along = std::max(vision_config.position_noise_along, vision_config.position_noise) * scale;
    const double heading_std = std::max(vision_config.heading_noise, 1e-6) * scale;

    measurement_out.position_variance = std::max(pos_std_lat * pos_std_lat, 1e-6);
    measurement_out.position_variance_along = std::max(pos_std_along * pos_std_along,
                                                       measurement_out.position_variance);
    measurement_out.heading_variance = std::max(heading_std * heading_std, 1e-6);
    measurement_out.stamp = measurement.header.stamp.isZero() ? ros::Time::now() : measurement.header.stamp;
    measurement_out.valid = true;

    lateral_correction_out = lateral_correction;
    heading_correction_out = heading_correction;

    return true;
}

void configureVisionConstraint(const Json::Value& params) {
    if (!params.isMember("vision_constraint")) {
        vision_config.enabled = false;
        return;
    }
    const auto& cfg = params["vision_constraint"];
    vision_config.enabled = cfg.get("enabled", false).asBool();
    vision_config.topic = cfg.get("measurement_topic", vision_config.topic).asString();
    vision_config.confidence_threshold = cfg.get("confidence_threshold", vision_config.confidence_threshold).asDouble();
    vision_config.measurement_timeout = cfg.get("measurement_timeout", vision_config.measurement_timeout).asDouble();
    vision_config.lateral_gain = cfg.get("lateral_gain", vision_config.lateral_gain).asDouble();
    vision_config.heading_gain = cfg.get("heading_gain", vision_config.heading_gain).asDouble();
    vision_config.max_lateral_correction = cfg.get("max_lateral_correction", vision_config.max_lateral_correction).asDouble();
    double max_heading_deg = cfg.get("max_heading_correction_deg", 10.0).asDouble();
    vision_config.max_heading_correction = max_heading_deg * M_PI / 180.0;
    if (cfg.isMember("max_lateral_error")) {
        vision_config.max_lateral_error = cfg.get("max_lateral_error", vision_config.max_lateral_error).asDouble();
    }
    if (cfg.isMember("max_heading_error_deg")) {
        double max_err_deg = cfg.get("max_heading_error_deg", vision_config.max_heading_error * 180.0 / M_PI).asDouble();
        vision_config.max_heading_error = deg2rad(max_err_deg);
    }
    if (cfg.isMember("min_lookahead_for_heading")) {
        vision_config.min_lookahead_for_heading =
            cfg.get("min_lookahead_for_heading", vision_config.min_lookahead_for_heading).asDouble();
    }

    vision_config.dynamic_noise = cfg.get("dynamic_noise", vision_config.dynamic_noise).asBool();
    if (cfg.isMember("noise_scale_min")) {
        vision_config.noise_scale_min = std::max(0.1, cfg.get("noise_scale_min", vision_config.noise_scale_min).asDouble());
    }
    if (cfg.isMember("noise_scale_max")) {
        vision_config.noise_scale_max = std::max(vision_config.noise_scale_min,
                                                 cfg.get("noise_scale_max", vision_config.noise_scale_max).asDouble());
    }
    if (cfg.isMember("nis_gate_3d")) {
        vision_config.nis_gate_3d = cfg.get("nis_gate_3d", vision_config.nis_gate_3d).asDouble();
    }

    vision_config.boost_when_rtk_degraded =
        cfg.get("boost_when_rtk_degraded", vision_config.boost_when_rtk_degraded).asBool();
    if (cfg.isMember("degraded_noise_scale")) {
        vision_config.degraded_noise_scale =
            cfg.get("degraded_noise_scale", vision_config.degraded_noise_scale).asDouble();
    }
    if (cfg.isMember("degraded_max_lateral_correction")) {
        vision_config.degraded_max_lateral_correction =
            cfg.get("degraded_max_lateral_correction", vision_config.degraded_max_lateral_correction).asDouble();
    }
    if (cfg.isMember("degraded_max_heading_correction_deg")) {
        double deg = cfg.get("degraded_max_heading_correction_deg",
                             vision_config.degraded_max_heading_correction * 180.0 / M_PI).asDouble();
        vision_config.degraded_max_heading_correction = deg2rad(deg);
    }

    vision_config.invert_lateral = cfg.get("invert_lateral", vision_config.invert_lateral).asBool();
    vision_config.invert_heading = cfg.get("invert_heading", vision_config.invert_heading).asBool();
    if (cfg.isMember("apply_camera_extrinsics")) {
        vision_config.apply_camera_extrinsics =
            cfg.get("apply_camera_extrinsics", vision_config.apply_camera_extrinsics).asBool();
    }
    if (cfg.isMember("force_apply_extrinsics")) { // backward compat for older drafts
        vision_config.apply_camera_extrinsics =
            cfg.get("force_apply_extrinsics", vision_config.apply_camera_extrinsics).asBool();
    }
    vision_config.apply_post_correction = cfg.get("apply_post_correction", vision_config.apply_post_correction).asBool();
    vision_config.suppress_turning = cfg.get("suppress_turning", vision_config.suppress_turning).asBool();
    vision_config.turn_linear_thresh = cfg.get("turn_linear_thresh", vision_config.turn_linear_thresh).asDouble();
    vision_config.turn_angular_thresh = cfg.get("turn_angular_thresh", vision_config.turn_angular_thresh).asDouble();

    if (cfg.isMember("pose_noise")) {
        const auto& pose_noise = cfg["pose_noise"];
        vision_config.position_noise = pose_noise.get("position", vision_config.position_noise).asDouble();
        vision_config.position_noise_along =
            pose_noise.get("position_along", vision_config.position_noise_along).asDouble();
        double heading_deg = pose_noise.get("heading_deg", vision_config.heading_noise * 180.0 / M_PI).asDouble();
        vision_config.heading_noise = deg2rad(heading_deg);
    }

    if (cfg.isMember("camera_extrinsics")) {
        const auto& cam = cfg["camera_extrinsics"];
        vision_config.camera.x = cam.get("x", vision_config.camera.x).asDouble();
        vision_config.camera.y = cam.get("y", vision_config.camera.y).asDouble();
        vision_config.camera.z = cam.get("z", vision_config.camera.z).asDouble();
        double roll_deg = cam.get("roll_deg", 0.0).asDouble();
        double pitch_deg = cam.get("pitch_deg", 0.0).asDouble();
        double yaw_deg = cam.get("yaw_deg", 0.0).asDouble();
        vision_config.camera.roll = deg2rad(roll_deg);
        vision_config.camera.pitch = deg2rad(pitch_deg);
        vision_config.camera.yaw = deg2rad(yaw_deg);
    }
}

} // namespace

namespace {

Json::Value yamlNodeToJson(const YAML::Node& node) {
    if (!node) {
        return Json::Value();
    }
    if (node.IsScalar()) {
        const std::string s = node.Scalar();
        if (s == "true" || s == "True" || s == "TRUE") return Json::Value(true);
        if (s == "false" || s == "False" || s == "FALSE") return Json::Value(false);
        try {
            std::size_t idx = 0;
            long long i = std::stoll(s, &idx);
            if (idx == s.size()) return Json::Value(static_cast<Json::Int64>(i));
        } catch (...) {}
        try {
            std::size_t idx = 0;
            double d = std::stod(s, &idx);
            if (idx == s.size()) return Json::Value(d);
        } catch (...) {}
        return Json::Value(s);
    }
    if (node.IsSequence()) {
        Json::Value arr(Json::arrayValue);
        for (std::size_t i = 0; i < node.size(); ++i) {
            arr.append(yamlNodeToJson(node[i]));
        }
        return arr;
    }
    if (node.IsMap()) {
        Json::Value obj(Json::objectValue);
        for (const auto& kv : node) {
            obj[kv.first.as<std::string>()] = yamlNodeToJson(kv.second);
        }
        return obj;
    }
    return Json::Value();
}

bool hasYamlExtension(const std::string& path) {
    auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return false;
    std::string ext = path.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == "yaml" || ext == "yml";
}

} // namespace

bool loadParameters(const std::string& param_file, Json::Value& root) {
    if (hasYamlExtension(param_file)) {
        try {
            YAML::Node yaml_root = YAML::LoadFile(param_file);
            root = yamlNodeToJson(yaml_root);
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse YAML parameter file: " << param_file
                      << ", error: " << e.what() << std::endl;
            return false;
        }
    }

    std::ifstream file(param_file);
    if (!file.is_open()) {
        std::cerr << "Failed to open parameter file: " << param_file << std::endl;
        return false;
    }

    Json::CharReaderBuilder builder;
    if (!Json::parseFromStream(builder, file, &root, nullptr)) {
        std::cerr << "Failed to parse JSON parameter file: " << param_file << std::endl;
        return false;
    }
    return true;
}

bool initSensors(const Json::Value& params, ros::NodeHandle& nh) {
    if (!use_hardware_sources) {
        return true;
    }

    int baudrate = params["rtk"]["baudrate"].asInt();
    front_rtk_parser = std::make_unique<RTKParser>(nh, params["rtk"]["front_port"].asString(), baudrate, "front_rtk");
    if (!front_rtk_parser->init()) {
        std::cerr << "Failed to initialize front RTK parser." << std::endl;
        return false;
    }

    rear_rtk_parser = std::make_unique<RTKParser>(nh, params["rtk"]["rear_port"].asString(), baudrate, "rear_rtk");
    if (!rear_rtk_parser->init()) {
        std::cerr << "Failed to initialize rear RTK parser." << std::endl;
        return false;
    }

    encoder_handler = std::make_unique<EncoderHandler>(nh, params["encoder"]["websocket_url"].asString(), "encoder");
    if (!encoder_handler->init()) {
        std::cerr << "Failed to initialize encoder handler." << std::endl;
        return false;
    }

    return true;
}

bool initFusion(const Json::Value& params) {
    std::string algorithm = "ekf";
    if (params.isMember("fusion") && params["fusion"].isMember("algorithm")) {
        algorithm = params["fusion"]["algorithm"].asString();
    }

    if (algorithm == "ukf") {
        ukf_fusion = std::make_unique<UKFFusion>();
        fusion = std::move(ukf_fusion);

        fusion->init();

        double position_noise = params["ukf"]["process_noise"]["position"].asDouble();
        double velocity_noise = params["ukf"]["process_noise"]["velocity"].asDouble();
        double acceleration_noise = params["ukf"]["process_noise"]["acceleration"].asDouble();
        static_cast<UKFFusion*>(fusion.get())->setProcessNoise(position_noise, velocity_noise, acceleration_noise);

        double rtk_position_noise = params["ukf"]["measurement_noise"]["rtk_position"].asDouble();
        double rtk_orientation_noise = params["ukf"]["measurement_noise"]["rtk_orientation"].asDouble();
        double imu_angular_velocity_noise = params["ukf"]["measurement_noise"]["imu_angular_velocity"].asDouble();
        double imu_linear_acceleration_noise = params["ukf"]["measurement_noise"]["imu_linear_acceleration"].asDouble();
        double encoder_velocity_noise = params["ukf"]["measurement_noise"]["encoder_velocity"].asDouble();
        static_cast<UKFFusion*>(fusion.get())->setMeasurementNoise(rtk_position_noise, rtk_orientation_noise,
                                       imu_angular_velocity_noise, imu_linear_acceleration_noise,
                                       encoder_velocity_noise);

        if (params["ukf"].isMember("alpha")) {
            double alpha = params["ukf"]["alpha"].asDouble();
            static_cast<UKFFusion*>(fusion.get())->setAlpha(alpha);
        }
        if (params["ukf"].isMember("beta")) {
            double beta = params["ukf"]["beta"].asDouble();
            static_cast<UKFFusion*>(fusion.get())->setBeta(beta);
        }
        if (params["ukf"].isMember("kappa")) {
            double kappa = params["ukf"]["kappa"].asDouble();
            static_cast<UKFFusion*>(fusion.get())->setKappa(kappa);
        }
    } else {
        ekf_fusion = std::make_unique<EKFFusion>();
        fusion = std::move(ekf_fusion);

        fusion->init();

        double position_noise = params["ekf"]["process_noise"]["position"].asDouble();
        double velocity_noise = params["ekf"]["process_noise"]["velocity"].asDouble();
        double acceleration_noise = params["ekf"]["process_noise"]["acceleration"].asDouble();
        static_cast<EKFFusion*>(fusion.get())->setProcessNoise(position_noise, velocity_noise, acceleration_noise);

        double rtk_position_noise = params["ekf"]["measurement_noise"]["rtk_position"].asDouble();
        double rtk_orientation_noise = params["ekf"]["measurement_noise"]["rtk_orientation"].asDouble();
        double imu_angular_velocity_noise = params["ekf"]["measurement_noise"]["imu_angular_velocity"].asDouble();
        double imu_linear_acceleration_noise = params["ekf"]["measurement_noise"]["imu_linear_acceleration"].asDouble();
        double encoder_velocity_noise = params["ekf"]["measurement_noise"]["encoder_velocity"].asDouble();
        static_cast<EKFFusion*>(fusion.get())->setMeasurementNoise(rtk_position_noise, rtk_orientation_noise,
                                       imu_angular_velocity_noise, imu_linear_acceleration_noise,
                                       encoder_velocity_noise);
    }

    double origin_lat = params["origin"]["latitude"].asDouble();
    double origin_lon = params["origin"]["longitude"].asDouble();
    fusion->setOrigin(origin_lat, origin_lon);
    ::origin_latitude = origin_lat;
    ::origin_longitude = origin_lon;
    initialization_converter.Reset(origin_lat, origin_lon, 0.0);
    initialization_converter_ready = true;

    double offset_x = params["rtk"]["antenna_offset"]["x"].asDouble();
    double offset_y = params["rtk"]["antenna_offset"]["y"].asDouble();
    double offset_theta = params["rtk"]["antenna_offset"]["theta"].asDouble();
    fusion->setAntennaOffset(offset_x, offset_y, offset_theta);

    double baseline_dist = params["rtk"]["rtk_baseline_distance"].asDouble();
    double baseline_tolerance = params["rtk"]["rtk_baseline_tolerance"].asDouble();
    fusion->setRTKBaseline(baseline_dist, baseline_tolerance);
    rtk_baseline_distance_config = baseline_dist;
    rtk_baseline_tolerance_config = baseline_tolerance;
    if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
        double soft_tol = params["rtk"].get("rtk_baseline_soft_tolerance", baseline_tolerance).asDouble();
        rtk_baseline_soft_tolerance_config = soft_tol;
        double default_hard = std::max(soft_tol * 1.8, soft_tol + 0.05);
        double hard_tol = params["rtk"].get("rtk_baseline_hard_tolerance", default_hard).asDouble();
        rtk_baseline_hard_tolerance_config = hard_tol;
        double soft_cap = params["rtk"].get("rtk_baseline_soft_limit", -1.0).asDouble();
        double hard_cap = params["rtk"].get("rtk_baseline_hard_limit", -1.0).asDouble();
        ukf_ptr->setBaselineOutlierPolicy(soft_tol, hard_tol, soft_cap, hard_cap);
        double heading_limit_deg = params["rtk"].get("rtk_heading_consistency_deg", 55.0).asDouble();
        double heading_limit_rad = (heading_limit_deg > 0.0)
            ? heading_limit_deg * (std::acos(-1.0) / 180.0)
            : heading_limit_deg;
        double heading_relax_speed = params["rtk"].get("rtk_heading_relax_speed", 0.05).asDouble();
        ukf_ptr->setBaselineConsistencyChecks(heading_limit_rad, heading_relax_speed);

        double nis_gate_2d = params["rtk"].get("rtk_nis_gate_2d", 9.21).asDouble();
        double nis_gate_3d = params["rtk"].get("rtk_nis_gate_3d", 11.34).asDouble();
        ukf_ptr->setRtkNisGate(nis_gate_2d, nis_gate_3d);

	        int hold_frames = params["rtk"].get("rtk_selector_hold_frames", 5).asInt();
	        double switch_penalty = params["rtk"].get("rtk_selector_switch_penalty", 0.5).asDouble();
	        ukf_ptr->setRtkSelectorConfig(hold_frames, switch_penalty);

		        double jump_threshold_m = params["rtk"].get("rtk_single_step_jump_threshold_m", 0.30).asDouble();
		        double jump_max_dt_s = params["rtk"].get("rtk_single_step_jump_max_dt_s", 0.5).asDouble();
			        double jump_hold_s = params["rtk"].get("rtk_single_step_hold_s", 2.0).asDouble();
			        double jump_score_penalty = params["rtk"].get("rtk_single_step_score_penalty", 0.8).asDouble();
			        ukf_ptr->setRtkSingleStepPolicy(jump_threshold_m, jump_max_dt_s, jump_hold_s, jump_score_penalty);

			        double course_consistency_deg =
			            params["rtk"].get("rtk_baseline_course_consistency_deg", 10.0).asDouble();
			        double course_consistency_rad = (course_consistency_deg > 0.0)
			            ? course_consistency_deg * (std::acos(-1.0) / 180.0)
			            : course_consistency_deg;
			        ukf_ptr->setBaselineCourseConsistency(course_consistency_rad);
			        double degraded_front_penalty =
			            params["rtk"].get("rtk_baseline_degraded_front_penalty", 0.6).asDouble();
			        ukf_ptr->setBaselineDegradedFrontPenalty(degraded_front_penalty);

			        bool pref_enabled = params["rtk"].get("rtk_preference_enabled", true).asBool();
			        double pref_alpha = params["rtk"].get("rtk_preference_ema_alpha", 0.05).asDouble();
			        double pref_penalty_scale = params["rtk"].get("rtk_preference_penalty_scale", 0.8).asDouble();
			        double pref_good = params["rtk"].get("rtk_preference_residual_good_m", 0.08).asDouble();
			        double pref_bad = params["rtk"].get("rtk_preference_residual_bad_m", 0.30).asDouble();
			        double pref_max_penalty = params["rtk"].get("rtk_preference_max_penalty", 2.0).asDouble();
			        ukf_ptr->setRtkAntennaPreferenceConfig(pref_enabled,
			                                              pref_alpha,
			                                              pref_penalty_scale,
			                                              pref_good,
			                                              pref_bad,
			                                              pref_max_penalty);

				        double no_heading_pos_min =
				            params["rtk"].get("rtk_no_heading_position_noise_scale_min", 2.5).asDouble();
			        double no_heading_pos_max =
			            params["rtk"].get("rtk_no_heading_position_noise_scale_max", 200.0).asDouble();
			        ukf_ptr->setRtkNoHeadingPositionNoiseScale(no_heading_pos_min, no_heading_pos_max);

			        double stationary_speed_thr =
			            params["rtk"].get("rtk_stationary_speed_threshold", 0.0).asDouble();
			        double stationary_pos_scale =
			            params["rtk"].get("rtk_stationary_position_noise_scale", 1.0).asDouble();
			        double stationary_release_time_s =
			            params["rtk"].get("rtk_stationary_release_time_s", 1.5).asDouble();
			        ukf_ptr->setRtkStationaryPositionNoiseScale(
			            stationary_speed_thr, stationary_pos_scale, stationary_release_time_s);

			        double turn_speed_thr =
			            params["rtk"].get("rtk_turn_speed_threshold", 0.0).asDouble();
			        double turn_yaw_rate_thr =
			            params["rtk"].get("rtk_turn_yaw_rate_threshold", 0.08).asDouble();
			        double turn_pos_scale =
			            params["rtk"].get("rtk_turn_position_noise_scale", 4.0).asDouble();
			        ukf_ptr->setRtkTurnInPlacePositionNoiseScale(
			            turn_speed_thr, turn_yaw_rate_thr, turn_pos_scale);

				        if (vision_config.enabled) {
				            ukf_ptr->setVisionNisGate(vision_config.nis_gate_3d);
				        }
		    }

    Eigen::Vector3d lever_arm;
    lever_arm(0) = params["imu"]["lever_arm"]["x"].asDouble();
    lever_arm(1) = params["imu"]["lever_arm"]["y"].asDouble();
    lever_arm(2) = params["imu"]["lever_arm"]["z"].asDouble();

    IMUData bias;
    bias.linear_acceleration_x = params["imu"]["bias"]["ax"].asDouble();
    bias.linear_acceleration_y = params["imu"]["bias"]["ay"].asDouble();
    bias.linear_acceleration_z = params["imu"]["bias"]["az"].asDouble();
    bias.angular_velocity_x = params["imu"]["bias"]["gx"].asDouble();
    bias.angular_velocity_y = params["imu"]["bias"]["gy"].asDouble();
    bias.angular_velocity_z = params["imu"]["bias"]["gz"].asDouble();
    fusion->setIMUCalibration(lever_arm, bias);

    double wheel_base = params["encoder"].get("wheel_base", 0.6).asDouble();
    fusion->setWheelBase(wheel_base);
    encoder_wheel_base_m = wheel_base;
    encoder_wheel_diameter_m = params["encoder"].get("wheel_diameter", encoder_wheel_diameter_m).asDouble();
    encoder_max_velocity_limit = params["encoder"].get("max_velocity", 0.0).asDouble();
    encoder_max_velocity_diff = params["encoder"].get("max_velocity_difference", 0.0).asDouble();
    encoder_ticks_per_rev = params["encoder"].get("ticks_per_rev", encoder_ticks_per_rev).asDouble();
    encoder_ticks_velocity_min_dt_s =
        params["encoder"].get("ticks_velocity_min_dt_s", encoder_ticks_velocity_min_dt_s).asDouble();
    encoder_ticks_velocity_max_dt_s =
        params["encoder"].get("ticks_velocity_max_dt_s", encoder_ticks_velocity_max_dt_s).asDouble();

    if (!std::isfinite(encoder_wheel_diameter_m) || encoder_wheel_diameter_m < 0.0) {
        encoder_wheel_diameter_m = 0.0;
    }
    if (!std::isfinite(encoder_wheel_base_m) || encoder_wheel_base_m <= 0.0) {
        encoder_wheel_base_m = 0.0;
    }
    if (!std::isfinite(encoder_ticks_per_rev) || encoder_ticks_per_rev <= 0.0) {
        encoder_ticks_per_rev = 65536.0;
    }
    if (!std::isfinite(encoder_ticks_velocity_min_dt_s) || encoder_ticks_velocity_min_dt_s < 0.0) {
        encoder_ticks_velocity_min_dt_s = 0.0;
    }
    if (!std::isfinite(encoder_ticks_velocity_max_dt_s) || encoder_ticks_velocity_max_dt_s < 0.0) {
        encoder_ticks_velocity_max_dt_s = 0.0;
    }

    if (params["rtk"].isMember("min_satellites")) {
        fusion->setMinSatellites(params["rtk"]["min_satellites"].asInt());
    }

    if (params.isMember("fusion")) {
        const auto& fusion_cfg = params["fusion"];
        zero_velocity_speed_threshold = fusion_cfg.get("zero_velocity_speed_threshold", 0.0).asDouble();
        max_position_jump = fusion_cfg.get("max_position_jump", 0.2).asDouble();
        output_smoothing_alpha = fusion_cfg.get("output_smoothing_alpha", 0.3).asDouble();
        hold_speed_threshold = fusion_cfg.get("hold_speed_threshold", 0.05).asDouble();
        zero_velocity_hold_duration = ros::Duration(fusion_cfg.get("zero_velocity_hold_time", 0.5).asDouble());
        output_stationary_hold_enabled =
            fusion_cfg.get("output_stationary_hold_enabled", output_stationary_hold_enabled).asBool();
        if (fusion_cfg.isMember("output_stationary_speed_threshold")) {
            output_stationary_speed_threshold =
                fusion_cfg.get("output_stationary_speed_threshold", output_stationary_speed_threshold).asDouble();
        } else {
            output_stationary_speed_threshold = zero_velocity_speed_threshold;
        }
        if (fusion_cfg.isMember("output_stationary_hold_time_s")) {
            output_stationary_hold_time_s =
                fusion_cfg.get("output_stationary_hold_time_s", output_stationary_hold_time_s).asDouble();
        }
        output_pivot_hold_enabled =
            fusion_cfg.get("output_pivot_hold_enabled", output_pivot_hold_enabled).asBool();
        if (fusion_cfg.isMember("output_pivot_linear_threshold")) {
            output_pivot_linear_threshold =
                fusion_cfg.get("output_pivot_linear_threshold", output_pivot_linear_threshold).asDouble();
        }
        if (fusion_cfg.isMember("output_pivot_yaw_rate_threshold")) {
            output_pivot_yaw_rate_threshold =
                fusion_cfg.get("output_pivot_yaw_rate_threshold", output_pivot_yaw_rate_threshold).asDouble();
        }
        if (!std::isfinite(output_stationary_speed_threshold)) {
            output_stationary_speed_threshold = zero_velocity_speed_threshold;
        }
        output_stationary_speed_threshold = std::max(output_stationary_speed_threshold, 0.0);
        if (!std::isfinite(output_stationary_hold_time_s)) {
            output_stationary_hold_time_s = 0.0;
        }
        output_stationary_hold_time_s = std::max(output_stationary_hold_time_s, 0.0);
        if (!std::isfinite(output_pivot_linear_threshold)) {
            output_pivot_linear_threshold = 0.0;
        }
        output_pivot_linear_threshold = std::max(output_pivot_linear_threshold, 0.0);
        if (!std::isfinite(output_pivot_yaw_rate_threshold)) {
            output_pivot_yaw_rate_threshold = 0.0;
        }
        output_pivot_yaw_rate_threshold = std::max(output_pivot_yaw_rate_threshold, 0.0);
        output_continuous_yaw = fusion_cfg.get("continuous_yaw_output", 0).asBool();
        straight_yaw_smoothing_enabled = fusion_cfg.get("straight_yaw_smoothing_enabled", straight_yaw_smoothing_enabled).asBool();
        if (fusion_cfg.isMember("straight_yaw_speed_threshold")) {
            straight_yaw_speed_threshold = fusion_cfg.get("straight_yaw_speed_threshold", straight_yaw_speed_threshold).asDouble();
        }
        if (fusion_cfg.isMember("straight_yaw_rate_threshold")) {
            straight_yaw_rate_threshold = fusion_cfg.get("straight_yaw_rate_threshold", straight_yaw_rate_threshold).asDouble();
        }
        if (fusion_cfg.isMember("straight_yaw_prev_weight")) {
            straight_yaw_prev_weight = fusion_cfg.get("straight_yaw_prev_weight", straight_yaw_prev_weight).asDouble();
        }
        if (fusion_cfg.isMember("straight_yaw_jump_limit_rad")) {
            straight_yaw_jump_limit_rad = fusion_cfg.get("straight_yaw_jump_limit_rad", straight_yaw_jump_limit_rad).asDouble();
        }
        if (fusion_cfg.isMember("straight_yaw_severity_max")) {
            straight_yaw_severity_max = fusion_cfg.get("straight_yaw_severity_max", straight_yaw_severity_max).asDouble();
        }
        straight_yaw_speed_threshold = std::max(straight_yaw_speed_threshold, 0.0);
        straight_yaw_rate_threshold = std::max(straight_yaw_rate_threshold, 0.0);
        straight_yaw_prev_weight = std::clamp(straight_yaw_prev_weight, 0.0, 0.995);
        if (!std::isfinite(straight_yaw_jump_limit_rad)) {
            straight_yaw_jump_limit_rad = 0.0;
        }
        straight_yaw_severity_max = std::max(straight_yaw_severity_max, 0.0);
        straight_yaw_course_pull_enabled =
            fusion_cfg.get("straight_yaw_course_pull_enabled", straight_yaw_course_pull_enabled).asBool();
        if (fusion_cfg.isMember("straight_yaw_course_gain")) {
            straight_yaw_course_gain = fusion_cfg.get("straight_yaw_course_gain", straight_yaw_course_gain).asDouble();
        }
        if (fusion_cfg.isMember("straight_yaw_course_max_error_rad")) {
            straight_yaw_course_max_error_rad =
                fusion_cfg.get("straight_yaw_course_max_error_rad", straight_yaw_course_max_error_rad).asDouble();
        }
        if (!std::isfinite(straight_yaw_course_gain)) {
            straight_yaw_course_gain = 0.0;
        }
        if (!std::isfinite(straight_yaw_course_max_error_rad)) {
            straight_yaw_course_max_error_rad = 0.0;
        }
        straight_yaw_course_gain = std::clamp(straight_yaw_course_gain, 0.0, 1.0);
        straight_yaw_course_max_error_rad = std::clamp(straight_yaw_course_max_error_rad, 0.0, 3.14159);

        straight_pos_smoothing_enabled =
            fusion_cfg.get("straight_pos_smoothing_enabled", straight_pos_smoothing_enabled).asBool();
        if (fusion_cfg.isMember("straight_pos_lateral_step_scale")) {
            straight_pos_lateral_step_scale =
                fusion_cfg.get("straight_pos_lateral_step_scale", straight_pos_lateral_step_scale).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_lateral_step_scale_degraded")) {
            straight_pos_lateral_step_scale_degraded =
                fusion_cfg.get("straight_pos_lateral_step_scale_degraded", straight_pos_lateral_step_scale_degraded).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_lateral_decay_degraded")) {
            straight_pos_lateral_decay_degraded =
                fusion_cfg.get("straight_pos_lateral_decay_degraded", straight_pos_lateral_decay_degraded).asDouble();
        }
        if (!std::isfinite(straight_pos_lateral_step_scale)) {
            straight_pos_lateral_step_scale = 1.0;
        }
        if (!std::isfinite(straight_pos_lateral_step_scale_degraded)) {
            straight_pos_lateral_step_scale_degraded = 1.0;
        }
        if (!std::isfinite(straight_pos_lateral_decay_degraded)) {
            straight_pos_lateral_decay_degraded = 0.0;
        }
        if (fusion_cfg.isMember("straight_pos_lateral_abs_limit_m")) {
            straight_pos_lateral_abs_limit_m =
                fusion_cfg.get("straight_pos_lateral_abs_limit_m", straight_pos_lateral_abs_limit_m).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_dir_update_alpha")) {
            straight_pos_dir_update_alpha =
                fusion_cfg.get("straight_pos_dir_update_alpha", straight_pos_dir_update_alpha).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_dir_min_step_m")) {
            straight_pos_dir_min_step_m =
                fusion_cfg.get("straight_pos_dir_min_step_m", straight_pos_dir_min_step_m).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_dir_lock_distance_m")) {
            straight_pos_dir_lock_distance_m =
                fusion_cfg.get("straight_pos_dir_lock_distance_m", straight_pos_dir_lock_distance_m).asDouble();
        }
        if (fusion_cfg.isMember("straight_pos_frame_keep_s")) {
            straight_pos_frame_keep_s =
                fusion_cfg.get("straight_pos_frame_keep_s", straight_pos_frame_keep_s).asDouble();
        }
        if (!std::isfinite(straight_pos_lateral_abs_limit_m)) {
            straight_pos_lateral_abs_limit_m = 0.0;
        }
        if (!std::isfinite(straight_pos_dir_update_alpha)) {
            straight_pos_dir_update_alpha = 0.0;
        }
        if (!std::isfinite(straight_pos_dir_min_step_m)) {
            straight_pos_dir_min_step_m = 0.0;
        }
        if (!std::isfinite(straight_pos_dir_lock_distance_m)) {
            straight_pos_dir_lock_distance_m = 0.0;
        }
        if (!std::isfinite(straight_pos_frame_keep_s)) {
            straight_pos_frame_keep_s = 0.0;
        }
        straight_pos_lateral_step_scale = std::clamp(straight_pos_lateral_step_scale, 0.0, 1.0);
        straight_pos_lateral_step_scale_degraded = std::clamp(straight_pos_lateral_step_scale_degraded, 0.0, 1.0);
        straight_pos_lateral_decay_degraded = std::clamp(straight_pos_lateral_decay_degraded, 0.0, 1.0);
        straight_pos_lateral_abs_limit_m = std::max(straight_pos_lateral_abs_limit_m, 0.0);
        straight_pos_dir_update_alpha = std::clamp(straight_pos_dir_update_alpha, 0.0, 1.0);
        straight_pos_dir_min_step_m = std::max(straight_pos_dir_min_step_m, 0.0);
        straight_pos_dir_lock_distance_m = std::max(straight_pos_dir_lock_distance_m, 0.0);
        straight_pos_frame_keep_s = std::max(straight_pos_frame_keep_s, 0.0);
        publish_predict_odom = fusion_cfg.get("publish_predict_odom", publish_predict_odom).asBool();
        if (fusion_cfg.isMember("predict_odom_max_rate_hz")) {
            predict_odom_max_rate_hz = fusion_cfg.get("predict_odom_max_rate_hz", predict_odom_max_rate_hz).asDouble();
            if (!std::isfinite(predict_odom_max_rate_hz) || predict_odom_max_rate_hz < 0.0) {
                predict_odom_max_rate_hz = 0.0;
            }
            predict_odom_max_rate_hz = std::clamp(predict_odom_max_rate_hz, 0.0, 400.0);
        }
        bootstrap_required_frames = fusion_cfg.get("bootstrap_frames", 5).asInt();
        bootstrap_max_residual = fusion_cfg.get("bootstrap_max_residual", 0.5).asDouble();
        initialization_required_samples = std::max(fusion_cfg.get("initialization_samples", 3).asInt(), 1);
        double init_timeout_sec = fusion_cfg.get("initialization_timeout", 2.0).asDouble();
        initialization_timeout = ros::Duration(std::max(init_timeout_sec, 0.0));

        double warmup_time = fusion_cfg.get("startup_warmup_time", 5.0).asDouble();
        if (warmup_time <= 0.0) {
            startup_warmup_enabled = false;
            startup_warmup_duration = ros::Duration(0.0);
        } else {
        startup_warmup_enabled = true;
        startup_warmup_duration = ros::Duration(warmup_time);
        }
        startup_warmup_complete = !startup_warmup_enabled;
        startup_reference_set = false;
        startup_alignment_max_samples = std::max(5, fusion_cfg.get("startup_alignment_samples", 20).asInt());
    }

    if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
        if (params.isMember("health_monitor")) {
            const auto& hm = params["health_monitor"];
            if (hm.isMember("rtk_residual_threshold")) {
                ukf_ptr->setResidualThreshold(hm["rtk_residual_threshold"].asDouble());
            }
            if (hm.isMember("encoder_mismatch_threshold")) {
                ukf_ptr->setEncoderMismatchThreshold(hm["encoder_mismatch_threshold"].asDouble());
            }
            if (hm.isMember("rtk_residual_window")) {
                ukf_ptr->setResidualWindowSize(static_cast<std::size_t>(hm["rtk_residual_window"].asUInt()));
            }
            if (hm.isMember("encoder_mismatch_window")) {
                ukf_ptr->setEncoderWindowSize(static_cast<std::size_t>(hm["encoder_mismatch_window"].asUInt()));
            }
            if (hm.isMember("residual_soft_reset_multiplier")) {
                ukf_ptr->setResidualSoftResetMultiplier(hm["residual_soft_reset_multiplier"].asDouble());
            }
            if (hm.isMember("residual_surge_limit")) {
                ukf_ptr->setResidualSurgeLimit(hm["residual_surge_limit"].asInt());
            }
            if (hm.isMember("recovery_release_ratio")) {
                ukf_ptr->setRecoveryReleaseRatio(hm["recovery_release_ratio"].asDouble());
            }
        }
        double fallback_process = 0.2;
        double fallback_noise = 0.2;
        double fallback_noise_max = 1.0;
        double fallback_bias_alpha = 0.02;
        double fallback_velocity_noise = 0.1;
        double fallback_velocity_speed_threshold = 0.05;
        double fallback_course_window_s = 1.5;
        double fallback_course_min_distance_m = 0.08;
        double fallback_course_gain = 0.35;
        bool nonholonomic_enabled = true;
        double nonholonomic_lateral_velocity_noise = 0.05;
        double nonholonomic_speed_threshold = 0.05;
        if (params.isMember("fusion")) {
            const auto& fusion_cfg = params["fusion"];
            fallback_process = fusion_cfg.get("fallback_heading_process_noise", fallback_process).asDouble();
            fallback_noise = fusion_cfg.get("fallback_heading_noise", fallback_noise).asDouble();
            fallback_noise_max = fusion_cfg.get("fallback_heading_noise_max", fallback_noise_max).asDouble();
            fallback_bias_alpha = fusion_cfg.get("fallback_heading_bias_alpha", fallback_bias_alpha).asDouble();
            fallback_velocity_noise = fusion_cfg.get("fallback_heading_velocity_noise", fallback_velocity_noise).asDouble();
            fallback_velocity_speed_threshold = fusion_cfg.get("fallback_heading_velocity_speed_threshold",
                                                              fallback_velocity_speed_threshold).asDouble();
            if (fusion_cfg.isMember("fallback_course_window_s")) {
                fallback_course_window_s = fusion_cfg.get("fallback_course_window_s", fallback_course_window_s).asDouble();
            }
            if (fusion_cfg.isMember("fallback_course_min_distance_m")) {
                fallback_course_min_distance_m =
                    fusion_cfg.get("fallback_course_min_distance_m", fallback_course_min_distance_m).asDouble();
            }
            if (fusion_cfg.isMember("fallback_course_gain")) {
                fallback_course_gain = fusion_cfg.get("fallback_course_gain", fallback_course_gain).asDouble();
            }
            nonholonomic_enabled = fusion_cfg.get("nonholonomic_enabled", nonholonomic_enabled).asBool();
            nonholonomic_lateral_velocity_noise =
                fusion_cfg.get("nonholonomic_lateral_velocity_noise", nonholonomic_lateral_velocity_noise).asDouble();
            nonholonomic_speed_threshold =
                fusion_cfg.get("nonholonomic_speed_threshold", nonholonomic_speed_threshold).asDouble();
        }
        if (!std::isfinite(fallback_course_window_s)) {
            fallback_course_window_s = 1.5;
        }
        if (!std::isfinite(fallback_course_min_distance_m)) {
            fallback_course_min_distance_m = 0.08;
        }
        if (!std::isfinite(fallback_course_gain)) {
            fallback_course_gain = 0.35;
        }
        fallback_course_window_s = std::clamp(fallback_course_window_s, 0.2, 6.0);
        fallback_course_min_distance_m = std::clamp(fallback_course_min_distance_m, 0.02, 1.0);
        fallback_course_gain = std::clamp(fallback_course_gain, 0.0, 1.0);
        ukf_ptr->setNonholonomicConstraint(nonholonomic_enabled,
                                           nonholonomic_lateral_velocity_noise,
                                           nonholonomic_speed_threshold);
        bool slip_enabled = false;
        double slip_min_speed = 0.05;
        double slip_yaw_diff = 0.35;
        double slip_yaw_rel = 0.7;
        double slip_ema_alpha = 0.2;
        double slip_encoder_floor = 0.25;
        double slip_nonholonomic_scale = 15.0;
        if (params.isMember("encoder")) {
            const auto& enc_cfg = params["encoder"];
            slip_enabled = enc_cfg.get("wheel_slip_enabled", slip_enabled).asBool();
            slip_min_speed = enc_cfg.get("wheel_slip_min_speed_mps", slip_min_speed).asDouble();
            slip_yaw_diff = enc_cfg.get("wheel_slip_yaw_rate_diff_threshold", slip_yaw_diff).asDouble();
            slip_yaw_rel = enc_cfg.get("wheel_slip_yaw_rate_rel_threshold", slip_yaw_rel).asDouble();
            slip_ema_alpha = enc_cfg.get("wheel_slip_ema_alpha", slip_ema_alpha).asDouble();
            slip_encoder_floor = enc_cfg.get("wheel_slip_encoder_weight_floor_scale", slip_encoder_floor).asDouble();
            slip_nonholonomic_scale =
                enc_cfg.get("wheel_slip_nonholonomic_noise_scale_max", slip_nonholonomic_scale).asDouble();
        }
        ukf_ptr->setWheelSlipConfig(slip_enabled,
                                    slip_min_speed,
                                    slip_yaw_diff,
                                    slip_yaw_rel,
                                    slip_ema_alpha,
                                    slip_encoder_floor,
                                    slip_nonholonomic_scale);
        ukf_ptr->setFallbackHeadingConfig(fallback_process,
                                          fallback_noise,
                                          fallback_noise_max,
                                          fallback_bias_alpha,
                                          fallback_velocity_noise,
                                          fallback_velocity_speed_threshold,
                                          fallback_course_window_s,
                                          fallback_course_min_distance_m,
                                          fallback_course_gain);
    }

    raw_output_state = OutputSmoothingState();
    centered_output_state = OutputSmoothingState();
    bootstrap_pending = true;
    bootstrap_frame_count = 0;
    initialization_buffer.clear();
    initialization_start_time = ros::Time();
    initialization_active = false;
    return true;
}

bool initNetwork(const Json::Value& params) {
    if (!use_network_sender) {
        return true;
    }

    std::string post_url = params["network"]["post_url"].asString();
    network_sender = std::make_unique<NetworkSender>(post_url);

    if (!network_sender->init()) {
        std::cerr << "Failed to initialize network sender." << std::endl;
        return false;
    }

    int send_frequency = params["network"]["send_frequency"].asInt();
    network_sender->setSendFrequency(send_frequency);

    int output_precision = params["network"]["output_precision"].asInt();
    network_sender->setOutputPrecision(output_precision);

    bool out_predict_enabled = false;
    double out_predict_max_dt_s = 0.15;
    bool out_smooth_enabled = false;
    double out_smooth_alpha = 0.85;
    if (params.isMember("network")) {
        const auto& net_cfg = params["network"];
        out_predict_enabled = net_cfg.get("output_predict_enabled", out_predict_enabled).asBool();
        out_predict_max_dt_s = net_cfg.get("output_predict_max_dt_s", out_predict_max_dt_s).asDouble();
        out_smooth_enabled = net_cfg.get("output_smooth_enabled", out_smooth_enabled).asBool();
        out_smooth_alpha = net_cfg.get("output_smooth_alpha", out_smooth_alpha).asDouble();
    }
    network_sender->setOutputPrediction(out_predict_enabled, out_predict_max_dt_s);
    network_sender->setOutputSmoothing(out_smooth_enabled, out_smooth_alpha);

    NetworkSender::MapOutputConfig map_cfg;
    map_cfg.enu_origin_lat = ::origin_latitude;
    map_cfg.enu_origin_lon = ::origin_longitude;
    if (params.isMember("network")) {
        const auto& net_cfg = params["network"];
        if (net_cfg.isMember("map_output")) {
            const auto& map = net_cfg["map_output"];
            map_cfg.enabled = map.get("enabled", false).asBool();
            map_cfg.input_is_wgs84 = map.get("input_is_wgs84", true).asBool();
            map_cfg.swap_xy = map.get("swap_xy", true).asBool();

            if (map.isMember("origin_latitude") && map.isMember("origin_longitude")) {
                map_cfg.has_map_origin = true;
                map_cfg.map_origin_lat = map.get("origin_latitude", 0.0).asDouble();
                map_cfg.map_origin_lon = map.get("origin_longitude", 0.0).asDouble();
            }

            map_cfg.xdir = map.get("xdir", 1.0).asDouble();
            map_cfg.ydir = map.get("ydir", 1.0).asDouble();
            map_cfg.thetadir = map.get("thetadir", 1.0).asDouble();
            map_cfg.pose_x = map.get("poseX", 0.0).asDouble();
            map_cfg.pose_y = map.get("poseY", 0.0).asDouble();
            map_cfg.pose_theta = map.get("poseTheta", 0.0).asDouble();
        }
    }
    network_sender->setMapOutputConfig(map_cfg);

    return true;
}

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_fusion/Rtk.h"
#include "sensor_fusion/Encoder.h"

typedef sensor_fusion::Rtk RtkMsg;
typedef sensor_fusion::Encoder EncoderMsg;

typedef message_filters::sync_policies::ApproximateTime<RtkMsg, RtkMsg, EncoderMsg> MySyncPolicy;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    IMUData imu_data;
    ros::Time imu_stamp = imu_msg->header.stamp;
    if (imu_stamp.isZero()) {
        imu_stamp = ros::Time::now();
    }
    imu_data.timestamp = imu_stamp;
    imu_data.orientation_x = imu_msg->orientation.x;
    imu_data.orientation_y = imu_msg->orientation.y;
    imu_data.orientation_z = imu_msg->orientation.z;
    imu_data.orientation_w = imu_msg->orientation.w;
    imu_data.angular_velocity_x = imu_msg->angular_velocity.x;
    imu_data.angular_velocity_y = imu_msg->angular_velocity.y;
    imu_data.angular_velocity_z = imu_msg->angular_velocity.z;
    imu_data.linear_acceleration_x = imu_msg->linear_acceleration.x;
    imu_data.linear_acceleration_y = imu_msg->linear_acceleration.y;
    imu_data.linear_acceleration_z = imu_msg->linear_acceleration.z;
    imu_data.valid = true;

    {
        std::lock_guard<std::mutex> lock(latest_data_mutex);
        latest_imu_data = imu_data;
        latest_imu_valid = true;
    }

    startup_imu_ready.store(true);

    if (!fusion_time_initialized) {
        fusion->setInitialTime(imu_data.timestamp);
        fusion_time_initialized = true;
    }

    fusion->predict(imu_data);

    if (global_params["fusion"].isMember("enable_zero_velocity_update") &&
        global_params["fusion"]["enable_zero_velocity_update"].asBool()) {
        EncoderData encoder_snapshot;
        bool encoder_valid = false;
        {
            std::lock_guard<std::mutex> lock(latest_data_mutex);
            encoder_snapshot = latest_encoder_data;
            encoder_valid = latest_encoder_valid;
        }

        static ros::Time zero_candidate_start;

        bool encoder_stationary = encoder_valid &&
                                  std::abs(encoder_snapshot.left_velocity) <= zero_velocity_speed_threshold &&
                                  std::abs(encoder_snapshot.right_velocity) <= zero_velocity_speed_threshold;

        if (encoder_stationary) {
            if (zero_candidate_start.isZero()) {
                zero_candidate_start = imu_data.timestamp;
            }

            if ((imu_data.timestamp - zero_candidate_start) >= zero_velocity_hold_duration) {
                if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
                    ukf_ptr->zeroVelocityUpdate();
                } else if (auto ekf_ptr = dynamic_cast<EKFFusion*>(fusion.get())) {
                    ekf_ptr->zeroVelocityUpdate();
                }
            }
        } else {
            zero_candidate_start = ros::Time();
        }
    }

    if (publish_predict_odom && odom_predict_publisher) {
        ros::Time stamp = imu_data.timestamp;
        if (stamp.isZero()) {
            stamp = ros::Time::now();
        }

        static ros::Time last_pub_stamp;
        bool allow_publish = true;
        if (predict_odom_max_rate_hz > 1e-3) {
            double min_dt = 1.0 / std::max(predict_odom_max_rate_hz, 1e-3);
            if (!last_pub_stamp.isZero()) {
                double dt = (stamp - last_pub_stamp).toSec();
                if (dt >= 0.0 && dt < min_dt) {
                    allow_publish = false;
                }
            }
        }

        if (allow_publish) {
            last_pub_stamp = stamp;
            FusionResult predicted = fusion->getFusionResult();
            if (predicted.valid) {
                publishOdometryMessage(predicted, odom_predict_publisher, stamp);
            }
        }
    }
}

void syncCallback(const RtkMsg::ConstPtr& front_rtk_msg,
                  const RtkMsg::ConstPtr& rear_rtk_msg,
                  const EncoderMsg::ConstPtr& encoder_msg)
{
    ros::Time current_time = ros::Time::now();

    if (startup_warmup_enabled && !startup_reference_set && !current_time.isZero()) {
        startup_reference_time = current_time;
        startup_reference_set = true;
    }

	    bool front_quality_ok = front_rtk_msg->fix_mode == accuracy_threshold &&
	                            front_rtk_msg->satellite_count >= min_satellites;
	    if (!front_quality_ok) {
	        ROS_WARN_THROTTLE(1.0, "Front RTK quality insufficient, entering de-weighted mode.");
	    }

	    bool rear_quality_ok = rear_rtk_msg->fix_mode == accuracy_threshold &&
	                            rear_rtk_msg->satellite_count >= min_satellites;
	    if (!rear_quality_ok) {
	        ROS_WARN_THROTTLE(1.0, "Rear RTK quality insufficient, entering de-weighted mode.");
	    }

    RTKData front_rtk_data;
    front_rtk_data.timestamp = current_time;
    front_rtk_data.latitude = front_rtk_msg->latitude;
    front_rtk_data.longitude = front_rtk_msg->longitude;
	    front_rtk_data.fix_mode = front_rtk_msg->fix_mode;
	    front_rtk_data.satellite_count = front_rtk_msg->satellite_count;
	    front_rtk_data.raw_valid = static_cast<bool>(front_rtk_msg->valid);
	    front_rtk_data.valid = front_rtk_data.raw_valid && (rtk_soft_quality_gate || front_quality_ok);

	    RTKData rear_rtk_data;
	    rear_rtk_data.timestamp = current_time;
	    rear_rtk_data.latitude = rear_rtk_msg->latitude;
	    rear_rtk_data.longitude = rear_rtk_msg->longitude;
	    rear_rtk_data.fix_mode = rear_rtk_msg->fix_mode;
	    rear_rtk_data.satellite_count = rear_rtk_msg->satellite_count;
	    rear_rtk_data.raw_valid = static_cast<bool>(rear_rtk_msg->valid);
	    rear_rtk_data.valid = rear_rtk_data.raw_valid && (rtk_soft_quality_gate || rear_quality_ok);

    if (startup_warmup_enabled && !startup_warmup_complete &&
        front_rtk_data.valid && rear_rtk_data.valid) {
        startup_alignment_buffer.emplace_back(front_rtk_data, rear_rtk_data);
        if (static_cast<int>(startup_alignment_buffer.size()) > startup_alignment_max_samples) {
            startup_alignment_buffer.erase(startup_alignment_buffer.begin());
        }
    }

    IMUData imu_data;
    {
        std::lock_guard<std::mutex> lock(latest_data_mutex);
        if (!latest_imu_valid) {
            ROS_WARN_THROTTLE(1.0, "IMU not ready, skipping this update.");
            return;
        }
        imu_data = latest_imu_data;
    }

    EncoderData encoder_data;
    encoder_data.timestamp = current_time;
    encoder_data.left_encoder = encoder_msg->left_encoder;
    encoder_data.right_encoder = encoder_msg->right_encoder;

    const bool encoder_msg_valid = static_cast<bool>(encoder_msg->valid);
    double ticks_left_mps = std::numeric_limits<double>::quiet_NaN();
    double ticks_right_mps = std::numeric_limits<double>::quiet_NaN();
    double ticks_dt_s = std::numeric_limits<double>::quiet_NaN();
    bool ticks_vel_valid = false;
    if (encoder_msg_valid) {
        const double abs_limit =
            (encoder_max_velocity_limit > 0.0) ? encoder_max_velocity_limit : 10.0;
        ticks_vel_valid = computeWheelVelocityFromEncoderTicks(encoder_data.left_encoder,
                                                               encoder_data.right_encoder,
                                                               current_time,
                                                               encoder_ticks_per_rev,
                                                               encoder_wheel_diameter_m,
                                                               encoder_ticks_velocity_min_dt_s,
                                                               encoder_ticks_velocity_max_dt_s,
                                                               abs_limit,
                                                               encoder_ticks_velocity_state,
                                                               ticks_left_mps,
                                                               ticks_right_mps,
                                                               ticks_dt_s);
        if (ticks_vel_valid && encoder_max_velocity_diff > 0.0) {
            if (std::abs(ticks_left_mps - ticks_right_mps) > encoder_max_velocity_diff) {
                ticks_vel_valid = false;
            }
        }
    }

    encoder_data.left_velocity = ticks_vel_valid ? ticks_left_mps : 0.0;
    encoder_data.right_velocity = ticks_vel_valid ? ticks_right_mps : 0.0;
    encoder_data.valid = ticks_vel_valid;

    {
        std::lock_guard<std::mutex> lock(latest_data_mutex);
        latest_encoder_data = encoder_data;
        latest_encoder_valid = encoder_data.valid;
    }

    if (!fusion->isInitialized()) {
        if (!initialization_active) {
            initialization_active = true;
            initialization_buffer.clear();
            initialization_start_time = current_time;
        }

        bool quality_ok = front_rtk_data.valid && rear_rtk_data.valid;
        bool baseline_ok = true;
        if (quality_ok) {
            double measured_distance = 0.0;
            baseline_ok = baselineWithinTolerance(front_rtk_data, rear_rtk_data, measured_distance);
            if (!baseline_ok) {
                ROS_WARN_STREAM_THROTTLE(1.0, "Abnormal RTK baseline during initialization: measured=" << measured_distance
                                         << " expected=" << rtk_baseline_distance_config);
            }
        }

        if (quality_ok && baseline_ok) {
            initialization_buffer.emplace_back(front_rtk_data, rear_rtk_data);
            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "Initialization: buffered RTK sample " << initialization_buffer.size() << " / "
                                                       << initialization_required_samples);
        }

        bool timeout_reached = initialization_timeout.toSec() > 0.0 &&
                                !initialization_start_time.isZero() &&
                                (current_time - initialization_start_time) >= initialization_timeout;

        if (initialization_buffer.size() >= static_cast<std::size_t>(initialization_required_samples)) {
            if (finalizeInitializationFromBuffer(current_time)) {
                return;
            } else {
                initialization_buffer.clear();
                initialization_start_time = current_time;
            }
        } else if (timeout_reached) {
            if (!initialization_buffer.empty()) {
                ROS_WARN_STREAM("Initialization timeout: finalizing with "
                                << initialization_buffer.size() << " samples.");
                if (finalizeInitializationFromBuffer(current_time)) {
                    return;
                }
            }
            ROS_WARN("Initialization timeout: fallback initialize using current RTK frame.");
            if (fusion->initializeFromRTK(front_rtk_data, rear_rtk_data)) {
                initialization_buffer.clear();
                initialization_start_time = ros::Time();
                initialization_active = false;
                return;
            } else {
                initialization_start_time = current_time;
            }
        }

        return;
    }

    fusion->update(front_rtk_data, rear_rtk_data, imu_data, encoder_data);

    if (startup_warmup_enabled && !startup_warmup_complete) {
        if (!startup_reference_set) {
            ROS_INFO_THROTTLE(1.0, "Waiting for startup warmup reference time...");
            return;
        }

        double warmup_elapsed = (current_time - startup_reference_time).toSec();
        bool time_ready = warmup_elapsed >= startup_warmup_duration.toSec();
        bool filter_ready = fusion->isInitialized();
        bool imu_ready = startup_imu_ready.load();

        if (time_ready && filter_ready && imu_ready) {
            if (!startup_alignment_buffer.empty()) {
                RTKData front_avg;
                RTKData rear_avg;
                if (computeAverageRTKSamples(startup_alignment_buffer, current_time, front_avg, rear_avg)) {
                    fusion->reset();
                    fusion_time_initialized = false;
                    if (fusion->initializeFromRTK(front_avg, rear_avg)) {
                        ROS_INFO("Startup warmup: fusion re-initialized using %zu buffered RTK samples.",
                                 startup_alignment_buffer.size());
                    } else {
                        ROS_WARN("Startup warmup: failed to re-initialize fusion with buffered RTK samples.");
                    }
                }
                startup_alignment_buffer.clear();
            }

            startup_warmup_complete = true;
            raw_output_state = OutputSmoothingState();
            centered_output_state = OutputSmoothingState();
            startup_reference_set = true;
            if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
                ukf_ptr->resetHealthMonitor();
            }
            ROS_INFO("Startup warmup finished after %.2f s.", warmup_elapsed);
            return;
        } else {
            ROS_INFO_THROTTLE(1.0,
                              "Startup warmup in progress (%.1f/%.1f s, filter_ready=%s, imu_ready=%s)",
                              warmup_elapsed,
                              startup_warmup_duration.toSec(),
                              filter_ready ? "true" : "false",
                              imu_ready ? "true" : "false");
            return;
        }
    }

		    FusionResult result = fusion->getFusionResult();
		    if (result.valid) {
		        bool rtk_reliable = true;
		        UKFFusion::HealthStatus status;
		        bool has_status = false;
		        UKFFusion::RtkDebugInfo rtk_debug;
		        bool has_rtk_debug = false;
		        bool selector_ok = true;
		        bool selector_single_ok = false;

		        if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
		            status = ukf_ptr->getHealthStatus();
		            has_status = true;
		            rtk_debug = ukf_ptr->getRtkDebugInfo();
		            has_rtk_debug = true;
		            selector_ok = rtk_debug.gate_passed && rtk_debug.selected_dim > 0;
		            selector_single_ok = selector_ok &&
		                                 (rtk_debug.selected_mode == UKFFusion::RtkMeasurementData::Mode::FRONT_ONLY ||
		                                  rtk_debug.selected_mode == UKFFusion::RtkMeasurementData::Mode::REAR_ONLY);
		            rtk_reliable = selector_ok && !status.drift_detected;
		        }

		        bool baseline_degraded = false;
		        double baseline_severity = 0.0;
		        if (front_rtk_data.valid && rear_rtk_data.valid &&
		            initialization_converter_ready && rtk_baseline_distance_config > 1e-6) {
	            double measured_distance = 0.0;
	            baselineWithinTolerance(front_rtk_data, rear_rtk_data, measured_distance);
	            double deviation = std::abs(measured_distance - rtk_baseline_distance_config);
	            double hard_tol = rtk_baseline_hard_tolerance_config;
	            if (hard_tol <= 0.0) {
	                double base_soft = (rtk_baseline_soft_tolerance_config > 0.0)
	                    ? rtk_baseline_soft_tolerance_config
	                    : std::max(rtk_baseline_tolerance_config, 0.2);
	                hard_tol = std::max(base_soft * 1.8, base_soft + 0.05);
	            }
	            double ratio = measured_distance / std::max(rtk_baseline_distance_config, 1e-6);
	            bool gross = ratio < 0.5 || ratio > 1.6;
	            baseline_degraded = gross || (hard_tol > 1e-6 && deviation > hard_tol);
	            if (baseline_degraded) {
	                baseline_severity = hard_tol > 1e-6
	                    ? std::clamp(deviation / hard_tol, 0.0, 10.0)
	                    : 1.0;
	                ROS_WARN_STREAM_THROTTLE(
	                    1.0,
	                    "RTK baseline degraded: measured=" << measured_distance
	                                                     << " expected=" << rtk_baseline_distance_config
	                                                     << " hard_tol=" << hard_tol
	                                                     << " gross=" << (gross ? "true" : "false"));
		            }
		        }

			        bool baseline_blocks_output = baseline_degraded;
			        if (baseline_blocks_output) {
			            rtk_reliable = false;
			        }

			        bool yaw_reliable = rtk_reliable;
			        if (has_rtk_debug && rtk_debug.selected_dim < 3) {
			            yaw_reliable = false;
			        }

			        double encoder_speed_metric = 0.0;
			        bool encoder_stationary = false;
			        bool force_position_hold = false;
			        if (encoder_data.valid) {
			            encoder_speed_metric = 0.5 * (std::abs(encoder_data.left_velocity) +
			                                          std::abs(encoder_data.right_velocity));
			            double thr = output_stationary_speed_threshold;
			            if (!std::isfinite(thr) || thr <= 0.0) {
			                thr = zero_velocity_speed_threshold;
			            }
			            encoder_stationary = (thr > 0.0) &&
			                                 (std::abs(encoder_data.left_velocity) <= thr) &&
			                                 (std::abs(encoder_data.right_velocity) <= thr);

			            if (!encoder_stationary &&
			                output_pivot_hold_enabled &&
			                output_pivot_linear_threshold > 0.0 &&
			                encoder_wheel_base_m > 1e-6 &&
			                std::isfinite(encoder_data.left_velocity) &&
			                std::isfinite(encoder_data.right_velocity)) {
			                const double v_lin = 0.5 * (encoder_data.left_velocity + encoder_data.right_velocity);
			                const double yaw_rate =
			                    (encoder_data.right_velocity - encoder_data.left_velocity) / encoder_wheel_base_m;
			                if (std::isfinite(v_lin) && std::isfinite(yaw_rate)) {
			                    const double wheel_product = encoder_data.left_velocity * encoder_data.right_velocity;
			                    const bool opposite_sign = std::isfinite(wheel_product) && (wheel_product < 0.0);
			                    bool turning = opposite_sign;
			                    if (!turning &&
			                        output_pivot_yaw_rate_threshold > 0.0 &&
			                        std::abs(yaw_rate) >= output_pivot_yaw_rate_threshold) {
			                        turning = true;
			                    }
			                    if (turning && std::abs(v_lin) <= output_pivot_linear_threshold) {
			                        encoder_stationary = true;
			                        force_position_hold = true;
			                    }
			                }
			            }
		        }

		        FusionResult stabilized_result = stabilizeOutput(
		            result,
		            rtk_reliable,
		            yaw_reliable,
		            has_status ? &status : nullptr,
			            encoder_speed_metric,
			            encoder_data.valid,
		            encoder_stationary,
		            force_position_hold,
			            baseline_severity);

			        if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
			            if (has_status && has_rtk_debug && status.drift_detected && selector_ok &&
			                !baseline_blocks_output && std::isfinite(status.rtk_residual_avg) &&
			                status.rtk_residual_avg <= bootstrap_max_residual) {
			                ukf_ptr->resetHealthMonitor();
			                status = ukf_ptr->getHealthStatus();
			                rtk_reliable = selector_ok && !status.drift_detected;
			                yaw_reliable = rtk_reliable;
			                if (has_rtk_debug && rtk_debug.selected_dim < 3) {
			                    yaw_reliable = false;
			                }

			                bootstrap_pending = true;
			                bootstrap_frame_count = 0;
			                stabilized_result = stabilizeOutput(
			                    result,
			                    rtk_reliable,
			                    yaw_reliable,
			                    &status,
			                    encoder_speed_metric,
			                    encoder_data.valid,
			                    encoder_stationary,
			                    force_position_hold,
			                    baseline_severity);
			            }
			        }

	        FusionResult centered_result = stabilized_result;
	        VisionPoseMeasurement vision_pose;
	        double lateral_correction = 0.0;
	        double heading_correction = 0.0;
	        bool vision_used = false;
	        bool vision_gate_passed = false;
	        UKFFusion::VisionDebugInfo vision_debug;
	        bool has_vision_debug = false;

		        bool quality_degraded = !front_quality_ok || !rear_quality_ok;
		        bool rtk_degraded_for_vision =
		            baseline_degraded || quality_degraded || !selector_ok || selector_single_ok ||
		            (has_status && status.drift_detected);

		        if (computeVisionPoseMeasurement(result,
		                                         rtk_degraded_for_vision,
		                                         vision_pose,
		                                         lateral_correction,
		                                         heading_correction)) {
	            fusion->applyVisionPoseMeasurement(vision_pose);
	            vision_gate_passed = true;

	            if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
	                vision_debug = ukf_ptr->getVisionDebugInfo();
	                has_vision_debug = true;
	                vision_gate_passed = vision_debug.gate_passed;
	                vision_stats.last_nis = vision_debug.nis;
	                vision_stats.last_gate_passed = vision_debug.gate_passed;
	            } else {
	                vision_stats.last_nis = std::numeric_limits<double>::quiet_NaN();
	                vision_stats.last_gate_passed = true;
	            }

	            if (vision_gate_passed) {
	                vision_used = true;
	                vision_stats.used_total += 1;
	                vision_stats.used += 1;
	                centered_result = fusion->getFusionResult();
	            } else {
	                vision_stats.drop_nis_total += 1;
	                vision_stats.drop_nis += 1;
	            }
	        } else if (vision_config.apply_post_correction) {
	            centered_result = applyVisionConstraint(stabilized_result);
	        }

	        if (rtk_debug_publisher && has_rtk_debug) {
	            std_msgs::String dbg_msg;
	            std::ostringstream oss;
		            oss << "{"
		                << "\"selected_mode\":\"" << rtkModeToString(rtk_debug.selected_mode) << "\","
		                << "\"selected_dim\":" << rtk_debug.selected_dim << ","
		                << "\"gate_passed\":" << (rtk_debug.gate_passed ? "true" : "false") << ","
	                << "\"baseline_ok\":" << (rtk_debug.baseline_ok ? "true" : "false") << ","
	                << "\"measured_baseline_m\":" << jsonNumberOrNull(rtk_debug.measured_baseline_m) << ","
	                << "\"baseline_deviation_m\":" << jsonNumberOrNull(rtk_debug.baseline_deviation_m) << ","
	                << "\"baseline_soft_limit_m\":" << jsonNumberOrNull(rtk_debug.baseline_soft_limit_m) << ","
	                << "\"baseline_hard_limit_m\":" << jsonNumberOrNull(rtk_debug.baseline_hard_limit_m) << ","
	                << "\"nis\":" << jsonNumberOrNull(rtk_debug.nis) << ","
	                << "\"residual_pos_m\":" << jsonNumberOrNull(rtk_debug.residual_pos_m) << ","
	                << "\"candidate_count\":" << rtk_debug.candidate_count << ","
	                << "\"front_bad_score\":" << jsonNumberOrNull(rtk_debug.front_bad_score) << ","
	                << "\"rear_bad_score\":" << jsonNumberOrNull(rtk_debug.rear_bad_score) << ","
		                << "\"rtk_weight\":" << jsonNumberOrNull(rtk_debug.rtk_weight) << ","
		                << "\"imu_weight\":" << jsonNumberOrNull(rtk_debug.imu_weight) << ","
		                << "\"encoder_weight\":" << jsonNumberOrNull(rtk_debug.encoder_weight) << ","
		                << "\"encoder_ticks_vel_valid\":" << (ticks_vel_valid ? "true" : "false") << ","
		                << "\"encoder_ticks_left_mps\":" << jsonNumberOrNull(ticks_left_mps) << ","
		                << "\"encoder_ticks_right_mps\":" << jsonNumberOrNull(ticks_right_mps) << ","
		                << "\"encoder_ticks_dt_s\":" << jsonNumberOrNull(ticks_dt_s) << ","
		                << "\"rtk_heading_weight\":" << jsonNumberOrNull(rtk_debug.rtk_heading_weight) << ","
		                << "\"drift_detected\":" << (has_status && status.drift_detected ? "true" : "false") << ","
		                << "\"rtk_residual_avg_m\":" << (has_status ? jsonNumberOrNull(status.rtk_residual_avg) : "null")
	                << ","
	                << "\"rtk_recent_residual_m\":"
	                << (has_status ? jsonNumberOrNull(status.rtk_recent_residual) : "null") << ","
	                << "\"consistency_warning\":"
	                << (has_status && status.consistency_warning ? "true" : "false") << ","
		                << "\"encoder_mismatch_avg_mps\":"
		                << (has_status ? jsonNumberOrNull(status.encoder_mismatch_avg) : "null") << ","
		                << "\"wheel_slip_detected\":"
		                << (has_status && status.wheel_slip_detected ? "true" : "false") << ","
		                << "\"wheel_slip_score\":"
		                << (has_status ? jsonNumberOrNull(status.wheel_slip_score) : "null") << ","
		                << "\"rtk_soft_quality_gate\":" << (rtk_soft_quality_gate ? "true" : "false") << ","
		                << "\"front_quality_ok\":" << (front_quality_ok ? "true" : "false") << ","
		                << "\"rear_quality_ok\":" << (rear_quality_ok ? "true" : "false") << ","
		                << "\"baseline_degraded\":" << (baseline_degraded ? "true" : "false") << ","
		                << "\"baseline_severity\":" << jsonNumberOrNull(baseline_severity) << ","
		                << "\"output_rtk_reliable\":" << (rtk_reliable ? "true" : "false") << ","
	                << "\"vision_used\":" << (vision_used ? "true" : "false") << ","
	                << "\"vision_lateral_correction_m\":" << jsonNumberOrNull(lateral_correction) << ","
	                << "\"vision_heading_correction_rad\":" << jsonNumberOrNull(heading_correction) << ","
	                << "\"vision_gate_passed\":" << (vision_gate_passed ? "true" : "false") << ","
	                << "\"vision_nis\":" << (has_vision_debug ? jsonNumberOrNull(vision_debug.nis) : "null") << ","
	                << "\"vision_residual_pos_m\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.residual_pos_m) : "null") << ","
	                << "\"vision_residual_along_m\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.residual_along_m) : "null") << ","
	                << "\"vision_residual_lateral_m\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.residual_lateral_m) : "null") << ","
	                << "\"vision_residual_heading_rad\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.residual_heading_rad) : "null") << ","
	                << "\"vision_pos_var_lateral_m2\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.pos_var_lateral) : "null") << ","
	                << "\"vision_pos_var_along_m2\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.pos_var_along) : "null") << ","
	                << "\"vision_heading_var_rad2\":"
	                << (has_vision_debug ? jsonNumberOrNull(vision_debug.heading_var) : "null") << ","
	                << "\"vision_age_s\":" << jsonNumberOrNull(vision_stats.last_age_s) << ","
	                << "\"vision_confidence\":" << jsonNumberOrNull(vision_stats.last_confidence)
	                << "}";
	            dbg_msg.data = oss.str();
	            rtk_debug_publisher.publish(dbg_msg);
	        }

	        if (bootstrap_pending && rtk_reliable) {
	            bootstrap_frame_count++;
	            if (bootstrap_frame_count >= bootstrap_required_frames) {
	                bootstrap_pending = false;
                if (auto ukf_ptr = dynamic_cast<UKFFusion*>(fusion.get())) {
                    ukf_ptr->resetHealthMonitor();
                }
                raw_output_state.last_reliable_result = stabilized_result;
                raw_output_state.has_reliable_result = true;
            }
        }

		        const FusionResult& centered_input =
		            (vision_used || vision_config.apply_post_correction) ? centered_result : result;
		        if (straight_pos_smoothing_enabled) {
		            if (raw_output_state.has_straight_frame) {
		                centered_output_state.straight_dir_x = raw_output_state.straight_dir_x;
		                centered_output_state.straight_dir_y = raw_output_state.straight_dir_y;
		                centered_output_state.straight_origin_x = raw_output_state.straight_origin_x;
		                centered_output_state.straight_origin_y = raw_output_state.straight_origin_y;
		                centered_output_state.has_straight_frame = true;
		                centered_output_state.straight_travel_m = raw_output_state.straight_travel_m;
		                centered_output_state.straight_dir_locked = raw_output_state.straight_dir_locked;
		                centered_output_state.last_straight_time = raw_output_state.last_straight_time;
		                centered_output_state.has_last_straight_time = raw_output_state.has_last_straight_time;
		            } else {
		                centered_output_state.has_straight_frame = false;
		                centered_output_state.has_last_straight_time = false;
		            }
		        }
		        FusionResult centered_smoothed_result = stabilizeCenteredOutput(
		            centered_input,
		            rtk_reliable,
		            yaw_reliable,
		            has_status ? &status : nullptr,
		            encoder_speed_metric,
		            encoder_data.valid,
		            encoder_stationary,
		            force_position_hold,
		            baseline_severity);

	        ros::Time odom_stamp = current_time;
	        publishOdometryMessage(result, odom_ukf_raw_publisher, odom_stamp);
	        publishOdometryMessage(centered_input, odom_ukf_centered_input_publisher, odom_stamp);
	        publishOdometryMessage(stabilized_result, odom_raw_publisher, odom_stamp);
	        publishOdometryMessage(centered_smoothed_result, odom_centered_publisher, odom_stamp);
	        if (network_sender) {
	            network_sender->addFusionResult(centered_smoothed_result);
	        }

        if (has_status) {
            if (status.drift_detected) {
                ROS_WARN_STREAM_THROTTLE(1.0, "High average RTK residual, drift warning. avg_residual="
                                          << status.rtk_residual_avg << " m");
            }
            if (status.consistency_warning) {
                ROS_WARN_STREAM_THROTTLE(1.0, "Encoder consistency warning. avg_speed_mismatch="
                                          << status.encoder_mismatch_avg << " m/s");
            }
        }
    } else {
        ROS_WARN_THROTTLE(1.0, "Fusion result invalid.");
    }
}


void signalHandler(int signum) {
    std::cout << "Received signal " << signum << std::endl;

    running = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_fusion_node");

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_public;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::vector<std::string> nodes;
    if (ros::master::getNodes(nodes)) {
        int count = 0;
        std::string this_node_name = ros::this_node::getName();
        for (const auto& node : nodes) {
            if (node == this_node_name || node.find("/fusion_node") != std::string::npos) {
                count++;
            }
        }
        if (count > 1) {
            std::cerr << "Another fusion_node is already running. Exiting to avoid conflict." << std::endl;
            std::cerr << "Current running nodes: " << count << std::endl;
            return -1;
        }
    }

    std::string param_file;
    if (!nh.getParam("param_file", param_file)) {
        param_file = "config/params.yaml";
    }

    std::cout << "Using parameter file: " << param_file << std::endl;

    Json::Value params;
    if (!loadParameters(param_file, params)) {
        return -1;
    }

    global_params = params;
    configureVisionConstraint(params);

    nh.param<std::string>("odom_frame", odom_frame_id, odom_frame_id);
    nh.param<std::string>("base_frame", base_frame_id, base_frame_id);
    std::string raw_odom_topic = "/sensor_fusion/odom_raw";
    nh.param<std::string>("raw_odom_topic", raw_odom_topic, raw_odom_topic);
    std::string centered_odom_topic = "/sensor_fusion/odom_centered";
    nh.param<std::string>("centered_odom_topic", centered_odom_topic, centered_odom_topic);
    std::string ukf_raw_odom_topic = "/sensor_fusion/odom_ukf_raw";
    nh.param<std::string>("ukf_raw_odom_topic", ukf_raw_odom_topic, ukf_raw_odom_topic);
    std::string ukf_centered_input_odom_topic = "/sensor_fusion/odom_ukf_centered_input";
    nh.param<std::string>("ukf_centered_input_odom_topic",
                          ukf_centered_input_odom_topic,
                          ukf_centered_input_odom_topic);
    if (vision_config.enabled) {
        std::string measurement_topic_param = vision_config.topic;
        nh.param<std::string>("vision_measurement_topic", measurement_topic_param, measurement_topic_param);
        vision_config.topic = measurement_topic_param;
    }

    if (params.isMember("imu") && params["imu"].isMember("topic")) {
        imu_topic = params["imu"]["topic"].asString();
    }
    nh.param<std::string>("imu_topic", imu_topic, imu_topic);

    if (params.isMember("fusion") && params["fusion"].isMember("predict_odom_topic")) {
        predict_odom_topic = params["fusion"]["predict_odom_topic"].asString();
    }
    nh.param<std::string>("predict_odom_topic", predict_odom_topic, predict_odom_topic);

    odom_raw_publisher = nh_public.advertise<nav_msgs::Odometry>(raw_odom_topic, 10);
    odom_centered_publisher = nh_public.advertise<nav_msgs::Odometry>(centered_odom_topic, 10);
    odom_ukf_raw_publisher = nh_public.advertise<nav_msgs::Odometry>(ukf_raw_odom_topic, 10);
    odom_ukf_centered_input_publisher =
        nh_public.advertise<nav_msgs::Odometry>(ukf_centered_input_odom_topic, 10);
    odom_predict_publisher = nh_public.advertise<nav_msgs::Odometry>(predict_odom_topic, 50);
    rtk_debug_publisher = nh_public.advertise<std_msgs::String>("/sensor_fusion/rtk_debug", 10);

    if (params.isMember("fusion")) {
        const auto& fusion_cfg = params["fusion"];
        use_hardware_sources = fusion_cfg.get("use_hardware_sources", true).asBool();
        use_network_sender = fusion_cfg.get("enable_network_sender", true).asBool();
    } else {
        use_hardware_sources = true;
        use_network_sender = true;
    }

    if (!initSensors(params, nh)) {
        return -1;
    }

    if (!initFusion(params)) {
        return -1;
    }

    if (!initNetwork(params)) {
        return -1;
    }

	    if (params.isMember("rtk") && params["rtk"].isMember("accuracy_threshold")) {
	        accuracy_threshold = params["rtk"]["accuracy_threshold"].asInt();
	    }
	    if (params.isMember("rtk") && params["rtk"].isMember("min_satellites")) {
	        min_satellites = params["rtk"]["min_satellites"].asInt();
	    }
	    if (params.isMember("rtk")) {
	        rtk_soft_quality_gate = params["rtk"].get("soft_quality_gate", rtk_soft_quality_gate).asBool();
	    }

    if (front_rtk_parser) {
        front_rtk_parser->start();
    }
    if (rear_rtk_parser) {
        rear_rtk_parser->start();
    }
    if (encoder_handler) {
        encoder_handler->start();
    }
    if (network_sender) {
        network_sender->start();
    }

	    if (vision_config.enabled) {
	        vision_subscriber = nh_public.subscribe<sensor_fusion::VisionMeasurement>(
	            vision_config.topic, 10, visionMeasurementCallback);
	        ROS_INFO_STREAM("Vision constraint enabled, subscribing: " << vision_config.topic);
	        vision_stats_publisher = nh_public.advertise<std_msgs::String>("/sensor_fusion/vision_stats", 10);
	        vision_stats_timer = nh_public.createTimer(ros::Duration(1.0), publishVisionStats);
	    } else {
	        ROS_INFO("Vision constraint disabled or not configured.");
	    }

    message_filters::Subscriber<RtkMsg> front_rtk_sub(nh, "/fusion_node/front_rtk", 10);
    message_filters::Subscriber<RtkMsg> rear_rtk_sub(nh, "/fusion_node/rear_rtk", 10);
    message_filters::Subscriber<EncoderMsg> encoder_sub(nh, "encoder", 10);

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), front_rtk_sub, rear_rtk_sub, encoder_sub);

    sync.getPolicy()->setMaxIntervalDuration(ros::Duration(0.2));

    sync.registerCallback(boost::bind(&syncCallback, _1, _2, _3));

    imu_subscriber = nh_public.subscribe<sensor_msgs::Imu>(imu_topic, 200, imuCallback);
    ROS_INFO_STREAM("IMU topic: " << imu_topic);


    std::cout << "Sensor fusion node started." << std::endl;

    ros::spin();

    if (front_rtk_parser) {
        front_rtk_parser->stop();
    }
    if (rear_rtk_parser) {
        rear_rtk_parser->stop();
    }
    if (encoder_handler) {
        encoder_handler->stop();
    }
    if (network_sender) {
        network_sender->stop();
    }

    std::cout << "Sensor fusion node stopped." << std::endl;

    return 0;
}
