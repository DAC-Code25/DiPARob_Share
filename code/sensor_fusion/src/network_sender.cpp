#include "sensor_fusion/network_sender.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

#include <cstring>

#include <ros/ros.h>

namespace {
struct GeoCoord {
    double lon = 0.0;
    double lat = 0.0;
};

constexpr double kPi = 3.14159265358979323846;
constexpr double kXPi = kPi * 3000.0 / 180.0;
constexpr double kEe = 0.00669342162296594323;
constexpr double kA = 6378245.0;

constexpr double kLlBand[] = {75.0, 60.0, 45.0, 30.0, 15.0, 0.0};
constexpr double kLl2Mc[][10] = {
    {-0.0015702102444, 111320.7020616939, 1704480524535203.0, -10338987376042340.0,
     26112667856603880.0, -35149669176653700.0, 26595700718403920.0, -10725012454188240.0,
     1800819912950474.0, 82.5},
    {0.0008277824516172526, 111320.7020463578, 647795574.6671607, -4082003173.641316,
     10774905663.51142, -15171875531.51559, 12053065338.62167, -5124939663.577472,
     913311935.9512032, 67.5},
    {0.00337398766765, 111320.7020202162, 4481351.045890365, -23393751.19931662,
     79682215.47186455, -115964993.2797253, 97236711.15602145, -43661946.33752821,
     8477230.501135234, 52.5},
    {0.00220636496208, 111320.7020209128, 51751.86112841131, 3796837.749470245,
     992013.7397791013, -1221952.21711287, 1340652.697009075, -620943.6990984312,
     144416.9293806241, 37.5},
    {-0.0003441963504368392, 111320.7020576856, 278.2353980772752, 2485758.690035394,
     6070.750963243378, 54821.18345352118, 9540.606633304236, -2710.55326746645,
     1405.483844121726, 22.5},
    {-0.0003218135878613132, 111320.7020701615, 0.00369383431289, 823725.6402795718,
     0.46104986909093, 2351.343141331292, 1.58060784298199, 8.77738589078284,
     0.37238884252424, 7.45}
};

static double transformLat(const GeoCoord& point) {
    const double lat = point.lat - 35.0;
    const double lng = point.lon - 105.0;
    double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lat * lng + 0.2 * std::sqrt(std::fabs(lng));
    ret += (20.0 * std::sin(6.0 * lng * kPi) + 20.0 * std::sin(2.0 * lng * kPi)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(lat * kPi) + 40.0 * std::sin(lat / 3.0 * kPi)) * 2.0 / 3.0;
    ret += (160.0 * std::sin(lat / 12.0 * kPi) + 320.0 * std::sin(lat * kPi / 30.0)) * 2.0 / 3.0;
    return ret;
}

static double transformLng(const GeoCoord& point) {
    const double lat = point.lat - 35.0;
    const double lng = point.lon - 105.0;
    double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * std::sqrt(std::fabs(lng));
    ret += (20.0 * std::sin(6.0 * lng * kPi) + 20.0 * std::sin(2.0 * lng * kPi)) * 2.0 / 3.0;
    ret += (20.0 * std::sin(lng * kPi) + 40.0 * std::sin(lng / 3.0 * kPi)) * 2.0 / 3.0;
    ret += (150.0 * std::sin(lng / 12.0 * kPi) + 300.0 * std::sin(lng / 30.0 * kPi)) * 2.0 / 3.0;
    return ret;
}

static bool isOutOfChina(const GeoCoord& point) {
    return !(point.lon > 73.66 && point.lon < 135.05 && point.lat > 3.86 && point.lat < 53.55);
}

static GeoCoord wgs84ToGcj02(const GeoCoord& src) {
    if (isOutOfChina(src)) {
        return src;
    }
    GeoCoord dst = src;
    double dLat = transformLat(src);
    double dLng = transformLng(src);
    double radLat = src.lat / 180.0 * kPi;
    double magic = std::sin(radLat);
    magic = 1.0 - kEe * magic * magic;
    double sqrtMagic = std::sqrt(magic);
    dLat = (dLat * 180.0) / ((kA * (1 - kEe)) / (magic * sqrtMagic) * kPi);
    dLng = (dLng * 180.0) / (kA / sqrtMagic * std::cos(radLat) * kPi);
    dst.lat = src.lat + dLat;
    dst.lon = src.lon + dLng;
    return dst;
}

static GeoCoord gcj02ToBd09(const GeoCoord& src) {
    const double x = src.lon;
    const double y = src.lat;
    const double z = std::sqrt(x * x + y * y) + 0.00002 * std::sin(y * kXPi);
    const double theta = std::atan2(y, x) + 0.000003 * std::cos(x * kXPi);
    GeoCoord dst;
    dst.lon = z * std::cos(theta) + 0.0065;
    dst.lat = z * std::sin(theta) + 0.006;
    return dst;
}

static GeoCoord wgs84ToBd09(const GeoCoord& src) {
    return gcj02ToBd09(wgs84ToGcj02(src));
}

static double getLoop(double x, double low, double high) {
    const double dif = high - low;
    while (x > high) {
        x -= dif;
    }
    while (x < low) {
        x += dif;
    }
    return x;
}

static double getRange(double x, double low, double high) {
    return x > high ? high : (x < low ? low : x);
}

static GeoCoord converterToMc(const GeoCoord& pos, const double p[10]) {
    double xTemp = p[0] + p[1] * std::fabs(pos.lon);
    double cC = std::fabs(pos.lat) / p[9];
    double yTemp = p[2];
    for (int i = 1; i <= 6; ++i) {
        yTemp += p[2 + i] * std::pow(cC, i);
    }
    xTemp *= (pos.lon < 0 ? -1 : 1);
    yTemp *= (pos.lat < 0 ? -1 : 1);
    return GeoCoord{xTemp, yTemp};
}

static GeoCoord bd09ToMc(const GeoCoord& src) {
    if (isOutOfChina(src)) {
        return GeoCoord{};
    }
    double lng = getLoop(src.lon, -180.0, 180.0);
    double lat = getRange(src.lat, -74.0, 74.0);
    const double* p = nullptr;
    for (int i = 0; i < static_cast<int>(sizeof(kLlBand) / sizeof(kLlBand[0])); ++i) {
        if (lat >= kLlBand[i]) {
            p = kLl2Mc[i];
            break;
        }
    }
    if (!p) {
        for (int i = static_cast<int>(sizeof(kLlBand) / sizeof(kLlBand[0])) - 1; i >= 0; --i) {
            if (lat <= -kLlBand[i]) {
                p = kLl2Mc[i];
                break;
            }
        }
    }
    if (!p) {
        return GeoCoord{};
    }
    return converterToMc(GeoCoord{lng, lat}, p);
}

static Json::Value jsonNumberOrNull(double value) {
    if (!std::isfinite(value)) {
        return Json::Value();
    }
    return Json::Value(value);
}

static Json::Value fusionResultToJsonValue(const FusionResult& result) {
    Json::Value root(Json::objectValue);
    root["timestamp"] = jsonNumberOrNull(result.timestamp);
    root["valid"] = result.valid;

    Json::Value pose(Json::objectValue);
    pose["x"] = jsonNumberOrNull(result.x);
    pose["y"] = jsonNumberOrNull(result.y);
    pose["theta"] = jsonNumberOrNull(result.theta);
    root["pose"] = pose;

    Json::Value twist(Json::objectValue);
    twist["vx"] = jsonNumberOrNull(result.vx);
    twist["vy"] = jsonNumberOrNull(result.vy);
    twist["vtheta"] = jsonNumberOrNull(result.vtheta);
    root["twist"] = twist;

    Json::Value acceleration(Json::objectValue);
    acceleration["ax"] = jsonNumberOrNull(result.ax);
    acceleration["ay"] = jsonNumberOrNull(result.ay);
    root["acceleration"] = acceleration;

    return root;
}
} // namespace

NetworkSender::NetworkSender(const std::string& post_url)
    : post_url_(post_url)
    , curl_(nullptr)
    , running_(false)
    , send_frequency_(50)
    , send_interval_ms_(20)
    , output_precision_(4)
{
}

NetworkSender::~NetworkSender() {
    stop();

    cleanupCURL();
}

bool NetworkSender::init() {
    if (!initCURL()) {
        std::cerr << "Failed to initialize CURL." << std::endl;
        return false;
    }

    return true;
}

void NetworkSender::start() {
    running_ = true;

    send_thread_ = std::thread(&NetworkSender::sendThread, this);
}

void NetworkSender::stop() {
    running_ = false;

    queue_cv_.notify_all();

    if (send_thread_.joinable()) {
        send_thread_.join();
    }
}

void NetworkSender::addFusionResult(const FusionResult& result) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    while (!result_queue_.empty()) {
        result_queue_.pop();
    }

    result_queue_.push(result);

    queue_cv_.notify_one();
}

void NetworkSender::setSendFrequency(int frequency) {
    send_frequency_ = frequency;

    send_interval_ms_ = 1000 / frequency;
}

void NetworkSender::setOutputPrecision(int precision) {
    output_precision_ = precision;
}

void NetworkSender::setOutputPrediction(bool enabled, double max_dt_s) {
    output_predict_enabled_ = enabled;
    if (!std::isfinite(max_dt_s) || max_dt_s <= 0.0) {
        output_predict_max_dt_s_ = 0.0;
    } else {
        output_predict_max_dt_s_ = std::clamp(max_dt_s, 0.01, 0.5);
    }
}

void NetworkSender::setOutputSmoothing(bool enabled, double alpha) {
    output_smooth_enabled_ = enabled;
    if (!std::isfinite(alpha)) {
        output_smooth_alpha_ = 0.85;
        return;
    }
    output_smooth_alpha_ = std::clamp(alpha, 0.0, 1.0);
}

void NetworkSender::setMapOutputConfig(const MapOutputConfig& cfg) {
    map_output_cfg_ = cfg;
    map_origin_mc_valid_ = false;
    enu_converter_ready_ = false;

    if (!map_output_cfg_.enabled) {
        return;
    }

    if (!std::isfinite(map_output_cfg_.enu_origin_lat) ||
        !std::isfinite(map_output_cfg_.enu_origin_lon)) {
        map_output_cfg_.enabled = false;
        return;
    }

    enu_converter_.Reset(map_output_cfg_.enu_origin_lat, map_output_cfg_.enu_origin_lon, 0.0);
    enu_converter_ready_ = true;

    const double origin_lat = map_output_cfg_.has_map_origin
        ? map_output_cfg_.map_origin_lat
        : map_output_cfg_.enu_origin_lat;
    const double origin_lon = map_output_cfg_.has_map_origin
        ? map_output_cfg_.map_origin_lon
        : map_output_cfg_.enu_origin_lon;

    if (!std::isfinite(origin_lat) || !std::isfinite(origin_lon)) {
        map_output_cfg_.enabled = false;
        return;
    }

    GeoCoord origin{origin_lon, origin_lat};
    if (map_output_cfg_.input_is_wgs84) {
        origin = wgs84ToBd09(origin);
    }
    GeoCoord origin_mc = bd09ToMc(origin);
    map_origin_mc_x_ = origin_mc.lon;
    map_origin_mc_y_ = origin_mc.lat;
    map_origin_mc_valid_ = std::isfinite(map_origin_mc_x_) && std::isfinite(map_origin_mc_y_);
    if (!map_origin_mc_valid_) {
        map_output_cfg_.enabled = false;
    }
}

bool NetworkSender::isQueueEmpty() {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    return result_queue_.empty();
}

void NetworkSender::sendThread() {
    auto last_send_time = std::chrono::steady_clock::now();

    while (running_) {
        FusionResult result;
        bool has_data = false;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!result_queue_.empty()) {
                result = result_queue_.front();
                result_queue_.pop();
                has_data = true;
            }
        }

        if (has_data) {
            FusionResult send_result = prepareSendResult(result);
            sendPOSTRequest(send_result);

            auto current_time = std::chrono::steady_clock::now();

            auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_send_time).count();

            if (elapsed_time < send_interval_ms_) {
                auto wait_time = send_interval_ms_ - elapsed_time;
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
            }

            last_send_time = std::chrono::steady_clock::now();
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

static double normalizeAngleRad(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

static double unwrapAngleNear(double reference_unwrapped, double angle_wrapped) {
    if (!std::isfinite(reference_unwrapped) || !std::isfinite(angle_wrapped)) {
        return angle_wrapped;
    }
    double d = normalizeAngleRad(angle_wrapped - reference_unwrapped);
    return reference_unwrapped + d;
}

FusionResult NetworkSender::prepareSendResult(const FusionResult& result) {
    FusionResult out = result;

    if (output_predict_enabled_ && output_predict_max_dt_s_ > 1e-6 && std::isfinite(out.timestamp)) {
        const double now = ros::Time::now().toSec();
        double dt = now - out.timestamp;
        if (!std::isfinite(dt) || dt <= 0.0) {
            dt = 0.0;
        }
        dt = std::min(dt, output_predict_max_dt_s_);

        if (dt > 1e-6) {
            if (std::isfinite(out.vx)) {
                out.x += out.vx * dt;
            }
            if (std::isfinite(out.vy)) {
                out.y += out.vy * dt;
            }
            if (std::isfinite(out.theta) && std::isfinite(out.vtheta)) {
                out.theta = normalizeAngleRad(out.theta + out.vtheta * dt);
            } else if (std::isfinite(out.theta)) {
                out.theta = normalizeAngleRad(out.theta);
            }
        }
    }

    if (map_output_cfg_.enabled && enu_converter_ready_ && map_origin_mc_valid_) {
        double lat = 0.0, lon = 0.0, h = 0.0;
        enu_converter_.Reverse(out.x, out.y, 0.0, lat, lon, h);

        GeoCoord coord{lon, lat};
        if (map_output_cfg_.input_is_wgs84) {
            coord = wgs84ToBd09(coord);
        }
        GeoCoord mc = bd09ToMc(coord);
        double east = mc.lon - map_origin_mc_x_;
        double north = mc.lat - map_origin_mc_y_;

        double base_x = map_output_cfg_.swap_xy ? north : east;
        double base_y = map_output_cfg_.swap_xy ? east : north;

        double x = map_output_cfg_.xdir * base_x;
        double y = map_output_cfg_.ydir * base_y;

        if (std::isfinite(out.theta)) {
            const double ve = std::cos(out.theta);
            const double vn = std::sin(out.theta);
            const double v_map_x = map_output_cfg_.swap_xy ? (map_output_cfg_.xdir * vn)
                                                           : (map_output_cfg_.xdir * ve);
            const double v_map_y = map_output_cfg_.swap_xy ? (map_output_cfg_.ydir * ve)
                                                           : (map_output_cfg_.ydir * vn);
            double theta = std::atan2(v_map_y, v_map_x);
            theta *= map_output_cfg_.thetadir;

            x += std::cos(theta) * map_output_cfg_.pose_x - std::sin(theta) * map_output_cfg_.pose_y;
            y += std::sin(theta) * map_output_cfg_.pose_x + std::cos(theta) * map_output_cfg_.pose_y;
            theta = normalizeAngleRad(theta + map_output_cfg_.pose_theta);
            out.theta = theta;
        } else {
            x += map_output_cfg_.pose_x;
            y += map_output_cfg_.pose_y;
        }

        out.x = x;
        out.y = y;
    }

    if (output_smooth_enabled_ && has_last_sent_) {
        const double a = std::clamp(output_smooth_alpha_, 0.0, 1.0);
        if (std::isfinite(out.x) && std::isfinite(last_sent_result_.x)) {
            out.x = a * out.x + (1.0 - a) * last_sent_result_.x;
        }
        if (std::isfinite(out.y) && std::isfinite(last_sent_result_.y)) {
            out.y = a * out.y + (1.0 - a) * last_sent_result_.y;
        }

        if (std::isfinite(out.theta)) {
            const double prev = has_last_sent_unwrapped_theta_
                ? last_sent_unwrapped_theta_
                : (std::isfinite(last_sent_result_.theta) ? last_sent_result_.theta : out.theta);
            const double cur_unwrapped = unwrapAngleNear(prev, normalizeAngleRad(out.theta));
            const double blended_unwrapped = a * cur_unwrapped + (1.0 - a) * prev;
            last_sent_unwrapped_theta_ = blended_unwrapped;
            has_last_sent_unwrapped_theta_ = true;
            out.theta = normalizeAngleRad(blended_unwrapped);
        }
    } else {
        if (std::isfinite(out.theta)) {
            last_sent_unwrapped_theta_ = normalizeAngleRad(out.theta);
            has_last_sent_unwrapped_theta_ = true;
        }
    }

    last_sent_result_ = out;
    has_last_sent_ = true;
    return out;
}

bool NetworkSender::sendPOSTRequest(const FusionResult& result) {
    if (!curl_) {
        return false;
    }
    curl_easy_setopt(curl_, CURLOPT_URL, post_url_.c_str());

    curl_easy_setopt(curl_, CURLOPT_POST, 1L);

    std::string json_data = fusionResultToJSON(result);

    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_data.c_str());

    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_data.length());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);

    std::string response;
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl_);

    curl_slist_free_all(headers);

    if (res != CURLE_OK) {
        std::cerr << "HTTP POST request failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    return true;
}

bool NetworkSender::sendBatchPOSTRequest(const std::vector<FusionResult>& results) {
    if (!curl_) {
        return false;
    }

    if (results.empty()) {
        return true;
    }

    curl_easy_setopt(curl_, CURLOPT_URL, post_url_.c_str());

    curl_easy_setopt(curl_, CURLOPT_POST, 1L);

    std::string json_data = fusionBatchResultToJSON(results);

    curl_easy_setopt(curl_, CURLOPT_POSTFIELDS, json_data.c_str());

    curl_easy_setopt(curl_, CURLOPT_POSTFIELDSIZE, json_data.length());

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl_, CURLOPT_HTTPHEADER, headers);

    std::string response;
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl_);

    curl_slist_free_all(headers);

    if (res != CURLE_OK) {
        std::cerr << "HTTP batch POST request failed: " << curl_easy_strerror(res) << std::endl;
        return false;
    }

    return true;
}

std::string NetworkSender::fusionResultToJSON(const FusionResult& result) {
    double theta = result.theta;
    if (std::isfinite(theta)) {
        theta = std::atan2(std::sin(theta), std::cos(theta));
    }

    std::cout << "[FUSION_OUTPUT] Timestamp: " << std::fixed << std::setprecision(6) << result.timestamp
              << " | X: " << std::fixed << std::setprecision(output_precision_) << result.x
              << " | Y: " << std::fixed << std::setprecision(output_precision_) << result.y
              << " | Heading: " << std::fixed << std::setprecision(output_precision_) << theta
              << " rad (" << std::fixed << std::setprecision(2) << (theta * 180.0 / M_PI) << " deg)"
              << std::endl;
    FusionResult normalized = result;
    normalized.theta = theta;
    Json::Value root = fusionResultToJsonValue(normalized);

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";

    return Json::writeString(builder, root);
}

std::string NetworkSender::fusionBatchResultToJSON(const std::vector<FusionResult>& results) {
    Json::Value root_array(Json::arrayValue);

    for (auto result : results) {
        double theta = result.theta;
        if (std::isfinite(theta)) {
            theta = std::atan2(std::sin(theta), std::cos(theta));
        }
        result.theta = theta;
        root_array.append(fusionResultToJsonValue(result));
    }

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";

    return Json::writeString(builder, root_array);
}

size_t NetworkSender::writeCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t realsize = size * nmemb;

    response->append((char*)contents, realsize);

    return realsize;
}

bool NetworkSender::initCURL() {
    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl_ = curl_easy_init();

    if (!curl_) {
        std::cerr << "Failed to initialize CURL handle." << std::endl;
        return false;
    }

    return true;
}

void NetworkSender::cleanupCURL() {
    if (curl_) {
        curl_easy_cleanup(curl_);
        curl_ = nullptr;
    }

    curl_global_cleanup();
}
