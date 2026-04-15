#ifndef SENSOR_FUSION_NETWORK_SENDER_H
#define SENSOR_FUSION_NETWORK_SENDER_H

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>

#include <curl/curl.h>

#include <json/json.h>

#include "sensor_fusion/base_fusion.h"

#include <GeographicLib/LocalCartesian.hpp>

class NetworkSender {
public:
    struct MapOutputConfig {
        bool enabled = false;
        bool input_is_wgs84 = true;
        bool swap_xy = true;
        double enu_origin_lat = 0.0;
        double enu_origin_lon = 0.0;
        bool has_map_origin = false;
        double map_origin_lat = 0.0;
        double map_origin_lon = 0.0;
        double xdir = 1.0;
        double ydir = 1.0;
        double thetadir = 1.0;
        double pose_x = 0.0;
        double pose_y = 0.0;
        double pose_theta = 0.0;
    };
    NetworkSender(const std::string& post_url);

    ~NetworkSender();

    bool init();

    void start();

    void stop();

    void addFusionResult(const FusionResult& result);

    void setSendFrequency(int frequency);

    void setOutputPrecision(int precision);

    void setOutputPrediction(bool enabled, double max_dt_s);
    void setOutputSmoothing(bool enabled, double alpha);

    void setMapOutputConfig(const MapOutputConfig& cfg);

    bool isQueueEmpty();

private:
    std::string post_url_;

    CURL* curl_;

    bool running_;

    int send_frequency_;

    int send_interval_ms_;

    int output_precision_;

    bool output_predict_enabled_ = false;
    double output_predict_max_dt_s_ = 0.15;
    bool output_smooth_enabled_ = false;
    double output_smooth_alpha_ = 0.85;

    bool has_last_sent_ = false;
    FusionResult last_sent_result_{};
    bool has_last_sent_unwrapped_theta_ = false;
    double last_sent_unwrapped_theta_ = 0.0;

    MapOutputConfig map_output_cfg_{};
    GeographicLib::LocalCartesian enu_converter_{};
    bool enu_converter_ready_ = false;
    double map_origin_mc_x_ = 0.0;
    double map_origin_mc_y_ = 0.0;
    bool map_origin_mc_valid_ = false;

    std::queue<FusionResult> result_queue_;

    std::mutex queue_mutex_;

    std::condition_variable queue_cv_;

    std::thread send_thread_;

    void sendThread();

    FusionResult prepareSendResult(const FusionResult& result);

    bool sendPOSTRequest(const FusionResult& result);

    bool sendBatchPOSTRequest(const std::vector<FusionResult>& results);

    std::string fusionResultToJSON(const FusionResult& result);

    std::string fusionBatchResultToJSON(const std::vector<FusionResult>& results);

    static size_t writeCallback(void* contents, size_t size, size_t nmemb, std::string* response);

    bool initCURL();

    void cleanupCURL();
};

#endif
