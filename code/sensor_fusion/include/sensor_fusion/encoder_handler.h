#ifndef SENSOR_FUSION_ENCODER_HANDLER_H
#define SENSOR_FUSION_ENCODER_HANDLER_H

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>
#include <queue>

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include <json/json.h>

#include <ros/ros.h>
#include "sensor_fusion/Encoder.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;

class EncoderHandler {
public:
    EncoderHandler(ros::NodeHandle& nh, const std::string& websocket_url, const std::string& topic);

    ~EncoderHandler();

    bool init();

    void start();

    void stop();


private:
    ros::NodeHandle& nh_;

    ros::Publisher encoder_pub_;

    std::string topic_;

    std::string websocket_url_;


    client ws_client_;

    websocketpp::connection_hdl ws_hdl_;

    std::queue<std::string> message_queue_;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;

    std::atomic<bool> connected_;

    bool running_;


    std::thread receive_thread_;

    std::thread ws_thread_;

    void receiveAndPublish();

    void wsThread();

    void on_open(websocketpp::connection_hdl hdl);
    void on_close(websocketpp::connection_hdl hdl);
    void on_fail(websocketpp::connection_hdl hdl);
    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);

    bool parseWebSocketMessage(const std::string& message, sensor_fusion::Encoder& msg);

    bool parseJSONData(const Json::Value& json_value, sensor_fusion::Encoder& msg);

    void closeConnection();
};

#endif
