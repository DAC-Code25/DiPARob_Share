#include "sensor_fusion/encoder_handler.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <regex>
#include <functional>
#include <cmath>

EncoderHandler::EncoderHandler(ros::NodeHandle& nh, const std::string& websocket_url, const std::string& topic)
    : websocket_url_(websocket_url)
    , connected_(false)
    , running_(false)
    , nh_(nh)
    , topic_(topic)
{
    encoder_pub_ = nh_.advertise<sensor_fusion::Encoder>(topic_, 1);
}

EncoderHandler::~EncoderHandler() {
    stop();
}

bool EncoderHandler::init() {
    try {
        ws_client_.clear_access_channels(websocketpp::log::alevel::all);
        ws_client_.clear_error_channels(websocketpp::log::elevel::all);

        ws_client_.init_asio();

        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::placeholders::_2;
        using websocketpp::lib::bind;

        ws_client_.set_open_handler(bind(&EncoderHandler::on_open, this, _1));
        ws_client_.set_fail_handler(bind(&EncoderHandler::on_fail, this, _1));
        ws_client_.set_close_handler(bind(&EncoderHandler::on_close, this, _1));
        ws_client_.set_message_handler(bind(&EncoderHandler::on_message, this, _1, _2));

    } catch (const std::exception & e) {
        std::cerr << "Failed to initialize WebSocket client: " << e.what() << std::endl;
        return false;
    }
    return true;
}

void EncoderHandler::start() {
    running_ = true;
    connected_ = false;

    receive_thread_ = std::thread(&EncoderHandler::receiveAndPublish, this);

    ws_thread_ = std::thread(&EncoderHandler::wsThread, this);
}

void EncoderHandler::stop() {
    running_ = false;

    queue_cv_.notify_all();

    closeConnection();

    if (ws_thread_.joinable()) {
        ws_thread_.join();
    }
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!message_queue_.empty()) {
            message_queue_.pop();
        }
    }
}

void EncoderHandler::closeConnection() {
    if (connected_.exchange(false)) {
        websocketpp::lib::error_code ec;
        ws_client_.close(ws_hdl_, websocketpp::close::status::going_away, "", ec);
        if (ec) {
            std::cerr << "Error closing WebSocket connection: " << ec.message() << std::endl;
        }
    }
}

void EncoderHandler::wsThread() {
    while(running_) {
        if (!connected_) {
            try {
                websocketpp::lib::error_code ec;
                client::connection_ptr con = ws_client_.get_connection(websocket_url_, ec);
                if (ec) {
                    std::cerr << "Connection failed: " << ec.message() << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                ws_client_.connect(con);
            } catch (const std::exception & e) {
                std::cerr << "WebSocket thread exception: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        try {
            ws_client_.run();
        } catch (const websocketpp::exception& e) {
            std::cerr << "WebSocket run exception: " << e.what() << std::endl;
            connected_ = false;
        } catch (const std::exception & e) {
            std::cerr << "WebSocket run thread exception: " << e.what() << std::endl;
            connected_ = false;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

void EncoderHandler::receiveAndPublish() {
    while (running_) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (queue_cv_.wait_for(lock, std::chrono::milliseconds(1), [this] { return !message_queue_.empty() || !running_; })) {
            while (!message_queue_.empty() && running_) {
                std::string payload = message_queue_.front();
                message_queue_.pop();
                lock.unlock();

                sensor_fusion::Encoder encoder_msg;
                if (parseWebSocketMessage(payload, encoder_msg)) {
                    if (connected_) {
                        encoder_pub_.publish(encoder_msg);
                    }
                }

                lock.lock();
            }
        }
        if (!connected_ && !message_queue_.empty()) {
            while (!message_queue_.empty()) {
                message_queue_.pop();
            }
        }
    }
}

void EncoderHandler::on_open(websocketpp::connection_hdl hdl) {
    ws_hdl_ = hdl;
    connected_ = true;
    static bool first_connect = true;
    if (first_connect) {
        std::cout << "WebSocket connected." << std::endl;
        first_connect = false;
    }

    std::string json_data = "{\"packet\":{\"cmd\":\"join\",\"region\":\"ScriptDeal\"},\"msg\":{}}";

    websocketpp::lib::error_code ec;
    ws_client_.send(ws_hdl_, json_data, websocketpp::frame::opcode::text, ec);
    if (ec) {
        std::cerr << "Failed to send encoder data request: " << ec.message() << std::endl;
    }
}

void EncoderHandler::on_fail(websocketpp::connection_hdl hdl) {
    if (connected_.exchange(false)) {
        std::cerr << "WebSocket connection failed." << std::endl;
    }
}

void EncoderHandler::on_close(websocketpp::connection_hdl hdl) {
    if (connected_.exchange(false)) {
        std::cout << "WebSocket closed." << std::endl;
    }
}

void EncoderHandler::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg) {
    if (!running_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (message_queue_.size() < 10) {
            message_queue_.push(msg->get_payload());
        } else {
            message_queue_.pop();
            message_queue_.push(msg->get_payload());
        }
    }
    queue_cv_.notify_one();
}


bool EncoderHandler::parseWebSocketMessage(const std::string& message, sensor_fusion::Encoder& msg) {
    msg.valid = false;
    msg.header.stamp = ros::Time::now();

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::istringstream s(message);
    if (!Json::parseFromStream(builder, s, &root, nullptr)) {
        return false;
    }

    if (parseJSONData(root, msg)) {
        msg.valid = true;
        return true;
    }

    return false;
}

bool EncoderHandler::parseJSONData(const Json::Value& json_value, sensor_fusion::Encoder& msg) {
    if (!json_value.isMember("msg") || !json_value["msg"].isMember("console") ||
        json_value["msg"]["console"].isNull()) {
        return false;
    }

    std::string console_msg = json_value["msg"]["console"].asString();

    msg.left_velocity = 0.0;
    msg.right_velocity = 0.0;

    std::regex encoder_regex(R"(wheelencode\s*:\s*(-?\d+\.?\d*)\s*(-?\d+\.?\d*))");
    std::smatch encoder_match;

    if (std::regex_search(console_msg, encoder_match, encoder_regex) && encoder_match.size() == 3) {
        try {
            msg.left_encoder = std::stod(encoder_match[1].str());
            msg.right_encoder = std::stod(encoder_match[2].str());
        } catch (const std::exception& e) {
            return false;
        }

        return true;
    }

    return false;
}
