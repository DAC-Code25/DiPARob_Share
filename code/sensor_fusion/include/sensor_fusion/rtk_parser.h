#ifndef SENSOR_FUSION_RTK_PARSER_H
#define SENSOR_FUSION_RTK_PARSER_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>

#include "sensor_fusion/Rtk.h"

class RTKParser {
public:
    RTKParser(ros::NodeHandle& nh, const std::string& device, int baudrate, const std::string& topic);

    ~RTKParser();

    bool init();

    void start();

    void stop();

    bool isHealthy() const;

    void setUseSystemTime(bool use_system_time);

private:
    int fd_;

    std::string device_;

    int baudrate_;

    bool running_;

    std::string buffer_;

    ros::NodeHandle& nh_;

    ros::Publisher rtk_pub_;

    std::string topic_;

    std::thread read_thread_;

    bool use_system_time_;
    double time_offset_;
    bool time_offset_initialized_;

    void readAndPublish();

    bool parseNMEA(const std::string& sentence, sensor_fusion::Rtk& msg);

    unsigned char calculateChecksum(const std::string& sentence);

    bool configurePort();

    void closePort();

    speed_t convertBaudRate(int baudrate);
};

#endif
