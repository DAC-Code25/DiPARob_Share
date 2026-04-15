#include "sensor_fusion/rtk_parser.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>

#include <cstring>
#include <cerrno>
#include <ctime>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RTKParser::RTKParser(ros::NodeHandle& nh, const std::string& device, int baudrate, const std::string& topic)
    : fd_(-1)
    , device_(device)
    , baudrate_(baudrate)
    , running_(false)
    , nh_(nh)
    , topic_(topic)
{
    rtk_pub_ = nh_.advertise<sensor_fusion::Rtk>(topic_, 10);
}

RTKParser::~RTKParser() {
    stop();

    closePort();
}

bool RTKParser::init() {
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd_ == -1) {
        std::cerr << "Failed to open serial device: " << device_ << ", error: " << strerror(errno) << std::endl;
        return false;
    }

    if (!configurePort()) {
        closePort();
        return false;
    }

    return true;
}

void RTKParser::start() {
    running_ = true;
    read_thread_ = std::thread(&RTKParser::readAndPublish, this);
}

void RTKParser::stop() {
    running_ = false;

    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

bool RTKParser::isHealthy() const {
    return fd_ != -1;
}

void RTKParser::readAndPublish() {
#define MAX_BUFFER_SIZE 4096
    char read_buffer[MAX_BUFFER_SIZE];
    while (running_) {
        ssize_t bytes_read = read(fd_, read_buffer, MAX_BUFFER_SIZE - 1);
        if (bytes_read >= MAX_BUFFER_SIZE - 1) {
            std::cerr << "Warning: read size close to buffer limit" << std::endl;
        }
        if (bytes_read > 0) {
            read_buffer[bytes_read] = '\0';
            buffer_ += read_buffer;

            size_t pos_start = 0;
            size_t pos_end;
            while ((pos_end = buffer_.find("\r\n", pos_start)) != std::string::npos) {
                std::string sentence = buffer_.substr(pos_start, pos_end - pos_start);

                sensor_fusion::Rtk msg;
                if (parseNMEA(sentence, msg)) {
                    rtk_pub_.publish(msg);
                }

                pos_start = pos_end + 2;
            }

            if (pos_start > 0) {
                buffer_.erase(0, pos_start);
            }
        } else if (bytes_read < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                std::cerr << "Error reading serial data: " << strerror(errno) << std::endl;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

bool RTKParser::parseNMEA(const std::string& sentence, sensor_fusion::Rtk& msg) {
    if (sentence.length() < 6 || sentence.substr(0, 6) != "$QFPOS") {
        std::cerr << "Warning: not a QFPOS sentence" << std::endl;
        msg.valid = false;
        return false;
    }

    size_t checksum_pos = sentence.find('*');
    if (checksum_pos == std::string::npos) {
        std::cerr << "Warning: invalid format, missing checksum separator '*'" << std::endl;
        msg.valid = false;
        return false;
    }

    std::string data_part = sentence.substr(1, checksum_pos - 1);
    std::string checksum_str = sentence.substr(checksum_pos + 1);

    unsigned char calculated_checksum = calculateChecksum(data_part);

    unsigned char received_checksum;
    try {
        received_checksum = static_cast<unsigned char>(std::stoi(checksum_str, nullptr, 16));
    } catch (const std::exception& e) {
        std::cerr << "Warning: failed to parse checksum: " << checksum_str << std::endl;
        msg.valid = false;
        return false;
    }

    if (calculated_checksum != received_checksum) {
        std::cerr << "Warning: checksum mismatch, calculated: " << std::hex << static_cast<int>(calculated_checksum)
                  << ", received: " << static_cast<int>(received_checksum) << std::dec << std::endl;
        msg.valid = false;
        return false;
    }

    size_t start = 0;
    size_t end = 0;
    int field_index = 0;

    std::string timestamp_str;
    std::string latitude_str;
    std::string longitude_str;
    std::string fix_mode_str;
    std::string satellite_count_str;

    while (end != std::string::npos && field_index <= 16) {
        end = data_part.find(',', start);
        std::string field;
        if (end == std::string::npos) {
            field = data_part.substr(start);
        } else {
            field = data_part.substr(start, end - start);
        }

        if (field_index == 1) {
            timestamp_str = field;
        } else if (field_index == 2) {
            latitude_str = field;
        } else if (field_index == 4) {
            longitude_str = field;
        } else if (field_index == 13) {
            fix_mode_str = field;
        } else if (field_index == 14) {
            satellite_count_str = field;
        }

        field_index++;
        start = end + 1;
    }

    msg.valid = false;

    if (latitude_str.empty() || longitude_str.empty() ||
        latitude_str == "00000000000000.00" || longitude_str == "000.0000000" ||
        satellite_count_str.empty() || satellite_count_str == "0") {
        std::cerr << "Warning: invalid data, key fields missing or zero: " << sentence << std::endl;
    }

    msg.header.stamp = ros::Time::now();

    if (!latitude_str.empty()) {
        try {
            msg.latitude = std::stod(latitude_str);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to parse latitude: " << latitude_str << std::endl;
            msg.latitude = 0.0;
        }
    } else {
        msg.latitude = 0.0;
    }

    if (!longitude_str.empty()) {
        try {
            msg.longitude = std::stod(longitude_str);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to parse longitude: " << longitude_str << std::endl;
            msg.longitude = 0.0;
        }
    } else {
        msg.longitude = 0.0;
    }

    if (!fix_mode_str.empty()) {
        try {
            msg.fix_mode = std::stoi(fix_mode_str);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to parse fix mode: " << fix_mode_str << std::endl;
            msg.fix_mode = 0;
        }
    } else {
        msg.fix_mode = 0;
    }

    if (!satellite_count_str.empty()) {
        try {
            msg.satellite_count = std::stoi(satellite_count_str);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to parse satellite count: " << satellite_count_str << std::endl;
            msg.satellite_count = 0;
        }
    } else {
        msg.satellite_count = 0;
    }

    if (!latitude_str.empty() && !longitude_str.empty() &&
        latitude_str != "00000000000000.00" && longitude_str != "000.0000000" &&
        !satellite_count_str.empty() && satellite_count_str != "0") {
        msg.valid = true;
    }

    return true;
}

unsigned char RTKParser::calculateChecksum(const std::string& sentence) {
    unsigned char checksum = 0;

    const char* data = sentence.c_str();
    size_t len = sentence.length();

    for (size_t i = 0; i < len; ++i) {
        checksum ^= static_cast<unsigned char>(data[i]);
    }

    return checksum;
}

speed_t RTKParser::convertBaudRate(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default: return B115200;
    }
}

bool RTKParser::configurePort() {
    struct termios tty;

    if (tcgetattr(fd_, &tty) != 0) {
        std::cerr << "Failed to get serial attributes: " << strerror(errno) << std::endl;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    cfmakeraw(&tty);

    speed_t baudrate_termios = convertBaudRate(baudrate_);
    cfsetispeed(&tty, baudrate_termios);
    cfsetospeed(&tty, baudrate_termios);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;

    tty.c_cflag &= ~CSTOPB;

    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;

    tty.c_lflag &= ~ECHO;

    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_iflag &= ~(CRTSCTS);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "Failed to set serial attributes: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

void RTKParser::closePort() {
    if (fd_ != -1) {
        close(fd_);

        fd_ = -1;
    }
}
