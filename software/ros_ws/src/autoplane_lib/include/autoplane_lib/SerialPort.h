#pragma once

#include <termios.h>
#include <filesystem>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <optional>

class SerialPort
{
public:

    typedef std::function<void(const std::vector<uint8_t>&)> DataCallback;

    struct ControlSignals
    {
        bool dtr;
        bool rts;
    };

    SerialPort(
        const rclcpp::Node* node, 
        const std::filesystem::path& filepath, 
        const uint32_t& baudrate,
        const std::optional<ControlSignals> control_signals,
        const DataCallback& data_callback);

    ~SerialPort();

private:

    void run();
    void read();
    void open();

    const rclcpp::Node* _node;
    std::filesystem::path _filepath;
    uint32_t _baudrate;
    DataCallback _data_callback;

    std::atomic_bool _deleted = false;
    std::thread _thread;

    std::optional<ControlSignals> _control_signals;

    int _fd = -1;
    int _event_fd = -1;
};