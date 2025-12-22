#pragma once

#include <termios.h>
#include <filesystem>
#include <thread>
#include <rclcpp/rclcpp.hpp>

// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
class SerialPort
{
public:

    enum Baudrate
    {
        _50 = B50,
        _75 = B75,
        _110 = B110,
        _134 = B134,
        _150 = B150,
        _200 = B200,
        _300 = B300,
        _600 = B600,
        _1200 = B1200,
        _1800 = B1800,
        _2400 = B2400,
        _4800 = B4800,
        _9600 = B9600,
        _19200 = B19200,
        _38400 = B38400,
    };

    typedef std::function<void(const std::vector<uint8_t>&)> DataCallback;

    SerialPort(
        const rclcpp::Node* node, 
        const std::filesystem::path& filepath, 
        const Baudrate& baudrate,
        const DataCallback& data_callback);

    ~SerialPort();

private:

    void run();
    void read();
    void open();

    const rclcpp::Node* _node;
    std::filesystem::path _filepath;
    Baudrate _baudrate;
    DataCallback _data_callback;

    std::atomic_bool _deleted = false;
    std::thread _thread;

    int _fd = -1;
    int _event_fd = -1;
};