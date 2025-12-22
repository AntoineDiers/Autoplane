#include "SerialPort.h"

#include <sys/eventfd.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>

#define BUFFER_SIZE 256

SerialPort::SerialPort(
        const rclcpp::Node* node, 
        const std::filesystem::path& filepath, 
        const Baudrate& baudrate,
        const DataCallback& data_callback)
    {
        _node = node;
        _filepath = filepath;
        _baudrate = baudrate;
        _data_callback = data_callback;
        _event_fd = eventfd(0,0);
        if(_event_fd < 0)
        {
            throw std::runtime_error("Failed to create eventfd, exiting");
        }


        _thread = std::thread([this](){run();});
    }

SerialPort::~SerialPort()
{
    _deleted = true;

    uint64_t event_val = 1;
    write(_event_fd, &event_val, sizeof(event_val));

    _thread.join();

    if(_fd >= 0){ close(_fd); }
}

void SerialPort::run()
{
    open();

    while(!_deleted)
    {
        if(_fd < 0) 
        { 
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            open(); 
        }
        else
        {
            read();
        }
    }
}

void SerialPort::open()
{
    RCLCPP_INFO_STREAM(_node->get_logger(), "Opening serial port " << _filepath.string() << " (baudrate = " << _baudrate << ")");

    // Open file descriptor
    _fd = ::open(_filepath.c_str(), O_RDONLY | O_NDELAY);
    if(_fd < 0) 
    { 
        
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Failed to open serial port " << _filepath.string() << " : " << strerror(errno));
        return;
    }

    // Configure serial port
    struct termios config;
    if(tcgetattr(_fd, &config) != 0) 
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Failed to read serial port config " << _filepath.string() << " : " << strerror(errno));
        return;
    }

    cfsetspeed(&config, _baudrate);

    config.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    config.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    config.c_cflag &= ~CSIZE; // Clear all the size bits
    config.c_cflag |= CS8; // 8 bits per byte (most common)
    config.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    config.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    config.c_lflag &= ~ICANON;
    config.c_lflag &= ~ECHO; // Disable echo
    config.c_lflag &= ~ECHOE; // Disable erasure
    config.c_lflag &= ~ECHONL; // Disable new-line echo
    config.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    config.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    config.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    config.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    config.c_cc[VMIN] = 0;

    // Flush and apply configuration
    if(tcflush( _fd, TCIFLUSH ) != 0)
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Failed to flush serial port " << _filepath.string() << " : " << strerror(errno));
        _fd = -1;
        return;
    }
    if (tcsetattr ( _fd, TCSANOW, &config ) != 0) 
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Failed to configure serial port " << _filepath.string() << " : " << strerror(errno));
        _fd = -1;
        return;
    }
}

void SerialPort::read()
{
    uint8_t buffer[BUFFER_SIZE];
    struct pollfd fds[2];

    fds[0].fd = _fd;
	fds[0].events = POLLIN;

    fds[1].fd = _event_fd;
	fds[1].events = POLLIN;

    int ret = poll(fds, 2, 1000);

    if(fds[1].revents) { return; } // On reçoit un eventfd, on est dans le destructeur de la classe, on ne fait rien

    if(fds[0].revents != 0 && fds[0].revents != POLLIN)
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Error received on file descriptor for serial port " << _filepath.string());
        _fd = -1;
        return;
    }



    // Dans tous les autres cas, on essaie de lire
    int n = ::read(_fd, &buffer, sizeof(buffer));
    if(n < 0) 
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), "Error reading from serial port " << _filepath.string() << " : " << strerror(errno));
        _fd = -1;
        return;
    }

    if(n > 0)
    {
        _data_callback(std::vector<uint8_t>(buffer, buffer + n));
    }
}