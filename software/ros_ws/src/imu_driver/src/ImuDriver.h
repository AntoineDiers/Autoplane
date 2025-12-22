#include "SerialPort.h"

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#define FRAME_START_BYTE 0x55

#define ATTITUDE_START_BYTE 0x53
#define ATTITUDE_FRAME_SIZE 11

#define PI_DEG 180.0

class ImuDriver : public rclcpp::Node
{
public:

    ImuDriver();

private:

    struct Attitude
    {
        double roll_deg;
        double pitch_deg;
        double yaw_deg;
    };

    void onSerialData(const std::vector<uint8_t>& data);
    void goToNextFrame(std::vector<uint8_t>& buffer);
    void parseAttitude(std::vector<uint8_t>& buffer);
    bool verifyChecksum(std::vector<uint8_t>& buffer, uint32_t frame_size);

    std::shared_ptr<SerialPort> _serial_port;
    std::vector<uint8_t> _buffer;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _roll_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pitch_publisher;
};