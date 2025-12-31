#include <autoplane_lib/SerialPort.h>

#include <std_msgs/msg/float64.hpp>

class GpsDriver : public rclcpp::Node
{
public:

    GpsDriver();

private:

    struct NmeaFrame
    {
        std::string id;
        std::vector<std::string> fields;
    };

    void onSerialData(const std::vector<uint8_t>& data);
    void goToNextFrame(std::vector<uint8_t>& buffer);
    std::string calculateChecksum(const std::string& checksum_data);
    void parseGGA(const std::vector<std::string>& fields);
    void parsePALYSBLS(const std::vector<std::string>& fields);


    std::shared_ptr<SerialPort> _serial_port;
    std::vector<uint8_t> _buffer;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _latitude_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _longitude_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _heading_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _altitude_publisher;
};