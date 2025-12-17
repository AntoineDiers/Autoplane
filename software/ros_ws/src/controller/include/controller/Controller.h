#include "Pid.h"

#include <random>

#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/duration.hpp>

class Controller : public rclcpp::Node
{
public:

    Controller() : Node("controller")
    {
        _imu_sub = create_subscription<sensor_msgs::msg::Imu>("~/imu", 10, [this](const sensor_msgs::msg::Imu& msg){ onImu(msg); });

        _left_flap_cmd_pub = create_publisher<std_msgs::msg::Float64>("~/left_flap_cmd", 10);
        _right_flap_cmd_pub = create_publisher<std_msgs::msg::Float64>("~/right_flap_cmd", 10);
        _elevators_cmd_pub = create_publisher<std_msgs::msg::Float64>("~/elevators_cmd", 10);
        _thruster_cmd_pub = create_publisher<std_msgs::msg::Float64>("~/thruster_cmd", 10);
        
        _roll_pid = std::make_shared<Pid>(this, "roll_pid");
        _pitch_pid = std::make_shared<Pid>(this, "pitch_pid");

        _rng = std::mt19937(_rng_device());
    }

private:

    void onImu(const sensor_msgs::msg::Imu& msg)
    {
        // Get roll pitch yaw from imy message
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg.orientation, quaternion);
        tf2::Matrix3x3 rotation_matrix(quaternion);
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        roll += _noise(_rng);
        pitch += _noise(_rng);

        double time = get_clock()->now().seconds(); // msg.header.stamp.sec + 1.0e-9 * msg.header.stamp.nanosec;
        double roll_cmd = _roll_pid->tick(Pid::StampedInput{time, roll, 0}).value_or(0);
        double pitch_cmd = _pitch_pid->tick(Pid::StampedInput{time, pitch, 0}).value_or(0);

        std_msgs::msg::Float64 cmd;
        cmd.data = roll_cmd;
        _left_flap_cmd_pub->publish(cmd);

        cmd.data = -roll_cmd;
        _right_flap_cmd_pub->publish(cmd);

        cmd.data = pitch_cmd;
        _elevators_cmd_pub->publish(cmd);

        cmd.data = 600;
        _thruster_cmd_pub->publish(cmd);
    }

    std::random_device _rng_device;
    std::mt19937 _rng;
    std::normal_distribution<double> _noise{0, 0.1};


    std::shared_ptr<Pid> _roll_pid;
    std::shared_ptr<Pid> _pitch_pid;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _left_flap_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _right_flap_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _elevators_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _thruster_cmd_pub;
};