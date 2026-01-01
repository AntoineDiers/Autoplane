#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "DataBuffer.h"

class Autopilot : public rclcpp::Node
{

public:

    Autopilot() : 
        rclcpp::Node("autopilot")
    {
        _roll_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/roll",
            .buffer_duration_s = 1.0,
            .derivate_delta_samples = 1,
            .expected_rate_hz = 200,
            .min_rate_warn_hz = 150,
            .min_rate_err_hz = 50,
            .max_rate_warn_hz = 250,
            .max_rate_err_hz = 500
        });

        _pitch_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/pitch",
            .buffer_duration_s = 1.0,
            .derivate_delta_samples = 1,
            .expected_rate_hz = 200,
            .min_rate_warn_hz = 150,
            .min_rate_err_hz = 50,
            .max_rate_warn_hz = 250,
            .max_rate_err_hz = 500
        });

        _heading_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/heading",
            .buffer_duration_s = 2.0,
            .derivate_delta_samples = 3,
            .expected_rate_hz = 5,
            .min_rate_warn_hz = 3,
            .min_rate_err_hz = 1,
            .max_rate_warn_hz = 7,
            .max_rate_err_hz = 10
        });

        _latitude_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/latitude",
            .buffer_duration_s = 2.0,
            .derivate_delta_samples = 3,
            .expected_rate_hz = 5,
            .min_rate_warn_hz = 3,
            .min_rate_err_hz = 1,
            .max_rate_warn_hz = 7,
            .max_rate_err_hz = 10
        });

        _longitude_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/longitude",
            .buffer_duration_s = 2.0,
            .derivate_delta_samples = 3,
            .expected_rate_hz = 5,
            .min_rate_warn_hz = 3,
            .min_rate_err_hz = 1,
            .max_rate_warn_hz = 7,
            .max_rate_err_hz = 10
        });

        _altitude_buffer = std::make_shared<DataBuffer>(this, DataBuffer::Settings
        {
            .topic = "~/altitude",
            .buffer_duration_s = 2.0,
            .derivate_delta_samples = 3,
            .expected_rate_hz = 5,
            .min_rate_warn_hz = 3,
            .min_rate_err_hz = 1,
            .max_rate_warn_hz = 7,
            .max_rate_err_hz = 10
        });
    }

private:

    std::shared_ptr<DataBuffer> _roll_buffer;
    std::shared_ptr<DataBuffer> _pitch_buffer;
    std::shared_ptr<DataBuffer> _heading_buffer;
    std::shared_ptr<DataBuffer> _latitude_buffer;
    std::shared_ptr<DataBuffer> _longitude_buffer;
    std::shared_ptr<DataBuffer> _altitude_buffer;

};