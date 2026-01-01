#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class DataBuffer
{
public:

    struct Settings
    {
        std::string topic;
        double buffer_duration_s;
        uint32_t derivate_delta_samples;
        double expected_rate_hz;
        double min_rate_warn_hz;
        double min_rate_err_hz;
        double max_rate_warn_hz;
        double max_rate_err_hz;
    };

    enum State
    {
        OK,
        WARN,
        ERR
    };

    DataBuffer(rclcpp::Node* node, const Settings& settings) : 
        _settings(settings)
    {
        _subscriber = node->create_subscription<std_msgs::msg::Float64>(settings.topic, 10, [this](const std_msgs::msg::Float64& msg){onData(msg);});
    }

    State getState() 
    {
        cleanup();
        double rate = getRate();
        if(rate < _settings.min_rate_err_hz || rate > _settings.max_rate_err_hz) { return State::ERR; }
        if(rate < _settings.min_rate_warn_hz || rate > _settings.max_rate_warn_hz) { return State::WARN; }
        return State::OK;
    }

    double getRate()
    {
        return _data.size() / _settings.buffer_duration_s;
    }

    double getExpectedRate()
    {
        return _settings.expected_rate_hz;
    }

    std::optional<double> getLast()
    {
        cleanup();
        if(_data.empty() || getState() == State::ERR) { return std::nullopt; }
        return _data.back().second;
    }

    std::optional<double> getDerivate()
    {
        cleanup();
        if(_data.size() <= _settings.derivate_delta_samples || getState() == State::ERR) { return std::nullopt; }
        
        uint32_t previous_sample_index = _data.size() - 1 - _settings.derivate_delta_samples;
        return (_data.back().second - _data[previous_sample_index].second) / (_data.back().first - _data[previous_sample_index].first).seconds();
    }

private:

    void onData(const std_msgs::msg::Float64& msg)
    {
        _data.push_back({_clock.now(), msg.data});
    }

    void cleanup()
    {
        rclcpp::Time threshold = _clock.now() - rclcpp::Duration::from_seconds(_settings.buffer_duration_s);

        while(!_data.empty() && _data.back().first < threshold)
        {
            _data.erase(_data.begin());
        }
    }

    Settings _settings;
    rclcpp::Clock _clock = rclcpp::Clock(RCL_STEADY_TIME);
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _subscriber;

    std::vector<std::pair<rclcpp::Time, double>> _data;
};