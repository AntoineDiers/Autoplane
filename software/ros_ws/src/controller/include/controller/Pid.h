#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "PolyFilter.h"

class Pid
{
public:

    struct StampedInput
    {
        double stamp;
        double data;
        double setpoint;
    };

    Pid(rclcpp::Node* node, const std::string& name)
    {
        _clock =         node->get_clock();

        node->declare_parameter(name + "/kp",               rclcpp::PARAMETER_DOUBLE); 
        node->declare_parameter(name + "/ki",               rclcpp::PARAMETER_DOUBLE); 
        node->declare_parameter(name + "/kd",               rclcpp::PARAMETER_DOUBLE);
        node->declare_parameter(name + "/imax",             rclcpp::PARAMETER_DOUBLE);
        node->declare_parameter(name + "/buffer_size",      rclcpp::PARAMETER_INTEGER);
        node->declare_parameter(name + "/polyfit_order",    rclcpp::PARAMETER_INTEGER);

        _kp =               node->get_parameter(name + "/kp").as_double();
        _ki =               node->get_parameter(name + "/ki").as_double();
        _kd =               node->get_parameter(name + "/kd").as_double();
        _imax =             node->get_parameter(name + "/imax").as_double();
        int buffer_size =   node->get_parameter(name + "/buffer_size").as_int();
        int polyfit_order = node->get_parameter(name + "/polyfit_order").as_int();

        std::cout << _kp << " " << _ki << " " << _kd << " " << buffer_size << " " << polyfit_order << "\n";

        _filter = std::make_shared<PolyFilter>(buffer_size, polyfit_order);
    }

    struct RegressionResult
    {
        double error;
        double derivate;
        double integral;
    };

    std::optional<double> tick(const StampedInput& input)
    {
        _tick_counter++;

        // Update integral term
        if(_previous_stamp.has_value())
        {
            _integral += (input.data - input.setpoint) * (input.stamp - _previous_stamp.value());
            _integral = std::clamp(_integral, -_imax, _imax);
        }
        _previous_stamp = input.stamp;

        _filter->feed(input.stamp, input.data);
        std::optional<Polynomial> polynomial = _filter->process();

        if(!polynomial.has_value()) { return std::nullopt; }

        double err = polynomial->getValueAtT(_clock->now().seconds()) - input.setpoint;
        double der = polynomial->getDerivateAtT(_clock->now().seconds());

        return _kp * err + _ki * _integral + _kd * der;
    }

private:

    rclcpp::Clock::ConstSharedPtr _clock;
    double  _kp, _ki, _kd, _imax;

    uint64_t _tick_counter = 0;

    double _integral = 0;
    std::optional<double> _previous_stamp;

    std::shared_ptr<PolyFilter> _filter;
};


/*#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <Eigen/Core>
#include <Eigen/QR>

#include <queue>

class Pid
{
public:

    struct StampedInput
    {
        rclcpp::Time stamp;
        double data;
        double setpoint;
    };

    Pid(const rclcpp::Node* node, const std::string& name)
    {
        _clock =         node->get_clock();

        //_kp =            node->get_parameter(name + "/kp").as_double();
        //_ki =            node->get_parameter(name + "/ki").as_double();
        //_kd =            node->get_parameter(name + "/kd").as_double();

        //_buffer_size =   node->get_parameter(name + "/buffer_size").as_int();
        //_polyfit_order = node->get_parameter(name + "/polyfit_order").as_int();

        _buffer_size = 100;
        _polyfit_order = 3;
    }

    struct RegressionResult
    {
        double error;
        double derivate;
        double integral;
    };

    std::optional<RegressionResult> tick(const StampedInput& input)
    {
        _input_buffer.push_back(input);
        if(_input_buffer.size() > _buffer_size)
        {
            _input_buffer = std::vector<StampedInput>(_input_buffer.end() - _buffer_size, _input_buffer.end());
        }

        return doRegression();

        //std::optional<RegressionResult> regression_res = doRegression();
        //if(!regression_res.has_value()) { return std::nullopt; }

        //return _kp * regression_res->error + _ki * regression_res->integral + _kd * regression_res->derivate;
    }

private:

    

    rclcpp::Clock::ConstSharedPtr _clock;
    double  _kp = 0.2, 
            _ki = 0.2, 
            _kd = 0.2;

    int _buffer_size;
    int _polyfit_order;

    std::vector<StampedInput> _input_buffer;
    double _i;
    std::optional<StampedInput> _last_error;

    std::optional<RegressionResult> doRegression()
    {
        if(_input_buffer.size() != _buffer_size) { return std::nullopt; }

        double t0 = _input_buffer.front().stamp.seconds();
        

        std::vector<double> coefficients;

        std::vector<double> values;
        std::vector<double> times;
        for(auto& item : _input_buffer) 
        { 
            values.push_back(item.data); 
            times.push_back(item.stamp.seconds() - t0);
        }

        // Create Matrix Placeholder of size n x k, n= number of datapoints, k = order of polynomial, for exame k = 3 for cubic polynomial
        Eigen::MatrixXd T(_buffer_size, _polyfit_order + 1);
        Eigen::VectorXd V = Eigen::VectorXd::Map(&values.front(), values.size());
        Eigen::VectorXd result;

        // Populate the matrix
        for(size_t i = 0 ; i < _buffer_size; ++i)
        {
            for(size_t j = 0; j < _polyfit_order + 1; ++j)
            {
                T(i, j) = pow(times.at(i), j);
            }
        }
        
        // Solve for linear least square fit
        result  = T.householderQr().solve(V);
        coefficients.resize(_polyfit_order+1);
        for (int k = 0; k < _polyfit_order+1; k++)
        {
            coefficients[k] = result[k];
        }

        double now = _clock->now().seconds() - t0;

        RegressionResult res;
        res.error = -_input_buffer.back().setpoint;
        for(int i = 0; i < coefficients.size(); i++)
        {
            res.error += coefficients[i] * std::pow(now, i);
        }

        res.derivate = 0;
        for(int i = 1; i < coefficients.size(); i++)
        {
            res.derivate += coefficients[i] * i * std::pow(now, i - 1);
        }

        res.integral = 0;
        for(int i = 0; i < _buffer_size - 1; i++)
        {
            res.integral += (_input_buffer[i+1].stamp.seconds() - _input_buffer[i].stamp.seconds()) * ((_input_buffer[i+1].data - _input_buffer[i+1].setpoint) + (_input_buffer[i].data -  - _input_buffer[i].setpoint)) / 2.0;
        }
        res.integral += (_clock->now().seconds() - _input_buffer.back().stamp.seconds()) * ((_input_buffer.back().data - _input_buffer.back().setpoint) + res.error) / 2.0;

        return res;
    }
};*/