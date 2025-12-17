#pragma once

#include <Eigen/Core>
#include <Eigen/QR>

#include <deque>
#include <optional>

class Polynomial : private std::vector<double>
{
public:
    Polynomial(const std::vector<double>& coefficients, double t0) : 
        std::vector<double>(coefficients),
        _t0(t0) {}

    double getValueAtT(double t)
    {
        double res = 0;
        for(int i = 0; i < size(); i++)
        {
            res += at(i) * std::pow(t - _t0, i);
        }
        return res;
    }

    double getDerivateAtT(double t)
    {
        double res = 0;
        for(int i = 1; i < size(); i++)
        {
            res += at(i) * i * std::pow(t - _t0, i - 1);
        }
        return res;
    }

private:

    double _t0;
};

class PolyFilter
{
public:

    PolyFilter(const uint32_t& buffer_size, const uint8_t& polynomial_order) : 
        _buffer_size(buffer_size),
        _polynomial_order(polynomial_order) 
    {
        _matrix = Eigen::MatrixXd(_buffer_size, polynomial_order + 1);
        _values_vector = Eigen::VectorXd(_buffer_size);
    }

    void feed(const double& time, const double& value)
    {
        if(_time_buffer.size() == _buffer_size)
        {
            _time_buffer.pop_front();
            _value_buffer.pop_front();
        }
        _time_buffer.push_back(time);
        _value_buffer.push_back(value);
    }

    std::optional<Polynomial> process()
    {
        if(_time_buffer.size() < _buffer_size) { return std::nullopt; }

        double t0 = _time_buffer.front();

        // Populate the matrix
        for(size_t i = 0 ; i < _buffer_size; ++i)
        {
            for(size_t j = 0; j < _polynomial_order + 1; ++j)
            {
                _matrix(i, j) = pow(_time_buffer.at(i) - t0, j);
            }
        }

        // Populate the values vector
        for(size_t i = 0 ; i < _buffer_size; ++i)
        {
            _values_vector[i] = _value_buffer[i];
        }

        // Solve for linear least square fit
        Eigen::VectorXd coefficients  = _matrix.householderQr().solve(_values_vector);

        return Polynomial(std::vector<double>(coefficients.data(), coefficients.data() + _polynomial_order + 1), t0);
    }

private:

    Eigen::MatrixXd _matrix;
    Eigen::VectorXd _values_vector;

    std::deque<double> _time_buffer;
    std::deque<double> _value_buffer;
    uint32_t _buffer_size;
    uint8_t _polynomial_order;
};