#include "Bridge.h"

#define PI 3.14159265358979323846

Bridge::Bridge()
{
    _state.navigation_data.position.latitude_deg = 49.44;
    _state.navigation_data.position.longitude_deg = 1.09;
}

void Bridge::postMode(const nlohmann::json &data)
{
    // TODO
    _state.mode = modeFromString(data.at("mode"));
}

void Bridge::postRcInputs(const nlohmann::json &data)
{
    // TODO
    _rc_inputs = RcInputs::fromJson(data);
}

void Bridge::update()
{
    // TODO
    _state.navigation_data.speed_m_s += 0.1 * (_rc_inputs.throttle - _rc_inputs.brake);

    _state.navigation_data.attitude.roll_deg += 3 * _rc_inputs.steering;
    _state.navigation_data.attitude.roll_deg -= 0.1 * _state.navigation_data.attitude.roll_deg; // damping
    _state.navigation_data.attitude.pitch_deg += - 3 * _rc_inputs.elevation;
    _state.navigation_data.attitude.pitch_deg -= 0.1 * _state.navigation_data.attitude.pitch_deg; // damping
    _state.navigation_data.attitude.yaw_deg += 0.1 * _state.navigation_data.attitude.roll_deg;

    _state.navigation_data.position.latitude_deg += 0.0001 * _state.navigation_data.speed_m_s * cos(PI * _state.navigation_data.attitude.yaw_deg / 180.0);
    _state.navigation_data.position.longitude_deg += 0.0001 * _state.navigation_data.speed_m_s * sin(PI * _state.navigation_data.attitude.yaw_deg / 180.0);
    _state.navigation_data.position.altitude_m += - _state.navigation_data.speed_m_s * sin(PI * _state.navigation_data.attitude.pitch_deg / 180.0);
}
