#pragma once

#include <string>
#include <stdexcept>
#include <nlohmann/json.hpp>

struct Attitude
{
    double roll_deg = 0;
    double pitch_deg = 0;
    double yaw_deg = 0;
};

struct Position
{
    double latitude_deg = 0;
    double longitude_deg = 0;
    double altitude_m = 0;
};

struct Navigation
{
    Attitude attitude;
    Position position;
    double speed_m_s = 0;
};

enum class Mode
{
    IDLE,
    TAKEOFF,
    MANUAL,
    AUTO,
    LANDING,
    EMERGENCY
};

inline std::string modeToString(Mode mode)
{
    switch (mode)
    {
        case Mode::IDLE: return "IDLE";
        case Mode::TAKEOFF: return "TAKEOFF";
        case Mode::MANUAL: return "MANUAL";
        case Mode::AUTO: return "AUTO";
        case Mode::LANDING: return "LANDING";
        case Mode::EMERGENCY: return "EMERGENCY";
        default: throw std::invalid_argument("Unknown mode");
    }
}

inline Mode modeFromString(const std::string &mode_str)
{
    if (mode_str == "IDLE") return Mode::IDLE;
    if (mode_str == "TAKEOFF") return Mode::TAKEOFF;
    if (mode_str == "MANUAL") return Mode::MANUAL;
    if (mode_str == "AUTO") return Mode::AUTO;
    if (mode_str == "LANDING") return Mode::LANDING;
    if (mode_str == "EMERGENCY") return Mode::EMERGENCY;
    throw std::invalid_argument("Unknown mode : " + mode_str);
}

struct State
{
    Navigation navigation_data;
    Mode mode = Mode::IDLE;

    nlohmann::json toJson() const
    {
        nlohmann::json res;
        res["navigation_data"]["attitude"]["roll_deg"] = navigation_data.attitude.roll_deg;
        res["navigation_data"]["attitude"]["pitch_deg"] = navigation_data.attitude.pitch_deg;
        res["navigation_data"]["attitude"]["yaw_deg"] = navigation_data.attitude.yaw_deg;

        res["navigation_data"]["position"]["latitude_deg"] = navigation_data.position.latitude_deg;
        res["navigation_data"]["position"]["longitude_deg"] = navigation_data.position.longitude_deg;
        res["navigation_data"]["position"]["altitude_m"] = navigation_data.position.altitude_m;

        res["navigation_data"]["speed_m_s"] = navigation_data.speed_m_s;

        res["mode"] = modeToString(mode);

        return res;
    }
};
