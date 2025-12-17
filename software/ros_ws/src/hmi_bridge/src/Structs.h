#pragma once

#include <string>
#include <stdexcept>

struct Attitude
{
    double roll_deg;
    double pitch_deg;
    double yaw_deg;
};

struct Position
{
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
};

struct Navigation
{
    Attitude attitude;
    Position position;
    double speed_m_s;
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
    Mode mode;
};