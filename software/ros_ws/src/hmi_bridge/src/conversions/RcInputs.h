#pragma once

#include <nlohmann/json.hpp>

struct RcInputs
{
    double throttle = 0;    // 0.0 to 1.0
    double brake = 0;       // 0.0 to 1.0
    double elevation = 0;   // -1.0 to 1.0
    double steering = 0;    // -1.0 to 1.0

    static RcInputs fromJson(const nlohmann::json &json)
    {
        RcInputs inputs;
        inputs.throttle =   json["throttle"];
        inputs.brake =      json["brake"];
        inputs.elevation =  json["elevation"];
        inputs.steering =   json["steering"];
        return inputs;
    }
};