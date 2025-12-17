#pragma once

#include <nlohmann/json.hpp>
#include <iostream>

#include "conversions/State.h"
#include "conversions/RcInputs.h"

class Bridge 
{
public:
    
    Bridge();

    // Get requests
    nlohmann::json getState() { return _state.toJson(); }
    
    // Post requests
    void postMode(const nlohmann::json &data);
    void postRcInputs(const nlohmann::json &data);


    void update(); // MOCK

private:

    State _state;

    RcInputs _rc_inputs;

};