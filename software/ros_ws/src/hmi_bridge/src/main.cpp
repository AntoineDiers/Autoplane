#include "Bridge.h"

#include <cpp-httplib/httplib.h>
#include <thread>
#include <iostream>

#define LISTEN_PORT 8080
#define LISTEN_IP "0.0.0.0"

#define GET_STATE_URL "/state"

#define POST_MODE_URL "/mode"
#define POST_RC_INPUTS_URL "/rc_inputs"

#define GET(url, function) server.Get(url, [&](const httplib::Request &req, httplib::Response &res) {   \
    res.set_header("Access-Control-Allow-Origin", "*");                                                 \
    std::lock_guard lock(bridge_mutex);                                                                 \
    nlohmann::json res_json = function();                                                               \
    res.set_content(res_json.dump(), "application/json");                                               \
    res.status = 200;                                                                                   \
});

#define POST(url, function) server.Post(url, [&](const httplib::Request &req, httplib::Response &res) { \
    res.set_header("Access-Control-Allow-Origin", "*");                                                 \
    std::lock_guard lock(bridge_mutex);                                                                 \
    try                                                                                                 \
    {                                                                                                   \
        nlohmann::json req_json = nlohmann::json::parse(req.body);                                      \
        function(req_json);                                                                             \
    }                                                                                                   \
    catch (const std::exception &e) {                                                                   \
        res.status = 400;                                                                               \
        res.set_content(e.what(), "text/plain");                                                        \
        return; }                                                                                       \
    res.status = 200;                                                                                   \
});

int main()
{
    httplib::Server server;

    std::mutex bridge_mutex;
    Bridge bridge;

    // Get requests
    GET(GET_STATE_URL, bridge.getState);

    // Post requests
    POST(POST_MODE_URL, bridge.postMode);
    POST(POST_RC_INPUTS_URL, bridge.postRcInputs);

    // Options
    server.Options(POST_RC_INPUTS_URL, [](const auto& req, auto& res) 
    {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Allow", "GET, POST, HEAD, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "X-Requested-With, Content-Type, Accept, Origin, Authorization");
        res.set_header("Access-Control-Allow-Methods", "OPTIONS, GET, POST, HEAD");
    });



    std::atomic_bool stop_requested = false;

    std::thread http_thread([&server]() 
    {
        server.listen(LISTEN_IP, LISTEN_PORT);
    });

    std::thread ros_thread([&]() 
    {
        while(!stop_requested)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::lock_guard lock(bridge_mutex);
            bridge.update();
        }
        server.stop();
    });

    ros_thread.join();
    http_thread.join();
}