#include "GpsDriver.h"

#include <autoplane_lib/Utils.h>

#include <iomanip>

#define FRAME_START_BYTE '$'
#define FRAME_END_BYTE '\n'
#define MAX_BUFFER_SIZE 1000

GpsDriver::GpsDriver() : Node("gps_driver")
{
    _latitude_publisher =   create_publisher<std_msgs::msg::Float64>("~/latitude",  10);
    _longitude_publisher =  create_publisher<std_msgs::msg::Float64>("~/longitude", 10);
    _heading_publisher =    create_publisher<std_msgs::msg::Float64>("~/heading",   10);
    _altitude_publisher =   create_publisher<std_msgs::msg::Float64>("~/altitude",  10);

    std::string serial_port = declare_parameter("serial_port", "/dev/gps");
    uint32_t    baudrate =    declare_parameter("baudrate", 230400);

    SerialPort::ControlSignals control_signals;
    control_signals.dtr = true;
    control_signals.rts = false;
    _serial_port = std::make_shared<SerialPort>(this, serial_port, baudrate, control_signals, [this](const std::vector<uint8_t>& data)
    {
        onSerialData(data);
    });
}

void GpsDriver::onSerialData(const std::vector<uint8_t> &data)
{
    _buffer.insert(_buffer.end(), data.begin(), data.end());

    while(true)
    {
        // Go to the beginning of the frame if we are not there yet
        if(!_buffer.empty() && _buffer[0] != FRAME_START_BYTE)
        {
            goToNextFrame(_buffer);
        }

        // Find the end of the frame
        auto end_of_frame = std::find(_buffer.begin(), _buffer.end(), FRAME_END_BYTE);
        if(end_of_frame == _buffer.end())
        {
            // if the buffer does not contain the end of the frame, wait for more data
            if(_buffer.size() > MAX_BUFFER_SIZE) 
            {
                // There is something wrong
                RCLCPP_WARN(get_logger(), "Invalid data in serial buffer, flushing...");
                _buffer.clear();
            }
            return;
        }

        std::string frame(_buffer.begin(), end_of_frame - 1); // Throw away \r\n
        _buffer = std::vector<uint8_t>(end_of_frame + 1, _buffer.end());

        std::vector<std::string> frame_split = split(frame, "*");
        if(frame_split.size() != 2 || frame_split[0].size() == 0 || frame_split[1].size() != 2)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Got invalid gps frame : " << frame);
            continue;
        }

        std::string frame_content = frame_split[0].substr(1); // Throw away $
        std::string calculated_checksum = calculateChecksum(frame_content); 
        if(frame_split[1] != calculated_checksum)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Got invalid checksum : " << frame << " claculated : " << calculated_checksum);
            continue;
        }
        
        std::vector<std::string> frame_fields = split(frame_content, ",");
        if(frame_fields.empty())
        {
            RCLCPP_WARN_STREAM(get_logger(), "Got invalid gps frame : " << frame);
            continue;
        }

        std::string identifier = frame_fields[0];
        frame_fields.erase(frame_fields.begin());

        RCLCPP_INFO_STREAM(get_logger(), "Got frame " << frame);
        if(identifier == "GNGGA")
        {
            parseGGA(frame_fields);
        }
        else if(identifier == "PALYSBLS")
        {
            parsePALYSBLS(frame_fields);
        }
    }
}

void GpsDriver::goToNextFrame(std::vector<uint8_t> &buffer)
{
    if(!buffer.empty()){ buffer.erase(buffer.begin()); }
    while(!buffer.empty() && buffer.front() != FRAME_START_BYTE)
    {
        buffer.erase(buffer.begin());
    }
}

std::string GpsDriver::calculateChecksum(const std::string &checksum_data)
{
    uint8_t checksum = 0;

    for(auto& c : checksum_data)
    {
        checksum = checksum ^ c;
    }

    std::stringstream checksum_str;
    checksum_str << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)checksum;

    return checksum_str.str();
}

void GpsDriver::parseGGA(const std::vector<std::string> &fields)
{
    if(fields.size() != 14) 
    { 
        RCLCPP_WARN_STREAM(get_logger(), "GGA frame does not have the correct number of fields : " << fields.size());
        return;
    }

    // Latitude
    std_msgs::msg::Float64 latitude;
    std::string lat_str = fields[1];
    try
    {
        int lat_deg = std::stoi(lat_str.substr(0,2));
        double lat_min = std::stod(lat_str.substr(2));
        latitude.data = lat_deg + lat_min / 60.0;

        if(fields[2] == "N") {}
        else if(fields[2] == "S") { latitude.data *= -1; }
        else 
        {
            RCLCPP_WARN_STREAM(get_logger(), "Invalid latitude direction in GGA frame : " << fields[2]);
            return;
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "GGA frame got invalid latitude field : " << lat_str);
        return;
    }

    // Longitude
    std_msgs::msg::Float64 longitude;
    std::string lon_str = fields[3];
    try
    {
        int lon_deg = std::stoi(lon_str.substr(0,3));
        double lon_min = std::stod(lon_str.substr(3));
        longitude.data = lon_deg + lon_min / 60.0;

        if(fields[4] == "E") {}
        else if(fields[4] == "W") { longitude.data *= -1; }
        else 
        {
            RCLCPP_WARN_STREAM(get_logger(), "Invalid longitude direction in GGA frame : " << fields[2]);
            return;
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "GGA frame got invalid longitude field : " << lat_str);
        return;
    }

    // Altitude
    std_msgs::msg::Float64 altitude;
    try
    {
        altitude.data = std::stod(fields[8]);
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "GGA frame got invalid altitude field : " << fields[8]);
        return;
    }
    if(fields[9] != "M")
    {
        RCLCPP_WARN_STREAM(get_logger(), "Invalid altitude unit in GGA frame : " << fields[9]);
        return;
    }

    _latitude_publisher->publish(latitude);
    _longitude_publisher->publish(longitude);
    _altitude_publisher->publish(altitude);
}

void GpsDriver::parsePALYSBLS(const std::vector<std::string> &fields)
{
    if(fields.size() != 8) 
    { 
        RCLCPP_WARN_STREAM(get_logger(), "PALYSBLS frame does not have the correct number of fields : " << fields.size());
        return;
    }

    std_msgs::msg::Float64 heading;

    try
    {
        heading.data = std::stod(fields[5]);
    }
    catch(const std::exception& e)
    {
        RCLCPP_WARN_STREAM(get_logger(), "Invalid field in PALYSBLS frame : " << e.what());
        return;
    }

    _heading_publisher->publish(heading);
}


