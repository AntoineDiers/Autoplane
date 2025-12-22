#include "ImuDriver.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ImuDriver::ImuDriver() : Node("imu_driver")
{
    _imu_publisher = create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
    _roll_publisher = create_publisher<std_msgs::msg::Float64>("~/roll", 10);
    _pitch_publisher = create_publisher<std_msgs::msg::Float64>("~/pitch", 10);

    _serial_port = std::make_shared<SerialPort>(this, "/dev/ttyUSB0", SerialPort::Baudrate::_9600, [this](const std::vector<uint8_t>& data)
    {
        onSerialData(data);
    });
}

void ImuDriver::onSerialData(const std::vector<uint8_t> &data)
{
    _buffer.insert(_buffer.end(), data.begin(), data.end());

    while(true)
    {
        // Go to next FRAME_START_BYTE if the buffer does not start with FRAME_START_BYTE
        if(!_buffer.empty() && _buffer[0] != FRAME_START_BYTE)
        {
            goToNextFrame(_buffer);
        }

        // Do nothing if the frame type has not been received yet
        if(_buffer.size() < 2)
        {
            return;
        }

        // Parse accordingly to the frame type
        uint8_t frame_type = _buffer[1];
        switch (frame_type)
        {
            case ATTITUDE_START_BYTE:
            {
                if(_buffer.size() < ATTITUDE_FRAME_SIZE) { return; } // On attend de recevoir plus de données
                parseAttitude(_buffer);
                break;
            }
            default : goToNextFrame(_buffer); break;
        }
    }
}

void ImuDriver::goToNextFrame(std::vector<uint8_t> &buffer)
{
    if(!buffer.empty()){ buffer.erase(buffer.begin()); }
    while(!buffer.empty() && buffer.front() != FRAME_START_BYTE)
    {
        buffer.erase(buffer.begin());
    }
}

void ImuDriver::parseAttitude(std::vector<uint8_t> &buffer)
{
    // Buffer state : 0x55 0x53 <8 bytes attitude_data> <checksum>

    if(verifyChecksum(buffer, ATTITUDE_FRAME_SIZE))
    {
        uint8_t rxl = buffer[2];
        uint8_t rxh = buffer[3];
        uint8_t ryl = buffer[4];
        uint8_t ryh = buffer[5];
        uint8_t rzl = buffer[6];
        uint8_t rzh = buffer[7];
        double angle_x = (rxh << 8 | rxl) / 32768.0 * PI_DEG;
        double angle_y = (ryh << 8 | ryl) / 32768.0 * PI_DEG;
        double angle_z = (rzh << 8 | rzl) / 32768.0 * PI_DEG;
        if(angle_x >= PI_DEG) { angle_x -= 2 * PI_DEG; }
        if(angle_y >= PI_DEG) { angle_y -= 2 * PI_DEG; }
        if(angle_z >= PI_DEG) { angle_z -= 2 * PI_DEG; }

        tf2::Quaternion quaternion;
        quaternion.setEuler(angle_z, angle_y, angle_x);

        sensor_msgs::msg::Imu imu;
        imu.orientation.x = quaternion.x();
        imu.orientation.y = quaternion.y();
        imu.orientation.z = quaternion.z();
        imu.orientation.w = quaternion.w();

        imu.header.stamp = get_clock()->now();

        _imu_publisher->publish(imu);

        std_msgs::msg::Float64 msg;
        msg.data = angle_x;
        _roll_publisher->publish(msg);

        msg.data = angle_y;
        _pitch_publisher->publish(msg);
    }

    goToNextFrame(buffer);
}

bool ImuDriver::verifyChecksum(std::vector<uint8_t> &buffer, uint32_t frame_size)
{
    uint32_t checksum = 0;
    for(uint32_t i = 0; i < frame_size - 1; i++)
    {
        checksum += buffer[i];
    }

    return buffer[frame_size - 1] == (checksum & 0xFF);
}
