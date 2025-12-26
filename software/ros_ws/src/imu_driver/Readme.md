# IMU Driver

This driver gets the roll and pitch of the plane from the IMU sensor.

### Parameters

| Name          | Default   | Description
|---------------|-----------|-------------------------
| serial_port   | /dev/imu  | The serial port on which this driver will listen
| baudrate      | 38400     | The baud rate of the serial port

### Topics

| Name          | Type              | Description
|---------------|-------------------|-------------------------
| ~/roll        | std_msgs/Float64  | The roll of the plane
| ~/pitch       | std_msgs/Float64  | The pitch of the plane

### IMU frames decoding 

The IMU frames received on a serial port, they are formatted as follows :

| Value             | Size (bytes)  | Description
| ----------------- | ------------- |-------------------------
| 0x55              | 1             | All frames start with this byte
| frame ID          | 1             | This byte identifies the content of the frame, **0x53** means the frame contains the euler angles
| frame content     | ?             | The contents of the frame
| frame checksum    | 1             | The checksum of the frame (the sum of all the previous bytes)

The frame we are interested in (Euler angles) has the following frame content

| Value             | Size (bytes)  | Description
| ----------------- | ------------- |-------------------------
| rxl               | 1             | least inportant byte for the X rotation
| rxh               | 1             | most inportant byte for the X rotation
| ryl               | 1             | least inportant byte for the Y rotation
| ryh               | 1             | most inportant byte for the Y rotation
| rzl               | 1             | least inportant byte for the Z rotation
| rzh               | 1             | most inportant byte for the Z rotation
| unused            | 1             | Unused byte
| unused            | 1             | Unused byte

> The euler angle for the X axis is **(rxh << 8 | rxl) / 32768.0 * 180**

