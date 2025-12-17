# rider-m
Zhiyun Rider-M Control via ESP32 Bluetooth Module

This is a C++ sketch written in the Arduino IDE that allows a user to control a Zhiyun Rider-M gimbal.
The current implementation is for use on a prototype mine exploration robot being developed by the 501st robotics team in Mammoth Lakes, CA. A Garmin Lidar sensor is attached to the gimbal which is attached to the top of the robot. An ESP32 module is also attached to the robot and wired to the Lidar sensor. The ESP32 module is connected to a "drive computer" via USB. When a command is given via the serial monitor on the drive computer, the gimbal tilts and pans to get a 360 degree scan of its environment. The result of the scan is displayed in a 3d model on the drive computer.

The hard part of this project was figuring out the bluetooth protocol the the Rider-M uses. That protocol is not published by Zhiyun, and their customer service refuses to reach out to the development team to get the protocol information.
To use this code with a different gimbal, you will need to replace the MAC address that is currently hardcoded into the file with the MAC address of your device.
