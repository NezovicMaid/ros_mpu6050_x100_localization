# ros_mpu6050_x100_localization
A ROS package to integrate an MPU-6050 IMU with a mobile robot via Arduino. Fuses IMU data with a laser scanner to achieve robust and accurate localization.

## Software Dependencies

-   Ubuntu & [ROS](https://wiki.ros.org/ROS/Installation) (tested on Noetic)
-   [rosserial](https://wiki.ros.org/rosserial): For Arduino-ROS communication.
-   [laser_scan_matcher](https://wiki.ros.org/laser_scan_matcher): For sensor fusion.
-   Arduino IDE with the [**TinyMPU6050**](https://github.com/gabriel-milan/TinyMPU6050) library.


# X100 Robot IMU Integration for ROS Localization

This repository contains a ROS package for integrating an MPU-6050 Inertial Measurement Unit (IMU) with an XMachines X100 mobile robot. The primary goal is to fuse IMU data with the existing sensors (Wheel Odometry, 3D LiDAR) to achieve more robust and accurate robot localization, particularly by correcting orientation drift.

The project demonstrates the full workflow: from hardware setup and data acquisition with an Arduino to publishing, processing, and fusing the data within the ROS ecosystem for improved trajectory tracking.

![Trajectory Comparison](https://github.com/user-attachments/assets/52354751-5ddb-4211-9851-77f5e4b41e67)

## Project Overview

The localization of the X100 robot, equipped with an NVIDIA Jetson, relies on multiple sensor inputs. While wheel odometry and LiDAR provide position estimates, they are susceptible to errors (e.g., wheel slippage, lack of environmental features), which leads to drift over time. An IMU provides crucial orientation data (specifically yaw) that can significantly improve the localization estimate by correcting rotational errors.

This project implements a complete pipeline for this integration:
1.  An **Arduino Uno** reads data from an **MPU-6050** sensor using the [**TinyMPU6050**](https://github.com/gabriel-milan/TinyMPU6050) library.
2.  The `rosserial` package establishes communication between the Arduino and the main ROS system running on the robot.
3.  A custom **C++ ROS node** converts the raw orientation data into standard `sensor_msgs/Imu` messages and broadcasts a `tf` transform.
4.  This processed IMU data is then fused with the LiDAR scans by the `laser_scan_matcher` node, resulting in a more accurate and robust pose estimate, as demonstrated by the improved trajectory in the image above.


## Hardware Requirements

-   XMachines X100 Robot
-   NVIDIA Jetson (or other main computer)
-   Arduino Uno
-   MPU-6050 IMU sensor

## Contact

Maid Nezović - mnezovic1@gmail.com
