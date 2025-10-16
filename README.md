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

## Repository Structure

-   `/src/x100_localization/arduino/imu_publisher/`: Contains the `.ino` sketch for the Arduino Uno.
-   `/src/x100_localization/src/`: Contains the `imu_converter.cpp` C++ ROS node for processing IMU data.
-   `/src/x100_localization/launch/`: Contains example launch files to start all necessary nodes (`rosserial`, the C++ converter, `laser_scan_matcher`, and RViz).

## Hardware Requirements

-   XMachines X100 Robot
-   NVIDIA Jetson (or other main computer)
-   Arduino Uno
-   MPU-6050 IMU sensor
-   Jumper wires


## How to Use

### 1. Arduino Setup

1.  Open the sketch located at `x100_localization/arduino/imu_publisher/imu_publisher.ino` in the Arduino IDE.
2.  Install the required `TinyMPU6050` library.
3.  Upload the sketch to your Arduino Uno.

### 2. ROS Workspace Setup

1.  Clone this repository into the `src/` folder of your Catkin workspace.
    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/tvoje-ime/x100-imu-ros-integration.git
    ```
2.  Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
3.  Source the workspace:
    ```bash
    source devel/setup.bash
    ```

### 3. Running the System

1.  Connect the Arduino to the robot's computer via USB.
2.  Launch the main launch file to start all nodes required for localization:
    ```bash
    roslaunch x100_localization localization_setup.launch
    ```
3.  To replicate the trajectory experiment, you can use the provided `talker.cpp` node to command the robot to move in a square, while `rosbag` records the `/pose_with_covariance_stamped` topic for later analysis.

## Contact

Maid Nezović - mnezovic1@gmail.com
