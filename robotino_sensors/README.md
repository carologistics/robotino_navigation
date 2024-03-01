# robotino_sensor
The Git repository contains extensive code allowing for the seamless integration of various sensors mounted on Robotino robots
with the ROS 2 environment. The Robotinos are equipped with multiple sensors including LiDAR sensors, IMUs, depth cameras, among others. The repository provides a detailed
list of these sensors along with their manufacturers and links to their respective drivers. This facilitates easy access and
setup of the required drivers for each sensor, ensuring smooth communication between the sensors and the ROS 2 framework. This
integration enables us to leverage the sensor data effectively for tasks such as mapping, navigation, and obstacle avoidance,
enhancing the efficiency and reliability of our robotic systems in industrial settings.

## List of sensors, product catalogue and drivers:

| Sensor_name | Product catalogue | Ros2-DRiver  |
| --------    | --------          | --------     |
| Sick Lidar - TIM5XX   | [Data sheet](https://cdn.sick.com/media/pdf/4/44/444/dataSheet_TiM571-2050101_1075091_en.pdf)      | [ROS2 DRiver](https://github.com/SICKAG/sick_scan_xd)      |
| IMU: Microinfinity(Integral part of robotino) | [Data sheet](chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://ip.festo-didactic.com/InfoPortal/Robotino/document/gyro.pdf)           | [ROS2 Driver]       |

### 1. Lidar Sensors: SICK-TIM5XX

#### Launching the sensor interface for single SICK sensors in ROS2 environment:

    ros2 launch robotino_sensors robotino_singlelaser.launch.py namespace:=robotinobase4

#### Launching the sensor interface for multiple SICK sensors in ROS2 environment:

    ros2 launch robotino_sensors robotino_multilaser.launch.py namespace:=robotinobase4

#### Launching the sensor interface for multiple SICK sensors, integrate the scans from these sensors (currentls two sensors mounted on Front and  back of the robotino), publish over the topic /<namespace/scan >- in ROS2 environment:

     ros2 launch robotino_sensors robotino_integratedlaser.launch.py namespace:=robotinobase4

 - Parameters: Configure the current instance of sick lidar launch using the [laserSens_config.yaml](https://github.com/carologistics/ros2-navigation/blob/sborse/ros2_fullstack/robotino_sensors/config/laserSens_config.yaml) file located in config directory
 - namespace: It's a launch configuration used to spawn the corresponding robotinobase(1/2/3), its controllers, and node parameters
 - launch_rviz: It's a launch configuration for starting the Rviz2 with the predefined config file, parse 'false' when using nav2_stack
 - static_transform: Static transformation between the co-ordinate frame associated with Lidar sensor and base_link can also be configured via launch file

Along with the drivers for laidar sensors, this launches the launch file to integrate the laser scan from multiple laser sensors (currently
configured for TWO lidars, could be modified for multiple Lidars, feel free to raise the PR if you do one !)
Well, it's important to integrate the lase scan, as we can use the laserscan published over /<namespace/scan topic to configure the AMCL and Nav2 stack
implementation for autonomous navigation.


### 2. IMU: Microinfinity R6093U (Integrated with Robotino)

A c++ api to acces the external IMU which is an integral part of the robotino can be accessed from [here](https://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/classrec_1_1robotino_1_1api2_1_1_gyroscope_ext.html)
A ROS2 driver for this IMU is an integral part of the robotino driver, please refer the following repository for robotino ROS2 driver.
Launch of robotino driver creates the interface for IMU sensor and data is being published over the topic /<namespace>/imu

### 3. Sensor Fusion: Using Extended kalman filter/ Unscented Kalman filter

In the domain of robot localization, the accuracy of wheel odometry is paramount. However, relying solely on one source of odometry can be problematic. For instance, wheel encoder odometry provides continuous data but may suffer from inaccuracies during wheel slippage. On the other hand, odometry from IMU sensors may drift over time, leading to intermittent reliability.

To address these challenges, we've adopted the ROS 2 robot_localization package, leveraging its pre-built functionality. This package employs either Extended or Unscented Kalman filters to fuse data from multiple sensors, including wheel encoders, IMUs, and GPS sensors. The filtered odometry is then published on the topic /<namespace>/odom_filtered, providing a more accurate and robust estimation of the robot's pose.
More information about the robot_localization package and corresponding parameters is available [here](http://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

While using this repository, the EKF node can be launched with following command:

    ros2 launch robotino_sensors robotino_ekffusion.launch.py namespace:=robotinobase4


## Launching Sensor bringup:

To launch the ROS2 driver for Lidar sensors, IMU and sensor fusion using Extended kalman filter run following command:

    ros2 launch robotino_sensors robotino_sensorbringup.launch.py namespace:=robotinobase4

## Author:

- [Saurabh Borse](https://github.com/borsesaurabh2022)
