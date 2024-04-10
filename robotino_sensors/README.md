# robotino_sensor
The Git repository contains extensive code allowing for the seamless integration of various sensors mounted on Robotino robots
with the ROS 2 environment.
Robotinos are equipped with multiple sensors including infrared sensors, IMUs, depth cameras, among others.
Additionally, we added two Sick TIM5XX lidars using 3D printed mounts.

This repository utilizes the existing
list of these sensors along with their manufacturers and links to their respective drivers. This facilitates easy access and
setup of the required drivers for each sensor, ensuring smooth communication between the sensors and the ROS 2 framework. This
integration enables us to leverage the sensor data effectively for tasks such as mapping, navigation, and obstacle avoidance,
enhancing the efficiency and reliability of our robotic systems in industrial settings.

## List of sensors, product catalogue and drivers:

| Sensor_name | Product catalogue | Ros2-DRiver  |
| --------    | --------          | --------     |
| Sick Lidar - TIM5XX   | [Data sheet](https://cdn.sick.com/media/pdf/4/44/444/dataSheet_TiM571-2050101_1075091_en.pdf)      | [ROS2 DRiver](https://github.com/SICKAG/sick_scan_xd)      |
| IMU: Microinfinity | [Data sheet](https://ip.festo-didactic.com/InfoPortal/Robotino/document/gyro.pdf)           | [ROS2 Driver](https://github.com/grips-robocup/robotino)       |

## Launching Sensor Bringup:
This Assumes that the robotino driver or an appropriate simulations is running for the necessary odometry and IMU data.

To launch the ROS2 driver for Lidar sensors, static transforms for the lidars, as well as sensor fusion using Extended kalman filter run following command:

```bash
ros2 launch robotino_sensors robotino_sensorbringup.launch.py namespace:=robotinobase1
```

Parameters:
 - namespace: It's a launch configuration used to spawn the corresponding lasers for robotinobase(1/2/3)
 - launch_ekf: whether to launch ekf, defaults to true
 - launch_rviz: It's a launch configuration for starting rviz2 with the predefined config file, defaults to false
 - use_sim_time: whether to use sim time, defaults to false

More details of the launch are described in the following sections, regarding the main comonents, the lidar sensor launch and sensor fusion along with information about the Robotino-internal IMU.
### 1. Lidar Sensors: SICK-TIM5XX

#### Laser Mounts:
The [STL files for the laser mounts](https://github.com/carologistics/hardware/tree/master/cad/robotino/stl) allow to mount SICK-TIM5XX lasers to the tower of the Robotino.
he relevant files are:
 - backlasertop.stl
 - backlaserbottom.stl
 - frontlasermounting.stl

Here are pictures of [front](https://raw.githubusercontent.com/carologistics/hardware/master/pictures/frontlaser_mount.jpg) and [back mounts](https://raw.githubusercontent.com/carologistics/hardware/master/pictures/backlaser_mount.jpg) for reference.

#### Launching the sensor interfaces in ROS2 environment:
The following command launches the corresponding lidar nodes from Sick along with static transforms for them and the [laser scan integrator](https://github.com/carologistics/laser_scan_integrator) to combine the laser data.
```bash
ros2 launch robotino_sensors robotino_multilaser.launch.py namespace:=robotinobase1
```

Parameters:
 - namespace: It's a launch configuration used to spawn the corresponding lasers for robotinobase(1/2/3)
 - sensor_config: Configuration file for the lasers, defaults to config/laser_config.yaml
 - host_config: is loaded after sensor_config for host specific configurations. Defaults to config/<namespace>.yaml
 - launch_rviz: It's a launch configuration for starting rviz2 with the predefined config file, defaults to false
 - use_sim_time: whether to use sim time, defaults to false

### 2. IMU: Microinfinity R6093U (Integrated with Robotino)

A c++ api to acces the external IMU which is an integral part of the robotino can be accessed from [here](https://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/classrec_1_1robotino_1_1api2_1_1_gyroscope_ext.html)
A ROS2 driver for this IMU is an integral part of the [robotino driver](https://github.com/grips-robocup/robotino).
Launch of robotino driver creates the interface for IMU sensor and data is being published over the topic /<namespace>/imu

### 3. Sensor Fusion: Using Extended kalman filter/ Unscented Kalman filter

In the domain of robot localization, the accuracy of wheel odometry is paramount. However, relying solely on one source of odometry can be problematic. For instance, wheel encoder odometry provides continuous data but may suffer from inaccuracies during wheel slippage. On the other hand, odometry from IMU sensors may drift over time, leading to intermittent reliability.

To address these challenges, we've adopted the ROS 2 robot_localization package, leveraging its pre-built functionality. This package employs either Extended or Unscented Kalman filters to fuse data from multiple sensors, including wheel encoders, IMUs, and GPS sensors. The filtered odometry is then published on the topic /<namespace>/odom_filtered, providing a more accurate and robust estimation of the robot's pose.
More information about the robot_localization package and corresponding parameters is available [here](http://docs.ros.org/en/latest/api/robot_localization/html/index.html)

While using this repository, the EKF node can be launched with following command:

```bash
ros2 launch robotino_sensors robotino_ekffusion.launch.py namespace:=robotinobase1
```
