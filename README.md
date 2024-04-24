# robotino-navigation
This repository provides a navigation configuration for robotino robots from Festo Didactic.
It is set up to seamlessly integrate with [our webots simulation](https://github.com/carologistics/rcll_simulation_webots/tree/main)
as well as with real robots controlled via [this ROS 2 driver](https://github.com/robocup-logistics/ros2-robotino).

It is configured for Robotinos that are extended by lidar sensors and this repository also offers instructions for an example setup using [two SICK tim571lidars](https://github.com/carologistics/hardware/tree/master/pictures).

## Installation Premise
This repository has been tested on [ROS2 Humble] It is recommended to use the same versions to avoid any issues;
These instructions assume that you have already installed ROS2 Humble on your machine. If not, please follow the recommended recommended ubuntu installation tutorial.

## Installation

### Workspace Setup

After installing ROS2, create a fresh workspace and clone this repository:
```bash
mkdir -p ~/ros2/robotino_navigation_ws/src
cd ~/robotino_navigation_ws/src
git clone https://github.com/carologistics/robotino_navigation.git
```


Then create a workspace called "robotino_navigation_deps_ws" to install the packages for lidar interfaces and the robotino driver

```bash
mkdir -p ~/ros2/robotino_navigation_deps_ws/src
cd ~/robotino-navigation_deps_ws/src
vcs import < ~/robotino-navigation_ws/src/robotino_navigation/dependencies.repos
```

### Building

After installation, build the dependency workspace first with the following sequence:

  1. (Optional) To build dependencies for SICK-Laser sensors please refer to the instruction provided [here](https://github.com/SICKAG/sick_scan_xd/blob/master/INSTALL-ROS2.md#build-on-linux-ros2)
    under the subsection: **Build sick_generic_caller**.
  2. Build the package "laser_scan_integrator":
```bash
cd ~/robotino-navigation_deps_ws
colcon build --packages-select laser_scan_integrator --symlink-install
```
  3. (Optional) Build the robotino driver:
```bash
cd ~/robotino-navigation_deps_ws
colcon build --symlink-install
```
  4. Now build the main workspace:
```bash
cd ~/robotino-navigation_ws
colcon build --symlink-install
```

Finally, make sure to source the workspaces before usage:

```bash
source /opt/ros/humble/setup.bash
source ~/robotino-navigation_deps_ws/install/setup.bash
source ~/robotino-navigation_ws/install/setup.bash
```

## Launch sensor_bringup

```bash
ros2 launch robotino_sensors robotino_sensorbringup.launch.py namespace:=robotinobase1 launch_rviz:=true
```

- namespace: It's a launch configuration used to spawn the corresponding robotinobase(1/2/3), its controllers, and node parameters
- launch_rviz: It's a launch configuration for starting the Rviz2 with the predefined config file, parse 'false' when using nav2_stack


## Launch SLAM toolbox

For mapping the environment, first launch the driver bringup, ensure the joystick device is connected with correct device ID (by default, device_id=0), then launch the SLAM toolbox by running the following command in the root of your workspace:

```bash
    ros2 launch robotino_slamtoolbox robotino_slam.launch.py namespace:=robotinobase1
```

Map the environment using the joystick, once the map is ready, save the map by running the following command in the root of your workspace:


    ros2 run nav2_map_server map_saver_cli -f ~/robotino-navigation_ws/src/robotino-navigation/robotino_navigation/map/<nampe of map file>

## Navigating on a known Map

```bash
ros2 launch robotino_navigation robotino_bringup.launch.py namespace:=robotinobase1 use_sim_time:=false launch_nav2rviz:=true map:=map.yaml
```

- namespace: It's a launch configuration used to spawn the map server, amcl, nav2_stack, collision monitor, and rviz2 with predefined configs for corresponding robotinobase(1/2/3)
- use_sim_time: to use sim time instead of system time
- launch_nav2rviz: whether to launch an rviz to visualize the navigation, including tools for localization and to send poses.
- map: yaml file for the map, if a relative path is given the map directory of the robotino_navigation package is also searched

Once the robot is localized, use the 2D Nav Goal tool in Rviz2 to send a goal to the robot.

