
# robotino-navigation
This Git repository encompasses a comprehensive collection of code files for autonomous navigation of Robotino robots from Festo GmbH. 
Additionally, the repository introduces a representative industrial production scenario. 
It provides essential packages for constructing detailed robot descriptions, implementing sensor interfaces, integrating a SLAM toolbox for mapping, and 
deploying the Navigation2 (Nav2) stack for autonomous navigation.

Encouraging collaboration and contributions, the repository aims to serve as a reliable resource for researchers, developers, and enthusiasts in the fields of robotics and simulation. 
Regular updates ensure compatibility with evolving dependencies, making it a sustainable and valuable tool for the robotics community.

## Table of content 
- Installation Premise
- Installation
    - Dependencies
    - Installation from source
- Building
- Launch sensor_bringup 
- Launch SLAM toolbox
- Launch NAV2 stack    
- Nodes and Topics to look into
- Research and References
- Bugs and Issues

## Installation Premise
This repository has been tested on [ROS2 Humble] It is recommended to use the same versions to avoid any issues;
These instructions assume that you have already installed ROS2 Humble on your machine. If not, please follow the recommended recommended ubuntu installation tutorial;
Create a ros2 workspace, Once you have created the workspace, clone this repository in the source folder of your workspace.

## Installation
    
### Installing from source:
ATTENTION: These commands assume that you have created a workspace called "robotino-navigation_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

    cd ~/robotino-navigation_ws
    git clone -b sborse/ros2_fullstack https://github.com/carologistics/ros2-navigation.git

After cloning the repository, create a workspace called "robotino-navigation_deps_ws" to install the packages/dependencies required for successfully launching the sensor interface and navigation stack.

    cd ~/robotino-navigation_deps_ws

Install the dependencies by running the following command:

    vcs import < dependencies.repos
     
## Building:

After installation, build the dependency workspace first with the following sequence: 

    cd ~/robotino-navigation_deps_ws

  1. To build dependencies for SICK-Laser sensors please refer to the instruction provided [here](https://github.com/SICKAG/sick_scan_xd/blob/master/INSTALL-ROS2.md#build-on-linux-ros2) 
    under the subsection: **Build sick_generic_caller**

  2. THen build the package "Laser_scan_integrator" using colon build:

    colcon build --packages-select laser_scan_integrator --symlink-install
     
Now build the main workspace, and run the following command in the root of your workspace:

    cd ~/robotino-navigation_ws
    colcon build --symlink-install

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

    source /opt/ros/foxy/setup.bash

Then, source the workspace by running the following command:

    cd ~/simulation_ws
    source ~/robotino-navigation_deps_ws/install/setup.bash
    source install/setup.bash
    

## Launch sensor_bringup 

### Launching the sensor interface required for autonomous navigation 

     
    ros2 launch robotino_sensors robotino_sensorbringup.launch.py namespace:=robotinobase4 launch_rviz:=true
 

- namespace: It's a launch configuration used to spawn the corresponding robotinobase(1/2/3), its controllers, and node parameters 
- launch_rviz: It's a launch configuration for starting the Rviz2 with the predefined config file, parse 'false' when using nav2_stack 


## Launch SLAM toolbox

For mapping the environment, first launch the driver bringup, ensure the joystick device is connected with correct device ID (by default, device_id=0), then launch the SLAM toolbox by running the following command in the root of your workspace:

    ros2 launch robotino_slamtoolbox robotino_slamasync.launch.py namespace:=robotinobase4

Map the environment using the joystick, once the map is ready, save the map by running the following command in the root of your workspace:


    ros2 run nav2_map_server map_saver_cli -f ~/robotino-navigation_ws/src/robotino-navigation/robotino_navigation/map/<nampe of map file>

## Launch NAV2 stack

For autonomous navigation, first launch the single instance of robotinobase in simulation as described above, then launch the NAV2 stack by running the following command in the root of your workspace:

    ros2 launch robotino_navigation robotino_navigationbringup.launch.py namespace:=robotinobase4
        
- namespace: It's a launch configuration used to spawn the map server, amcl, nav2_stack, collision monitor, and rviz2 with predefined configs for corresponding robotinobase(1/2/3)
- Initial pose is being set from 'robotinobase(1/2/3)_nav2_params.yaml'param file

Once the robot is localized, use the 2D Nav Goal tool in Rviz2 to send a goal to the robot.

## Nodes and Topics to look into

Important notes and topics to look into for understanding the simulation environment and its working:

## Research and References
- Omnidirectional robot kinematics and dynamics:
  Moreno, J.; Clotet, E.; Lupiañez, R.; Tresanchez, M.; Martínez, D.; Pallejà, T.; Casanovas, J.; Palacín, J. Design, Implementation and Validation of the Three-Wheel Holonomic Motion System of the Assistant Personal Robot (APR). Sensors 2016, 16, 1658. [Google Scholar]

  Jordi Palacín; Elena Rubies; Eduard Clotet; and David Martínez; Evaluation of the Path-Tracking Accuracy of a Three-Wheeled Omnidirectional Mobile Robot Designed as a personal Assistant https://doi.org/10.3390/s21217216

  https://github.com/mateusmenezes95/omnidirectional_controllers?tab=readme-ov-file

- [Webots](https://cyberbotics.com/)

- [ROS2](https://docs.ros.org/en/foxy/index.html)
    
- [Navigation2](https://navigation.ros.org/)

## Bugs and Issues

Please report bugs and request features using the Issue Tracker
