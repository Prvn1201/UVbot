# UVBOT (An Autonomous UV disinsfection Robot)

This repository contains the codebase and necessary information to build an Autonomous UV Disinfection Robot. The robot utilizes various hardware components and software libraries to achieve autonomous navigation and UV disinfection capabilities.

--Pravin--

## Hardware Components

- NVIDIA Jetson Nano
- Dual-Channel Servo Driver ZLAC8015D
- ZLLG40ASM100 V1.0 Hub Servo Motor 
- Ouster OS-1 LiDAR
- MPU 6050 IMU
- Wireless Controller 

## Software Dependencies

- Ubuntu 20.04 LTS
- ROS 1 Neotic
- Python 3.8.10

## Installation Instructions
### Dependencies Installation :-

```
sudo apt-get install ros-neotic-base-local-local-planner
```
```
sudo apt-get install ros-neotic-carrot-planner
```
```
sudo apt-get install ros-neotic-dwa-local-planner
```
```
sudo apt-get install ros-neotic-teb-local-planner
```

### Hardware Setup

1. **NVIDIA Jetson Nano Configuration**
    - Follow NVIDIA's official documentation for setting up Jetson Nano.

2. **ZLAC8015D Motor Integration**
    - Connect and configure the ZLAC8015D Motor as per the provided datasheet.

3. **Ouster LiDAR OS LiDAR 16**
    - Install necessary drivers and ROS packages for Ouster Lidar. Refer to the Ouster Lidar documentation.

4. **MPU6050 IMU**
    - Connect and calibrate the MPU6050 IMU sensor for accurate readings.

### Software Setup

1. **ROS 1 Neotic Installation**
    - Follow the instructions on ROS wiki to install ROS 1 Neotic.

2. **Repository Cloning**
    - Clone this repository to your local machine.

3. **Build and Compilation**
    - Navigate to the cloned repository and build the ROS workspace using `catkin_make`.

4. **Configuration Files (Parameters)**
- Base_local_planner_params.yaml
- Costmap_common_params.yaml
- Dwa_local_planner.yaml
- Global_costmap.yaml
- Local_costmap.yaml
- Move_base.yaml
- AMCL (Adaptive Monte Carlo Localization)
- UVbot hardware configure (twist mux node)

## Usage

1. **Launching ROS Nodes**
    - Make sure the Lidar is active for perform mapping.
    ```
    roslaunch uvbot_psas uvbot_hardware.launch
    ```
    ```
    roslaunch uvbot_psas uvbot_gmapping.launch
    ```
    - Save the map
    ```
    rosrun map_server map_saver -f map
    ```
    - Launch Autonomous Navigation 
    ```
    roslaunch uvbot_psas uvbot_navigation.launch
    ```