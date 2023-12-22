# UVBOT (An Autonomous UV disinsfection Robot)

This repository contains the codebase and necessary information to build an Autonomous UV Disinfection Robot. The robot utilizes various hardware components and software libraries to achieve autonomous navigation and UV disinfection capabilities.

--Pravin--

## Hardware Components

- NVIDIA Jetson Nano
- Dual-Channel Servo Driver ZLAC8015D
- Ouster LiDAR OS LiDAR 16
- MPU6050 IMU

## Software Dependencies

- Ubuntu 20.04 LTS
- ROS 1 Neotic
- Python 3.8.10

## Installation Instructions

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

4. **Configuration Files**
    - Modify configuration files in the `config/` directory to suit your specific hardware setup.

## Usage

1. **Launching ROS Nodes**
    - Launch the necessary ROS nodes using provided launch files.
    ```
    roslaunch my_robot_navigation.launch
    ```

2. **Teleoperation (Optional)**
    - Use ROS teleoperation nodes to manually control the robot for testing purposes.

3. **Autonomous Navigation**
    - Implement your navigation algorithms using ROS and sensor data (LiDAR, IMU) for autonomous movement.

4. **UV Disinfection**
    - Integrate UV disinfection capabilities with appropriate safety measures and controls.

## Contributing

Contributions to enhance the project are welcome! Please fork the repository, make changes, and submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- Mention any contributors or libraries that you've used or that have inspired your project.