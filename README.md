# True_North_Navigator

# Overview
True North Calculator is a ROS 2 package that provides real-time True North heading calculations for robotics applications. It is specifically designed to work for the Casia G system, enabling precise orientation and navigation capabilities.
This package calculates the True North direction by applying the magnetic declination to the magnetic north heading, providing a reliable reference point for autonomous navigation.

# Features

Real-time True North calculations: Calculate the offset between magnetic and true north based on location and time.

Magnetic Declination Model: Implements the World Magnetic Model (WMM) framework for accurate declination values.

ROS 2 Service-based Architecture: Easy integration into existing robotic platforms.

Auto-start Capability: Systemd integration for automatic startup on boot.

Casia G Integration: Optimized for use with the Casia G robotics platform.


# How It Works


Magnetic Declination Module (magdec.py): Implements the World Magnetic Model (WMM) to calculate magnetic declination based on geographic location and time.

Time Utilities (time_utils.py): Provides functions for time conversion and manipulation needed for accurate magnetic model calculations.

Core Calculation Engine (core.py): Contains the main algorithms for calculating magnetic declination to attain true north.

Service Interface (true_north_service.py): Exposes the calculation functionality as a ROS 2 service.

Client Interface (true_north_client.py): Provides a convenient API for other nodes to request true north calculations.

Controller (true_north_controller.py): Manages the internal state and coordinates the calculation process.

Manager (true_north_manager.py): Orchestrates the overall operation of the system.

Custom Service Definition (TrueNorthCalculation.srv): Defines the request/response format for the calculation service.


# Installation


Ubuntu 18.04 or newer

ROS 2 Galactic

Colcon build tools

# Building from Source

1) Create a workspace (if you don't have one already):

        mkdir -p ~/nav_systems_ws/src
  
       cd ~/nav_systems_ws/src

2) Clone this repository

3) Build the package

       cd ~/nav_systems_ws
  
       colcon build --packages-select true_north_calculator true_north_interfaces

4) Source the setup files

       source ~/nav_systems_ws/install/setup.bash

# Running the True North Calculator

To launch the True North Calculator node:

       ros2 launch true_north_calculator true_north_system.launch.py


# Auto-Starting on Boot

This repository includes scripts to automatically start the True North Calculator when your robot boots. The setup uses systemd to manage the service.

Setup Instructions

Create the startup script:

       nano /home/nav_systems_ws/start_true_north.sh 

With the following content ( Please update the following content to better align with your robot's settings and structure ) :

       #!/bin/bash

       #Source ROS 2 Galactic and your workspace

       source /opt/ros/galactic/setup.bash

       source /home/ghost/nav_systems_ws/install/setup.bash

       #Launch your system

       ros2 launch true_north_calculator true_north_system.launch.py

Make the script executable :


       chmod +x /home//nav_systems_ws/start_true_north.sh 

Switch to root :

       sudo i 

Create and enable the systemd service:

       sudo nano /etc/systemd/system/true_north.service

With the following content ( Please update the following content to better align with your robot's settings and structure ) :

       [Unit]

       Description=Start True North ROS2 System

       After=network.target

       [Service]

       Type=simple

       User=ghost

       WorkingDirectory=/home/nav_systems_ws

       ExecStart=/home/nav_systems_ws/start_true_north.sh

       Restart=on-failure

       Environment=ROS_DOMAIN_ID=123

       Environment=ROS_VERSION=2

       Environment=ROS_LOCALHOST_ONLY=0

       Environment=ROS_PYTHON_VERSION=3

       Environment=ROS_PACKAGE_PATH=/home/ghost/current_ros2/share

       Environment=ROS_DISTRO=galactic

       [Install]

       WantedBy=multi-user.target

Enable and start the service:

       sudo systemctl daemon-reload
       sudo systemctl enable true_north.service
       sudo systemctl start true_north.service

The system utilizes a GX5 IMU and GNSS (Navigation Satellite System) to obtain compass readings, latitude, longitude, and altitude data. This configuration ensures that when the robot is aligned to True North, the Casia G can appropriately detect aircraft, birds, and other aerial objects. This package was used with the GhostRobotics vision60 robot.

# Troubleshooting
Common issues and their solutions:

Service not found: Ensure that both packages (true_north_calculator and true_north_interfaces) are properly built and sourced.

WMM.COF not found: Check that the path to the magnetic model coefficient file is correct.

Auto-start service fails: Check the journal logs with journalctl -u true_north.service for detailed error messages.


# Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

# Acknowledgments

World Magnetic Model (WMM) data provided by NOAA

ROS 2 Galactic framework

