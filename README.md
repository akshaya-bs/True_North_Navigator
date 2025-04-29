# True_North_Navigator

Overview
True North Calculator is a ROS 2 package that provides real-time True North heading calculations for robotics applications. It is specifically designed to work for the Casia G system, enabling precise orientation and navigation capabilities.
This package calculates the True North direction by applying the magnetic declination to the magnetic north heading, providing a reliable reference point for autonomous navigation.

Features

Real-time True North calculations: Calculate the offset between magnetic and true north based on location and time
Magnetic Declination Model: Implements the World Magnetic Model (WMM) for accurate declination values
ROS 2 Service-based Architecture: Easy integration into existing robotic platforms
Auto-start Capability: Systemd integration for automatic startup on boot
Casia G Integration: Optimized for use with the Casia G robotics platform



Project Structure

nav_systems_ws/
├── build/              # Build files (generated)
├── install/            # Install files (generated)
├── log/                # Log files (generated)
└── src/
    ├── true_north_calculator/
    │   ├── launch/
    │   │   └── true_north_system.launch.py
    │   ├── resource/
    │   ├── true_north_calculator/
    │   │   ├── **init**.py
    │   │   ├── core.py
    │   │   ├── magdec.py
    │   │   ├── time_utils.py
    │   │   ├── true_north_client.py
    │   │   ├── true_north_controller.py
    │   │   ├── true_north_manager.py
    │   │   ├── true_north_service.py
    │   │   └── WMM.COF            # World Magnetic Model coefficient file
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
    └── true_north_interfaces/
        ├── include/
        ├── src/
        ├── srv/
        │   └── TrueNorthCalculation.srv
        ├── CMakeLists.txt
        └── package.xml
![nav_structure](https://github.com/user-attachments/assets/f3a4a012-a23d-424b-b134-7eab71732c30)
