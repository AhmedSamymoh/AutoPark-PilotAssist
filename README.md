# AutoPark-PilotAssist

An innovative and open-source self-parking system that incorporates Advanced Driver Assistance System (ADAS) features, including Autonomous Parking, Pilot Assistant, and Collision Avoidance. It combines hardware components such as Raspberry Pi 3B, compass , cameras, ultrasonic sensors, and ESP32 with FreeRTOS for efficient multitasking to create a comprehensive autonomous parking solution.

![p](PerviousVersions/Version_2/P.jpg)

## Project Final Version:
   The project's final version consists of two main components:

   1. **Auto Park:**
      - The camera detects empty parking spaces using a Computer Vision (CV) script.
      - The parking node initializes, and the robot autonomously parks in the identified space.
      - FreeRTOS tasks manage ultrasonic sensor readings and IMU updates for precise navigation and
        sends real-time sensor readings to the Raspberry Pi.

   2. **Pilot Assist:**
      - Utilizing a phone camera through TCP/IP with a laptop, the YOLOv8 Nano model is deployed.
      - A Python script initializes the node to send the vehicle's position using PID controller and ultrasonic readings.
      - The system follows the target vehicle, providing a pilot assist feature for smooth navigation.

## [Hardware System & PCB](PCB/)
![3D](PCB/visuals/3D.png)


## Previous Versions
I incorporated unit testing and adopted Agile software development practices throughout the project.

- Version 0: Manual Phase Park Assist
Initial testing utilizing Arduino Nano, Bluetooth module HC-05, and L298n.
- Version 1: Sensor Calibration
Focuses on calibrating and testing sensors for accurate data collection.
- Version 2: ROS Workspace
Catkin_ws: ROS workspace for communication.
ESP32: ESP32 receives and transmits sensor data.



## Contributing
We welcome contributions to enhance and improve the Project.

