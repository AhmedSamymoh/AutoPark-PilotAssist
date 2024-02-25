# AutoPark-PilotAssist

An innovative and open-source self-parking system that incorporates Advanced Driver Assistance System (ADAS) features, including Autonomous Parking, Pilot Assistant, and Collision Avoidance. It combines hardware components such as Raspberry Pi 3B, ESP32, cameras, ultrasonic sensors, and a compass to create a comprehensive autonomous parking solution.

## Project Final Version:
   The project's final version consists of two main components:

   1. **Auto Park:**
      - The camera detects empty parking spaces using a Computer Vision (CV) script.
      - The parking node initializes, and the robot autonomously parks in the identified space.
      - ESP32 sends real-time sensor readings to the Raspberry Pi via rosserial.

   2. **Pilot Assist:**
      - Utilizing a phone camera through TCP/IP with a laptop, the YOLOv8 Nano model is deployed.
      - A Python script initializes the node to send the vehicle's position using PID controller and ultrasonic readings.
      - The system follows the target vehicle, providing a pilot assist feature for smooth navigation.

## [Hardware System & PCB](PCB/)
![3D](PCB/visuals/3D.png)

## Past Versions
- [Version 0](Version_0/): Manual Phase Park Assist
   Initial testing phase utilizing Arduino Nano, Bluetooth module HC-05, and L298n for motor control.

- [Version 1](Version_1/): Sensors Calibration and Test
   Focuses on calibrating and testing sensors to ensure accurate data collection during parking maneuvers.

- [Version 2](Version_2/):
   - [Catkin_ws](Version_2/Raspberry/): ROS Workspace and Data Transmission
      Introduces a ROS (Robot Operating System) workspace for the Raspberry Pi, enabling communication between devices and efficient data transmission.
   - [ESP32](Version_2/Version2_Esp32/): ESP32 Receives data from sensors and sends it via USB to Raspberry Pi.
      The ESP32 module in Version 2 is responsible for receiving sensor data and transmitting it to the Raspberry Pi through USB.


## Contributing
We welcome contributions to enhance and improve the Project.
--
