# AutoPilot-ParkAssistst
---
AutoPilot ParkAssist is an advanced self-parking system designed to make parking in tight spaces effortless. This project utilizes a combination of Raspberry Pi 3B, ESP32, camera, ultrasonic sensors, and a compass to provide real-time feedback during parking maneuvers. The entire system is open-source, fostering innovation and collaboration in the development of autonomous parking solutions.

# Versions
- [Version 0](Version_0/) - [ManualPhaseParkAssistV0](Version_0/ManualPhaseParkAssistV0.ino/):
   Manual Phase Park Assist utilizing Arduino Nano for testing, Bluetooth module HC-05, and L298n for motor control.

- [Version 1](Version_1/): Sensors Calibration and Test
   Version 1 focuses on calibrating and testing sensors for accurate data collection during parking maneuvers.

- [Version 2](Version_2/):
   - [Catkin_ws](Version_2/Raspberry/): ROS Workspace and Data Transmission
      Version 2 introduces a ROS (Robot Operating System) workspace for the Raspberry Pi, facilitating communication between devices and data transmission.
   - [ESP32](Version_2/Version2_Esp32/): ESP32 Receives data from sensors and sends it via USB to Raspberry Pi.
      The ESP32 module in Version 2 is responsible for receiving sensor data and transmitting it to the Raspberry Pi via USB.



## Contributing

We welcome contributions to enhance and improve the Project. If you have ideas, bug fixes, or new features to propose, Please feel free to open issues or pull requests to help improve.

## License

AutoPilot ParkAssist is released under the [MIT License](LICENSE)

---
