# AutoPilot-ParkAssistst
---
AutoPilot ParkAssist is an advanced self-parking system designed to make parking in tight spaces effortless. This project utilizes a combination of Raspberry Pi, STM32, camera, ultrasonic sensors, and a compass to provide real-time feedback during parking maneuvers. The entire system is open-source, fostering innovation and collaboration in the development of autonomous parking solutions.

# Versions
- [Version 0](Version_0/) [ManualPhaseParkAssistV0](Version_0/ManualPhaseParkAssistV0.ino/):
   Manual Phase Park Assist using Arduino nano test and Bluetooth module HC-05 and L298n
- [Version 1](Version_1/): Sensors Calibration and Test
- [Version 2](Version_2/): - [Catkin_ws](Version_2/Raspberry/) ROS Workspace and Data Transmission
                           - [ESP32](Version_2/Version2_Esp32/): ESP32 Recieve data from sensors and send it via usb to raspberry.



---
## Hardware Setup

To get started with AutoPilot ParkAssist, follow these steps for setting up the required hardware components:

1. **Raspberry Pi:** Ensure you have a Raspberry Pi board.

2. **STM32F103:** Obtain an STM32 microcontroller for interfacing with sensors and controlling the parking system.

3. **Camera:** Connect a compatible camera for visual input.

4. **Ultrasonic Sensors:** Include ultrasonic sensors for proximity detection during parking maneuvers.

5. **Compass:** Integrate a compass to provide directional information.

## Contributing

We welcome contributions to enhance and improve the Project. If you have ideas, bug fixes, or new features to propose, Please feel free to open issues or pull requests to help improve.

## License

AutoPilot ParkAssist is released under the [MIT License](LICENSE)

---
