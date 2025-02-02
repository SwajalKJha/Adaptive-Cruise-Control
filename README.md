**Adaptive Cruise Control**

**Overview**
This project implements an adaptive cruise control system using the STM32 Discovery board. It utilizes ultrasonic sensors to monitor the distance between vehicles and adjusts the speed of the vehicle accordingly. The system also detects steering wheel movement for an overtake feature and logs real-time data to the cloud for black-box functionality using CAN and ESP32.

**Features**
-> Distance Detection: Uses Ultrasonic sensors to measure the distance between the vehicle and others.
-> Automatic Speed Adjustment: Adjusts the vehicle's speed based on the detected distance.
-> Overtake Detection: Activated by steering wheel movement, detected using the LIS3DSH accelerometer.
-> Cloud Data Logging: Sends real-time vehicle data to the cloud via CAN and ESP32 for tracking and analysis.

**Components Used**
-> STM32 Discovery Board: Main microcontroller for system control and calculations.
-> Ultrasonic Sensors: For measuring distance between vehicles.
-> LIS3DSH Accelerometer: Detects steering wheel movements for the overtake feature.
-> CAN Bus: Used for vehicle data communication.
-> ESP32: Sends data to the cloud for black-box functionality.

**Setup Instructions**

Hardware Setup:
-> Connect the STM32 Discovery board to the ultrasonic sensors, LIS3DSH accelerometer, CAN bus, and ESP32 as per the wiring diagram provided.

Software Setup:
-> Clone this repository to your local machine.
-> Open the project in your preferred IDE (e.g., STM32CubeIDE).
-> Build and upload the code to the STM32 Discovery board.

Cloud Integration:
-> Configure the ESP32 to send data to your preferred cloud platform (e.g., AWS, Google Cloud) by editing the appropriate settings in the code.

Testing:
-> Power on the system and test the adaptive cruise control system by simulating vehicle distances and steering wheel movements.

**How It Works**

Distance Measurement: The ultrasonic sensors continuously measure the distance to nearby vehicles.
Speed Adjustment: Based on the distance readings, the system adjusts the speed of the vehicle by controlling throttle/brake.
Overtake Detection: If the driver moves the steering wheel (detected by the LIS3DSH accelerometer), the system triggers an overtake mode, temporarily overriding the cruise control.
Black-Box Logging: The ESP32 communicates with the cloud, sending real-time vehicle data for analysis and tracking.

**License**
This project is created for the benefit of learning please feel free to use the codes and design ideas as you like, but please contribute to developing this project more effectively by adding more optimized and efficient code.

**Core Contributors**

**AKSHIT BANGARWA** **SHRADDHA ANKUSH MANE** **SWAJAL KUMAR JHA** **VAIBHAVI PRAMOD SHAHARE** **VELEGU VENKATA KOONAL** 
