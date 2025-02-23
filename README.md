**# Adaptive Cruise Control**

## **Overview**
This project implements an **Adaptive Cruise Control (ACC) System** using the **STM32 Discovery Board**. The system utilizes ultrasonic sensors to monitor the distance between vehicles and dynamically adjusts speed accordingly. Additionally, it detects **steering wheel movement** to facilitate overtaking and logs real-time vehicle data to the cloud for **black-box functionality** using **CAN bus and ESP32**.

## **Key Features**
- **Distance Detection**: Utilizes ultrasonic sensors to measure vehicle distances in real time.
- **Automatic Speed Adjustment**: Modulates vehicle speed based on distance readings.
- **Overtake Detection**: Detects steering wheel movement via the **LIS3DSH accelerometer** to trigger overtaking mode.
- **Cloud Data Logging**: Sends real-time vehicle data to the cloud via **CAN Bus and ESP32** for tracking and analysis.

## **Components Used**
- **STM32 Discovery Board**: Main microcontroller for system control and computation.
- **Ultrasonic Sensors**: Measure distances between vehicles.
- **LIS3DSH Accelerometer**: Detects steering movements for overtaking.
- **CAN Bus**: Facilitates vehicle data communication.
- **ESP32**: Sends real-time data to the cloud.

## **Setup Instructions**
### **Hardware Setup**
1. Connect the **STM32 Discovery Board** to:
   - Ultrasonic sensors
   - LIS3DSH accelerometer
   - CAN bus
   - ESP32
2. Follow the wiring diagram provided in the documentation for accurate connections.

### **Software Setup**
1. Clone this repository to your local machine.
2. Open the project in **STM32CubeIDE** or your preferred development environment.
3. Build and upload the firmware to the **STM32 Discovery Board**.

### **Cloud Integration**
1. Configure the **ESP32** to send data to a cloud platform such as **AWS** or **Google Cloud**.
2. Modify the necessary cloud credentials in the provided code.

### **Testing the System**
1. Power on the system.
2. Simulate different vehicle distances and monitor **automatic speed adjustments**.
3. Move the steering wheel to test **overtake detection**.
4. Verify real-time data logging to the cloud.

## **System Workflow**
1. **Distance Measurement**: Ultrasonic sensors continuously measure vehicle distance.
2. **Speed Regulation**: The system adjusts the vehicle speed based on proximity readings.
3. **Overtaking Mode**: Detected steering wheel movement overrides cruise control for safe overtaking.
4. **Data Logging**: The **ESP32** transmits real-time vehicle telemetry to the cloud for future analysis.

## **License**
 **DO NOT USE THE PROJECT_REPORT.pdf for other uses, it is only meant as a read-only file**.
This project is open-source and intended for educational purposes. Feel free to utilize and modify the code.
Contributions for **optimization and efficiency improvements** are highly encouraged.

## **Core Contributors**
- **Akshit Bangarwa**
- **Shraddha Ankush Mane**
- **Swajal Kumar Jha**
- **Vaibhavi Pramod Shahare**
- **Velegu Venkata Koonal**

