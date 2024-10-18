# Digital Twin Motor Control (DTMC)
Capstone Project for British Columbia Institute of Technology (BCIT) 2024\
By Simon Nguyen, Eddie Sherban, Martin Tran, and  William Wang

# Description
Accurately controlled DC motor system implementing IoT and digital twin technology.\
\
This system aims to replace the current ECP torsional plants used within the Feedback Control Lab at BCIT.\
\
The DTMC is currently a fully functional single system station replacement. However, many user-friendly features and improvements need to be implemented before more systems can be created and given to students.

# Objective:
* Solve current problems with the ECP torsional plant such as:
  * Reduce expensive yearly repair cost
  * Improve portability by greatly reduce size and weight
* Add additional features and improvements such as:
  * Allow remote usage by implementing wireless control
  * Introduce students to digital twin and IoT technology, by implementing it into the DTMC
  * Implement current sensing of the dc motor
  * Improve system accuracy for position and velocity

# System Design:
* Hardware:
  * ESP32-S3 Microcontroller
  * L298N Motor Driver
  * JGY-370 DC Motor
  * ACS724 Current Sensor
* Software:
  * Languages: C++, C, C#, Python
  * FreeRTOS
  * ESP-IDF
  * Azure IoT Middleware for FreeRTOS (https://github.com/Azure/azure-iot-middleware-freertos)
  * Azure (Azure IoT Hub, Function App, Data Explorer, Azure Digital Twin)

# Results:
* Significantly reduced system cost from 5 figures yearly repair to under $100 for the whole system by using cheaper yet equivalent components
* Reduced size and weight significantly while maintaining all functionality
* Implemented full-duplex wireless control implementing Azure communication using MQTT through Azure IoT Middleware for FreeRTOS
* Set up basic digital twin functionality which can be improved upon to allow system data prediction and visualization
* Added current sensing for enhanced dc motor understanding
* Used tachometer to calibrate DC motor controller which is able to accurately control velocity within +-2% of the setpoint (can be lowered however it will introduce oscillations)

# To Do:
* Reduce server cost while retaining all data, by rethinking data sending methodology
* Allow users to input their own Wi-Fi username and password into the system
* Enhance speed and reliability of sending commands to system remotely
* Create user-friendly interface for students to control and monitor the system
