# Digital Twin Motor Control (DTMC)
Capstone Project for British Columbia Institute of Technology (BCIT) 2024\
By Simon Nguyen, Eddie Sherban, Martin Tran, and  William Wang

# Description
Accurately controlled DC motor system implementing IoT and digital twin technology.\
\
This system aims to replace the current ECP torsional plants (https://www.ecpsystems.com/controls_torplant.htm) used within the Feedback Control Lab at BCIT.\
\
** The DTMC is currently a fully functional single-system station replacement. However, many user-friendly features and improvements need to be implemented before more systems can be created and used within an educational setting.

# Objective:
* Solve current problems with the ECP torsional plant such as:
  * Reduce expensive yearly repair cost
  * Improve portability by greatly reducing size and weight
* Add additional features and improvements such as:
  * Allow remote control and monitoring by implementing wireless data communication
  * Introduce students to digital twin and IoT technology, by implementing it into the DTMC
  * Implement current sensing of the dc motor
  * Improve system accuracy for position and velocity setpoint tracking

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
  * Matlab
  * KiCad

# Results:
* Significantly reduced system cost from a 5 figure yearly repair cost to under $100 for the whole system by engineering cheaper yet equivalent components
* Reduced size and weight significantly while maintaining all functionality by re-engineering the entire system including custom PCB
* Implemented full-duplex wireless control by using a modified Azure IoT Middleware MQTT communication
* Set up basic digital twin functionality for remote-controlled operation and data collection, which can be built upon to include remote diagnostics, predictive system maintenance and enhanced data visualization
* Enhanced educational understanding of dc motor by introducing current sensing
* Accurately calibrated DC motor controller using tachometer to control velocity within ±2% RPM of the setpoint (accuracy can be increased, but will increase oscillations and increase settling time)

# To Do:
* Reduce server cost while retaining all data, by rethinking data-sending methodology
* Improve portability by allowing users to input their own Wi-Fi username and password into the system
* Enhance speed and reliability of sending commands to the system remotely
* Create a user-friendly interface for users to control and monitor the system
