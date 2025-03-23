# Automated Aquarium Management System (AAMS)

## Overview
The Automated Aquarium Management System (AAMS) is an IoT-based smart system designed to automate and monitor essential aquarium maintenance tasks, ensuring optimal conditions for aquatic life with minimal human intervention.

## Features
- **Real-time Monitoring**: Monitors water quality parameters like pH, temperature, and turbidity.
- **Automated Feeding**: Ensures consistent fish nutrition with scheduled feeding.
- **Smart Lighting Control**: Regulates the aquarium’s day-night cycle for fish well-being.
- **Cloud-Based Data Logging & Alerts**: Enables remote monitoring and real-time notifications.
- **Energy Efficiency**: Uses power-optimized microcontrollers and actuators.
- **Scalability**: Modular architecture supporting different aquarium sizes and configurations.

## System Components
### Hardware
- ESP32 Microcontroller
- pH Sensor (E201)
- Temperature Sensor (DS18B20)
- Ultrasonic Sensor (HC-SR04)
- 4-Channel Relay Module
- Servo Motor (SG90) for automated feeding
- LED Lighting System

### Software
- **Firmware**: Arduino IDE (C++/MicroPython)
- **Cloud Integration**: Firebase/MQTT for real-time data exchange
- **Web Dashboard**: Built with React.js and Node.js
- **Mobile Support**: Future scalability for mobile application integration

## System Workflow
1. **Data Collection**: Sensors continuously track environmental conditions.
2. **Processing & Decision Making**: The ESP32 processes sensor data to trigger automation rules.
3. **Automated Actions**:
   - pH regulation
   - Temperature control
   - Water level adjustment
   - Feeding schedule execution
4. **Cloud Storage & Alerts**: Data is logged, and alerts are sent to users.
5. **User Control**: Remote control via web dashboard.

## Installation & Setup
1. **Hardware Setup**:
   - Connect sensors and actuators to the ESP32.
   - Ensure power supply is stable.
2. **Software Setup**:
   - Install Arduino IDE and required libraries.
   - Flash ESP32 with firmware.
   - Configure cloud services.
3. **Deployment**:
   - Set up the web dashboard.
   - Test system responses to environmental changes.

## Project Structure
```
├── firmware/                  # Embedded software for ESP32
├── web-dashboard/             # Web-based monitoring system
├── mobile-app/ (future)       # Mobile application (planned)
├── docs/                      # Documentation
├── README.md                  # Project ReadMe file
```

## Future Enhancements
- AI-driven predictive maintenance for aquarium health.
- Mobile application for enhanced accessibility.
- Integration with voice assistants (Google Assistant, Alexa).

## Contributors
- **S.Saalujan**
- **R.Sainthavi**
- **K.Pradeeshan**
- **A.M.Nusnan**

## References
- Sharma, P., & Kumar, R. (2021). IoT-based smart aquarium monitoring.
- Haque, S., Ahmed, M., & Rahman, T. (2019). Automated fish feeding system using servo motors.
- Gupta, A., Verma, K., & Singh, R. (2018). Smart aquarium water quality management systems.

## License
This project is open-source and available under the MIT License.
