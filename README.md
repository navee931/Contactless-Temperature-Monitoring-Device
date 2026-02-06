# 🔥 Contactless Temperature Monitoring Device

📌 Overview
This project implements an IoT-based automated thermal monitoring system using an ESP32 microcontroller and a contactless infrared temperature sensor. A stepper motor rotates the sensor to scan multiple positions, enabling multi-point temperature monitoring using a single sensor. The system provides real-time temperature visualization and alerts through the Blynk IoT platform.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
🎯 Objectives
- To monitor temperature at multiple locations using a single infrared sensor  
- To reduce hardware cost compared to multi-sensor systems  
- To provide real-time IoT-based monitoring and alerts  
- To enable contactless and automated thermal scanning  

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
## ⚙️ System Components

- ESP32 Microcontroller (Wi-Fi enabled)  
- MLX90614 Infrared Temperature Sensor  
- 28BYJ-48 Stepper Motor with ULN2003 Driver  
- Dual Buzzers (Alert indication)  
- Blynk IoT Platform  
- 5V Power Supply  

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
🧠 Working Principle

The MLX90614 infrared sensor is mounted on a stepper motor controlled by the ESP32. The motor rotates the sensor across predefined angles. At each position, the sensor measures temperature without physical contact. The ESP32 sends the measured data to the Blynk IoT platform using Wi-Fi. Based on predefined temperature thresholds, the system triggers audible alerts and halts operation in critical conditions.

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
🔁 Operating Modes

Manual Mode
- Used for calibration and positioning
- Motor movement controlled via Serial commands or Blynk button

Automatic Mode
- Sensor scans multiple angles automatically
- Temperature is measured at each stop
- Alerts generated based on temperature level

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
🚨 Alert Levels

- **Normal:** Short buzzer indication  
- **Alert:** Continuous buzzer warning  
- **Critical:** Continuous alarm and motor halt until reset  

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
## ⌨️ Serial Commands

| Command | Description |
|-------|-------------|
| MOV `<angle>` | Rotate motor by given angle |
| SET `<angle>` | Set zero/start position |
| ZERO | Move motor to start position |
| AUTO | Start automatic scanning |
| STOP | Stop motor and auto mode |
| RESET | Clear critical halt |

---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
🌟 Conclusion

This project successfully demonstrates a low-cost and scalable IoT-based automated thermal monitoring system using a single rotating infrared temperature sensor. By integrating an ESP32 microcontroller with IoT connectivity and a stepper motor scanning mechanism, the system enables reliable multi-point temperature monitoring with real-time visualization and alerts. The solution reduces hardware complexity while maintaining accuracy, making it suitable for healthcare, industrial, and server-room monitoring applications. The project highlights the practical use of IoT for smart sensing and provides a strong foundation for future enhancements such as cloud analytics and AI-based anomaly detection.

👤 Author - Navee
