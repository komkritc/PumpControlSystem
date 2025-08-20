# 💧 Water Pump & Tank Monitoring System

**An IoT solution for intelligent water pump control and tank level monitoring with energy consumption tracking — designed for residential and agricultural applications.**

![Project Screenshot](https://github.com/komkritc/PumpControlSystem/blob/master/images/screenshot1.jpg)

- This project presents a robust, IoT-enabled liquid level monitoring system leveraging ultrasonic sensors and ESP8266 microcontrollers for real-time measurement of water, oil, or fuel levels in tanks. The system employs ESP-NOW for low-latency peer-to-peer communication between sensor nodes and a central controller, supplemented by WiFi for web-based dashboards and remote configuration via a captive portal. Key technical innovations include optimized ultrasonic sensor readings with burst-mode averaging (60–120ms response time), outlier detection, and dynamic calibration, achieving ±1cm accuracy. The controller integrates a CSE7766 power monitor for energy consumption tracking and safety cutoffs, while EEPROM storage ensures persistent configuration. Modular firmware design allows scalability across tank sizes, with preset and custom volume calculations (e.g., 500L–2000L tanks) and OTA updates for maintenance. The system’s hybrid wireless architecture (ESP-NOW + WiFi) addresses reliability challenges in industrial environments, offering sub-500ms latency for critical control signals while maintaining web accessibility. Performance metrics, including 95%+ sensor read success rates and adaptive filtering, are validated through benchmark tests. The integration of power monitoring with level control (e.g., pump cutoffs at 1000W thresholds) enhances safety, distinguishing it from conventional single-function systems. Open-source firmware and detailed sensor health diagnostics further support reproducibility and deployment in diverse applications, from agriculture to fuel storage. This work advances IoT-based liquid monitoring by combining low-cost hardware with optimized algorithms for accuracy, scalability, and energy-aware operation.
---

## 🚀 Key Features

- **Automated Pump Control**  
  Manages the water pump based on configurable tank level thresholds.

- **Real-time Monitoring**  
  Tracks:
  - Water levels  
  - Voltage  
  - Current  
  - Power  
  - Energy consumption  

- **Safety Features**  
  Automatic pump cutoff on over-power conditions.

- **Wireless Communication**  
  ESP-NOW protocol between pump controller and tank monitor.

- **Mobile-Friendly Web Interface**  
  - Visual tank representation  
  - Energy usage statistics  
  - Manual pump override  

- **Captive Portal**  
  Automatic redirection to control page (`192.168.4.1`).

- **OTA Updates**  
  Wireless firmware updates without disassembly.

---

## 🛠 System Components

### 1. Pump Controller (`MainControl_v6.ino`)
- **Platform:** ESP8266 (SonOff S31)  
- Relay control for water pump  
- CSE7766 power monitoring IC  
- ESP-NOW communication  
- Captive portal & web server

### 2. Tank Monitor (`TankMonitor_SR04Captive.ino`)
- Ultrasonic distance sensor (JSN-SR04T) for level measurement  
- Calculates volume for different tank shapes  
- ESP-NOW communication  
- Web dashboard with live tank visualization
![Project Screenshot](https://github.com/komkritc/PumpControlSystem/blob/master/images/screenshot_tank1.jpg)
---

## 📦 Hardware Requirements
- 2× ESP8266 modules (NodeMCU, Wemos D1 mini, etc.)  
- 1× Ultrasonic distance sensor (JSN-SR04T or equivalent)  
- 1× Relay module  
- 1× CSE7766 energy meter IC  
- Power supply & wiring components

---

## ⚙️ Installation

1. **Flash the Firmware**  
   - Upload `MainControl_v6.ino` to the pump controller ESP8266.  
   - Upload `TankMonitor_SR04Captive.ino` to the tank monitor ESP8266.

2. **Configure ESP-NOW**  
   - Set the peer MAC addresses in both sketches for two-way communication.

3. **Upload Web Interface**  
   - Use Arduino LittleFS uploader to store HTML/CSS/JS assets on each device.

4. **Connect Hardware**  
   - Follow pin definitions in the sketches for wiring sensors, relay, and power monitor.

---

## 📖 Usage

1. Power on both ESP8266 devices.  
2. Connect your phone/PC to the **`WaterPumpAP`** WiFi network.  
