/*
 * Smart Water Tank Monitoring System
 *
 * Description:
 * This ESP8266-based system provides comprehensive monitoring of water tank levels with:
 * - Ultrasonic distance measurement for accurate water level detection
 * - Multiple tank size presets with automatic volume calculations
 * - ESP-NOW communication for wireless sensor networking
 * - Real-time web dashboard with visual tank representation and captive portal
 * - OTA firmware update capability
 * - Configuration interface for tank dimensions and calibration
 *
 * Features:
 * - Real-time water level percentage and volume calculations
 * - Mobile-responsive web interface with captive portal
 * - Sensor calibration capability
 * - Data persistence in EEPROM
 * - Wireless communication with other ESP devices
 *
 * Hardware Requirements:
 * - ESP8266 (NodeMCU or similar)
 * - Ultrasonic distance sensor (JSN-SR04T or similar)
 * - Optional: Secondary ESP8266 for remote monitoring
 *
 * Designed by: Komkrit Chooraung
 * Date: 10-8-2025
 * Version: 1.1 (Captive Portal Update)
 */

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <DNSServer.h> // Added for Captive Portal

// --- Constants ---
#define EEPROM_SIZE 64
#define ADDR_WIDTH 0       // float (4 bytes)
#define ADDR_HEIGHT 4      // float (4 bytes)
#define ADDR_VOL_FACTOR 8  // float (4 bytes)
#define ADDR_CALIB 12      // int (4 bytes)

#define RX_PIN D1
#define TX_PIN D2
#define SENSOR_READ_TIMEOUT 100   // ms
#define SERIAL_INPUT_TIMEOUT 5000 // ms
#define UPDATE_INTERVAL 2000      // ms for web dashboard updates

const byte DNS_PORT = 53; // DNS server port for captive portal

// Tank presets {Width, Height, VolumeFactor}
const float TANK_PRESETS[][3] = {
  { 68.0, 162.0, 3.086f },   // 500L  (68cm W × 45.4cm L × 162cm H)
  { 79.5, 170.0, 4.118f },   // 700L  (79.5cm W × 51.8cm L × 170cm H)
  { 92.5, 181.0, 5.513f },   // 1000L (92.5cm W × 59.6cm L × 181cm H)
  { 109.0, 196.0, 7.663f },  // 1500L (109cm W × 70.3cm L × 196cm H)
  { 123.0, 205.0, 9.742f }   // 2000L (123cm W × 79.2cm L × 205cm H)
};

// --- Global Variables ---
float tankWidth = TANK_PRESETS[0][0];  // Default to 500L tank
float tankHeight = TANK_PRESETS[0][1];
float volumeFactor = TANK_PRESETS[0][2];
int calibration_mm = 0;
float percent = 0.0f;
float volume = 0.0f;

// Sensor data
uint16_t mm = 0;
float cm = 0.0f;

// ESP-NOW
uint8_t senderMAC[] = { 0x5C, 0xCF, 0x7F, 0xF5, 0x3D, 0xE1 };

typedef struct {
  char cmd[32];
} struct_message;

typedef struct {
  char json[128];
} response_message;

String jsonDataOut;
volatile bool pendingDistanceRequest = false;
volatile bool pendingDistanceRequest_2 = false;
uint8_t requestingMAC[6];

// Web Server & DNS
ESP8266WebServer server(80);
DNSServer dnsServer; // Added for Captive Portal
SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

// --- EEPROM Functions ---
void saveConfig() {
  EEPROM.put(ADDR_WIDTH, tankWidth);
  EEPROM.put(ADDR_HEIGHT, tankHeight);
  EEPROM.put(ADDR_VOL_FACTOR, volumeFactor);
  EEPROM.put(ADDR_CALIB, calibration_mm);
  if (!EEPROM.commit()) {
    Serial.println("{\"error\":\"EEPROM commit failed\"}");
  }
}

void loadConfig() {
  EEPROM.get(ADDR_WIDTH, tankWidth);
  EEPROM.get(ADDR_HEIGHT, tankHeight);
  EEPROM.get(ADDR_VOL_FACTOR, volumeFactor);
  EEPROM.get(ADDR_CALIB, calibration_mm);

  // Validate loaded values
  if (isnan(tankWidth) || tankWidth <= 0) tankWidth = TANK_PRESETS[0][0];
  if (isnan(tankHeight) || tankHeight <= 0) tankHeight = TANK_PRESETS[0][1];
  if (isnan(volumeFactor) || volumeFactor <= 0) volumeFactor = TANK_PRESETS[0][2];
}

// --- Sensor Functions ---
bool readDistance(uint16_t &out_mm, float &out_cm) {
  sensorSerial.flush();  // Clear any leftover data in buffer
  sensorSerial.write(0x55);
  delay(100);  // Wait for sensor response

  if (sensorSerial.available() >= 4) {
    uint8_t buffer[4];
    sensorSerial.readBytes(buffer, 4);

    if (buffer[0] == 0xFF) {
      uint16_t raw_mm = (buffer[1] << 8) | buffer[2];
      out_mm = raw_mm + calibration_mm;
      out_cm = out_mm / 10.0;
      return true;
    }
  }
  return false;
}

// --- ESP-NOW Functions ---
void sendESPNowResponse(bool success, const char *jsonData = nullptr) {
  response_message msg;

  if (success && jsonData) {
    snprintf(msg.json, sizeof(msg.json), "%s", jsonData);
  } else {
    snprintf(msg.json, sizeof(msg.json),
             "{\"error\":\"%s\"}",
             success ? "unknown_error" : "sensor_read_failed");
  }

  int result = esp_now_send(requestingMAC, (uint8_t *)&msg, sizeof(msg));
  if (result != 0) {
    Serial.println("{\"error\":\"ESP-NOW send failed\"}");
  }
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len != sizeof(struct_message)) return;

  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  msg.cmd[sizeof(msg.cmd) - 1] = '\0';

  if (strcmp(msg.cmd, "get_distance") == 0) {
    memcpy(requestingMAC, mac, 6);
    pendingDistanceRequest = true;
  } else if (strcmp(msg.cmd, "get_measure") == 0) {
    memcpy(requestingMAC, mac, 6);
    pendingDistanceRequest_2 = true;
  }
}

// --- Volume Calculations ---
float calcWaterLevelPercent() {
  float distanceFromTop = cm;
  float waterHeight = tankHeight - distanceFromTop;
  waterHeight = constrain(waterHeight, 0.0f, tankHeight);
  return (waterHeight / tankHeight) * 100.0f;
}

float calcWaterVolumeLiters() {
  float distanceFromTop = cm;
  float waterHeight = tankHeight - distanceFromTop;
  waterHeight = constrain(waterHeight, 0.0f, tankHeight);
  return volumeFactor * waterHeight;  // Uses volumeFactor instead of W×W
}

void updateMeasurements() {
  percent = calcWaterLevelPercent();
  volume = calcWaterVolumeLiters();

  jsonDataOut = String("{\"distance\":") + String(cm, 1) + ",\"percent\":" + String(percent, 1) + ",\"volume\":" + String(volume, 1) + "}";
}

// --- Web Server Functions ---
String formatRuntime(unsigned long ms) {
  unsigned long sec = ms / 1000;
  unsigned int h = sec / 3600;
  unsigned int m = (sec % 3600) / 60;
  unsigned int s = sec % 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", h, m, s);
  return String(buf);
}

void handleRoot() {
  updateMeasurements();
  String waterColor = (percent < 20) ? "#ff4444" : "#4285f4";

  String html = FPSTR(R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Water Tank Dashboard</title>
  <link href="https://fonts.googleapis.com/css2?family=Roboto:wght@300;400;500&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Roboto', sans-serif; background-color: #f5f5f5; margin: 0; padding: 20px; color: #333; }
    .container { max-width: 600px; margin: 0 auto; background: white; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); padding: 20px; }
    h1 { color: #4285f4; margin-top: 0; text-align: center; }
    .gauge-container { display: flex; justify-content: center; margin: 20px 0; }
    .gauge { width: 150px; height: 150px; position: relative; border-radius: 50%; background: #f1f1f1; display: flex; align-items: center; justify-content: center; font-size: 24px; font-weight: 500; box-shadow: inset 0 0 10px rgba(0,0,0,0.1); }
    .gauge::before { content: ''; position: absolute; width: 100%; height: 100%; border-radius: 50%; background: conic-gradient(var(--water-color) 0% var(--water-level), transparent var(--water-level) 100%); transform: rotate(0deg); transition: all 0.5s ease; }
    .gauge-inner { width: 80%; height: 80%; background: white; border-radius: 50%; display: flex; align-items: center; justify-content: center; z-index: 1; box-shadow: 0 0 5px rgba(0,0,0,0.1); }
    .stats { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin: 20px 0; }
    .stat-card { background: #f9f9f9; padding: 15px; border-radius: 8px; text-align: center; }
    .stat-card h3 { margin: 0 0 5px 0; font-size: 14px; color: #666; font-weight: 400; }
    .stat-card p { margin: 0; font-size: 20px; font-weight: 500; }
    .tank-visual { height: 200px; width: 100%; background: #e0e0e0; border-radius: 5px; position: relative; overflow: hidden; margin: 20px 0; border: 1px solid #ddd; }
    .water-level { position: absolute; bottom: 0; width: 100%; background: var(--water-color); transition: all 0.5s ease; }
    .footer { margin-top: 20px; font-size: 12px; color: #999; text-align: center; border-top: 1px solid #eee; padding-top: 15px; }
    .config-link { display: inline-block; margin-top: 15px; color: #4285f4; text-decoration: none; font-weight: 500; }
    .config-link:hover { text-decoration: underline; }
    .last-update { font-size: 12px; color: #999; text-align: right; margin-top: 10px; }
  </style>
</head>
<body>
  <div class="container">
    <h1>Water Tank Dashboard</h1>
    <div class="last-update" id="lastUpdate">Last update: Just now</div>
    
    <div class="gauge-container">
      <div class="gauge" id="gauge" style="--water-color: )=====");
  html += waterColor + "; --water-level: " + String(percent) + "%;";
  html += FPSTR(R"=====(">
        <div class="gauge-inner">
          <span id="percentValue">)=====");
  html += String(percent, 1) + "%";
  html += FPSTR(R"=====(</span>
        </div>
      </div>
    </div>
    
    <div class="stats">
      <div class="stat-card">
        <h3>Distance</h3>
        <p id="distanceValue">)=====");
  html += String(cm, 1) + " cm";
  html += FPSTR(R"=====(</p>
      </div>
      <div class="stat-card">
        <h3>Volume</h3>
        <p id="volumeValue">)=====");
  html += String(volume, 1) + " L";
  html += FPSTR(R"=====(</p>
      </div>
      <div class="stat-card">
        <h3>Tank Width</h3>
        <p>)=====");
  html += String(tankWidth) + " cm";
  html += FPSTR(R"=====(</p>
      </div>
      <div class="stat-card">
        <h3>Tank Height</h3>
        <p>)=====");
  html += String(tankHeight) + " cm";
  html += FPSTR(R"=====(</p>
      </div>
    </div>
    
    <div class="tank-visual">
      <div class="water-level" id="waterLevel" style="height: )=====");
  html += String(percent) + "%; background: " + waterColor;
  html += FPSTR(R"=====(;"></div>
    </div>
    
    <div class="footer">
      <p>MAC: )=====");
  html += WiFi.macAddress();
  html += FPSTR(R"=====(<br>
      Uptime: <span id="uptimeValue">)=====");
  html += formatRuntime(millis());
  html += FPSTR(R"=====(</span></p>
      <a href="/config" class="config-link">Configure Tank</a>
    </div>
  </div>

  <script>
    function updateDashboard() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          const waterColor = data.percent < 20 ? '#ff4444' : '#4285f4';
          
          // Update values
          document.getElementById('percentValue').textContent = data.percent.toFixed(1) + '%';
          document.getElementById('distanceValue').textContent = data.distance.toFixed(1) + ' cm';
          document.getElementById('volumeValue').textContent = data.volume.toFixed(0);
          document.getElementById('uptimeValue').textContent = data.uptime;
          
          // Update water level and gauge
          const waterLevel = document.getElementById('waterLevel');
          const gauge = document.getElementById('gauge');
          
          waterLevel.style.height = data.percent + '%';
          waterLevel.style.backgroundColor = waterColor;
          
          gauge.style.setProperty('--water-color', waterColor);
          gauge.style.setProperty('--water-level', data.percent + '%');
          
          // Update last update time
          const now = new Date();
          document.getElementById('lastUpdate').textContent = 
            'Last update: ' + now.toLocaleTimeString();
        })
        .catch(error => {
          console.error('Error fetching data:', error);
        });
    }

    updateDashboard();
    setInterval(updateDashboard, )=====");
  html += String(UPDATE_INTERVAL);
  html += FPSTR(R"=====();
  </script>
</body>
</html>
)=====");

  server.send(200, "text/html", html);
}

void handleData() {
  updateMeasurements();

  String json = "{";
  json += "\"percent\":" + String(percent, 1) + ",";
  json += "\"distance\":" + String(cm, 1) + ",";
  json += "\"volume\":" + String(volume, 1) + ",";
  json += "\"uptime\":\"" + formatRuntime(millis()) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleConfig() {
  if (server.hasArg("tank_size")) {
    int presetIndex = server.arg("tank_size").toInt();
    if (presetIndex >= 0 && presetIndex < sizeof(TANK_PRESETS) / sizeof(TANK_PRESETS[0])) {
      tankWidth = TANK_PRESETS[presetIndex][0];
      tankHeight = TANK_PRESETS[presetIndex][1];
      volumeFactor = TANK_PRESETS[presetIndex][2];
      saveConfig();
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  String html = FPSTR(R"=====(
<!DOCTYPE html>
<html lang='en'>
<head>
  <meta charset='UTF-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>Tank Configuration</title>
  <style>
    :root {
      --primary-color: #3498db;
      --secondary-color: #2980b9;
      --text-color: #333;
      --light-bg: #f9f9f9;
      --border-color: #e0e0e0;
    }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      line-height: 1.6;
      color: var(--text-color);
      background-color: var(--light-bg);
      margin: 0;
      padding: 20px;
      max-width: 600px;
      margin: 0 auto;
    }
    h1, h2, h3 {
      color: var(--primary-color);
      text-align: center;
    }
    .config-form {
      background: white;
      padding: 25px;
      border-radius: 8px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.1);
      margin-top: 20px;
    }
    .form-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 600;
    }
    select, input[type='number'] {
      width: 100%;
      padding: 10px;
      border: 1px solid var(--border-color);
      border-radius: 4px;
      font-size: 16px;
      box-sizing: border-box;
    }
    input[type='submit'] {
      background-color: var(--primary-color);
      color: white;
      border: none;
      padding: 12px 20px;
      border-radius: 4px;
      font-size: 16px;
      cursor: pointer;
      width: 100%;
      transition: background-color 0.3s;
    }
    input[type='submit']:hover {
      background-color: var(--secondary-color);
    }
    .back-link {
      display: block;
      text-align: center;
      margin-top: 20px;
      color: var(--primary-color);
      text-decoration: none;
    }
    .back-link:hover {
      text-decoration: underline;
    }
  </style>
</head>
<body>
  <div class="config-form">
    <h2>Tank Configuration</h2>
    <form method='GET'>
      <div class="form-group">
        <label for='tank_size'>Tank Size</label>
        <select id='tank_size' name='tank_size' required>
          <option value='0'>500L (68cm × 162cm)</option>
          <option value='1'>700L (79.5cm × 170cm)</option>
          <option value='2'>1000L (92.5cm × 181cm)</option>
          <option value='3'>1500L (109cm × 196cm)</option>
          <option value='4'>2000L (123cm × 205cm)</option>
        </select>
      </div>
      <input type='submit' value='Save Configuration'>
    </form>
  </div>
  <a href='/' class='back-link'>&larr; Back to Dashboard</a>
</body>
</html>
)=====");

  server.send(200, "text/html", html);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  sensorSerial.begin(9600);

  EEPROM.begin(EEPROM_SIZE);
  loadConfig();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("TankMonitor", "12345678");
  Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // --- Captive Portal Setup ---
  // Start the DNS server to redirect all traffic to the ESP's IP
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  // OTA Setup
  ArduinoOTA.setHostname("TankMonitor");
  ArduinoOTA.begin();

  // Web Server Setup
  server.on("/", handleRoot);
  server.on("/config", handleConfig);
  server.on("/data", handleData);
  // Redirect any other requests to the root page
  server.onNotFound(handleRoot); 
  server.begin();

  // ESP-NOW Setup
  if (esp_now_init() != 0) {
    Serial.println("{\"error\":\"ESP-NOW init failed\"}");
    delay(1000);
    ESP.restart();
  }

  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);

  if (esp_now_add_peer(senderMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) != 0) {
    Serial.println("{\"error\":\"Failed to add peer\"}");
    delay(1000);
    ESP.restart();
  }

  Serial.printf("{\"status\":\"Ready\",\"calibration_cm\":%.1f}\n", calibration_mm / 10.0f);

  // Initial reading
  if (readDistance(mm, cm)) {
    updateMeasurements();
  }
}

// --- Main Loop ---
void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  dnsServer.processNextRequest(); // Process DNS requests for captive portal

  if (pendingDistanceRequest || pendingDistanceRequest_2) {
    bool success = readDistance(mm, cm);

    if (success) {
      updateMeasurements();

      if (pendingDistanceRequest) {
        String json = "{\"distance_cm\":" + String(cm, 1) + ",\"calibration_cm\":" + String(calibration_mm / 10.0f) + "}";
        sendESPNowResponse(true, json.c_str());
      } else if (pendingDistanceRequest_2) {
        sendESPNowResponse(true, jsonDataOut.c_str());
      }
    } else {
      sendESPNowResponse(false);
    }

    pendingDistanceRequest = false;
    pendingDistanceRequest_2 = false;
  }

  // Non-blocking delay
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 100) {
    lastUpdate = millis();
    ESP.wdtFeed();
  }
}
