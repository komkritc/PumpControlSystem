/*
 * Smart Water Tank Monitoring System with Optimized Sensor Reading and RS232 Commands
 *
 * Description:
 * This ESP8266-based system provides comprehensive monitoring of water tank levels with:
 * - Optimized ultrasonic distance measurement for faster and more accurate readings
 * - Multiple tank size presets AND manual dimension input
 * - ESP-NOW communication for wireless sensor networking
 * - Real-time web dashboard with visual tank representation and captive portal
 * - OTA firmware update capability
 * - Configuration interface for tank dimensions and calibration
 * - Advanced sensor health monitoring and performance metrics
 * - RS232 command interface for remote control
 *
 * Features:
 * - Non-blocking sensor reading with 60-120ms response time
 * - Buffered averaging for stable measurements
 * - Burst reading mode for ESP-NOW requests
 * - Automatic outlier detection and filtering
 * - Sensor health monitoring and diagnostics
 * - Performance benchmarking capabilities
 * - Serial command processing (mac, read, benchmark, reset)
 *
 * Hardware Requirements:
 * - ESP8266 (NodeMCU or similar)
 * - Ultrasonic distance sensor (JSN-SR04T or similar)
 * - Optional: Secondary ESP8266 for remote monitoring
 *
 * Designed by: Komkrit Chooraung
 * Date: 11-8-2025
 * Version: 2.1 (Added RS232 Commands)
 * Version: 3.1 (Added WiFi SSID Connection)
 * Version: 3.2 (Added WiFi SSID Connection Config Page)
 * Version: 4.0 (1st Release Candidate - AsyncMQTT - Web Benchmark)
 */

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include <Ticker.h>
#include <AsyncMqtt_Generic.h>


WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600);  // UTC+7 (Bangkok time)

// --- Constants ---
#define EEPROM_SIZE 128
#define ADDR_WIDTH 0       // float (4 bytes)
#define ADDR_HEIGHT 4      // float (4 bytes)
#define ADDR_VOL_FACTOR 8  // float (4 bytes)
#define ADDR_CALIB 12      // int (4 bytes)
#define ADDR_WIFI_SSID 16  // 32 bytes
#define ADDR_WIFI_PASS 48  // 32 bytes
#define ADDR_WIFI_MAC 80   // 6 bytes for MAC address

// --- Config Macros ---
#define DEVICE_NAME "TankMonitor888"  // Device name (AP/mDNS/OTA)
#define WIFI_AP_PASSWORD "12345678"   // AP mode password
#define OTA_PASSWORD "12345678"       // OTA update password (optional)
#define MQTT_HOST "broker.emqx.io"    // Broker address
#define MQTT_PORT 1883

#define LED_BLUE D4   // Status/Activity LED
#define LED_RED D7    // Error/AP Mode LED
#define LED_GREEN D8  // WiFi Connected LED
#define LED_ON HIGH   // Assuming common cathode (LOW turns on)
#define LED_OFF LOW

#define RX_PIN D1
#define TX_PIN D2
#define SENSOR_READ_TIMEOUT 100        // ms
#define SERIAL_INPUT_TIMEOUT 5000      // ms
#define UPDATE_INTERVAL 2000           // ms for web dashboard updates
#define SERIAL_COMMAND_BUFFER_SIZE 32  // Buffer size for serial commands

const char *PubTopic = "TankMonitor144/status";  // Topic to publish

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

const byte DNS_PORT = 53;  // DNS server port for captive portal
bool wifiConnected = false;

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
DNSServer dnsServer;
SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

// Serial command buffer
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Performance Monitoring ---
struct PerformanceMetrics {
  unsigned long totalReadings;
  unsigned long successfulReadings;
  unsigned long averageReadTime;
  unsigned long maxReadTime;
  unsigned long minReadTime;

  void reset() {
    totalReadings = 0;
    successfulReadings = 0;
    averageReadTime = 0;
    maxReadTime = 0;
    minReadTime = ULONG_MAX;
  }

  void addMeasurement(unsigned long readTime, bool success) {
    totalReadings++;
    if (success) {
      successfulReadings++;

      if (readTime > maxReadTime) maxReadTime = readTime;
      if (readTime < minReadTime) minReadTime = readTime;

      // Simple moving average
      averageReadTime = (averageReadTime * (successfulReadings - 1) + readTime) / successfulReadings;
    }
  }

  float getSuccessRate() {
    if (totalReadings == 0) return 0.0f;
    return (float)successfulReadings / totalReadings * 100.0f;
  }
};

PerformanceMetrics performanceMetrics;

// ===== UTILITY FUNCTIONS =====
void blinkBlueLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_BLUE, LED_ON);
    delay(delayTime);
    digitalWrite(LED_BLUE, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void blinkGreenLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_GREEN, LED_ON);
    delay(delayTime);
    digitalWrite(LED_GREEN, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void blinkRedLED(int count, int delayTime = 200) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_RED, LED_ON);
    delay(delayTime);
    digitalWrite(LED_RED, LED_OFF);
    if (i < count - 1) delay(delayTime);
  }
}

void initLEDs() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

// --- Buffered Reading Manager ---
class SensorReadingManager {
private:
  static const int BUFFER_SIZE = 5;
  float readings[BUFFER_SIZE];
  int bufferIndex;
  int validReadings;
  unsigned long lastSuccessfulRead;
  bool initialized;

  // Statistics for error detection
  float lastValidReading;
  int consecutiveErrors;

public:
  SensorReadingManager()
    : bufferIndex(0), validReadings(0), lastSuccessfulRead(0),
      initialized(false), lastValidReading(0), consecutiveErrors(0) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
      readings[i] = 0;
    }
  }

  bool getStableReading(uint16_t &out_mm, float &out_cm);
  float getAverage();
  bool hasRecentReading();
  void reset();
  int getValidReadingsCount();
};

// Global sensor manager instance
SensorReadingManager sensorManager;

// --- Serial Command Functions ---

// High-speed burst reading for ESP-NOW requests
bool readDistanceBurst(uint16_t &out_mm, float &out_cm, int attempts = 3) {
  for (int i = 0; i < attempts; i++) {
    unsigned long startTime = millis();

    // Quick trigger and read cycle
    sensorSerial.flush();
    sensorSerial.write(0x55);

    // Wait for response with shorter timeout
    unsigned long timeoutStart = millis();
    while (millis() - timeoutStart < 60) {  // 60ms timeout
      if (sensorSerial.available() >= 4) {
        uint8_t buffer[4];
        if (sensorSerial.readBytes(buffer, 4) == 4 && buffer[0] == 0xFF) {
          uint16_t raw_mm = (buffer[1] << 8) | buffer[2];

          if (raw_mm > 50 && raw_mm < 5000) {
            out_mm = raw_mm + calibration_mm;
            out_cm = out_mm / 10.0f;
            return true;
          }
        }
        break;
      }
      delay(1);  // Small delay to prevent tight polling
    }

    // Small delay between attempts
    if (i < attempts - 1) {
      delay(20);
    }
  }

  return false;
}

// Serial benchmarking function (different from web benchmark)
void runSerialBenchmark(int iterations = 30) {
  Serial.printf("Starting serial benchmark with %d iterations...\n", iterations);
  performanceMetrics.reset();

  unsigned long benchmarkStart = millis();
  int successCount = 0;

  for (int i = 0; i < iterations; i++) {
    uint16_t test_mm;
    float test_cm;

    unsigned long startTime = millis();
    bool success = readDistanceBurst(test_mm, test_cm, 1);
    unsigned long endTime = millis();

    unsigned long readTime = endTime - startTime;
    performanceMetrics.addMeasurement(readTime, success);

    if (success) {
      successCount++;
      Serial.printf("[%d/%d] SUCCESS: %.1f cm (%d ms)\n",
                    i + 1, iterations, test_cm, readTime);
    } else {
      Serial.printf("[%d/%d] FAILED (%d ms)\n",
                    i + 1, iterations, readTime);
    }

    delay(50);  // Small delay between tests
  }

  unsigned long benchmarkTotal = millis() - benchmarkStart;

  Serial.println("=== Benchmark Results ===");
  Serial.printf("Total time: %lu ms\n", benchmarkTotal);
  Serial.printf("Success rate: %.1f%% (%d/%d)\n",
                performanceMetrics.getSuccessRate(), successCount, iterations);
  Serial.printf("Average read time: %lu ms\n", performanceMetrics.averageReadTime);
  Serial.printf("Min read time: %lu ms\n", performanceMetrics.minReadTime);
  Serial.printf("Max read time: %lu ms\n", performanceMetrics.maxReadTime);
  Serial.println("=========================");
}


void processSerialCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "mac") {
    // Display MAC address in the requested format
    String macStr = WiFi.macAddress();
    macStr.replace(":", ", 0x");
    macStr = "0x" + macStr;
    Serial.printf("MAC[] = { %s }\n", macStr.c_str());

    // Also show ESP-NOW peer MAC
    Serial.printf("ESP-NOW Peer MAC[] = { 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X }\n",
                  senderMAC[0], senderMAC[1], senderMAC[2],
                  senderMAC[3], senderMAC[4], senderMAC[5]);

  } else if (command == "read") {
    // Read sensor and display value
    Serial.println("Reading sensor...");

    uint16_t read_mm;
    float read_cm;

    // Try burst reading for immediate response
    bool success = readDistanceBurst(read_mm, read_cm, 3);

    if (success) {
      // Update global variables
      mm = read_mm;
      cm = read_cm;
      updateMeasurements();

      Serial.printf("Sensor Reading:\n");
      Serial.printf("  Distance: %.1f cm (%d mm)\n", cm, mm);
      Serial.printf("  Water Level: %.1f%%\n", percent);
      Serial.printf("  Volume: %.1f L\n", volume);
      Serial.printf("  Calibration: %.1f cm\n", calibration_mm / 10.0f);
    } else {
      Serial.println("Error: Failed to read sensor");
    }

  } else if (command == "benchmark") {
    // Run benchmark
    Serial.println("Starting sensor benchmark...");
    runSerialBenchmark(30);  // Run 30 iterations by default

  } else if (command == "reset") {
    // Reset ESP
    Serial.println("Resetting ESP8266...");
    Serial.flush();
    delay(100);
    ESP.restart();

  } else if (command.length() > 0) {
    // Unknown command
    Serial.printf("Unknown command: '%s'\n", command.c_str());
    Serial.println("Available commands: mac, read, benchmark, reset");
  }
}

void handleSerialInput() {
  while (Serial.available()) {
    char inChar = Serial.read();

    // Handle newline characters (CR or LF)
    if (inChar == '\n' || inChar == '\r') {
      if (serialBufferIndex > 0) {
        serialCommandBuffer[serialBufferIndex] = '\0';
        String command = String(serialCommandBuffer);
        processSerialCommand(command);
        serialBufferIndex = 0;
      }
    }
    // Handle backspace
    else if (inChar == '\b' || inChar == 127) {
      if (serialBufferIndex > 0) {
        serialBufferIndex--;
        Serial.print("\b \b");  // Echo backspace
      }
    }
    // Regular characters
    else if (inChar >= 32 && inChar <= 126) {  // Printable characters
      if (serialBufferIndex < SERIAL_COMMAND_BUFFER_SIZE - 1) {
        serialCommandBuffer[serialBufferIndex] = inChar;
        serialBufferIndex++;
        Serial.print(inChar);  // Echo character
      }
    }
  }
}

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

// --- Optimized Sensor Functions ---

// Non-blocking sensor reading with improved speed
bool readDistanceFast(uint16_t &out_mm, float &out_cm) {
  static unsigned long lastTrigger = 0;
  static bool waitingForResponse = false;
  static unsigned long responseStartTime = 0;

  unsigned long currentTime = millis();

  // Trigger sensor reading if not already waiting and enough time has passed
  if (!waitingForResponse && (currentTime - lastTrigger >= 50)) {  // Reduced from 100ms
    sensorSerial.flush();
    sensorSerial.write(0x55);
    waitingForResponse = true;
    responseStartTime = currentTime;
    lastTrigger = currentTime;
    return false;  // Data not ready yet
  }

  // Check for response if we're waiting
  if (waitingForResponse) {
    // Timeout check - reduced from 100ms to 80ms
    if (currentTime - responseStartTime > 80) {
      waitingForResponse = false;
      return false;  // Timeout
    }

    // Check if we have enough data
    if (sensorSerial.available() >= 4) {
      uint8_t buffer[4];
      int bytesRead = sensorSerial.readBytes(buffer, 4);

      if (bytesRead == 4 && buffer[0] == 0xFF) {
        uint16_t raw_mm = (buffer[1] << 8) | buffer[2];

        // Basic range validation to filter out obvious errors
        if (raw_mm > 50 && raw_mm < 5000) {  // 5cm to 500cm range
          out_mm = raw_mm + calibration_mm;
          out_cm = out_mm / 10.0f;
          waitingForResponse = false;
          return true;
        }
      }
      waitingForResponse = false;
    }
  }

  return false;
}

// Sensor Reading Manager Implementation
bool SensorReadingManager::getStableReading(uint16_t &out_mm, float &out_cm) {
  uint16_t raw_mm;
  float raw_cm;

  if (readDistanceFast(raw_mm, raw_cm)) {
    // Outlier detection - reject readings that are too different from recent average
    if (initialized && validReadings > 2) {
      float currentAverage = getAverage();
      float difference = abs(raw_cm - currentAverage);

      // Reject reading if it's more than 20cm different from average (adjustable)
      if (difference > 20.0f) {
        consecutiveErrors++;

        // If we have too many consecutive errors, reset the buffer
        if (consecutiveErrors > 3) {
          reset();
        }
        return false;
      }
    }

    // Add reading to buffer
    readings[bufferIndex] = raw_cm;
    bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

    if (validReadings < BUFFER_SIZE) {
      validReadings++;
    }

    consecutiveErrors = 0;
    lastSuccessfulRead = millis();
    lastValidReading = raw_cm;
    initialized = true;

    // Return averaged result
    float averaged = getAverage();
    out_cm = averaged;
    out_mm = (uint16_t)(averaged * 10);

    return true;
  }

  return false;
}

float SensorReadingManager::getAverage() {
  if (validReadings == 0) return 0;

  float sum = 0;
  for (int i = 0; i < validReadings; i++) {
    sum += readings[i];
  }
  return sum / validReadings;
}

bool SensorReadingManager::hasRecentReading() {
  return (millis() - lastSuccessfulRead) < 5000;  // 5 seconds
}

void SensorReadingManager::reset() {
  bufferIndex = 0;
  validReadings = 0;
  consecutiveErrors = 0;
  initialized = false;
}

int SensorReadingManager::getValidReadingsCount() {
  return validReadings;
}

// Main reading function that uses the optimized methods
bool readDistance(uint16_t &out_mm, float &out_cm) {
  return sensorManager.getStableReading(out_mm, out_cm);
}

// Function to check sensor health and connectivity
bool checkSensorHealth() {
  uint16_t test_mm;
  float test_cm;

  // Try a burst read to test connectivity
  return readDistanceBurst(test_mm, test_cm, 2);
}

// Function to get sensor statistics
String getSensorStats() {
  String stats = "{";
  stats += "\"validReadings\":" + String(sensorManager.getValidReadingsCount()) + ",";
  stats += "\"hasRecentData\":" + String(sensorManager.hasRecentReading() ? "true" : "false") + ",";
  stats += "\"averageReading\":" + String(sensorManager.getAverage(), 1) + ",";
  stats += "\"performanceSuccessRate\":" + String(performanceMetrics.getSuccessRate(), 1) + ",";
  stats += "\"totalReadings\":" + String(performanceMetrics.totalReadings) + ",";
  stats += "\"avgReadTime\":" + String(performanceMetrics.averageReadTime);
  stats += "}";
  return stats;
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

// Modified ESP-NOW handler functions for faster response
void handleESPNowRequest() {
  if (pendingDistanceRequest || pendingDistanceRequest_2) {
    uint16_t burst_mm;
    float burst_cm;

    unsigned long startTime = millis();

    // Use burst reading for ESP-NOW requests for faster response
    bool success = readDistanceBurst(burst_mm, burst_cm, 2);

    unsigned long readTime = millis() - startTime;
    performanceMetrics.addMeasurement(readTime, success);

    if (success) {
      // Update global variables for consistency
      mm = burst_mm;
      cm = burst_cm;
      updateMeasurements();
      blinkBlueLED(1);

      if (pendingDistanceRequest) {
        String json = "{\"distance_cm\":" + String(cm, 1) + ",\"calibration_cm\":" + String(calibration_mm / 10.0f, 1) + "}";
        sendESPNowResponse(true, json.c_str());
      } else if (pendingDistanceRequest_2) {
        sendESPNowResponse(true, jsonDataOut.c_str());
      }
    } else {
      sendESPNowResponse(false);
    }

    pendingDistanceRequest = false;
    pendingDistanceRequest_2 = false;
    blinkGreenLED(1);
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
  float maxVolume = volumeFactor * tankHeight;

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
    .tank-container { margin: 20px 0; position: relative; }
    .tank-visual { width: 100%; height: 200px; background: #e0e0e0; border-radius: 5px; position: relative; overflow: hidden; border: 1px solid #ddd; }
    .water-level { position: absolute; bottom: 0; width: 100%; background: var(--water-color); transition: all 0.5s ease; }
    .tank-labels { display: flex; flex-direction: column; justify-content: flex-start; align-items: flex-start; margin-top: 5px;}
    .tank-label { font-size: 12px; color: #666; }
    .current-level { position: absolute; left: 50%; transform: translateX(-50%); bottom: calc(%PERCENT%% - 12px); font-size: 12px; background: white; padding: 2px 5px; border-radius: 3px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); z-index: 2; }
    .footer { margin-top: 20px; font-size: 12px; color: #999; text-align: center; border-top: 1px solid #eee; padding-top: 15px; }
    .config-link { display: inline-block; margin: 5px; color: #4285f4; text-decoration: none; font-weight: 500; }
    .config-link:hover { text-decoration: underline; }
    .last-update { font-size: 12px; color: #999; text-align: right; margin-top: 10px; }
    .health-indicator { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 5px; }
    .health-healthy { background-color: #4CAF50; }
    .health-warning { background-color: #FF9800; }
    .health-error { background-color: #F44336; }
    /* Modern Footer */
.footer {
  margin-top: 2rem;
  padding-top: 1.5rem;
  border-top: 1px solid #eee;
  font-size: 0.9rem;
}

.footer-links {
  display: flex;
  justify-content: center;
  gap: 1.5rem;
  margin-bottom: 1.5rem;
  flex-wrap: wrap;
}

.footer-link {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  color: #4361ee;
  text-decoration: none;
  font-weight: 500;
  transition: color 0.2s;
}

.footer-link:hover {
  color: #3a56d4;
}

.icon {
  width: 18px;
  height: 18px;
  fill: currentColor;
}

.footer-meta {
  text-align: center;
  color: #666;
  line-height: 1.6;
}

.footer-meta-item {
  margin: 0.3rem 0;
}

@media (max-width: 600px) {
  .footer-links {
    gap: 1rem;
  }
}
  </style>
</head>
<body>
  <div class="container">
    <h1>Water Tank Dashboard</h1>
    <div class="last-update" id="lastUpdate">
      <span class="health-indicator" id="healthIndicator"></span>
      Last update: Just now
    </div>
    
    <div class="gauge-container">
      <div class="gauge" id="gauge" style="--water-color: %WATER_COLOR%; --water-level: %PERCENT%%;">
        <div class="gauge-inner">
          <span id="percentValue">%PERCENT% %</span>
        </div>
      </div>
    </div>
    
    <div class="stats">
      <div class="stat-card">
        <h3>Distance</h3>
        <p id="distanceValue">%DISTANCE% cm</p>
      </div>
      <div class="stat-card">
        <h3>Volume</h3>
        <p id="volumeValue">%VOLUME% L</p>
      </div>
      <div class="stat-card">
        <h3>Tank Width</h3>
        <p>%TANK_WIDTH% cm</p>
      </div>
      <div class="stat-card">
        <h3>Tank Height</h3>
        <p>%TANK_HEIGHT% cm</p>
      </div>
    </div>
    
    <div class="tank-container">
      <div class="tank-labels">
        <span class="tank-label">%MAX_VOLUME% L (MAX)</span>
      </div>
      <div class="tank-visual">
        <div class="water-level" id="waterLevel" style="height: %PERCENT%%; background: %WATER_COLOR%;"></div>
        <div class="current-level" id="currentLevel">%VOLUME% L (%PERCENT%%)</div>
      </div>
      <div class="tank-labels">
        <span class="tank-label">0 L (MIN)</span>
      </div>
    </div>
    
<div class="footer">
  <div class="footer-links">
    <a href="/config" class="footer-link">
      <svg class="icon" viewBox="0 0 24 24"><path d="M19.14 12.94c.04-.3.06-.61.06-.94 0-.32-.02-.64-.07-.94l2.03-1.58c.18-.14.23-.41.12-.61l-1.92-3.32c-.12-.22-.37-.29-.59-.22l-2.39.96c-.5-.38-1.03-.7-1.62-.94l-.36-2.54c-.04-.22-.2-.4-.43-.4h-3.84c-.23 0-.39.18-.43.4l-.36 2.54c-.59.24-1.13.57-1.62.94l-2.39-.96c-.22-.08-.47 0-.59.22L2.74 8.87c-.12.21-.08.47.12.61l2.03 1.58c-.05.3-.09.63-.09.94s.02.64.07.94l-2.03 1.58c-.18.14-.23.41-.12.61l1.92 3.32c.12.22.37.29.59.22l2.39-.96c.5.38 1.03.7 1.62.94l.36 2.54c.04.22.2.4.43.4h3.84c.23 0 .39-.18.43-.4l.36-2.54c.59-.24 1.13-.56 1.62-.94l2.39.96c.22.08.47 0 .59-.22l1.92-3.32c.12-.22.07-.47-.12-.61l-2.01-1.58zM12 15.6c-1.98 0-3.6-1.62-3.6-3.6s1.62-3.6 3.6-3.6 3.6 1.62 3.6 3.6-1.62 3.6-3.6 3.6z"/></svg>
      Configure
    </a>
    <a href="/sensor-stats" class="footer-link">
      <svg class="icon" viewBox="0 0 24 24"><path d="M16 11c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM8 11c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM8 19c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3zM16 19c1.66 0 3-1.34 3-3s-1.34-3-3-3-3 1.34-3 3 1.34 3 3 3z"/></svg>
      Stats
    </a>
    <a href="/benchmark" class="footer-link">
      <svg class="icon" viewBox="0 0 24 24"><path d="M15.6 10.79c.97-.67 1.65-1.77 1.65-2.79 0-2.26-1.75-4-4-4H7v14h7.04c2.09 0 3.71-1.7 3.71-3.79 0-1.52-.86-2.82-2.15-3.42zM10 6.5h3c.83 0 1.5.67 1.5 1.5s-.67 1.5-1.5 1.5h-3v-3zm3.5 9H10v-3h3.5c.83 0 1.5.67 1.5 1.5s-.67 1.5-1.5 1.5z"/></svg>
      Benchmark
    </a>
    <a href="/wifi-setup" class="footer-link">
      <svg class="icon" viewBox="0 0 24 24"><path d="M1 9l2 2c4.97-4.97 13.03-4.97 18 0l2-2C16.93 2.93 7.08 2.93 1 9zm8 8l3 3 3-3c-1.65-1.66-4.34-1.66-6 0zm-4-4l2 2c2.76-2.76 7.24-2.76 10 0l2-2C15.14 9.14 8.87 9.14 5 13z"/></svg>
      WiFi
    </a>
  </div>
  <div class="footer-meta">
    <p class="footer-meta-item">MAC: %MAC_ADDRESS%</p>
    <p class="footer-meta-item">Uptime: <span id="uptimeValue">%UPTIME%</span></p>
    <p class="footer-meta-item">Valid Readings: <span id="validReadings">%VALID_READINGS%</span></p>
  </div>
</div>

  <script>
    function updateDashboard() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          const waterColor = data.percent < 20 ? '#ff4444' : '#4285f4';
          
          document.getElementById('percentValue').textContent = data.percent.toFixed(1) + '%';
          document.getElementById('distanceValue').textContent = data.distance.toFixed(1) + ' cm';
          document.getElementById('volumeValue').textContent = data.volume.toFixed(1) + ' L';
          document.getElementById('uptimeValue').textContent = data.uptime;
          document.getElementById('validReadings').textContent = data.valid_readings || '0';
          
          const waterLevel = document.getElementById('waterLevel');
          const currentLevel = document.getElementById('currentLevel');
          const gauge = document.getElementById('gauge');
          const healthIndicator = document.getElementById('healthIndicator');
          
          waterLevel.style.height = data.percent + '%';
          waterLevel.style.backgroundColor = waterColor;
          
          currentLevel.style.bottom = 'calc(' + data.percent + '% - 12px)';
          currentLevel.textContent = data.volume.toFixed(1) + ' L (' + data.percent.toFixed(1) + '%)';
          
          gauge.style.setProperty('--water-color', waterColor);
          gauge.style.setProperty('--water-level', data.percent + '%');
          
          // Update health indicator
          if (data.sensor_health) {
            healthIndicator.className = 'health-indicator health-healthy';
          } else {
            healthIndicator.className = 'health-indicator health-error';
          }
          
          const now = new Date();
          document.getElementById('lastUpdate').innerHTML = 
            '<span class="health-indicator ' + (data.sensor_health ? 'health-healthy' : 'health-error') + '"></span>' +
            'Last update: ' + now.toLocaleTimeString();
        })
        .catch(error => {
          console.error('Error fetching data:', error);
          const healthIndicator = document.getElementById('healthIndicator');
          healthIndicator.className = 'health-indicator health-error';
        });
    }

    updateDashboard();
    setInterval(updateDashboard, %UPDATE_INTERVAL%);
  </script>
</body>
</html>
)=====");

  // Replace placeholders with actual values
  html.replace("%WATER_COLOR%", waterColor);
  html.replace("%PERCENT%", String(percent, 1));
  html.replace("%DISTANCE%", String(cm, 1));
  html.replace("%VOLUME%", String(volume, 1));
  html.replace("%TANK_WIDTH%", String(tankWidth));
  html.replace("%TANK_HEIGHT%", String(tankHeight));
  html.replace("%MAX_VOLUME%", String(maxVolume, 1));
  html.replace("%MAC_ADDRESS%", WiFi.macAddress());
  html.replace("%UPTIME%", formatRuntime(millis()));
  html.replace("%VALID_READINGS%", String(sensorManager.getValidReadingsCount()));
  html.replace("%UPDATE_INTERVAL%", String(UPDATE_INTERVAL));

  server.send(200, "text/html", html);
}

void handleData() {
  updateMeasurements();

  // Include sensor health information
  bool hasRecent = sensorManager.hasRecentReading();
  int validReadings = sensorManager.getValidReadingsCount();

  String json = "{";
  json += "\"percent\":" + String(percent, 1) + ",";
  json += "\"distance\":" + String(cm, 1) + ",";
  json += "\"volume\":" + String(volume, 1) + ",";
  json += "\"uptime\":\"" + formatRuntime(millis()) + "\",";
  json += "\"sensor_health\":" + String(hasRecent ? "true" : "false") + ",";
  json += "\"valid_readings\":" + String(validReadings);
  json += "}";

  server.send(200, "application/json", json);
}

void handleSensorStats() {
  String json = getSensorStats();
  server.send(200, "application/json", json);
}

void handleConfig() {
  // Handle form submission
  if (server.hasArg("save")) {
    if (server.hasArg("preset_mode") && server.arg("preset_mode") == "1") {
      // Preset mode selected
      int presetIndex = server.arg("preset_size").toInt();
      if (presetIndex >= 0 && presetIndex < sizeof(TANK_PRESETS) / sizeof(TANK_PRESETS[0])) {
        tankWidth = TANK_PRESETS[presetIndex][0];
        tankHeight = TANK_PRESETS[presetIndex][1];
        volumeFactor = TANK_PRESETS[presetIndex][2];
      }
    } else {
      // Manual mode selected
      tankWidth = server.arg("manual_width").toFloat();
      tankHeight = server.arg("manual_height").toFloat();

      // Calculate volume factor (assuming rectangular tank)
      volumeFactor = (tankWidth * tankWidth) / 1000.0f;  // converts cm³ to liters per cm height
    }

    // Save calibration if provided
    if (server.hasArg("calibration")) {
      calibration_mm = server.arg("calibration").toInt() * 10;  // convert cm to mm
    }

    saveConfig();
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  // Generate the configuration page
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
    select, input[type='number'], input[type='text'] {
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
      margin-top: 15px;
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
    .mode-selector {
      display: flex;
      margin-bottom: 20px;
      border-bottom: 1px solid var(--border-color);
      padding-bottom: 15px;
    }
    .mode-option {
      flex: 1;
      text-align: center;
      padding: 10px;
      cursor: pointer;
      border-bottom: 3px solid transparent;
    }
    .mode-option.active {
      border-bottom-color: var(--primary-color);
      font-weight: bold;
    }
    .mode-option input {
      display: none;
    }
    .manual-inputs {
      display: none;
    }
    .preset-inputs {
      display: block;
    }
    .calibration-input {
      margin-top: 20px;
      padding-top: 20px;
      border-top: 1px solid var(--border-color);
    }
  </style>
</head>
<body>
  <div class="config-form">
    <h2>Tank Configuration</h2>
    <form method='GET' action='/config'>
      <input type='hidden' name='save' value='1'>
      
      <div class="mode-selector">
        <label class="mode-option active" onclick="toggleMode('preset')">
          <input type="radio" name="preset_mode" value="1" checked> Preset Tank
        </label>
        <label class="mode-option" onclick="toggleMode('manual')">
          <input type="radio" name="preset_mode" value="0"> Custom Tank
        </label>
      </div>
      
      <div class="preset-inputs" id="preset-inputs">
        <div class="form-group">
          <label for='preset_size'>Tank Size</label>
          <select id='preset_size' name='preset_size' required>
            <option value='0'>500L (68cm × 162cm)</option>
            <option value='1'>700L (79.5cm × 170cm)</option>
            <option value='2'>1000L (92.5cm × 181cm)</option>
            <option value='3'>1500L (109cm × 196cm)</option>
            <option value='4'>2000L (123cm × 205cm)</option>
          </select>
        </div>
      </div>
      
      <div class="manual-inputs" id="manual-inputs">
        <div class="form-group">
          <label for='manual_width'>Tank Width (cm)</label>
          <input type='number' id='manual_width' name='manual_width' step='0.1' min='10' max='300' value=')=====");
  html += String(tankWidth, 1);
  html += FPSTR(R"=====('>
        </div>
        <div class="form-group">
          <label for='manual_height'>Tank Height (cm)</label>
          <input type='number' id='manual_height' name='manual_height' step='0.1' min='10' max='300' value=')=====");
  html += String(tankHeight, 1);
  html += FPSTR(R"=====('>
        </div>
      </div>
      
      <div class="calibration-input">
        <div class="form-group">
          <label for='calibration'>Sensor Calibration (cm)</label>
          <input type='number' id='calibration' name='calibration' step='0.1' value=')=====");
  html += String(calibration_mm / 10.0f, 1);
  html += FPSTR(R"=====('>
          <small>Positive values move water level up, negative down</small>
        </div>
      </div>
      
      <input type='submit' value='Save Configuration'>
    </form>
  </div>
  <a href='/' class='back-link'>&larr; Back to Dashboard</a>
  
  <script>
    function toggleMode(mode) {
      const presetInputs = document.getElementById('preset-inputs');
      const manualInputs = document.getElementById('manual-inputs');
      const presetOption = document.querySelector('.mode-option:nth-child(1)');
      const manualOption = document.querySelector('.mode-option:nth-child(2)');
      
      if (mode === 'preset') {
        presetInputs.style.display = 'block';
        manualInputs.style.display = 'none';
        presetOption.classList.add('active');
        manualOption.classList.remove('active');
        document.querySelector('input[name="preset_mode"][value="1"]').checked = true;
      } else {
        presetInputs.style.display = 'none';
        manualInputs.style.display = 'block';
        presetOption.classList.remove('active');
        manualOption.classList.add('active');
        document.querySelector('input[name="preset_mode"][value="0"]').checked = true;
      }
    }
  </script>
</body>
</html>
)=====");

  server.send(200, "text/html", html);
}


// Benchmarking Function (for web)

void runSensorBenchmark(int iterations = 50) {
  Serial.println("{\"benchmark\":\"starting\"}");
  performanceMetrics.reset();

  for (int i = 0; i < iterations; i++) {
    uint16_t test_mm;
    float test_cm;

    unsigned long startTime = micros();
    bool success = readDistanceBurst(test_mm, test_cm, 1);
    unsigned long endTime = micros();

    unsigned long readTime = endTime - startTime;
    performanceMetrics.addMeasurement(readTime / 1000, success);

    Serial.printf("{\"iteration\":%d,\"time_ms\":%lu,\"success\":%s,\"distance\":%.1f}\n",
                  i + 1, readTime / 1000, success ? "true" : "false",
                  success ? test_cm : 0.0f);

    delay(100);
  }

  Serial.printf("{\"benchmark_complete\":{\"success_rate\":%.1f,\"avg_time_ms\":%lu,\"min_time_ms\":%lu,\"max_time_ms\":%lu}}\n",
                performanceMetrics.getSuccessRate(),
                performanceMetrics.averageReadTime,
                performanceMetrics.minReadTime,
                performanceMetrics.maxReadTime);
}


void handleBenchmark() {
  if (server.hasArg("run")) {
    int iterations = server.hasArg("iterations") ? server.arg("iterations").toInt() : 20;
    iterations = constrain(iterations, 5, 30);  // Safe limit for ESP8266

    // Use String with pre-allocation
    String html = String();
    html.reserve(4000);  // Reserve memory to prevent fragmentation

    // HTML Header with styles
    html += F(R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Benchmark Results</title>
    <style>
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
            color: #333;
            line-height: 1.6;
        }
        .card {
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            padding: 25px;
            margin-top: 20px;
        }
        h2 {
            color: #2c3e50;
            margin-top: 0;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        .progress-bar {
            height: 20px;
            background-color: #e0e0e0;
            border-radius: 10px;
            overflow: hidden;
            margin: 20px 0;
        }
        .progress-fill {
            height: 100%;
            background-color: #4CAF50;
            width: 0%;
            transition: width 0.5s ease;
        }
        .summary-stats {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        .stat-card {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 8px;
        }
        .stat-card h3 {
            margin: 0 0 5px 0;
            font-size: 14px;
            color: #666;
        }
        .stat-card p {
            margin: 0;
            font-size: 18px;
            font-weight: 500;
        }
        .success-rate {
            color: #4CAF50;
            font-weight: 600;
        }
        .readings-table {
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
            font-size: 14px;
        }
        .readings-table th, .readings-table td {
            padding: 8px 12px;
            text-align: left;
            border-bottom: 1px solid #eee;
        }
        .readings-table th {
            background-color: #f8f9fa;
            font-weight: 500;
            position: sticky;
            top: 0;
        }
        .success {
            color: #4CAF50;
        }
        .failure {
            color: #F44336;
        }
        .back-link {
            display: inline-block;
            margin-top: 20px;
            color: #3498db;
            text-decoration: none;
            font-weight: 500;
        }
        .back-link:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="card">
        <h2>Benchmark Results</h2>
        
        <div class="progress-bar">
            <div class="progress-fill" id="progressFill"></div>
        </div>
        
        <div class="summary-stats">
            <div class="stat-card">
                <h3>Iterations</h3>
                <p id="iterationsCount">)=====");
    html += iterations;
    html += F(R"=====(</p>
            </div>
            <div class="stat-card">
                <h3>Successful Readings</h3>
                <p id="successCount">0</p>
            </div>
            <div class="stat-card">
                <h3>Success Rate</h3>
                <p class="success-rate" id="successRate">0%</p>
            </div>
            <div class="stat-card">
                <h3>Avg. Time</h3>
                <p id="avgTime">0 ms</p>
            </div>
        </div>
        
        <div class="table-container">
            <table class="readings-table">
                <thead>
                    <tr>
                        <th>#</th>
                        <th>Status</th>
                        <th>Distance (cm)</th>
                        <th>Time (ms)</th>
                    </tr>
                </thead>
                <tbody id="resultsTable">
    )=====");

    // Run benchmark in chunks for stability
    int successCount = 0;
    unsigned long totalTime = 0;
    unsigned long minTime = ULONG_MAX;
    unsigned long maxTime = 0;
    const int chunkSize = 5;

    for (int chunkStart = 0; chunkStart < iterations; chunkStart += chunkSize) {
      int chunkEnd = min(chunkStart + chunkSize, iterations);

      for (int i = chunkStart; i < chunkEnd; i++) {
        uint16_t test_mm;
        float test_cm;

        unsigned long startTime = micros();
        bool success = readDistanceBurst(test_mm, test_cm, 1);
        unsigned long endTime = micros();
        unsigned long readTime = endTime - startTime;

        // Add row to HTML table
        html += F("<tr><td>");
        html += i + 1;
        html += F("</td><td class='");
        html += success ? F("success'>Success") : F("failure'>Failed");
        html += F("</td><td>");
        html += success ? String(test_cm, 1) : F("N/A");
        html += F("</td><td>");
        html += String(readTime / 1000.0, 1);
        html += F("</td></tr>");

        // Update stats
        if (success) {
          successCount++;
          totalTime += readTime;
          if (readTime < minTime) minTime = readTime;
          if (readTime > maxTime) maxTime = readTime;
        }

        delay(50);

        // Prevent watchdog timeout
        if (i % 3 == 0) {
          server.handleClient();
          yield();
        }
      }

      // Maintain system stability
      ESP.wdtFeed();
      yield();
    }

    float successRate = iterations > 0 ? (float)successCount / iterations * 100.0f : 0;
    unsigned long avgTime = successCount > 0 ? totalTime / successCount : 0;

    // Complete the HTML
    html += F(R"=====(
                </tbody>
            </table>
        </div>
        
        <a href="/benchmark" class="back-link">← Run Another Benchmark</a>
        <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
    
    <script>
        // Update stats with final values
        document.addEventListener('DOMContentLoaded', function() {
            document.getElementById('successCount').textContent = ')=====");
    html += successCount;
    html += F(R"=====(';
            document.getElementById('successRate').textContent = ')=====");
    html += String(successRate, 1);
    html += F(R"=====(%';
            document.getElementById('avgTime').textContent = ')=====");
    html += String(avgTime / 1000.0, 1);
    html += F(R"=====( ms';
            
            const progressFill = document.getElementById('progressFill');
            const successRate = )=====");
    html += successRate;
    html += F(R"=====(;
            progressFill.style.width = successRate + '%';
            
            // Color coding based on success rate
            if (successRate < 50) {
                progressFill.style.backgroundColor = '#F44336';
                document.getElementById('successRate').className = 'failure';
            } else if (successRate < 80) {
                progressFill.style.backgroundColor = '#FF9800';
                document.getElementById('successRate').className = '';
            }
        });
    </script>
</body>
</html>
)=====");

    server.send(200, "text/html", html);
    html = String();  // Free memory
  } else {
    // Show benchmark form
    String html = F(R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Sensor Benchmark</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .card {
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            padding: 25px;
            margin-top: 20px;
        }
        h2 {
            color: #2c3e50;
            margin-top: 0;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        label {
            display: block;
            margin-bottom: 5px;
            font-weight: 500;
        }
        input[type="number"] {
            width: 100px;
            padding: 8px;
            border: 1px solid #ddd;
            border-radius: 4px;
        }
        input[type="submit"] {
            background-color: #3498db;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 16px;
            transition: background-color 0.3s;
        }
        input[type="submit"]:hover {
            background-color: #2980b9;
        }
        .info-box {
            background-color: #f8f9fa;
            border-left: 4px solid #3498db;
            padding: 15px;
            margin: 20px 0;
        }
        .back-link {
            display: inline-block;
            margin-top: 20px;
            color: #3498db;
            text-decoration: none;
        }
        .back-link:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="card">
        <h2>Sensor Benchmark</h2>
        <form method="get">
            <div class="form-group">
                <label for="iterations">Iterations (5-30):</label>
                <input type="number" id="iterations" name="iterations" value="20" min="5" max="30">
            </div>
            <input type="hidden" name="run" value="1">
            <input type="submit" value="Start Benchmark">
        </form>
        <div class="info-box">
            <strong>Note:</strong> Test will show all individual readings by default.
            Maximum 30 iterations recommended for stability.
        </div>
        <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
</body>
</html>
)=====");

    server.send(200, "text/html", html);
  }
}
// Add this function to handle WiFi setup page
void handleWiFiSetup() {
  if (server.hasArg("save")) {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("password");
    String newMAC = server.arg("mac");

    char ssidBuf[32] = { 0 }, passBuf[32] = { 0 };
    strlcpy(ssidBuf, newSSID.c_str(), 32);
    strlcpy(passBuf, newPass.c_str(), 32);

    // Parse MAC address
    uint8_t mac[6];
    if (newMAC.length() == 17) {
      // Format: "XX:XX:XX:XX:XX:XX"
      sscanf(newMAC.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
             &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
      EEPROM.put(ADDR_WIFI_MAC, mac);
    }

    EEPROM.put(ADDR_WIFI_SSID, ssidBuf);
    EEPROM.put(ADDR_WIFI_PASS, passBuf);

    if (EEPROM.commit()) {
      Serial.println("WiFi credentials and MAC saved to EEPROM");

      // Send reboot countdown page
      String html = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Settings Saved</title>
  <style>
    :root {
      --primary: #4361ee;
      --success: #4bb543;
    }
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background-color: #f5f5f5;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
      margin: 0;
      text-align: center;
    }
    .card {
      background: white;
      padding: 2rem;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
      max-width: 500px;
      width: 100%;
    }
    .success-icon {
      color: var(--success);
      font-size: 3rem;
      margin-bottom: 1rem;
    }
    h1 {
      color: var(--primary);
      margin-bottom: 1rem;
    }
    p {
      margin-bottom: 1.5rem;
      color: #555;
    }
    .countdown {
      font-size: 1.2rem;
      font-weight: bold;
      color: var(--primary);
    }
    .progress-bar {
      height: 4px;
      background: #e0e0e0;
      border-radius: 2px;
      margin-top: 1rem;
      overflow: hidden;
    }
    .progress {
      height: 100%;
      background: var(--primary);
      width: 100%;
      animation: progress 5s linear forwards;
    }
    @keyframes progress {
      from { width: 100%; }
      to { width: 0%; }
    }
  </style>
</head>
<body>
  <div class="card">
    <div class="success-icon">✓</div>
    <h1>Settings Saved Successfully</h1>
    <p>The device will reboot to apply changes.</p>
    <div class="countdown" id="countdown">Rebooting in 5 seconds...</div>
    <div class="progress-bar">
      <div class="progress"></div>
    </div>
  </div>
  <script>
    let seconds = 5;
    const countdownEl = document.getElementById('countdown');
    
    const timer = setInterval(() => {
      seconds--;
      countdownEl.textContent = `Rebooting in ${seconds} second${seconds !== 1 ? 's' : ''}...`;
      
      if (seconds <= 0) {
        clearInterval(timer);
      }
    }, 1000);
    
    // Redirect after 5 seconds
    setTimeout(() => {
      window.location.href = '/';
    }, 5000);
  </script>
</body>
</html>
)=====";
      server.send(200, "text/html", html);
      delay(5000);
      ESP.restart();
    } else {
      server.send(200, "text/html", "<h1>Error saving settings!</h1>");
    }
    return;
  }

  char ssidBuf[32] = { 0 }, passBuf[32] = { 0 };
  uint8_t mac[6] = { 0 };

  EEPROM.get(ADDR_WIFI_SSID, ssidBuf);
  EEPROM.get(ADDR_WIFI_PASS, passBuf);
  EEPROM.get(ADDR_WIFI_MAC, mac);

  // Format MAC address for display
  String macStr;
  if (mac[0] != 0 || mac[1] != 0 || mac[2] != 0 || mac[3] != 0 || mac[4] != 0 || mac[5] != 0) {
    char macBuf[18];
    sprintf(macBuf, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    macStr = String(macBuf);
  }

  String html = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>WiFi Setup</title>
  <style>
    :root {
      --primary: #4361ee;
      --light: #f8f9fa;
      --dark: #212529;
      --border: #dee2e6;
      --success: #4bb543;
    }
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    }
    body {
      background-color: #f5f5f5;
      color: var(--dark);
      line-height: 1.6;
      padding: 20px;
    }
    .container {
      max-width: 500px;
      margin: 30px auto;
      background: white;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
    }
    h2 {
      color: var(--primary);
      margin-bottom: 20px;
      text-align: center;
    }
    .form-group {
      margin-bottom: 20px;
    }
    label {
      display: block;
      margin-bottom: 5px;
      font-weight: 500;
    }
    input[type="text"],
    input[type="password"] {
      width: 100%;
      padding: 10px;
      border: 1px solid var(--border);
      border-radius: 4px;
      font-size: 16px;
    }
    .input-hint {
      font-size: 0.8rem;
      color: #666;
      margin-top: 5px;
    }
    input[type="submit"] {
      background-color: var(--primary);
      color: white;
      border: none;
      padding: 12px 20px;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      transition: background-color 0.3s;
      margin-top: 10px;
    }
    input[type="submit"]:hover {
      background-color: #3a56d4;
    }
    .back-link {
      display: block;
      text-align: center;
      margin-top: 20px;
      color: var(--primary);
      text-decoration: none;
    }
    .back-link:hover {
      text-decoration: underline;
    }
    .current-mac {
      background: var(--light);
      padding: 10px;
      border-radius: 4px;
      font-family: monospace;
      margin-bottom: 15px;
      text-align: center;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>WiFi Configuration</h2>
    
    <div class="current-mac">
      Current MAC: )=====";
  html += WiFi.macAddress();
  html += R"=====(
    </div>
    
    <form method="post" action="/wifi-setup">
      <input type="hidden" name="save" value="1">
      
      <div class="form-group">
        <label for="ssid">Network SSID</label>
        <input type="text" id="ssid" name="ssid" value=")=====";
  html += ssidBuf;
  html += R"=====(" placeholder="Enter WiFi network name">
      </div>
      
      <div class="form-group">
        <label for="password">Password</label>
        <input type="password" id="password" name="password" value=")=====";
  html += passBuf;
  html += R"=====(" placeholder="Enter WiFi password">
      </div>
      
      <div class="form-group">
        <label for="mac">ESP-NOW Peer MAC (Optional)</label>
        <input type="text" id="mac" name="mac" value=")=====";
  html += macStr;
  html += R"=====(" placeholder="XX:XX:XX:XX:XX:XX">
        <div class="input-hint">Format: 00:11:22:33:44:55 (leave empty to keep current)</div>
      </div>
      
      <input type="submit" value="Save Settings">
    </form>
    <a href="/" class="back-link">Back to Dashboard</a>
  </div>
</body>
</html>
)=====";

  server.send(200, "text/html", html);
}

// Optimized sensor handling
void handleSensorReading() {
  static unsigned long lastWebUpdate = 0;
  unsigned long currentTime = millis();

  // For web dashboard, use stable averaged readings
  if (currentTime - lastWebUpdate >= 1000) {  // Update every second for web
    uint16_t stable_mm;
    float stable_cm;

    if (readDistance(stable_mm, stable_cm)) {
      mm = stable_mm;
      cm = stable_cm;
      updateMeasurements();
      lastWebUpdate = currentTime;
    }
  }

  // Handle ESP-NOW requests immediately with burst reading
  handleESPNowRequest();
}

String getCustomTimestamp() {
  timeClient.update();

  // Get individual time components
  int day = timeClient.getDay();
  int hours = timeClient.getHours();
  int minutes = timeClient.getMinutes();

  // Get date components (month starts from 0)
  time_t rawtime = timeClient.getEpochTime();
  struct tm *ti;
  ti = localtime(&rawtime);

  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp),
           "%02d-%02d-%04dT%02d:%02d",
           ti->tm_mday,         // Day of month (1-31)
           ti->tm_mon + 1,      // Month (0-11) + 1
           ti->tm_year + 1900,  // Year (since 1900)
           hours,
           minutes);

  return String(timestamp);
}


void publishMQTTStatus() {

  // Serial.printf("System health - Free heap: %d bytes, WiFi: %s, MQTT: %s\n",
  //               ESP.getFreeHeap(),
  //               WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
  //               mqttClient.connected() ? "Connected" : "Disconnected");

  if (!mqttClient.connected()) {
    Serial.println("MQTT not connected, skip publish");
    return;
  }

  char payload[256];  // Adjust size as needed
  String NTPDateTime = getCustomTimestamp();

  snprintf(payload, sizeof(payload),
           "{\"timestamp\":\"%s\","
           "\"device\":\"%s\","
           "\"ip_address\":\"%s\","
           "\"distance_cm\":%.1f,"
           "\"level_percent\":%.1f,"
           "\"volume_liters\":%.1f}",
           NTPDateTime.c_str(),
           DEVICE_NAME,
           WiFi.localIP().toString().c_str(),
           cm,
           percent,
           volume);

  if (mqttClient.publish(PubTopic, 0, true, payload)) {
    Serial.println("MQTT published: " + String(payload));
  } else {
    Serial.println("MQTT publish failed");
  }
}


//=============== start of MQTT funtions ====================

void printSeparationLine() {
  Serial.println("************************************************");
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  Serial.print("PubTopic: ");
  Serial.println(PubTopic);
  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  (void)reason;

  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttMessage(char *topic, char *payload, const AsyncMqttClientMessageProperties &properties,
                   const size_t &len, const size_t &index, const size_t &total) {
  (void)payload;

  // Serial.println("Publish received.");
  // Serial.print("  topic: ");
  // Serial.println(topic);
  // Serial.print("  qos: ");
  // Serial.println(properties.qos);
  // Serial.print("  dup: ");
  // Serial.println(properties.dup);
  // Serial.print("  retain: ");
  // Serial.println(properties.retain);
  // Serial.print("  len: ");
  // Serial.println(len);
  // Serial.print("  index: ");
  // Serial.println(index);
  // Serial.print("  total: ");
  // Serial.println(total);
}

void onMqttPublish(const uint16_t &packetId) {
  // Serial.println("Publish acknowledged.");
  // Serial.print("  packetId: ");
  // Serial.println(packetId);
  blinkBlueLED(1);
}

//=============== end of MQTT funtions ====================

bool connectToWiFi() {
  char ssidBuf[32] = { 0 };
  char passBuf[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssidBuf);
  EEPROM.get(ADDR_WIFI_PASS, passBuf);

  String ssid = String(ssidBuf);
  String password = String(passBuf);

  if (ssid.length() == 0) {
    Serial.println("{\"error\":\"No saved SSID\"}");
    return false;
  }

  Serial.printf("Connecting to WiFi: %s\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    connectToMqtt();
    wifiConnected = true;
    digitalWrite(LED_GREEN, LED_ON);
    digitalWrite(LED_RED, LED_OFF);
    blinkBlueLED(2);
    return true;
  } else {
    Serial.println("\nWiFi connection failed → Starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    digitalWrite(LED_RED, LED_ON);
    return false;
  }
}


void setup() {
  // 1. Initialize basic hardware first
  initLEDs();
  Serial.begin(115200);
  delay(100);  // Allow serial to stabilize

  Serial.println();
  Serial.println("=== Water Tank Monitor with RS232 Commands ===");
  Serial.println("Available commands: mac, read, benchmark, reset");
  Serial.println("===============================================");
  Serial.println(DEVICE_NAME);
  Serial.println("===============================================");

  // 2. Initialize EEPROM and load config EARLY
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();  // This should happen before any WiFi operations

  // 3. Initialize sensor serial
  sensorSerial.begin(9600);
  sensorSerial.setTimeout(50);

  // === Check for D6 hold to force AP mode ===
  pinMode(D6, INPUT_PULLUP);
  delay(10);  // Stabilization delay
  bool forceAPMode = (digitalRead(D6) == LOW);

  if (forceAPMode) {
    Serial.println("D6 held on boot - Forcing AP mode for 10 minutes");
    digitalWrite(LED_RED, LED_ON);

    // Start in Access Point mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD);
    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("AP Mode (Forced): %s\n", apIP.toString().c_str());

    // Setup web server routes
    server.on("/", handleRoot);
    server.on("/wifi-setup", handleWiFiSetup);
    server.on("/config", handleConfig);
    server.on("/data", handleData);
    server.on("/sensor-stats", handleSensorStats);
    server.on("/benchmark", handleBenchmark);
    server.onNotFound(handleRoot);
    server.begin();

    // Start DNS server for captive portal
    dnsServer.start(DNS_PORT, "*", apIP);

    // Start mDNS responder
    if (MDNS.begin(DEVICE_NAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("mDNS responder started");
      Serial.printf("Access at: http://%s.local\n", DEVICE_NAME);
    }

    // OTA Setup for AP mode
    ArduinoOTA.setHostname(DEVICE_NAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();

    unsigned long apStartTime = millis();
    const unsigned long apDuration = 10 * 60 * 1000;

    Serial.printf("AP mode will last for %.1f minutes\n", apDuration / 60000.0f);

    while (millis() - apStartTime < apDuration) {
      ArduinoOTA.handle();
      server.handleClient();
      dnsServer.processNextRequest();
      MDNS.update();
      handleSensorReading();
      yield();

      // Blink LED every 2 seconds
      static unsigned long lastBlink = 0;
      if (millis() - lastBlink >= 2000) {
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        lastBlink = millis();
      }
      delay(10);
    }

    Serial.println("AP mode timeout reached. Restarting to normal mode...");
    delay(1000);
    ESP.restart();
  }

  // === Normal Setup Continues Below ===

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);


  // 4. Try normal Wi-Fi connection
  WiFi.mode(WIFI_STA);  // Explicitly set to station mode
  if (!connectToWiFi()) {
    Serial.println("WiFi failed, starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD);
    Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    blinkRedLED(3);
  } else {
    Serial.println("WiFi connected successfully");
    digitalWrite(LED_GREEN, LED_ON);
  }

  // 5. OTA Setup (should be after WiFi connection)
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update Start");
    digitalWrite(LED_RED, LED_ON);  // Indicate OTA in progress
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update End");
    digitalWrite(LED_RED, LED_OFF);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %d%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    digitalWrite(LED_RED, LED_OFF);
    ESP.restart();
  });
  ArduinoOTA.begin();

  // 6. mDNS Setup
  if (MDNS.begin(DEVICE_NAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ota", "tcp", 3232);
    Serial.println("mDNS responder started");
  }

  // 7. DNS Server Setup
  IPAddress localIP = WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP();
  dnsServer.start(DNS_PORT, "*", localIP);

  // 8. Web Server Setup
  server.on("/", handleRoot);
  server.on("/wifi-setup", handleWiFiSetup);
  server.on("/config", handleConfig);
  server.on("/data", handleData);
  server.on("/sensor-stats", handleSensorStats);
  server.on("/benchmark", handleBenchmark);
  server.onNotFound(handleRoot);
  server.begin();

  // 10. ESP-NOW Setup
  if (esp_now_init() != 0) {
    Serial.println("{\"error\":\"ESP-NOW init failed\"}");
    delay(1000);
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_recv_cb(OnDataRecv);

  // Load MAC from EEPROM
  uint8_t storedMAC[6] = { 0 };
  EEPROM.get(ADDR_WIFI_MAC, storedMAC);
  if (storedMAC[0] || storedMAC[1] || storedMAC[2] || storedMAC[3] || storedMAC[4] || storedMAC[5]) {
    memcpy(senderMAC, storedMAC, 6);
    Serial.printf("Using stored peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  senderMAC[0], senderMAC[1], senderMAC[2],
                  senderMAC[3], senderMAC[4], senderMAC[5]);
  }

  if (esp_now_add_peer(senderMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) != 0) {
    Serial.println("{\"error\":\"Failed to add peer\"}");
  }

  // 11. Initial sensor operations
  Serial.printf("{\"status\":\"Ready\",\"calibration_cm\":%.1f}\n", calibration_mm / 10.0f);

  // Sensor health check
  if (checkSensorHealth()) {
    Serial.println("{\"sensor\":\"healthy\"}");
    // Get initial reading
    unsigned long initStart = millis();
    while (millis() - initStart < 5000) {  // 5 second timeout
      uint16_t temp_mm;
      float temp_cm;
      if (readDistance(temp_mm, temp_cm)) {
        mm = temp_mm;
        cm = temp_cm;
        updateMeasurements();
        Serial.printf("{\"initial_reading\":\"%.1f cm\"}\n", cm);
        break;
      }
      delay(100);
    }
  } else {
    Serial.println("{\"warning\":\"sensor_not_responding\"}");
  }

  // 12. Final setup complete indication
  blinkGreenLED(2);
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  Serial.println("Setup complete");
}


void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  dnsServer.processNextRequest();
  MDNS.update();

  handleSensorReading();  // Keep this frequent for responsive readings

  // Handle WiFi reconnection less frequently
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck >= 60000) {  // Every 1min
    lastWifiCheck = millis();
    Serial.printf("System health - Free heap: %d bytes, WiFi: %s, MQTT: %s\n",
                  ESP.getFreeHeap(),
                  WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected",
                  mqttClient.connected() ? "Connected" : "Disconnected");
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting reconnect");
      WiFi.disconnect();
      delay(100);
      connectToWiFi();
    } //else mqttClient.publish(PubTopic, 0, true, "esp8266-water-monitor-mqtt");
  }

  // MQTT publishing with proper timing 10s
  static unsigned long lastMQTTPublish = 0;
  if (WiFi.status() == WL_CONNECTED && millis() - lastMQTTPublish >= 30000) {
    lastMQTTPublish = millis();
    publishMQTTStatus();
  }

  // Watchdog feeding
  ESP.wdtFeed();

  yield();
  delay(10);
}
