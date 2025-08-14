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
 * Version: 1.0rc (add LEDs and fix mqtt re-connect wifi if lost, add time-sync)
 */

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <sys/time.h>
#include <time.h>          // For time functions
#include <ESP8266mDNS.h>   // Add this line with the other includes
#include <PubSubClient.h>  // MQTT client library

// --- Constants ---
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define TIME_ZONE_OFFSET 7  // UTC time (adjust for your timezone if needed)
#define EEPROM_SIZE 128
#define ADDR_WIDTH 0       // float (4 bytes)
#define ADDR_HEIGHT 4      // float (4 bytes)
#define ADDR_VOL_FACTOR 8  // float (4 bytes)
#define ADDR_CALIB 12      // int (4 bytes)
#define ADDR_WIFI_SSID 16  // 32 bytes
#define ADDR_WIFI_PASS 48  // 32 bytes

#define WIFI_CONNECTED_LED D8     // GPIO15 for WiFi connected status
#define WIFI_DISCONNECTED_LED D7  // GPIO13 for WiFi disconnected status

// Add these right after your other global variables (around line 150)
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
unsigned long lastMqttPublishTime = 0;
String mqttClientId;


// --- Config Macros ---
#define DEVICE_NAME "TankMonitor144"  // Device name (AP/mDNS/OTA)
#define WIFI_AP_PASSWORD "12345678"   // AP mode password
#define OTA_PASSWORD "12345678"       // OTA update password (optional)

// --- MQTT Configuration ---
#define MQTT_ENABLED true                  // Set to false to disable MQTT
#define MQTT_SERVER "broker.hivemq.com"    // Your MQTT broker address
#define MQTT_PORT 1883                     // MQTT port (typically 1883 or 8883)
#define MQTT_USER ""                       // MQTT username (leave empty if none)
#define MQTT_PASSWORD ""                   // MQTT password (leave empty if none)
#define MQTT_TOPIC DEVICE_NAME "/status"   // Base MQTT topic (now uses DEVICE_NAME)
#define MQTT_PUBLISH_INTERVAL 15000        // Publish interval in ms (15 seconds)
#define MQTT_CLIENT_ID "TankMonitor_%06X"  // Client ID with chip ID suffix

#define RX_PIN D1
#define TX_PIN D2
#define SENSOR_READ_TIMEOUT 100        // ms
#define SERIAL_INPUT_TIMEOUT 5000      // ms
#define UPDATE_INTERVAL 2000           // ms for web dashboard updates
#define SERIAL_COMMAND_BUFFER_SIZE 32  // Buffer size for serial commands

const byte DNS_PORT = 53;  // DNS server port for captive portal

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

// --- MQTT Functions --- (Revised)
void mqttReconnect() {
  if (!MQTT_ENABLED || WiFi.status() != WL_CONNECTED) return;

  static unsigned long lastAttempt = 0;
  const unsigned long retryInterval = 5000;  // 5 seconds between attempts

  if (millis() - lastAttempt >= retryInterval) {
    Serial.print("Attempting MQTT connection...");

    if (mqttClient.connect(mqttClientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");
      // Once connected, publish an announcement
      String statusTopic = String(MQTT_TOPIC) + "/status";
      mqttClient.publish(statusTopic.c_str(), "online", true);
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
    lastAttempt = millis();
  }
}

bool publishMqttData() {
  if (!MQTT_ENABLED || !mqttClient.connected()) return false;

  updateMeasurements();

  // Create JSON payload with timestamp
  String payload = "{";
  payload += "\"timestamp\":\"" + getUKTimestamp() + "\",";  // New timestamp field
  payload += "\"distance\":" + String(cm, 1) + ",";
  payload += "\"percent\":" + String(percent, 1) + ",";
  payload += "\"volume\":" + String(volume, 1) + ",";
  payload += "\"tank_width\":" + String(tankWidth, 1) + ",";
  payload += "\"tank_height\":" + String(tankHeight, 1) + ",";
  payload += "\"calibration\":" + String(calibration_mm / 10.0f, 1) + ",";
  payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  payload += "\"uptime\":" + String(millis() / 1000) + ",";
  payload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  payload += "\"mac\":\"" + WiFi.macAddress() + "\"";
  payload += "}";

  // Publish to MQTT topic
  bool published = mqttClient.publish(MQTT_TOPIC, payload.c_str(), true);

  if (published) {
    Serial.print("MQTT_TOPIC ====> ");
    Serial.println(MQTT_TOPIC);
    Serial.println(payload);
    Serial.println("MQTT data published successfully");
    return true;
  } else {
    Serial.println("MQTT publish failed");
    return false;
  }
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

String getUKTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);  // Get local UK time (auto-adjusts for BST/GMT)
  
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%d-%m-%Y %H:%M:%S", &timeinfo); // DD-MM-YYYY HH:MM:SS
  return String(buffer);
}

String getCurrentTimestamp() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
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
    performanceMetrics.addMeasurement(readTime / 1000, success);  // Convert to milliseconds

    Serial.printf("{\"iteration\":%d,\"time_ms\":%lu,\"success\":%s,\"distance\":%.1f}\n",
                  i + 1, readTime / 1000, success ? "true" : "false",
                  success ? test_cm : 0.0f);

    delay(100);  // Small delay between tests
  }

  Serial.printf("{\"benchmark_complete\":{\"success_rate\":%.1f,\"avg_time_ms\":%lu,\"min_time_ms\":%lu,\"max_time_ms\":%lu}}\n",
                performanceMetrics.getSuccessRate(),
                performanceMetrics.averageReadTime,
                performanceMetrics.minReadTime,
                performanceMetrics.maxReadTime);
}

// Benchmark endpoint handler
void handleBenchmark() {
  if (server.hasArg("run")) {
    int iterations = server.hasArg("iterations") ? server.arg("iterations").toInt() : 20;
    iterations = constrain(iterations, 5, 100);  // Limit range

    String response = "Benchmark running with " + String(iterations) + " iterations. Check serial output.";
    server.send(200, "text/plain", response);

    // Run benchmark in background
    runSensorBenchmark(iterations);
  } else {
    String html = R"=====(
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
            color: #333;
            line-height: 1.6;
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
        <h2>Sensor Benchmark Tool</h2>
        
        <div class="form-group">
            <form method="get">
                <label for="iterations">Iteration Count (5-100):</label>
                <input type="number" id="iterations" name="iterations" value="20" min="5" max="100">
                
                <input type="hidden" name="run" value="1">
                <br><br>
                <input type="submit" value="Start Benchmark">
            </form>
        </div>
        
        <div class="info-box">
            <strong>Note:</strong> The benchmark will run with the specified number of iterations.
            Results will be displayed in the serial monitor output.
            Please don't navigate away during testing.
        </div>
        
        <a href="/" class="back-link">← Back to Dashboard</a>
    </div>
</body>
</html>
)=====";

    server.send(200, "text/html", html);
  }
}

// Add this function to handle WiFi setup page
void handleWiFiSetup() {
  if (server.hasArg("save")) {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("password");

    char ssidBuf[32] = { 0 }, passBuf[32] = { 0 };
    strlcpy(ssidBuf, newSSID.c_str(), 32);
    strlcpy(passBuf, newPass.c_str(), 32);

    EEPROM.put(ADDR_WIFI_SSID, ssidBuf);
    EEPROM.put(ADDR_WIFI_PASS, passBuf);

    if (EEPROM.commit()) {
      Serial.println("WiFi credentials saved to EEPROM");

    } else {
      Serial.println("EEPROM commit failed!");
    }

    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Settings saved");
    delay(100);
    WiFi.begin(ssidBuf, passBuf);  // Reconnect
    return;
  }

  char ssidBuf[32] = { 0 }, passBuf[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssidBuf);
  EEPROM.get(ADDR_WIFI_PASS, passBuf);

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
  </style>
</head>
<body>
  <div class="container">
    <h2>WiFi Configuration</h2>
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

// --- WiFi Improvement Functions ---

void initWiFiDependentServices() {
  if (MQTT_ENABLED) {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setBufferSize(512);
  }
}

// --- WiFi Connection Management ---

void startAPMode() {
  Serial.println("Starting AP mode");
  WiFi.mode(WIFI_AP);
  bool apStarted = WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD);

  if (apStarted) {
    Serial.printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    // In AP mode, both LEDs have special behavior
    digitalWrite(WIFI_CONNECTED_LED, LOW);
    digitalWrite(WIFI_DISCONNECTED_LED, HIGH);
  } else {
    Serial.println("Failed to start AP!");
    // Error state - alternate blinking
    digitalWrite(WIFI_CONNECTED_LED, !digitalRead(WIFI_CONNECTED_LED));
    digitalWrite(WIFI_DISCONNECTED_LED, !digitalRead(WIFI_DISCONNECTED_LED));
    delay(500);
  }
}

void updateWiFiStatusLEDs() {
  static bool lastWiFiStatus = false;
  bool currentWiFiStatus = (WiFi.status() == WL_CONNECTED);

  if (currentWiFiStatus != lastWiFiStatus) {
    if (currentWiFiStatus) {
      // WiFi connected - turn on connected LED, turn off disconnected LED
      digitalWrite(WIFI_CONNECTED_LED, HIGH);
      digitalWrite(WIFI_DISCONNECTED_LED, LOW);
      Serial.println("WiFi connected - Connected LED ON, Disconnected LED OFF");
    } else {
      // WiFi disconnected - turn off connected LED, blink disconnected LED
      digitalWrite(WIFI_CONNECTED_LED, LOW);
      Serial.println("WiFi disconnected - Connected LED OFF, Disconnected LED blinking");
    }
    lastWiFiStatus = currentWiFiStatus;
  }

  // Blink disconnected LED if WiFi is not connected
  if (!currentWiFiStatus) {
    static unsigned long lastBlinkTime = 0;
    if (millis() - lastBlinkTime >= 500) {  // Blink every 500ms
      digitalWrite(WIFI_DISCONNECTED_LED, !digitalRead(WIFI_DISCONNECTED_LED));
      lastBlinkTime = millis();
    }
  }
}

bool hasValidWiFiCredentials() {
  char ssid[32] = { 0 };
  char pass[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssid);
  EEPROM.get(ADDR_WIFI_PASS, pass);
  return (strlen(ssid) > 0);
}

bool safeMqttReconnect() {
  if (!MQTT_ENABLED || WiFi.status() != WL_CONNECTED) {
    return false;
  }

  static unsigned long lastAttempt = 0;
  const unsigned long retryInterval = 5000;  // 5 seconds between attempts

  if (millis() - lastAttempt < retryInterval) {
    return false;
  }

  Serial.println("Attempting MQTT reconnection...");

  // Disconnect first if needed
  if (mqttClient.connected()) {
    mqttClient.disconnect();
    delay(100);
  }

  // Generate fresh client ID each time
  String clientId = String(MQTT_CLIENT_ID);
  clientId.replace("%06X", String(ESP.getChipId(), HEX));

  // Additional connection parameters
  bool connected = mqttClient.connect(
    clientId.c_str(),
    MQTT_USER,
    MQTT_PASSWORD,
    (String(MQTT_TOPIC) + "/status").c_str(),
    1,         // QoS 1
    true,      // Retain
    "offline"  // Will message
  );

  if (connected) {
    Serial.println("MQTT connected!");
    // Publish online status
    mqttClient.publish(
      (String(MQTT_TOPIC) + "/status").c_str(),
      "online",
      true  // Retain
    );

    // Resubscribe to topics if needed
    // mqttClient.subscribe("topic/subtopic");

    return true;
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
    return false;
  }

  lastAttempt = millis();
}

void connectToWiFi() {
  char ssid[32] = { 0 };
  char pass[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssid);
  EEPROM.get(ADDR_WIFI_PASS, pass);

  if (strlen(ssid)) {
    Serial.printf("Attempting to connect to WiFi: %s\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);

    unsigned long startTime = millis();
    bool connected = false;

    // Try to connect for 20 seconds max
    while (millis() - startTime < 20000) {
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        break;
      }
      updateWiFiStatusLEDs();  // Update LED status during connection attempt
      delay(500);
      Serial.print(".");
    }

    if (connected) {
      Serial.println("\nWiFi connected!");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      digitalWrite(WIFI_CONNECTED_LED, HIGH);
      digitalWrite(WIFI_DISCONNECTED_LED, LOW);

      // Configure NTP time synchronization
      configTime(TIME_ZONE_OFFSET * 3600, 0, NTP_SERVER1, NTP_SERVER2);
      Serial.println("Waiting for NTP time sync...");

      time_t now = time(nullptr);
      int retries = 0;
      while (now < 8 * 3600 * 2 && retries < 30) {  // Wait for valid time
        delay(500);
        now = time(nullptr);
        retries++;
        Serial.print(".");
      }

      if (now >= 8 * 3600 * 2) {
        Serial.println("\nTime synchronized:");
        Serial.println(getUKTimestamp());
      } else {
        Serial.println("\nFailed to get NTP time");
      }

      if (!mqttClient.connected()) {
        safeMqttReconnect();
      } else {
        // Handle regular MQTT operations
        mqttClient.loop();
      }
      return;
    }
  }

  // If no credentials or connection failed, start AP mode
  startAPMode();
}

// Helper function to translate WiFi status codes to strings
const char *getWiFiStatusName(int status) {
  switch (status) {
    case WL_IDLE_STATUS: return "IDLE";
    case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    default: return "UNKNOWN";
  }
}


// --- WiFi Connection Management --- (Revised)
void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  static unsigned long lastReconnectAttempt = 0;
  const unsigned long checkInterval = 5000;        // 5 seconds
  const unsigned long reconnectInterval = 300000;  // 5 minutes
  //const unsigned long reconnectInterval = 60000;  // 60 sec

  if (millis() - lastCheck >= checkInterval) {
    int status = WiFi.status();

    if (status != WL_CONNECTED) {
      Serial.printf("WiFi disconnected (status: %d - %s)\n",
                    status, getWiFiStatusName(status));

      // If we have credentials and it's time to try again
      if ((millis() - lastReconnectAttempt >= reconnectInterval)) {
        Serial.println("Attempting WiFi reconnection...");
        WiFi.disconnect();
        char ssid[32] = { 0 };
        char pass[32] = { 0 };
        EEPROM.get(ADDR_WIFI_SSID, ssid);
        EEPROM.get(ADDR_WIFI_PASS, pass);
        String storedSSID = String(ssid);
        String storedPass = String(pass);
        loadConfig();
        Serial.println("===================");
        Serial.println(storedSSID);
        Serial.println(storedPass);
        Serial.println("===================");

        connectToWiFi();

        lastReconnectAttempt = millis();
      }
      // If no wifi, ensure AP mode is running
      else if (WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
        startAPMode();
      } else {
        Serial.println("Already in AP mode, skipping startAPMode()");
      }
    }

    lastCheck = millis();
  }
}



// --- Setup Function ---
void setup() {
  pinMode(D4, OUTPUT);
  digitalWrite(D4, LOW);

  Serial.begin(115200);
  Serial.println();
  Serial.println("=== Water Tank Monitor with RS232 Commands ===");
  Serial.println("Available commands: mac, read, benchmark, reset");
  Serial.println("===============================================");
  Serial.println(DEVICE_NAME);
  Serial.println("===============================================");

  // Initialize sensor serial with optimized settings
  sensorSerial.begin(9600);
  sensorSerial.setTimeout(50);  // Reduced timeout for faster response

  EEPROM.begin(EEPROM_SIZE);
  char ssid[32] = { 0 };
  char pass[32] = { 0 };
  EEPROM.get(ADDR_WIFI_SSID, ssid);
  EEPROM.get(ADDR_WIFI_PASS, pass);
  String storedSSID = String(ssid);
  String storedPass = String(pass);
  Serial.println(storedSSID);
  Serial.println(storedPass);
  loadConfig();

  // Initialize LEDs
  pinMode(WIFI_CONNECTED_LED, OUTPUT);
  pinMode(WIFI_DISCONNECTED_LED, OUTPUT);
  digitalWrite(WIFI_CONNECTED_LED, LOW);
  digitalWrite(WIFI_DISCONNECTED_LED, LOW);

  // Connect to WiFi or start AP
  connectToWiFi();

  // Generate unique MQTT client ID
  mqttClientId = String(MQTT_CLIENT_ID);
  mqttClientId.replace("%06X", String(ESP.getChipId(), HEX));


  // if (storedSSID.length() > 0) {
  //   WiFi.mode(WIFI_AP_STA);
  //   WiFi.begin(storedSSID.c_str(), storedPass.c_str());
  //   Serial.println("Connecting to WiFi...");
  //   digitalWrite(D4, HIGH);
  //   int attempts = 0;
  //   while (WiFi.status() != WL_CONNECTED && attempts < 50) {
  //     delay(200);
  //     Serial.print(".");
  //     digitalWrite(D4, !digitalRead(D4));
  //     attempts++;
  //   }

  updateWiFiStatusLEDs();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize MQTT if enabled
    if (MQTT_ENABLED) {
      mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
      mqttClient.setBufferSize(512);  // Increase buffer size for larger messages
    }

    if (MDNS.begin(DEVICE_NAME)) {
      Serial.println("mDNS responder started");
    } else {
      Serial.println("Error setting up mDNS responder!");
    }
  }


  // Fall back to AP mode if no credentials or connection failed
  // In setup():
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi, starting AP mode");
    WiFi.mode(WIFI_AP);
    bool apStarted = WiFi.softAP(DEVICE_NAME, WIFI_AP_PASSWORD);
    if (!apStarted) {
      Serial.println("Failed to start AP!");
    } else {
      Serial.printf("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
      delay(500);
      dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());  // Start DNS only after AP
    }
  }

  // OTA Setup
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);  // Add password protection

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    ESP.restart();
  });

  ArduinoOTA.begin();
  Serial.print("OTA Update URL: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":3232");

  // Add mDNS for service discovery (at end of setup())
  if (MDNS.begin(DEVICE_NAME)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ota", "tcp", 3232);
    Serial.println("mDNS responder started");
    Serial.print("Access at: http://");
    Serial.print(DEVICE_NAME);
    Serial.println(".local");
  }

  // Web Server Setup
  server.on("/", handleRoot);
  server.on("/wifi-setup", handleWiFiSetup);
  server.on("/config", handleConfig);
  server.on("/data", handleData);
  server.on("/sensor-stats", handleSensorStats);  // New endpoint for sensor diagnostics
  server.on("/benchmark", handleBenchmark);       // New benchmark endpoint
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

  // Initial sensor health check
  if (checkSensorHealth()) {
    Serial.println("{\"sensor\":\"healthy\"}");

    // Get initial stable reading (may take a few seconds)
    uint16_t init_mm;
    float init_cm;
    unsigned long initStart = millis();

    while (millis() - initStart < 10000) {  // Try for 10 seconds
      if (readDistance(init_mm, init_cm)) {
        mm = init_mm;
        cm = init_cm;
        updateMeasurements();
        Serial.printf("{\"initial_reading\":\"%.1f cm\"}\n", cm);
        break;
      }
      delay(100);
    }
  } else {
    Serial.println("{\"warning\":\"sensor_not_responding\"}");
  }

  // Initialize serial command buffer
  serialBufferIndex = 0;
  memset(serialCommandBuffer, 0, SERIAL_COMMAND_BUFFER_SIZE);
  updateWiFiStatusLEDs();
}

// --- Optimized Main Loop ---
void loop() {
  static unsigned long lastSensorCheck = 0;
  static unsigned long lastHealthCheck = 0;
  static bool sensorHealthy = true;

  MDNS.update();  // Keep mDNS service running
  checkWiFiConnection();
  updateWiFiStatusLEDs();

  unsigned long currentTime = millis();

  handleSerialInput();
  ArduinoOTA.handle();
  server.handleClient();

  if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
    dnsServer.processNextRequest();
  }

  // Handle MQTT only if WiFi is connected
  if (MQTT_ENABLED && WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      mqttReconnect();  // Non-blocking now
    } else {
      mqttClient.loop();

      // Publish data at regular intervals (using your existing lastMqttPublishTime)
      if (millis() - lastMqttPublishTime >= MQTT_PUBLISH_INTERVAL) {
        if (publishMqttData()) {
          lastMqttPublishTime = millis();  // Update the timestamp on success
        }
      }
    }
  }


  // Handle sensor readings optimally
  handleSensorReading();

  // Periodic sensor health check (every 30 seconds)
  if (currentTime - lastHealthCheck >= 30000) {
    bool currentHealth = checkSensorHealth();

    if (currentHealth != sensorHealthy) {
      sensorHealthy = currentHealth;
      Serial.printf("{\"sensor_health_changed\":\"%s\"}\n",
                    sensorHealthy ? "healthy" : "unhealthy");

      if (!sensorHealthy) {
        // Reset sensor manager if unhealthy
        sensorManager.reset();
      }
    }

    lastHealthCheck = currentTime;
  }

  // Feed watchdog more frequently due to optimized loop
  static unsigned long lastWdtFeed = 0;
  if (currentTime - lastWdtFeed >= 50) {  // Feed every 50ms instead of 100ms
    ESP.wdtFeed();
    lastWdtFeed = currentTime;
  }

  // Small yield to prevent watchdog issues
  yield();
  delay(10);  // Small delay to prevent WDT timeout
}