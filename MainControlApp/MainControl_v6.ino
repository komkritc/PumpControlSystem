/*
 * Water Pump Controller with Power Monitoring and Captive Portal
 * * Description:
 * This ESP8266-based system provides intelligent control of a water pump with:
 * - Water level monitoring via ultrasonic sensor (remote ESP-NOW node)
 * - Automatic pump control with configurable thresholds
 * - Power consumption monitoring via CSE7766 energy meter IC
 * - Safety cutoff for over-power conditions
 * - Web interface for monitoring and control with Captive Portal
 * - OTA firmware updates
 * * Features:
 * - Automatically opens control page (192.168.4.1) on any device upon connection
 * - Real-time water level visualization
 * - Power usage statistics (voltage, current, power, energy)
 * - Manual override capability
 * - Settings persistence in EEPROM
 * - Mobile-friendly web interface
 * * Hardware Requirements:
 * - ESP8266 (NodeMCU or similar)
 * - Relay module
 * - CSE7766 power monitoring IC
 * - Secondary ESP8266 with ultrasonic sensor
 * * Designed by: Komkrit Chooraung
 * Date: 10-08-2025
 * Version: 1.1 (Captive Portal Added)
 */

#include <FS.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// ===== STRUCTURES =====
struct Settings {
  float pumpOnLevel;
  float pumpOffLevel;
  float powerCutoff;
  bool autoControlEnabled;
  uint8_t checksum;
};

typedef struct {
  char cmd[32];
} struct_message;

typedef struct {
  char json[128];
} response_message;

// ===== GLOBAL VARIABLES =====
// ESP-NOW
struct_message outgoing;
response_message incoming;
uint8_t receiverMAC[] = { 0x84, 0xF3, 0xEB, 0x67, 0x34, 0xD9 };  // UPDATE THIS!
bool espnow_ready = false;
unsigned long lastSuccessfulComm = 0;  // Track last successful communication

// Shared data
String latestData = "{}";
String lastUpdateTime = "Never";

// Relay control
bool relayState = false;
const int relayPin = 12;  // GPIO12

// Automatic control settings
bool autoControlEnabled = true;
float pumpOnLevel = 30.0;
float pumpOffLevel = 80.0;
float powerCutoff = 1000.0;

// CSE7766
#define CSE_RX_PIN 3
#define CSE_HEADER1 0x55
#define CSE_HEADER2 0x5A
#define CSE_PACKET_LEN 24
#define CSE_BUFFER_SIZE 25

// Calibration defaults
#define DEFAULT_VOLTAGE_CAL 1912
#define DEFAULT_CURRENT_CAL 16140
#define DEFAULT_POWER_CAL 5364

uint8_t rx_buffer[CSE_BUFFER_SIZE];
int byte_counter = 0;

// Measurement variables
float voltage = 0.0;
float current = 0.0;
float power = 0.0;
float energy = 0.0;
float accumulated_energy = 0.0;
unsigned long cf_pulses = 0;
unsigned long last_energy_calc = 0;
float last_power = 0.0;
unsigned long prev_time = 0;

// Calibration variables
long voltage_cal = DEFAULT_VOLTAGE_CAL;
long current_cal = DEFAULT_CURRENT_CAL;
long power_cal = DEFAULT_POWER_CAL;

// Timing
unsigned long lastSendTime = 0;
const long sendInterval = 5000;

// Web server & Captive Portal
ESP8266WebServer server(80);
DNSServer dnsServer;
const IPAddress apIP(192, 168, 4, 1);
const IPAddress apGateway(192, 168, 4, 1);
const IPAddress apSubnet(255, 255, 255, 0);

// ===== EEPROM FUNCTIONS =====
bool loadSettings() {
  Settings settings;
  EEPROM.begin(sizeof(Settings));
  EEPROM.get(0, settings);

  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(Settings) - 1; i++) {
    checksum ^= ((uint8_t *)&settings)[i];
  }

  if (checksum != settings.checksum) {
    settings.pumpOnLevel = 30.0;
    settings.pumpOffLevel = 80.0;
    settings.powerCutoff = 1000.0;
    settings.autoControlEnabled = true;
    saveSettings(settings);
  }

  pumpOnLevel = settings.pumpOnLevel;
  pumpOffLevel = settings.pumpOffLevel;
  powerCutoff = settings.powerCutoff;
  autoControlEnabled = settings.autoControlEnabled;

  EEPROM.end();
  return true;
}

bool saveSettings(const Settings &settings) {
  Settings settingsWithChecksum = settings;
  settingsWithChecksum.checksum = 0;
  for (size_t i = 0; i < sizeof(Settings) - 1; i++) {
    settingsWithChecksum.checksum ^= ((uint8_t *)&settingsWithChecksum)[i];
  }

  EEPROM.begin(sizeof(Settings));
  EEPROM.put(0, settingsWithChecksum);
  bool result = EEPROM.commit();
  EEPROM.end();
  return result;
}

// ===== CSE7766 FUNCTIONS =====
void cseProcessByte(uint8_t b) {
  rx_buffer[byte_counter++] = b;

  if (byte_counter == CSE_PACKET_LEN) {
    uint8_t checksum = 0;
    for (int i = 2; i < (CSE_PACKET_LEN - 1); i++) checksum += rx_buffer[i];
    if (checksum == rx_buffer[CSE_PACKET_LEN - 1]) cseParsePacket();
    byte_counter = 0;
  }

  if (byte_counter == 1 && rx_buffer[0] != CSE_HEADER1) byte_counter = 0;
  if (byte_counter == 2 && rx_buffer[1] != CSE_HEADER2) {
    rx_buffer[0] = rx_buffer[1];
    byte_counter = (rx_buffer[0] == CSE_HEADER1) ? 1 : 0;
  }
}

void cseParsePacket() {
  uint8_t header = rx_buffer[0];
  uint8_t adj = rx_buffer[20];

  if (header != 0xAA) {
    long new_voltage_cal = ((rx_buffer[2] << 16) | (rx_buffer[3] << 8) | rx_buffer[4]) / 100;
    long new_current_cal = (rx_buffer[8] << 16) | (rx_buffer[9] << 8) | rx_buffer[10];
    long new_power_cal = ((rx_buffer[14] << 16) | (rx_buffer[15] << 8) | rx_buffer[16]) / 1000;
    if (new_voltage_cal != voltage_cal || new_current_cal != current_cal || new_power_cal != power_cal) {
      voltage_cal = new_voltage_cal;
      current_cal = new_current_cal;
      power_cal = new_power_cal;
    }
  }

  long voltage_cycle = (rx_buffer[5] << 16) | (rx_buffer[6] << 8) | rx_buffer[7];
  long current_cycle = (rx_buffer[11] << 16) | (rx_buffer[12] << 8) | rx_buffer[13];
  long power_cycle = (rx_buffer[17] << 16) | (rx_buffer[18] << 8) | rx_buffer[19];

  if ((adj & 0x40) && voltage_cycle > 0) {
    voltage = (voltage_cal * 100.0) / voltage_cycle;
    if (voltage < 50 || voltage > 300) voltage = 0;
  }

  if ((adj & 0x10) && power_cycle > 0) {
    if ((header & 0xF2) == 0xF2) {
      power = 0;
    } else {
      power = (power_cal * 1000.0) / power_cycle;
      if (power < 0 || power > 5000) power = 0;
    }
  }

  if ((adj & 0x20) && current_cycle > 0 && power > 0.1) {
    current = current_cal / (float)current_cycle;
    if (current < 0 || current > 20) current = 0;
  }

  unsigned long now = millis();
  if (now - prev_time >= 1000) {
    if (last_energy_calc > 0 && power > 0 && last_power > 0) {
      float avg_power = (power + last_power) / 2.0;
      float hours_elapsed = (now - last_energy_calc) / 3600000.0;
      accumulated_energy += (avg_power / 1000.0) * hours_elapsed;
    }
    last_power = power;
    last_energy_calc = now;
    prev_time = now;
    energy = accumulated_energy;
  }
}

void readCSE7766() {
  while (Serial.available()) {
    int rb = Serial.read();
    if (rb != -1) cseProcessByte((uint8_t)rb);
  }
}

void setupCSE7766() {
  Serial.begin(4800, SERIAL_8E1);
  last_energy_calc = millis();
}

// ===== ESP-NOW FUNCTIONS =====
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus != 0) {
    Serial.println("ESP-NOW send failed");
  }
}

void onDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incoming, incomingData, sizeof(incoming));
  lastSuccessfulComm = millis();

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, incoming.json);
  if (error) return;

  float distance = doc["distance"];
  float percent = doc["percent"];
  float volume = doc["volume"];

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "{\"distance\":%.2f,\"percent\":%.2f,\"volume\":%.2f}",
           distance, percent, volume);

  latestData = String(buffer);
  lastUpdateTime = getUptime();

  if (autoControlEnabled) {
    if (power > powerCutoff) {
      if (relayState) {
        digitalWrite(relayPin, LOW);
        relayState = false;
      }
      return;
    }

    if (percent < pumpOnLevel && !relayState) {
      digitalWrite(relayPin, HIGH);
      relayState = true;
    } else if (percent > pumpOffLevel && relayState) {
      digitalWrite(relayPin, LOW);
      relayState = false;
    }
  }
}

bool setupESPNOW() {
  for (int i = 0; i < 3; i++) { // Try 3 times
    esp_now_deinit();
    delay(100);
    
    if (esp_now_init() == 0) {
      esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
      esp_now_register_send_cb(onDataSent);
      esp_now_register_recv_cb(onDataRecv);
      
      if (esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0) == 0) {
        espnow_ready = true;
        return true;
      }
    }
    delay(500); // Wait before retry
  }
  espnow_ready = false;
  return false;
}

bool sendRequest() {
  if (!espnow_ready && !setupESPNOW()) return false;

  struct_message msg;
  strcpy(msg.cmd, "get_measure");
  int result = esp_now_send(receiverMAC, (uint8_t *)&msg, sizeof(msg));
  return result == 0;
}

// ===== UTILITY FUNCTIONS =====
String getUptime() {
  unsigned long ms = millis();
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  seconds %= 60;
  minutes %= 60;
  char buf[20];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hours, minutes, seconds);
  return String(buf);
}

void toggleRelay() {
  relayState = !relayState;
  digitalWrite(relayPin, relayState ? HIGH : LOW);
}

// ===== WEB SERVER HANDLERS =====
void handleRoot() {
  if (!LittleFS.exists("/index.html")) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  server.streamFile(file, "text/html");
  file.close();
}

void handleData() {
  server.send(200, "application/json", latestData);
}

void handlePowerData() {
  String json = "{\"voltage\":" + String(voltage, 1) + ",\"current\":" + String(current, 3) + ",\"power\":" + String(power, 1) + ",\"energy\":" + String(energy, 5) + "}";
  server.send(200, "application/json", json);
}

void handleNetworkInfo() {
  String json = "{\"ip\":\"192.168.4.1\",\"hostname\":\"waterpump.local\"}";
  server.send(200, "application/json", json);
}

void handleSend() {
  sendRequest();
  server.send(200, "text/plain", "Sent");
}

void handleToggleRelay() {
  toggleRelay();
  server.send(200, "application/json", String(relayState));
}

void handleRelayState() {
  server.send(200, "application/json", String(relayState));
}

void handleUptime() {
  server.send(200, "text/plain", getUptime());
}

void handleLastUpdate() {
  server.send(200, "text/plain", lastUpdateTime);
}

void handleResetEnergy() {
  accumulated_energy = 0;
  energy = 0;
  server.send(200, "text/plain", "OK");
}

void handleESPNOWStatus() {
  bool isConnected = (millis() - lastSuccessfulComm) < 15000 && espnow_ready;
  String status = "{\"ready\":" + String(isConnected ? "true" : "false") + 
                 ",\"lastUpdate\":" + String(millis() - lastSuccessfulComm) + "}";
  server.send(200, "application/json", status);
}

void handleReboot() {
  server.send(200, "text/plain", "Rebooting...");
  delay(1000);
  ESP.restart();
}

void handleListFiles() {
  String html = "<h1>Files in LittleFS</h1><pre>";

  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    html += dir.fileName();
    html += "\t";
    html += String(dir.fileSize());
    html += " bytes\n";
  }

  html += "</pre>";
  server.send(200, "text/html", html);
}

void handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";

  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";
  else if (path.endsWith(".png")) contentType = "image/png";
  else if (path.endsWith(".jpg")) contentType = "image/jpeg";
  else if (path.endsWith(".ico")) contentType = "image/x-icon";

  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return;
  }
  
  server.sendHeader("Location", "http://192.168.4.1", true);
  server.send(302, "text/plain", "Redirecting to portal...");
}

void handleSetAutoSettings() {
  if (server.hasArg("on") && server.hasArg("off") && server.hasArg("cutoff")) {
    float newOnLevel = server.arg("on").toFloat();
    float newOffLevel = server.arg("off").toFloat();
    float newCutoff = server.arg("cutoff").toFloat();

    if (newOnLevel >= newOffLevel) {
      server.send(400, "text/plain", "Error: On level must be less than Off level");
      return;
    }

    if (newCutoff <= 0) {
      server.send(400, "text/plain", "Error: Power cutoff must be positive");
      return;
    }

    Settings newSettings;
    newSettings.pumpOnLevel = newOnLevel;
    newSettings.pumpOffLevel = newOffLevel;
    newSettings.powerCutoff = newCutoff;
    newSettings.autoControlEnabled = autoControlEnabled;

    if (saveSettings(newSettings)) {
      pumpOnLevel = newOnLevel;
      pumpOffLevel = newOffLevel;
      powerCutoff = newCutoff;
      server.send(200, "text/plain", "Settings updated and saved");
    } else {
      server.send(500, "text/plain", "Error saving settings");
    }
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void handleToggleAutoControl() {
  autoControlEnabled = !autoControlEnabled;

  Settings currentSettings;
  currentSettings.pumpOnLevel = pumpOnLevel;
  currentSettings.pumpOffLevel = pumpOffLevel;
  currentSettings.powerCutoff = powerCutoff;
  currentSettings.autoControlEnabled = autoControlEnabled;

  saveSettings(currentSettings);

  server.send(200, "application/json", String(autoControlEnabled ? "true" : "false"));
}

void handleGetAutoSettings() {
  String json = "{\"enabled\":" + String(autoControlEnabled ? "true" : "false") + ",\"onLevel\":" + String(pumpOnLevel, 1) + ",\"offLevel\":" + String(pumpOffLevel, 1) + ",\"powerCutoff\":" + String(powerCutoff, 1) + "}";
  server.send(200, "application/json", json);
}

// ===== MAIN FUNCTIONS =====
void setup() {
  EEPROM.begin(sizeof(Settings));
  loadSettings();

  Serial.begin(115200);

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }

  server.onNotFound([]() {
    handleFileRead(server.uri());
  });

  Dir dir = LittleFS.openDir("/");
  while (dir.next()) {
    Serial.print(dir.fileName());
    Serial.print("\t");
    Serial.println(dir.fileSize());
  }

  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    Serial.println("Failed to open file");
    return;
  }

  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP("WaterPumpAP", "waterpump123");
  delay(500);

  dnsServer.start(53, "*", apIP);
  
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  setupCSE7766();
  setupESPNOW();

  ArduinoOTA.setHostname("waterpump");
  ArduinoOTA.setPassword("waterpump123");
  ArduinoOTA.begin();

  MDNS.begin("waterpump");

  server.on("/data", handleData);
  server.on("/powerData", handlePowerData);
  server.on("/networkInfo", handleNetworkInfo);
  server.on("/send", handleSend);
  server.on("/toggleRelay", handleToggleRelay);
  server.on("/relayState", handleRelayState);
  server.on("/uptime", handleUptime);
  server.on("/lastUpdate", handleLastUpdate);
  server.on("/resetEnergy", handleResetEnergy);
  server.on("/espnow_status", handleESPNOWStatus);
  server.on("/reboot", handleReboot);
  server.on("/setAutoSettings", handleSetAutoSettings);
  server.on("/toggleAutoControl", handleToggleAutoControl);
  server.on("/getAutoSettings", handleGetAutoSettings);
  server.on("/ls", HTTP_GET, handleListFiles);
  server.on("/", handleRoot);
  server.begin();

  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ota", "tcp", 3232);
}

void loop() {
  dnsServer.processNextRequest();
  ArduinoOTA.handle();
  server.handleClient();
  readCSE7766();

  if (millis() - lastSendTime >= sendInterval) {
    if (sendRequest()) {
      lastSendTime = millis();
    } else {
      Serial.println("Failed to send ESP-NOW request");
      delay(1000); // Wait before retry
    }
  }

  if (autoControlEnabled && power > powerCutoff && relayState) {
    digitalWrite(relayPin, LOW);
    relayState = false;
  }
}