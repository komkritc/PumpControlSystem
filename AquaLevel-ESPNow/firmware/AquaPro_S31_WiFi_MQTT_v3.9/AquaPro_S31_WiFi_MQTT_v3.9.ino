/**
 * ================================================================================================
 * SMART PUMP CONTROLLER - v3.9 (ASYNC MQTT + NTP TIME + IMPROVED WIFI SCAN)
 * ================================================================================================
 * @author: Smart Pump Team
 * @version: 3.9
 * @date: 2026-05-22
 * 
 * NEW IN v3.9:
 *   - IMPROVED WiFi Scanning: Single-click with auto-retry (no more double-click needed)
 *   - Scan timeout handling (10 seconds max)
 *   - Visual scan status with spinner animation
 *   - Better error handling for scan failures
 *   - Network selection with signal strength indicators
 * 
 * v3.8 FEATURES (PRESERVED):
 *   - WiFi settings save with 5-second countdown reboot
 *   - Statistics reset feature (runtime, cycles, event counters)
 *   - Individual reset (runtime/cycles only) and full reset (all counters)
 *   - Toast notifications for user actions
 *   - Confirmation dialog with "RESET" typing for full stats reset
 * 
 * v3.7 FIXES (PRESERVED):
 *   - FIXED: ESP-NOW disabled no longer shows "Weak" message
 *   - FIXED: Sensor badge properly hides when ESP-NOW disabled
 *   - FIXED: Clean UI when no sensor connected (no false warnings)
 *   - IMPROVED: Better ESP-NOW status handling with proper disabled state
 *   - ADDED: Gray "Disabled" indicator for optional ESP-NOW feature
 * 
 * v3.6 FEATURES (PRESERVED):
 *   - ASYNC MQTT Client (non-blocking, no delays)
 *   - NTP Time Synchronization (GMT+7 Bangkok timezone)
 *   - HIGH VOLTAGE PROTECTION (>240V shuts off pump instantly)
 *   - LOW VOLTAGE PROTECTION (<180V shuts off pump instantly)
 *   - AUTO→MANUAL mode cancels ALL timers including cooldowns
 *   - Full MQTT integration with auto-reconnect
 * 
 * v3.5 UI FEATURES (PRESERVED):
 *   - WiFi Scanner - Scan and select available networks (NOW SINGLE-CLICK)
 *   - Pressure Lockout Presets (5min to 3 days)
 *   - Real-time power monitoring (V, A, W, kWh)
 *   - Virtual tank gauge with water level animation
 *   - Reboot countdown (5 seconds with cancel option)
 *   - Engineering mode with all configuration tabs
 *   - Protection timers display (Overload, Dry Run, Low/High Voltage)
 *   - Mode toggle (AUTO/MANUAL) with visual feedback
 *   - Real-time uptime display
 *   - MQTT connection status indicator
 *   - NTP time display when connected
 * ================================================================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SonoffS31.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <espnow.h>
#include <DNSServer.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <time.h>

// ================================================================================================
// @section     VERSION & FEATURE FLAGS
// ================================================================================================
#define FIRMWARE_VERSION        "3.9"
#define FIRMWARE_DATE           "2026-05-22"
#define ENABLE_MDNS             true
#define ENABLE_OTA              true
#define ENABLE_CAPTIVE_PORTAL   true
#define ENABLE_WIFI_RECOVERY    true
#define ENABLE_MEMORY_MONITOR   true
#define ENABLE_ASYNC_MQTT       true
#define ENABLE_NTP              true

// ================================================================================================
// @section     NTP CONSTANTS (Bangkok GMT+7)
// ================================================================================================
#define NTP_TIMEZONE_OFFSET     25200
#define NTP_DAYLIGHT_OFFSET     0
#define NTP_SERVER1             "pool.ntp.org"
#define NTP_SERVER2             "time.google.com"
#define NTP_SERVER3             "th.pool.ntp.org"
#define NTP_UPDATE_INTERVAL     3600000

// ================================================================================================
// @section     DEBUG CONFIGURATION
// ================================================================================================
#define DEBUG_ENABLED           false

// ================================================================================================
// @section     SYSTEM TIMING CONSTANTS
// ================================================================================================
#define S31_UPDATE_INTERVAL     100
#define CONTROL_INTERVAL        100
#define WEB_SERVER_INTERVAL     10
#define OTA_INTERVAL            50
#define MDNS_INTERVAL           1000
#define PRESSURE_DEBOUNCE_MS    50
#define AUTO_RETURN_TIMEOUT_MS  600000
#define WATCHDOG_FEED_INTERVAL  1000
#define MEMORY_CHECK_INTERVAL   30000
#define SAVE_STATS_INTERVAL     60000
#define WIFI_RECOVERY_INTERVAL  300000
#define MIN_SAVE_INTERVAL_MS    2000
#define WIFI_CONNECT_TIMEOUT    15000
#define REBOOT_COUNTDOWN_SECONDS 5
#define WIFI_SCAN_TIMEOUT       10000

// ================================================================================================
// @section     MQTT CONSTANTS
// ================================================================================================
#define MQTT_RECONNECT_INTERVAL 3000
#define MQTT_TELEMETRY_INTERVAL 10000
#define MQTT_STATUS_INTERVAL    30000
#define MQTT_KEEPALIVE          30

// ================================================================================================
// @section     PRESSURE LOCKOUT CONSTANTS
// ================================================================================================
#define DEFAULT_PRESSURE_LOCKOUT_HOURS    2.0
#define MIN_PRESSURE_LOCKOUT_MINUTES      5
#define MAX_PRESSURE_LOCKOUT_HOURS        72

// ================================================================================================
// @section     BUTTON CONFIGURATION
// ================================================================================================
#define BUTTON_PIN              0
#define BUTTON_DEBOUNCE_MS      50
#define SHORT_PRESS_MAX_MS      3000
#define LONG_PRESS_MIN_MS       5000
#define LONG_PRESS_MAX_MS       10000

// ================================================================================================
// @section     PRESSURE SWITCH CONFIGURATION
// ================================================================================================
#define PRESSURE_SWITCH_PIN     4

// ================================================================================================
// @section     ESP-NOW TIMING CONSTANTS
// ================================================================================================
#define ESP_NOW_SEND_INTERVAL   15000
#define ESP_NOW_DATA_TIMEOUT    30000
#define MAX_LOG_ENTRIES         50
#define ESP_NOW_RETRY_COUNT     3
#define ESP_NOW_RECOVERY_INTERVAL 60000

// ================================================================================================
// @section     HARDWARE PIN DEFINITIONS
// ================================================================================================
#define RELAY_PIN               12
#define LED_PIN                 13

// ================================================================================================
// @section     FILESYSTEM CONFIGURATION
// ================================================================================================
#define CONFIG_FILE             "/config.json"
#define CONFIG_BACKUP_FILE      "/config_backup.json"

// ================================================================================================
// @section     VOLTAGE PROTECTION CONSTANTS
// ================================================================================================
#define DEFAULT_MIN_VOLTAGE             180.0
#define DEFAULT_MAX_VOLTAGE             240.0
#define MIN_VOLTAGE_THRESHOLD           100.0
#define MAX_VOLTAGE_THRESHOLD           300.0
#define LOW_VOLTAGE_IMMEDIATE_THRESHOLD 160.0
#define HIGH_VOLTAGE_IMMEDIATE_THRESHOLD 250.0
#define LOW_VOLTAGE_DELAY_SECONDS       2
#define HIGH_VOLTAGE_DELAY_SECONDS      1

// Forward declarations
void addFailureLogEntry(const char* message);
void addFailureLogEntry(String message);
void updatePumpStatistics(bool currentState);
void parseEspNowData(String data);
void startInchingTimer();
void cancelInchingTimer();
void checkInchingTimer();
void saveConfigToFile();
void loadConfigFromFile();
void factoryReset();
void checkMemoryAndCleanup();
void recoverWiFiConnection();
void recoverEspNow();
void emergencySave();
void connectToWiFi();
void cancelAllTimers();
void stopPumpAndCancelTimers(const char* reason);
void publishMqttEvent(const char* event, const char* details);
void publishMqttStatus();
void publishMqttTelemetry();
void cancelSoftStartDelay();
void resetPressureLockout();
uint32_t getLockoutRemainingSeconds();
uint32_t getOverloadCooldownRemaining();
uint32_t getDryRunCooldownRemaining();
uint32_t getLowVoltageCooldownRemaining();
uint32_t getHighVoltageCooldownRemaining();
void updatePowerQuality();
void initNTP();
void syncNTPTime();
String getFormattedTimestamp();
String getISO8601Timestamp();
String getLogTimestamp();
void setupWebServer();
void controlPump();
void initButton();
void handleButton();
void flashLed(uint32_t duration_ms);
void initPressureSwitch();
bool readPressureSwitch();
bool pressureAllowsPumpOn();
void updatePressureLockout(bool pressureLow);
bool isPressureLockoutActive();
void initEspNow();
void sendEspNowCommand(const char* command);
void requestSensorData();
void initMqttTopics();
void connectToMqtt();
String macToString(const uint8_t* mac);
bool stringToMac(const String& macStr, uint8_t* mac);
void rebootDevice();
String formatTimeRemaining(uint32_t seconds);
void saveStatistics();
void resetStatistics(bool resetAll);
void rebootWithCountdown();
void checkRebootCountdown();

// ================================================================================================
// @section     GLOBAL VARIABLES
// ================================================================================================
SonoffS31 s31(RELAY_PIN);
ESP8266WebServer server(80);
DNSServer dnsServer;
String deviceName = "s31-pump";

// NTP variables
bool ntpSynced = false;
uint32_t lastNTPUpdate = 0;
time_t nowEpoch = 0;
struct tm timeInfo;
char timeBuffer[64];

// Async MQTT Client
AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
Ticker mqttTelemetryTimer;
Ticker mqttStatusTimer;

// Save protection
bool saveInProgress = false;
uint32_t lastSaveTime = 0;
bool pendingSave = false;

// Global JSON document
StaticJsonDocument<2048> configDoc;

// Fixed ring buffer for logs
String failureLog[MAX_LOG_ENTRIES];
uint8_t logIndex = 0;
uint8_t logCount = 0;

// System tracking
uint32_t systemStartTime = 0;
uint32_t wifiConnectAttempts = 0;
uint32_t lastWifiRecovery = 0;
uint32_t lastEspNowRecovery = 0;
uint32_t lastMemoryCheck = 0;
uint32_t lastEmergencySave = 0;
bool wifiRecoveryInProgress = false;
bool wifiConnected = false;
uint32_t wifiConnectStartTime = 0;

// WiFi scan tracking
bool scanInProgress = false;
uint32_t scanStartTime = 0;

// Reboot countdown
bool rebootPending = false;
uint32_t rebootTime = 0;

// Timing variables
uint32_t lastS31Update = 0;
uint32_t lastControlCheck = 0;
uint32_t lastWebServer = 0;
uint32_t lastOTA = 0;
uint32_t lastMDNS = 0;
uint32_t lastStatsSave = 0;
uint32_t lastWdtFeed = 0;

// Configuration variables
bool auto_mode = true;
bool dry_run_enabled = true;
float min_power_threshold = 10.0;
uint32_t pump_protection_time = 3;
float max_power_threshold = 1000.0;
bool pump_load_protection_enabled = true;
uint32_t overload_cooldown_seconds = 60;
uint32_t dryrun_cooldown_seconds = 60;
bool inrush_enabled = true;
uint32_t inrush_tolerance_ms = 2000;
bool use_espnow = false;
uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char peer_mac_str[18] = "FF:FF:FF:FF:FF:FF";
int espnow_channel = 1;
char hostname[32] = "s31-pump";
bool button_enabled = true;
bool pressure_switch_inverted = true;
bool soft_start_enabled = true;
uint32_t soft_start_delay_ms = 2000;
bool inching_enabled = false;
uint32_t inching_duration_minutes = 1;
bool auto_return_enabled = true;
uint32_t total_runtime_seconds = 0;
float total_energy_kwh = 0;
uint32_t pump_cycles = 0;
uint32_t overload_events = 0;
uint32_t dryrun_events = 0;
uint32_t soft_start_count = 0;
uint32_t button_press_count = 0;
uint32_t low_voltage_events = 0;
uint32_t high_voltage_events = 0;

// WiFi configuration
char wifi_ssid[64] = "";
char wifi_password[64] = "";
bool wifi_sta_enabled = false;

// Pressure lockout
float pressure_lockout_hours = 2.0;

// Voltage protection
bool low_voltage_enabled = true;
float min_voltage_threshold = 180.0;
uint32_t low_voltage_cooldown_seconds = 30;
bool high_voltage_enabled = true;
float max_voltage_threshold = 240.0;
uint32_t high_voltage_cooldown_seconds = 30;

// MQTT Configuration
bool mqtt_enabled = false;
char mqtt_broker[64] = "mqtt.dashboard.com";
uint16_t mqtt_port = 1883;
char mqtt_username[32] = "";
char mqtt_password[32] = "";
char mqtt_client_id[64] = "";
bool mqtt_use_auth = false;
char mqtt_base_topic[64] = "pump";
char mqtt_status_topic[128] = "";
char mqtt_telemetry_topic[128] = "";
char mqtt_event_topic[128] = "";
char mqtt_command_topic[128] = "";
char mqtt_will_topic[128] = "";
bool mqttConnected = false;
uint32_t mqttReconnectAttempts = 0;
uint32_t mqttConnectStartTime = 0;

// State Structures
struct OverloadProtectionState {
  bool active = false;
  uint32_t cooldownUntil = 0;
  bool inrushActive = false;
  uint32_t pumpStartTime = 0;
  bool lastRelayState = false;
  bool bootInitialized = false;
  uint32_t overloadCount = 0;
} overload;

struct DryRunState {
  uint32_t lowPowerStartTime = 0;
  bool protectionTriggered = false;
  uint32_t cooldownUntil = 0;
  bool inCooldown = false;
  uint32_t events = 0;
  bool lowPowerDetected = false;
} dryRun;

struct LowVoltageState {
  bool active = false;
  uint32_t cooldownUntil = 0;
  uint32_t lowVoltageStartTime = 0;
  bool lowVoltageDetected = false;
  uint32_t events = 0;
} lowVoltage;

struct HighVoltageState {
  bool active = false;
  uint32_t cooldownUntil = 0;
  uint32_t highVoltageStartTime = 0;
  bool highVoltageDetected = false;
  uint32_t events = 0;
} highVoltage;

struct PressureSwitchState {
  bool lastReading = HIGH;
  bool currentState = HIGH;
  bool pressureLow = false;
  uint32_t lastDebounceTime = 0;
  uint32_t lastChangeTime = 0;
} pressureSwitch;

struct PressureLockoutState {
  bool active = false;
  uint32_t lockoutUntil = 0;
  uint32_t lastHighPressureTime = 0;
  bool waitingForLowPressure = false;
} pressureLockout;

struct SoftStartState {
  bool delayActive = false;
  uint32_t delayStartTime = 0;
  bool pendingPumpState = false;
  uint32_t delayCount = 0;
} softStart;

struct ButtonHandler {
  uint32_t pressStartTime = 0;
  uint32_t lastDebounceTime = 0;
  bool lastButtonState = HIGH;
  bool currentButtonState = HIGH;
  bool buttonPressed = false;
  uint32_t shortPressCount = 0;
  uint32_t longPressCount = 0;
  uint32_t lastManualModeTime = 0;
  bool manualModeActive = false;
} button;

struct InchingState {
  bool active = false;
  uint32_t startTime = 0;
  uint32_t duration = 0;
  bool pendingStop = false;
} inching;

// Sensor data
float currentWaterLevel = 0;
float currentDistance = 0;
float currentVolume = 0;
float batteryVoltage = 0;
uint32_t lastEspNowData = 0;
bool espnowDataValid = false;
bool sensorIsDead = false;
float powerFactor = 0.0;
float apparentPower = 0.0;

// ESP-NOW
typedef struct __attribute__((packed)) {
  uint32_t seq;
  uint32_t timestamp;
  char msg[64];
} EspNowPacket;

EspNowPacket outgoing;
EspNowPacket incoming;
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool espnow_initialized = false;
uint32_t lastEspNowSend = 0;
uint32_t espnow_seq = 0;

// ================================================================================================
// @section     DEBUG MACROS
// ================================================================================================
#if DEBUG_ENABLED
  #define DEBUG_LOG(msg) addFailureLogEntry("[DEBUG] " + String(msg));
  #define DEBUG_OVERLOAD(msg) addFailureLogEntry("[OVERLOAD] " + String(msg));
  #define DEBUG_DRYRUN(msg) addFailureLogEntry("[DRYRUN] " + String(msg));
  #define DEBUG_LOWVOLTAGE(msg) addFailureLogEntry("[LOWVOLTAGE] " + String(msg));
  #define DEBUG_HIGHVOLTAGE(msg) addFailureLogEntry("[HIGHVOLTAGE] " + String(msg));
  #define DEBUG_BUTTON(msg) addFailureLogEntry("[BUTTON] " + String(msg));
  #define DEBUG_PRESSURE(msg) addFailureLogEntry("[PRESSURE] " + String(msg));
  #define DEBUG_SOFTSTART(msg) addFailureLogEntry("[SOFTSTART] " + String(msg));
  #define DEBUG_MODE(msg) addFailureLogEntry("[MODE] " + String(msg));
  #define DEBUG_FS(msg) addFailureLogEntry("[FS] " + String(msg));
  #define DEBUG_ESPNOW(msg) addFailureLogEntry("[ESPNOW] " + String(msg));
  #define DEBUG_INCHING(msg) addFailureLogEntry("[INCHING] " + String(msg));
  #define DEBUG_INRUSH(msg) addFailureLogEntry("[INRUSH] " + String(msg));
  #define DEBUG_SYSTEM(msg) addFailureLogEntry("[SYSTEM] " + String(msg));
  #define DEBUG_LOCKOUT(msg) addFailureLogEntry("[LOCKOUT] " + String(msg));
  #define DEBUG_WIFI_MSG(msg) addFailureLogEntry("[WIFI] " + String(msg));
  #define DEBUG_MQTT(msg) addFailureLogEntry("[MQTT] " + String(msg));
  #define DEBUG_NTP(msg) addFailureLogEntry("[NTP] " + String(msg));
#else
  #define DEBUG_LOG(msg)
  #define DEBUG_OVERLOAD(msg)
  #define DEBUG_DRYRUN(msg)
  #define DEBUG_LOWVOLTAGE(msg)
  #define DEBUG_HIGHVOLTAGE(msg)
  #define DEBUG_BUTTON(msg)
  #define DEBUG_PRESSURE(msg)
  #define DEBUG_SOFTSTART(msg)
  #define DEBUG_MODE(msg)
  #define DEBUG_FS(msg)
  #define DEBUG_ESPNOW(msg)
  #define DEBUG_INCHING(msg)
  #define DEBUG_INRUSH(msg)
  #define DEBUG_SYSTEM(msg)
  #define DEBUG_LOCKOUT(msg)
  #define DEBUG_WIFI_MSG(msg)
  #define DEBUG_MQTT(msg)
  #define DEBUG_NTP(msg)
#endif

// ================================================================================================
// @section     NTP TIME FUNCTIONS
// ================================================================================================

void initNTP() {
  if (!ENABLE_NTP) return;
  configTime(NTP_TIMEZONE_OFFSET, NTP_DAYLIGHT_OFFSET, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  DEBUG_NTP("NTP initialized with GMT+7 (Bangkok timezone)");
}

void syncNTPTime() {
  if (!ENABLE_NTP) return;
  if (WiFi.status() != WL_CONNECTED) return;
  
  uint32_t now = millis();
  if (now - lastNTPUpdate < NTP_UPDATE_INTERVAL && ntpSynced) return;
  
  DEBUG_NTP("Syncing NTP time...");
  
  int retries = 0;
  while (!time(nullptr) && retries < 10) {
    delay(500);
    retries++;
  }
  
  if (time(nullptr)) {
    ntpSynced = true;
    lastNTPUpdate = now;
    time(&nowEpoch);
    localtime_r(&nowEpoch, &timeInfo);
    DEBUG_NTP("NTP synced! Time: " + getFormattedTimestamp());
  } else {
    DEBUG_NTP("NTP sync failed, using system uptime");
  }
}

String getFormattedTimestamp() {
  if (ntpSynced && time(nullptr)) {
    time(&nowEpoch);
    localtime_r(&nowEpoch, &timeInfo);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", &timeInfo);
    return String(timeBuffer);
  } else {
    uint32_t uptime = millis() / 1000;
    uint32_t hours = uptime / 3600;
    uint32_t minutes = (uptime % 3600) / 60;
    uint32_t seconds = uptime % 60;
    snprintf(timeBuffer, sizeof(timeBuffer), "UPTIME %02lu:%02lu:%02lu", hours, minutes, seconds);
    return String(timeBuffer);
  }
}

String getISO8601Timestamp() {
  if (ntpSynced && time(nullptr)) {
    time(&nowEpoch);
    localtime_r(&nowEpoch, &timeInfo);
    strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%dT%H:%M:%S+07:00", &timeInfo);
    return String(timeBuffer);
  } else {
    return String(millis() / 1000);
  }
}

String getLogTimestamp() {
  if (ntpSynced && time(nullptr)) {
    time(&nowEpoch);
    localtime_r(&nowEpoch, &timeInfo);
    strftime(timeBuffer, sizeof(timeBuffer), "[%H:%M:%S]", &timeInfo);
    return String(timeBuffer);
  } else {
    char buf[20];
    snprintf(buf, sizeof(buf), "[%lus]", millis() / 1000);
    return String(buf);
  }
}

// ================================================================================================
// @section     SYSTEM MONITORING
// ================================================================================================

void feedSystemWatchdog() {
  uint32_t now = millis();
  if (now - lastWdtFeed >= WATCHDOG_FEED_INTERVAL) {
    ESP.wdtFeed();
    lastWdtFeed = now;
  }
}

void checkMemoryAndCleanup() {
  uint32_t freeHeap = ESP.getFreeHeap();
  if (freeHeap < 8192) {
    addFailureLogEntry("⚠️ LOW MEMORY: " + String(freeHeap));
    if (logCount > MAX_LOG_ENTRIES / 2) {
      logCount = MAX_LOG_ENTRIES / 4;
      logIndex = logCount;
    }
  }
}

void emergencySave() {
  uint32_t now = millis();
  if (now - lastEmergencySave > 30000 && !saveInProgress) {
    lastEmergencySave = now;
    saveConfigToFile();
  }
}

// ================================================================================================
// @section     SAFETY PRIORITY - Cancel All Timers
// ================================================================================================

void cancelAllTimers() {
  if (inching.active) {
    inching.active = false;
    inching.pendingStop = false;
    DEBUG_LOG("All timers cancelled: Inching stopped");
    publishMqttEvent("INCHING_CANCELLED", "Safety override");
  }
  
  if (softStart.delayActive) {
    softStart.delayActive = false;
    DEBUG_LOG("All timers cancelled: Soft start cancelled");
    publishMqttEvent("SOFTSTART_CANCELLED", "Safety override");
  }
}

void cancelAllCooldowns() {
  overload.active = false;
  overload.cooldownUntil = 0;
  dryRun.inCooldown = false;
  dryRun.cooldownUntil = 0;
  dryRun.protectionTriggered = false;
  dryRun.lowPowerDetected = false;
  lowVoltage.active = false;
  lowVoltage.cooldownUntil = 0;
  lowVoltage.lowVoltageDetected = false;
  highVoltage.active = false;
  highVoltage.cooldownUntil = 0;
  highVoltage.highVoltageDetected = false;
  
  DEBUG_LOG("All cooldowns cancelled");
  publishMqttEvent("COOLDOWNS_CANCELLED", "All protection cooldowns reset");
}

void stopPumpAndCancelTimers(const char* reason) {
  cancelAllTimers();
  if (s31.getRelayState()) {
    s31.setRelay(false);
    char msg[80];
    snprintf(msg, sizeof(msg), "⏹️ Pump stopped - %s", reason);
    addFailureLogEntry(msg);
    publishMqttEvent("PUMP_STOP", reason);
  }
}

void switchToManualMode() {
  if (auto_mode) {
    auto_mode = false;
    cancelAllTimers();
    cancelAllCooldowns();
    if (s31.getRelayState()) {
      s31.setRelay(false);
    }
    button.lastManualModeTime = millis();
    addFailureLogEntry("🔄 Switched to MANUAL mode - All timers and cooldowns cancelled");
    publishMqttEvent("MODE_CHANGE", "Manual mode - All timers cancelled");
    saveConfigToFile();
  }
}

void switchToAutoMode() {
  if (!auto_mode) {
    auto_mode = true;
    button.lastManualModeTime = 0;
    addFailureLogEntry("🔄 Switched to AUTO mode - Pressure control enabled");
    publishMqttEvent("MODE_CHANGE", "Auto mode");
    saveConfigToFile();
  }
}

// ================================================================================================
// @section     MQTT FUNCTIONS
// ================================================================================================

void initMqttTopics() {
  String deviceId = String(hostname);
  snprintf(mqtt_status_topic, sizeof(mqtt_status_topic), "%s/%s/status", mqtt_base_topic, deviceId.c_str());
  snprintf(mqtt_telemetry_topic, sizeof(mqtt_telemetry_topic), "%s/%s/telemetry", mqtt_base_topic, deviceId.c_str());
  snprintf(mqtt_event_topic, sizeof(mqtt_event_topic), "%s/%s/event", mqtt_base_topic, deviceId.c_str());
  snprintf(mqtt_command_topic, sizeof(mqtt_command_topic), "%s/%s/command", mqtt_base_topic, deviceId.c_str());
  snprintf(mqtt_will_topic, sizeof(mqtt_will_topic), "%s/%s/lwt", mqtt_base_topic, deviceId.c_str());
}

void connectToMqtt() {
  if (!mqtt_enabled) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;
  
  DEBUG_MQTT("Connecting to MQTT broker: " + String(mqtt_broker) + ":" + String(mqtt_port));
  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
  mqttClient.setCleanSession(true);
  mqttClient.setWill(mqtt_will_topic, 1, true, "offline");
  
  String clientId = String(mqtt_client_id);
  if (clientId.length() == 0 || clientId == "auto") {
    clientId = String(hostname) + "-" + String(ESP.getChipId(), HEX);
  }
  mqttClient.setClientId(clientId.c_str());
  
  if (mqtt_use_auth && strlen(mqtt_username) > 0) {
    mqttClient.setCredentials(mqtt_username, mqtt_password);
  }
  
  mqttConnectStartTime = millis();
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  mqttConnected = true;
  mqttReconnectAttempts = 0;
  addFailureLogEntry("✅ MQTT Connected to " + String(mqtt_broker) + ":" + String(mqtt_port));
  mqttClient.subscribe(mqtt_command_topic, 1);
  mqttClient.publish(mqtt_will_topic, 1, true, "online");
  publishMqttStatus();
  publishMqttTelemetry();
  mqttTelemetryTimer.once(5, []() { mqttTelemetryTimer.attach(MQTT_TELEMETRY_INTERVAL / 1000.0, publishMqttTelemetry); });
  mqttStatusTimer.attach(MQTT_STATUS_INTERVAL / 1000.0, publishMqttStatus);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  mqttConnected = false;
  addFailureLogEntry("⚠️ MQTT disconnected");
  mqttTelemetryTimer.detach();
  mqttStatusTimer.detach();
  if (WiFi.status() == WL_CONNECTED && mqtt_enabled) {
    mqttReconnectTimer.once(MQTT_RECONNECT_INTERVAL / 1000.0, connectToMqtt);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String message;
  for (size_t i = 0; i < len; i++) message += (char)payload[i];
  DEBUG_MQTT("MQTT Command: " + String(topic) + " = " + message);
  
  if (String(topic) == String(mqtt_command_topic)) {
    if (message == "ON") {
      if (auto_mode) switchToManualMode();
      if (!pressureLockout.active && millis() >= overload.cooldownUntil && millis() >= dryRun.cooldownUntil && 
          millis() >= lowVoltage.cooldownUntil && millis() >= highVoltage.cooldownUntil) {
        cancelInchingTimer();
        cancelSoftStartDelay();
        s31.setRelay(true);
        addFailureLogEntry("MQTT: Pump turned ON");
        publishMqttEvent("PUMP_ON", "MQTT command");
      }
    }
    else if (message == "OFF") {
      s31.setRelay(false);
      cancelInchingTimer();
      cancelSoftStartDelay();
      addFailureLogEntry("MQTT: Pump turned OFF");
      publishMqttEvent("PUMP_OFF", "MQTT command");
    }
    else if (message == "AUTO") switchToAutoMode();
    else if (message == "MANUAL") switchToManualMode();
    else if (message == "RESET_LOCKOUT") resetPressureLockout();
    else if (message == "GET_STATUS") { publishMqttStatus(); publishMqttTelemetry(); }
  }
}

void publishMqttStatus() {
  if (!mqtt_enabled || !mqttClient.connected()) return;
  StaticJsonDocument<768> doc;
  doc["state"] = s31.getRelayState() ? "ON" : "OFF";
  doc["mode"] = auto_mode ? "AUTO" : "MANUAL";
  doc["pressure_low"] = pressureSwitch.pressureLow;
  doc["lockout_active"] = pressureLockout.active;
  if (pressureLockout.active) doc["lockout_remaining_s"] = getLockoutRemainingSeconds();
  doc["overload_active"] = overload.active;
  if (overload.active) doc["overload_remaining_s"] = getOverloadCooldownRemaining();
  doc["dryrun_active"] = (dry_run_enabled && dryRun.lowPowerStartTime != 0);
  if (dryRun.cooldownUntil > millis()) doc["dryrun_remaining_s"] = getDryRunCooldownRemaining();
  doc["low_voltage_active"] = (low_voltage_enabled && lowVoltage.lowVoltageDetected);
  if (lowVoltage.cooldownUntil > millis()) doc["low_voltage_remaining_s"] = getLowVoltageCooldownRemaining();
  doc["high_voltage_active"] = (high_voltage_enabled && highVoltage.highVoltageDetected);
  if (highVoltage.cooldownUntil > millis()) doc["high_voltage_remaining_s"] = getHighVoltageCooldownRemaining();
  doc["inching_active"] = inching.active;
  doc["softstart_active"] = softStart.delayActive;
  doc["uptime_s"] = (millis() - systemStartTime) / 1000;
  doc["version"] = FIRMWARE_VERSION;
  doc["ntp_synced"] = ntpSynced;
  doc["timestamp"] = getISO8601Timestamp();
  
  String output;
  serializeJson(doc, output);
  mqttClient.publish(mqtt_status_topic, 0, true, output.c_str());
}

void publishMqttTelemetry() {
  if (!mqtt_enabled || !mqttClient.connected()) return;
  updatePowerQuality();
  StaticJsonDocument<1024> doc;
  doc["timestamp"] = getISO8601Timestamp();
  
  JsonObject electrical = doc.createNestedObject("electrical");
  electrical["voltage"] = s31.getVoltage();
  electrical["current"] = s31.getCurrent();
  electrical["power_w"] = s31.getPower();
  electrical["energy_kwh"] = s31.getEnergy();
  electrical["power_factor"] = powerFactor;
  
  if (use_espnow && espnow_initialized && espnowDataValid && !sensorIsDead) {
    JsonObject tank = doc.createNestedObject("tank");
    tank["water_level_percent"] = currentWaterLevel;
    tank["volume_liters"] = currentVolume;
    tank["battery_voltage"] = batteryVoltage;
  }
  
  JsonObject stats = doc.createNestedObject("statistics");
  stats["runtime_min"] = total_runtime_seconds / 60;
  stats["pump_cycles"] = pump_cycles;
  stats["overload_events"] = overload_events;
  stats["dryrun_events"] = dryrun_events;
  stats["low_voltage_events"] = low_voltage_events;
  stats["high_voltage_events"] = high_voltage_events;
  
  String output;
  serializeJson(doc, output);
  mqttClient.publish(mqtt_telemetry_topic, 0, false, output.c_str());
}

void publishMqttEvent(const char* event, const char* details) {
  if (!mqtt_enabled || !mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["event"] = event;
  doc["details"] = details;
  doc["timestamp"] = getISO8601Timestamp();
  doc["uptime_s"] = (millis() - systemStartTime) / 1000;
  String output;
  serializeJson(doc, output);
  mqttClient.publish(mqtt_event_topic, 1, false, output.c_str());
}

void publishMqttEvent(String event, String details) {
  publishMqttEvent(event.c_str(), details.c_str());
}

// ================================================================================================
// @section     STATISTICS RESET FUNCTIONS
// ================================================================================================

void resetStatistics(bool resetAll) {
  if (resetAll) {
    total_runtime_seconds = 0;
    total_energy_kwh = 0;
    pump_cycles = 0;
    overload_events = 0;
    dryrun_events = 0;
    low_voltage_events = 0;
    high_voltage_events = 0;
    
    overload.overloadCount = 0;
    dryRun.events = 0;
    lowVoltage.events = 0;
    highVoltage.events = 0;
    
    addFailureLogEntry("📊 ALL statistics reset - All counters cleared");
    publishMqttEvent("STATISTICS_RESET_ALL", "All counters reset to zero");
  } else {
    total_runtime_seconds = 0;
    total_energy_kwh = 0;
    pump_cycles = 0;
    
    addFailureLogEntry("📊 Runtime and cycles reset - Protection counters preserved");
    publishMqttEvent("STATISTICS_RESET_RUNTIME", "Runtime and cycles reset");
  }
  
  saveConfigToFile();
}

// ================================================================================================
// @section     REBOOT WITH COUNTDOWN
// ================================================================================================

void rebootWithCountdown() {
  rebootPending = true;
  rebootTime = millis() + (REBOOT_COUNTDOWN_SECONDS * 1000);
  addFailureLogEntry("🔄 Reboot scheduled in " + String(REBOOT_COUNTDOWN_SECONDS) + " seconds");
  publishMqttEvent("REBOOT_SCHEDULED", String(REBOOT_COUNTDOWN_SECONDS) + "s countdown");
}

void checkRebootCountdown() {
  if (rebootPending && millis() >= rebootTime) {
    rebootPending = false;
    addFailureLogEntry("🔄 Executing reboot...");
    delay(100);
    ESP.restart();
  }
}

// ================================================================================================
// @section     IMPROVED WIFI SCAN (v3.9)
// ================================================================================================

void handleWiFiScan() {
  int n = WiFi.scanComplete();
  
  // No scan in progress - start one
  if (n == -2 && !scanInProgress) {
    WiFi.scanNetworks(true);
    scanInProgress = true;
    scanStartTime = millis();
    DEBUG_WIFI_MSG("WiFi scan started");
    server.send(200, "application/json", "{\"scanning\":true}");
    return;
  }
  
  // Scan in progress
  if (scanInProgress && n == -2) {
    // Check timeout
    if (millis() - scanStartTime > WIFI_SCAN_TIMEOUT) {
      WiFi.scanDelete();
      scanInProgress = false;
      server.send(200, "application/json", "{\"error\":\"timeout\"}");
      DEBUG_WIFI_MSG("WiFi scan timeout");
    } else {
      server.send(200, "application/json", "{\"scanning\":true}");
    }
    return;
  }
  
  // Scan complete
  if (n >= 0) {
    String json = "[";
    for (int i = 0; i < n; ++i) {
      if (i) json += ",";
      String ssid = WiFi.SSID(i);
      // Escape special characters in SSID
      ssid.replace("\\", "\\\\");
      ssid.replace("\"", "\\\"");
      json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + ",\"encryption\":" + String(WiFi.encryptionType(i)) + "}";
    }
    json += "]";
    WiFi.scanDelete();
    scanInProgress = false;
    server.send(200, "application/json", json);
    DEBUG_WIFI_MSG("WiFi scan complete: " + String(n) + " networks found");
    return;
  }
  
  // Error case
  server.send(200, "application/json", "{\"error\":\"scan_failed\"}");
  scanInProgress = false;
}

// ================================================================================================
// @section     WIFI FUNCTIONS
// ================================================================================================

void connectToWiFi() {
  if (!wifi_sta_enabled || strlen(wifi_ssid) == 0) return;
  DEBUG_WIFI_MSG("Connecting to WiFi: " + String(wifi_ssid));
  addFailureLogEntry("📡 Connecting to WiFi: " + String(wifi_ssid));
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(wifi_ssid, wifi_password);
  wifiConnectStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiConnectStartTime) < WIFI_CONNECT_TIMEOUT) {
    delay(500);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    char msg[96];
    snprintf(msg, sizeof(msg), "✅ WiFi connected! IP: %s", WiFi.localIP().toString().c_str());
    addFailureLogEntry(msg);
    if (ENABLE_NTP) syncNTPTime();
    if (mqtt_enabled && ENABLE_ASYNC_MQTT) connectToMqtt();
  } else {
    wifiConnected = false;
    addFailureLogEntry("⚠️ WiFi connection failed! Staying in AP mode");
  }
}

void recoverWiFiConnection() {
  if (!ENABLE_WIFI_RECOVERY) return;
  if (wifiRecoveryInProgress) return;
  if (!wifi_sta_enabled) return;
  
  uint32_t now = millis();
  if (now - lastWifiRecovery < WIFI_RECOVERY_INTERVAL) return;
  
  if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    wifiRecoveryInProgress = true;
    wifiConnectAttempts++;
    addFailureLogEntry("⚠️ WiFi lost, attempting recovery #" + String(wifiConnectAttempts));
    WiFi.disconnect();
    delay(100);
    WiFi.begin(wifi_ssid, wifi_password);
    lastWifiRecovery = now;
    wifiRecoveryInProgress = false;
  } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    addFailureLogEntry("✅ WiFi reconnected!");
    if (ENABLE_NTP) syncNTPTime();
    if (mqtt_enabled && ENABLE_ASYNC_MQTT && !mqttClient.connected()) connectToMqtt();
  }
}

// ================================================================================================
// @section     FILESYSTEM & JSON
// ================================================================================================

void saveConfigToFile() {
  if (saveInProgress) { pendingSave = true; return; }
  uint32_t now = millis();
  if (now - lastSaveTime < MIN_SAVE_INTERVAL_MS) { pendingSave = true; return; }
  
  saveInProgress = true;
  lastSaveTime = now;
  yield();
  ESP.wdtFeed();
  configDoc.clear();
  
  configDoc["version"] = FIRMWARE_VERSION;
  configDoc["auto_mode"] = auto_mode;
  configDoc["dry_run_enabled"] = dry_run_enabled;
  configDoc["min_power_threshold"] = min_power_threshold;
  configDoc["pump_protection_time"] = pump_protection_time;
  configDoc["max_power_threshold"] = max_power_threshold;
  configDoc["pump_load_protection_enabled"] = pump_load_protection_enabled;
  configDoc["overload_cooldown_seconds"] = overload_cooldown_seconds;
  configDoc["dryrun_cooldown_seconds"] = dryrun_cooldown_seconds;
  configDoc["inrush_enabled"] = inrush_enabled;
  configDoc["inrush_tolerance_ms"] = inrush_tolerance_ms;
  configDoc["use_espnow"] = use_espnow;
  configDoc["espnow_channel"] = espnow_channel;
  configDoc["button_enabled"] = button_enabled;
  configDoc["pressure_switch_inverted"] = pressure_switch_inverted;
  configDoc["soft_start_enabled"] = soft_start_enabled;
  configDoc["soft_start_delay_ms"] = soft_start_delay_ms;
  configDoc["inching_enabled"] = inching_enabled;
  configDoc["inching_duration_minutes"] = inching_duration_minutes;
  configDoc["auto_return_enabled"] = auto_return_enabled;
  configDoc["total_runtime_seconds"] = total_runtime_seconds;
  configDoc["total_energy_kwh"] = total_energy_kwh;
  configDoc["pump_cycles"] = pump_cycles;
  configDoc["overload_events"] = overload_events;
  configDoc["dryrun_events"] = dryrun_events;
  configDoc["soft_start_count"] = soft_start_count;
  configDoc["button_press_count"] = button_press_count;
  configDoc["hostname"] = String(hostname);
  configDoc["peer_mac"] = String(peer_mac_str);
  configDoc["pressure_lockout_hours"] = pressure_lockout_hours;
  configDoc["wifi_ssid"] = String(wifi_ssid);
  configDoc["wifi_password"] = String(wifi_password);
  configDoc["wifi_sta_enabled"] = wifi_sta_enabled;
  configDoc["low_voltage_enabled"] = low_voltage_enabled;
  configDoc["min_voltage_threshold"] = min_voltage_threshold;
  configDoc["low_voltage_cooldown_seconds"] = low_voltage_cooldown_seconds;
  configDoc["high_voltage_enabled"] = high_voltage_enabled;
  configDoc["max_voltage_threshold"] = max_voltage_threshold;
  configDoc["high_voltage_cooldown_seconds"] = high_voltage_cooldown_seconds;
  configDoc["low_voltage_events"] = low_voltage_events;
  configDoc["high_voltage_events"] = high_voltage_events;
  configDoc["mqtt_enabled"] = mqtt_enabled;
  configDoc["mqtt_broker"] = String(mqtt_broker);
  configDoc["mqtt_port"] = mqtt_port;
  configDoc["mqtt_username"] = String(mqtt_username);
  configDoc["mqtt_password"] = String(mqtt_password);
  configDoc["mqtt_client_id"] = String(mqtt_client_id);
  configDoc["mqtt_use_auth"] = mqtt_use_auth;
  configDoc["mqtt_base_topic"] = String(mqtt_base_topic);
  
  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) { saveInProgress = false; return; }
  yield();
  ESP.wdtFeed();
  serializeJson(configDoc, file);
  file.flush();
  file.close();
  saveInProgress = false;
  pendingSave = false;
}

void loadConfigFromFile() {
  if (!LittleFS.exists(CONFIG_FILE)) { saveConfigToFile(); return; }
  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) return;
  configDoc.clear();
  DeserializationError error = deserializeJson(configDoc, file);
  file.close();
  if (error) return;
  
  auto_mode = configDoc["auto_mode"] | true;
  dry_run_enabled = configDoc["dry_run_enabled"] | true;
  min_power_threshold = configDoc["min_power_threshold"] | 10.0;
  pump_protection_time = configDoc["pump_protection_time"] | 3;
  max_power_threshold = configDoc["max_power_threshold"] | 1000.0;
  pump_load_protection_enabled = configDoc["pump_load_protection_enabled"] | true;
  overload_cooldown_seconds = configDoc["overload_cooldown_seconds"] | 60;
  dryrun_cooldown_seconds = configDoc["dryrun_cooldown_seconds"] | 60;
  inrush_enabled = configDoc["inrush_enabled"] | true;
  inrush_tolerance_ms = configDoc["inrush_tolerance_ms"] | 2000;
  use_espnow = configDoc["use_espnow"] | false;
  espnow_channel = configDoc["espnow_channel"] | 1;
  button_enabled = configDoc["button_enabled"] | true;
  pressure_switch_inverted = configDoc["pressure_switch_inverted"] | true;
  soft_start_enabled = configDoc["soft_start_enabled"] | true;
  soft_start_delay_ms = configDoc["soft_start_delay_ms"] | 2000;
  inching_enabled = configDoc["inching_enabled"] | false;
  inching_duration_minutes = configDoc["inching_duration_minutes"] | 1;
  auto_return_enabled = configDoc["auto_return_enabled"] | true;
  total_runtime_seconds = configDoc["total_runtime_seconds"] | 0;
  total_energy_kwh = configDoc["total_energy_kwh"] | 0.0;
  pump_cycles = configDoc["pump_cycles"] | 0;
  overload_events = configDoc["overload_events"] | 0;
  dryrun_events = configDoc["dryrun_events"] | 0;
  soft_start_count = configDoc["soft_start_count"] | 0;
  button_press_count = configDoc["button_press_count"] | 0;
  pressure_lockout_hours = configDoc["pressure_lockout_hours"] | 2.0;
  
  low_voltage_enabled = configDoc["low_voltage_enabled"] | true;
  min_voltage_threshold = configDoc["min_voltage_threshold"] | 180.0;
  low_voltage_cooldown_seconds = configDoc["low_voltage_cooldown_seconds"] | 30;
  high_voltage_enabled = configDoc["high_voltage_enabled"] | true;
  max_voltage_threshold = configDoc["max_voltage_threshold"] | 240.0;
  high_voltage_cooldown_seconds = configDoc["high_voltage_cooldown_seconds"] | 30;
  low_voltage_events = configDoc["low_voltage_events"] | 0;
  high_voltage_events = configDoc["high_voltage_events"] | 0;
  
  String ssidTmp = configDoc["wifi_ssid"] | "";
  String pwdTmp = configDoc["wifi_password"] | "";
  ssidTmp.toCharArray(wifi_ssid, sizeof(wifi_ssid));
  pwdTmp.toCharArray(wifi_password, sizeof(wifi_password));
  wifi_sta_enabled = configDoc["wifi_sta_enabled"] | false;
  
  String hostnameStr = configDoc["hostname"] | "s31-pump";
  hostnameStr.toCharArray(hostname, sizeof(hostname));
  
  String peerMacStr = configDoc["peer_mac"] | "FF:FF:FF:FF:FF:FF";
  peerMacStr.toCharArray(peer_mac_str, sizeof(peer_mac_str));
  stringToMac(peerMacStr, peer_mac);
  
  mqtt_enabled = configDoc["mqtt_enabled"] | false;
  String brokerTmp = configDoc["mqtt_broker"] | "mqtt.dashboard.com";
  brokerTmp.toCharArray(mqtt_broker, sizeof(mqtt_broker));
  mqtt_port = configDoc["mqtt_port"] | 1883;
  String userTmp = configDoc["mqtt_username"] | "";
  userTmp.toCharArray(mqtt_username, sizeof(mqtt_username));
  String pwdMqttTmp = configDoc["mqtt_password"] | "";
  pwdMqttTmp.toCharArray(mqtt_password, sizeof(mqtt_password));
  String clientIdTmp = configDoc["mqtt_client_id"] | "";
  clientIdTmp.toCharArray(mqtt_client_id, sizeof(mqtt_client_id));
  mqtt_use_auth = configDoc["mqtt_use_auth"] | false;
  String baseTopicTmp = configDoc["mqtt_base_topic"] | "pump";
  baseTopicTmp.toCharArray(mqtt_base_topic, sizeof(mqtt_base_topic));
  
  min_power_threshold = constrain(min_power_threshold, 0.0, 3500.0);
  pump_protection_time = constrain(pump_protection_time, 3, 300);
  max_power_threshold = constrain(max_power_threshold, 10.0, 3500.0);
  overload_cooldown_seconds = constrain(overload_cooldown_seconds, 0, 300);
  dryrun_cooldown_seconds = constrain(dryrun_cooldown_seconds, 0, 300);
  inrush_tolerance_ms = constrain(inrush_tolerance_ms, 500, 5000);
  soft_start_delay_ms = constrain(soft_start_delay_ms, 0, 10000);
  espnow_channel = constrain(espnow_channel, 1, 13);
  inching_duration_minutes = constrain(inching_duration_minutes, 1, 60);
  min_voltage_threshold = constrain(min_voltage_threshold, MIN_VOLTAGE_THRESHOLD, DEFAULT_MAX_VOLTAGE);
  max_voltage_threshold = constrain(max_voltage_threshold, DEFAULT_MIN_VOLTAGE, MAX_VOLTAGE_THRESHOLD);
  low_voltage_cooldown_seconds = constrain(low_voltage_cooldown_seconds, 0, 300);
  high_voltage_cooldown_seconds = constrain(high_voltage_cooldown_seconds, 0, 300);
  
  overload.overloadCount = overload_events;
  dryRun.events = dryrun_events;
  lowVoltage.events = low_voltage_events;
  highVoltage.events = high_voltage_events;
  softStart.delayCount = soft_start_count;
  button.shortPressCount = button_press_count;
  
  initMqttTopics();
}

void saveStatistics() {
  if (saveInProgress) { pendingSave = true; return; }
  overload_events = overload.overloadCount;
  dryrun_events = dryRun.events;
  low_voltage_events = lowVoltage.events;
  high_voltage_events = highVoltage.events;
  soft_start_count = softStart.delayCount;
  button_press_count = button.shortPressCount + button.longPressCount;
  saveConfigToFile();
}

void factoryReset() {
  if (LittleFS.exists(CONFIG_FILE)) LittleFS.remove(CONFIG_FILE);
  if (LittleFS.exists(CONFIG_BACKUP_FILE)) LittleFS.remove(CONFIG_BACKUP_FILE);
  delay(100);
  ESP.restart();
}

// ================================================================================================
// @section     UTILITY FUNCTIONS
// ================================================================================================

void addFailureLogEntry(const char* message) {
  String timestamp = getLogTimestamp();
  String entry = timestamp + " " + String(message);
  failureLog[logIndex] = entry;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) logCount++;
  if (DEBUG_ENABLED) Serial.println(entry);
}

void addFailureLogEntry(String message) { addFailureLogEntry(message.c_str()); }

String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

bool stringToMac(const String& macStr, uint8_t* mac) {
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) { if (values[i] < 0 || values[i] > 255) return false; mac[i] = (uint8_t)values[i]; }
    return true;
  }
  return false;
}

void rebootDevice() { 
  saveStatistics();
  if (saveInProgress) {
    uint32_t waitStart = millis();
    while (saveInProgress && (millis() - waitStart < 3000)) { yield(); delay(10); }
  }
  delay(500);
  ESP.restart(); 
}

String formatTimeRemaining(uint32_t seconds) {
  if (seconds >= 86400) {
    uint32_t days = seconds / 86400, hours = (seconds % 86400) / 3600;
    char buf[32]; snprintf(buf, sizeof(buf), "%lud %luh", days, hours); return String(buf);
  } else if (seconds >= 3600) {
    uint32_t hours = seconds / 3600, minutes = (seconds % 3600) / 60;
    char buf[32]; snprintf(buf, sizeof(buf), "%luh %lum", hours, minutes); return String(buf);
  } else if (seconds >= 60) {
    uint32_t minutes = seconds / 60, secs = seconds % 60;
    char buf[32]; snprintf(buf, sizeof(buf), "%lum %lus", minutes, secs); return String(buf);
  } else {
    char buf[32]; snprintf(buf, sizeof(buf), "%lus", seconds); return String(buf);
  }
}

// ================================================================================================
// @section     PROTECTION FUNCTIONS
// ================================================================================================

void startInchingTimer() {
  if (inching_enabled && inching_duration_minutes > 0) {
    inching.active = true;
    inching.startTime = millis();
    inching.duration = inching_duration_minutes * 60 * 1000;
    inching.pendingStop = false;
    char msg[64]; snprintf(msg, sizeof(msg), "⏱️ INCHING: %lu min", inching_duration_minutes);
    addFailureLogEntry(msg);
    publishMqttEvent("INCHING_START", String(inching_duration_minutes) + " min");
  }
}

void cancelInchingTimer() {
  if (inching.active) {
    inching.active = false;
    addFailureLogEntry("⏱️ INCHING cancelled");
    publishMqttEvent("INCHING_STOP", "Cancelled");
  }
}

void checkInchingTimer() {
  if (!inching.active) return;
  if (millis() - inching.startTime >= inching.duration) {
    if (!inching.pendingStop) {
      inching.pendingStop = true;
      addFailureLogEntry("⏱️ INCHING: Auto-stopping");
      publishMqttEvent("INCHING_COMPLETE", "Auto-stopped");
      s31.setRelay(false);
      inching.active = false;
    }
  }
}

void initPressureSwitch() {
  pinMode(PRESSURE_SWITCH_PIN, INPUT_PULLUP);
  pressureSwitch.lastReading = digitalRead(PRESSURE_SWITCH_PIN);
  pressureSwitch.currentState = pressureSwitch.lastReading;
  pressureSwitch.pressureLow = !pressureSwitch.lastReading;
}

bool readPressureSwitch() {
  uint32_t now = millis();
  bool reading = digitalRead(PRESSURE_SWITCH_PIN);
  if (pressure_switch_inverted) reading = !reading;
  if (reading != pressureSwitch.lastReading) pressureSwitch.lastDebounceTime = now;
  if ((now - pressureSwitch.lastDebounceTime) > PRESSURE_DEBOUNCE_MS) {
    if (reading != pressureSwitch.currentState) {
      pressureSwitch.currentState = reading;
      pressureSwitch.pressureLow = !reading;
      publishMqttEvent(!pressureSwitch.pressureLow ? "PRESSURE_HIGH" : "PRESSURE_LOW", "");
    }
  }
  pressureSwitch.lastReading = reading;
  return pressureSwitch.currentState;
}

bool pressureAllowsPumpOn() { readPressureSwitch(); return pressureSwitch.pressureLow; }

void resetPressureLockout() {
  pressureLockout.active = false;
  pressureLockout.lockoutUntil = 0;
  pressureLockout.lastHighPressureTime = 0;
  addFailureLogEntry("🔓 Lockout reset");
  publishMqttEvent("LOCKOUT_RESET", "Manual reset");
}

bool isPressureLockoutActive() {
  if (pressureLockout.active && millis() >= pressureLockout.lockoutUntil) {
    pressureLockout.active = false;
    addFailureLogEntry("🔓 Lockout expired");
    publishMqttEvent("LOCKOUT_EXPIRED", "Timer completed");
    return false;
  }
  return pressureLockout.active;
}

void updatePressureLockout(bool pressureLow) {
  if (pressureLockout.active) return;
  if (!pressureLow) {
    if (pressureLockout.lastHighPressureTime == 0) {
      pressureLockout.lastHighPressureTime = millis();
      uint32_t lockoutMs = (uint32_t)(pressure_lockout_hours * 3600000.0);
      if (pressure_lockout_hours > 0 && lockoutMs < (MIN_PRESSURE_LOCKOUT_MINUTES * 60000)) lockoutMs = MIN_PRESSURE_LOCKOUT_MINUTES * 60000;
      pressureLockout.lockoutUntil = millis() + lockoutMs;
      pressureLockout.active = true;
      char msg[64];
      if (pressure_lockout_hours >= 24) snprintf(msg, sizeof(msg), "🔒 Lockout: %.1f days", pressure_lockout_hours / 24.0);
      else if (pressure_lockout_hours >= 1) snprintf(msg, sizeof(msg), "🔒 Lockout: %.1f h", pressure_lockout_hours);
      else snprintf(msg, sizeof(msg), "🔒 Lockout: %d min", (int)(pressure_lockout_hours * 60));
      addFailureLogEntry(msg);
      publishMqttEvent("LOCKOUT_ACTIVE", msg);
      cancelAllTimers();
    }
  } else {
    pressureLockout.lastHighPressureTime = 0;
  }
}

uint32_t getLockoutRemainingSeconds() {
  if (!pressureLockout.active) return 0;
  if (millis() >= pressureLockout.lockoutUntil) return 0;
  return (pressureLockout.lockoutUntil - millis()) / 1000;
}

void startSoftStartDelay(bool desiredState) {
  if (!softStart.delayActive) {
    softStart.delayActive = true;
    softStart.delayStartTime = millis();
    softStart.pendingPumpState = desiredState;
    softStart.delayCount++;
    soft_start_count = softStart.delayCount;
    char msg[48]; snprintf(msg, sizeof(msg), "SOFT START: %lu ms", soft_start_delay_ms);
    addFailureLogEntry(msg);
    publishMqttEvent("SOFTSTART_START", String(soft_start_delay_ms) + "ms");
  }
}

bool updateSoftStartState(bool desiredState) {
  if (!softStart.delayActive) {
    if (soft_start_enabled && desiredState && !s31.getRelayState()) {
      startSoftStartDelay(desiredState);
      return false;
    }
    return desiredState;
  }
  if (millis() - softStart.delayStartTime >= soft_start_delay_ms) {
    softStart.delayActive = false;
    publishMqttEvent("SOFTSTART_COMPLETE", "Delay finished");
    return softStart.pendingPumpState;
  }
  return false;
}

void cancelSoftStartDelay() {
  if (softStart.delayActive) {
    softStart.delayActive = false;
    publishMqttEvent("SOFTSTART_CANCELLED", "Safety override");
  }
}

uint32_t getSoftStartRemainingMs() {
  if (!softStart.delayActive) return 0;
  uint32_t elapsed = millis() - softStart.delayStartTime;
  return (elapsed >= soft_start_delay_ms) ? 0 : soft_start_delay_ms - elapsed;
}

void updatePowerQuality() {
  float voltage = s31.getVoltage();
  float current = s31.getCurrent();
  float realPower = s31.getPower();
  bool relayState = s31.getRelayState();
  apparentPower = voltage * current;
  if (!relayState || realPower < 0.5) powerFactor = 0.0;
  else if (apparentPower > 0.01) powerFactor = constrain(realPower / apparentPower, 0.0, 1.0);
  else powerFactor = 0.0;
  total_energy_kwh = s31.getEnergy();
}

void parseEspNowData(String data) {
  float d = 0, l = 0, v = 0, b = 0;
  int dIndex = data.indexOf("\"d\":");
  if (dIndex != -1) { int start = dIndex + 4, end = data.indexOf(",", start); if (end == -1) end = data.indexOf("}", start); if (end != -1) d = data.substring(start, end).toFloat(); }
  int lIndex = data.indexOf("\"l\":");
  if (lIndex != -1) { int start = lIndex + 4, end = data.indexOf(",", start); if (end == -1) end = data.indexOf("}", start); if (end != -1) l = data.substring(start, end).toFloat(); }
  int vIndex = data.indexOf("\"v\":");
  if (vIndex != -1) { int start = vIndex + 4, end = data.indexOf(",", start); if (end == -1) end = data.indexOf("}", start); if (end != -1) v = data.substring(start, end).toFloat(); }
  int bIndex = data.indexOf("\"b\":");
  if (bIndex != -1) { int start = bIndex + 4, end = data.indexOf(",", start); if (end == -1) end = data.indexOf("}", start); if (end != -1) b = data.substring(start, end).toFloat(); }
  if (d > 0 || l > 0 || v > 0 || b > 0) { currentDistance = d; currentWaterLevel = l; currentVolume = v; batteryVoltage = b; lastEspNowData = millis(); espnowDataValid = true; sensorIsDead = false; }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {}
void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) { if (len == sizeof(EspNowPacket)) { memcpy(&incoming, data, sizeof(incoming)); parseEspNowData(String(incoming.msg)); } }

void initEspNow() {
  if (!use_espnow) { espnow_initialized = false; return; }
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != 0) { espnow_initialized = false; return; }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  uint8_t* peerMac = (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) ? peer_mac : broadcastMac;
  if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_COMBO, espnow_channel, NULL, 0) != 0) { espnow_initialized = false; return; }
  espnow_initialized = true;
}

void sendEspNowCommand(const char* command) {
  if (!espnow_initialized) return;
  for (uint8_t retry = 0; retry < ESP_NOW_RETRY_COUNT; retry++) {
    outgoing.seq = espnow_seq++;
    outgoing.timestamp = micros();
    strncpy(outgoing.msg, command, sizeof(outgoing.msg)-1);
    uint8_t* peerMac = (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) ? peer_mac : broadcastMac;
    if (esp_now_send(peerMac, (uint8_t *)&outgoing, sizeof(outgoing)) == 0) return;
    delay(50);
  }
}

void requestSensorData() { if (espnow_initialized && millis() - lastEspNowSend >= ESP_NOW_SEND_INTERVAL) { lastEspNowSend = millis(); sendEspNowCommand("get_measure"); } }

void recoverEspNow() {
  if (!use_espnow) return;
  if (millis() - lastEspNowRecovery < ESP_NOW_RECOVERY_INTERVAL) return;
  bool dataStale = (millis() - lastEspNowData > ESP_NOW_DATA_TIMEOUT * 2);
  if (!espnow_initialized || (dataStale && lastEspNowData > 0)) { esp_now_deinit(); delay(100); initEspNow(); lastEspNowRecovery = millis(); }
}

void updateInrushState(bool currentState) {
  if (!inrush_enabled) { overload.inrushActive = false; overload.lastRelayState = currentState; return; }
  if (currentState && !overload.lastRelayState) { overload.pumpStartTime = millis(); overload.inrushActive = true; }
  if (overload.inrushActive && currentState && (millis() - overload.pumpStartTime >= inrush_tolerance_ms)) overload.inrushActive = false;
  if (!currentState) overload.inrushActive = false;
  overload.lastRelayState = currentState;
}

bool checkOverloadProtection(float currentPower, bool currentState) {
  updateInrushState(currentState);
  if (overload.cooldownUntil > millis()) { if (currentState) s31.setRelay(false); return false; }
  if (overload.active && millis() >= overload.cooldownUntil) overload.active = false;
  if (!pump_load_protection_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  if (currentPower > max_power_threshold && !overload.active) {
    if (millis() - overload.pumpStartTime > 500) {
      overload.active = true;
      overload.cooldownUntil = millis() + (overload_cooldown_seconds * 1000);
      overload.overloadCount++;
      overload_events = overload.overloadCount;
      char msg[80]; snprintf(msg, sizeof(msg), "⚠️ OVERLOAD! %.0fW - Pump stopped - All timers cancelled", currentPower);
      addFailureLogEntry(msg);
      publishMqttEvent("OVERLOAD", String(currentPower) + "W exceeded threshold");
      s31.setRelay(false);
      cancelAllTimers();
      return false;
    }
  }
  return true;
}

uint32_t getOverloadCooldownRemaining() {
  if (overload.cooldownUntil <= millis()) return 0;
  return (overload.cooldownUntil - millis()) / 1000;
}

bool checkDryRunProtection(float currentPower, bool currentState) {
  if (dryRun.cooldownUntil > millis()) { if (currentState) s31.setRelay(false); return false; }
  if (dryRun.inCooldown && millis() >= dryRun.cooldownUntil) {
    dryRun.inCooldown = false;
    dryRun.protectionTriggered = false;
    dryRun.lowPowerDetected = false;
    addFailureLogEntry("✅ Dry run cooldown ended - Pump can restart");
    publishMqttEvent("DRY_RUN_COOLDOWN_END", "Pump can restart");
  }
  if (!dry_run_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  if (min_power_threshold <= 0.1) return true;
  if (currentPower < min_power_threshold) {
    if (!dryRun.lowPowerDetected) {
      dryRun.lowPowerDetected = true;
      dryRun.lowPowerStartTime = millis();
      char msg[80]; snprintf(msg, sizeof(msg), "⚠️ Low power detected: %.1fW < %.1fW - Timer started (%lus)", currentPower, min_power_threshold, pump_protection_time);
      addFailureLogEntry(msg);
      publishMqttEvent("DRY_RUN_WARNING", "Low power detected, timer started");
    }
    uint32_t elapsedSeconds = (millis() - dryRun.lowPowerStartTime) / 1000;
    if (elapsedSeconds >= pump_protection_time) {
      char msg[80]; snprintf(msg, sizeof(msg), "💧 DRY RUN! Pump stopped INSTANTLY after %lus - All timers cancelled", pump_protection_time);
      addFailureLogEntry(msg);
      publishMqttEvent("DRY_RUN_TRIGGERED", "Pump stopped - low power for " + String(pump_protection_time) + "s");
      dryRun.protectionTriggered = true;
      dryRun.inCooldown = true;
      dryRun.cooldownUntil = millis() + (dryrun_cooldown_seconds * 1000);
      dryRun.events++;
      dryrun_events = dryRun.events;
      dryRun.lowPowerDetected = false;
      cancelAllTimers();
      return false;
    }
  } else {
    if (dryRun.lowPowerDetected) {
      dryRun.lowPowerDetected = false;
      dryRun.lowPowerStartTime = 0;
      addFailureLogEntry("✅ Power restored - Dry run condition cleared");
      publishMqttEvent("DRY_RUN_CLEARED", "Power restored");
    }
  }
  return true;
}

uint32_t getDryRunCooldownRemaining() {
  if (dryRun.cooldownUntil <= millis()) return 0;
  return (dryRun.cooldownUntil - millis()) / 1000;
}

bool checkLowVoltageProtection(float currentVoltage, bool currentState) {
  if (lowVoltage.cooldownUntil > millis()) { if (currentState) s31.setRelay(false); return false; }
  if (!low_voltage_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  if (currentVoltage < min_voltage_threshold && currentVoltage > 0) {
    if (!lowVoltage.lowVoltageDetected) {
      lowVoltage.lowVoltageDetected = true;
      lowVoltage.lowVoltageStartTime = millis();
      char msg[80]; snprintf(msg, sizeof(msg), "⚠️ Low voltage detected: %.1fV < %.1fV", currentVoltage, min_voltage_threshold);
      addFailureLogEntry(msg);
      publishMqttEvent("LOW_VOLTAGE_WARNING", String(currentVoltage) + "V below threshold");
    }
    uint32_t elapsedSeconds = (millis() - lowVoltage.lowVoltageStartTime) / 1000;
    if (currentVoltage < LOW_VOLTAGE_IMMEDIATE_THRESHOLD || elapsedSeconds >= LOW_VOLTAGE_DELAY_SECONDS) {
      char msg[80]; snprintf(msg, sizeof(msg), "⚡ LOW VOLTAGE! %.1fV - Pump stopped - All timers cancelled", currentVoltage);
      addFailureLogEntry(msg);
      publishMqttEvent("LOW_VOLTAGE_TRIGGERED", String(currentVoltage) + "V - pump stopped");
      lowVoltage.active = true;
      lowVoltage.cooldownUntil = millis() + (low_voltage_cooldown_seconds * 1000);
      lowVoltage.events++;
      low_voltage_events = lowVoltage.events;
      lowVoltage.lowVoltageDetected = false;
      cancelAllTimers();
      return false;
    }
  } else {
    if (lowVoltage.lowVoltageDetected) {
      lowVoltage.lowVoltageDetected = false;
      lowVoltage.lowVoltageStartTime = 0;
      addFailureLogEntry("✅ Voltage restored - Low voltage condition cleared");
      publishMqttEvent("LOW_VOLTAGE_CLEARED", "Voltage restored");
    }
  }
  return true;
}

uint32_t getLowVoltageCooldownRemaining() {
  if (lowVoltage.cooldownUntil <= millis()) return 0;
  return (lowVoltage.cooldownUntil - millis()) / 1000;
}

bool checkHighVoltageProtection(float currentVoltage, bool currentState) {
  if (highVoltage.cooldownUntil > millis()) { if (currentState) s31.setRelay(false); return false; }
  if (!high_voltage_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  if (currentVoltage > max_voltage_threshold && currentVoltage > 0) {
    if (!highVoltage.highVoltageDetected) {
      highVoltage.highVoltageDetected = true;
      highVoltage.highVoltageStartTime = millis();
      char msg[80]; snprintf(msg, sizeof(msg), "⚠️ High voltage detected: %.1fV > %.1fV", currentVoltage, max_voltage_threshold);
      addFailureLogEntry(msg);
      publishMqttEvent("HIGH_VOLTAGE_WARNING", String(currentVoltage) + "V above threshold");
    }
    uint32_t elapsedSeconds = (millis() - highVoltage.highVoltageStartTime) / 1000;
    if (currentVoltage > HIGH_VOLTAGE_IMMEDIATE_THRESHOLD || elapsedSeconds >= HIGH_VOLTAGE_DELAY_SECONDS) {
      char msg[80]; snprintf(msg, sizeof(msg), "⚡ HIGH VOLTAGE! %.1fV - Pump stopped - All timers cancelled", currentVoltage);
      addFailureLogEntry(msg);
      publishMqttEvent("HIGH_VOLTAGE_TRIGGERED", String(currentVoltage) + "V - pump stopped");
      highVoltage.active = true;
      highVoltage.cooldownUntil = millis() + (high_voltage_cooldown_seconds * 1000);
      highVoltage.events++;
      high_voltage_events = highVoltage.events;
      highVoltage.highVoltageDetected = false;
      cancelAllTimers();
      return false;
    }
  } else {
    if (highVoltage.highVoltageDetected) {
      highVoltage.highVoltageDetected = false;
      highVoltage.highVoltageStartTime = 0;
      addFailureLogEntry("✅ Voltage restored - High voltage condition cleared");
      publishMqttEvent("HIGH_VOLTAGE_CLEARED", "Voltage restored");
    }
  }
  return true;
}

uint32_t getHighVoltageCooldownRemaining() {
  if (highVoltage.cooldownUntil <= millis()) return 0;
  return (highVoltage.cooldownUntil - millis()) / 1000;
}

void updatePumpStatistics(bool currentState) {
  static uint32_t lastRuntimeUpdate = 0;
  uint32_t now = millis();
  if (currentState && !overload.lastRelayState) { pump_cycles++; lastRuntimeUpdate = now; }
  else if (!currentState && overload.lastRelayState && lastRuntimeUpdate > 0) { total_runtime_seconds += (now - lastRuntimeUpdate) / 1000; total_energy_kwh = s31.getEnergy(); }
}

void checkAutoReturnToAutoMode() {
  if (auto_return_enabled && !auto_mode && button.lastManualModeTime > 0 && millis() - button.lastManualModeTime >= AUTO_RETURN_TIMEOUT_MS) {
    switchToAutoMode();
  }
}

bool getDesiredPumpState(bool currentState) {
  if (!auto_mode) return currentState;
  bool pressureLow = pressureAllowsPumpOn();
  updatePressureLockout(pressureLow);
  if (isPressureLockoutActive()) return false;
  return pressureLow;
}

void controlPump() {
  updatePowerQuality();
  checkAutoReturnToAutoMode();
  checkInchingTimer();
  if (millis() - lastControlCheck < CONTROL_INTERVAL) return;
  lastControlCheck = millis();
  yield();
  ESP.wdtFeed();
  
  bool currentState = s31.getRelayState();
  float currentPower = s31.getPower();
  float currentVoltage = s31.getVoltage();
  
  if (!checkOverloadProtection(currentPower, currentState)) {
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  if (!checkHighVoltageProtection(currentVoltage, currentState)) {
    s31.setRelay(false);
    cancelSoftStartDelay();
    cancelInchingTimer();
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  if (!checkLowVoltageProtection(currentVoltage, currentState)) {
    s31.setRelay(false);
    cancelSoftStartDelay();
    cancelInchingTimer();
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  if (!checkDryRunProtection(currentPower, currentState)) {
    s31.setRelay(false);
    cancelSoftStartDelay();
    cancelInchingTimer();
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  bool desiredState = getDesiredPumpState(currentState);
  bool finalState = updateSoftStartState(desiredState);
  
  if (finalState && !currentState && inching_enabled && auto_mode) startInchingTimer();
  if (!finalState && currentState && inching.active) cancelInchingTimer();
  
  if (finalState != currentState) {
    s31.setRelay(finalState);
    const char* reason;
    if (inching.active) reason = "INCHING";
    else if (softStart.delayActive) reason = "SOFT START";
    else if (!auto_mode) reason = "MANUAL";
    else if (pressureLockout.active) reason = "LOCKOUT";
    else if (pressureSwitch.pressureLow) reason = "LOW PRESSURE";
    else reason = "HIGH PRESSURE";
    char msg[48]; snprintf(msg, sizeof(msg), "Pump %s - %s", finalState ? "ON" : "OFF", reason);
    addFailureLogEntry(msg);
    publishMqttEvent(finalState ? "PUMP_ON" : "PUMP_OFF", reason);
  }
  
  updatePumpStatistics(s31.getRelayState());
  overload.lastRelayState = currentState;
}

void initButton() { pinMode(BUTTON_PIN, INPUT_PULLUP); pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH); }
void flashLed(uint32_t duration_ms) { uint32_t delayTime = duration_ms > 100 ? 100 : duration_ms; digitalWrite(LED_PIN, LOW); delay(delayTime); digitalWrite(LED_PIN, HIGH); }

void handleShortPress() {
  button.shortPressCount++;
  button_press_count = button.shortPressCount + button.longPressCount;
  flashLed(50);
  
  if (millis() < overload.cooldownUntil || millis() < dryRun.cooldownUntil || millis() < lowVoltage.cooldownUntil || millis() < highVoltage.cooldownUntil) {
    addFailureLogEntry("Button: Cooldown active - Cannot switch mode");
    flashLed(1000);
    return;
  }
  
  if (auto_mode) {
    addFailureLogEntry("🔘 Switching from AUTO to MANUAL mode");
    switchToManualMode();
    flashLed(100); delay(100); flashLed(100);
  } else {
    button.lastManualModeTime = millis();
    if (pressureLockout.active) { addFailureLogEntry("Button: Pressure lockout active - Cannot start pump"); flashLed(1000); return; }
    bool newState = !s31.getRelayState();
    s31.setRelay(newState);
    if (newState) { cancelInchingTimer(); cancelSoftStartDelay(); addFailureLogEntry("Button: Pump turned ON in MANUAL mode"); publishMqttEvent("PUMP_ON", "Manual button press"); }
    else { addFailureLogEntry("Button: Pump turned OFF in MANUAL mode"); publishMqttEvent("PUMP_OFF", "Manual button press"); }
    flashLed(50);
  }
}

void handleLongPress() {
  button.longPressCount++;
  button_press_count = button.shortPressCount + button.longPressCount;
  if (auto_mode) {
    switchToManualMode();
    for (int i = 0; i < 3; i++) { flashLed(100); delay(150); }
  } else {
    switchToAutoMode();
    for (int i = 0; i < 2; i++) { flashLed(100); delay(150); }
  }
}

void handleButton() {
  if (!button_enabled) return;
  uint32_t now = millis();
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != button.lastButtonState) button.lastDebounceTime = now;
  if ((now - button.lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
    if (reading != button.currentButtonState) {
      button.currentButtonState = reading;
      if (button.currentButtonState == LOW) { button.pressStartTime = now; button.buttonPressed = true; }
      else if (button.buttonPressed) {
        uint32_t pressDuration = now - button.pressStartTime;
        if (pressDuration < SHORT_PRESS_MAX_MS) handleShortPress();
        else if (pressDuration >= LONG_PRESS_MIN_MS && pressDuration <= LONG_PRESS_MAX_MS) handleLongPress();
        button.buttonPressed = false;
      }
    }
  }
  button.lastButtonState = reading;
}

// ================================================================================================
// @section     WEB SERVER & HTML UI (Preserved v3.8 UI with improved scan)
// ================================================================================================

void setupAPMode() { String apSSID = "SmartPump-" + String(ESP.getChipId() & 0xFFFF, HEX); WiFi.softAP(apSSID.c_str(), "12345678"); if (ENABLE_CAPTIVE_PORTAL) dnsServer.start(53, "*", IPAddress(192, 168, 4, 1)); }
void setupArduinoOTA() { if (!ENABLE_OTA) return; ArduinoOTA.setHostname(deviceName.c_str()); ArduinoOTA.begin(); }

// HTML UI - Main Dashboard (Preserved v3.8 UI)
const char simple_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
<title>Smart Pump v3.9</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',Roboto,sans-serif;background:#f1f5f9;padding:16px}
.container{max-width:550px;margin:0 auto}
.card{background:white;border-radius:32px;padding:20px;margin-bottom:18px;box-shadow:0 4px 12px rgba(0,0,0,0.05)}
.header{text-align:center}
h1{font-size:1.7rem;background:linear-gradient(135deg,#0f172a,#2563eb);background-clip:text;-webkit-background-clip:text;color:transparent}
.version{background:#2563eb20;color:#1e40af;padding:4px 12px;border-radius:40px;font-size:0.7rem;display:inline-block;margin:6px 0}
.uptime{background:#dbeafe;padding:10px;border-radius:20px;text-align:center;margin-bottom:12px;font-size:0.85rem}
.wifi-status{background:#f0fdf4;padding:8px;border-radius:20px;margin-bottom:8px;font-size:0.75rem;display:flex;justify-content:space-between;flex-wrap:wrap}
.mqtt-status{background:#e0f2fe;padding:8px;border-radius:20px;margin-bottom:12px;font-size:0.75rem;display:flex;justify-content:space-between;flex-wrap:wrap}
.ntp-status{background:#fef3c7;padding:4px 8px;border-radius:16px;font-size:0.65rem;display:inline-block;margin-left:8px}
.wifi-connected{color:#059669}
.wifi-disconnected{color:#dc2626}
.mqtt-connected{color:#059669}
.mqtt-disconnected{color:#dc2626}
.gauge{text-align:center;margin:10px 0}
.gauge-title{font-size:0.8rem;color:#475569;margin-bottom:8px}
.vertical-gauge{width:180px;height:200px;margin:0 auto;background:#e2e8f0;border-radius:30px;position:relative;overflow:hidden}
.water-fill{background:linear-gradient(180deg,#3b82f6,#1e40af);position:absolute;bottom:0;left:0;right:0;transition:height 0.5s;display:flex;align-items:center;justify-content:center;color:white;font-weight:bold}
.level-text{font-size:2rem;font-weight:800;margin-top:12px}
.hidden{display:none}
.stats-row{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin:16px 0}
.stat-block{background:#f8fafc;border-radius:24px;padding:12px;text-align:center}
.stat-value{font-size:1.8rem;font-weight:700}
.stat-label{font-size:0.7rem;color:#475569}
.mode-row{display:flex;justify-content:space-between;align-items:center;background:#f1f5f9;padding:12px 16px;border-radius:60px;margin:16px 0}
.toggle-switch{position:relative;display:inline-block;width:56px;height:28px}
.toggle-switch input{opacity:0;width:0;height:0}
.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#cbd5e1;transition:0.3s;border-radius:34px}
.slider:before{position:absolute;content:"";height:22px;width:22px;left:3px;bottom:3px;background-color:white;transition:0.3s;border-radius:50%}
input:checked+.slider{background-color:#2563eb}
input:checked+.slider:before{transform:translateX(28px)}
.pump-btn{width:100%;padding:18px;border-radius:60px;border:none;font-weight:700;font-size:1.4rem;background:#dc2626;color:white;margin:12px 0;cursor:pointer;transition:all 0.3s ease}
.pump-btn.running{background:#10b981;animation:pulse 1.8s infinite}
.pump-btn.cooldown{background:#6b7280;cursor:not-allowed}
.pump-btn.lockout{background:#dc2626;opacity:0.7;cursor:not-allowed}
.pump-btn.softstart{background:#06b6d4;animation:pulse 1s infinite}
.pump-btn.inching{background:#8b5cf6;animation:pulse 0.8s infinite}
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(16,185,129,0.7)}70%{box-shadow:0 0 0 15px rgba(16,185,129,0)}100%{box-shadow:0 0 0 0 rgba(16,185,129,0)}}
.flex-between{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
.btn-secondary{background:#e2e8f0;border:none;padding:10px 16px;border-radius:40px;font-weight:500;width:100%;cursor:pointer;margin-top:8px}
.engmode-link{text-align:center;margin-top:12px;font-size:0.7rem}
.engmode-link a{color:#94a3b8;text-decoration:none}
hr{margin:14px 0}
.protection-badge{background:#fef3c7;color:#92400e;padding:4px 8px;border-radius:20px;font-size:0.7rem;margin-top:8px;display:inline-block;margin-right:8px}
.timer-badge{background:#e0e7ff;color:#4338ca;padding:4px 8px;border-radius:20px;font-size:0.7rem;margin-top:8px;display:inline-block}
.sensor-chip{background:#e6f7ec;padding:5px 10px;border-radius:50px;font-size:0.7rem;display:inline-flex;align-items:center;gap:6px}
.led{width:10px;height:10px;border-radius:10px;display:inline-block}
.led-green{background:#22c55e}
.led-red{background:#ef4444}
.led-yellow{background:#eab308}
.led-gray{background:#94a3b8}
.electrical-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:12px 0}
.espnow-warning{background:#fff7ed;color:#9a3412;padding:10px;border-radius:12px;margin-bottom:12px;text-align:center;font-size:0.8rem}
.display-only-badge{background:#fef3c7;color:#92400e;padding:4px 8px;border-radius:20px;font-size:0.65rem;margin-left:8px}
.countdown-overlay{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.8);display:flex;align-items:center;justify-content:center;z-index:1000;flex-direction:column}
.countdown-box{background:white;border-radius:32px;padding:40px;text-align:center;max-width:300px}
.countdown-number{font-size:5rem;font-weight:800;color:#2563eb}
.countdown-text{font-size:1.2rem;margin-top:10px;color:#333}
.countdown-cancel{background:#ef4444;color:white;border:none;padding:12px 24px;border-radius:40px;margin-top:20px;cursor:pointer;font-weight:bold}
.voltage-warning{background:#fef2f2;border-left:4px solid #ef4444;padding:8px 12px;margin:8px 0;border-radius:12px;font-size:0.75rem}
.voltage-ok{background:#f0fdf4;border-left:4px solid #22c55e;padding:8px 12px;margin:8px 0;border-radius:12px;font-size:0.75rem}
.stat-reset-btn{background:#e2e8f0;border:none;border-radius:20px;width:28px;height:28px;font-size:14px;cursor:pointer;color:#64748b;transition:all 0.2s ease;margin-left:8px}
.stat-reset-btn:hover{background:#cbd5e1;color:#ef4444;transform:scale(1.05)}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%);padding:12px 24px;border-radius:50px;font-size:14px;font-weight:bold;z-index:2000;animation:slideUp 0.3s ease;box-shadow:0 4px 12px rgba(0,0,0,0.15)}
.toast.success{background:#10b981;color:white}
.toast.error{background:#ef4444;color:white}
.toast.info{background:#3b82f6;color:white}
@keyframes slideUp{from{opacity:0;transform:translateX(-50%) translateY(20px)}to{opacity:1;transform:translateX(-50%) translateY(0)}}
@keyframes slideDown{from{opacity:1;transform:translateX(-50%) translateY(0)}to{opacity:0;transform:translateX(-50%) translateY(20px)}}
.spinner{display:inline-block;width:16px;height:16px;border:2px solid #e2e8f0;border-top-color:#667eea;border-radius:50%;animation:spin 0.6s linear infinite;margin-right:8px;vertical-align:middle}
@keyframes spin{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<div class="container">
<div class="card">
<div class="header"><h1>💧 AquaPro S31</h1><div class="version">v3.9 IMPROVED WIFI SCAN</div></div>
<div class="uptime">⏱️ Uptime: <strong id="uptime">0</strong> <span id="ntpTime" class="ntp-status">--:--:--</span></div>
<div class="wifi-status"><span>📡 WiFi:</span><span id="wifiStatus">AP Mode</span><span id="wifiIp"></span></div>
<div class="mqtt-status"><span>📨 MQTT:</span><span id="mqttStatus">Disabled</span></div>
<div class="flex-between"><span>📡 Sensor</span><span id="sensorBadge" class="sensor-chip"><span class="led led-gray"></span> ⚪ Disabled</span></div>
<div class="flex-between"><span>🔘 Pressure</span><span id="pressureStatus" class="sensor-chip"><span class="led led-green"></span> OK</span></div>

<div id="timerOverload" style="display:none" class="timer-badge">⚠️ Overload: <span id="overloadTimer">0</span></div>
<div id="timerDryRun" style="display:none" class="timer-badge">💧 Dry Run Cooldown: <span id="dryrunTimer">0</span></div>
<div id="timerLowVoltage" style="display:none" class="timer-badge">⚡ Low Voltage: <span id="lowVoltageTimer">0</span></div>
<div id="timerHighVoltage" style="display:none" class="timer-badge">⚡ High Voltage: <span id="highVoltageTimer">0</span></div>
<div id="timerLockout" style="display:none" class="timer-badge">🔒 Lockout: <span id="lockoutTimer">0</span></div>
<div id="timerSoftStart" style="display:none" class="timer-badge">🌊 Soft Start: <span id="softStartTimer">0</span>ms</div>
<div id="timerInching" style="display:none" class="timer-badge">⏱️ Inching: <span id="inchingTimer">0</span></div>

<div id="lockoutDiv" style="background:#fee2e2;color:#991b1b;padding:8px;border-radius:20px;margin-top:10px;text-align:center;display:none">🔒 LOCKOUT ACTIVE <button onclick="resetLockout()" style="background:#991b1b;color:white;border:none;padding:2px 10px;border-radius:15px">Reset</button></div>
<div id="espnowWarning" class="espnow-warning" style="display:none"></div>
</div>

<div id="virtualTankCard" class="card hidden">
<div class="gauge"><div class="gauge-title">💧 Water Tank Level <span class="display-only-badge">DISPLAY ONLY</span></div><div class="vertical-gauge"><div class="water-fill" id="waterFill" style="height:0%">0%</div></div><div class="level-text"><span id="level">0</span>%</div></div>
<div class="stats-row"><div class="stat-block"><div class="stat-value"><span id="sensorVoltage">0.00</span></div><div class="stat-label">Battery</div></div><div class="stat-block"><div class="stat-value"><span id="volume">0</span> L</div><div class="stat-label">Volume</div></div></div>
</div>

<div class="card">
<div class="electrical-grid">
<div class="stat-block"><div class="stat-value"><span id="voltage">0</span> V</div><div class="stat-label">Voltage</div></div>
<div class="stat-block"><div class="stat-value"><span id="current">0.00</span> A</div><div class="stat-label">Current</div></div>
<div class="stat-block"><div class="stat-value"><span id="power">0</span> W</div><div class="stat-label">Power</div></div>
<div class="stat-block"><div class="stat-value"><span id="energy">0.0</span> kWh</div><div class="stat-label">Energy</div></div>
</div>
<div id="voltageStatus"></div>
<div id="warnings"></div>
</div>

<div class="card">
<div class="mode-row"><span id="modeText">🤖 AUTO</span><label class="toggle-switch"><input type="checkbox" id="autoToggle" onchange="toggleAuto()"><span class="slider"></span></label><span>MANUAL</span></div>
<button id="pumpBtn" class="pump-btn" onclick="manualToggle()">PUMP OFF</button>
<div id="reason" style="font-size:0.7rem;text-align:center;margin-top:5px"></div>
<hr>
<div class="flex-between"><span>📦 Runtime</span><strong><span id="runtime">0</span> min <button class="stat-reset-btn" onclick="resetRuntimeStats()" title="Reset runtime and cycles">↺</button></strong></div>
<div class="flex-between"><span>🔄 Cycles</span><strong><span id="cycles">0</span></strong></div>
<div class="flex-between"><span>⚠️ Overload</span><strong><span id="overloadCount">0</span></strong></div>
<div class="flex-between"><span>💧 Dry run</span><strong><span id="dryrunCount">0</span></strong></div>
<div class="flex-between"><span>⚡ Low V</span><strong><span id="lowVoltageCount">0</span></strong></div>
<div class="flex-between"><span>⚡ High V</span><strong><span id="highVoltageCount">0</span></strong></div>
<button class="btn-secondary" style="margin-top:16px;background:#fee2e2;color:#dc2626;border:1px solid #dc2626" onclick="resetAllStats()">📊 Reset All Statistics</button>
</div>

<div class="card">
  <button class="btn-secondary" onclick="requestSensor()">📡 Request sensor</button>
  <button class="btn-secondary" style="background:#fee2e2;color:#b91c1c" id="rebootBtn" onclick="startRebootCountdown()">🔄 Reboot Device</button>
</div>
<div class="engmode-link"><a href="/engmode">🔧 Engineering Mode</a></div>
</div>

<div id="countdownOverlay" style="display:none" class="countdown-overlay">
<div class="countdown-box">
<div class="countdown-number" id="countdownNumber">5</div>
<div class="countdown-text">Rebooting in <span id="countdownText">5</span> seconds...</div>
<button class="countdown-cancel" onclick="cancelReboot()">Cancel</button>
</div>
</div>

<script>
let countdownInterval = null;
let rebootCancelled = false;
let scanRetryCount = 0;
let scanInterval = null;

function formatTimeRemaining(seconds){
    if(seconds>=86400) return Math.floor(seconds/86400)+"d "+Math.floor((seconds%86400)/3600)+"h";
    if(seconds>=3600) return Math.floor(seconds/3600)+"h "+Math.floor((seconds%3600)/60)+"m";
    if(seconds>=60) return Math.floor(seconds/60)+"m "+Math.floor(seconds%60)+"s";
    return seconds+"s";
}

function showToast(message, type){
    const toast=document.createElement('div');
    toast.className='toast '+type;
    toast.innerHTML=message;
    document.body.appendChild(toast);
    setTimeout(()=>{
        toast.style.animation='slideDown 0.3s ease';
        setTimeout(()=>toast.remove(),300);
    },2000);
}

async function fetchJSON(u){try{const r=await fetch(u);return await r.json()}catch(e){return null}}
async function toggleAuto(){const isAuto=document.getElementById('autoToggle').checked;await fetch(`/mode?mode=${isAuto?'auto':'manual'}`);refresh()}
async function manualToggle(){const d=await fetchJSON('/data');if(d&&d.pressureLockoutActive){alert("Lockout active!");return}await fetch('/toggle');refresh()}
async function resetLockout(){await fetch('/reset_lockout');refresh();showToast('Lockout reset','success')}
async function requestSensor(){await fetch('/espnow/request');refresh();showToast('Sensor data requested','info')}

async function resetRuntimeStats(){
    if(confirm('Reset runtime and cycles only?\n\nThis will reset:\n- Runtime counter\n- Cycle counter\n\nProtection event counters will NOT be affected.\n\nContinue?')){
        const response=await fetch('/reset_stats?type=runtime');
        if(response.ok){showToast('✅ Runtime & cycles reset successfully','success');setTimeout(()=>refresh(),500);}
        else showToast('❌ Failed to reset statistics','error');
    }
}

async function resetAllStats(){
    if(confirm('⚠️⚠️ RESET ALL STATISTICS ⚠️⚠️\n\nThis will reset ALL counters:\n- 📦 Runtime (0 min)\n- 🔄 Cycles (0)\n- ⚠️ Overload events (0)\n- 💧 Dry run events (0)\n- ⚡ Low Voltage events (0)\n- ⚡ High Voltage events (0)\n\n⚠️ This action CANNOT be undone!\n\nContinue?')){
        const confirmation=prompt('Type "RESET" to confirm statistics reset:');
        if(confirmation==='RESET'){
            const response=await fetch('/reset_stats?type=all');
            if(response.ok){showToast('📊 All statistics reset successfully','success');setTimeout(()=>refresh(),500);}
            else showToast('❌ Failed to reset statistics','error');
        }else{showToast('❌ Reset cancelled','error');}
    }
}

// IMPROVED WiFi Scan - Single click with auto-retry
async function scanWiFi() {
    const scanBtn = event.target;
    const rd = document.getElementById('wifiScanResult');
    const sd = document.getElementById('wifiScanning');
    
    if(!rd || !sd) return;
    
    // Disable button during scan
    scanBtn.disabled = true;
    scanBtn.style.opacity = '0.5';
    scanBtn.innerHTML = '<span class="spinner"></span> Scanning...';
    
    rd.style.display = 'none';
    sd.style.display = 'block';
    sd.innerHTML = '<div style="padding:20px"><span class="spinner"></span> Scanning WiFi networks...<br><span style="font-size:12px">Please wait 3-5 seconds</span></div>';
    
    scanRetryCount = 0;
    
    function checkScan() {
        fetch('/scan_wifi')
            .then(res => res.json())
            .then(data => {
                if(data.scanning === true) {
                    scanRetryCount++;
                    const dots = '.'.repeat(scanRetryCount % 4);
                    sd.innerHTML = `<div style="padding:20px"><span class="spinner"></span> Scanning WiFi networks${dots}<br><span style="font-size:12px">Please wait...</span></div>`;
                    
                    if(scanRetryCount < 10) {
                        setTimeout(checkScan, 1000);
                    } else {
                        sd.style.display = 'none';
                        rd.style.display = 'block';
                        rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan timeout. Please try again.</div>';
                        scanBtn.disabled = false;
                        scanBtn.style.opacity = '1';
                        scanBtn.innerHTML = '📡 Scan';
                    }
                } else if(data.error) {
                    sd.style.display = 'none';
                    rd.style.display = 'block';
                    rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan failed. Please try again.</div>';
                    scanBtn.disabled = false;
                    scanBtn.style.opacity = '1';
                    scanBtn.innerHTML = '📡 Scan';
                } else if(Array.isArray(data)) {
                    sd.style.display = 'none';
                    rd.style.display = 'block';
                    
                    if(data.length === 0) {
                        rd.innerHTML = '<div style="padding:20px">📡 No WiFi networks found</div>';
                    } else {
                        let html = '<div style="font-weight:bold;padding:10px;background:#e2e8f0;border-radius:10px;margin-bottom:10px">📡 Found ' + data.length + ' networks:</div>';
                        for(let i = 0; i < data.length; i++) {
                            let signalIcon = '';
                            let signalColor = '';
                            if(data[i].rssi > -50) { signalIcon = '📶 Excellent'; signalColor = '#10b981'; }
                            else if(data[i].rssi > -70) { signalIcon = '📶 Good'; signalColor = '#f59e0b'; }
                            else { signalIcon = '📶 Fair'; signalColor = '#ef4444'; }
                            
                            let encryptionIcon = data[i].encryption > 0 ? '🔒' : '🔓';
                            
                            html += `<div class="wifi-network" style="padding:12px;border-bottom:1px solid #e2e8f0;cursor:pointer;display:flex;justify-content:space-between;align-items:center" onclick="selectNetwork('${data[i].ssid.replace(/'/g, "\\'")}')">
                                        <div><strong>${data[i].ssid}</strong><br><span style="font-size:11px;color:${signalColor}">${signalIcon}</span></div>
                                        <div><span style="font-size:12px">${data[i].rssi}dBm</span> ${encryptionIcon}</div>
                                     </div>`;
                        }
                        rd.innerHTML = html;
                    }
                    scanBtn.disabled = false;
                    scanBtn.style.opacity = '1';
                    scanBtn.innerHTML = '📡 Scan';
                }
            })
            .catch(err => {
                console.error('Scan error:', err);
                sd.style.display = 'none';
                rd.style.display = 'block';
                rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan error. Please try again.</div>';
                scanBtn.disabled = false;
                scanBtn.style.opacity = '1';
                scanBtn.innerHTML = '📡 Scan';
            });
    }
    
    checkScan();
}

function selectNetwork(ssid) {
    document.getElementById('wifiSsid').value = ssid;
    document.getElementById('wifiScanResult').style.display = 'none';
    showToast(`Selected: ${ssid}`, 'success');
}

function startRebootCountdown(){
    document.getElementById('countdownOverlay').style.display='flex';
    rebootCancelled=false;
    let countdown=5;
    const countdownNumber=document.getElementById('countdownNumber');
    const countdownText=document.getElementById('countdownText');
    const rebootBtn=document.getElementById('rebootBtn');
    rebootBtn.disabled=true;
    rebootBtn.style.opacity='0.5';
    countdownInterval=setInterval(()=>{
        countdown--;
        if(countdown>=0){
            countdownNumber.innerText=countdown;
            countdownText.innerText=countdown;
        }
        if(countdown<0){
            clearInterval(countdownInterval);
            if(!rebootCancelled){
                fetch('/reboot');
                countdownNumber.innerText='✓';
                countdownText.innerText='Rebooting...';
                setTimeout(()=>{location.reload();},500);
            }
        }
    },1000);
}

function cancelReboot(){
    rebootCancelled=true;
    clearInterval(countdownInterval);
    document.getElementById('countdownOverlay').style.display='none';
    const rebootBtn=document.getElementById('rebootBtn');
    rebootBtn.disabled=false;
    rebootBtn.style.opacity='1';
    showToast('Reboot cancelled','info');
}

function formatUptime(s){let d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60);if(d>0)return d+"d "+h+"h";if(h>0)return h+"h "+m+"m";return m+"m"}

async function refresh(){
const d=await fetchJSON('/data');if(!d)return;
document.getElementById('uptime').innerText=formatUptime(d.systemUptime||0);
if(d.ntpTime) document.getElementById('ntpTime').innerText=d.ntpTime;

const ws=document.getElementById('wifiStatus');const wi=document.getElementById('wifiIp');
if(d.wifiConnected){ws.innerHTML='✅ '+d.wifiSsid;ws.className='wifi-connected';wi.innerHTML=d.wifiIp}
else if(d.wifiEnabled){ws.innerHTML='⚠️ Connecting';ws.className='wifi-disconnected';wi.innerHTML=''}
else{ws.innerHTML='📡 AP Mode';ws.className='';wi.innerHTML='192.168.4.1'}

const mqttStatusEl=document.getElementById('mqttStatus');
if(d.mqttEnabled){
    if(d.mqttConnected){
        mqttStatusEl.innerHTML='✅ Connected to ' + d.mqttBroker;
        mqttStatusEl.className='mqtt-connected';
    } else {
        mqttStatusEl.innerHTML='⚠️ Disconnected';
        mqttStatusEl.className='mqtt-disconnected';
    }
} else {
    mqttStatusEl.innerHTML='⭕ Disabled';
    mqttStatusEl.className='';
}

const ss=document.getElementById('sensorBadge');
if(!d.useEspnow){
    ss.innerHTML='<span class="led led-gray"></span> ⚪ ESP-NOW Disabled';
    ss.style.background='#f1f5f9';
    ss.style.color='#475569';
} else if(!d.espnowActive){
    ss.innerHTML='<span class="led led-red"></span> ❌ Init Failed';
    ss.style.background='#fee2e2';
} else if(d.sensorHealthy){
    ss.innerHTML='<span class="led led-green"></span> ✅ Connected';
    ss.style.background='#e6f7ec';
} else if(d.sensorWarning){
    ss.innerHTML='<span class="led led-yellow"></span> ⚠️ Weak Signal';
    ss.style.background='#fef3c7';
} else {
    ss.innerHTML='<span class="led led-red"></span> ❌ No Data';
    ss.style.background='#fee2e2';
}

const voltage = d.voltage || 0;
document.getElementById('voltage').innerHTML=voltage.toFixed(1);
document.getElementById('current').innerText=(d.current||0).toFixed(2);
document.getElementById('power').innerText=(d.power||0).toFixed(1);
document.getElementById('energy').innerText=(d.energy||0).toFixed(2);

const voltageDiv = document.getElementById('voltageStatus');
if(voltage < 180 && voltage > 0) voltageDiv.innerHTML='<div class="voltage-warning">⚠️ LOW VOLTAGE: '+voltage.toFixed(1)+'V (Below 180V)</div>';
else if(voltage > 240 && voltage > 0) voltageDiv.innerHTML='<div class="voltage-warning">⚠️ HIGH VOLTAGE: '+voltage.toFixed(1)+'V (Above 240V)</div>';
else if(voltage > 0) voltageDiv.innerHTML='<div class="voltage-ok">✅ Voltage OK: '+voltage.toFixed(1)+'V</div>';
else voltageDiv.innerHTML='';

const overloadCooldown=d.overloadCooldownRemaining||0;
const dryrunCooldown=d.dryrunCooldownRemaining||0;
const lowVoltageCooldown=d.lowVoltageCooldownRemaining||0;
const highVoltageCooldown=d.highVoltageCooldownRemaining||0;
const lockoutRemaining=d.pressureLockoutRemaining||0;
const softStartRemaining=d.softStartRemaining||0;
const inchingRemaining=d.inchingRemaining||0;

if(overloadCooldown>0){
    document.getElementById('timerOverload').style.display='inline-block';
    document.getElementById('overloadTimer').innerText=formatTimeRemaining(overloadCooldown);
}else{document.getElementById('timerOverload').style.display='none';}

if(dryrunCooldown>0){
    document.getElementById('timerDryRun').style.display='inline-block';
    document.getElementById('dryrunTimer').innerText=formatTimeRemaining(dryrunCooldown);
}else{document.getElementById('timerDryRun').style.display='none';}

if(lowVoltageCooldown>0){
    document.getElementById('timerLowVoltage').style.display='inline-block';
    document.getElementById('lowVoltageTimer').innerText=formatTimeRemaining(lowVoltageCooldown);
}else{document.getElementById('timerLowVoltage').style.display='none';}

if(highVoltageCooldown>0){
    document.getElementById('timerHighVoltage').style.display='inline-block';
    document.getElementById('highVoltageTimer').innerText=formatTimeRemaining(highVoltageCooldown);
}else{document.getElementById('timerHighVoltage').style.display='none';}

if(lockoutRemaining>0){
    document.getElementById('timerLockout').style.display='inline-block';
    document.getElementById('lockoutTimer').innerText=formatTimeRemaining(lockoutRemaining);
}else{document.getElementById('timerLockout').style.display='none';}

if(softStartRemaining>0){
    document.getElementById('timerSoftStart').style.display='inline-block';
    document.getElementById('softStartTimer').innerText=softStartRemaining;
}else{document.getElementById('timerSoftStart').style.display='none';}

if(inchingRemaining>0){
    document.getElementById('timerInching').style.display='inline-block';
    document.getElementById('inchingTimer').innerText=formatTimeRemaining(inchingRemaining);
}else{document.getElementById('timerInching').style.display='none';}

const tankCard=document.getElementById('virtualTankCard');
const espnowWarning=document.getElementById('espnowWarning');

if(d.useEspnow && d.espnowActive){
    tankCard.classList.remove('hidden');
    espnowWarning.style.display='none';
    let lvl=Math.min(100,Math.max(0,d.waterLevel||0));
    document.getElementById('level').innerText=Math.floor(lvl);
    document.getElementById('waterFill').style.height=lvl+'%';
    document.getElementById('waterFill').innerText=Math.floor(lvl)+'%';
    document.getElementById('sensorVoltage').innerHTML=(d.batteryVoltage||0).toFixed(2);
    document.getElementById('volume').innerText=(d.volume||0).toFixed(0);
} else if(d.useEspnow && !d.espnowActive){
    tankCard.classList.add('hidden');
    espnowWarning.style.display='block';
    espnowWarning.innerHTML='⚠️ ESP-NOW INIT FAILED - Check peer MAC address';
} else {
    tankCard.classList.add('hidden');
    espnowWarning.style.display='none';
}

document.getElementById('overloadCount').innerText=d.overloadCount||0;
document.getElementById('dryrunCount').innerText=d.dryrunCount||0;
document.getElementById('lowVoltageCount').innerText=d.lowVoltageCount||0;
document.getElementById('highVoltageCount').innerText=d.highVoltageCount||0;

const mt=document.getElementById('modeText');const at=document.getElementById('autoToggle');
if(d.autoMode){mt.innerHTML='🤖 AUTO MODE';mt.style.color='#059669';at.checked=true}
else{mt.innerHTML='👆 MANUAL MODE';mt.style.color='#d97706';at.checked=false}

const pe=document.getElementById('pressureStatus');
if(d.pressureLockoutActive){pe.innerHTML='<span class="led led-red"></span> 🔒 LOCKOUT';pe.style.background='#fee2e2'}
else if(d.pressureLow){pe.innerHTML='<span class="led led-red"></span> ⚠️ LOW PRESSURE';pe.style.background='#fee2e2'}
else{pe.innerHTML='<span class="led led-green"></span> ✅ PRESSURE OK';pe.style.background='#e6f7ec'}

const ld=document.getElementById('lockoutDiv');
if(d.pressureLockoutActive&&d.pressureLockoutRemaining>0){
    ld.style.display='block';
}else{ld.style.display='none'}

const btn=document.getElementById('pumpBtn');
if(d.pressureLockoutActive){btn.innerText="🔒 LOCKOUT";btn.className='pump-btn lockout';btn.disabled=true}
else if(overloadCooldown>0){btn.innerText="⏱️ COOLDOWN "+formatTimeRemaining(overloadCooldown);btn.className='pump-btn cooldown';btn.disabled=true}
else if(dryrunCooldown>0){btn.innerText="💧 DRY COOLDOWN "+formatTimeRemaining(dryrunCooldown);btn.className='pump-btn cooldown';btn.disabled=true}
else if(lowVoltageCooldown>0){btn.innerText="⚡ LOW VOLTAGE "+formatTimeRemaining(lowVoltageCooldown);btn.className='pump-btn cooldown';btn.disabled=true}
else if(highVoltageCooldown>0){btn.innerText="⚡ HIGH VOLTAGE "+formatTimeRemaining(highVoltageCooldown);btn.className='pump-btn cooldown';btn.disabled=true}
else if(softStartRemaining>0){btn.innerText="🌊 SOFT START "+(softStartRemaining/1000).toFixed(1)+"s";btn.className='pump-btn softstart';btn.disabled=true}
else if(inchingRemaining>0){btn.innerText="⏱️ INCHING "+formatTimeRemaining(inchingRemaining);btn.className='pump-btn inching';btn.disabled=false}
else if(d.pumpState){btn.innerText="💧 PUMP ON";btn.className='pump-btn running';btn.disabled=false}
else{btn.innerText="⏹️ PUMP OFF";btn.className='pump-btn';btn.disabled=false}

let r="";
if(d.pressureLockoutActive)r="🔒 Lockout - Pump disabled";
else if(overloadCooldown>0)r="⚠️ Overload cooldown - Pump disabled";
else if(dryrunCooldown>0)r="💧 Dry run cooldown - Pump disabled";
else if(lowVoltageCooldown>0)r="⚡ Low voltage cooldown - Pump disabled";
else if(highVoltageCooldown>0)r="⚡ High voltage cooldown - Pump disabled";
else if(softStartRemaining>0)r="🌊 Soft start - Delaying pump start";
else if(inchingRemaining>0)r="⏱️ Inching mode - Auto stop in "+(inchingRemaining>60?Math.floor(inchingRemaining/60)+"m "+(inchingRemaining%60)+"s":inchingRemaining+"s");
else if(!d.autoMode)r="👆 Manual control - Press button to control pump";
else if(d.pressureLow)r="🔘 Low pressure detected - Pump ON";
else r="🔘 Pressure OK - Pump OFF";
document.getElementById('reason').innerHTML=r;

let warn="";
if(d.overloadProtectionActive)warn+='<div class="protection-badge">⚠️ OVERLOAD ACTIVE</div>';
if(d.dryRunEnabled && d.dryRunActive)warn+='<div class="protection-badge">💧 DRY RUN DETECTED</div>';
if(d.lowVoltageEnabled && d.lowVoltageActive)warn+='<div class="protection-badge">⚡ LOW VOLTAGE DETECTED</div>';
if(d.highVoltageEnabled && d.highVoltageActive)warn+='<div class="protection-badge">⚡ HIGH VOLTAGE DETECTED</div>';
if(d.inInrushPeriod)warn+='<div class="protection-badge">⚡ INRUSH ACTIVE</div>';
document.getElementById('warnings').innerHTML=warn;

const stats=await fetchJSON('/stats');
if(stats){document.getElementById('runtime').innerText=stats.total_runtime||0;document.getElementById('cycles').innerText=stats.pump_cycles||0}
}
setInterval(refresh,1000);window.onload=refresh;
</script>
</body>
</html>
)rawliteral";

// Engineering Mode HTML (same as v3.8, kept for compatibility)
const char engineering_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Engineering v3.9</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);min-height:100vh;padding:20px}
.container{max-width:1200px;margin:0 auto}
.card{background:white;border-radius:20px;padding:25px;margin-bottom:20px;box-shadow:0 10px 30px rgba(0,0,0,0.2)}
h1{text-align:center;color:#333;margin-bottom:5px}
.version{text-align:center;color:#10B981;font-size:12px;margin-bottom:5px}
.nav-buttons{display:flex;gap:10px;margin-bottom:20px;flex-wrap:wrap}
.nav-btn{flex:1;background:#e5e7eb;color:#333;padding:10px;border:none;border-radius:10px;cursor:pointer;font-weight:bold;font-size:14px}
.nav-btn.active{background:#667eea;color:white}
.config-group{margin-bottom:20px}
label{display:block;font-weight:bold;margin-bottom:8px;color:#333;font-size:14px}
input,select{width:100%;padding:12px;border:1px solid #ddd;border-radius:8px;font-size:14px}
button{background:#667eea;color:white;border:none;padding:12px 24px;border-radius:10px;cursor:pointer;font-weight:bold;margin-top:10px;margin-right:10px;font-size:14px}
button.danger{background:#EF4444}
button.success{background:#10b981}
.info-text{font-size:12px;color:#666;margin-top:5px}
.simple-link{text-align:center;margin-top:20px;padding:10px;background:#e0e7ff;border-radius:10px}
.simple-link a{color:#4338ca;text-decoration:none;font-weight:bold}
.save-status{position:fixed;bottom:20px;right:20px;background:#10b981;color:white;padding:10px 20px;border-radius:10px;display:none;font-weight:bold}
.checkbox-large{display:flex;align-items:center;padding:12px;background:#f0fdf4;border-radius:12px;border:2px solid #22c55e;cursor:pointer;margin-bottom:15px}
.checkbox-large:hover{background:#dcfce7}
.checkbox-large input{width:24px;height:24px;margin-right:15px;cursor:pointer}
.checkbox-large label{flex:1;margin-bottom:0;font-size:16px;cursor:pointer}
h3{margin:20px 0 15px;padding-bottom:8px;border-bottom:2px solid #667eea;color:#333}
.lockout-preset{display:inline-block;background:#e0e7ff;padding:5px 10px;border-radius:20px;margin:5px;cursor:pointer;font-size:12px}
.lockout-preset:hover{background:#c7d2fe}
.wifi-scan-result{max-height:300px;overflow-y:auto;border:1px solid #ddd;border-radius:8px;margin-top:10px}
.wifi-network{padding:12px;border-bottom:1px solid #eee;cursor:pointer;display:flex;justify-content:space-between;align-items:center}
.wifi-network:hover{background:#f0f0f0}
.reboot-countdown{background:#fef3c7;padding:10px;border-radius:10px;margin-top:15px;text-align:center;font-weight:bold}
.spinner{display:inline-block;width:14px;height:14px;border:2px solid #e2e8f0;border-top-color:#667eea;border-radius:50%;animation:spin 0.6s linear infinite;margin-right:8px;vertical-align:middle}
@keyframes spin{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<div class="container">
<div class="card">
<h1>🔧 Engineering v3.9</h1>
<div class="version">ASYNC MQTT | NTP TIME (GMT+7) | IMPROVED WIFI SCAN</div>
<div class="nav-buttons">
<button class="nav-btn active" onclick="showSection('lockout')">🔒 Lockout</button>
<button class="nav-btn" onclick="showSection('wifi')">📡 WiFi</button>
<button class="nav-btn" onclick="showSection('mqtt')">📨 MQTT</button>
<button class="nav-btn" onclick="showSection('protection')">🛡️ Protection</button>
<button class="nav-btn" onclick="showSection('inching')">⏱️ Inching</button>
<button class="nav-btn" onclick="showSection('inrush')">⚡ Inrush</button>
<button class="nav-btn" onclick="showSection('softstart')">🌊 Soft Start</button>
<button class="nav-btn" onclick="showSection('pressure')">🔘 Pressure</button>
<button class="nav-btn" onclick="showSection('system')">⚙️ System</button>
</div>

<div id="lockoutSection">
<h3>🔒 Pressure Lockout</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#fee2e2;border-radius:10px">Prevents rapid pump cycling when pressure fluctuates.</div>
<div class="config-group"><label>Lockout (hours)</label><input type="number" id="lockoutHours" min="0" max="72" step="0.5"></div>
<div><strong>Quick Presets:</strong> <span class="lockout-preset" onclick="setLockout(0.083)">5min</span> <span class="lockout-preset" onclick="setLockout(0.5)">30min</span> <span class="lockout-preset" onclick="setLockout(1)">1h</span> <span class="lockout-preset" onclick="setLockout(2)">2h</span> <span class="lockout-preset" onclick="setLockout(4)">4h</span> <span class="lockout-preset" onclick="setLockout(8)">8h</span> <span class="lockout-preset" onclick="setLockout(12)">12h</span> <span class="lockout-preset" onclick="setLockout(24)">1d</span> <span class="lockout-preset" onclick="setLockout(48)">2d</span> <span class="lockout-preset" onclick="setLockout(72)">3d</span></div>
<button onclick="saveLockout()">💾 Save</button>
</div>

<div id="wifiSection" style="display:none">
<h3>📡 WiFi Client</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#dbeafe;border-radius:10px">AP always available: SmartPump-XXXX / 12345678</div>
<div class="checkbox-large" onclick="toggleCheckbox('wifiEnabled')"><input type="checkbox" id="wifiEnabled" onclick="event.stopPropagation()"><label>Enable WiFi Client</label></div>
<div class="config-group"><label>SSID</label><input type="text" id="wifiSsid" placeholder="Your WiFi"></div>
<div class="config-group"><label>Password</label><input type="password" id="wifiPassword" placeholder="Password"></div>
<button onclick="saveWifi()">💾 Save & Reboot (5s countdown)</button>
<button onclick="scanWiFi()" class="success" id="scanBtn">📡 Scan Networks</button>
<div id="wifiScanResult" class="wifi-scan-result" style="display:none"></div>
<div id="wifiScanning" style="display:none;text-align:center;padding:20px"><span class="spinner"></span> Scanning...</div>
<div id="rebootCountdown" class="reboot-countdown" style="display:none">🔄 Rebooting in <span id="countdownSeconds">5</span> seconds... <button onclick="cancelReboot()" style="background:#ef4444;color:white;border:none;padding:4px 12px;border-radius:20px;margin-left:10px;cursor:pointer">Cancel</button></div>
</div>

<div id="mqttSection" style="display:none">
<h3>📨 MQTT Client (Async + NTP Timestamps)</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#e0f2fe;border-radius:10px">
Non-blocking MQTT with ISO8601 timestamps (GMT+7 Bangkok time).<br><br>
📡 Topics: pump/[device]/status, pump/[device]/telemetry, pump/[device]/event, pump/[device]/command<br>
⚡ Commands: ON, OFF, AUTO, MANUAL, RESET_LOCKOUT, GET_STATUS
</div>
<div class="checkbox-large" onclick="toggleCheckbox('mqttEnabled')"><input type="checkbox" id="mqttEnabled" onclick="event.stopPropagation()"><label>Enable MQTT Client</label></div>
<div class="config-group"><label>Broker</label><input type="text" id="mqttBroker" placeholder="mqtt.dashboard.com"></div>
<div class="config-group"><label>Port</label><input type="number" id="mqttPort" placeholder="1883"></div>
<div class="config-group"><label>Base Topic</label><input type="text" id="mqttBaseTopic" placeholder="pump"></div>
<div class="checkbox-large" onclick="toggleCheckbox('mqttAuth')"><input type="checkbox" id="mqttAuth" onclick="event.stopPropagation()"><label>Use Authentication</label></div>
<div id="mqttAuthFields" style="display:none">
<div class="config-group"><label>Username</label><input type="text" id="mqttUsername" placeholder="username"></div>
<div class="config-group"><label>Password</label><input type="password" id="mqttPassword" placeholder="password"></div>
</div>
<div class="config-group"><label>Client ID (optional)</label><input type="text" id="mqttClientId" placeholder="auto-generated"></div>
<button onclick="saveMqttSettings()" class="success">💾 Save MQTT</button>
</div>

<div id="protectionSection" style="display:none">
<h3>💧 Dry Run Protection (SAFETY PRIORITY)</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#d1fae5;border-radius:10px">When power drops below threshold for the set time, pump stops INSTANTLY and cancels all timers.</div>
<div class="checkbox-large" onclick="toggleCheckbox('dryRunEnabled')"><input type="checkbox" id="dryRunEnabled" onclick="event.stopPropagation()"><label>Enable Dry Run</label></div>
<div class="config-group"><label>Detection Time (sec)</label><input type="number" id="dryRunProtection" min="3" max="300" value="3"><div class="info-text">Pump stops instantly when low power persists this long</div></div>
<div class="config-group"><label>Min Power (W)</label><input type="number" id="minPower" min="0" max="3500" step="any"><div class="info-text">Below this power = dry run condition</div></div>
<div class="config-group"><label>Cooldown (sec)</label><input type="number" id="dryrunCooldown" min="0" max="300"></div>

<h3>⚡ Voltage Protection (SAFETY PRIORITY)</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#fee2e2;border-radius:10px">When voltage goes out of safe range, pump stops and cancels all timers.</div>

<div class="checkbox-large" onclick="toggleCheckbox('lowVoltageEnabled')"><input type="checkbox" id="lowVoltageEnabled" onclick="event.stopPropagation()"><label>Enable Low Voltage Protection</label></div>
<div class="config-group"><label>Min Voltage (V)</label><input type="number" id="minVoltage" min="100" max="260" step="5" value="180"><div class="info-text">Pump stops when voltage drops below this value (instant if &lt;160V)</div></div>

<div class="checkbox-large" onclick="toggleCheckbox('highVoltageEnabled')"><input type="checkbox" id="highVoltageEnabled" onclick="event.stopPropagation()"><label>Enable High Voltage Protection</label></div>
<div class="config-group"><label>Max Voltage (V)</label><input type="number" id="maxVoltage" min="100" max="300" step="5" value="240"><div class="info-text">Pump stops when voltage exceeds this value (instant if &gt;250V)</div></div>

<div class="config-group"><label>Voltage Cooldown (sec)</label><input type="number" id="voltageCooldown" min="0" max="300" value="30"><div class="info-text">Time before pump can restart after voltage protection event</div></div>

<h3>⚠️ Overload Protection</h3>
<div class="checkbox-large" onclick="toggleCheckbox('loadProtectionToggle')"><input type="checkbox" id="loadProtectionToggle" onclick="event.stopPropagation()"><label>Enable Overload Protection</label></div>
<div class="config-group"><label>Max Power (W)</label><input type="number" id="maxPower" min="10" max="3500"><div class="info-text">Pump stops when power exceeds this value</div></div>
<div class="config-group"><label>Overload Cooldown (sec)</label><input type="number" id="overloadCooldown" min="0" max="300"><div class="info-text">Time before pump can restart after overload event</div></div>

<button onclick="saveProtection()">💾 Save All Protection Settings</button>
</div>

<div id="inchingSection" style="display:none">
<h3>⏱️ Inching Mode</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#ede9fe;border-radius:10px">Timer will be CANCELLED if any protection is triggered.</div>
<div class="checkbox-large" onclick="toggleCheckbox('inchingToggle')"><input type="checkbox" id="inchingToggle" onclick="event.stopPropagation()"><label>Enable Inching</label></div>
<div class="config-group"><label>Duration (min)</label><input type="number" id="inchingDuration" min="1" max="60"></div>
<button onclick="saveInching()">💾 Save</button>
</div>

<div id="inrushSection" style="display:none">
<h3>⚡ Inrush Current</h3>
<div class="checkbox-large" onclick="toggleCheckbox('inrushEnabled')"><input type="checkbox" id="inrushEnabled" onclick="event.stopPropagation()"><label>Enable Inrush Protection</label></div>
<div class="config-group"><label>Tolerance (ms)</label><input type="range" id="inrushToleranceSlider" min="500" max="5000" step="100" oninput="updateInrushValue(this.value)"><input type="number" id="inrushTolerance" min="500" max="5000" step="100" oninput="updateInrushSlider(this.value)"></div>
<button onclick="saveInrush()">💾 Save</button>
</div>

<div id="softstartSection" style="display:none">
<h3>🌊 Soft Start</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#cffafe;border-radius:10px">Delay will be CANCELLED if any protection is triggered.</div>
<div class="checkbox-large" onclick="toggleCheckbox('softStartToggle')"><input type="checkbox" id="softStartToggle" onclick="event.stopPropagation()"><label>Enable Soft Start</label></div>
<div class="config-group"><label>Delay (ms)</label><input type="number" id="softStartDelay" min="0" max="10000" step="100"></div>
<button onclick="saveSoftStart()">💾 Save</button>
</div>

<div id="pressureSection" style="display:none">
<h3>🔘 Pressure Switch</h3>
<div class="checkbox-large" onclick="toggleCheckbox('pressureInverted')"><input type="checkbox" id="pressureInverted" onclick="event.stopPropagation()"><label>Invert Logic</label></div>
<button onclick="savePressure()">💾 Save</button>
</div>

<div id="systemSection" style="display:none">
<h3>⚙️ System</h3>
<div class="config-group"><label>Device Name</label><input type="text" id="hostname" placeholder="s31-pump"></div>
<div class="config-group"><label>ESP-NOW Peer MAC</label><input type="text" id="peerMac" placeholder="FF:FF:FF:FF:FF:FF"></div>
<div class="config-group"><label>ESP-NOW Channel</label><input type="number" id="espnowChannel" min="1" max="13"></div>
<div class="checkbox-large" onclick="toggleCheckbox('useEspnow')"><input type="checkbox" id="useEspnow" onclick="event.stopPropagation()"><label>Enable ESP-NOW</label></div>
<div class="checkbox-large" onclick="toggleCheckbox('autoReturnToggle')"><input type="checkbox" id="autoReturnToggle" onclick="event.stopPropagation()"><label>Auto-Return to AUTO (10 min)</label></div>
<button onclick="saveSystem()">💾 Save & Reboot (5s countdown)</button>
<button class="danger" onclick="factoryReset()">⚠️ Factory Reset</button>
<button class="danger" onclick="reboot()">🔄 Reboot</button>
</div>

<div class="simple-link"><a href="/">← Back</a></div>
</div>
</div>
<div id="saveStatus" class="save-status">✓ Saved!</div>

<script>
let countdownInterval=null;
let scanRetryCount=0;

function toggleCheckbox(id){const cb=document.getElementById(id);if(cb)cb.checked=!cb.checked}
function showSaveStatus(){const s=document.getElementById('saveStatus');s.style.display='block';setTimeout(()=>s.style.display='none',2000)}
function setLockout(h){document.getElementById('lockoutHours').value=h}
async function saveLockout(){const h=parseFloat(document.getElementById('lockoutHours').value);const r=await fetch('/config/lockout',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pressure_lockout_hours:h})});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveWifi(){const e=document.getElementById('wifiEnabled').checked;const s=document.getElementById('wifiSsid').value;const p=document.getElementById('wifiPassword').value;const r=await fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({wifi_enabled:e,wifi_ssid:s,wifi_password:p})});if(r.ok){showSaveStatus();startCountdown();}else alert("Failed")}
async function saveMqttSettings(){const s={mqtt_enabled:document.getElementById('mqttEnabled').checked,mqtt_broker:document.getElementById('mqttBroker').value,mqtt_port:parseInt(document.getElementById('mqttPort').value),mqtt_base_topic:document.getElementById('mqttBaseTopic').value,mqtt_use_auth:document.getElementById('mqttAuth').checked,mqtt_username:document.getElementById('mqttUsername').value,mqtt_password:document.getElementById('mqttPassword').value,mqtt_client_id:document.getElementById('mqttClientId').value};const r=await fetch('/config/mqtt',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});if(r.ok){showSaveStatus();alert("MQTT Saved! Rebooting...");setTimeout(()=>location.reload(),2000)}else alert("Failed")}
function updateInrushValue(v){document.getElementById('inrushTolerance').value=v;document.getElementById('inrushToleranceSlider').value=v}
function updateInrushSlider(v){document.getElementById('inrushToleranceSlider').value=v;document.getElementById('inrushTolerance').value=v}
async function saveProtection(){const s={dry_run_enabled:document.getElementById('dryRunEnabled').checked,dry_run_protection:parseInt(document.getElementById('dryRunProtection').value),min_power:parseFloat(document.getElementById('minPower').value),dryrun_cooldown_seconds:parseInt(document.getElementById('dryrunCooldown').value),pump_load_protection_enabled:document.getElementById('loadProtectionToggle').checked,max_power_threshold:parseFloat(document.getElementById('maxPower').value),overload_cooldown_seconds:parseInt(document.getElementById('overloadCooldown').value),low_voltage_enabled:document.getElementById('lowVoltageEnabled').checked,min_voltage_threshold:parseFloat(document.getElementById('minVoltage').value),high_voltage_enabled:document.getElementById('highVoltageEnabled').checked,max_voltage_threshold:parseFloat(document.getElementById('maxVoltage').value),low_voltage_cooldown_seconds:parseInt(document.getElementById('voltageCooldown').value)};const r=await fetch('/config/protection',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveInching(){const d=parseInt(document.getElementById('inchingDuration').value);const s={inching_enabled:document.getElementById('inchingToggle').checked,inching_duration_minutes:d};const r=await fetch('/config/inching',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveInrush(){const t=parseInt(document.getElementById('inrushTolerance').value);const e=document.getElementById('inrushEnabled').checked;const r=await fetch('/config/inrush',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({inrush_enabled:e,inrush_tolerance_ms:t})});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1500)}else alert("Failed")}
async function saveSoftStart(){const s={soft_start_enabled:document.getElementById('softStartToggle').checked,soft_start_delay_ms:parseInt(document.getElementById('softStartDelay').value)};await fetch('/config/softstart',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();setTimeout(()=>location.reload(),1000)}
async function savePressure(){const s={pressure_switch_inverted:document.getElementById('pressureInverted').checked};await fetch('/config/pressure',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();setTimeout(()=>location.reload(),1000)}
async function saveSystem(){const s={hostname:document.getElementById('hostname').value,peer_mac:document.getElementById('peerMac').value,espnow_channel:parseInt(document.getElementById('espnowChannel').value),use_espnow:document.getElementById('useEspnow').checked,auto_return_enabled:document.getElementById('autoReturnToggle').checked};await fetch('/config/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();startCountdown();}
async function factoryReset(){if(confirm('FACTORY RESET? All settings lost!')){await fetch('/factoryreset')}}
async function reboot(){if(confirm('Reboot device?')){await fetch('/reboot')}}
function startCountdown(){let countdown=5;const countdownDiv=document.getElementById('rebootCountdown');const countdownSpan=document.getElementById('countdownSeconds');if(countdownDiv){countdownDiv.style.display='block';if(countdownInterval)clearInterval(countdownInterval);countdownInterval=setInterval(()=>{countdown--;if(countdown>=0){countdownSpan.innerText=countdown;}if(countdown<0){clearInterval(countdownInterval);window.location.href='/';}},1000);}}
function cancelReboot(){if(countdownInterval){clearInterval(countdownInterval);const countdownDiv=document.getElementById('rebootCountdown');if(countdownDiv)countdownDiv.style.display='none';window.location.href='/';}}
async function loadSettings(){const r=await fetch('/config');const d=await r.json();document.getElementById('lockoutHours').value=d.pressure_lockout_hours||2;document.getElementById('wifiEnabled').checked=d.wifi_sta_enabled||false;document.getElementById('wifiSsid').value=d.wifi_ssid||'';document.getElementById('wifiPassword').value=d.wifi_password||'';document.getElementById('mqttEnabled').checked=d.mqtt_enabled||false;document.getElementById('mqttBroker').value=d.mqtt_broker||'mqtt.dashboard.com';document.getElementById('mqttPort').value=d.mqtt_port||1883;document.getElementById('mqttBaseTopic').value=d.mqtt_base_topic||'pump';document.getElementById('mqttAuth').checked=d.mqtt_use_auth||false;document.getElementById('mqttUsername').value=d.mqtt_username||'';document.getElementById('mqttPassword').value=d.mqtt_password||'';document.getElementById('mqttClientId').value=d.mqtt_client_id||'';const mqttAuthFields=document.getElementById('mqttAuthFields');if(mqttAuthFields)mqttAuthFields.style.display=document.getElementById('mqttAuth').checked?'block':'none';document.getElementById('dryRunEnabled').checked=d.dry_run_enabled;document.getElementById('dryRunProtection').value=d.dry_run_protection||3;document.getElementById('minPower').value=d.min_power;document.getElementById('dryrunCooldown').value=d.dryrun_cooldown_seconds;document.getElementById('loadProtectionToggle').checked=d.pump_load_protection_enabled;document.getElementById('maxPower').value=d.max_power_threshold;document.getElementById('overloadCooldown').value=d.overload_cooldown_seconds;document.getElementById('lowVoltageEnabled').checked=d.low_voltage_enabled!==undefined?d.low_voltage_enabled:true;document.getElementById('minVoltage').value=d.min_voltage_threshold||180;document.getElementById('highVoltageEnabled').checked=d.high_voltage_enabled!==undefined?d.high_voltage_enabled:true;document.getElementById('maxVoltage').value=d.max_voltage_threshold||240;document.getElementById('voltageCooldown').value=d.low_voltage_cooldown_seconds||30;document.getElementById('inrushEnabled').checked=d.inrush_enabled;document.getElementById('inrushTolerance').value=d.inrush_tolerance_ms;document.getElementById('inrushToleranceSlider').value=d.inrush_tolerance_ms;document.getElementById('inchingToggle').checked=d.inching_enabled;document.getElementById('inchingDuration').value=d.inching_duration_minutes;document.getElementById('softStartToggle').checked=d.soft_start_enabled;document.getElementById('softStartDelay').value=d.soft_start_delay_ms;document.getElementById('pressureInverted').checked=d.pressure_switch_inverted;document.getElementById('autoReturnToggle').checked=d.auto_return_enabled;document.getElementById('hostname').value=d.hostname||'s31-pump';document.getElementById('peerMac').value=d.peer_mac||'FF:FF:FF:FF:FF:FF';document.getElementById('espnowChannel').value=d.espnow_channel;document.getElementById('useEspnow').checked=d.use_espnow}

// Improved WiFi Scan for Engineering Mode
async function scanWiFi() {
    const scanBtn = event.target;
    const rd = document.getElementById('wifiScanResult');
    const sd = document.getElementById('wifiScanning');
    
    if(!rd || !sd) return;
    
    scanBtn.disabled = true;
    scanBtn.innerHTML = '<span class="spinner"></span> Scanning...';
    
    rd.style.display = 'none';
    sd.style.display = 'block';
    sd.innerHTML = '<div><span class="spinner"></span> Scanning WiFi networks...<br><span style="font-size:12px">Please wait 3-5 seconds</span></div>';
    
    scanRetryCount = 0;
    
    function checkScan() {
        fetch('/scan_wifi')
            .then(res => res.json())
            .then(data => {
                if(data.scanning === true) {
                    scanRetryCount++;
                    const dots = '.'.repeat(scanRetryCount % 4);
                    sd.innerHTML = `<div><span class="spinner"></span> Scanning WiFi networks${dots}<br><span style="font-size:12px">Please wait...</span></div>`;
                    
                    if(scanRetryCount < 10) {
                        setTimeout(checkScan, 1000);
                    } else {
                        sd.style.display = 'none';
                        rd.style.display = 'block';
                        rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan timeout. Please try again.</div>';
                        scanBtn.disabled = false;
                        scanBtn.innerHTML = '📡 Scan Networks';
                    }
                } else if(data.error) {
                    sd.style.display = 'none';
                    rd.style.display = 'block';
                    rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan failed. Please try again.</div>';
                    scanBtn.disabled = false;
                    scanBtn.innerHTML = '📡 Scan Networks';
                } else if(Array.isArray(data)) {
                    sd.style.display = 'none';
                    rd.style.display = 'block';
                    
                    if(data.length === 0) {
                        rd.innerHTML = '<div style="padding:20px">📡 No WiFi networks found</div>';
                    } else {
                        let html = '<div style="font-weight:bold;padding:10px;background:#e2e8f0;border-radius:10px;margin-bottom:10px">📡 Found ' + data.length + ' networks:</div>';
                        for(let i = 0; i < data.length; i++) {
                            let signalIcon = '';
                            let signalColor = '';
                            if(data[i].rssi > -50) { signalIcon = '📶 Excellent'; signalColor = '#10b981'; }
                            else if(data[i].rssi > -70) { signalIcon = '📶 Good'; signalColor = '#f59e0b'; }
                            else { signalIcon = '📶 Fair'; signalColor = '#ef4444'; }
                            
                            let encryptionIcon = data[i].encryption > 0 ? '🔒' : '🔓';
                            
                            html += `<div class="wifi-network" onclick="document.getElementById('wifiSsid').value='${data[i].ssid.replace(/'/g, "\\'")}'; rd.style.display='none'">
                                        <div><strong>${data[i].ssid}</strong><br><span style="font-size:11px;color:${signalColor}">${signalIcon}</span></div>
                                        <div><span style="font-size:12px">${data[i].rssi}dBm</span> ${encryptionIcon}</div>
                                     </div>`;
                        }
                        rd.innerHTML = html;
                    }
                    scanBtn.disabled = false;
                    scanBtn.innerHTML = '📡 Scan Networks';
                }
            })
            .catch(err => {
                console.error('Scan error:', err);
                sd.style.display = 'none';
                rd.style.display = 'block';
                rd.innerHTML = '<div style="padding:20px;color:red">❌ Scan error. Please try again.</div>';
                scanBtn.disabled = false;
                scanBtn.innerHTML = '📡 Scan Networks';
            });
    }
    
    checkScan();
}

function showSection(s){const sections=['lockout','wifi','mqtt','protection','inching','inrush','softstart','pressure','system'];sections.forEach(sec=>{const el=document.getElementById(sec+'Section');if(el)el.style.display=sec===s?'block':'none';});document.querySelectorAll('.nav-btn').forEach(btn=>btn.classList.remove('active'));event.target.classList.add('active')}
loadSettings();
</script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     WEB SERVER HANDLERS
// ================================================================================================

void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", simple_html); });
  server.on("/engmode", HTTP_GET, []() { server.send_P(200, "text/html", engineering_html); });
  
  server.onNotFound([]() {
    if (ENABLE_CAPTIVE_PORTAL) {
      server.sendHeader("Location", "http://192.168.4.1/", true);
      server.send(302, "text/plain", "Redirecting...");
    } else {
      server.send(404, "text/plain", "Not found");
    }
  });
  
  server.on("/data", HTTP_GET, []() {
    updatePowerQuality();
    readPressureSwitch();
    StaticJsonDocument<1200> doc;
    doc["waterLevel"] = currentWaterLevel;
    doc["volume"] = currentVolume;
    doc["batteryVoltage"] = batteryVoltage;
    doc["power"] = s31.getPower();
    doc["voltage"] = s31.getVoltage();
    doc["current"] = s31.getCurrent();
    doc["energy"] = s31.getEnergy();
    doc["pumpState"] = s31.getRelayState();
    doc["autoMode"] = auto_mode;
    doc["pressureLow"] = pressureSwitch.pressureLow;
    doc["pressureLockoutActive"] = pressureLockout.active;
    doc["pressureLockoutRemaining"] = getLockoutRemainingSeconds();
    doc["softStartActive"] = softStart.delayActive;
    if (softStart.delayActive) doc["softStartRemaining"] = getSoftStartRemainingMs();
    doc["overloadProtectionActive"] = overload.active;
    doc["overloadCooldownRemaining"] = getOverloadCooldownRemaining();
    doc["dryrunCooldownRemaining"] = getDryRunCooldownRemaining();
    doc["dryRunActive"] = (dry_run_enabled && dryRun.lowPowerStartTime != 0);
    doc["dryRunEnabled"] = dry_run_enabled;
    doc["lowVoltageCooldownRemaining"] = getLowVoltageCooldownRemaining();
    doc["lowVoltageActive"] = (low_voltage_enabled && lowVoltage.lowVoltageDetected);
    doc["lowVoltageEnabled"] = low_voltage_enabled;
    doc["highVoltageCooldownRemaining"] = getHighVoltageCooldownRemaining();
    doc["highVoltageActive"] = (high_voltage_enabled && highVoltage.highVoltageDetected);
    doc["highVoltageEnabled"] = high_voltage_enabled;
    doc["overloadCount"] = overload.overloadCount;
    doc["dryrunCount"] = dryRun.events;
    doc["lowVoltageCount"] = lowVoltage.events;
    doc["highVoltageCount"] = highVoltage.events;
    doc["systemUptime"] = (millis() - systemStartTime) / 1000;
    doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
    doc["wifiEnabled"] = wifi_sta_enabled;
    doc["wifiSsid"] = String(wifi_ssid);
    doc["wifiIp"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";
    doc["espnowActive"] = espnow_initialized;
    doc["useEspnow"] = use_espnow;
    doc["inchingActive"] = inching.active;
    if (inching.active) {
      uint32_t elapsed = millis() - inching.startTime;
      uint32_t remainingMs = (inching.duration > elapsed) ? (inching.duration - elapsed) : 0;
      doc["inchingRemaining"] = remainingMs / 1000;
    }
    doc["inInrushPeriod"] = (inrush_enabled && overload.inrushActive);
    doc["sensorHealthy"] = use_espnow && espnow_initialized && !sensorIsDead && (millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
    doc["sensorWarning"] = use_espnow && espnow_initialized && !sensorIsDead && (millis() - lastEspNowData >= ESP_NOW_DATA_TIMEOUT);
    doc["mqttEnabled"] = mqtt_enabled;
    doc["mqttConnected"] = mqttClient.connected();
    doc["mqttBroker"] = String(mqtt_broker);
    doc["ntpSynced"] = ntpSynced;
    doc["ntpTime"] = getFormattedTimestamp();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/config", HTTP_GET, []() {
    StaticJsonDocument<1024> doc;
    doc["auto_mode"] = auto_mode;
    doc["auto_return_enabled"] = auto_return_enabled;
    doc["dry_run_enabled"] = dry_run_enabled;
    doc["dry_run_protection"] = pump_protection_time;
    doc["min_power"] = min_power_threshold;
    doc["dryrun_cooldown_seconds"] = dryrun_cooldown_seconds;
    doc["pump_load_protection_enabled"] = pump_load_protection_enabled;
    doc["max_power_threshold"] = max_power_threshold;
    doc["overload_cooldown_seconds"] = overload_cooldown_seconds;
    doc["inrush_enabled"] = inrush_enabled;
    doc["inrush_tolerance_ms"] = inrush_tolerance_ms;
    doc["inching_enabled"] = inching_enabled;
    doc["inching_duration_minutes"] = inching_duration_minutes;
    doc["soft_start_enabled"] = soft_start_enabled;
    doc["soft_start_delay_ms"] = soft_start_delay_ms;
    doc["use_espnow"] = use_espnow;
    doc["peer_mac"] = String(peer_mac_str);
    doc["espnow_channel"] = espnow_channel;
    doc["hostname"] = String(hostname);
    doc["pressure_switch_inverted"] = pressure_switch_inverted;
    doc["pressure_lockout_hours"] = pressure_lockout_hours;
    doc["wifi_sta_enabled"] = wifi_sta_enabled;
    doc["wifi_ssid"] = String(wifi_ssid);
    doc["mqtt_enabled"] = mqtt_enabled;
    doc["mqtt_broker"] = String(mqtt_broker);
    doc["mqtt_port"] = mqtt_port;
    doc["mqtt_base_topic"] = String(mqtt_base_topic);
    doc["mqtt_use_auth"] = mqtt_use_auth;
    doc["mqtt_username"] = String(mqtt_username);
    doc["mqtt_client_id"] = String(mqtt_client_id);
    doc["low_voltage_enabled"] = low_voltage_enabled;
    doc["min_voltage_threshold"] = min_voltage_threshold;
    doc["high_voltage_enabled"] = high_voltage_enabled;
    doc["max_voltage_threshold"] = max_voltage_threshold;
    doc["low_voltage_cooldown_seconds"] = low_voltage_cooldown_seconds;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/reset_stats", HTTP_GET, []() {
    String type = server.arg("type");
    if (type == "all") {
      resetStatistics(true);
    } else if (type == "runtime") {
      resetStatistics(false);
    } else {
      resetStatistics(false);
    }
    
    StaticJsonDocument<128> doc;
    doc["status"] = "ok";
    doc["message"] = "Statistics reset successfully";
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Improved WiFi scan endpoint
  server.on("/scan_wifi", HTTP_GET, []() {
    handleWiFiScan();
  });
  
  server.on("/config/mqtt", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("mqtt_enabled")) mqtt_enabled = doc["mqtt_enabled"];
      if (doc.containsKey("mqtt_broker")) { String broker = doc["mqtt_broker"].as<String>(); broker.toCharArray(mqtt_broker, sizeof(mqtt_broker)); }
      if (doc.containsKey("mqtt_port")) mqtt_port = doc["mqtt_port"].as<uint16_t>();
      if (doc.containsKey("mqtt_base_topic")) { String topic = doc["mqtt_base_topic"].as<String>(); topic.toCharArray(mqtt_base_topic, sizeof(mqtt_base_topic)); }
      if (doc.containsKey("mqtt_use_auth")) mqtt_use_auth = doc["mqtt_use_auth"];
      if (doc.containsKey("mqtt_username")) { String user = doc["mqtt_username"].as<String>(); user.toCharArray(mqtt_username, sizeof(mqtt_username)); }
      if (doc.containsKey("mqtt_password")) { String pwd = doc["mqtt_password"].as<String>(); pwd.toCharArray(mqtt_password, sizeof(mqtt_password)); }
      if (doc.containsKey("mqtt_client_id")) { String cid = doc["mqtt_client_id"].as<String>(); cid.toCharArray(mqtt_client_id, sizeof(mqtt_client_id)); }
      saveConfigToFile();
      initMqttTopics();
      if (mqtt_enabled && WiFi.status() == WL_CONNECTED) { if (mqttClient.connected()) mqttClient.disconnect(); connectToMqtt(); }
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/lockout", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<64> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("pressure_lockout_hours")) {
        float h = doc["pressure_lockout_hours"].as<float>();
        if (h < 0) h = 0;
        if (h > 0 && h < (MIN_PRESSURE_LOCKOUT_MINUTES / 60.0)) h = MIN_PRESSURE_LOCKOUT_MINUTES / 60.0;
        if (h > MAX_PRESSURE_LOCKOUT_HOURS) h = MAX_PRESSURE_LOCKOUT_HOURS;
        pressure_lockout_hours = h;
        saveConfigToFile();
      }
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/wifi", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("wifi_enabled")) wifi_sta_enabled = doc["wifi_enabled"];
      if (doc.containsKey("wifi_ssid")) { String ssid = doc["wifi_ssid"].as<String>(); ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid)); }
      if (doc.containsKey("wifi_password")) { String pwd = doc["wifi_password"].as<String>(); pwd.toCharArray(wifi_password, sizeof(wifi_password)); }
      saveConfigToFile();
      if (wifi_sta_enabled && strlen(wifi_ssid) > 0) {
        connectToWiFi();
        rebootWithCountdown();
      }
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/reset_lockout", HTTP_GET, []() { resetPressureLockout(); server.send(200, "text/plain", "OK"); });
  server.on("/mode", HTTP_GET, []() {
    if (server.hasArg("mode")) {
      if (server.arg("mode") == "auto") switchToAutoMode();
      else switchToManualMode();
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/toggle", HTTP_GET, []() {
    if (!auto_mode) {
      if (pressureLockout.active) { server.send(403, "text/plain", "Lockout active"); return; }
      if (softStart.delayActive) { server.send(403, "text/plain", "Soft start active"); return; }
      if (millis() < overload.cooldownUntil) { server.send(403, "text/plain", "Overload cooldown"); return; }
      if (millis() < dryRun.cooldownUntil) { server.send(403, "text/plain", "Dry run cooldown"); return; }
      if (millis() < lowVoltage.cooldownUntil) { server.send(403, "text/plain", "Low voltage cooldown"); return; }
      if (millis() < highVoltage.cooldownUntil) { server.send(403, "text/plain", "High voltage cooldown"); return; }
      bool newState = !s31.getRelayState();
      s31.setRelay(newState);
      button.lastManualModeTime = millis();
      if (newState) { cancelInchingTimer(); cancelSoftStartDelay(); }
      server.send(200, "text/plain", "OK");
    } else server.send(403, "text/plain", "In AUTO mode");
  });
  
  server.on("/config/protection", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<384> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("dry_run_enabled")) dry_run_enabled = doc["dry_run_enabled"];
      if (doc.containsKey("dry_run_protection")) pump_protection_time = constrain(doc["dry_run_protection"].as<uint32_t>(), 3, 300);
      if (doc.containsKey("min_power")) min_power_threshold = constrain(doc["min_power"].as<float>(), 0.0, 3500.0);
      if (doc.containsKey("dryrun_cooldown_seconds")) dryrun_cooldown_seconds = constrain(doc["dryrun_cooldown_seconds"].as<uint32_t>(), 0, 300);
      if (doc.containsKey("pump_load_protection_enabled")) pump_load_protection_enabled = doc["pump_load_protection_enabled"];
      if (doc.containsKey("max_power_threshold")) max_power_threshold = constrain(doc["max_power_threshold"].as<float>(), 10.0, 3500.0);
      if (doc.containsKey("overload_cooldown_seconds")) overload_cooldown_seconds = constrain(doc["overload_cooldown_seconds"].as<uint32_t>(), 0, 300);
      if (doc.containsKey("low_voltage_enabled")) low_voltage_enabled = doc["low_voltage_enabled"];
      if (doc.containsKey("min_voltage_threshold")) min_voltage_threshold = constrain(doc["min_voltage_threshold"].as<float>(), MIN_VOLTAGE_THRESHOLD, DEFAULT_MAX_VOLTAGE);
      if (doc.containsKey("high_voltage_enabled")) high_voltage_enabled = doc["high_voltage_enabled"];
      if (doc.containsKey("max_voltage_threshold")) max_voltage_threshold = constrain(doc["max_voltage_threshold"].as<float>(), DEFAULT_MIN_VOLTAGE, MAX_VOLTAGE_THRESHOLD);
      if (doc.containsKey("low_voltage_cooldown_seconds")) {
        uint32_t cd = constrain(doc["low_voltage_cooldown_seconds"].as<uint32_t>(), 0, 300);
        low_voltage_cooldown_seconds = cd;
        high_voltage_cooldown_seconds = cd;
      }
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/inching", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("inching_enabled")) inching_enabled = doc["inching_enabled"];
      if (doc.containsKey("inching_duration_minutes")) inching_duration_minutes = constrain(doc["inching_duration_minutes"].as<uint32_t>(), 1, 60);
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/inrush", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("inrush_enabled")) inrush_enabled = doc["inrush_enabled"];
      if (doc.containsKey("inrush_tolerance_ms")) inrush_tolerance_ms = constrain(doc["inrush_tolerance_ms"].as<uint32_t>(), 500, 5000);
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/softstart", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("soft_start_enabled")) soft_start_enabled = doc["soft_start_enabled"];
      if (doc.containsKey("soft_start_delay_ms")) soft_start_delay_ms = constrain(doc["soft_start_delay_ms"].as<uint32_t>(), 0, 10000);
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/pressure", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<128> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("pressure_switch_inverted")) pressure_switch_inverted = doc["pressure_switch_inverted"];
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/system", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("hostname")) { String nh = doc["hostname"].as<String>(); if (nh.length() > 0) { nh.toCharArray(hostname, sizeof(hostname)); deviceName = String(hostname); } }
      if (doc.containsKey("peer_mac")) { String macStr = doc["peer_mac"].as<String>(); if (macStr.length() > 0 && stringToMac(macStr, peer_mac)) macStr.toCharArray(peer_mac_str, sizeof(peer_mac_str)); }
      if (doc.containsKey("espnow_channel")) espnow_channel = constrain(doc["espnow_channel"].as<int>(), 1, 13);
      if (doc.containsKey("use_espnow")) use_espnow = doc["use_espnow"];
      if (doc.containsKey("auto_return_enabled")) auto_return_enabled = doc["auto_return_enabled"];
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
      delay(100);
      rebootWithCountdown();
    }
  });
  
  server.on("/stats", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["total_runtime"] = total_runtime_seconds / 60;
    doc["pump_cycles"] = pump_cycles;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/espnow/request", HTTP_GET, []() { sendEspNowCommand("get_measure"); server.send(200, "text/plain", "OK"); });
  server.on("/factoryreset", HTTP_GET, []() { server.send(200, "text/plain", "Resetting..."); delay(100); factoryReset(); });
  server.on("/reboot", HTTP_GET, []() { server.send(200, "text/plain", "Rebooting..."); delay(100); rebootDevice(); });
  
  server.begin();
}

// ================================================================================================
// @section     SETUP & LOOP
// ================================================================================================

void setup() {
  systemStartTime = millis();
  if (DEBUG_ENABLED) Serial.begin(115200);
  
  int fsRetries = 0;
  while (!LittleFS.begin() && fsRetries < 3) { delay(500); fsRetries++; }
  if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }
  
  loadConfigFromFile();
  deviceName = String(hostname);
  s31.begin();
  setupAPMode();
  
  if (wifi_sta_enabled && strlen(wifi_ssid) > 0) connectToWiFi();
  
  if (ENABLE_NTP) initNTP();
  
  if (ENABLE_ASYNC_MQTT) {
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
  }
  
  initEspNow();
  setupWebServer();
  setupArduinoOTA();
  initButton();
  initPressureSwitch();
  
  if (ENABLE_MDNS) { MDNS.begin(deviceName.c_str()); MDNS.addService("http", "tcp", 80); }
  
  dryRun.lowPowerStartTime = 0;
  overload.lastRelayState = s31.getRelayState();
  overload.pumpStartTime = overload.lastRelayState ? millis() : 0;
  overload.bootInitialized = true;
  softStart.delayActive = false;
  inching.active = false;
  pressureLockout.active = false;
  if (!auto_mode) button.lastManualModeTime = millis();
  
  if (DEBUG_ENABLED) {
    Serial.println("\n╔════════════════════════════════════════════════════════════════════════════════╗");
    Serial.println("║              SMART PUMP CONTROLLER v" FIRMWARE_VERSION " - IMPROVED WIFI SCAN                  ║");
    Serial.println("╠════════════════════════════════════════════════════════════════════════════════╣");
    Serial.println("║  NEW v3.9: Improved WiFi Scan - Single click with auto-retry                   ║");
    Serial.println("║  NEW v3.9: Scan spinner animation and better error handling                   ║");
    Serial.println("║  NEW v3.9: Signal strength indicators (Excellent/Good/Fair)                   ║");
    Serial.println("║  NEW v3.9: 10-second timeout for WiFi scans                                   ║");
    Serial.println("║  ✓ Statistics Reset (Runtime, Cycles, Events)                                 ║");
    Serial.println("║  ✓ WiFi Save with 5-Second Countdown Reboot                                   ║");
    Serial.println("║  ✓ High Voltage Protection (>240V stops pump instantly)                       ║");
    Serial.println("║  ✓ Low Voltage Protection (<180V stops pump instantly)                        ║");
    Serial.println("║  ✓ AUTO→MANUAL cancels ALL timers including cooldowns                         ║");
    Serial.println("║  ✓ Async MQTT - Non-blocking telemetry and commands                           ║");
    Serial.println("║  ✓ NTP Time Sync - Accurate timestamps (GMT+7 Bangkok)                        ║");
    Serial.println("║  ✓ All v3.5 UI Features Preserved (Lockout Presets, Real-time Monitoring)     ║");
    Serial.println("╚════════════════════════════════════════════════════════════════════════════════╝\n");
  }
}

void loop() {
  uint32_t now = millis();
  
  feedSystemWatchdog();
  handleButton();
  readPressureSwitch();
  
  if (ENABLE_NTP && wifiConnected && (now - lastNTPUpdate >= NTP_UPDATE_INTERVAL || !ntpSynced)) syncNTPTime();
  
  if (ENABLE_CAPTIVE_PORTAL) dnsServer.processNextRequest();
  if (now - lastS31Update >= S31_UPDATE_INTERVAL) { lastS31Update = now; s31.update(); }
  if (ENABLE_OTA && now - lastOTA >= OTA_INTERVAL) { lastOTA = now; ArduinoOTA.handle(); }
  if (now - lastWebServer >= WEB_SERVER_INTERVAL) { lastWebServer = now; server.handleClient(); }
  
  requestSensorData();
  controlPump();
  checkRebootCountdown();
  
  if (ENABLE_MDNS && now - lastMDNS >= MDNS_INTERVAL) { lastMDNS = now; MDNS.update(); }
  if (now - lastStatsSave >= SAVE_STATS_INTERVAL) { lastStatsSave = now; saveStatistics(); }
  if (ENABLE_MEMORY_MONITOR && now - lastMemoryCheck >= MEMORY_CHECK_INTERVAL) { lastMemoryCheck = now; checkMemoryAndCleanup(); }
  if (pendingSave && !saveInProgress && now - lastSaveTime >= MIN_SAVE_INTERVAL_MS) saveConfigToFile();
  if (ENABLE_WIFI_RECOVERY) recoverWiFiConnection();
  recoverEspNow();
  delay(1);
}

// ================================================================================================
// @section     END OF CODE - SMART PUMP CONTROLLER v3.9
// @section     IMPROVED: WiFi Scan now works with single click (auto-retry)
// @section     v3.5 UI FEATURES PRESERVED: All original UI layouts and styles
// ================================================================================================