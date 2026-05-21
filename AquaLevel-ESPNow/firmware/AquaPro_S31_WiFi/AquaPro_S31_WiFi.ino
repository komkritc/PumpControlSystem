/**
 * ================================================================================================
 * SMART PUMP CONTROLLER - v3.5 (COMPLETE WITH NICE UI)
 * ================================================================================================
 * @author: Smart Pump Team
 * @version: 3.5
 * @date: 2026-05-21
 * 
 * @features:
 *   - SAFETY PRIORITY: Dry Run, Overload, Pressure Lockout CANCEL all other timers
 *   - AUTO → MANUAL mode: Cancels ALL timers and stops pump immediately
 *   - Dry Run stops pump INSTANTLY when low power persists (configurable 3-300 seconds)
 *   - Overload protection with cooldown period
 *   - Pressure lockout timer (5 minutes to 3 days, configurable)
 *   - Real-time power monitoring (Voltage, Current, Power, Energy kWh)
 *   - Virtual tank gauge with water level animation
 *   - Dual WiFi mode (STA + AP fallback with mDNS support)
 *   - WiFi configuration page with network scanner
 *   - Reboot countdown UI (5 seconds with visual feedback)
 *   - Integrated timer display for all protection modes
 *   - No pump cycling - prevents rapid on/off
 *   - No reboots on settings save (mutex protection)
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

// ================================================================================================
// @section     VERSION & FEATURE FLAGS
// ================================================================================================
#define FIRMWARE_VERSION        "3.5"
#define FIRMWARE_DATE           "2026-05-21"
#define ENABLE_MDNS             true
#define ENABLE_OTA              true
#define ENABLE_CAPTIVE_PORTAL   true
#define ENABLE_WIFI_RECOVERY    true
#define ENABLE_MEMORY_MONITOR   true

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
#define MAX_LOG_ENTRIES         30
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
void verifyConfigSave();
void checkMemoryAndCleanup();
void recoverWiFiConnection();
void recoverEspNow();
void emergencySave();
void connectToWiFi();
void cancelAllTimers();
void stopPumpAndCancelTimers(const char* reason);

// ================================================================================================
// @section     GLOBAL VARIABLES
// ================================================================================================
SonoffS31 s31(RELAY_PIN);
ESP8266WebServer server(80);
DNSServer dnsServer;
String deviceName = "s31-pump";

// Save protection
bool saveInProgress = false;
uint32_t lastSaveTime = 0;
bool pendingSave = false;

// Global JSON document
StaticJsonDocument<1536> configDoc;

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
bool use_espnow = true;
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

// WiFi configuration
char wifi_ssid[64] = "";
char wifi_password[64] = "";
bool wifi_sta_enabled = false;

// Pressure lockout
float pressure_lockout_hours = 2.0;

// ================================================================================================
// @section     STATE STRUCTURES
// ================================================================================================
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
#else
  #define DEBUG_LOG(msg)
  #define DEBUG_OVERLOAD(msg)
  #define DEBUG_DRYRUN(msg)
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
#endif

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
  }
  if (softStart.delayActive) {
    softStart.delayActive = false;
    DEBUG_LOG("All timers cancelled: Soft start cancelled");
  }
}

void stopPumpAndCancelTimers(const char* reason) {
  cancelAllTimers();
  if (s31.getRelayState()) {
    s31.setRelay(false);
    char msg[80];
    snprintf(msg, sizeof(msg), "⏹️ Pump stopped - %s", reason);
    addFailureLogEntry(msg);
  } else {
    char msg[80];
    snprintf(msg, sizeof(msg), "✓ Pump already OFF - %s", reason);
    addFailureLogEntry(msg);
  }
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
    addFailureLogEntry("WiFi lost, attempting recovery #" + String(wifiConnectAttempts));
    
    WiFi.disconnect();
    delay(100);
    WiFi.begin(wifi_ssid, wifi_password);
    
    lastWifiRecovery = now;
    wifiRecoveryInProgress = false;
  } else if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    wifiConnected = true;
    addFailureLogEntry("✅ WiFi reconnected!");
  }
}

// ================================================================================================
// @section     FILESYSTEM & JSON
// ================================================================================================

void backupConfigFile() {
  if (LittleFS.exists(CONFIG_FILE)) {
    File src = LittleFS.open(CONFIG_FILE, "r");
    if (src) {
      File dst = LittleFS.open(CONFIG_BACKUP_FILE, "w");
      if (dst) {
        while (src.available()) dst.write(src.read());
        dst.close();
      }
      src.close();
    }
  }
}

void verifyConfigSave() {
  if (LittleFS.exists(CONFIG_FILE)) {
    File file = LittleFS.open(CONFIG_FILE, "r");
    if (file) {
      configDoc.clear();
      deserializeJson(configDoc, file);
      file.close();
    }
  }
}

void saveConfigToFile() {
  if (saveInProgress) {
    pendingSave = true;
    return;
  }
  
  uint32_t now = millis();
  if (now - lastSaveTime < MIN_SAVE_INTERVAL_MS) {
    pendingSave = true;
    return;
  }
  
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
  
  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) {
    saveInProgress = false;
    return;
  }
  
  yield();
  ESP.wdtFeed();
  
  serializeJson(configDoc, file);
  file.flush();
  file.close();
  
  saveInProgress = false;
  pendingSave = false;
}

void loadConfigFromFile() {
  if (!LittleFS.exists(CONFIG_FILE)) {
    saveConfigToFile();
    return;
  }
  
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
  use_espnow = configDoc["use_espnow"] | true;
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
  
  if (pressure_lockout_hours < 0) pressure_lockout_hours = 0;
  if (pressure_lockout_hours > 0 && pressure_lockout_hours < (MIN_PRESSURE_LOCKOUT_MINUTES / 60.0)) {
    pressure_lockout_hours = MIN_PRESSURE_LOCKOUT_MINUTES / 60.0;
  }
  if (pressure_lockout_hours > MAX_PRESSURE_LOCKOUT_HOURS) pressure_lockout_hours = MAX_PRESSURE_LOCKOUT_HOURS;
  
  min_power_threshold = constrain(min_power_threshold, 0.0, 3500.0);
  pump_protection_time = constrain(pump_protection_time, 3, 300);
  max_power_threshold = constrain(max_power_threshold, 10.0, 3500.0);
  overload_cooldown_seconds = constrain(overload_cooldown_seconds, 0, 300);
  dryrun_cooldown_seconds = constrain(dryrun_cooldown_seconds, 0, 300);
  inrush_tolerance_ms = constrain(inrush_tolerance_ms, 500, 5000);
  soft_start_delay_ms = constrain(soft_start_delay_ms, 0, 10000);
  espnow_channel = constrain(espnow_channel, 1, 13);
  inching_duration_minutes = constrain(inching_duration_minutes, 1, 60);
  
  overload.overloadCount = overload_events;
  dryRun.events = dryrun_events;
  softStart.delayCount = soft_start_count;
  button.shortPressCount = button_press_count;
}

void saveStatistics() {
  if (saveInProgress) {
    pendingSave = true;
    return;
  }
  overload_events = overload.overloadCount;
  dryrun_events = dryRun.events;
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
  char timestamp[20];
  snprintf(timestamp, sizeof(timestamp), "[%lus]", millis() / 1000);
  String entry = String(timestamp) + " " + String(message);
  
  failureLog[logIndex] = entry;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) logCount++;
}

void addFailureLogEntry(String message) {
  addFailureLogEntry(message.c_str());
}

String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

bool stringToMac(const String& macStr, uint8_t* mac) {
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", 
             &values[0], &values[1], &values[2], 
             &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) {
      if (values[i] < 0 || values[i] > 255) return false;
      mac[i] = (uint8_t)values[i];
    }
    return true;
  }
  return false;
}

void rebootDevice() { 
  saveStatistics();
  if (saveInProgress) {
    uint32_t waitStart = millis();
    while (saveInProgress && (millis() - waitStart < 3000)) {
      yield();
      delay(10);
    }
  }
  delay(500);
  ESP.restart(); 
}

String getUptimeString() {
  uint32_t uptimeSeconds = (millis() - systemStartTime) / 1000;
  uint32_t days = uptimeSeconds / 86400;
  uint32_t hours = (uptimeSeconds % 86400) / 3600;
  uint32_t minutes = (uptimeSeconds % 3600) / 60;
  
  char buf[32];
  if (days > 0) snprintf(buf, sizeof(buf), "%lud %luh", days, hours);
  else if (hours > 0) snprintf(buf, sizeof(buf), "%luh %lum", hours, minutes);
  else snprintf(buf, sizeof(buf), "%lum", minutes);
  return String(buf);
}

String formatTimeRemaining(uint32_t seconds) {
  if (seconds >= 86400) {
    uint32_t days = seconds / 86400;
    uint32_t hours = (seconds % 86400) / 3600;
    char buf[32];
    snprintf(buf, sizeof(buf), "%lud %luh", days, hours);
    return String(buf);
  } else if (seconds >= 3600) {
    uint32_t hours = seconds / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    char buf[32];
    snprintf(buf, sizeof(buf), "%luh %lum", hours, minutes);
    return String(buf);
  } else if (seconds >= 60) {
    uint32_t minutes = seconds / 60;
    uint32_t secs = seconds % 60;
    char buf[32];
    snprintf(buf, sizeof(buf), "%lum %lus", minutes, secs);
    return String(buf);
  } else {
    char buf[32];
    snprintf(buf, sizeof(buf), "%lus", seconds);
    return String(buf);
  }
}

// ================================================================================================
// @section     INCHING MODE
// ================================================================================================

void startInchingTimer() {
  if (inching_enabled && inching_duration_minutes > 0) {
    inching.active = true;
    inching.startTime = millis();
    inching.duration = inching_duration_minutes * 60 * 1000;
    inching.pendingStop = false;
    char msg[64];
    snprintf(msg, sizeof(msg), "⏱️ INCHING: %lu min", inching_duration_minutes);
    addFailureLogEntry(msg);
  }
}

void cancelInchingTimer() {
  if (inching.active) {
    inching.active = false;
    addFailureLogEntry("⏱️ INCHING cancelled");
  }
}

void checkInchingTimer() {
  if (!inching.active) return;
  if (millis() - inching.startTime >= inching.duration) {
    if (!inching.pendingStop) {
      inching.pendingStop = true;
      addFailureLogEntry("⏱️ INCHING: Auto-stopping");
      s31.setRelay(false);
      inching.active = false;
    }
  }
}

// ================================================================================================
// @section     PRESSURE SWITCH
// ================================================================================================

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
    }
  }
  
  pressureSwitch.lastReading = reading;
  return pressureSwitch.currentState;
}

bool pressureAllowsPumpOn() {
  readPressureSwitch();
  return pressureSwitch.pressureLow;
}

// ================================================================================================
// @section     PRESSURE LOCKOUT
// ================================================================================================

void resetPressureLockout() {
  pressureLockout.active = false;
  pressureLockout.lockoutUntil = 0;
  pressureLockout.lastHighPressureTime = 0;
  pressureLockout.waitingForLowPressure = false;
  addFailureLogEntry("🔓 Lockout reset");
}

bool isPressureLockoutActive() {
  uint32_t now = millis();
  if (pressureLockout.active && now >= pressureLockout.lockoutUntil) {
    pressureLockout.active = false;
    addFailureLogEntry("🔓 Lockout expired");
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
      if (pressure_lockout_hours > 0 && lockoutMs < (MIN_PRESSURE_LOCKOUT_MINUTES * 60000)) {
        lockoutMs = MIN_PRESSURE_LOCKOUT_MINUTES * 60000;
      }
      pressureLockout.lockoutUntil = millis() + lockoutMs;
      pressureLockout.active = true;
      
      char msg[64];
      if (pressure_lockout_hours >= 24) {
        snprintf(msg, sizeof(msg), "🔒 Lockout: %.1f days", pressure_lockout_hours / 24.0);
      } else if (pressure_lockout_hours >= 1) {
        snprintf(msg, sizeof(msg), "🔒 Lockout: %.1f h", pressure_lockout_hours);
      } else {
        snprintf(msg, sizeof(msg), "🔒 Lockout: %d min", (int)(pressure_lockout_hours * 60));
      }
      addFailureLogEntry(msg);
      cancelAllTimers();
    }
  } else {
    pressureLockout.lastHighPressureTime = 0;
  }
}

uint32_t getLockoutRemainingSeconds() {
  if (!pressureLockout.active) return 0;
  uint32_t now = millis();
  if (now >= pressureLockout.lockoutUntil) return 0;
  return (pressureLockout.lockoutUntil - now) / 1000;
}

// ================================================================================================
// @section     SOFT START
// ================================================================================================

void startSoftStartDelay(bool desiredState) {
  if (!softStart.delayActive) {
    softStart.delayActive = true;
    softStart.delayStartTime = millis();
    softStart.pendingPumpState = desiredState;
    softStart.delayCount++;
    soft_start_count = softStart.delayCount;
    char msg[48];
    snprintf(msg, sizeof(msg), "SOFT START: %lu ms", soft_start_delay_ms);
    addFailureLogEntry(msg);
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
    return softStart.pendingPumpState;
  }
  return false;
}

void cancelSoftStartDelay() {
  softStart.delayActive = false;
}

uint32_t getSoftStartRemainingMs() {
  if (!softStart.delayActive) return 0;
  uint32_t elapsed = millis() - softStart.delayStartTime;
  if (elapsed >= soft_start_delay_ms) return 0;
  return soft_start_delay_ms - elapsed;
}

// ================================================================================================
// @section     POWER QUALITY
// ================================================================================================

void updatePowerQuality() {
  float voltage = s31.getVoltage();
  float current = s31.getCurrent();
  float realPower = s31.getPower();
  bool relayState = s31.getRelayState();
  
  apparentPower = voltage * current;
  
  if (!relayState || realPower < 0.5) {
    powerFactor = 0.0;
  } else if (apparentPower > 0.01) {
    powerFactor = realPower / apparentPower;
    powerFactor = constrain(powerFactor, 0.0, 1.0);
  } else {
    powerFactor = 0.0;
  }
  
  total_energy_kwh = s31.getEnergy();
}

// ================================================================================================
// @section     ESP-NOW
// ================================================================================================

void parseEspNowData(String data) {
  float d = 0, l = 0, v = 0, b = 0;
  
  int dIndex = data.indexOf("\"d\":");
  if (dIndex != -1) {
    int start = dIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1) d = data.substring(start, end).toFloat();
  }
  
  int lIndex = data.indexOf("\"l\":");
  if (lIndex != -1) {
    int start = lIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1) l = data.substring(start, end).toFloat();
  }
  
  int vIndex = data.indexOf("\"v\":");
  if (vIndex != -1) {
    int start = vIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1) v = data.substring(start, end).toFloat();
  }
  
  int bIndex = data.indexOf("\"b\":");
  if (bIndex != -1) {
    int start = bIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1) b = data.substring(start, end).toFloat();
  }
  
  if (d > 0 || l > 0 || v > 0 || b > 0) {
    currentDistance = d;
    currentWaterLevel = l;
    currentVolume = v;
    batteryVoltage = b;
    lastEspNowData = millis();
    espnowDataValid = true;
    sensorIsDead = false;
  }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {}
void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len != sizeof(EspNowPacket)) return;
  memcpy(&incoming, data, sizeof(incoming));
  parseEspNowData(String(incoming.msg));
}

void initEspNow() {
  if (!use_espnow) {
    espnow_initialized = false;
    return;
  }
  
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != 0) {
    espnow_initialized = false;
    return;
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  uint8_t* peerMac = broadcastMac;
  if (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) peerMac = peer_mac;
  
  if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_COMBO, espnow_channel, NULL, 0) != 0) {
    espnow_initialized = false;
    return;
  }
  
  espnow_initialized = true;
}

void sendEspNowCommand(const char* command) {
  if (!espnow_initialized) return;
  
  for (uint8_t retry = 0; retry < ESP_NOW_RETRY_COUNT; retry++) {
    outgoing.seq = espnow_seq++;
    outgoing.timestamp = micros();
    strncpy(outgoing.msg, command, sizeof(outgoing.msg)-1);
    
    uint8_t* peerMac = broadcastMac;
    if (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) peerMac = peer_mac;
    
    if (esp_now_send(peerMac, (uint8_t *)&outgoing, sizeof(outgoing)) == 0) return;
    delay(50);
  }
}

void requestSensorData() {
  if (!espnow_initialized) return;
  if (millis() - lastEspNowSend >= ESP_NOW_SEND_INTERVAL) {
    lastEspNowSend = millis();
    sendEspNowCommand("get_measure");
  }
}

void recoverEspNow() {
  if (!use_espnow) return;
  if (millis() - lastEspNowRecovery < ESP_NOW_RECOVERY_INTERVAL) return;
  
  bool dataStale = (millis() - lastEspNowData > ESP_NOW_DATA_TIMEOUT * 2);
  if (!espnow_initialized || (dataStale && lastEspNowData > 0)) {
    esp_now_deinit();
    delay(100);
    initEspNow();
    lastEspNowRecovery = millis();
  }
}

// ================================================================================================
// @section     OVERLOAD PROTECTION
// ================================================================================================

void updateInrushState(bool currentState) {
  if (!inrush_enabled) {
    overload.inrushActive = false;
    overload.lastRelayState = currentState;
    return;
  }
  
  if (currentState && !overload.lastRelayState) {
    overload.pumpStartTime = millis();
    overload.inrushActive = true;
  }
  
  if (overload.inrushActive && currentState && (millis() - overload.pumpStartTime >= inrush_tolerance_ms)) {
    overload.inrushActive = false;
  }
  
  if (!currentState) overload.inrushActive = false;
  overload.lastRelayState = currentState;
}

bool checkOverloadProtection(float currentPower, bool currentState) {
  updateInrushState(currentState);
  
  if (overload.cooldownUntil > millis()) {
    if (currentState) s31.setRelay(false);
    return false;
  }
  if (overload.active && millis() >= overload.cooldownUntil) {
    overload.active = false;
  }
  
  if (!pump_load_protection_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  
  if (currentPower > max_power_threshold && !overload.active) {
    if (millis() - overload.pumpStartTime > 500) {
      overload.active = true;
      overload.cooldownUntil = millis() + (overload_cooldown_seconds * 1000);
      overload.overloadCount++;
      overload_events = overload.overloadCount;
      char msg[80];
      snprintf(msg, sizeof(msg), "⚠️ OVERLOAD! %.0fW - Pump stopped - All timers cancelled", currentPower);
      addFailureLogEntry(msg);
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

// ================================================================================================
// @section     DRY RUN PROTECTION
// ================================================================================================

bool checkDryRunProtection(float currentPower, bool currentState) {
  if (dryRun.cooldownUntil > millis()) {
    if (currentState) s31.setRelay(false);
    return false;
  }
  if (dryRun.inCooldown && millis() >= dryRun.cooldownUntil) {
    dryRun.inCooldown = false;
    dryRun.protectionTriggered = false;
    dryRun.lowPowerDetected = false;
    addFailureLogEntry("✅ Dry run cooldown ended - Pump can restart");
  }
  
  if (!dry_run_enabled || !currentState) return true;
  if (inrush_enabled && overload.inrushActive) return true;
  if (min_power_threshold <= 0.1) return true;
  
  if (currentPower < min_power_threshold) {
    if (!dryRun.lowPowerDetected) {
      dryRun.lowPowerDetected = true;
      dryRun.lowPowerStartTime = millis();
      char msg[80];
      snprintf(msg, sizeof(msg), "⚠️ Low power detected: %.1fW < %.1fW - Timer started (%lus)", 
               currentPower, min_power_threshold, pump_protection_time);
      addFailureLogEntry(msg);
    }
    
    uint32_t elapsedSeconds = (millis() - dryRun.lowPowerStartTime) / 1000;
    
    if (elapsedSeconds >= pump_protection_time) {
      char msg[80];
      snprintf(msg, sizeof(msg), "💧 DRY RUN! Pump stopped INSTANTLY after %lus - All timers cancelled", pump_protection_time);
      addFailureLogEntry(msg);
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
    }
  }
  return true;
}

uint32_t getDryRunCooldownRemaining() {
  if (dryRun.cooldownUntil <= millis()) return 0;
  return (dryRun.cooldownUntil - millis()) / 1000;
}

// ================================================================================================
// @section     PUMP CONTROL
// ================================================================================================

void updatePumpStatistics(bool currentState) {
  static uint32_t lastRuntimeUpdate = 0;
  uint32_t now = millis();
  
  if (currentState && !overload.lastRelayState) {
    pump_cycles++;
    lastRuntimeUpdate = now;
  } else if (!currentState && overload.lastRelayState && lastRuntimeUpdate > 0) {
    total_runtime_seconds += (now - lastRuntimeUpdate) / 1000;
    total_energy_kwh = s31.getEnergy();
  }
}

void checkAutoReturnToAutoMode() {
  if (auto_return_enabled && !auto_mode && button.lastManualModeTime > 0) {
    if (millis() - button.lastManualModeTime >= AUTO_RETURN_TIMEOUT_MS) {
      auto_mode = true;
      saveConfigToFile();
      addFailureLogEntry("🔄 Auto-return: AUTO mode");
      button.lastManualModeTime = 0;
    }
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
  
  if (!checkOverloadProtection(currentPower, currentState)) {
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
  
  if (finalState && !currentState && inching_enabled && auto_mode) {
    startInchingTimer();
  }
  if (!finalState && currentState && inching.active) {
    cancelInchingTimer();
  }
  
  if (finalState != currentState) {
    s31.setRelay(finalState);
    const char* reason;
    if (inching.active) reason = "INCHING";
    else if (softStart.delayActive) reason = "SOFT START";
    else if (!auto_mode) reason = "MANUAL";
    else if (pressureLockout.active) reason = "LOCKOUT";
    else if (pressureSwitch.pressureLow) reason = "LOW PRESSURE";
    else reason = "HIGH PRESSURE";
    
    char msg[48];
    snprintf(msg, sizeof(msg), "Pump %s - %s", finalState ? "ON" : "OFF", reason);
    addFailureLogEntry(msg);
  }
  
  updatePumpStatistics(s31.getRelayState());
  overload.lastRelayState = currentState;
}

// ================================================================================================
// @section     BUTTON HANDLING
// ================================================================================================

void initButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void flashLed(uint32_t duration_ms) {
  uint32_t delayTime = duration_ms > 100 ? 100 : duration_ms;
  digitalWrite(LED_PIN, LOW);
  delay(delayTime);
  digitalWrite(LED_PIN, HIGH);
}

void handleShortPress() {
  button.shortPressCount++;
  button_press_count = button.shortPressCount + button.longPressCount;
  flashLed(50);
  
  if (millis() < overload.cooldownUntil) {
    addFailureLogEntry("Button: Overload cooldown active - Cannot switch mode");
    flashLed(1000);
    return;
  }
  if (dryRun.cooldownUntil > millis()) {
    addFailureLogEntry("Button: Dry run cooldown active - Cannot switch mode");
    flashLed(1000);
    return;
  }
  
  if (auto_mode) {
    addFailureLogEntry("🔘 Switching from AUTO to MANUAL mode");
    stopPumpAndCancelTimers("Switched to MANUAL mode");
    auto_mode = false;
    saveConfigToFile();
    button.lastManualModeTime = millis();
    addFailureLogEntry("→ MANUAL mode (Pump OFF, all timers cancelled)");
    flashLed(100);
    delay(100);
    flashLed(100);
  } else {
    button.lastManualModeTime = millis();
    if (pressureLockout.active) {
      addFailureLogEntry("Button: Pressure lockout active - Cannot start pump");
      flashLed(1000);
      return;
    }
    bool newState = !s31.getRelayState();
    s31.setRelay(newState);
    if (newState) {
      cancelInchingTimer();
      cancelSoftStartDelay();
      addFailureLogEntry("Button: Pump turned ON in MANUAL mode");
    } else {
      addFailureLogEntry("Button: Pump turned OFF in MANUAL mode");
    }
    char msg[48];
    snprintf(msg, sizeof(msg), "Button: Pump %s (Manual mode)", newState ? "ON" : "OFF");
    addFailureLogEntry(msg);
    flashLed(50);
  }
}

void handleLongPress() {
  button.longPressCount++;
  button_press_count = button.shortPressCount + button.longPressCount;
  
  auto_mode = !auto_mode;
  saveConfigToFile();
  
  if (auto_mode) {
    button.lastManualModeTime = 0;
    addFailureLogEntry("→ AUTO mode (Pressure control)");
    for (int i = 0; i < 3; i++) { flashLed(100); delay(150); }
  } else {
    addFailureLogEntry("Long press: Switching to MANUAL mode");
    stopPumpAndCancelTimers("Switched to MANUAL mode");
    button.lastManualModeTime = millis();
    addFailureLogEntry("→ MANUAL mode (Pump OFF, all timers cancelled)");
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
      
      if (button.currentButtonState == LOW) {
        button.pressStartTime = now;
        button.buttonPressed = true;
      } else if (button.buttonPressed) {
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
// @section     WEB SERVER & AP MODE
// ================================================================================================

void setupAPMode() {
  String apSSID = "SmartPump-" + String(ESP.getChipId() & 0xFFFF, HEX);
  WiFi.softAP(apSSID.c_str(), "12345678");
  if (ENABLE_CAPTIVE_PORTAL) {
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
  }
}

void setupArduinoOTA() {
  if (!ENABLE_OTA) return;
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.begin();
}

// ================================================================================================
// @section     HTML UI (Main Dashboard with Nice UI)
// ================================================================================================

const char simple_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
<title>Smart Pump v3.5</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',Roboto,sans-serif;background:#f1f5f9;padding:16px}
.container{max-width:550px;margin:0 auto}
.card{background:white;border-radius:32px;padding:20px;margin-bottom:18px;box-shadow:0 4px 12px rgba(0,0,0,0.05)}
.header{text-align:center}
h1{font-size:1.7rem;background:linear-gradient(135deg,#0f172a,#2563eb);background-clip:text;-webkit-background-clip:text;color:transparent}
.version{background:#2563eb20;color:#1e40af;padding:4px 12px;border-radius:40px;font-size:0.7rem;display:inline-block;margin:6px 0}
.uptime{background:#dbeafe;padding:10px;border-radius:20px;text-align:center;margin-bottom:12px;font-size:0.85rem}
.wifi-status{background:#f0fdf4;padding:8px;border-radius:20px;margin-bottom:12px;font-size:0.75rem;display:flex;justify-content:space-between;flex-wrap:wrap}
.wifi-connected{color:#059669}
.wifi-disconnected{color:#dc2626}
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
.pump-btn.reboot-countdown{background:#f59e0b;animation:pulse 0.5s infinite}
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
.electrical-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin:12px 0}
.espnow-warning{background:#fff7ed;color:#9a3412;padding:10px;border-radius:12px;margin-bottom:12px;text-align:center;font-size:0.8rem}
.display-only-badge{background:#fef3c7;color:#92400e;padding:4px 8px;border-radius:20px;font-size:0.65rem;margin-left:8px}
.countdown-overlay{position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.8);display:flex;align-items:center;justify-content:center;z-index:1000;flex-direction:column}
.countdown-box{background:white;border-radius:32px;padding:40px;text-align:center;max-width:300px}
.countdown-number{font-size:5rem;font-weight:800;color:#2563eb}
.countdown-text{font-size:1.2rem;margin-top:10px;color:#333}
.countdown-cancel{background:#ef4444;color:white;border:none;padding:12px 24px;border-radius:40px;margin-top:20px;cursor:pointer;font-weight:bold}
</style>
</head>
<body>
<div class="container">
<div class="card">
<div class="header"><h1>💧 AquaPro S31</h1><div class="version">v3.5 SAFETY PRIORITY</div></div>
<div class="uptime">⏱️ Uptime: <strong id="uptime">0</strong></div>
<div class="wifi-status"><span>📡 WiFi:</span><span id="wifiStatus">AP Mode</span><span id="wifiIp"></span></div>
<div class="flex-between"><span>📡 Sensor</span><span id="sensorBadge" class="sensor-chip"><span class="led led-green"></span> Healthy</span></div>
<div class="flex-between"><span>🔘 Pressure</span><span id="pressureStatus" class="sensor-chip"><span class="led led-green"></span> OK</span></div>

<!-- Integrated Timer Display -->
<div id="timerOverload" style="display:none" class="timer-badge">⚠️ Overload: <span id="overloadTimer">0</span></div>
<div id="timerDryRun" style="display:none" class="timer-badge">💧 Dry Run Cooldown: <span id="dryrunTimer">0</span></div>
<div id="timerLockout" style="display:none" class="timer-badge">🔒 Lockout: <span id="lockoutTimer">0</span></div>
<div id="timerSoftStart" style="display:none" class="timer-badge">🌊 Soft Start: <span id="softStartTimer">0</span>ms</div>
<div id="timerInching" style="display:none" class="timer-badge">⏱️ Inching: <span id="inchingTimer">0</span></div>

<div id="lockoutDiv" style="background:#fee2e2;color:#991b1b;padding:8px;border-radius:20px;margin-top:10px;text-align:center;display:none">🔒 LOCKOUT ACTIVE <button onclick="resetLockout()" style="background:#991b1b;color:white;border:none;padding:2px 10px;border-radius:15px">Reset</button></div>
<div id="espnowWarning" class="espnow-warning" style="display:none">⚠️ ESP-NOW DISABLED - Tank hidden</div>
</div>

<div id="virtualTankCard" class="card">
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
<div id="warnings"></div>
</div>

<div class="card">
<div class="mode-row"><span id="modeText">🤖 AUTO</span><label class="toggle-switch"><input type="checkbox" id="autoToggle" onchange="toggleAuto()"><span class="slider"></span></label><span>MANUAL</span></div>
<button id="pumpBtn" class="pump-btn" onclick="manualToggle()">PUMP OFF</button>
<div id="reason" style="font-size:0.7rem;text-align:center;margin-top:5px"></div>
<hr>
<div class="flex-between"><span>📦 Runtime</span><strong><span id="runtime">0</span> min</strong></div>
<div class="flex-between"><span>🔄 Cycles</span><strong><span id="cycles">0</span></strong></div>
<div class="flex-between"><span>⚠️ Overload</span><strong><span id="overloadCount">0</span></strong></div>
<div class="flex-between"><span>💧 Dry run</span><strong><span id="dryrunCount">0</span></strong></div>
</div>

<div class="card"><button class="btn-secondary" onclick="requestSensor()">📡 Request sensor</button><button class="btn-secondary" style="background:#fee2e2;color:#b91c1c" id="rebootBtn" onclick="startRebootCountdown()">🔄 Reboot</button></div>
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

function formatTimeRemaining(seconds){
    if(seconds>=86400) return Math.floor(seconds/86400)+"d "+Math.floor((seconds%86400)/3600)+"h";
    if(seconds>=3600) return Math.floor(seconds/3600)+"h "+Math.floor((seconds%3600)/60)+"m";
    if(seconds>=60) return Math.floor(seconds/60)+"m "+Math.floor(seconds%60)+"s";
    return seconds+"s";
}

async function fetchJSON(u){try{const r=await fetch(u);return await r.json()}catch(e){return null}}
async function toggleAuto(){const isAuto=document.getElementById('autoToggle').checked;await fetch(`/mode?mode=${isAuto?'auto':'manual'}`);refresh()}
async function manualToggle(){const d=await fetchJSON('/data');if(d&&d.pressureLockoutActive){alert("Lockout active!");return}await fetch('/toggle');refresh()}
async function resetLockout(){await fetch('/reset_lockout');refresh()}
async function requestSensor(){await fetch('/espnow/request');refresh()}

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
}

function formatUptime(s){let d=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60);if(d>0)return d+"d "+h+"h";if(h>0)return h+"h "+m+"m";return m+"m"}

async function refresh(){
const d=await fetchJSON('/data');if(!d)return;
document.getElementById('uptime').innerText=formatUptime(d.systemUptime||0);
const ws=document.getElementById('wifiStatus');const wi=document.getElementById('wifiIp');
if(d.wifiConnected){ws.innerHTML='✅ '+d.wifiSsid;ws.className='wifi-connected';wi.innerHTML=d.wifiIp}
else if(d.wifiEnabled){ws.innerHTML='⚠️ Connecting';ws.className='wifi-disconnected';wi.innerHTML=''}
else{ws.innerHTML='📡 AP Mode';ws.className='';wi.innerHTML='192.168.4.1'}

// Update power values
document.getElementById('voltage').innerText=(d.voltage||0).toFixed(1);
document.getElementById('current').innerText=(d.current||0).toFixed(2);
document.getElementById('power').innerText=(d.power||0).toFixed(1);
document.getElementById('energy').innerText=(d.energy||0).toFixed(2);

// Update all timer displays
const overloadCooldown=d.overloadCooldownRemaining||0;
const dryrunCooldown=d.dryrunCooldownRemaining||0;
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

// Show/hide virtual tank based on ESP-NOW
const tankCard=document.getElementById('virtualTankCard');
const espnowWarning=document.getElementById('espnowWarning');

if(d.espnowActive && d.useEspnow){
    tankCard.classList.remove('hidden');
    espnowWarning.style.display='none';
    let lvl=Math.min(100,Math.max(0,d.waterLevel||0));
    document.getElementById('level').innerText=Math.floor(lvl);
    document.getElementById('waterFill').style.height=lvl+'%';
    document.getElementById('waterFill').innerText=Math.floor(lvl)+'%';
    document.getElementById('sensorVoltage').innerHTML=(d.batteryVoltage||0).toFixed(2);
    document.getElementById('volume').innerText=(d.volume||0).toFixed(0);
}else{
    tankCard.classList.add('hidden');
    espnowWarning.style.display='block';
}

document.getElementById('overloadCount').innerText=d.overloadCount||0;
document.getElementById('dryrunCount').innerText=d.dryrunCount||0;

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
else if(softStartRemaining>0){btn.innerText="🌊 SOFT START "+(softStartRemaining/1000).toFixed(1)+"s";btn.className='pump-btn softstart';btn.disabled=true}
else if(inchingRemaining>0){btn.innerText="⏱️ INCHING "+formatTimeRemaining(inchingRemaining);btn.className='pump-btn inching';btn.disabled=false}
else if(d.pumpState){btn.innerText="💧 PUMP ON";btn.className='pump-btn running';btn.disabled=false}
else{btn.innerText="⏹️ PUMP OFF";btn.className='pump-btn';btn.disabled=false}

let r="";
if(d.pressureLockoutActive)r="🔒 Lockout - Pump disabled";
else if(overloadCooldown>0)r="⚠️ Overload cooldown - Pump disabled";
else if(dryrunCooldown>0)r="💧 Dry run cooldown - Pump disabled";
else if(softStartRemaining>0)r="🌊 Soft start - Delaying pump start";
else if(inchingRemaining>0)r="⏱️ Inching mode - Auto stop in "+(inchingRemaining>60?Math.floor(inchingRemaining/60)+"m "+(inchingRemaining%60)+"s":inchingRemaining+"s");
else if(!d.autoMode)r="👆 Manual control";
else if(d.pressureLow)r="🔘 Low pressure - Pump ON";
else r="🔘 Pressure OK - Pump OFF";
document.getElementById('reason').innerHTML=r;

let warn="";
if(d.overloadProtectionActive)warn+='<div class="protection-badge">⚠️ OVERLOAD ACTIVE</div>';
if(d.dryRunActive)warn+='<div class="protection-badge">💧 DRY RUN DETECTED</div>';
if(d.inInrushPeriod)warn+='<div class="protection-badge">⚡ INRUSH ACTIVE</div>';
document.getElementById('warnings').innerHTML=warn;

const ss=document.getElementById('sensorBadge');
if(d.sensorHealthy)ss.innerHTML='<span class="led led-green"></span> ✅ Healthy';
else if(d.sensorWarning)ss.innerHTML='<span class="led led-yellow"></span> ⚠️ Weak';
else ss.innerHTML='<span class="led led-red"></span> ❌ No data';

const stats=await fetchJSON('/stats');
if(stats){document.getElementById('runtime').innerText=stats.total_runtime||0;document.getElementById('cycles').innerText=stats.pump_cycles||0}
}
setInterval(refresh,1000);window.onload=refresh;
</script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     ENGINEERING HTML
// ================================================================================================

const char engineering_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Engineering v3.5</title>
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
.wifi-scan-result{max-height:200px;overflow-y:auto;border:1px solid #ddd;border-radius:8px;margin-top:10px}
.wifi-network{padding:8px;border-bottom:1px solid #eee;cursor:pointer;display:flex;justify-content:space-between}
.wifi-network:hover{background:#f0f0f0}
</style>
</head>
<body>
<div class="container">
<div class="card">
<h1>🔧 Engineering v3.5</h1>
<div class="version">SAFETY PRIORITY | DRY RUN INSTANT</div>
<div class="nav-buttons">
<button class="nav-btn active" onclick="showSection('lockout')">🔒 Lockout</button>
<button class="nav-btn" onclick="showSection('wifi')">📡 WiFi</button>
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
<button onclick="saveWifi()">💾 Save</button>
<button onclick="scanWiFi()" class="success">📡 Scan</button>
<div id="wifiScanResult" class="wifi-scan-result" style="display:none"></div>
<div id="wifiScanning" style="display:none;text-align:center;padding:20px">Scanning...</div>
</div>
<div id="protectionSection" style="display:none">
<h3>💧 Dry Run Protection (SAFETY PRIORITY)</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#d1fae5;border-radius:10px">When power drops below threshold for the set time, pump stops INSTANTLY and cancels all timers.</div>
<div class="checkbox-large" onclick="toggleCheckbox('dryRunEnabled')"><input type="checkbox" id="dryRunEnabled" onclick="event.stopPropagation()"><label>Enable Dry Run</label></div>
<div class="config-group"><label>Detection Time (sec)</label><input type="number" id="dryRunProtection" min="3" max="300" value="3"><div class="info-text">Pump stops instantly when low power persists this long</div></div>
<div class="config-group"><label>Min Power (W)</label><input type="number" id="minPower" min="0" max="3500" step="any"><div class="info-text">Below this power = dry run condition</div></div>
<div class="config-group"><label>Cooldown (sec)</label><input type="number" id="dryrunCooldown" min="0" max="300"></div>
<h3>⚠️ Overload Protection (SAFETY PRIORITY)</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#fee2e2;border-radius:10px">When power exceeds max threshold, pump stops and cancels all timers.</div>
<div class="checkbox-large" onclick="toggleCheckbox('loadProtectionToggle')"><input type="checkbox" id="loadProtectionToggle" onclick="event.stopPropagation()"><label>Enable Overload</label></div>
<div class="config-group"><label>Max Power (W)</label><input type="number" id="maxPower" min="10" max="3500"></div>
<div class="config-group"><label>Cooldown (sec)</label><input type="number" id="overloadCooldown" min="0" max="300"></div>
<button onclick="saveProtection()">💾 Save</button>
</div>
<div id="inchingSection" style="display:none">
<h3>⏱️ Inching Mode</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#ede9fe;border-radius:10px">Timer will be CANCELLED if dry run or overload is detected.</div>
<div class="checkbox-large" onclick="toggleCheckbox('inchingToggle')"><input type="checkbox" id="inchingToggle" onclick="event.stopPropagation()"><label>Enable Inching</label></div>
<div class="config-group"><label>Duration (min)</label><input type="number" id="inchingDuration" min="1" max="60"></div>
<button onclick="saveInching()">💾 Save</button>
</div>
<div id="inrushSection" style="display:none">
<h3>⚡ Inrush</h3>
<div class="checkbox-large" onclick="toggleCheckbox('inrushEnabled')"><input type="checkbox" id="inrushEnabled" onclick="event.stopPropagation()"><label>Enable Inrush</label></div>
<div class="config-group"><label>Tolerance (ms)</label><input type="range" id="inrushToleranceSlider" min="500" max="5000" step="100" oninput="updateInrushValue(this.value)"><input type="number" id="inrushTolerance" min="500" max="5000" step="100" oninput="updateInrushSlider(this.value)"></div>
<button onclick="saveInrush()">💾 Save</button>
</div>
<div id="softstartSection" style="display:none">
<h3>🌊 Soft Start</h3>
<div class="info-text" style="margin-bottom:15px;padding:10px;background:#cffafe;border-radius:10px">Delay will be CANCELLED if dry run or overload is detected.</div>
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
<button onclick="saveSystem()">💾 Save & Reboot</button>
<button class="danger" onclick="factoryReset()">⚠️ Factory Reset</button>
<button class="danger" onclick="reboot()">🔄 Reboot</button>
</div>
<div class="simple-link"><a href="/">← Back</a></div>
</div>
</div>
<div id="saveStatus" class="save-status">✓ Saved!</div>
<script>
function toggleCheckbox(id){const cb=document.getElementById(id);if(cb)cb.checked=!cb.checked}
function showSaveStatus(){const s=document.getElementById('saveStatus');s.style.display='block';setTimeout(()=>s.style.display='none',2000)}
function setLockout(h){document.getElementById('lockoutHours').value=h}
async function saveLockout(){const h=parseFloat(document.getElementById('lockoutHours').value);const r=await fetch('/config/lockout',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pressure_lockout_hours:h})});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveWifi(){const e=document.getElementById('wifiEnabled').checked;const s=document.getElementById('wifiSsid').value;const p=document.getElementById('wifiPassword').value;const r=await fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({wifi_enabled:e,wifi_ssid:s,wifi_password:p})});if(r.ok){showSaveStatus();alert("Saved! Rebooting...");setTimeout(()=>location.reload(),2000)}else alert("Failed")}
async function scanWiFi(){const rd=document.getElementById('wifiScanResult');const sd=document.getElementById('wifiScanning');rd.style.display='none';sd.style.display='block';try{const resp=await fetch('/scan_wifi');const nets=await resp.json();sd.style.display='none';rd.style.display='block';if(nets.scanning){rd.innerHTML='<div style="padding:20px">Scanning...</div>';setTimeout(scanWiFi,3000);return}if(nets.length===0){rd.innerHTML='<div style="padding:20px">No networks</div>';return}let html='';for(let i=0;i<nets.length;i++){html+=`<div class="wifi-network" onclick="document.getElementById('wifiSsid').value='${nets[i].ssid.replace(/'/g,"\\'")}';rd.style.display='none'"><div><strong>${nets[i].ssid}</strong></div><div>${nets[i].rssi>-50?'📶 Excellent':nets[i].rssi>-70?'📶 Good':'📶 Fair'}</div></div>`;}rd.innerHTML=html}catch(e){sd.style.display='none';rd.style.display='block';rd.innerHTML='<div style="padding:20px;color:red">Error</div>'}}
function updateInrushValue(v){document.getElementById('inrushTolerance').value=v;document.getElementById('inrushToleranceSlider').value=v}
function updateInrushSlider(v){document.getElementById('inrushToleranceSlider').value=v;document.getElementById('inrushTolerance').value=v}
async function saveProtection(){const s={dry_run_enabled:document.getElementById('dryRunEnabled').checked,dry_run_protection:parseInt(document.getElementById('dryRunProtection').value),min_power:parseFloat(document.getElementById('minPower').value),dryrun_cooldown_seconds:parseInt(document.getElementById('dryrunCooldown').value),pump_load_protection_enabled:document.getElementById('loadProtectionToggle').checked,max_power_threshold:parseFloat(document.getElementById('maxPower').value),overload_cooldown_seconds:parseInt(document.getElementById('overloadCooldown').value)};const r=await fetch('/config/protection',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveInching(){const d=parseInt(document.getElementById('inchingDuration').value);const s={inching_enabled:document.getElementById('inchingToggle').checked,inching_duration_minutes:d};const r=await fetch('/config/inching',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1000)}else alert("Failed")}
async function saveInrush(){const t=parseInt(document.getElementById('inrushTolerance').value);const e=document.getElementById('inrushEnabled').checked;const r=await fetch('/config/inrush',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({inrush_enabled:e,inrush_tolerance_ms:t})});if(r.ok){showSaveStatus();setTimeout(()=>location.reload(),1500)}else alert("Failed")}
async function saveSoftStart(){const s={soft_start_enabled:document.getElementById('softStartToggle').checked,soft_start_delay_ms:parseInt(document.getElementById('softStartDelay').value)};await fetch('/config/softstart',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();setTimeout(()=>location.reload(),1000)}
async function savePressure(){const s={pressure_switch_inverted:document.getElementById('pressureInverted').checked};await fetch('/config/pressure',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();setTimeout(()=>location.reload(),1000)}
async function saveSystem(){const s={hostname:document.getElementById('hostname').value,peer_mac:document.getElementById('peerMac').value,espnow_channel:parseInt(document.getElementById('espnowChannel').value),use_espnow:document.getElementById('useEspnow').checked,auto_return_enabled:document.getElementById('autoReturnToggle').checked};await fetch('/config/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});showSaveStatus();alert('Rebooting...');setTimeout(()=>location.reload(),3000)}
async function factoryReset(){if(confirm('FACTORY RESET? All settings lost!')){await fetch('/factoryreset')}}
async function reboot(){if(confirm('Reboot device?')){await fetch('/reboot')}}
async function loadSettings(){const r=await fetch('/config');const d=await r.json();document.getElementById('lockoutHours').value=d.pressure_lockout_hours||2;document.getElementById('wifiEnabled').checked=d.wifi_sta_enabled||false;document.getElementById('wifiSsid').value=d.wifi_ssid||'';document.getElementById('wifiPassword').value=d.wifi_password||'';document.getElementById('dryRunEnabled').checked=d.dry_run_enabled;document.getElementById('dryRunProtection').value=d.dry_run_protection||3;document.getElementById('minPower').value=d.min_power;document.getElementById('dryrunCooldown').value=d.dryrun_cooldown_seconds;document.getElementById('loadProtectionToggle').checked=d.pump_load_protection_enabled;document.getElementById('maxPower').value=d.max_power_threshold;document.getElementById('overloadCooldown').value=d.overload_cooldown_seconds;document.getElementById('inrushEnabled').checked=d.inrush_enabled;document.getElementById('inrushTolerance').value=d.inrush_tolerance_ms;document.getElementById('inrushToleranceSlider').value=d.inrush_tolerance_ms;document.getElementById('inchingToggle').checked=d.inching_enabled;document.getElementById('inchingDuration').value=d.inching_duration_minutes;document.getElementById('softStartToggle').checked=d.soft_start_enabled;document.getElementById('softStartDelay').value=d.soft_start_delay_ms;document.getElementById('pressureInverted').checked=d.pressure_switch_inverted;document.getElementById('autoReturnToggle').checked=d.auto_return_enabled;document.getElementById('hostname').value=d.hostname||'s31-pump';document.getElementById('peerMac').value=d.peer_mac||'FF:FF:FF:FF:FF:FF';document.getElementById('espnowChannel').value=d.espnow_channel;document.getElementById('useEspnow').checked=d.use_espnow}
function showSection(s){document.getElementById('lockoutSection').style.display=s==='lockout'?'block':'none';document.getElementById('wifiSection').style.display=s==='wifi'?'block':'none';document.getElementById('protectionSection').style.display=s==='protection'?'block':'none';document.getElementById('inchingSection').style.display=s==='inching'?'block':'none';document.getElementById('inrushSection').style.display=s==='inrush'?'block':'none';document.getElementById('softstartSection').style.display=s==='softstart'?'block':'none';document.getElementById('pressureSection').style.display=s==='pressure'?'block':'none';document.getElementById('systemSection').style.display=s==='system'?'block':'none';document.querySelectorAll('.nav-btn').forEach(btn=>btn.classList.remove('active'));event.target.classList.add('active')}
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
    StaticJsonDocument<1024> doc;
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
    if (softStart.delayActive) {
      doc["softStartRemaining"] = getSoftStartRemainingMs();
    }
    doc["overloadProtectionActive"] = overload.active;
    doc["inCooldown"] = (millis() < overload.cooldownUntil);
    doc["overloadCooldownRemaining"] = getOverloadCooldownRemaining();
    doc["dryRunCooldown"] = (millis() < dryRun.cooldownUntil);
    doc["dryrunCooldownRemaining"] = getDryRunCooldownRemaining();
    doc["dryRunActive"] = (dryRun.lowPowerStartTime != 0);
    doc["overloadCount"] = overload.overloadCount;
    doc["dryrunCount"] = dryRun.events;
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
    if (overload.inrushActive && inrush_enabled) {
      uint32_t elapsed = millis() - overload.pumpStartTime;
      uint32_t remaining = (inrush_tolerance_ms > elapsed) ? (inrush_tolerance_ms - elapsed) : 0;
      doc["inrushRemaining"] = remaining;
    }
    
    if (!auto_mode && button.lastManualModeTime > 0) {
      uint32_t elapsed = millis() - button.lastManualModeTime;
      if (elapsed < AUTO_RETURN_TIMEOUT_MS) doc["manualTimeLeft"] = (AUTO_RETURN_TIMEOUT_MS - elapsed) / 1000;
    }
    doc["sensorHealthy"] = !sensorIsDead && (millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
    doc["sensorWarning"] = !sensorIsDead && (millis() - lastEspNowData >= ESP_NOW_DATA_TIMEOUT);
    if (lastEspNowData > 0) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%lus ago", (millis() - lastEspNowData)/1000);
      doc["sensorLastSeen"] = String(buf);
    }
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
    doc["inrush_enabled"] = inrush_enabled;
    doc["inrush_tolerance_ms"] = inrush_tolerance_ms;
    doc["overload_cooldown_seconds"] = overload_cooldown_seconds;
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
    doc["wifi_password"] = String(wifi_password);
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
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
      if (doc.containsKey("wifi_ssid")) {
        String ssid = doc["wifi_ssid"].as<String>();
        ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
      }
      if (doc.containsKey("wifi_password")) {
        String pwd = doc["wifi_password"].as<String>();
        pwd.toCharArray(wifi_password, sizeof(wifi_password));
      }
      saveConfigToFile();
      if (wifi_sta_enabled && strlen(wifi_ssid) > 0) connectToWiFi();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/scan_wifi", HTTP_GET, []() {
    String json = "[";
    int n = WiFi.scanComplete();
    if (n == -2) {
      WiFi.scanNetworks(true);
      json = "{\"scanning\":true}";
    } else if (n >= 0) {
      for (int i = 0; i < n; ++i) {
        if (i) json += ",";
        json += "{\"ssid\":\"" + WiFi.SSID(i) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
      }
      WiFi.scanDelete();
    }
    json += "]";
    server.send(200, "application/json", json);
  });
  
  server.on("/reset_lockout", HTTP_GET, []() { resetPressureLockout(); server.send(200, "text/plain", "OK"); });
  server.on("/mode", HTTP_GET, []() {
    if (server.hasArg("mode")) {
      bool newMode = (server.arg("mode") == "auto");
      if (auto_mode != newMode) {
        if (auto_mode && !newMode) {
          addFailureLogEntry("🌐 Web UI: Switching from AUTO to MANUAL mode");
          stopPumpAndCancelTimers("Web UI mode change");
        }
        auto_mode = newMode;
        if (auto_mode) {
          button.lastManualModeTime = 0;
          addFailureLogEntry("🌐 Web UI: Switched to AUTO mode");
        } else {
          button.lastManualModeTime = millis();
          addFailureLogEntry("🌐 Web UI: Switched to MANUAL mode");
        }
        saveConfigToFile();
      }
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/toggle", HTTP_GET, []() {
    if (!auto_mode) {
      if (pressureLockout.active) { server.send(403, "text/plain", "Lockout active"); return; }
      if (softStart.delayActive) { server.send(403, "text/plain", "Soft start active"); return; }
      if (millis() < overload.cooldownUntil) { server.send(403, "text/plain", "Cooldown active"); return; }
      if (millis() < dryRun.cooldownUntil) { server.send(403, "text/plain", "Dry run cooldown"); return; }
      bool newState = !s31.getRelayState();
      s31.setRelay(newState);
      button.lastManualModeTime = millis();
      if (newState) {
        cancelInchingTimer();
        cancelSoftStartDelay();
      }
      server.send(200, "text/plain", "OK");
    } else server.send(403, "text/plain", "In AUTO mode");
  });
  
  server.on("/config/protection", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("dry_run_enabled")) dry_run_enabled = doc["dry_run_enabled"];
      if (doc.containsKey("dry_run_protection")) pump_protection_time = constrain(doc["dry_run_protection"].as<uint32_t>(), 3, 300);
      if (doc.containsKey("min_power")) min_power_threshold = constrain(doc["min_power"].as<float>(), 0.0, 3500.0);
      if (doc.containsKey("dryrun_cooldown_seconds")) dryrun_cooldown_seconds = constrain(doc["dryrun_cooldown_seconds"].as<uint32_t>(), 0, 300);
      if (doc.containsKey("pump_load_protection_enabled")) pump_load_protection_enabled = doc["pump_load_protection_enabled"];
      if (doc.containsKey("max_power_threshold")) max_power_threshold = constrain(doc["max_power_threshold"].as<float>(), 10.0, 3500.0);
      if (doc.containsKey("overload_cooldown_seconds")) overload_cooldown_seconds = constrain(doc["overload_cooldown_seconds"].as<uint32_t>(), 0, 300);
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
      if (doc.containsKey("hostname")) {
        String nh = doc["hostname"].as<String>();
        if (nh.length() > 0) {
          nh.toCharArray(hostname, sizeof(hostname));
          deviceName = String(hostname);
        }
      }
      if (doc.containsKey("peer_mac")) {
        String macStr = doc["peer_mac"].as<String>();
        if (macStr.length() > 0 && stringToMac(macStr, peer_mac)) {
          macStr.toCharArray(peer_mac_str, sizeof(peer_mac_str));
        }
      }
      if (doc.containsKey("espnow_channel")) espnow_channel = constrain(doc["espnow_channel"].as<int>(), 1, 13);
      if (doc.containsKey("use_espnow")) use_espnow = doc["use_espnow"];
      if (doc.containsKey("auto_return_enabled")) auto_return_enabled = doc["auto_return_enabled"];
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
      delay(100);
      rebootDevice();
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
  while (!LittleFS.begin() && fsRetries < 3) {
    delay(500);
    fsRetries++;
  }
  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }
  
  loadConfigFromFile();
  
  deviceName = String(hostname);
  s31.begin();
  setupAPMode();
  
  if (wifi_sta_enabled && strlen(wifi_ssid) > 0) {
    connectToWiFi();
  }
  
  initEspNow();
  setupWebServer();
  setupArduinoOTA();
  initButton();
  initPressureSwitch();
  
  if (ENABLE_MDNS) {
    MDNS.begin(deviceName.c_str());
    MDNS.addService("http", "tcp", 80);
  }
  
  dryRun.lowPowerStartTime = 0;
  overload.lastRelayState = s31.getRelayState();
  overload.pumpStartTime = overload.lastRelayState ? millis() : 0;
  overload.bootInitialized = true;
  softStart.delayActive = false;
  inching.active = false;
  pressureLockout.active = false;
  if (!auto_mode) button.lastManualModeTime = millis();
  
  if (DEBUG_ENABLED) {
    Serial.println("\n╔═══════════════════════════════════════════════════════════════════════╗");
    Serial.println("║              SMART PUMP CONTROLLER v" FIRMWARE_VERSION " - SAFETY PRIORITY             ║");
    Serial.println("╠═══════════════════════════════════════════════════════════════════════╣");
    Serial.println("║  ✓ SAFETY FIRST: Dry Run & Overload CANCEL all timers                ║");
    Serial.println("║  ✓ AUTO → MANUAL: Stops pump & cancels ALL timers immediately        ║");
    Serial.println("║  ✓ Dry Run stops pump INSTANTLY (configurable 3-300s)                ║");
    Serial.println("║  ✓ Real-time power monitoring (V, A, W, kWh)                         ║");
    Serial.println("║  ✓ Virtual tank gauge with water level animation                     ║");
    Serial.println("║  ✓ Pressure Lockout prevents rapid cycling                          ║");
    Serial.println("║  ✓ Dual WiFi (STA + AP) with mDNS                                    ║");
    Serial.println("║  ✓ Reboot Countdown (5s) with cancel option                          ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════════╝\n");
  }
}

void loop() {
  uint32_t now = millis();
  
  feedSystemWatchdog();
  handleButton();
  readPressureSwitch();
  
  if (ENABLE_CAPTIVE_PORTAL) dnsServer.processNextRequest();
  if (now - lastS31Update >= S31_UPDATE_INTERVAL) { lastS31Update = now; s31.update(); }
  if (ENABLE_OTA && now - lastOTA >= OTA_INTERVAL) { lastOTA = now; ArduinoOTA.handle(); }
  if (now - lastWebServer >= WEB_SERVER_INTERVAL) { lastWebServer = now; server.handleClient(); }
  
  requestSensorData();
  controlPump();
  
  if (ENABLE_MDNS && now - lastMDNS >= MDNS_INTERVAL) { lastMDNS = now; MDNS.update(); }
  if (now - lastStatsSave >= SAVE_STATS_INTERVAL) { lastStatsSave = now; saveStatistics(); }
  if (ENABLE_MEMORY_MONITOR && now - lastMemoryCheck >= MEMORY_CHECK_INTERVAL) { lastMemoryCheck = now; checkMemoryAndCleanup(); }
  if (pendingSave && !saveInProgress && now - lastSaveTime >= MIN_SAVE_INTERVAL_MS) saveConfigToFile();
  if (ENABLE_WIFI_RECOVERY) recoverWiFiConnection();
  recoverEspNow();
  delay(1);
}

// ================================================================================================
// @section     END OF CODE - v3.5 COMPLETE WITH NICE UI
// ================================================================================================