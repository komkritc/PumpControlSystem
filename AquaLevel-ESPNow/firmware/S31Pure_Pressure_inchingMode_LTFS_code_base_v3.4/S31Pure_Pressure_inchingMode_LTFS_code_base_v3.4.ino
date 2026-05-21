/**
 * ================================================================================================
 * SMART PUMP CONTROLLER - PURE PRESSURE SWITCH MODE v3.4 (PRESSURE LOCKOUT EDITION)
 * ================================================================================================
 * @improvements v3.4:
 *   - ADDED: Pressure lockout timer (5 minutes to 3 days configurable)
 *   - ADDED: Lockout prevents pump cycling when pressure fluctuates
 *   - ADDED: Manual lockout reset button in UI
 *   - ADDED: Lockout status display with countdown timer
 *   - FIXED: Rapid on/off oscillation when pressure hovers near threshold
 *   - FIXED: Random reboot on settings save (CRITICAL FIX from v3.3)
 *   - Added save mutex to prevent concurrent file writes
 *   - Global JSON document (no stack overflow)
 *   - Fixed ring buffer for logs (no heap fragmentation)
 *   - Rate limiting for file saves (min 2 seconds between saves)
 *   - Yield() and watchdog during file operations
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
#define FIRMWARE_VERSION        "3.4"
#define FIRMWARE_DATE           "2026-05-19"
#define ENABLE_MDNS             false
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
#define MIN_SAVE_INTERVAL_MS    2000    // Minimum 2 seconds between saves

// ================================================================================================
// @section     PRESSURE LOCKOUT CONSTANTS
// ================================================================================================
#define DEFAULT_PRESSURE_LOCKOUT_HOURS    2.0   // Default 2 hours
#define MIN_PRESSURE_LOCKOUT_MINUTES      5     // Minimum 5 minutes
#define MAX_PRESSURE_LOCKOUT_HOURS        72    // Maximum 3 days (72 hours)

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

// ================================================================================================
// @section     GLOBAL VARIABLES
// ================================================================================================
SonoffS31 s31(RELAY_PIN);
ESP8266WebServer server(80);
DNSServer dnsServer;
String deviceName = "s31-pump";
Ticker watchdogTicker;
Ticker memoryMonitorTicker;

// Save protection flags
bool saveInProgress = false;
uint32_t lastSaveTime = 0;
bool pendingSave = false;

// Global JSON document to avoid stack allocation
StaticJsonDocument<2048> configDoc;

// Fixed ring buffer for logs instead of vector (prevents heap fragmentation)
String failureLog[MAX_LOG_ENTRIES];
uint8_t logIndex = 0;
uint8_t logCount = 0;

// System uptime tracking
uint32_t systemStartTime = 0;
uint32_t lastUptimeUpdate = 0;
uint32_t wifiConnectAttempts = 0;
uint32_t lastWifiRecovery = 0;
uint32_t lastEspNowRecovery = 0;
uint32_t lastMemoryCheck = 0;
uint32_t lastEmergencySave = 0;
bool wifiRecoveryInProgress = false;

// Configuration variables (will be loaded from/saved to JSON file)
bool auto_mode = true;
bool dry_run_enabled = true;
float min_power_threshold = 10.0;
uint32_t pump_protection_time = 20;
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

// Pressure lockout configuration (in hours, can be fractional like 0.083 = 5 minutes)
float pressure_lockout_hours = 2.0;  // Default 2 hours

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

// ================================================================================================
// @section     TIMING VARIABLES
// ================================================================================================
uint32_t lastS31Update = 0;
uint32_t lastControlCheck = 0;
uint32_t lastWebServer = 0;
uint32_t lastOTA = 0;
uint32_t lastMDNS = 0;
uint32_t lastStatsSave = 0;
uint32_t lastWdtFeed = 0;

// ================================================================================================
// @section     SENSOR DATA
// ================================================================================================
float currentWaterLevel = 0;
float currentDistance = 0;
float currentVolume = 0;
float batteryVoltage = 0;
uint32_t lastEspNowData = 0;
bool espnowDataValid = false;
bool sensorIsDead = false;
float powerFactor = 0.0;
float apparentPower = 0.0;

// ================================================================================================
// @section     ESP-NOW
// ================================================================================================
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
  #define DEBUG_LOG(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[DEBUG] " + String(msg)); Serial.println("[DEBUG] " + String(msg)); }
  #define DEBUG_OVERLOAD(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[OVERLOAD] " + String(msg)); Serial.println("[OVERLOAD] " + String(msg)); }
  #define DEBUG_DRYRUN(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[DRYRUN] " + String(msg)); Serial.println("[DRYRUN] " + String(msg)); }
  #define DEBUG_BUTTON(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[BUTTON] " + String(msg)); Serial.println("[BUTTON] " + String(msg)); }
  #define DEBUG_PRESSURE(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[PRESSURE] " + String(msg)); Serial.println("[PRESSURE] " + String(msg)); }
  #define DEBUG_SOFTSTART(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[SOFTSTART] " + String(msg)); Serial.println("[SOFTSTART] " + String(msg)); }
  #define DEBUG_MODE(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[MODE] " + String(msg)); Serial.println("[MODE] " + String(msg)); }
  #define DEBUG_FS(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[FS] " + String(msg)); Serial.println("[FS] " + String(msg)); }
  #define DEBUG_ESPNOW(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[ESPNOW] " + String(msg)); Serial.println("[ESPNOW] " + String(msg)); }
  #define DEBUG_INCHING(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[INCHING] " + String(msg)); Serial.println("[INCHING] " + String(msg)); }
  #define DEBUG_INRUSH(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[INRUSH] " + String(msg)); Serial.println("[INRUSH] " + String(msg)); }
  #define DEBUG_SYSTEM(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[SYSTEM] " + String(msg)); Serial.println("[SYSTEM] " + String(msg)); }
  #define DEBUG_LOCKOUT(msg) if(DEBUG_ENABLED) { addFailureLogEntry("[LOCKOUT] " + String(msg)); Serial.println("[LOCKOUT] " + String(msg)); }
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
#endif

// ================================================================================================
// @section     SYSTEM MONITORING FUNCTIONS
// ================================================================================================

// Feed watchdog and check system health
void feedSystemWatchdog() {
  uint32_t now = millis();
  if (now - lastWdtFeed >= WATCHDOG_FEED_INTERVAL) {
    ESP.wdtFeed();
    lastWdtFeed = now;
    DEBUG_SYSTEM("Watchdog fed");
  }
}

// Check available memory and cleanup if needed
void checkMemoryAndCleanup() {
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxFreeBlock = ESP.getMaxFreeBlockSize();
  
  DEBUG_SYSTEM("Memory - Free: " + String(freeHeap) + " bytes, Max Block: " + String(maxFreeBlock) + " bytes");
  
  // Critical low memory condition
  if (freeHeap < 8192) {
    addFailureLogEntry("⚠️ CRITICAL LOW MEMORY: " + String(freeHeap) + " bytes");
    
    // Force garbage collection - clear logs if needed
    if (logCount > MAX_LOG_ENTRIES / 2) {
      logCount = MAX_LOG_ENTRIES / 4;
      logIndex = logCount;
      DEBUG_SYSTEM("Truncated failure log to free memory");
    }
  }
}

// Emergency save for critical situations
void emergencySave() {
  uint32_t now = millis();
  if (now - lastEmergencySave > 30000 && !saveInProgress) { // Max once per 30 seconds
    lastEmergencySave = now;
    DEBUG_SYSTEM("Emergency save triggered");
    saveConfigToFile();
  }
}

// Recover WiFi connection if lost
void recoverWiFiConnection() {
  if (!ENABLE_WIFI_RECOVERY) return;
  if (wifiRecoveryInProgress) return;
  
  uint32_t now = millis();
  if (now - lastWifiRecovery < WIFI_RECOVERY_INTERVAL) return;
  
  if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
    wifiRecoveryInProgress = true;
    wifiConnectAttempts++;
    addFailureLogEntry("WiFi lost, attempting recovery #" + String(wifiConnectAttempts));
    
    // Reset WiFi
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_AP_STA);
    
    // Try to reconnect
    if (WiFi.SSID() != "") {
      WiFi.begin();
      delay(500);
    }
    
    lastWifiRecovery = now;
    wifiRecoveryInProgress = false;
  }
}

// Recover ESP-NOW if needed
void recoverEspNow() {
  if (!use_espnow) return;
  
  uint32_t now = millis();
  if (now - lastEspNowRecovery < ESP_NOW_RECOVERY_INTERVAL) return;
  
  // Check if ESP-NOW is working (data received recently)
  bool dataStale = (millis() - lastEspNowData > ESP_NOW_DATA_TIMEOUT * 2);
  
  if (!espnow_initialized || (dataStale && lastEspNowData > 0)) {
    addFailureLogEntry("ESP-NOW recovery triggered");
    DEBUG_ESPNOW("Reinitializing ESP-NOW");
    
    // Reinitialize ESP-NOW
    esp_now_deinit();
    delay(100);
    initEspNow();
    lastEspNowRecovery = now;
  }
}

// ================================================================================================
// @section     FILESYSTEM + JSON FUNCTIONS
// ================================================================================================

// Create backup of config file
void backupConfigFile() {
  if (LittleFS.exists(CONFIG_FILE)) {
    File src = LittleFS.open(CONFIG_FILE, "r");
    if (src) {
      File dst = LittleFS.open(CONFIG_BACKUP_FILE, "w");
      if (dst) {
        while (src.available()) {
          dst.write(src.read());
        }
        dst.close();
        DEBUG_FS("Config backup created");
      }
      src.close();
    }
  }
}

// Verify config file contents
void verifyConfigSave() {
  if (LittleFS.exists(CONFIG_FILE)) {
    File file = LittleFS.open(CONFIG_FILE, "r");
    if (file) {
      configDoc.clear();
      DeserializationError error = deserializeJson(configDoc, file);
      if (!error) {
        uint32_t savedTolerance = configDoc["inrush_tolerance_ms"] | 0;
        bool savedInrushEnabled = configDoc["inrush_enabled"] | false;
        DEBUG_FS("Verified in file - Inrush: " + String(savedInrushEnabled ? "ON" : "OFF") + 
                 ", Tolerance: " + String(savedTolerance) + "ms");
      }
      file.close();
    }
  }
}

// Save all settings to LittleFS as JSON
void saveConfigToFile() {
  // Prevent concurrent saves
  if (saveInProgress) {
    DEBUG_FS("Save already in progress, skipping...");
    pendingSave = true;
    return;
  }
  
  // Rate limiting
  uint32_t now = millis();
  if (now - lastSaveTime < MIN_SAVE_INTERVAL_MS) {
    DEBUG_FS("Save too frequent, queuing...");
    pendingSave = true;
    return;
  }
  
  saveInProgress = true;
  lastSaveTime = now;
  
  DEBUG_FS("Saving configuration to " + String(CONFIG_FILE));
  
  // Feed watchdog before file operation
  yield();
  ESP.wdtFeed();
  
  // Clear and rebuild JSON document
  configDoc.clear();
  
  // Add all settings
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
  
  // Open file for writing
  File file = LittleFS.open(CONFIG_FILE, "w");
  if (!file) {
    DEBUG_FS("Failed to open file for writing");
    saveInProgress = false;
    return;
  }
  
  // Feed watchdog before serialization
  yield();
  ESP.wdtFeed();
  
  // Serialize JSON to file
  size_t bytesWritten = serializeJson(configDoc, file);
  
  // Flush to ensure data is written
  file.flush();
  
  // Feed watchdog after write
  yield();
  ESP.wdtFeed();
  
  if (bytesWritten == 0) {
    DEBUG_FS("Failed to write JSON to file");
    backupConfigFile();
  } else {
    char msg[96];
    snprintf(msg, sizeof(msg), "Settings saved (%u bytes, lockout=%.1fh)", 
             bytesWritten, pressure_lockout_hours);
    addFailureLogEntry(msg);
  }
  
  file.close();
  
  // Verify the save
  verifyConfigSave();
  
  saveInProgress = false;
  pendingSave = false;
}

// Load all settings from LittleFS JSON
void loadConfigFromFile() {
  DEBUG_FS("Loading configuration from " + String(CONFIG_FILE));
  
  // SAFE CHECK - No temporary file objects that can leak!
  bool configInvalid = false;
  
  if (!LittleFS.exists(CONFIG_FILE)) {
    configInvalid = true;
  } else {
    File testFile = LittleFS.open(CONFIG_FILE, "r");
    if (!testFile || testFile.size() == 0) {
      configInvalid = true;
    }
    testFile.close();
  }
  
  // Try backup if main config is corrupted
  if (configInvalid) {
    DEBUG_FS("Config file missing or empty, trying backup");
    if (LittleFS.exists(CONFIG_BACKUP_FILE)) {
      File backup = LittleFS.open(CONFIG_BACKUP_FILE, "r");
      if (backup && backup.size() > 0) {
        File main = LittleFS.open(CONFIG_FILE, "w");
        if (main) {
          while (backup.available()) {
            main.write(backup.read());
          }
          main.close();
          DEBUG_FS("Restored config from backup");
        }
        backup.close();
      }
    }
    
    // If still invalid, use defaults
    if (!LittleFS.exists(CONFIG_FILE)) {
      DEBUG_FS("Config file not found, using defaults");
      saveConfigToFile();
      return;
    }
  }
  
  // Open file for reading
  File file = LittleFS.open(CONFIG_FILE, "r");
  if (!file) {
    DEBUG_FS("Failed to open file for reading");
    return;
  }
  
  // Parse JSON using global document
  configDoc.clear();
  DeserializationError error = deserializeJson(configDoc, file);
  file.close();
  
  if (error) {
    DEBUG_FS("Failed to parse JSON: " + String(error.c_str()));
    return;
  }
  
  // Load all settings with error checking
  auto_mode = configDoc["auto_mode"] | true;
  dry_run_enabled = configDoc["dry_run_enabled"] | true;
  min_power_threshold = configDoc["min_power_threshold"] | 10.0;
  pump_protection_time = configDoc["pump_protection_time"] | 20;
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
  
  // Load pressure lockout hours
  pressure_lockout_hours = configDoc["pressure_lockout_hours"] | 2.0;
  // Constrain to valid range (0 = disabled, or between min and max)
  if (pressure_lockout_hours < 0) pressure_lockout_hours = 0;
  if (pressure_lockout_hours > 0 && pressure_lockout_hours < (MIN_PRESSURE_LOCKOUT_MINUTES / 60.0)) {
    pressure_lockout_hours = MIN_PRESSURE_LOCKOUT_MINUTES / 60.0;
  }
  if (pressure_lockout_hours > MAX_PRESSURE_LOCKOUT_HOURS) pressure_lockout_hours = MAX_PRESSURE_LOCKOUT_HOURS;
  
  // Load string values
  String hostnameStr = configDoc["hostname"] | "s31-pump";
  if (hostnameStr.length() == 0) hostnameStr = "s31-pump";
  hostnameStr.toCharArray(hostname, sizeof(hostname));
  
  String peerMacStr = configDoc["peer_mac"] | "FF:FF:FF:FF:FF:FF";
  if (peerMacStr.length() == 0) peerMacStr = "FF:FF:FF:FF:FF:FF";
  peerMacStr.toCharArray(peer_mac_str, sizeof(peer_mac_str));
  stringToMac(peerMacStr, peer_mac);
  
  // Validate and constrain
  min_power_threshold = constrain(min_power_threshold, 0.0, 3500.0);
  pump_protection_time = constrain(pump_protection_time, 5, 300);
  max_power_threshold = constrain(max_power_threshold, 10.0, 3500.0);
  overload_cooldown_seconds = constrain(overload_cooldown_seconds, 0, 300);
  dryrun_cooldown_seconds = constrain(dryrun_cooldown_seconds, 0, 300);
  inrush_tolerance_ms = constrain(inrush_tolerance_ms, 500, 5000);
  soft_start_delay_ms = constrain(soft_start_delay_ms, 0, 10000);
  espnow_channel = constrain(espnow_channel, 1, 13);
  inching_duration_minutes = constrain(inching_duration_minutes, 1, 60);
  
  // Sync state variables
  overload.overloadCount = overload_events;
  dryRun.events = dryrun_events;
  softStart.delayCount = soft_start_count;
  button.shortPressCount = button_press_count;
  
  DEBUG_FS("Configuration loaded successfully - Lockout: " + String(pressure_lockout_hours) + " hours");
}

// Save statistics periodically
void saveStatistics() {
  // Don't save if already saving
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

// Factory reset - delete config file
void factoryReset() {
  DEBUG_LOG("Factory reset initiated");
  
  if (LittleFS.exists(CONFIG_FILE)) {
    LittleFS.remove(CONFIG_FILE);
    DEBUG_LOG("Config file deleted");
  }
  if (LittleFS.exists(CONFIG_BACKUP_FILE)) {
    LittleFS.remove(CONFIG_BACKUP_FILE);
    DEBUG_LOG("Backup file deleted");
  }
  
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
  
  // Use ring buffer instead of vector
  failureLog[logIndex] = entry;
  logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
  if (logCount < MAX_LOG_ENTRIES) logCount++;
  
  if (DEBUG_ENABLED) {
    Serial.println(entry);
  }
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
  
  // Ensure all data is written
  if (saveInProgress) {
    addFailureLogEntry("Waiting for save to complete before reboot...");
    uint32_t waitStart = millis();
    while (saveInProgress && (millis() - waitStart < 3000)) {
      yield();
      delay(10);
    }
  }
  
  // Give time for flush to complete
  delay(500);
  
  addFailureLogEntry("System rebooting...");
  delay(100);
  ESP.restart(); 
}

String getUptimeString() {
  uint32_t uptimeSeconds = (millis() - systemStartTime) / 1000;
  uint32_t days = uptimeSeconds / 86400;
  uint32_t hours = (uptimeSeconds % 86400) / 3600;
  uint32_t minutes = (uptimeSeconds % 3600) / 60;
  uint32_t seconds = uptimeSeconds % 60;
  
  char buf[32];
  if (days > 0) {
    snprintf(buf, sizeof(buf), "%lud %luh %lum", days, hours, minutes);
  } else if (hours > 0) {
    snprintf(buf, sizeof(buf), "%luh %lum %lus", hours, minutes, seconds);
  } else if (minutes > 0) {
    snprintf(buf, sizeof(buf), "%lum %lus", minutes, seconds);
  } else {
    snprintf(buf, sizeof(buf), "%lus", seconds);
  }
  return String(buf);
}

// ================================================================================================
// @section     INCHING MODE FUNCTIONS
// ================================================================================================

void startInchingTimer() {
  if (inching_enabled && inching_duration_minutes > 0) {
    inching.active = true;
    inching.startTime = millis();
    inching.duration = inching_duration_minutes * 60 * 1000;
    inching.pendingStop = false;
    
    char msg[96];
    snprintf(msg, sizeof(msg), "⏱️ INCHING MODE ACTIVE: Pump will stop after %lu minute%s", 
             inching_duration_minutes, inching_duration_minutes > 1 ? "s" : "");
    addFailureLogEntry(msg);
    DEBUG_INCHING("Timer started: " + String(inching_duration_minutes) + " minutes");
  }
}

void cancelInchingTimer() {
  if (inching.active) {
    inching.active = false;
    inching.pendingStop = false;
    addFailureLogEntry("⏱️ INCHING MODE CANCELLED: Timer stopped");
    DEBUG_INCHING("Timer cancelled");
  }
}

void checkInchingTimer() {
  if (!inching.active) return;
  
  if (millis() - inching.startTime >= inching.duration) {
    if (!inching.pendingStop) {
      inching.pendingStop = true;
      addFailureLogEntry("⏱️ INCHING MODE: Auto-stopping pump");
      DEBUG_INCHING("Timer expired - stopping pump");
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
      pressureSwitch.lastChangeTime = now;
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
// @section     PRESSURE LOCKOUT - NEW FEATURE v3.4
// ================================================================================================

// Reset pressure lockout manually
void resetPressureLockout() {
  pressureLockout.active = false;
  pressureLockout.lockoutUntil = 0;
  pressureLockout.lastHighPressureTime = 0;
  pressureLockout.waitingForLowPressure = false;
  addFailureLogEntry("🔓 Pressure lockout manually reset by user");
  DEBUG_LOCKOUT("Lockout manually reset");
}

// Check if pump should be allowed to run based on lockout
bool isPressureLockoutActive() {
  uint32_t now = millis();
  
  // If lockout is active, check if it has expired
  if (pressureLockout.active) {
    if (now >= pressureLockout.lockoutUntil) {
      // Lockout expired
      pressureLockout.active = false;
      pressureLockout.waitingForLowPressure = false;
      addFailureLogEntry("🔓 Pressure lockout expired - System re-enabled");
      DEBUG_LOCKOUT("Lockout expired");
      return false;
    }
    return true;  // Still in lockout
  }
  
  return false;
}

// Update lockout state based on pressure
void updatePressureLockout(bool pressureLow) {
  uint32_t now = millis();
  
  // If lockout is active, don't update until it expires
  if (pressureLockout.active) {
    return;
  }
  
  // If pressure is HIGH (pump should be OFF)
  if (!pressureLow) {
    // Pressure is high - record this time and start lockout
    if (pressureLockout.lastHighPressureTime == 0) {
      pressureLockout.lastHighPressureTime = now;
      pressureLockout.waitingForLowPressure = false;
      
      // Calculate lockout duration in milliseconds
      uint32_t lockoutMs = (uint32_t)(pressure_lockout_hours * 3600000.0);
      
      // Ensure minimum lockout time
      if (pressure_lockout_hours > 0 && lockoutMs < (MIN_PRESSURE_LOCKOUT_MINUTES * 60000)) {
        lockoutMs = MIN_PRESSURE_LOCKOUT_MINUTES * 60000;
      }
      
      pressureLockout.lockoutUntil = now + lockoutMs;
      pressureLockout.active = true;
      
      char msg[96];
      if (pressure_lockout_hours >= 24) {
        float days = pressure_lockout_hours / 24.0;
        snprintf(msg, sizeof(msg), "🔒 Pressure HIGH - Lockout for %.1f day%s", 
                 days, days > 1 ? "s" : "");
      } else if (pressure_lockout_hours >= 1) {
        snprintf(msg, sizeof(msg), "🔒 Pressure HIGH - Lockout for %.1f hour%s", 
                 pressure_lockout_hours, pressure_lockout_hours > 1 ? "s" : "");
      } else {
        uint32_t minutes = (uint32_t)(pressure_lockout_hours * 60);
        snprintf(msg, sizeof(msg), "🔒 Pressure HIGH - Lockout for %lu minute%s", 
                 minutes, minutes > 1 ? "s" : "");
      }
      addFailureLogEntry(msg);
      DEBUG_LOCKOUT("Lockout started for " + String(lockoutMs/1000) + " seconds");
    }
  } else {
    // Pressure is LOW - reset the high pressure timer
    if (pressureLockout.lastHighPressureTime > 0) {
      pressureLockout.lastHighPressureTime = 0;
      pressureLockout.waitingForLowPressure = true;
      DEBUG_LOCKOUT("Pressure LOW detected, ready to run after lockout expires");
    }
  }
}

// Get remaining lockout time in seconds
uint32_t getLockoutRemainingSeconds() {
  if (!pressureLockout.active) return 0;
  uint32_t now = millis();
  if (now >= pressureLockout.lockoutUntil) return 0;
  return (pressureLockout.lockoutUntil - now) / 1000;
}

// ================================================================================================
// @section     SOFT START
// ================================================================================================
bool shouldApplySoftStartDelay(bool desiredState, bool currentState) {
  if (!soft_start_enabled) return false;
  if (!desiredState) return false;
  if (currentState) return false;
  return true;
}

void startSoftStartDelay(bool desiredState) {
  if (!softStart.delayActive) {
    softStart.delayActive = true;
    softStart.delayStartTime = millis();
    softStart.pendingPumpState = desiredState;
    softStart.delayCount++;
    soft_start_count = softStart.delayCount;
    char msg[64];
    snprintf(msg, sizeof(msg), "SOFT START: %lu ms delay", soft_start_delay_ms);
    addFailureLogEntry(msg);
  }
}

bool updateSoftStartState(bool desiredState) {
  if (!softStart.delayActive) {
    if (shouldApplySoftStartDelay(desiredState, s31.getRelayState())) {
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
  if (softStart.delayActive) softStart.delayActive = false;
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
}

// ================================================================================================
// @section     ESP-NOW
// ================================================================================================
void parseEspNowData(String data) {
  DEBUG_ESPNOW("Received: " + data);
  
  float d = 0, l = 0, v = 0, b = 0;
  
  // Use safer parsing with bounds checking
  int dIndex = data.indexOf("\"d\":");
  if (dIndex != -1 && dIndex + 4 < data.length()) { 
    int start = dIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1 && end <= data.length()) {
      String sub = data.substring(start, end);
      d = sub.toFloat();
    }
  }
  
  int lIndex = data.indexOf("\"l\":");
  if (lIndex != -1 && lIndex + 4 < data.length()) { 
    int start = lIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1 && end <= data.length()) {
      String sub = data.substring(start, end);
      l = sub.toFloat();
    }
  }
  
  int vIndex = data.indexOf("\"v\":");
  if (vIndex != -1 && vIndex + 4 < data.length()) { 
    int start = vIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1 && end <= data.length()) {
      String sub = data.substring(start, end);
      v = sub.toFloat();
    }
  }
  
  int bIndex = data.indexOf("\"b\":");
  if (bIndex != -1 && bIndex + 4 < data.length()) { 
    int start = bIndex + 4;
    int end = data.indexOf(",", start);
    if (end == -1) end = data.indexOf("}", start);
    if (end != -1 && end <= data.length()) {
      String sub = data.substring(start, end);
      b = sub.toFloat();
    }
  }
  
  if (d > 0 || l > 0 || v > 0 || b > 0) {
    currentDistance = d;
    currentWaterLevel = l;
    currentVolume = v;
    batteryVoltage = b;
    lastEspNowData = millis();
    espnowDataValid = true;
    sensorIsDead = false;
    char msg[64];
    snprintf(msg, sizeof(msg), "Sensor Data - Level: %.1f%%, Battery: %.2fV", l, b);
    DEBUG_ESPNOW(msg);
  }
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  DEBUG_ESPNOW("Send status: " + String(sendStatus));
}

void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len != sizeof(EspNowPacket)) { 
    DEBUG_ESPNOW("Invalid packet size");
    return; 
  }
  
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
    DEBUG_ESPNOW("ESP-NOW init failed");
    return; 
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  uint8_t* peerMac = broadcastMac;
  if (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) peerMac = peer_mac;
  
  if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_COMBO, espnow_channel, NULL, 0) != 0) { 
    espnow_initialized = false; 
    DEBUG_ESPNOW("Add peer failed");
    return; 
  }
  
  espnow_initialized = true;
  DEBUG_ESPNOW("ESP-NOW initialized, MAC: " + macToString(peerMac));
}

void sendEspNowCommand(const char* command) {
  if (!espnow_initialized) return;
  
  for (uint8_t retry = 0; retry < ESP_NOW_RETRY_COUNT; retry++) {
    outgoing.seq = espnow_seq++;
    outgoing.timestamp = micros();
    strncpy(outgoing.msg, command, sizeof(outgoing.msg)-1);
    
    uint8_t* peerMac = broadcastMac;
    if (peer_mac[0] != 0xFF && peer_mac[0] != 0x00) peerMac = peer_mac;
    
    if (esp_now_send(peerMac, (uint8_t *)&outgoing, sizeof(outgoing)) == 0) {
      DEBUG_ESPNOW("Command sent: " + String(command));
      return;
    }
    delay(50);
  }
  DEBUG_ESPNOW("Failed to send command after retries: " + String(command));
}

void requestSensorData() {
  if (!espnow_initialized) return;
  if (millis() - lastEspNowSend >= ESP_NOW_SEND_INTERVAL) {
    lastEspNowSend = millis();
    sendEspNowCommand("get_measure");
  }
}

// ================================================================================================
// @section     OVERLOAD PROTECTION
// ================================================================================================
bool detectPumpStartEvent(bool currentState) {
  if (!overload.bootInitialized) return false;
  if (currentState && !overload.lastRelayState) return true;
  if (currentState && !overload.inrushActive && overload.pumpStartTime == 0 && millis() < 5000) return true;
  return false;
}

void updateInrushState(bool currentState) {
  if (!inrush_enabled) {
    overload.inrushActive = false;
    overload.lastRelayState = currentState;
    return;
  }
  
  if (detectPumpStartEvent(currentState)) {
    overload.pumpStartTime = millis();
    overload.inrushActive = true;
    char msg[64];
    snprintf(msg, sizeof(msg), "Inrush active for %lu ms", inrush_tolerance_ms);
    DEBUG_INRUSH(msg);
  }
  
  if (overload.inrushActive && currentState && (millis() - overload.pumpStartTime >= inrush_tolerance_ms)) {
    overload.inrushActive = false;
    DEBUG_INRUSH("Inrush ended");
  }
  
  if (!currentState && overload.inrushActive) overload.inrushActive = false;
  overload.lastRelayState = currentState;
}

bool handleOverloadCooldownPeriod(bool currentState) {
  if (overload.cooldownUntil > millis()) {
    if (currentState) s31.setRelay(false);
    return true;
  }
  if (overload.active && millis() >= overload.cooldownUntil) {
    overload.active = false;
    addFailureLogEntry("Overload cooldown ended");
  }
  return false;
}

bool checkSustainedOverload(float currentPower, bool currentState) {
  if (!pump_load_protection_enabled || !currentState) return false;
  if (inrush_enabled && overload.inrushActive) return false;
  
  if (currentPower > max_power_threshold && !overload.active) {
    if (millis() - overload.pumpStartTime > 500) {
      overload.active = true;
      overload.cooldownUntil = millis() + (overload_cooldown_seconds * 1000);
      overload.overloadCount++;
      overload_events = overload.overloadCount;
      
      char msg[96];
      snprintf(msg, sizeof(msg), "OVERLOAD! %.1fW - Cooldown: %lu s", currentPower, overload_cooldown_seconds);
      addFailureLogEntry(msg);
      s31.setRelay(false);
      return true;
    }
  }
  return false;
}

bool checkOverloadProtection(float currentPower, bool currentState) {
  updateInrushState(currentState);
  if (handleOverloadCooldownPeriod(currentState)) return false;
  if (checkSustainedOverload(currentPower, currentState)) return false;
  return true;
}

// ================================================================================================
// @section     DRY RUN PROTECTION
// ================================================================================================
bool handleDryRunCooldownPeriod(bool currentState) {
  if (dryRun.cooldownUntil > millis()) {
    if (currentState) {
      s31.setRelay(false);
      DEBUG_DRYRUN("Dry run cooldown active: " + String((dryRun.cooldownUntil - millis()) / 1000) + "s remaining");
    }
    return true;
  }
  
  if (dryRun.inCooldown && millis() >= dryRun.cooldownUntil) {
    dryRun.inCooldown = false;
    dryRun.protectionTriggered = false;
    addFailureLogEntry("Dry run cooldown ended - Pump can restart");
  }
  return false;
}

bool checkDryRunProtection(float currentPower, bool currentState) {
  if (handleDryRunCooldownPeriod(currentState)) return true;
  if (!dry_run_enabled) { dryRun.lowPowerStartTime = 0; return false; }
  if (!currentState) { dryRun.lowPowerStartTime = 0; return false; }
  if (inrush_enabled && overload.inrushActive) return false;
  if (min_power_threshold <= 0.1) { dryRun.lowPowerStartTime = 0; return false; }
  
  if (currentPower < min_power_threshold) {
    if (dryRun.lowPowerStartTime == 0) dryRun.lowPowerStartTime = millis();
    
    if ((millis() - dryRun.lowPowerStartTime) / 1000 >= pump_protection_time) {
      char msg[96];
      snprintf(msg, sizeof(msg), "DRY RUN! Pump stopped - Cooldown: %lu s", dryrun_cooldown_seconds);
      addFailureLogEntry(msg);
      dryRun.protectionTriggered = true;
      dryRun.inCooldown = true;
      dryRun.cooldownUntil = millis() + (dryrun_cooldown_seconds * 1000);
      dryRun.events++;
      dryrun_events = dryRun.events;
      return true;
    }
  } else {
    if (dryRun.lowPowerStartTime != 0) dryRun.lowPowerStartTime = 0;
  }
  return false;
}

// ================================================================================================
// @section     PUMP CONTROL (UPDATED WITH LOCKOUT)
// ================================================================================================
void updatePumpStatistics(bool currentState) {
  static uint32_t lastRuntimeUpdate = 0;
  uint32_t now = millis();
  
  if (currentState && !overload.lastRelayState) {
    pump_cycles++;
    lastRuntimeUpdate = now;
  } 
  else if (!currentState && overload.lastRelayState && lastRuntimeUpdate > 0) {
    total_runtime_seconds += (now - lastRuntimeUpdate) / 1000;
    total_energy_kwh = s31.getEnergy();
  }
}

void checkAutoReturnToAutoMode() {
  if (auto_return_enabled && !auto_mode && button.lastManualModeTime > 0) {
    if (millis() - button.lastManualModeTime >= AUTO_RETURN_TIMEOUT_MS) {
      auto_mode = true;
      saveConfigToFile();
      addFailureLogEntry("🔄 Auto-return: Switched to AUTO mode (10 min timeout)");
      button.lastManualModeTime = 0;
    }
  }
}

bool getDesiredPumpState(bool currentState) {
  if (!auto_mode) return currentState;
  
  bool pressureLow = pressureAllowsPumpOn();
  
  // Update lockout state based on pressure
  updatePressureLockout(pressureLow);
  
  // Check if lockout is active
  if (isPressureLockoutActive()) {
    // In lockout - pump must stay OFF
    return false;
  }
  
  // No lockout - normal pressure-based operation
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
    if (currentState != s31.getRelayState()) {
      cancelSoftStartDelay();
      cancelInchingTimer();
    }
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  if (checkDryRunProtection(currentPower, currentState)) {
    s31.setRelay(false);
    cancelSoftStartDelay();
    cancelInchingTimer();
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  bool desiredState = getDesiredPumpState(currentState);
  bool finalState = updateSoftStartState(desiredState);
  
  if (finalState && !currentState && inching_enabled) startInchingTimer();
  if (!finalState && currentState && inching.active) cancelInchingTimer();
  
  if (finalState != currentState) {
    s31.setRelay(finalState);
    const char* reason;
    if (inching.active) reason = "⏱️ INCHING MODE";
    else if (softStart.delayActive) reason = "🌊 SOFT START";
    else if (!auto_mode) reason = "👆 Manual control";
    else if (pressureLockout.active) reason = "🔒 PRESSURE LOCKOUT";
    else if (pressureSwitch.pressureLow) reason = "🔘 Pressure LOW";
    else reason = "🔘 Pressure OK";
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Pump %s - %s", finalState ? "ON" : "OFF", reason);
    addFailureLogEntry(msg);
  }
  
  if (softStart.delayActive && desiredState != softStart.pendingPumpState) {
    if (!desiredState) {
      cancelSoftStartDelay();
      cancelInchingTimer();
    } else {
      softStart.pendingPumpState = desiredState;
    }
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
    addFailureLogEntry("Button: Pump in overload cooldown");
    flashLed(1000);
    return;
  }
  
  if (dryRun.cooldownUntil > millis()) {
    addFailureLogEntry("Button: Pump in dry run cooldown");
    flashLed(1000);
    return;
  }
  
  if (softStart.delayActive) {
    addFailureLogEntry("Soft start active - Please wait");
    flashLed(500);
    return;
  }
  
  if (auto_mode) {
    if (s31.getRelayState()) {
      s31.setRelay(false);
      cancelInchingTimer();
      addFailureLogEntry("🔘 AUTO→MANUAL: Pump turned OFF");
    }
    auto_mode = false;
    saveConfigToFile();
    button.lastManualModeTime = millis();
    addFailureLogEntry("Short press: Switched to MANUAL mode (pump OFF)");
    flashLed(100);
    delay(100);
    flashLed(100);
  } else {
    button.lastManualModeTime = millis();
    bool newState = !s31.getRelayState();
    s31.setRelay(newState);
    char msg[64];
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
    addFailureLogEntry("Long press: Switched to AUTO mode (Pressure control)");
    for (int i = 0; i < 3; i++) { flashLed(100); delay(150); }
  } else {
    if (s31.getRelayState()) {
      s31.setRelay(false);
      cancelInchingTimer();
      addFailureLogEntry("Long press: Switched to MANUAL mode - Pump OFF");
    } else {
      addFailureLogEntry("Long press: Switched to MANUAL mode");
    }
    button.lastManualModeTime = millis();
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
        
        if (pressDuration < SHORT_PRESS_MAX_MS) {
          handleShortPress();
        } else if (pressDuration >= LONG_PRESS_MIN_MS && pressDuration <= LONG_PRESS_MAX_MS) {
          handleLongPress();
        }
        button.buttonPressed = false;
      }
    }
  }
  
  button.lastButtonState = reading;
}

// ================================================================================================
// @section     WEB SERVER SETUP
// ================================================================================================
void setupAPMode() {
  String apSSID = "SmartPump-" + String(ESP.getChipId() & 0xFFFF, HEX);
  WiFi.softAP(apSSID.c_str(), "12345678");
  if (DEBUG_ENABLED) {
    Serial.println("AP Mode: " + apSSID);
    Serial.println("IP Address: 192.168.4.1");
  }
  
  if (ENABLE_CAPTIVE_PORTAL) {
    dnsServer.start(53, "*", IPAddress(192, 168, 4, 1));
  }
}

void setupArduinoOTA() {
  if (!ENABLE_OTA) return;
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.onStart([]() { if(DEBUG_ENABLED) Serial.println("OTA Update..."); });
  ArduinoOTA.onEnd([]() { if(DEBUG_ENABLED) Serial.println("OTA Complete"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    if (DEBUG_ENABLED && p % (t/10) == 0) Serial.printf("OTA: %u%%\r\n", (p * 100) / t);
  });
  ArduinoOTA.begin();
}

// ================================================================================================
// @section     HTML UI - v3.4 WITH LOCKOUT DISPLAY
// ================================================================================================
const char simple_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>Smart Pump | v3.4</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Roboto, sans-serif; background: #f1f5f9; padding: 16px; color: #0f172a; }
        .container { max-width: 550px; margin: 0 auto; }
        .card { background: white; border-radius: 32px; padding: 20px 18px; margin-bottom: 18px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); }
        .header { text-align: center; margin-bottom: 8px; }
        .header h1 { font-size: 1.7rem; font-weight: 600; background: linear-gradient(135deg, #0f172a, #2563eb); background-clip: text; -webkit-background-clip: text; color: transparent; }
        .version-badge { background: #2563eb20; color: #1e40af; padding: 4px 12px; border-radius: 40px; font-size: 0.7rem; display: inline-block; margin-top: 6px; }
        .badge { background: #e2e8f0; padding: 6px 12px; border-radius: 40px; font-size: 0.7rem; font-weight: 500; display: inline-block; margin-top: 6px; }
        .uptime-display { background: #dbeafe; padding: 10px; border-radius: 20px; text-align: center; margin-bottom: 12px; font-size: 0.85rem; }
        .level-gauge-container { text-align: center; margin: 10px 0 8px; }
        .gauge-title { font-size: 0.8rem; color: #475569; margin-bottom: 8px; font-weight: 500; }
        .vertical-gauge { width: 180px; height: 200px; margin: 0 auto; background: #e2e8f0; border-radius: 30px; position: relative; overflow: hidden; box-shadow: inset 0 0 0 3px white, 0 4px 12px rgba(0,0,0,0.1); }
        .water-fill-vertical { background: linear-gradient(180deg, #3b82f6, #1e40af); position: absolute; bottom: 0; left: 0; right: 0; transition: height 0.5s ease; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 1.2rem; }
        .level-text-large { font-size: 2rem; font-weight: 800; margin-top: 12px; color: #1e293b; }
        .stats-row { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin: 16px 0; }
        .stat-block { background: #f8fafc; border-radius: 24px; padding: 12px; text-align: center; }
        .stat-value { font-size: 1.8rem; font-weight: 700; }
        .stat-label { font-size: 0.7rem; text-transform: uppercase; color: #475569; }
        .info-row { display: flex; justify-content: space-between; align-items: center; padding: 8px 0; border-bottom: 1px solid #e2e8f0; }
        .info-row:last-child { border-bottom: none; }
        .info-label { font-size: 0.85rem; color: #475569; }
        .info-value { font-size: 1.1rem; font-weight: 600; color: #0f172a; }
        .mode-row { display: flex; justify-content: space-between; align-items: center; background: #f1f5f9; padding: 12px 16px; border-radius: 60px; margin: 16px 0 12px; }
        .toggle-switch { position: relative; display: inline-block; width: 56px; height: 28px; }
        .toggle-switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #cbd5e1; transition: 0.3s; border-radius: 34px; }
        .slider:before { position: absolute; content: ""; height: 22px; width: 22px; left: 3px; bottom: 3px; background-color: white; transition: 0.3s; border-radius: 50%; }
        input:checked + .slider { background-color: #2563eb; }
        input:checked + .slider:before { transform: translateX(28px); }
        .pump-btn { width: 100%; padding: 18px; border-radius: 60px; border: none; font-weight: 700; font-size: 1.4rem; background: #dc2626; color: white; margin: 12px 0 8px; cursor: pointer; transition: 0.2s; }
        .pump-btn.running { background: #10b981; animation: pulse 1.8s infinite; }
        .pump-btn.cooldown { background: #6b7280; cursor: not-allowed; }
        .pump-btn.inrush { background: #8b5cf6; animation: pulse 1s infinite; }
        .pump-btn.softstart { background: #06b6d4; animation: pulse 1.2s infinite; }
        .pump-btn.manual-mode { background: #f59e0b; }
        .pump-btn.inching { background: #8b5cf6; animation: pulse 0.8s infinite; }
        .pump-btn.lockout { background: #dc2626; opacity: 0.7; cursor: not-allowed; }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(16,185,129,0.7); } 70% { box-shadow: 0 0 0 15px rgba(16,185,129,0); } 100% { box-shadow: 0 0 0 0 rgba(16,185,129,0); } }
        .flex-between { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px; }
        .btn-secondary { background: #e2e8f0; border: none; padding: 10px 16px; border-radius: 40px; font-weight: 500; width: 100%; cursor: pointer; margin-top: 8px; }
        .engmode-link { text-align: center; margin-top: 12px; font-size: 0.7rem; }
        .engmode-link a { color: #94a3b8; text-decoration: none; }
        hr { margin: 14px 0; border: none; border-top: 1px solid #e2e8f0; }
        .protection-badge { display: inline-block; background: #fef3c7; color: #92400e; padding: 4px 8px; border-radius: 20px; font-size: 0.7rem; margin-top: 8px; animation: pulse 1s infinite; }
        .lockout-badge { display: inline-block; background: #fee2e2; color: #991b1b; padding: 4px 8px; border-radius: 20px; font-size: 0.7rem; margin-top: 8px; font-weight: bold; }
        .sensor-chip { background: #e6f7ec; padding: 5px 10px; border-radius: 50px; font-size: 0.7rem; font-weight: 500; display: inline-flex; align-items: center; gap: 6px; }
        .led { width: 10px; height: 10px; border-radius: 10px; display: inline-block; }
        .led-green { background: #22c55e; }
        .led-red { background: #ef4444; }
        .led-yellow { background: #eab308; }
        .display-only-badge { background: #fef3c7; color: #92400e; padding: 4px 8px; border-radius: 20px; font-size: 0.65rem; margin-left: 8px; }
        .manual-timer { font-size: 0.65rem; color: #d97706; margin-top: 5px; }
        .electrical-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin: 12px 0; }
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>💧 AquaPro S31 Controller</h1>
        <div><span class="version-badge">v3.4 PRESSURE LOCKOUT</span></div>
        <div class="badge">Short Press = Manual + Toggle | Long Press = Mode Switch</div>
    </div>
    
    <div class="card">
        <div class="uptime-display">
            ⏱️ System Uptime: <strong id="systemUptime">0s</strong>
        </div>
        
        <div class="flex-between">
            <span>📡 Sensor status</span>
            <span id="sensorBadge" class="sensor-chip"><span class="led led-green"></span> Healthy</span>
        </div>
        <div class="flex-between" style="margin-top: 10px;">
            <span>🔘 Pressure Switch (GPIO4):</span>
            <span id="pressureStatus" class="sensor-chip"><span class="led led-green"></span> OK (HIGH)</span>
        </div>
        <div id="lockoutDisplay" style="background: #fee2e2; color: #991b1b; padding: 8px 12px; border-radius: 20px; font-size: 0.8rem; text-align: center; margin-top: 10px; display: none;">
            🔒 PRESSURE LOCKOUT ACTIVE: <span id="lockoutTimeLeft">0</span>
            <button onclick="resetLockout()" style="margin-left: 10px; background: #991b1b; color: white; border: none; padding: 4px 12px; border-radius: 20px; cursor: pointer;">🔓 Reset</button>
        </div>
        <div class="flex-between" style="margin-top: 5px;">
            <span>🕒 Last reading:</span>
            <span id="lastSeenText" style="font-family: monospace;">--</span>
        </div>
    </div>
    
    <div id="dashboardSection">
        <div class="card">
            <div class="level-gauge-container">
                <div class="gauge-title">💧 Water Tank Level <span class="display-only-badge">DISPLAY ONLY</span></div>
                <div class="vertical-gauge">
                    <div class="water-fill-vertical" id="waterFillVertical" style="height: 0%;">0%</div>
                </div>
                <div class="level-text-large"><span id="levelPercent">0</span>%</div>
            </div>
            <div class="stats-row">
                <div class="stat-block"><div class="stat-value"><span id="sensorVoltage">0.00</span></div><div class="stat-label">🔋 Sensor Voltage</div></div>
                <div class="stat-block"><div class="stat-value"><span id="volumeVal">0</span> L</div><div class="stat-label">💧 Volume</div></div>
            </div>
        </div>
        
        <div class="card">
            <div class="electrical-grid">
                <div class="stat-block">
                    <div class="stat-value"><span id="powerNow">0</span> W</div>
                    <div class="stat-label">⚡ Power</div>
                </div>
                <div class="stat-block">
                    <div class="stat-value"><span id="energyToday">0.0</span> kWh</div>
                    <div class="stat-label">📊 Energy Used</div>
                </div>
                <div class="stat-block">
                    <div class="stat-value"><span id="voltageNow">0</span> V</div>
                    <div class="stat-label">⚡ Voltage</div>
                </div>
                <div class="stat-block">
                    <div class="stat-value"><span id="currentNow">0.00</span> A</div>
                    <div class="stat-label">💨 Current</div>
                </div>
            </div>
            <div class="info-row">
                <span class="info-label">⚡ Power Factor</span>
                <span class="info-value" id="pfValue">0.00</span>
            </div>
            <div id="loadWarning" class="protection-badge" style="display: none;">⚠️ OVERLOAD PROTECTION ACTIVE!</div>
            <div id="inrushWarning" class="protection-badge" style="display: none;">⚡ INRUSH TOLERANCE ACTIVE</div>
            <div id="cooldownWarning" class="protection-badge" style="display: none;">⏱️ Overload Cooldown active</div>
            <div id="dryRunCooldownWarning" class="protection-badge" style="display: none;">💧 Dry Run Cooldown active</div>
            <div id="dryRunWarning" class="protection-badge" style="display: none;">💧 DRY RUN DETECTED!</div>
            <div id="softStartWarning" class="protection-badge" style="display: none;">🌊 SOFT START DELAY ACTIVE</div>
            <div id="inchingWarning" class="protection-badge" style="display: none;">⏱️ INCHING MODE ACTIVE</div>
        </div>
        
        <div class="card">
            <div class="mode-row">
                <span class="mode-text" id="modeText">🤖 AUTO MODE</span>
                <label class="toggle-switch"><input type="checkbox" id="autoModeToggle" onchange="toggleAutoMode()"><span class="slider"></span></label>
                <span class="mode-text">👆 MANUAL</span>
            </div>
            <div id="autoReturnInfo" style="font-size: 0.65rem; color: #059669; text-align: center; margin-top: 5px;"></div>
            <div id="manualTimer" class="manual-timer" style="display: none;"></div>
            <button id="pumpActionBtn" class="pump-btn" onclick="manualPumpToggle()">PUMP OFF</button>
            <div id="inchingTimerDisplay" style="background: #ede9fe; color: #6d28d9; padding: 8px 12px; border-radius: 20px; font-size: 0.8rem; text-align: center; margin-top: 10px; display: none;">
                ⏱️ INCHING: Pump will stop in <span id="inchingTimeLeft">0</span>
            </div>
            <div id="softStartIndicator" style="background: #cffafe; color: #0891b2; padding: 8px 12px; border-radius: 20px; font-size: 0.8rem; text-align: center; margin-top: 10px; display: none;">
                🌊 SOFT START: Pump will start in <span id="softStartTimer">0</span>ms
            </div>
            <div id="controlReason" style="font-size: 0.7rem; text-align: center; margin-top: 5px;"></div>
            <hr>
            <div class="flex-between"><span>📦 Total runtime</span><strong><span id="totalRunMinutes">0</span> min</strong></div>
            <div class="flex-between"><span>🔄 Cycles count</span><strong><span id="cyclesCount">0</span></strong></div>
            <div class="flex-between"><span>⚠️ Overload events</span><strong><span id="overloadCount">0</span></strong></div>
            <div class="flex-between"><span>💧 Dry run events</span><strong><span id="dryrunCount">0</span></strong></div>
            <div class="flex-between"><span>🔘 Button presses</span><strong><span id="buttonPressCount">0</span></strong></div>
            <div class="flex-between"><span>🌊 Soft start delays</span><strong><span id="softStartCount">0</span></strong></div>
        </div>
        
        <div class="card">
            <button class="btn-secondary" onclick="triggerSensorRead()">📡 Request sensor reading now</button>
            <button class="btn-secondary" style="background:#fee2e2; color:#b91c1c;" onclick="confirmReboot()">🔄 Reboot device</button>
        </div>
    </div>
    
    <div class="engmode-link"><a href="/engmode">🔧 Engineering Mode (advanced settings)</a></div>
</div>
<script>
    let manualTimerInterval = null;
    
    function formatTime(seconds) {
        if(seconds < 60) return seconds + "s";
        let mins = Math.floor(seconds / 60);
        let secs = seconds % 60;
        if(secs > 0) return mins + "m " + secs + "s";
        return mins + "m";
    }
    
    function formatUptime(seconds) {
        let days = Math.floor(seconds / 86400);
        let hours = Math.floor((seconds % 86400) / 3600);
        let minutes = Math.floor((seconds % 3600) / 60);
        let secs = seconds % 60;
        
        if(days > 0) return days + "d " + hours + "h " + minutes + "m";
        if(hours > 0) return hours + "h " + minutes + "m " + secs + "s";
        if(minutes > 0) return minutes + "m " + secs + "s";
        return secs + "s";
    }
    
    async function fetchJSON(url) { try { const r = await fetch(url); return await r.json(); } catch(e) { return null; } }
    
    async function toggleAutoMode() {
        const isAuto = document.getElementById('autoModeToggle').checked;
        await fetch(`/mode?mode=${isAuto ? 'auto' : 'manual'}`);
        refreshDashboard();
    }
    
    async function manualPumpToggle() { 
        const d = await fetchJSON('/data');
        if(d && d.pressureLockoutActive) {
            alert("🔒 Pressure lockout active! Please wait or reset lockout.");
            return;
        }
        if(d && d.inCooldown) {
            alert("⏱️ Pump is in overload cooldown! Please wait.");
            return;
        }
        if(d && d.dryRunCooldown) {
            alert("💧 Pump is in dry run cooldown! Please wait.");
            return;
        }
        if(d && d.softStartActive) {
            alert("🌊 Pump is in soft start delay! Please wait.");
            return;
        }
        await fetch('/toggle'); 
        refreshDashboard(); 
    }
    
    async function resetLockout() {
        await fetch('/reset_lockout');
        refreshDashboard();
    }
    
    async function triggerSensorRead() { 
        const btn = event.target; 
        const orig = btn.innerText; 
        btn.innerText = "📡 Sending..."; 
        await fetch('/espnow/request'); 
        btn.innerText = "✅ Sent!"; 
        setTimeout(() => btn.innerText = orig, 1500); 
        refreshDashboard(); 
    }
    
    function confirmReboot() { if(confirm("Reboot pump controller?")) { fetch('/reboot'); alert("Rebooting..."); setTimeout(() => location.reload(), 3000); } }
    
    function getBatteryStatus(voltage) {
        if(voltage >= 3.8) return "🔋 Full";
        if(voltage >= 3.5) return "🔋 Good";
        if(voltage >= 3.2) return "🪫 Low";
        return "⚠️ Critical";
    }
    
    function formatLockoutTime(seconds) {
        if(seconds >= 86400) return Math.floor(seconds/86400) + " days";
        if(seconds >= 3600) return Math.floor(seconds/3600) + "h " + Math.floor((seconds%3600)/60) + "m";
        if(seconds >= 60) return Math.floor(seconds/60) + " minutes";
        return seconds + " seconds";
    }
    
    function updateManualTimer(secondsLeft, autoReturnEnabled) {
        const timerDiv = document.getElementById('manualTimer');
        const infoDiv = document.getElementById('autoReturnInfo');
        
        if(autoReturnEnabled) {
            infoDiv.innerHTML = '🔄 Auto-return to AUTO mode ENABLED (10 minutes)';
            infoDiv.style.color = '#059669';
        } else {
            infoDiv.innerHTML = '⏸️ Auto-return to AUTO mode DISABLED';
            infoDiv.style.color = '#6b7280';
        }
        
        if(secondsLeft > 0 && autoReturnEnabled) {
            timerDiv.style.display = 'block';
            let mins = Math.floor(secondsLeft / 60);
            let secs = secondsLeft % 60;
            timerDiv.innerHTML = `⏱️ Returns to AUTO mode in ${mins}:${secs.toString().padStart(2,'0')}`;
        } else {
            timerDiv.style.display = 'none';
            if(manualTimerInterval) clearInterval(manualTimerInterval);
        }
    }
    
    async function refreshDashboard() {
        const d = await fetchJSON('/data');
        if(!d) return;
        
        // Update uptime
        if(d.systemUptime !== undefined) {
            document.getElementById('systemUptime').innerText = formatUptime(d.systemUptime);
        }
        
        const level = Math.min(100, Math.max(0, d.waterLevel || 0));
        document.getElementById('levelPercent').innerText = Math.floor(level);
        document.getElementById('waterFillVertical').style.height = level + '%';
        document.getElementById('waterFillVertical').innerText = Math.floor(level) + '%';
        
        const voltage = (d.batteryVoltage || 0);
        document.getElementById('sensorVoltage').innerHTML = voltage.toFixed(2) + ' V <span style="font-size:0.7rem;">' + getBatteryStatus(voltage) + '</span>';
        document.getElementById('volumeVal').innerText = (d.volume || 0).toFixed(0);
        
        // Update electrical values
        document.getElementById('powerNow').innerText = (d.power || 0).toFixed(1);
        document.getElementById('energyToday').innerText = (d.energy || 0).toFixed(1);
        document.getElementById('voltageNow').innerText = (d.voltage || 0).toFixed(1);
        document.getElementById('currentNow').innerText = (d.current || 0).toFixed(2);
        document.getElementById('pfValue').innerHTML = (d.powerFactor || 0).toFixed(3);
        
        document.getElementById('overloadCount').innerText = d.overloadCount || 0;
        document.getElementById('dryrunCount').innerText = d.dryrunCount || 0;
        document.getElementById('softStartCount').innerText = d.softStartCount || 0;
        document.getElementById('buttonPressCount').innerText = d.buttonPressCount || 0;
        
        const modeText = document.getElementById('modeText');
        const modeToggle = document.getElementById('autoModeToggle');
        if(d.autoMode) {
            modeText.innerHTML = '🤖 AUTO MODE (Pressure Control)';
            modeText.style.color = '#059669';
            modeToggle.checked = true;
            updateManualTimer(0, d.autoReturnEnabled);
        } else {
            modeText.innerHTML = '👆 MANUAL MODE (Button Control)';
            modeText.style.color = '#d97706';
            modeToggle.checked = false;
            if(d.manualTimeLeft > 0 && d.autoReturnEnabled) updateManualTimer(d.manualTimeLeft, d.autoReturnEnabled);
            else updateManualTimer(0, d.autoReturnEnabled);
        }
        
        const pressureElem = document.getElementById('pressureStatus');
        if(d.pressureLockoutActive) {
            pressureElem.innerHTML = '<span class="led led-red"></span> 🔒 LOCKOUT ACTIVE - Pump disabled';
            pressureElem.style.background = '#fee2e2';
        } else if(d.pressureLow) {
            pressureElem.innerHTML = '<span class="led led-red"></span> ⚠️ LOW PRESSURE - Pump ON';
            pressureElem.style.background = '#fee2e2';
        } else {
            pressureElem.innerHTML = '<span class="led led-green"></span> ✅ Pressure OK - Pump OFF';
            pressureElem.style.background = '#e6f7ec';
        }
        
        // Lockout display
        const lockoutDiv = document.getElementById('lockoutDisplay');
        if(d.pressureLockoutActive && d.pressureLockoutRemaining > 0) {
            lockoutDiv.style.display = 'block';
            document.getElementById('lockoutTimeLeft').innerText = formatLockoutTime(d.pressureLockoutRemaining);
        } else {
            lockoutDiv.style.display = 'none';
        }
        
        const inchingDisplay = document.getElementById('inchingTimerDisplay');
        if(d.inchingActive && d.inchingRemaining > 0) {
            inchingDisplay.style.display = 'block';
            let remaining = d.inchingRemaining;
            if(remaining < 60) document.getElementById('inchingTimeLeft').innerText = remaining + " seconds";
            else {
                let mins = Math.floor(remaining / 60);
                let secs = remaining % 60;
                if(secs > 0) document.getElementById('inchingTimeLeft').innerText = mins + "m " + secs + "s";
                else document.getElementById('inchingTimeLeft').innerText = mins + " minutes";
            }
        } else {
            inchingDisplay.style.display = 'none';
        }
        
        const softStartIndicator = document.getElementById('softStartIndicator');
        if(d.softStartActive) {
            softStartIndicator.style.display = 'block';
            document.getElementById('softStartTimer').innerText = d.softStartRemaining;
        } else {
            softStartIndicator.style.display = 'none';
        }
        
        const btn = document.getElementById('pumpActionBtn');
        if(!d.autoMode) btn.classList.add('manual-mode');
        else btn.classList.remove('manual-mode');
        
        if(d.pressureLockoutActive) {
            btn.innerText = "🔒 PRESSURE LOCKOUT ACTIVE";
            btn.classList.add('lockout');
            btn.classList.remove('running', 'cooldown', 'softstart', 'inching', 'inrush');
            btn.disabled = true;
        } else if(d.inCooldown) {
            btn.innerText = "⏱️ OVERLOAD COOLDOWN - " + d.cooldownRemaining + "s";
            btn.classList.add('cooldown');
            btn.classList.remove('running', 'inrush', 'softstart', 'inching', 'lockout');
            btn.disabled = true;
        } else if(d.dryRunCooldown) {
            btn.innerText = "💧 DRY RUN COOLDOWN - " + d.dryrunCooldownRemaining + "s";
            btn.classList.add('cooldown');
            btn.classList.remove('running', 'inrush', 'softstart', 'inching', 'lockout');
            btn.disabled = true;
        } else if(d.softStartActive) {
            btn.innerText = "🌊 SOFT START - " + (d.softStartRemaining/1000).toFixed(1) + "s";
            btn.classList.add('softstart');
            btn.classList.remove('running', 'cooldown', 'inrush', 'inching', 'lockout');
            btn.disabled = true;
        } else if(d.inchingActive) {
            let remaining = d.inchingRemaining;
            if(remaining < 60) btn.innerText = "⏱️ INCHING - " + remaining + "s";
            else {
                let mins = Math.floor(remaining / 60);
                let secs = remaining % 60;
                if(secs > 0) btn.innerText = "⏱️ INCHING - " + mins + "m " + secs + "s";
                else btn.innerText = "⏱️ INCHING - " + mins + "m";
            }
            btn.classList.add('inching');
            btn.classList.remove('running', 'cooldown', 'softstart', 'inrush', 'lockout');
            btn.disabled = false;
        } else if(d.inInrushPeriod) {
            btn.innerText = "⚡ INRUSH - " + (d.inrushRemaining/1000).toFixed(0) + "s";
            btn.classList.add('inrush');
            btn.classList.remove('running', 'cooldown', 'softstart', 'inching', 'lockout');
            btn.disabled = false;
        } else if(d.pumpState) { 
            btn.innerText = "💧 PUMP RUNNING"; 
            btn.classList.add('running');
            btn.classList.remove('inrush', 'cooldown', 'softstart', 'inching', 'lockout');
            btn.disabled = false;
        } else { 
            btn.innerText = "⏹️ PUMP OFF"; 
            btn.classList.remove('running', 'inrush', 'cooldown', 'softstart', 'inching', 'lockout');
            btn.disabled = false;
        }
        
        let reason = d.pumpReason || "";
        if(d.pressureLockoutActive) reason = "🔒 Pressure lockout - Pump disabled for " + formatLockoutTime(d.pressureLockoutRemaining);
        document.getElementById('controlReason').innerHTML = reason;
        
        document.getElementById('loadWarning').style.display = d.overloadProtectionActive ? 'inline-block' : 'none';
        document.getElementById('inrushWarning').style.display = d.inInrushPeriod ? 'inline-block' : 'none';
        document.getElementById('cooldownWarning').style.display = d.inCooldown ? 'inline-block' : 'none';
        document.getElementById('dryRunCooldownWarning').style.display = d.dryRunCooldown ? 'inline-block' : 'none';
        document.getElementById('dryRunWarning').style.display = d.dryRunActive ? 'inline-block' : 'none';
        document.getElementById('softStartWarning').style.display = d.softStartActive ? 'inline-block' : 'none';
        document.getElementById('inchingWarning').style.display = d.inchingActive ? 'inline-block' : 'none';
        
        const sensorSpan = document.getElementById('sensorBadge');
        if(d.sensorHealthy) sensorSpan.innerHTML = '<span class="led led-green"></span> ✅ Healthy';
        else if(d.sensorWarning) sensorSpan.innerHTML = '<span class="led led-yellow"></span> ⚠️ Weak signal';
        else sensorSpan.innerHTML = '<span class="led led-red"></span> ❌ No sensor data';
        document.getElementById('lastSeenText').innerText = d.sensorLastSeen || 'never';
        
        const stats = await fetchJSON('/stats');
        if(stats) { 
            document.getElementById('totalRunMinutes').innerText = stats.total_runtime || 0; 
            document.getElementById('cyclesCount').innerText = stats.pump_cycles || 0; 
        }
    }
    
    setInterval(refreshDashboard, 1000);
    window.onload = () => { refreshDashboard(); };
</script>
</body>
</html>
)rawliteral";

// Engineering mode HTML (v3.4 with lockout setting)
const char engineering_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Pump - Engineering v3.4</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; border-radius: 20px; padding: 25px; margin-bottom: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
        h1 { text-align: center; color: #333; margin-bottom: 5px; font-size: 24px; }
        .version { text-align: center; color: #10B981; font-size: 12px; margin-bottom: 5px; font-weight: bold; }
        .nav-buttons { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }
        .nav-btn { flex: 1; background: #e5e7eb; color: #333; padding: 10px; border: none; border-radius: 10px; cursor: pointer; font-weight: bold; font-size: 14px; }
        .nav-btn.active { background: #667eea; color: white; }
        .config-group { margin-bottom: 20px; }
        label { display: block; font-weight: bold; margin-bottom: 8px; color: #333; font-size: 14px; }
        input, select { width: 100%; padding: 12px; border: 1px solid #ddd; border-radius: 8px; font-size: 14px; }
        button { background: #667eea; color: white; border: none; padding: 12px 24px; border-radius: 10px; cursor: pointer; font-weight: bold; margin-top: 10px; margin-right: 10px; font-size: 14px; }
        button.danger { background: #EF4444; }
        .info-text { font-size: 12px; color: #666; margin-top: 5px; }
        .simple-link { text-align: center; margin-top: 20px; padding: 10px; background: #e0e7ff; border-radius: 10px; }
        .simple-link a { color: #4338ca; text-decoration: none; font-weight: bold; }
        .save-status { position: fixed; bottom: 20px; right: 20px; background: #10b981; color: white; padding: 10px 20px; border-radius: 10px; display: none; font-weight: bold; }
        
        .checkbox-large {
            display: flex;
            align-items: center;
            padding: 12px;
            background: #f0fdf4;
            border-radius: 12px;
            border: 2px solid #22c55e;
            cursor: pointer;
            transition: all 0.2s;
        }
        .checkbox-large:hover {
            background: #dcfce7;
            transform: scale(1.01);
        }
        .checkbox-large input {
            width: 24px;
            height: 24px;
            margin-right: 15px;
            cursor: pointer;
            accent-color: #22c55e;
        }
        .checkbox-large label {
            flex: 1;
            margin-bottom: 0;
            font-size: 16px;
            cursor: pointer;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .checkbox-large .icon {
            font-size: 20px;
        }
        
        h3 {
            margin: 20px 0 15px 0;
            padding-bottom: 8px;
            border-bottom: 2px solid #667eea;
            color: #333;
        }
        
        .lockout-preset {
            display: inline-block;
            background: #e0e7ff;
            padding: 5px 10px;
            border-radius: 20px;
            margin: 5px;
            cursor: pointer;
            font-size: 12px;
        }
        .lockout-preset:hover {
            background: #c7d2fe;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>🔧 Engineering Mode v3.4</h1>
            <div class="version">PRESSURE LOCKOUT EDITION | No Pump Cycling | Stable Saves</div>
            
            <div class="nav-buttons">
                <button class="nav-btn active" onclick="showSection('lockout')">🔒 Lockout</button>
                <button class="nav-btn" onclick="showSection('protection')">🛡️ Protection</button>
                <button class="nav-btn" onclick="showSection('inching')">⏱️ Inching</button>
                <button class="nav-btn" onclick="showSection('inrush')">⚡ Inrush</button>
                <button class="nav-btn" onclick="showSection('softstart')">🌊 Soft Start</button>
                <button class="nav-btn" onclick="showSection('pressure')">🔘 Pressure</button>
                <button class="nav-btn" onclick="showSection('system')">⚙️ System</button>
            </div>
            
            <div id="lockoutSection">
                <h3>🔒 Pressure Lockout Timer</h3>
                <div class="info-text" style="margin-bottom: 15px; padding: 10px; background: #fee2e2; border-radius: 10px;">
                    ⚡ CRITICAL: Prevents pump from cycling on/off rapidly!<br>
                    When pressure becomes HIGH (pump off), system waits this long before checking again.<br>
                    <strong>Set to 0 to disable lockout (not recommended).</strong>
                </div>
                
                <div class="config-group">
                    <label>⏱️ Lockout Duration</label>
                    <input type="number" id="lockoutHours" min="0" max="72" step="0.5" value="2">
                    <div class="info-text">Hours (0 = disabled, 0.083 = 5 minutes, 1-72 hours)</div>
                </div>
                
                <div class="info-text" style="margin-top: 10px;">
                    <strong>Quick Presets:</strong><br>
                    <span class="lockout-preset" onclick="setLockout(0.083)">5 minutes</span>
                    <span class="lockout-preset" onclick="setLockout(0.5)">30 minutes</span>
                    <span class="lockout-preset" onclick="setLockout(1)">1 hour</span>
                    <span class="lockout-preset" onclick="setLockout(2)">2 hours</span>
                    <span class="lockout-preset" onclick="setLockout(4)">4 hours</span>
                    <span class="lockout-preset" onclick="setLockout(8)">8 hours</span>
                    <span class="lockout-preset" onclick="setLockout(12)">12 hours</span>
                    <span class="lockout-preset" onclick="setLockout(24)">1 day</span>
                    <span class="lockout-preset" onclick="setLockout(48)">2 days</span>
                    <span class="lockout-preset" onclick="setLockout(72)">3 days</span>
                </div>
                
                <button onclick="saveLockout()">💾 Save Lockout Settings</button>
            </div>
            
            <div id="protectionSection" style="display:none">
                <h3>💧 Dry Run Protection</h3>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('dryRunEnabled')">
                    <input type="checkbox" id="dryRunEnabled" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable Dry Run Protection</label>
                </div>
                <div class="config-group"><label>⏱️ Dry Run Time (seconds)</label><input type="number" id="dryRunProtection" min="5" max="300"></div>
                <div class="config-group"><label>📉 Min Power (Watts)</label><input type="number" id="minPower" min="0" max="3500" step="any"></div>
                <div class="config-group"><label>🔄 Dry Run Cooldown (seconds)</label><input type="number" id="dryrunCooldown" min="0" max="300"></div>
                
                <h3>⚠️ Overload Protection</h3>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('loadProtectionToggle')">
                    <input type="checkbox" id="loadProtectionToggle" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable Overload Protection</label>
                </div>
                <div class="config-group"><label>📈 Max Power (Watts)</label><input type="number" id="maxPower" min="10" max="3500"></div>
                <div class="config-group"><label>🔄 Overload Cooldown (seconds)</label><input type="number" id="overloadCooldown" min="0" max="300"></div>
                <button onclick="saveProtection()">💾 Save All Protection Settings</button>
            </div>
            
            <div id="inchingSection" style="display:none">
                <h3>⏱️ Inching Mode</h3>
                <div class="info-text" style="margin-bottom: 15px; padding: 10px; background: #ede9fe; border-radius: 10px;">
                    💡 When enabled, the pump will automatically turn OFF after the set duration.<br>
                    <strong>✓ Works in both AUTO and MANUAL modes!</strong>
                </div>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('inchingToggle')">
                    <input type="checkbox" id="inchingToggle" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable Inching Mode</label>
                </div>
                <div class="config-group"><label>⏱️ Duration (minutes) - 1 to 60 minutes</label>
                    <input type="number" id="inchingDuration" min="1" max="60" value="1" step="1">
                </div>
                <button onclick="saveInching()">💾 Save Inching Settings</button>
            </div>
            
            <div id="inrushSection" style="display:none">
                <h3>⚡ Inrush Current Protection</h3>
                <div class="info-text" style="margin-bottom: 15px; padding: 10px; background: #d1fae5; border-radius: 10px;">
                    ✅ Settings save safely with mutex protection.
                </div>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('inrushEnabled')">
                    <input type="checkbox" id="inrushEnabled" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable Inrush Protection</label>
                </div>
                <div class="config-group">
                    <label>⚡ Tolerance (ms) - 500 to 5000 ms</label>
                    <input type="range" id="inrushToleranceSlider" min="500" max="5000" step="100" value="2000" oninput="updateInrushValue(this.value)">
                    <input type="number" id="inrushTolerance" min="500" max="5000" step="100" value="2000" oninput="updateInrushSlider(this.value)">
                    <div class="info-text">Setting: <span id="inrushValueDisplay">2000</span> ms (<span id="inrushSecondsDisplay">2.0</span> seconds)</div>
                </div>
                <button onclick="saveInrush()">💾 Save Inrush Settings</button>
            </div>
            
            <div id="softstartSection" style="display:none">
                <h3>🌊 Soft Start Settings</h3>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('softStartToggle')">
                    <input type="checkbox" id="softStartToggle" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable Soft Start</label>
                </div>
                <div class="config-group"><label>⏱️ Delay (ms)</label><input type="number" id="softStartDelay" min="0" max="10000" step="100"></div>
                <button onclick="saveSoftStart()">💾 Save Soft Start Settings</button>
            </div>
            
            <div id="pressureSection" style="display:none">
                <h3>🔘 Pressure Switch Settings</h3>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('pressureInverted')">
                    <input type="checkbox" id="pressureInverted" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Invert Logic</label>
                </div>
                <button onclick="savePressure()">💾 Save Pressure Settings</button>
            </div>
            
            <div id="systemSection" style="display:none">
                <h3>⚙️ System Settings</h3>
                <div class="config-group"><label>🏷️ Device Name</label><input type="text" id="hostname" placeholder="s31-pump"></div>
                <div class="config-group"><label>📡 ESP-NOW Peer MAC</label><input type="text" id="peerMac" placeholder="FF:FF:FF:FF:FF:FF"></div>
                <div class="config-group"><label>📻 ESP-NOW Channel</label><input type="number" id="espnowChannel" min="1" max="13"></div>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('useEspnow')">
                    <input type="checkbox" id="useEspnow" onclick="event.stopPropagation()">
                    <label><span class="icon">🔧</span> Enable ESP-NOW</label>
                </div>
                <div class="config-group checkbox-large" onclick="toggleCheckbox('autoReturnToggle')">
                    <input type="checkbox" id="autoReturnToggle" onclick="event.stopPropagation()">
                    <label><span class="icon">🔄</span> Enable Auto-Return to AUTO Mode (10 minutes)</label>
                </div>
                <button onclick="saveSystem()">💾 Save &amp; Reboot</button>
                <button class="danger" onclick="factoryReset()">⚠️ Factory Reset</button>
                <button class="danger" onclick="reboot()">🔄 Reboot</button>
            </div>
            
            <div class="simple-link"><a href="/">← Back to Simple Mode</a></div>
        </div>
    </div>
    <div id="saveStatus" class="save-status">✓ Settings Saved!</div>
    
    <script>
        function toggleCheckbox(id) {
            const cb = document.getElementById(id);
            if(cb) cb.checked = !cb.checked;
        }
        
        function setLockout(hours) {
            document.getElementById('lockoutHours').value = hours;
        }
        
        function updateInrushValue(val) {
            document.getElementById('inrushTolerance').value = val;
            document.getElementById('inrushToleranceSlider').value = val;
            document.getElementById('inrushValueDisplay').innerText = val;
            document.getElementById('inrushSecondsDisplay').innerText = (val / 1000).toFixed(1);
        }
        
        function updateInrushSlider(val) {
            document.getElementById('inrushToleranceSlider').value = val;
            document.getElementById('inrushValueDisplay').innerText = val;
            document.getElementById('inrushSecondsDisplay').innerText = (val / 1000).toFixed(1);
        }
        
        async function fetchConfig(){const r=await fetch('/config');return await r.json();}
        function showSaveStatus() {
            const status = document.getElementById('saveStatus');
            status.style.display = 'block';
            setTimeout(() => { status.style.display = 'none'; }, 2000);
        }
        
        async function saveLockout() {
            const hours = parseFloat(document.getElementById('lockoutHours').value);
            const response = await fetch('/config/lockout', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ pressure_lockout_hours: hours })
            });
            if(response.ok) { showSaveStatus(); setTimeout(() => location.reload(), 1000); }
            else alert("Failed to save lockout settings!");
        }
        
        async function saveProtection(){
            const s={
                dry_run_enabled:document.getElementById('dryRunEnabled').checked,
                dry_run_protection:parseInt(document.getElementById('dryRunProtection').value),
                min_power:parseFloat(document.getElementById('minPower').value),
                dryrun_cooldown_seconds:parseInt(document.getElementById('dryrunCooldown').value),
                pump_load_protection_enabled:document.getElementById('loadProtectionToggle').checked,
                max_power_threshold:parseFloat(document.getElementById('maxPower').value),
                overload_cooldown_seconds:parseInt(document.getElementById('overloadCooldown').value)
            };
            const response = await fetch('/config/protection',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});
            if(response.ok) { showSaveStatus(); setTimeout(() => location.reload(), 1000); }
            else alert("Failed to save!");
        }
        
        async function saveInching(){
            const duration = parseInt(document.getElementById('inchingDuration').value);
            const s={ inching_enabled:document.getElementById('inchingToggle').checked, inching_duration_minutes:duration };
            const response = await fetch('/config/inching',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});
            if(response.ok) { showSaveStatus(); setTimeout(() => location.reload(), 1000); }
            else alert("Failed to save inching settings!");
        }
        
        async function saveInrush(){
            const tolerance = parseInt(document.getElementById('inrushTolerance').value);
            const enabled = document.getElementById('inrushEnabled').checked;
            const response = await fetch('/config/inrush', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ inrush_enabled: enabled, inrush_tolerance_ms: tolerance })
            });
            if(response.ok) { showSaveStatus(); setTimeout(() => location.reload(), 1500); }
            else alert("Failed to save inrush settings!");
        }
        
        async function saveSoftStart(){
            const s={ soft_start_enabled:document.getElementById('softStartToggle').checked, soft_start_delay_ms:parseInt(document.getElementById('softStartDelay').value) };
            await fetch('/config/softstart',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});
            showSaveStatus(); setTimeout(() => location.reload(), 1000);
        }
        
        async function savePressure(){
            const s={ pressure_switch_inverted:document.getElementById('pressureInverted').checked };
            await fetch('/config/pressure',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});
            showSaveStatus(); setTimeout(() => location.reload(), 1000);
        }
        
        async function saveSystem(){
            const s={ hostname:document.getElementById('hostname').value, peer_mac:document.getElementById('peerMac').value, espnow_channel:parseInt(document.getElementById('espnowChannel').value), use_espnow:document.getElementById('useEspnow').checked, auto_return_enabled:document.getElementById('autoReturnToggle').checked };
            await fetch('/config/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});
            showSaveStatus(); alert('Rebooting...'); setTimeout(()=>location.reload(),3000);
        }
        
        async function factoryReset(){ if(confirm('FACTORY RESET? All settings will be lost!')){ await fetch('/factoryreset'); } }
        async function reboot(){ if(confirm('Reboot device?')){ await fetch('/reboot'); } }
        
        async function loadSettings(){
            const cfg=await fetchConfig();
            document.getElementById('lockoutHours').value = cfg.pressure_lockout_hours || 2;
            document.getElementById('dryRunEnabled').checked=cfg.dry_run_enabled;
            document.getElementById('dryRunProtection').value=cfg.dry_run_protection;
            document.getElementById('minPower').value=cfg.min_power;
            document.getElementById('dryrunCooldown').value=cfg.dryrun_cooldown_seconds;
            document.getElementById('loadProtectionToggle').checked=cfg.pump_load_protection_enabled;
            document.getElementById('maxPower').value=cfg.max_power_threshold;
            document.getElementById('overloadCooldown').value=cfg.overload_cooldown_seconds;
            document.getElementById('inrushEnabled').checked=cfg.inrush_enabled;
            document.getElementById('inrushTolerance').value=cfg.inrush_tolerance_ms;
            document.getElementById('inrushToleranceSlider').value=cfg.inrush_tolerance_ms;
            document.getElementById('inrushValueDisplay').innerText=cfg.inrush_tolerance_ms;
            document.getElementById('inrushSecondsDisplay').innerText=(cfg.inrush_tolerance_ms/1000).toFixed(1);
            document.getElementById('inchingToggle').checked=cfg.inching_enabled;
            document.getElementById('inchingDuration').value=cfg.inching_duration_minutes;
            document.getElementById('softStartToggle').checked=cfg.soft_start_enabled;
            document.getElementById('softStartDelay').value=cfg.soft_start_delay_ms;
            document.getElementById('pressureInverted').checked=cfg.pressure_switch_inverted;
            document.getElementById('autoReturnToggle').checked=cfg.auto_return_enabled;
            document.getElementById('hostname').value=cfg.hostname || 's31-pump';
            document.getElementById('peerMac').value=cfg.peer_mac || 'FF:FF:FF:FF:FF:FF';
            document.getElementById('espnowChannel').value=cfg.espnow_channel;
            document.getElementById('useEspnow').checked=cfg.use_espnow;
        }
        
        function showSection(s){
            document.getElementById('lockoutSection').style.display = s === 'lockout' ? 'block' : 'none';
            document.getElementById('protectionSection').style.display = s === 'protection' ? 'block' : 'none';
            document.getElementById('inchingSection').style.display = s === 'inching' ? 'block' : 'none';
            document.getElementById('inrushSection').style.display = s === 'inrush' ? 'block' : 'none';
            document.getElementById('softstartSection').style.display = s === 'softstart' ? 'block' : 'none';
            document.getElementById('pressureSection').style.display = s === 'pressure' ? 'block' : 'none';
            document.getElementById('systemSection').style.display = s === 'system' ? 'block' : 'none';
            document.querySelectorAll('.nav-btn').forEach(btn=>btn.classList.remove('active'));
            event.target.classList.add('active');
        }
        
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
      return;
    }
    server.send(404, "text/plain", "Not found");
  });
  
  server.on("/data", HTTP_GET, []() {
    updatePowerQuality();
    readPressureSwitch();
    
    StaticJsonDocument<1024> doc;
    doc["waterLevel"] = currentWaterLevel;
    doc["distance"] = currentDistance;
    doc["volume"] = currentVolume;
    doc["batteryVoltage"] = batteryVoltage;
    doc["power"] = s31.getPower();
    doc["voltage"] = s31.getVoltage();
    doc["current"] = s31.getCurrent();
    doc["energy"] = s31.getEnergy();
    doc["powerFactor"] = powerFactor;
    doc["pumpState"] = s31.getRelayState();
    doc["autoMode"] = auto_mode;
    doc["autoReturnEnabled"] = auto_return_enabled;
    doc["pressureLow"] = pressureSwitch.pressureLow;
    doc["pressureLockoutActive"] = pressureLockout.active;
    doc["pressureLockoutRemaining"] = getLockoutRemainingSeconds();
    doc["softStartEnabled"] = soft_start_enabled;
    doc["softStartDelayMs"] = soft_start_delay_ms;
    doc["softStartActive"] = softStart.delayActive;
    if (softStart.delayActive) {
      uint32_t elapsed = millis() - softStart.delayStartTime;
      doc["softStartRemaining"] = (soft_start_delay_ms > elapsed) ? (soft_start_delay_ms - elapsed) : 0;
    }
    doc["softStartCount"] = softStart.delayCount;
    doc["overloadProtectionActive"] = overload.active;
    doc["inInrushPeriod"] = (inrush_enabled && overload.inrushActive);
    if (overload.inrushActive && inrush_enabled) {
      uint32_t elapsed = millis() - overload.pumpStartTime;
      uint32_t remaining = (inrush_tolerance_ms > elapsed) ? (inrush_tolerance_ms - elapsed) : 0;
      doc["inrushRemaining"] = remaining;
    }
    doc["inCooldown"] = (millis() < overload.cooldownUntil);
    doc["dryRunCooldown"] = (millis() < dryRun.cooldownUntil);
    doc["inchingActive"] = inching.active;
    if (inching.active) {
      uint32_t elapsed = millis() - inching.startTime;
      uint32_t remainingMs = (inching.duration > elapsed) ? (inching.duration - elapsed) : 0;
      doc["inchingRemaining"] = remainingMs / 1000;
    }
    if (millis() < overload.cooldownUntil) doc["cooldownRemaining"] = (overload.cooldownUntil - millis()) / 1000;
    if (millis() < dryRun.cooldownUntil) doc["dryrunCooldownRemaining"] = (dryRun.cooldownUntil - millis()) / 1000;
    doc["overloadCount"] = overload.overloadCount;
    doc["dryrunCount"] = dryRun.events;
    doc["dryRunActive"] = (dryRun.lowPowerStartTime != 0);
    doc["espnowActive"] = espnow_initialized;
    doc["espnowDataValid"] = espnowDataValid;
    doc["sensorHealthy"] = !sensorIsDead && (millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
    doc["sensorWarning"] = !sensorIsDead && (millis() - lastEspNowData >= ESP_NOW_DATA_TIMEOUT);
    if (lastEspNowData > 0) {
      char timeBuf[32];
      snprintf(timeBuf, sizeof(timeBuf), "%lus ago", (millis() - lastEspNowData)/1000);
      doc["sensorLastSeen"] = String(timeBuf);
    }
    doc["buttonPressCount"] = button_press_count;
    doc["systemUptime"] = (millis() - systemStartTime) / 1000;
    
    if (!auto_mode && button.lastManualModeTime > 0) {
      uint32_t elapsed = millis() - button.lastManualModeTime;
      if (elapsed < AUTO_RETURN_TIMEOUT_MS) doc["manualTimeLeft"] = (AUTO_RETURN_TIMEOUT_MS - elapsed) / 1000;
      else doc["manualTimeLeft"] = 0;
    } else doc["manualTimeLeft"] = 0;
    
    String reason = "";
    if (pressureLockout.active) {
      uint32_t remaining = getLockoutRemainingSeconds();
      if (remaining >= 3600) {
        char reasonBuf[64];
        snprintf(reasonBuf, sizeof(reasonBuf), "🔒 Lockout: %.1f hours remaining", remaining / 3600.0);
        reason = String(reasonBuf);
      } else if (remaining >= 60) {
        char reasonBuf[64];
        snprintf(reasonBuf, sizeof(reasonBuf), "🔒 Lockout: %lu minutes remaining", remaining / 60);
        reason = String(reasonBuf);
      } else {
        char reasonBuf[64];
        snprintf(reasonBuf, sizeof(reasonBuf), "🔒 Lockout: %lu seconds remaining", remaining);
        reason = String(reasonBuf);
      }
    } else if (softStart.delayActive) reason = "🌊 SOFT START - Delaying";
    else if (inrush_enabled && overload.inrushActive) reason = "⚡ Inrush active";
    else if (millis() < overload.cooldownUntil) reason = "⏱️ Overload Cooldown";
    else if (millis() < dryRun.cooldownUntil) reason = "💧 Dry Run Cooldown";
    else if (inching.active) {
      uint32_t remaining = (inching.duration - (millis() - inching.startTime)) / 1000;
      char reasonBuf[64];
      if (remaining < 60) snprintf(reasonBuf, sizeof(reasonBuf), "⏱️ INCHING - Stops in %lus", remaining);
      else snprintf(reasonBuf, sizeof(reasonBuf), "⏱️ INCHING - Stops in %lum %lus", remaining/60, remaining%60);
      reason = String(reasonBuf);
    }
    else if (!auto_mode) reason = "👆 MANUAL mode";
    else if (pressureSwitch.pressureLow) reason = "🔘 Pressure LOW - Pump ON";
    else reason = "🔘 Pressure OK - Pump OFF";
    doc["pumpReason"] = reason;
    
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
    doc["button_enabled"] = button_enabled;
    doc["pressure_lockout_hours"] = pressure_lockout_hours;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/config/lockout", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<64> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("pressure_lockout_hours")) {
        float newHours = doc["pressure_lockout_hours"].as<float>();
        if (newHours < 0) newHours = 0;
        if (newHours > 0 && newHours < (MIN_PRESSURE_LOCKOUT_MINUTES / 60.0)) {
          newHours = MIN_PRESSURE_LOCKOUT_MINUTES / 60.0;
        }
        if (newHours > MAX_PRESSURE_LOCKOUT_HOURS) newHours = MAX_PRESSURE_LOCKOUT_HOURS;
        pressure_lockout_hours = newHours;
        saveConfigToFile();
        
        char msg[96];
        if (pressure_lockout_hours == 0) {
          snprintf(msg, sizeof(msg), "Pressure lockout DISABLED");
        } else if (pressure_lockout_hours >= 24) {
          snprintf(msg, sizeof(msg), "Pressure lockout set to %.1f days", pressure_lockout_hours / 24.0);
        } else if (pressure_lockout_hours >= 1) {
          snprintf(msg, sizeof(msg), "Pressure lockout set to %.1f hours", pressure_lockout_hours);
        } else {
          snprintf(msg, sizeof(msg), "Pressure lockout set to %d minutes", (int)(pressure_lockout_hours * 60));
        }
        addFailureLogEntry(msg);
      }
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/reset_lockout", HTTP_GET, []() {
    resetPressureLockout();
    server.send(200, "text/plain", "Lockout reset");
  });
  
  server.on("/mode", HTTP_GET, []() {
    if (server.hasArg("mode")) {
      bool newMode = (server.arg("mode") == "auto");
      if (auto_mode != newMode) {
        if (auto_mode && !newMode && s31.getRelayState()) {
          s31.setRelay(false);
          cancelInchingTimer();
        }
        auto_mode = newMode;
        if (auto_mode) button.lastManualModeTime = 0;
        else button.lastManualModeTime = millis();
        saveConfigToFile();
      }
    }
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/toggle", HTTP_GET, []() {
    if (!auto_mode) {
      if (pressureLockout.active) { server.send(403, "text/plain", "Pressure lockout active"); return; }
      if (softStart.delayActive) { server.send(403, "text/plain", "Soft start active"); return; }
      if (millis() < overload.cooldownUntil) { server.send(403, "text/plain", "Overload cooldown active"); return; }
      if (millis() < dryRun.cooldownUntil) { server.send(403, "text/plain", "Dry run cooldown active"); return; }
      bool newState = !s31.getRelayState();
      s31.setRelay(newState);
      button.lastManualModeTime = millis();
      server.send(200, "text/plain", "OK");
    } else server.send(403, "text/plain", "In AUTO mode");
  });
  
  server.on("/config/protection", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("dry_run_enabled")) dry_run_enabled = doc["dry_run_enabled"];
      if (doc.containsKey("dry_run_protection")) pump_protection_time = constrain(doc["dry_run_protection"].as<uint32_t>(), 5, 300);
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
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "No data");
      return;
    }
    
    if (saveInProgress) {
      server.send(503, "text/plain", "System busy, please retry");
      return;
    }
    
    String body = server.arg("plain");
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      server.send(400, "text/plain", "Invalid JSON");
      return;
    }
    
    bool changed = false;
    if (doc.containsKey("inrush_enabled")) {
      inrush_enabled = doc["inrush_enabled"];
      changed = true;
    }
    if (doc.containsKey("inrush_tolerance_ms")) {
      inrush_tolerance_ms = constrain(doc["inrush_tolerance_ms"].as<uint32_t>(), 500, 5000);
      changed = true;
    }
    
    if (changed) {
      saveConfigToFile();
      server.send(200, "text/plain", "OK");
    } else {
      server.send(200, "text/plain", "No changes");
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
        String newHostname = doc["hostname"].as<String>();
        if (newHostname.length() > 0) {
          newHostname.toCharArray(hostname, sizeof(hostname));
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
    doc["total_energy"] = total_energy_kwh;
    doc["pump_cycles"] = pump_cycles;
    doc["overload_stop_count"] = overload_events;
    doc["dryrun_stop_count"] = dryrun_events;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/resetstats", HTTP_GET, []() {
    total_runtime_seconds = 0;
    total_energy_kwh = 0;
    pump_cycles = 0;
    overload_events = 0;
    dryrun_events = 0;
    soft_start_count = 0;
    button_press_count = 0;
    saveConfigToFile();
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/espnow/request", HTTP_GET, []() { sendEspNowCommand("get_measure"); server.send(200, "text/plain", "Request sent"); });
  server.on("/factoryreset", HTTP_GET, []() { server.send(200, "text/plain", "Resetting..."); delay(100); factoryReset(); });
  server.on("/reboot", HTTP_GET, []() { server.send(200, "text/plain", "Rebooting..."); delay(100); rebootDevice(); });
  
  server.begin();
}

// ================================================================================================
// @section     SETUP & LOOP
// ================================================================================================
void setup() {
  systemStartTime = millis();
  
  if (DEBUG_ENABLED) {
    Serial.begin(115200);
    delay(10);
  }
  
  // Initialize LittleFS with retry
  int fsRetries = 0;
  while (!LittleFS.begin() && fsRetries < 3) {
    DEBUG_FS("LittleFS mount failed, retrying...");
    delay(500);
    fsRetries++;
  }
  if (!LittleFS.begin()) {
    DEBUG_FS("LittleFS mount failed, formatting...");
    LittleFS.format();
    LittleFS.begin();
  }
  DEBUG_FS("LittleFS mounted successfully");
  
  // Load configuration
  loadConfigFromFile();
  
  uint32_t chipId = ESP.getChipId();
  deviceName = String(hostname);
  
  s31.begin();
  setupAPMode();
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
    Serial.println("\n\n╔════════════════════════════════════════════════════════════════╗");
    Serial.println("║           SMART PUMP CONTROLLER v" FIRMWARE_VERSION " - LOCKOUT EDITION        ║");
    Serial.println("╠════════════════════════════════════════════════════════════════╣");
    Serial.println("║  ✓ NEW: Pressure lockout timer (5min - 3 days)                ║");
    Serial.println("║  ✓ FIXED: No more pump cycling on/off                         ║");
    Serial.println("║  ✓ FIXED: Random reboot on settings save                      ║");
    Serial.println("║  ✓ Added save mutex to prevent concurrent writes              ║");
    Serial.println("║  ✓ Global JSON document (no stack overflow)                   ║");
    Serial.println("║  ✓ Fixed ring buffer for logs                                 ║");
    Serial.println("║  ✓ Rate limiting for file saves                               ║");
    Serial.println("╚════════════════════════════════════════════════════════════════╝");
    Serial.println("\n✓ System Ready v" FIRMWARE_VERSION);
    Serial.println("✓ WiFi AP: SmartPump-" + String(chipId & 0xFFFF, HEX));
    Serial.println("✓ Password: 12345678");
    Serial.println("✓ IP: 192.168.4.1");
    Serial.println("✓ Pressure Lockout: " + String(pressure_lockout_hours) + " hours");
    Serial.println("✓ Inrush: " + String(inrush_enabled ? "ON (" + String(inrush_tolerance_ms) + "ms)" : "OFF"));
    Serial.println("✓ Inching: " + String(inching_enabled ? "ON (" + String(inching_duration_minutes) + " min)" : "OFF"));
    Serial.println("✓ Mode: " + String(auto_mode ? "AUTO" : "MANUAL"));
    Serial.println("════════════════════════════════════════════════════════════════\n");
  }
}

void loop() {
  uint32_t currentMillis = millis();
  
  // Feed watchdog regularly
  feedSystemWatchdog();
  
  // Handle button with debouncing
  handleButton();
  readPressureSwitch();
  
  // DNS for captive portal
  if (ENABLE_CAPTIVE_PORTAL) dnsServer.processNextRequest();
  
  // Update S31 readings
  if (currentMillis - lastS31Update >= S31_UPDATE_INTERVAL) {
    lastS31Update = currentMillis;
    s31.update();
  }
  
  // Handle OTA updates
  if (ENABLE_OTA && currentMillis - lastOTA >= OTA_INTERVAL) {
    lastOTA = currentMillis;
    ArduinoOTA.handle();
  }
  
  // Handle web server
  if (currentMillis - lastWebServer >= WEB_SERVER_INTERVAL) {
    lastWebServer = currentMillis;
    server.handleClient();
  }
  
  // Request sensor data
  requestSensorData();
  
  // Control pump logic
  controlPump();
  
  // MDNS update
  if (ENABLE_MDNS && currentMillis - lastMDNS >= MDNS_INTERVAL) {
    lastMDNS = currentMillis;
    MDNS.update();
  }
  
  // Save statistics periodically
  if (currentMillis - lastStatsSave >= SAVE_STATS_INTERVAL) {
    lastStatsSave = currentMillis;
    saveStatistics();
  }
  
  // Memory monitoring
  if (ENABLE_MEMORY_MONITOR && currentMillis - lastMemoryCheck >= MEMORY_CHECK_INTERVAL) {
    lastMemoryCheck = currentMillis;
    checkMemoryAndCleanup();
  }
  
  // Handle pending saves with rate limiting
  if (pendingSave && !saveInProgress) {
    if (currentMillis - lastSaveTime >= MIN_SAVE_INTERVAL_MS) {
      saveConfigToFile();
    }
  }
  
  // WiFi recovery
  if (ENABLE_WIFI_RECOVERY) {
    recoverWiFiConnection();
  }
  
  // ESP-NOW recovery
  recoverEspNow();
  
  // Small delay to prevent watchdog issues
  delay(1);
}

// ================================================================================================
// @section     END OF CODE - v3.4 PRESSURE LOCKOUT EDITION
// ================================================================================================