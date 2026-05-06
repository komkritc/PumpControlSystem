/**
 * ================================================================================================
 * SMART PUMP CONTROLLER - COMPLETE v4.5.3 (WITH PHYSICAL BUTTON + ORIGINAL UI)
 * ================================================================================================
 * @file        AquaLevel_ESPNow_S31Controller_v4.5.3.ino
 * @version     4.5.3
 * @author      Smart Pump Controller Team
 * @license     MIT
 * @date        2025
 * @hardware    Sonoff S31 (ESP8266), Ultrasonic Sensor Node (ESP-NOW)
 * 
 * @brief       Production-grade pump controller with ESP-NOW telemetry, dual web interfaces,
 *              comprehensive protection suite, and PHYSICAL BUTTON CONTROL on GPIO0.
 * 
 * @section     FEATURES
 *              ✓ Intelligent overload protection with inrush current handling
 *              ✓ Configurable dry-run protection with enable/disable toggle
 *              ✓ ESP-NOW wireless sensor data reception
 *              ✓ Dual web UI (Simple + Engineering modes)
 *              ✓ PHYSICAL BUTTON control (GPIO0 - built-in button on Sonoff S31)
 *              ✓ EEPROM configuration with debounced writes
 *              ✓ Comprehensive protection logging
 *              ✓ Power quality monitoring (PF, apparent/reactive power)
 * 
 * @section     BUTTON OPERATION (NEW in v4.5.3)
 *              • SHORT PRESS (<3s): Toggle pump relay manually
 *                - In AUTO mode: Temporarily switches to MANUAL for 5 minutes
 *                - In MANUAL mode: Directly toggles pump ON/OFF
 *                - LED flash: Quick blink for feedback
 *              
 *              • LONG PRESS (3-10s): Force MANUAL mode permanently
 *                - Cancels any temporary auto-cancel
 *                - Switches to permanent manual control
 *                - LED flash: 3 quick blinks
 *              
 *              • SAFETY CHECKS:
 *                - Prevents pump start during overload cooldown
 *                - Respects dry run protection threshold
 *                - 50ms debounce to prevent false triggers
 *                - Hold feedback (LED flashes every second)
 * 
 * ================================================================================================
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SonoffS31.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <espnow.h>
#include <vector>
#include <cmath>

// ================================================================================================
// @section     FEATURE FLAGS & VERSION
// ================================================================================================
#define FIRMWARE_VERSION        "4.5.3"
#define ENABLE_MDNS             true
#define ENABLE_OTA              true
#define ENABLE_DEBUG_LOGGING    true
#define ENABLE_HEAP_MONITORING  true

// ================================================================================================
// @section     SYSTEM TIMING CONSTANTS
// ================================================================================================
#define S31_UPDATE_INTERVAL     100
#define CONTROL_INTERVAL        250
#define WEB_SERVER_INTERVAL     10
#define OTA_INTERVAL            50
#define MDNS_INTERVAL           1000
#define EEPROM_SAVE_DEBOUNCE_MS 2000
#define HEAP_LOG_INTERVAL_MS    30000

// ================================================================================================
// @section     BUTTON CONFIGURATION (GPIO0 - Built-in button on Sonoff S31)
// ================================================================================================
#define BUTTON_PIN              0                      /**< GPIO0 - Built-in button (LOW when pressed) */
#define BUTTON_DEBOUNCE_MS      50                     /**< Debounce time to prevent false triggers (ms) */
#define SHORT_PRESS_MAX_MS      3000                   /**< Max duration for short press (ms) */
#define LONG_PRESS_MIN_MS       3000                   /**< Min duration for long press (ms) */
#define LONG_PRESS_MAX_MS       10000                  /**< Max duration for long press (ms) */
#define BUTTON_HOLD_FEEDBACK_MS 1000                   /**< LED feedback interval when holding button (ms) */
#define LED_FEEDBACK_MS         50                     /**< LED flash duration for button feedback (ms) */

// ================================================================================================
// @section     ESP-NOW TIMING & RELIABILITY CONSTANTS
// ================================================================================================
#define ESP_NOW_SEND_INTERVAL   15000
#define ESP_NOW_DATA_TIMEOUT    30000
#define SENSOR_HEARTBEAT_TIMEOUT 120000
#define MAX_LOG_ENTRIES         50
#define ESP_NOW_RETRY_COUNT     3
#define ESP_NOW_RETRY_DELAY_MS  50

// ================================================================================================
// @section     PIN DEFINITIONS & HARDWARE CONFIG
// ================================================================================================
#define RELAY_PIN               12                     /**< Sonoff S31 relay control pin (GPIO12) */
#define LED_PIN                 13                     /**< Built-in LED on Sonoff S31 (GPIO13, active low) */
#define FLOAT_EPSILON           0.01f                  /**< Tolerance for float threshold comparisons */

// ================================================================================================
// @enum        ButtonState
// @brief       State machine states for physical button handling
// ================================================================================================
enum ButtonState {
  BUTTON_IDLE,                  /**< No button activity */
  BUTTON_PRESSED,               /**< Button currently pressed (not yet debounced) */
  BUTTON_DEBOUNCE_WAIT,         /**< Waiting for debounce period to complete */
  BUTTON_SHORT_PRESS_DETECTED,  /**< Short press action triggered */
  BUTTON_LONG_PRESS_DETECTED    /**< Long press action triggered */
};

// ================================================================================================
// @struct      Config
// @brief       System configuration structure persisted in EEPROM (512 bytes)
// @note        All numeric fields sanitized with constrain() on load to prevent invalid values
// ================================================================================================
struct Config {
  uint32_t magic = 0xDEADBEEF;                      /**< Validation magic number for EEPROM integrity */
  
  // Water level control settings (hysteresis band)
  float low_threshold = 30.0;                      /**< Level % below which pump turns ON (Auto mode) */
  float high_threshold = 80.0;                     /**< Level % above which pump turns OFF (Auto mode) */
  bool auto_mode = true;                           /**< true = Auto level control, false = Manual toggle */
  
  // Dry run protection settings (configurable enable/disable)
  bool dry_run_enabled = true;                     /**< Master toggle for dry run protection feature */
  float min_power_threshold = 5.0;                 /**< Minimum power (Watts) indicating valid pump operation */
  unsigned long pump_protection_time = 20;         /**< Seconds of low power before dry run shutdown */
  
  // Overload protection settings (works in BOTH Auto and Manual modes)
  float max_power_threshold = 500.0;               /**< Maximum safe power (Watts) - overload trigger */
  bool pump_load_protection_enabled = true;        /**< Master toggle for overload protection feature */
  unsigned long overload_cooldown_seconds = 30;    /**< Cooldown period after overload before restart allowed */
  unsigned long inrush_tolerance_ms = 2000;        /**< Time window to ignore high current at pump startup */
  
  // ESP-NOW communication settings
  bool use_espnow = true;                          /**< Enable/disable ESP-NOW sensor communication */
  uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  /**< Sensor MAC address (broadcast default) */
  char peer_mac_str[18] = "FF:FF:FF:FF:FF:FF";     /**< Human-readable MAC string for UI display */
  int espnow_channel = 1;                          /**< WiFi channel for ESP-NOW (1-13, must match sensor) */
  
  // Sensor failure handling settings
  uint8_t sensor_failure_strategy = 0;             /**< 0=Stop,1=Maintain,2=ForceON,3=Cyclic */
  unsigned long sensor_timeout = SENSOR_HEARTBEAT_TIMEOUT;  /**< Milliseconds before sensor marked dead */
  bool sensor_emergency_stop = true;               /**< Emergency stop on sensor failure (safety override) */
  unsigned long cyclic_on_duration = 300000;       /**< Cyclic mode: pump ON duration (ms) */
  unsigned long cyclic_off_duration = 1800000;     /**< Cyclic mode: pump OFF duration (ms) */
  
  // System settings
  char hostname[32] = "s31-pump";                  /**< Device hostname for mDNS/OTA */
  
  // Button settings (NEW in v4.5.3)
  bool button_enabled = true;                      /**< Enable/disable physical button control */
  bool button_auto_cancel = true;                  /**< Pressing in Auto mode temporarily cancels Auto */
  unsigned long button_auto_cancel_timeout = 300000; /**< Auto-cancel timeout (5min default, back to Auto) */
} config;

// ================================================================================================
// @struct      OverloadProtectionState
// @brief       Runtime state machine for intelligent overload protection with inrush handling
// ================================================================================================
struct OverloadProtectionState {
  // Protection state flags
  bool active = false;                             /**< Overload protection currently triggered? */
  unsigned long cooldownUntil = 0;                 /**< Timestamp (millis) when cooldown period ends */
  
  // Inrush current handling
  bool inrushActive = false;                       /**< Currently ignoring power during startup surge? */
  unsigned long pumpStartTime = 0;                 /**< millis() timestamp when pump was last started */
  bool lastRelayState = false;                     /**< Previous relay state for edge detection */
  bool bootInitialized = false;                    /**< Prevents false start detection at boot */
  
  // Statistics and diagnostic logging
  unsigned long overloadCount = 0;                 /**< Total number of overload events (lifetime) */
  float lastOverloadPower = 0;                     /**< Power reading from most recent overload event */
  unsigned long lastCooldownLogTime = 0;           /**< Prevents repetitive cooldown log messages */
  unsigned long lastDebugLogTime = 0;              /**< Prevents debug log spam during rapid events */
  
  // Manual mode coordination
  bool manualStartPending = false;                 /**< Manual start awaiting inrush period completion */
  unsigned long lastManualStartTime = 0;           /**< Timestamp of last manual pump start command */
} overload;

// ================================================================================================
// @struct      DryRunState
// @brief       Runtime state for configurable dry run protection
// ================================================================================================
struct DryRunState {
  unsigned long lowPowerStartTime = 0;             /**< millis() when low power first detected */
  bool protectionTriggered = false;                /**< Has dry run shutdown been activated? */
  float lastLoggedPower = 0;                       /**< Last power value logged (prevents spam) */
  unsigned long lastLogTime = 0;                   /**< Last log timestamp for rate limiting */
} dryRun;

// ================================================================================================
// @struct      ButtonHandler
// @brief       Runtime state for physical button handling (GPIO0)
// ================================================================================================
struct ButtonHandler {
  ButtonState state = BUTTON_IDLE;                 /**< Current button state machine state */
  unsigned long pressStartTime = 0;                /**< When button was first pressed (millis) */
  unsigned long lastDebounceTime = 0;              /**< Last debounce check timestamp */
  bool lastButtonState = HIGH;                     /**< Previous button reading (for edge detection) */
  bool currentButtonState = HIGH;                  /**< Current debounced button state */
  bool buttonPressed = false;                      /**< Flag: button currently being pressed */
  unsigned long lastFeedbackTime = 0;              /**< Last feedback timestamp (for hold feedback) */
  
  // Auto-cancel tracking (temporary manual override)
  bool autoModeCancelled = false;                  /**< Flag: Auto mode was temporarily cancelled */
  unsigned long autoModeCancelTime = 0;            /**< When Auto mode was cancelled (millis) */
  
  // Statistics
  unsigned long shortPressCount = 0;               /**< Total short press events (lifetime) */
  unsigned long longPressCount = 0;                /**< Total long press events (lifetime) */
} button;

// ================================================================================================
// @section     GLOBAL OBJECTS & INSTANCES
// ================================================================================================
SonoffS31 s31(RELAY_PIN);                          /**< Sonoff power monitoring and relay control */
ESP8266WebServer server(80);                       /**< HTTP web server instance */
String deviceName = "s31-pump";                    /**< Device hostname for mDNS/OTA identification */

// ================================================================================================
// @section     TIMING VARIABLES (millis() based, overflow-safe)
// ================================================================================================
unsigned long lastS31Update = 0;
unsigned long lastControlCheck = 0;
unsigned long lastWebServer = 0;
unsigned long lastOTA = 0;
unsigned long lastMDNS = 0;
unsigned long lastHeapLog = 0;

// ================================================================================================
// @section     EEPROM CONFIGURATION MANAGEMENT (Debounced Writes)
// ================================================================================================
unsigned long lastConfigWrite = 0;                 /**< Timestamp of last EEPROM write */
bool pendingConfigSave = false;                    /**< Flag: config changed, awaiting debounce */

void queueConfigSave() {
  pendingConfigSave = true;
  lastConfigWrite = millis();
}

void processPendingConfigSave() {
  if (pendingConfigSave && (millis() - lastConfigWrite >= EEPROM_SAVE_DEBOUNCE_MS)) {
    saveConfig();
    pendingConfigSave = false;
    #if ENABLE_DEBUG_LOGGING
      Serial.println("[CONFIG] EEPROM save completed (debounced)");
    #endif
  }
}

// ================================================================================================
// @section     SENSOR DATA & SYSTEM STATE VARIABLES
// ================================================================================================
float currentWaterLevel = 0;
float currentDistance = 0;
float currentVolume = 0;
float batteryVoltage = 0;
unsigned long lastEspNowData = 0;
bool espnowDataValid = false;
bool sensorIsDead = false;
bool sensorWarningIssued = false;
unsigned long sensorDeadStartTime = 0;

// ================================================================================================
// @section     CYCLIC MODE & POWER QUALITY VARIABLES
// ================================================================================================
unsigned long cyclicLastSwitchTime = 0;
bool cyclicPumpState = false;
float powerFactor = 0.0;
float apparentPower = 0.0;
float reactivePower = 0.0;

// ================================================================================================
// @struct      EspNowPacket
// @brief       Packed structure for ESP-NOW data packets (must match sensor firmware)
// ================================================================================================
typedef struct __attribute__((packed)) {
  uint32_t seq;                                    /**< Sequence number for message ordering/duplication */
  uint32_t timestamp;                              /**< Microsecond timestamp from sender */
  char msg[64];                                    /**< JSON payload or command string */
} EspNowPacket;

EspNowPacket outgoing;                             /**< Outgoing command packet buffer */
EspNowPacket incoming;                             /**< Incoming sensor data packet buffer */

// ================================================================================================
// @struct      EspNowLogEntry
// @brief       Structured log entry for ESP-NOW message history and diagnostics
// ================================================================================================
struct EspNowLogEntry {
  unsigned long timestamp;
  String mac;
  String rawData;
  float distance;
  float level;
  float volume;
  float battery;
  bool valid;
};

// ================================================================================================
// @section     ESP-NOW GLOBALS & LOGGING
// ================================================================================================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool espnow_initialized = false;
unsigned long lastEspNowSend = 0;
uint32_t espnow_seq = 0;
uint32_t espnow_msg_counter = 0;
std::vector<EspNowLogEntry> espnow_log;            /**< Circular buffer for ESP-NOW diagnostics */

// ================================================================================================
// @struct      PumpStats
// @brief       Pump operational statistics for maintenance and energy monitoring
// ================================================================================================
struct PumpStats {
  unsigned long totalRuntimeSeconds = 0;           /**< Cumulative pump ON time (seconds) */
  float totalEnergyKwh = 0;                        /**< Cumulative energy consumed (kWh) */
  int pumpCycles = 0;                              /**< Number of pump start events (lifetime) */
  char lastStartStr[32] = "Never";                 /**< Human-readable last start timestamp */
  unsigned long lastPumpOnTime = 0;                /**< millis() when pump was last turned ON */
  bool wasRunning = false;                         /**< Previous pump state for runtime calculation */
} pumpStats;

std::vector<String> failureLog;                    /**< Circular buffer of protection events */

// ================================================================================================
// @section     DEBUG LOGGING MACROS (Conditional Compilation)
// ================================================================================================
#if ENABLE_DEBUG_LOGGING
  #define DEBUG_LOG(msg) do { addFailureLogEntry("[DEBUG] " + String(msg)); \
                              if(ENABLE_HEAP_MONITORING && millis()-lastHeapLog>HEAP_LOG_INTERVAL_MS) { \
                                Serial.printf("[HEAP] Free: %d bytes\n", ESP.getFreeHeap()); \
                                lastHeapLog = millis(); \
                              } \
                            } while(0)
  #define DEBUG_OVERLOAD(msg) addFailureLogEntry("[OVERLOAD] " + String(msg))
  #define DEBUG_DRYRUN(msg) addFailureLogEntry("[DRYRUN] " + String(msg))
  #define DEBUG_BUTTON(msg) addFailureLogEntry("[BUTTON] " + String(msg))
  #define DEBUG_HEAP() Serial.printf("[HEAP] Free: %d bytes\n", ESP.getFreeHeap())
#else
  #define DEBUG_LOG(msg)
  #define DEBUG_OVERLOAD(msg)
  #define DEBUG_DRYRUN(msg)
  #define DEBUG_BUTTON(msg)
  #define DEBUG_HEAP()
#endif

// ================================================================================================
// @section     ORIGINAL V4.5.0 SIMPLE HTML UI (FULLY PRESERVED)
// ================================================================================================
const char simple_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>Smart Pump | Easy Control v4.5.3</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Roboto, system-ui, sans-serif; background: #f1f5f9; padding: 16px; color: #0f172a; }
        .container { max-width: 550px; margin: 0 auto; }
        .card { background: white; border-radius: 32px; padding: 20px 18px; margin-bottom: 18px; box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05); }
        .header { text-align: center; margin-bottom: 8px; }
        .header h1 { font-size: 1.7rem; font-weight: 600; background: linear-gradient(135deg, #0f172a, #2563eb); background-clip: text; -webkit-background-clip: text; color: transparent; }
        .version-badge { background: #2563eb20; color: #1e40af; padding: 4px 12px; border-radius: 40px; font-size: 0.7rem; display: inline-block; margin-top: 6px; }
        .badge { background: #e2e8f0; padding: 6px 12px; border-radius: 40px; font-size: 0.7rem; font-weight: 500; display: inline-block; margin-top: 6px; }
        .offline-tag { background: #fef9c3; color: #854d0e; font-size: 0.7rem; border-radius: 30px; padding: 4px 12px; display: inline-block; margin-top: 6px; }
        .nav-tabs { display: flex; gap: 8px; margin-bottom: 18px; background: white; padding: 6px; border-radius: 60px; }
        .tab-btn { flex: 1; padding: 10px; border: none; background: transparent; border-radius: 50px; font-weight: 600; font-size: 0.85rem; cursor: pointer; color: #64748b; }
        .tab-btn.active { background: #2563eb; color: white; }
        .level-gauge-container { text-align: center; margin: 10px 0 8px; }
        .gauge-title { font-size: 0.8rem; color: #475569; margin-bottom: 8px; font-weight: 500; }
        .vertical-gauge { width: 180px; height: 200px; margin: 0 auto; background: #e2e8f0; border-radius: 30px; position: relative; overflow: hidden; box-shadow: inset 0 0 0 3px white, 0 4px 12px rgba(0,0,0,0.1); }
        .water-fill-vertical { background: linear-gradient(180deg, #3b82f6, #1e40af); position: absolute; bottom: 0; left: 0; right: 0; transition: height 0.5s ease; display: flex; align-items: center; justify-content: center; color: white; font-weight: bold; font-size: 1.2rem; }
        .level-text-large { font-size: 2rem; font-weight: 800; margin-top: 12px; color: #1e293b; }
        .stats-row { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; margin: 16px 0; }
        .stat-block { background: #f8fafc; border-radius: 24px; padding: 12px; text-align: center; }
        .stat-value { font-size: 1.8rem; font-weight: 700; }
        .stat-label { font-size: 0.7rem; text-transform: uppercase; color: #475569; }
        .mode-row { display: flex; justify-content: space-between; align-items: center; background: #f1f5f9; padding: 12px 16px; border-radius: 60px; margin: 16px 0 12px; }
        .toggle-switch { position: relative; display: inline-block; width: 56px; height: 28px; }
        .toggle-switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #cbd5e1; transition: 0.3s; border-radius: 34px; }
        .slider:before { position: absolute; content: ""; height: 22px; width: 22px; left: 3px; bottom: 3px; background-color: white; transition: 0.3s; border-radius: 50%; }
        input:checked + .slider { background-color: #2563eb; }
        input:checked + .slider:before { transform: translateX(28px); }
        .pump-btn { width: 100%; padding: 18px; border-radius: 60px; border: none; font-weight: 700; font-size: 1.4rem; background: #dc2626; color: white; margin: 12px 0 8px; cursor: pointer; transition: 0.2s; }
        .pump-btn.running { background: #10b981; animation: pulse 1.8s infinite; }
        .pump-btn.overload-warning { background: #f59e0b; animation: pulse 0.5s infinite; }
        .pump-btn.cooldown { background: #6b7280; cursor: not-allowed; }
        .pump-btn.inrush { background: #8b5cf6; animation: pulse 1s infinite; }
        .pump-btn:disabled { opacity: 0.5; cursor: not-allowed; }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 #f59e0b80; } 70% { box-shadow: 0 0 0 15px #f59e0b00; } 100% { box-shadow: 0 0 0 0 #f59e0b00; } }
        .setting-group { margin-bottom: 20px; }
        .setting-group label { display: block; font-weight: 600; margin-bottom: 8px; }
        .setting-group input { width: 100%; padding: 12px; border: 1.5px solid #e2e8f0; border-radius: 20px; font-size: 1rem; }
        .setting-group input:focus { outline: none; border-color: #2563eb; }
        .setting-group .checkbox-label { display: flex; align-items: center; gap: 12px; cursor: pointer; }
        .setting-group .checkbox-label input { width: auto; margin-right: 8px; }
        .save-btn { background: #2563eb; color: white; border: none; padding: 12px 24px; border-radius: 40px; font-weight: 600; width: 100%; cursor: pointer; margin-top: 10px; }
        .sensor-chip { background: #e6f7ec; padding: 5px 10px; border-radius: 50px; font-size: 0.7rem; font-weight: 500; display: inline-flex; align-items: center; gap: 6px; }
        .led { width: 10px; height: 10px; border-radius: 10px; display: inline-block; }
        .led-green { background: #22c55e; }
        .led-red { background: #ef4444; }
        .led-yellow { background: #eab308; }
        .flex-between { display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; gap: 8px; }
        .btn-secondary { background: #e2e8f0; border: none; padding: 10px 16px; border-radius: 40px; font-weight: 500; width: 100%; cursor: pointer; margin-top: 8px; }
        .engmode-link { text-align: center; margin-top: 12px; font-size: 0.7rem; }
        .engmode-link a { color: #94a3b8; text-decoration: none; }
        hr { margin: 14px 0; border: none; border-top: 1px solid #e2e8f0; }
        .small-note { font-size: 0.7rem; color: #64748b; text-align: center; margin-top: 12px; }
        .warning-text { color: #d97706; font-size: 0.7rem; margin-top: 4px; }
        .info-text { color: #3b82f6; font-size: 0.7rem; margin-top: 4px; }
        .protection-badge { display: inline-block; background: #fef3c7; color: #92400e; padding: 4px 8px; border-radius: 20px; font-size: 0.7rem; margin-top: 8px; animation: pulse 1s infinite; }
        .cooldown-badge { background: #e5e7eb; color: #374151; }
        .inrush-badge { background: #ede9fe; color: #5b21b6; }
        .dryrun-badge { background: #fed7aa; color: #9a3412; }
        .button-info { background: #e0e7ff; padding: 8px 12px; border-radius: 20px; font-size: 0.7rem; margin-top: 10px; text-align: center; }
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>💧 AquaPro S31 Controller</h1>
        <div><span class="version-badge">v4.5.3 - Physical Button + Inrush Fixed ✓</span></div>
        <div class="badge">ESP-NOW | Physical Button (GPIO0)</div>
        <div class="offline-tag">🔌 Connect to AP: SmartPump-XXXX | 192.168.4.1</div>
    </div>
    <div class="nav-tabs">
        <button class="tab-btn active" onclick="switchTab('dashboard')">📊 Dashboard</button>
        <button class="tab-btn" onclick="switchTab('pumpsettings')">⚙️ Pump Settings</button>
    </div>
    <div id="dashboardSection">
        <div class="card">
            <div class="flex-between"><span>📡 Sensor status</span><span id="sensorBadge" class="sensor-chip"><span class="led led-green"></span> Healthy</span></div>
            <div class="flex-between" style="margin-top: 10px;"><span>🕒 Last reading:</span><span id="lastSeenText" style="font-family: monospace;">--</span></div>
            <div class="button-info">🔘 Physical Button: Short press = Toggle pump | Long press = Force Manual mode</div>
        </div>
        <div class="card">
            <div class="level-gauge-container">
                <div class="gauge-title">💧 Water Tank Level</div>
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
            <div class="stats-row">
                <div class="stat-block"><div class="stat-value"><span id="powerNow">0</span> W</div><div class="stat-label">Power</div></div>
                <div class="stat-block"><div class="stat-value"><span id="energyToday">0.0</span> kWh</div><div class="stat-label">Energy used</div></div>
            </div>
            <div class="flex-between"><span>⚡ Power factor</span><span id="pfValue" style="font-weight: 600;">0.00</span></div>
            <div id="loadWarning" class="protection-badge" style="display: none;">⚠️ OVERLOAD PROTECTION ACTIVE!</div>
            <div id="inrushWarning" class="protection-badge inrush-badge" style="display: none;">⚡ INRUSH TOLERANCE ACTIVE</div>
            <div id="cooldownWarning" class="protection-badge cooldown-badge" style="display: none;">⏱️ Cooldown period active</div>
            <div id="dryRunWarning" class="protection-badge dryrun-badge" style="display: none;">💧 DRY RUN DETECTED!</div>
        </div>
        <div class="card">
            <div class="mode-row"><span class="mode-text">🤖 Auto mode</span><label class="toggle-switch"><input type="checkbox" id="autoModeToggle" onchange="toggleAutoMode()"><span class="slider"></span></label><span class="mode-text">👆 Manual</span></div>
            <button id="pumpActionBtn" class="pump-btn" onclick="manualPumpToggle()">PUMP OFF</button>
            <div id="pumpHint" style="font-size: 0.7rem; text-align: center;">✅ Auto mode handles pump</div>
            <div id="protectionNote" style="font-size: 0.7rem; text-align: center; color: #d97706; margin-top: 4px;">🛡️ Overload+Inrush protection ACTIVE in BOTH modes | Dry Run configurable</div>
            <hr>
            <div class="flex-between"><span>📦 Total runtime</span><strong><span id="totalRunMinutes">0</span> min</strong></div>
            <div class="flex-between"><span>🔄 Cycles count</span><strong><span id="cyclesCount">0</span></strong></div>
            <div class="flex-between"><span>⚠️ Overload events</span><strong><span id="overloadCount">0</span></strong></div>
            <div class="flex-between"><span>🔘 Button presses</span><strong><span id="buttonPressCount">0</span> (Short: <span id="shortPressCount">0</span> / Long: <span id="longPressCount">0</span>)</strong></div>
        </div>
        <div class="card">
            <button class="btn-secondary" onclick="triggerSensorRead()">📡 Request sensor reading now</button>
            <button class="btn-secondary" style="background:#fee2e2; color:#b91c1c;" onclick="confirmReboot()">🔄 Reboot device</button>
        </div>
    </div>
    <div id="pumpSettingsSection" style="display: none;">
        <div class="card">
            <h3 style="margin-bottom: 16px;">⚙️ Pump Control Settings</h3>
            <div class="setting-group"><label>💧 Pump ON when water level below</label><input type="number" id="lowThreshold" step="5" min="0" max="100"><div class="small-note">Example: 30% → pump starts when tank ≤30%</div></div>
            <div class="setting-group"><label>🛑 Pump OFF when water level above</label><input type="number" id="highThreshold" step="5" min="0" max="100"><div class="small-note">Example: 80% → pump stops when tank ≥80%</div></div>
            <hr>
            <div class="setting-group">
                <label class="checkbox-label">
                    <input type="checkbox" id="dryRunToggle"> 💧 Enable Dry Run Protection
                </label>
                <div class="small-note">★ Stops pump if low power detected for configured time (Auto mode only) ★</div>
            </div>
            <div class="setting-group"><label>⚠️ Dry run protection (seconds)</label><input type="number" id="dryRunProtection" step="5" min="5"><div class="small-note">✅ Pump stops after EXACTLY this many seconds of low power</div></div>
            <div class="setting-group"><label>⚡ Minimum Power Threshold (Watts)</label><input type="number" id="minPower" step="1" min="1" placeholder="5"><div class="small-note">Power below this value = dry run condition</div></div>
            <hr>
            <div class="setting-group">
                <label class="checkbox-label">
                    <input type="checkbox" id="loadProtectionToggle"> 🛡️ Enable Pump Overload Protection
                </label>
                <div class="small-note">★ PROTECTS IN BOTH AUTO AND MANUAL MODES ★</div>
                <div class="warning-text">⚠️ Stops pump if power exceeds safe limit (AFTER inrush period)</div>
            </div>
            <div class="setting-group"><label>⚡ Max Safe Power (Watts)</label><input type="number" id="maxPowerThreshold" step="10" min="10" placeholder="500"><div class="warning-text">Pump will stop if power exceeds this value AFTER startup inrush</div></div>
            <div class="setting-group"><label>⏱️ Inrush Tolerance (milliseconds)</label><input type="number" id="inrushTolerance" step="100" min="100" max="10000" placeholder="2000"><div class="info-text">Time to ignore high current at startup (2-7x normal). 1000-3000ms recommended.</div></div>
            <div class="setting-group"><label>⏱️ Cooldown Period (seconds)</label><input type="number" id="cooldownPeriod" step="5" min="0" placeholder="30"><div class="small-note">How long to wait before allowing pump restart after overload</div></div>
            <hr>
            <h4>🔘 Button Settings</h4>
            <div class="setting-group">
                <label class="checkbox-label">
                    <input type="checkbox" id="buttonEnabled"> ✅ Enable Physical Button
                </label>
            </div>
            <div class="setting-group">
                <label class="checkbox-label">
                    <input type="checkbox" id="buttonAutoCancel"> 🔄 Auto-cancel on press (Auto → Temp Manual)
                </label>
            </div>
            <div class="setting-group"><label>⏱️ Auto-cancel timeout (minutes)</label><input type="number" id="buttonAutoCancelTimeout" step="1" min="1" max="60" placeholder="5"><div class="small-note">How long before returning to Auto mode after button press</div></div>
            <button class="save-btn" onclick="savePumpSettings()">💾 Save All Settings</button>
        </div>
    </div>
    <div class="engmode-link"><a href="/engmode">🔧 Engineering Mode (advanced settings)</a></div>
</div>
<script>
    function switchTab(tab) {
        const dashboard = document.getElementById('dashboardSection');
        const settings = document.getElementById('pumpSettingsSection');
        const btns = document.querySelectorAll('.tab-btn');
        if(tab === 'dashboard') {
            dashboard.style.display = 'block';
            settings.style.display = 'none';
            btns[0].classList.add('active');
            btns[1].classList.remove('active');
        } else {
            dashboard.style.display = 'none';
            settings.style.display = 'block';
            btns[0].classList.remove('active');
            btns[1].classList.add('active');
            loadPumpSettings();
        }
    }
    async function fetchJSON(url) { try { const r = await fetch(url); return await r.json(); } catch(e) { return null; } }
    async function loadPumpSettings() { 
        const cfg = await fetchJSON('/config'); 
        if(cfg) { 
            document.getElementById('lowThreshold').value = cfg.low_threshold || 30; 
            document.getElementById('highThreshold').value = cfg.high_threshold || 80; 
            document.getElementById('dryRunToggle').checked = cfg.dry_run_enabled !== false;
            document.getElementById('dryRunProtection').value = cfg.dry_run_protection || 20;
            document.getElementById('minPower').value = cfg.min_power || 5;
            document.getElementById('loadProtectionToggle').checked = cfg.pump_load_protection_enabled !== false;
            document.getElementById('maxPowerThreshold').value = cfg.max_power_threshold || 500;
            document.getElementById('inrushTolerance').value = cfg.inrush_tolerance_ms || 2000;
            document.getElementById('cooldownPeriod').value = cfg.overload_cooldown_seconds || 30;
            document.getElementById('buttonEnabled').checked = cfg.button_enabled !== false;
            document.getElementById('buttonAutoCancel').checked = cfg.button_auto_cancel !== false;
            document.getElementById('buttonAutoCancelTimeout').value = (cfg.button_auto_cancel_timeout || 300000) / 60000;
        } 
    }
    async function savePumpSettings() {
        const low = parseFloat(document.getElementById('lowThreshold').value);
        const high = parseFloat(document.getElementById('highThreshold').value);
        const dryRunEnabled = document.getElementById('dryRunToggle').checked;
        const dryTime = parseInt(document.getElementById('dryRunProtection').value);
        const minPower = parseFloat(document.getElementById('minPower').value);
        const loadProtection = document.getElementById('loadProtectionToggle').checked;
        const maxPower = parseFloat(document.getElementById('maxPowerThreshold').value);
        const inrush = parseInt(document.getElementById('inrushTolerance').value);
        const cooldown = parseInt(document.getElementById('cooldownPeriod').value);
        const buttonEnabled = document.getElementById('buttonEnabled').checked;
        const buttonAutoCancel = document.getElementById('buttonAutoCancel').checked;
        const buttonTimeout = parseInt(document.getElementById('buttonAutoCancelTimeout').value) * 60000;
        if(low >= high) { alert("Low threshold must be less than High threshold"); return; }
        if(maxPower < 10) { alert("Max power must be at least 10 watts"); return; }
        if(inrush < 100) { alert("Inrush tolerance must be at least 100ms"); return; }
        if(dryTime < 5) { alert("Dry run protection must be at least 5 seconds"); return; }
        if(minPower < 1) { alert("Minimum power threshold must be at least 1 watt"); return; }
        const response = await fetch('/config/pump', { 
            method: 'POST', 
            headers: { 'Content-Type': 'application/json' }, 
            body: JSON.stringify({ 
                low_threshold: low, 
                high_threshold: high,
                dry_run_enabled: dryRunEnabled,
                dry_run_protection: dryTime,
                min_power: minPower,
                pump_load_protection_enabled: loadProtection,
                max_power_threshold: maxPower,
                inrush_tolerance_ms: inrush,
                overload_cooldown_seconds: cooldown,
                button_enabled: buttonEnabled,
                button_auto_cancel: buttonAutoCancel,
                button_auto_cancel_timeout: buttonTimeout
            }) 
        });
        if(response.ok) alert("✅ Settings saved successfully!");
        else alert("❌ Failed to save settings");
    }
    async function toggleAutoMode() { const isAuto = document.getElementById('autoModeToggle').checked; await fetch(`/mode?mode=${isAuto ? 'auto' : 'manual'}`); updatePumpButtonAccess(); refreshDashboard(); }
    function updatePumpButtonAccess() { 
        const isAuto = document.getElementById('autoModeToggle').checked; 
        const btn = document.getElementById('pumpActionBtn'); 
        const hint = document.getElementById('pumpHint'); 
        if(isAuto) { 
            btn.disabled = true; 
            btn.style.opacity = "0.6"; 
            hint.innerText = "🔒 Auto mode active — pump managed automatically"; 
        } else { 
            btn.style.opacity = "1"; 
            hint.innerText = "✋ Manual mode: tap to turn pump ON/OFF (Overload+Inrush protection active)"; 
            fetchJSON('/data').then(d => {
                if(d && !d.inCooldown) btn.disabled = false;
                else if(d && d.inCooldown) btn.disabled = true;
            });
        } 
    }
    async function manualPumpToggle() { 
        if(document.getElementById('autoModeToggle').checked) { 
            alert("Switch to Manual mode first"); 
            return; 
        } 
        const d = await fetchJSON('/data');
        if(d && d.inCooldown) {
            alert("⏱️ Pump is in cooldown period after overload protection!\nPlease wait for cooldown to expire.");
            return;
        }
        await fetch('/toggle'); 
        refreshDashboard(); 
    }
    async function triggerSensorRead() { const btn = event.target; const orig = btn.innerText; btn.innerText = "📡 Sending..."; await fetch('/espnow/request'); btn.innerText = "✅ Sent!"; setTimeout(() => btn.innerText = orig, 1500); refreshDashboard(); }
    function confirmReboot() { if(confirm("Reboot pump controller?")) { fetch('/reboot'); alert("Rebooting..."); setTimeout(() => location.reload(), 3000); } }
    function getBatteryStatus(voltage) {
        if(voltage >= 3.8) return "🔋 Full";
        if(voltage >= 3.5) return "🔋 Good";
        if(voltage >= 3.2) return "🪫 Low";
        return "⚠️ Critical";
    }
    async function refreshDashboard() {
        const d = await fetchJSON('/data');
        if(!d) return;
        const level = Math.min(100, Math.max(0, d.waterLevel || 0));
        document.getElementById('levelPercent').innerText = Math.floor(level);
        document.getElementById('waterFillVertical').style.height = level + '%';
        document.getElementById('waterFillVertical').innerText = Math.floor(level) + '%';
        const voltage = (d.batteryVoltage || 0);
        document.getElementById('sensorVoltage').innerHTML = voltage.toFixed(2) + ' V <span style="font-size:0.7rem;">' + getBatteryStatus(voltage) + '</span>';
        document.getElementById('volumeVal').innerText = (d.volume || 0).toFixed(0);
        document.getElementById('powerNow').innerText = (d.power || 0).toFixed(1);
        document.getElementById('energyToday').innerText = (d.energy || 0).toFixed(1);
        document.getElementById('pfValue').innerHTML = (d.powerFactor || 0).toFixed(2);
        document.getElementById('overloadCount').innerText = d.overloadCount || 0;
        document.getElementById('shortPressCount').innerText = d.buttonShortPressCount || 0;
        document.getElementById('longPressCount').innerText = d.buttonLongPressCount || 0;
        document.getElementById('buttonPressCount').innerText = (d.buttonShortPressCount || 0) + (d.buttonLongPressCount || 0);
        
        const btn = document.getElementById('pumpActionBtn');
        if(d.inCooldown) {
            btn.innerText = "⏱️ COOLDOWN - " + d.cooldownRemaining + "s";
            btn.classList.add('cooldown');
            btn.classList.remove('running', 'inrush');
            btn.disabled = true;
        } else if(d.inInrushPeriod) {
            btn.innerText = "⚡ STARTING - Inrush mode";
            btn.classList.add('inrush');
            btn.classList.remove('running', 'cooldown');
            if(!document.getElementById('autoModeToggle').checked) btn.disabled = false;
        } else if(d.pumpState) { 
            btn.innerText = "💧 PUMP RUNNING"; 
            btn.classList.add('running');
            btn.classList.remove('inrush', 'cooldown');
            if(!document.getElementById('autoModeToggle').checked) btn.disabled = false;
        } else { 
            btn.innerText = "⏹️ PUMP OFF"; 
            btn.classList.remove('running', 'inrush', 'cooldown');
            if(!document.getElementById('autoModeToggle').checked) btn.disabled = false;
        }
        
        if(d.overloadProtectionActive) {
            document.getElementById('loadWarning').style.display = 'inline-block';
            btn.classList.add('overload-warning');
        } else {
            document.getElementById('loadWarning').style.display = 'none';
            btn.classList.remove('overload-warning');
        }
        
        if(d.inInrushPeriod) {
            document.getElementById('inrushWarning').style.display = 'inline-block';
        } else {
            document.getElementById('inrushWarning').style.display = 'none';
        }
        
        if(d.inCooldown) {
            document.getElementById('cooldownWarning').style.display = 'inline-block';
            document.getElementById('cooldownWarning').innerHTML = '⏱️ Cooldown: ' + d.cooldownRemaining + 's remaining';
        } else {
            document.getElementById('cooldownWarning').style.display = 'none';
        }
        
        if(d.dryRunActive) {
            document.getElementById('dryRunWarning').style.display = 'inline-block';
            document.getElementById('dryRunWarning').innerHTML = '💧 Dry Run: ' + d.dryRunTime + 's';
        } else {
            document.getElementById('dryRunWarning').style.display = 'none';
        }
        
        const sensorSpan = document.getElementById('sensorBadge');
        if(d.sensorHealthy) sensorSpan.innerHTML = '<span class="led led-green"></span> ✅ Healthy';
        else if(d.sensorWarning) sensorSpan.innerHTML = '<span class="led led-yellow"></span> ⚠️ Weak signal';
        else sensorSpan.innerHTML = '<span class="led led-red"></span> ❌ No sensor data';
        document.getElementById('lastSeenText').innerText = d.sensorLastSeen || 'never';
        const toggle = document.getElementById('autoModeToggle');
        if(toggle && toggle.checked !== (d.autoMode === true)) { toggle.checked = (d.autoMode === true); updatePumpButtonAccess(); }
        const stats = await fetchJSON('/stats');
        if(stats) { document.getElementById('totalRunMinutes').innerText = stats.total_runtime || 0; document.getElementById('cyclesCount').innerText = stats.pump_cycles || 0; }
    }
    setInterval(refreshDashboard, 1000);
    window.onload = () => { refreshDashboard(); loadPumpSettings(); updatePumpButtonAccess(); };
</script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     ORIGINAL V4.5.0 ENGINEERING HTML UI (PRESERVED)
// ================================================================================================
const char engineering_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Pump Controller - Engineering Mode v4.5.3</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; border-radius: 20px; padding: 25px; margin-bottom: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
        h1 { text-align: center; color: #333; margin-bottom: 5px; font-size: 24px; }
        .version { text-align: center; color: #10B981; font-size: 12px; margin-bottom: 5px; font-weight: bold; }
        .eng-badge { text-align: center; background: #D1FAE5; color: #065F46; padding: 8px; border-radius: 10px; font-size: 12px; margin-bottom: 15px; font-weight: bold; }
        .nav-buttons { display: flex; gap: 10px; margin-bottom: 20px; flex-wrap: wrap; }
        .nav-btn { flex: 1; background: #e5e7eb; color: #333; padding: 10px; border: none; border-radius: 10px; cursor: pointer; font-weight: bold; transition: all 0.3s; }
        .nav-btn.active { background: #667eea; color: white; }
        .water-level { background: linear-gradient(135deg, #3b82f6 0%, #1e3a8a 100%); border-radius: 15px; padding: 20px; text-align: center; margin-bottom: 20px; color: white; }
        .level-value { font-size: 56px; font-weight: bold; }
        .level-bar-container { background: rgba(255,255,255,0.2); border-radius: 10px; margin-top: 15px; height: 30px; overflow: hidden; }
        .level-bar { background: #10B981; height: 100%; transition: width 0.5s; display: flex; align-items: center; justify-content: center; color: white; font-size: 12px; font-weight: bold; }
        .stats-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; margin-bottom: 20px; }
        .stat { background: #f5f5f5; padding: 15px; border-radius: 12px; text-align: center; }
        .stat-label { font-size: 11px; color: #666; margin-bottom: 5px; text-transform: uppercase; }
        .stat-value { font-size: 22px; font-weight: bold; color: #333; }
        .pf-card { background: #f5f5f5; padding: 15px; border-radius: 12px; text-align: center; margin-bottom: 20px; }
        .pf-value { font-size: 28px; font-weight: bold; }
        .pf-good { color: #10B981; }
        .pf-warning { color: #F59E0B; }
        .pf-bad { color: #EF4444; }
        .pf-idle { color: #9CA3AF; }
        .mode-container { display: flex; align-items: center; justify-content: space-between; background: #f8f9fa; padding: 20px; border-radius: 15px; margin-bottom: 20px; flex-wrap: wrap; gap: 15px; }
        .mode-info { flex: 1; }
        .mode-label { font-size: 14px; color: #666; margin-bottom: 5px; }
        .mode-status { font-size: 18px; font-weight: bold; }
        .mode-status.auto { color: #10B981; }
        .mode-status.manual { color: #F59E0B; }
        .switch-container { display: flex; align-items: center; gap: 15px; }
        .switch-label { font-size: 14px; font-weight: bold; color: #666; }
        .switch { position: relative; display: inline-block; width: 70px; height: 34px; }
        .switch input { opacity: 0; width: 0; height: 0; }
        .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #EF4444; transition: 0.4s; border-radius: 34px; }
        .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: 0.4s; border-radius: 50%; }
        input:checked + .slider { background-color: #10B981; }
        input:checked + .slider:before { transform: translateX(36px); }
        .pump-btn { width: 140px; height: 140px; border-radius: 70px; border: none; font-size: 24px; font-weight: bold; cursor: pointer; transition: all 0.3s; margin: 10px auto; display: block; }
        .pump-btn.running { background: #10B981; color: white; animation: pulse 2s infinite; }
        .pump-btn.stopped { background: #EF4444; color: white; }
        .pump-btn.disabled { background: #9CA3AF; cursor: not-allowed; opacity: 0.6; }
        .pump-btn.cooldown { background: #6b7280; cursor: not-allowed; }
        .pump-btn.inrush { background: #8b5cf6; animation: pulse 1s infinite; }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(16,185,129,0.7); } 70% { box-shadow: 0 0 0 15px rgba(16,185,129,0); } 100% { box-shadow: 0 0 0 0 rgba(16,185,129,0); } }
        .config-section { margin-top: 20px; padding: 15px; background: #f8f9fa; border-radius: 12px; }
        .config-group { margin-bottom: 15px; }
        label { display: block; font-weight: bold; margin-bottom: 5px; color: #333; }
        input, select { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 8px; font-size: 14px; }
        button { background: #667eea; color: white; border: none; padding: 12px 24px; border-radius: 10px; cursor: pointer; font-weight: bold; margin-top: 10px; margin-right: 10px; }
        button:hover { background: #5a67d8; }
        button.danger { background: #EF4444; }
        button.warning { background: #F59E0B; }
        .info-text { font-size: 11px; color: #666; margin-top: 5px; }
        .status-led { display: inline-block; width: 8px; height: 8px; border-radius: 4px; margin-right: 5px; }
        .status-online { background: #10B981; animation: pulse 2s infinite; }
        .status-offline { background: #EF4444; }
        .status-warning { background: #F59E0B; }
        .mac-address { font-family: monospace; font-size: 14px; background: #f0f0f0; padding: 8px; border-radius: 5px; text-align: center; }
        .terminal-log { background: #1e1e1e; color: #d4d4d4; border-radius: 8px; padding: 15px; font-family: 'Courier New', monospace; font-size: 12px; height: 400px; overflow-y: auto; margin-top: 15px; }
        .terminal-entry { padding: 8px; border-bottom: 1px solid #333; font-family: monospace; font-size: 11px; }
        .terminal-time { color: #6a9955; }
        .terminal-mac { color: #ce9178; }
        .terminal-data { color: #9cdcfe; }
        .terminal-parsed { color: #c586c0; }
        .terminal-valid { border-left: 3px solid #4ec9b0; background: #1e3a2e; }
        .terminal-invalid { border-left: 3px solid #f48771; background: #3a1e1a; }
        .espnow-stats { display: flex; gap: 20px; margin-bottom: 15px; padding: 10px; background: #f0f0f0; border-radius: 8px; flex-wrap: wrap; }
        .espnow-stat { font-size: 12px; }
        .espnow-stat span { font-weight: bold; color: #667eea; }
        .manual-request-btn { background: #F59E0B; font-size: 16px; padding: 12px 24px; margin: 10px 0; }
        .sensor-health { display: inline-block; padding: 5px 10px; border-radius: 20px; font-size: 12px; font-weight: bold; }
        .sensor-health.healthy { background: #10B981; color: white; }
        .sensor-health.warning { background: #F59E0B; color: white; }
        .sensor-health.dead { background: #EF4444; color: white; animation: pulse 1s infinite; }
        .strategy-selector { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 10px; }
        .strategy-card { background: white; border: 2px solid #e5e7eb; border-radius: 10px; padding: 10px; cursor: pointer; transition: all 0.3s; }
        .strategy-card.selected { border-color: #667eea; background: #EEF2FF; }
        .strategy-title { font-weight: bold; margin-bottom: 5px; }
        .strategy-desc { font-size: 11px; color: #666; }
        .simple-link { text-align: center; margin-top: 20px; padding: 10px; background: #e0e7ff; border-radius: 10px; }
        .simple-link a { color: #4338ca; text-decoration: none; font-weight: bold; }
        .overload-badge { background: #fef3c7; color: #92400e; padding: 4px 8px; border-radius: 20px; font-size: 12px; margin-left: 10px; }
        .inrush-badge-active { background: #8b5cf6; color: white; padding: 4px 8px; border-radius: 20px; font-size: 12px; margin-left: 10px; animation: pulse 1s infinite; }
        .dryrun-badge-active { background: #fed7aa; color: #9a3412; padding: 4px 8px; border-radius: 20px; font-size: 12px; margin-left: 10px; }
        @media (max-width: 600px) { .stats-grid { grid-template-columns: 1fr; } .nav-buttons { flex-direction: column; } .strategy-selector { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>🔧 Smart Pump Controller - Engineering Mode</h1>
            <div class="version">v4.5.3 | ✓ Physical Button | ✓ INRUSH FIXED | ✓ DRY RUN TOGGLE</div>
            <div class="eng-badge">⚙️ ENGINEERING MODE - Overload+Inrush protection in Auto & Manual | Physical Button: Short=toggle | Long=Force Manual</div>
            
            <div class="nav-buttons">
                <button class="nav-btn active" onclick="showSection('dashboard')">Dashboard</button>
                <button class="nav-btn" onclick="showSection('espnow')">ESP-NOW</button>
                <button class="nav-btn" onclick="showSection('pump')">Pump Settings</button>
                <button class="nav-btn" onclick="showSection('safety')">⚡ Safety</button>
                <button class="nav-btn" onclick="showSection('stats')">Statistics</button>
                <button class="nav-btn" onclick="showSection('system')">System</button>
                <button class="nav-btn" onclick="showSection('button')">🔘 Button</button>
            </div>
            
            <div id="dashboardSection">
                <div class="config-section" style="margin-bottom:15px">
                    <div>📡 Sensor: <span id="sensorHealth" class="sensor-health healthy">Checking...</span>
                    <span id="sensorLastSeen" style="font-size:11px;margin-left:10px"></span></div>
                    <div>🛡️ Overload Protection: <span id="overloadStatus" style="font-weight:bold">Active</span> <span id="overloadBadge" class="overload-badge" style="display:none">⚠️ TRIGGERED!</span></div>
                    <div>⚡ Inrush Period: <span id="inrushStatus" class="inrush-badge-active" style="display:none">ACTIVE</span><span id="inrushStatusText">None</span></div>
                    <div>⏱️ Cooldown: <span id="cooldownStatus">None</span></div>
                    <div>💧 Dry Run: <span id="dryRunStatusText">Idle</span> <span id="dryRunBadge" class="dryrun-badge-active" style="display:none">ACTIVE</span></div>
                    <div>🎮 Control Mode: <span id="controlModeDisplay">Auto</span></div>
                    <div>🔧 Dry Run Enabled: <span id="dryRunEnabledStatus">Yes</span></div>
                    <div>🔘 Button Status: <span id="buttonStatus">Enabled</span> | Presses: <span id="totalPresses">0</span></div>
                </div>
                <div class="water-level">
                    <div>Current Water Level</div>
                    <div class="level-value"><span id="waterLevel">0</span>%</div>
                    <div class="level-bar-container"><div class="level-bar" id="levelBar" style="width:0%">0%</div></div>
                    <div style="font-size:12px;margin-top:5px">Distance: <span id="distance">0</span> cm | Volume: <span id="volume">0</span> L | Battery: <span id="battery">0.00</span>V</div>
                </div>
                <div class="stats-grid">
                    <div class="stat"><div class="stat-label">Power</div><div class="stat-value"><span id="power">0</span> W</div></div>
                    <div class="stat"><div class="stat-label">Voltage</div><div class="stat-value"><span id="voltage">0</span> V</div></div>
                    <div class="stat"><div class="stat-label">Current</div><div class="stat-value"><span id="current">0</span> A</div></div>
                    <div class="stat"><div class="stat-label">Energy</div><div class="stat-value"><span id="energy">0</span> kWh</div></div>
                </div>
                <div class="pf-card"><div class="stat-label">Power Factor</div><div class="pf-value"><span id="powerFactor">0.00</span></div><div id="pfQuality" style="font-size:11px;margin-top:5px"></div></div>
                
                <div class="mode-container">
                    <div class="mode-info"><div class="mode-label">Control Mode</div><div class="mode-status" id="modeStatus">🤖 Auto Mode</div></div>
                    <div class="switch-container">
                        <span class="switch-label">Manual</span>
                        <label class="switch"><input type="checkbox" id="modeToggle" onchange="toggleMode()"><div class="slider"></div></label>
                        <span class="switch-label">Auto</span>
                    </div>
                </div>
                
                <button id="pumpBtn" class="pump-btn stopped" onclick="togglePump()"><span id="pumpText">PUMP OFF</span></button>
                <div id="pumpReason" style="text-align:center;font-size:12px;margin-top:10px"></div>
                
                <div class="config-section">
                    <div><span class="status-led" id="espnowLed"></span> ESP-NOW: <span id="espnowStatus">Disabled</span></div>
                    <div>Data Source: <span id="dataSource">-</span></div>
                    <div>Active Strategy: <span id="activeStrategy">-</span></div>
                    <div>AP IP: 192.168.4.1</div>
                    <div id="lastUpdate" style="margin-top:10px;font-size:11px"></div>
                </div>
            </div>
            
            <div id="espnowSection" style="display:none">
                <h3>ESP-NOW Configuration</h3>
                <div class="config-group"><label>Enable ESP-NOW</label><select id="useEspNow"><option value="true">Enabled</option><option value="false">Disabled</option></select></div>
                <div class="config-group"><label>Sensor MAC Address</label><input type="text" id="peerMac" placeholder="AA:BB:CC:DD:EE:FF" maxlength="17"><div class="info-text">Leave as FF:FF:FF:FF:FF:FF for broadcast</div></div>
                <div class="config-group"><label>WiFi Channel (1-13)</label><input type="number" id="espnowChannel" min="1" max="13" placeholder="1"></div>
                <button onclick="saveEspNow()">Save Settings</button>
                <h3 style="margin-top:20px">📡 Manual Data Request</h3>
                <button id="manualRequestBtn" class="manual-request-btn" onclick="manualGetMeasure()">🔄 MANUAL: Send 'get_measure'</button>
                <div id="sendStatus" style="display:inline-block;margin-left:10px"></div>
                <h3 style="margin-top:20px">📡 ESP-NOW Terminal Log</h3>
                <div class="espnow-stats"><div class="espnow-stat">📨 Messages: <span id="espnowMsgCount">0</span></div><div class="espnow-stat">⏱️ Last: <span id="espnowLastTime">Never</span></div><div class="espnow-stat">📡 Status: <span id="espnowStatusText">Inactive</span></div><div class="espnow-stat">🔢 Last Seq: <span id="lastSeq">-</span></div></div>
                <div style="margin-bottom:10px"><button onclick="refreshEspNowLog()" style="background:#10B981">🔄 Refresh</button><button onclick="clearEspNowLog()" class="danger">🗑️ Clear Log</button><button onclick="exportEspNowLog()" style="background:#F59E0B">📥 Export Log</button></div>
                <div id="espnowTerminal" class="terminal-log">Waiting for ESP-NOW messages...</div>
                <div class="info-text">💡 Device sends "get_measure" every 15 seconds<br>📝 Expected: {"d":351.9,"l":0.0,"v":0.0,"b":3.80}</div>
                <h3 style="margin-top:20px">🖥️ Device Info</h3>
                <div class="mac-address" id="deviceMac"></div>
            </div>
            
            <div id="pumpSection" style="display:none">
                <h3>Pump Control Settings</h3>
                <div class="config-group"><label>Low Threshold (Pump ON below %)</label><input type="number" id="lowThreshold" step="5" placeholder="30"></div>
                <div class="config-group"><label>High Threshold (Pump OFF above %)</label><input type="number" id="highThreshold" step="5" placeholder="80"></div>
                <hr>
                <div class="config-group">
                    <label class="checkbox-label"><input type="checkbox" id="dryRunEnabled"> ✅ Enable Dry Run Protection</label>
                    <div class="info-text">★ Stops pump if low power detected for configured time (Auto mode only) ★</div>
                </div>
                <div class="config-group"><label>💧 Dry Run Protection (seconds)</label><input type="number" id="dryRunProtection" step="1" min="5" placeholder="20"><div class="info-text" style="color:#10B981; font-weight:bold;">★ Triggers after EXACTLY the configured seconds ★</div></div>
                <div class="config-group"><label>⚡ Minimum Power Threshold (Watts)</label><input type="number" id="minPower" step="1" min="1" placeholder="5"><div class="info-text">Power below this value indicates dry run condition</div></div>
                <hr>
                <div style="background:#D1FAE5; padding:10px; border-radius:10px; margin-bottom:15px;">
                    <strong>🛡️ OVERLOAD+INRUSH PROTECTION (v4.5.3 FIXED)</strong><br>
                    <span style="font-size:11px">✓ Works in BOTH Auto AND Manual modes</span><br>
                    <span style="font-size:11px">✓ Inrush period properly detected in all scenarios</span>
                </div>
                <div class="config-group"><label class="checkbox-label"><input type="checkbox" id="loadProtectionToggle"> ✅ Enable Overload Protection</label><div class="info-text">★ Stops pump for SUSTAINED overload AFTER inrush period</div></div>
                <div class="config-group"><label>⚡ Max Power Limit (Watts)</label><input type="number" id="maxPower" step="10" placeholder="500"><div class="info-text">Pump stops if power exceeds this value AFTER startup inrush</div></div>
                <div class="config-group"><label>⚡ Inrush Tolerance (milliseconds)</label><input type="number" id="inrushTolerance" step="100" min="100" max="10000" placeholder="2000"><div class="info-text">Time to ignore high current at startup (2-7x normal)</div></div>
                <div class="config-group"><label>⏱️ Cooldown Period (seconds)</label><input type="number" id="cooldownPeriod" step="5" placeholder="30"><div class="info-text">Time to wait before allowing pump restart after overload</div></div>
                <button onclick="savePumpSettings()">Save Settings</button>
            </div>
            
            <div id="safetySection" style="display:none">
                <h3>🔒 Sensor Failure Protection</h3>
                <div class="info-text" style="margin-bottom:15px; background:#10B98120; padding:10px; border-radius:8px;">✅ Strategy 0 (Stop Pump) works in BOTH Auto AND Manual modes for maximum safety!</div>
                <div class="config-group"><label>Sensor Timeout (seconds)</label><input type="number" id="sensorTimeout" step="10" placeholder="120"></div>
                <div class="config-group"><label>📋 Failure Strategy</label>
                    <div class="strategy-selector">
                        <div class="strategy-card" data-strategy="0" onclick="selectStrategy(0)"><div class="strategy-title">🛑 Stop Pump</div><div class="strategy-desc">Immediately stop pump for maximum safety (Auto & Manual)</div></div>
                        <div class="strategy-card" data-strategy="1" onclick="selectStrategy(1)"><div class="strategy-title">⚖️ Maintain State</div><div class="strategy-desc">Keep pump in last known safe state</div></div>
                        <div class="strategy-card" data-strategy="2" onclick="selectStrategy(2)"><div class="strategy-title">⚠️ Force ON</div><div class="strategy-desc">Force pump ON (RISKY - potential overflow!)</div></div>
                        <div class="strategy-card" data-strategy="3" onclick="selectStrategy(3)"><div class="strategy-title">🔄 Cyclic Mode</div><div class="strategy-desc">Run pump in cycles with 1-hour emergency timeout</div></div>
                    </div>
                </div>
                <div id="cyclicSettings" style="display:none"><div class="config-group"><label>ON Duration (seconds)</label><input type="number" id="cyclicOnTime" step="60" placeholder="300"></div><div class="config-group"><label>OFF Duration (seconds)</label><input type="number" id="cyclicOffTime" step="60" placeholder="1800"></div><div class="info-text">⚠️ Emergency stop after 1 hour if sensor remains dead</div></div>
                <button onclick="saveSafetySettings()">Save Safety Settings</button>
                <h3 style="margin-top:20px">📊 Failure Log</h3>
                <div id="failureLog" class="terminal-log" style="height:200px"></div>
                <button onclick="clearFailureLog()" class="warning">Clear Log</button>
            </div>
            
            <div id="statsSection" style="display:none">
                <h3>Pump Statistics</h3>
                <div class="stats-grid">
                    <div class="stat"><div class="stat-label">Total Runtime</div><div class="stat-value" id="totalRuntime">0 min</div></div>
                    <div class="stat"><div class="stat-label">Total Energy</div><div class="stat-value" id="totalEnergy">0 kWh</div></div>
                    <div class="stat"><div class="stat-label">Pump Cycles</div><div class="stat-value" id="pumpCycles">0</div></div>
                    <div class="stat"><div class="stat-label">Last Start</div><div class="stat-value" id="lastStart">Never</div></div>
                    <div class="stat"><div class="stat-label">Overload Events</div><div class="stat-value" id="overloadEvents">0</div></div>
                    <div class="stat"><div class="stat-label">Button Short Presses</div><div class="stat-value" id="shortPressCount">0</div></div>
                    <div class="stat"><div class="stat-label">Button Long Presses</div><div class="stat-value" id="longPressCount">0</div></div>
                </div>
                <button onclick="resetStats()">Reset Statistics</button>
                <button onclick="exportStats()">Export CSV</button>
            </div>
            
            <div id="systemSection" style="display:none">
                <h3>System</h3>
                <div class="config-group"><label>Device Name</label><input type="text" id="hostname" placeholder="Device Name"></div>
                <button onclick="saveSystem()">Save & Restart</button>
                <button class="danger" onclick="factoryReset()">Factory Reset</button>
                <button class="warning" onclick="rebootNow()">Reboot</button>
                <div style="margin-top:20px;padding:15px;background:#f0f0f0;border-radius:10px">
                    <strong>📡 Connection Info:</strong><br>SSID: <span id="apSsid"></span><br>Password: 12345678<br>IP: 192.168.4.1
                </div>
            </div>
            
            <div id="buttonSection" style="display:none">
                <h3>🔘 Physical Button Configuration (GPIO0)</h3>
                <div class="config-group">
                    <label class="checkbox-label"><input type="checkbox" id="btnEnabled"> ✅ Enable Physical Button</label>
                    <div class="info-text">Built-in button on Sonoff S31 (GPIO0)</div>
                </div>
                <div class="config-group">
                    <label class="checkbox-label"><input type="checkbox" id="btnAutoCancel"> 🔄 Auto-cancel on Short Press</label>
                    <div class="info-text">When in Auto mode, short press temporarily switches to Manual mode</div>
                </div>
                <div class="config-group"><label>⏱️ Auto-cancel Timeout (minutes)</label><input type="number" id="btnAutoCancelTimeout" step="1" min="1" max="60" placeholder="5"><div class="info-text">After this time, automatically returns to Auto mode</div></div>
                <hr>
                <h4>Button Statistics</h4>
                <div class="stats-grid">
                    <div class="stat"><div class="stat-label">Short Presses</div><div class="stat-value" id="statShortPress">0</div></div>
                    <div class="stat"><div class="stat-label">Long Presses</div><div class="stat-value" id="statLongPress">0</div></div>
                </div>
                <button onclick="resetButtonStats()">Reset Button Statistics</button>
                <div class="info-text" style="margin-top:15px; background:#e0e7ff; padding:10px; border-radius:8px;">
                    <strong>📖 Button Operation Guide:</strong><br>
                    • <strong>Short Press (&lt;3s):</strong> Toggle pump manually<br>
                    &nbsp;&nbsp;- In AUTO mode: Temporarily switches to MANUAL for 5min<br>
                    &nbsp;&nbsp;- In MANUAL mode: Directly toggles pump ON/OFF<br>
                    • <strong>Long Press (3-10s):</strong> Force MANUAL mode permanently<br>
                    • <strong>Safety:</strong> Won't start pump during overload cooldown<br>
                    • <strong>Feedback:</strong> LED flashes for button confirmation
                </div>
            </div>
            
            <div class="simple-link"><a href="/">← Back to Simple User Mode</a></div>
        </div>
    </div>
    
    <script>
        let currentSection = 'dashboard', selectedStrategy = 0;
        function showSection(s) {
            currentSection = s;
            document.getElementById('dashboardSection').style.display = s==='dashboard'?'block':'none';
            document.getElementById('espnowSection').style.display = s==='espnow'?'block':'none';
            document.getElementById('pumpSection').style.display = s==='pump'?'block':'none';
            document.getElementById('safetySection').style.display = s==='safety'?'block':'none';
            document.getElementById('statsSection').style.display = s==='stats'?'block':'none';
            document.getElementById('systemSection').style.display = s==='system'?'block':'none';
            document.getElementById('buttonSection').style.display = s==='button'?'block':'none';
            document.querySelectorAll('.nav-btn').forEach(btn=>btn.classList.remove('active'));
            event.target.classList.add('active');
            if(s==='espnow'){loadEspNowSettings();refreshEspNowLog();}
            if(s==='pump')loadPumpSettings();
            if(s==='safety'){loadSafetySettings();loadFailureLog();}
            if(s==='system')loadSystemSettings();
            if(s==='stats')loadStats();
            if(s==='button')loadButtonSettings();
        }
        async function toggleMode(){const mode=document.getElementById('modeToggle').checked?'auto':'manual';await fetch('/mode?mode='+mode);fetchData();updatePumpButtonState();}
        function updateModeUI(autoMode){const toggle=document.getElementById('modeToggle');const modeStatus=document.getElementById('modeStatus');const pumpBtn=document.getElementById('pumpBtn');const controlModeDisplay=document.getElementById('controlModeDisplay');if(autoMode){toggle.checked=true;modeStatus.innerHTML='🤖 Auto Mode';modeStatus.className='mode-status auto';controlModeDisplay.innerHTML='Auto';pumpBtn.classList.add('disabled');}else{toggle.checked=false;modeStatus.innerHTML='👆 Manual Mode';modeStatus.className='mode-status manual';controlModeDisplay.innerHTML='Manual';pumpBtn.classList.remove('disabled');}}
        function updatePumpButtonState(){const toggle=document.getElementById('modeToggle');const pumpBtn=document.getElementById('pumpBtn');if(toggle&&!toggle.checked)pumpBtn.classList.remove('disabled');else if(toggle&&toggle.checked)pumpBtn.classList.add('disabled');}
        function selectStrategy(s){selectedStrategy=s;document.querySelectorAll('.strategy-card').forEach(c=>c.classList.remove('selected'));document.querySelector(`.strategy-card[data-strategy="${s}"]`).classList.add('selected');document.getElementById('cyclicSettings').style.display=s===3?'block':'none';}
        async function manualGetMeasure(){const btn=document.getElementById('manualRequestBtn');const ot=btn.innerHTML;btn.innerHTML='⏳ Sending...';btn.disabled=true;document.getElementById('sendStatus').innerHTML='⏳ Sending...';try{const r=await fetch('/espnow/request');const t=await r.text();document.getElementById('sendStatus').innerHTML='✅ '+t;setTimeout(()=>document.getElementById('sendStatus').innerHTML='',3000);refreshEspNowLog();}catch(e){document.getElementById('sendStatus').innerHTML='❌ Failed';}finally{btn.innerHTML=ot;btn.disabled=false;}}
        function getPFClass(pf,running){if(!running)return'pf-idle';if(pf>=0.95)return'pf-good';if(pf>=0.8)return'pf-warning';return'pf-bad';}
        function getPFQuality(pf,running){if(!running)return'⏸ Idle (PF=0)';if(pf>=0.95)return'✓ Excellent';if(pf>=0.9)return'✓ Good';if(pf>=0.8)return'⚠ Fair';return'⚠ Poor';}
        async function loadSafetySettings(){const cfg=await fetchConfig();document.getElementById('sensorTimeout').value=cfg.sensor_timeout||120;selectedStrategy=cfg.failure_strategy||0;selectStrategy(selectedStrategy);document.getElementById('cyclicOnTime').value=cfg.cyclic_on_duration||300;document.getElementById('cyclicOffTime').value=cfg.cyclic_off_duration||1800;}
        async function loadEspNowSettings(){const cfg=await fetchConfig();document.getElementById('useEspNow').value=cfg.use_espnow?'true':'false';document.getElementById('peerMac').value=cfg.peer_mac||'FF:FF:FF:FF:FF:FF';document.getElementById('espnowChannel').value=cfg.espnow_channel||1;const r=await fetch('/info');const i=await r.json();document.getElementById('deviceMac').innerHTML=i.mac||'Unknown';document.getElementById('apSsid').innerHTML=i.ap_ssid||'Unknown';}
        async function loadPumpSettings(){const cfg=await fetchConfig();document.getElementById('lowThreshold').value=cfg.low_threshold||30;document.getElementById('highThreshold').value=cfg.high_threshold||80;document.getElementById('dryRunEnabled').checked=cfg.dry_run_enabled!==false;document.getElementById('dryRunProtection').value=cfg.dry_run_protection||20;document.getElementById('minPower').value=cfg.min_power||5;document.getElementById('loadProtectionToggle').checked=cfg.pump_load_protection_enabled!==false;document.getElementById('maxPower').value=cfg.max_power_threshold||500;document.getElementById('inrushTolerance').value=cfg.inrush_tolerance_ms||2000;document.getElementById('cooldownPeriod').value=cfg.overload_cooldown_seconds||30;}
        async function loadSystemSettings(){const cfg=await fetchConfig();document.getElementById('hostname').value=cfg.hostname||'s31-pump';}
        async function loadButtonSettings(){const cfg=await fetchConfig();document.getElementById('btnEnabled').checked=cfg.button_enabled!==false;document.getElementById('btnAutoCancel').checked=cfg.button_auto_cancel!==false;document.getElementById('btnAutoCancelTimeout').value=(cfg.button_auto_cancel_timeout||300000)/60000;const stats=await fetchJSON('/stats');if(stats){document.getElementById('statShortPress').innerText=stats.button_short_press||0;document.getElementById('statLongPress').innerText=stats.button_long_press||0;}}
        async function fetchConfig(){const r=await fetch('/config');return await r.json();}
        async function refreshEspNowLog(){const r=await fetch('/espnow/log');const h=await r.text();document.getElementById('espnowTerminal').innerHTML=h;}
        async function clearEspNowLog(){await fetch('/espnow/clear',{method:'POST'});refreshEspNowLog();}
        async function exportEspNowLog(){window.location.href='/espnow/export';}
        async function loadFailureLog(){const r=await fetch('/failurelog');const h=await r.text();document.getElementById('failureLog').innerHTML=h;}
        async function clearFailureLog(){await fetch('/failurelog/clear',{method:'POST'});loadFailureLog();}
        async function resetButtonStats(){await fetch('/button/resetstats');if(currentSection==='button')loadButtonSettings();fetchData();}
        async function fetchData(){try{const r=await fetch('/data');const d=await r.json();document.getElementById('waterLevel').innerText=d.waterLevel.toFixed(1);document.getElementById('distance').innerText=d.distance.toFixed(1);document.getElementById('volume').innerText=d.volume.toFixed(0);document.getElementById('battery').innerText=d.batteryVoltage.toFixed(2);document.getElementById('levelBar').style.width=d.waterLevel+'%';document.getElementById('levelBar').innerHTML=d.waterLevel.toFixed(0)+'%';document.getElementById('power').innerText=d.power.toFixed(1);document.getElementById('voltage').innerText=d.voltage.toFixed(1);document.getElementById('current').innerText=d.current.toFixed(2);document.getElementById('energy').innerText=d.energy.toFixed(2);const isRunning=d.pumpState;const pf=isRunning?d.powerFactor:0;document.getElementById('powerFactor').innerText=pf.toFixed(3);document.getElementById('powerFactor').className=getPFClass(pf,isRunning);document.getElementById('pfQuality').innerHTML=getPFQuality(pf,isRunning);const pumpBtn=document.getElementById('pumpBtn');const pumpText=document.getElementById('pumpText');if(d.inCooldown){pumpBtn.className='pump-btn cooldown';pumpText.innerText='COOLDOWN';pumpBtn.disabled=true;}else if(d.inInrushPeriod){pumpBtn.className='pump-btn inrush';pumpText.innerText='STARTING';if(!d.autoMode)pumpBtn.disabled=false;}else if(isRunning){pumpBtn.className='pump-btn running';pumpText.innerText='PUMP ON';if(!d.autoMode)pumpBtn.disabled=false;}else{pumpBtn.className='pump-btn stopped';pumpText.innerText='PUMP OFF';if(!d.autoMode)pumpBtn.disabled=false;}document.getElementById('pumpReason').innerText=d.pumpReason;updateModeUI(d.autoMode);const sH=document.getElementById('sensorHealth');if(d.sensorHealthy){sH.className='sensor-health healthy';sH.innerHTML='✅ Sensor Healthy';}else if(d.sensorWarning){sH.className='sensor-health warning';sH.innerHTML='⚠️ Sensor Warning';}else{sH.className='sensor-health dead';sH.innerHTML='❌ Sensor Dead';}document.getElementById('sensorLastSeen').innerHTML=d.sensorLastSeen||'';document.getElementById('espnowStatus').innerHTML=d.espnowActive?'Active':'Disabled';document.getElementById('espnowLed').className='status-led '+(d.espnowActive?(d.espnowDataValid?'status-online':'status-warning'):'status-offline');document.getElementById('dataSource').innerHTML=d.dataSource||'No Data';document.getElementById('activeStrategy').innerHTML=d.activeStrategy||'-';document.getElementById('lastUpdate').innerHTML='Updated: '+new Date().toLocaleTimeString();document.getElementById('overloadStatus').innerHTML=d.overloadProtectionActive?'⚠️ TRIGGERED!':'Active';document.getElementById('overloadBadge').style.display=d.overloadProtectionActive?'inline-block':'none';if(d.inInrushPeriod){document.getElementById('inrushStatus').style.display='inline-block';document.getElementById('inrushStatusText').style.display='none';document.getElementById('inrushStatus').innerHTML='ACTIVE ('+d.inrushRemaining+'ms)';}else{document.getElementById('inrushStatus').style.display='none';document.getElementById('inrushStatusText').style.display='inline';document.getElementById('inrushStatusText').innerHTML='None';}document.getElementById('cooldownStatus').innerHTML=d.inCooldown?d.cooldownRemaining+'s remaining':'None';if(d.dryRunActive){document.getElementById('dryRunStatusText').innerHTML=d.dryRunTime+'s';document.getElementById('dryRunBadge').style.display='inline-block';}else{document.getElementById('dryRunStatusText').innerHTML='Idle';document.getElementById('dryRunBadge').style.display='none';}document.getElementById('dryRunEnabledStatus').innerHTML=d.dryRunEnabled?'Yes (Active)':'No (Disabled)';document.getElementById('buttonStatus').innerHTML=d.buttonEnabled?'Enabled':'Disabled';document.getElementById('totalPresses').innerHTML=(d.buttonShortPressCount||0)+(d.buttonLongPressCount||0);if(currentSection==='espnow'){document.getElementById('espnowMsgCount').innerText=d.espnowMsgCount||0;document.getElementById('espnowLastTime').innerText=d.espnowLastTime||'Never';document.getElementById('espnowStatusText').innerHTML=d.espnowActive?(d.espnowDataValid?'✅ Receiving':'⚠️ No Data'):'❌ Disabled';document.getElementById('lastSeq').innerText=d.lastSeq||'-';}}catch(e){console.error(e);}}
        async function loadStats(){try{const r=await fetch('/stats');const s=await r.json();document.getElementById('totalRuntime').innerText=s.total_runtime+' min';document.getElementById('totalEnergy').innerText=s.total_energy+' kWh';document.getElementById('pumpCycles').innerText=s.pump_cycles;document.getElementById('lastStart').innerText=s.last_start||'Never';document.getElementById('overloadEvents').innerText=s.overload_stop_count||0;document.getElementById('shortPressCount').innerText=s.button_short_press||0;document.getElementById('longPressCount').innerText=s.button_long_press||0;}catch(e){}}
        async function saveEspNow(){const s={use_espnow:document.getElementById('useEspNow').value==='true',peer_mac:document.getElementById('peerMac').value,espnow_channel:parseInt(document.getElementById('espnowChannel').value)};await fetch('/config/espnow',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Saved! Rebooting...');setTimeout(()=>location.reload(),3000);}
        async function saveSafetySettings(){const s={sensor_timeout:parseInt(document.getElementById('sensorTimeout').value),failure_strategy:selectedStrategy,cyclic_on_duration:parseInt(document.getElementById('cyclicOnTime').value)||300,cyclic_off_duration:parseInt(document.getElementById('cyclicOffTime').value)||1800};await fetch('/config/safety',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Safety settings saved!');}
        async function savePumpSettings(){const s={low_threshold:parseFloat(document.getElementById('lowThreshold').value),high_threshold:parseFloat(document.getElementById('highThreshold').value),dry_run_enabled:document.getElementById('dryRunEnabled').checked,dry_run_protection:parseInt(document.getElementById('dryRunProtection').value),min_power:parseFloat(document.getElementById('minPower').value),pump_load_protection_enabled:document.getElementById('loadProtectionToggle').checked,max_power_threshold:parseFloat(document.getElementById('maxPower').value),inrush_tolerance_ms:parseInt(document.getElementById('inrushTolerance').value),overload_cooldown_seconds:parseInt(document.getElementById('cooldownPeriod').value)};await fetch('/config/pump',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Pump settings saved!');}
        async function saveSystem(){const s={hostname:document.getElementById('hostname').value};await fetch('/config/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Saved! Rebooting...');setTimeout(()=>location.reload(),3000);}
        async function saveButtonConfig(){const s={button_enabled:document.getElementById('btnEnabled').checked,button_auto_cancel:document.getElementById('btnAutoCancel').checked,button_auto_cancel_timeout:parseInt(document.getElementById('btnAutoCancelTimeout').value)*60000};await fetch('/config/button',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Button settings saved!');}
        async function togglePump(){const cfg=await fetchConfig();if(cfg.auto_mode){alert('Switch to Manual Mode first');return;}const d=await fetchJSON('/data');if(d&&d.inCooldown){alert('Pump in cooldown! Please wait.');return;}await fetch('/toggle');fetchData();}
        async function factoryReset(){if(confirm('FACTORY RESET - Erase ALL settings?')){await fetch('/factoryreset');}}
        async function rebootNow(){if(confirm('Reboot device?')){await fetch('/reboot');alert('Rebooting...');}}
        async function resetStats(){if(confirm('Reset statistics?')){await fetch('/resetstats');loadStats();}}
        async function exportStats(){window.location.href='/exportstats';}
        async function fetchJSON(url){try{const r=await fetch(url);return await r.json();}catch(e){return null;}}
        setInterval(fetchData,1000);setInterval(loadStats,10000);if(currentSection==='safety')setInterval(loadFailureLog,5000);fetchData();loadStats();
        
        // Add button config save to existing buttons
        const saveBtn = document.querySelector('#buttonSection button');
        if(saveBtn) saveBtn.onclick = saveButtonConfig;
    </script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     PHYSICAL BUTTON HANDLING FUNCTIONS (GPIO0)
// ================================================================================================

/**
 * @brief   Initialize GPIO pins for button and LED
 * @note    GPIO0 has internal pull-up, button connects to GND when pressed
 * @note    LED on GPIO13 is active low (LOW = ON, HIGH = OFF)
 */
void initButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // GPIO0 with internal pull-up
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);        // Start with LED off
  
  button.state = BUTTON_IDLE;
  button.lastButtonState = HIGH;
  button.currentButtonState = HIGH;
  
  DEBUG_BUTTON("Button initialized on GPIO" + String(BUTTON_PIN));
  addFailureLogEntry("🔘 Physical button ready - Short press=toggle | Long press=Force Manual");
}

/**
 * @brief   Flash LED for visual feedback
 * @param   duration_ms - How long to keep LED on (milliseconds)
 * @note    LED is active low on Sonoff S31 (GPIO13)
 */
void flashLed(unsigned long duration_ms) {
  digitalWrite(LED_PIN, LOW);          // Turn LED on
  delay(min(duration_ms, 100UL));      // Brief flash
  digitalWrite(LED_PIN, HIGH);         // Turn LED off
}

/**
 * @brief   Get current effective operation mode (considering button auto-cancel)
 * @return  true if in Auto mode (or Auto but temporarily cancelled), false if Manual
 * @note    Auto-cancel temporarily overrides config.auto_mode for a set timeout
 */
bool getEffectiveAutoMode() {
  // If button cancelled Auto mode and timeout hasn't expired
  if (button.autoModeCancelled && config.button_auto_cancel) {
    unsigned long cancelDuration = millis() - button.autoModeCancelTime;
    if (cancelDuration < config.button_auto_cancel_timeout) {
      return false;  // Temporarily in Manual mode
    } else {
      // Auto-cancel timeout expired - revert to Auto
      button.autoModeCancelled = false;
      addFailureLogEntry("🔄 Auto-cancel timeout expired - Returning to AUTO mode");
      DEBUG_BUTTON("Auto-cancel timeout expired, returning to AUTO mode");
    }
  }
  
  return config.auto_mode;
}

/**
 * @brief   Handle short press action - Toggle pump with safety checks
 * @note    Short press: < 3 seconds
 * @note    In Auto mode: Temporarily cancels Auto and switches to Manual
 * @note    In Manual mode: Directly toggles pump (with cooldown check)
 */
void handleShortPress() {
  DEBUG_BUTTON("🔘 Short press detected (" + String(millis() - button.pressStartTime) + "ms)");
  button.shortPressCount++;
  flashLed(50);  // Quick flash feedback
  
  // Check if we're in cooldown period (safety first)
  if (millis() < overload.cooldownUntil) {
    addFailureLogEntry("⏱️ Button: Pump in cooldown - " + 
                       String((overload.cooldownUntil - millis())/1000) + "s remaining");
    flashLed(1000);  // Long flash for error
    return;
  }
  
  // Get current effective mode (considering Auto-cancel)
  bool effectiveAutoMode = getEffectiveAutoMode();
  
  if (effectiveAutoMode) {
    // In Auto mode: Pressing cancels Auto temporarily and switches to Manual
    if (config.button_auto_cancel) {
      if (!button.autoModeCancelled) {
        button.autoModeCancelled = true;
        button.autoModeCancelTime = millis();
        addFailureLogEntry("🔘 Button: AUTO mode cancelled - Temporary MANUAL for " + 
                           String(config.button_auto_cancel_timeout/60000) + " minutes");
        DEBUG_BUTTON("Auto mode temporarily cancelled, switching to Manual");
        
        // Flash pattern for auto-cancel (2 quick flashes)
        flashLed(50);
        delay(100);
        flashLed(50);
      }
    } else {
      addFailureLogEntry("🔘 Button: Auto-cancel disabled - Use web interface to change mode");
      return;
    }
  } else {
    // Already in Manual mode - toggle pump directly
    // Additional safety: Check overload protection before allowing manual start
    bool newState = !s31.getRelayState();
    
    if (newState && overload.active) {
      addFailureLogEntry("🛡️ Button: Cannot start pump - Overload protection active");
      flashLed(500);
      return;
    }
    
    // Check dry run protection if enabled and trying to start
    if (newState && config.dry_run_enabled && s31.getPower() < config.min_power_threshold) {
      addFailureLogEntry("💧 Button: Warning - Low power condition may trigger dry run");
    }
    
    s31.setRelay(newState);
    addFailureLogEntry("🔘 Button: Pump " + String(newState ? "ON" : "OFF") + 
                       " (Mode: Manual" + (button.autoModeCancelled ? " - Temp" : "") + ")");
    
    DEBUG_BUTTON("Pump toggled to: " + String(newState ? "ON" : "OFF"));
  }
}

/**
 * @brief   Handle long press action - Force Manual mode permanently
 * @note    Long press: 3-10 seconds
 * @note    Forces permanent Manual mode, cancels any temporary auto-cancel
 */
void handleLongPress() {
  DEBUG_BUTTON("🔘 Long press detected (" + String(millis() - button.pressStartTime) + "ms)");
  button.longPressCount++;
  flashLed(200);  // Longer flash for long press
  
  // Force switch to Manual mode
  if (config.auto_mode) {
    config.auto_mode = false;
    queueConfigSave();  // Save to EEPROM
    button.autoModeCancelled = false;  // Clear auto-cancel flag since we're explicitly in Manual
    addFailureLogEntry("🔘 Long press: Switched to MANUAL mode permanently");
    DEBUG_BUTTON("Long press: Permanently switched to MANUAL mode");
    
    // Flash pattern: 3 flashes for mode change
    for (int i = 0; i < 3; i++) {
      flashLed(50);
      delay(100);
    }
  } else {
    // If already in Manual, toggle pump (with safety checks)
    if (millis() < overload.cooldownUntil) {
      addFailureLogEntry("⏱️ Button: Pump in cooldown - Cannot start");
      flashLed(1000);
      return;
    }
    
    bool newState = !s31.getRelayState();
    
    if (newState && overload.active) {
      addFailureLogEntry("🛡️ Button: Cannot start pump - Overload protection active");
      flashLed(500);
      return;
    }
    
    s31.setRelay(newState);
    addFailureLogEntry("🔘 Long press (Manual mode): Pump " + String(newState ? "ON" : "OFF"));
    DEBUG_BUTTON("Long press in Manual mode: Toggled pump");
  }
}

/**
 * @brief   Main button handler - called from loop() with debouncing and state machine
 * @note    Implements proper debouncing to prevent false triggers
 * @note    Provides haptic feedback via LED for button actions
 */
void handleButton() {
  if (!config.button_enabled) return;
  
  unsigned long now = millis();
  bool reading = digitalRead(BUTTON_PIN);
  
  // Debounce logic - wait for stable state
  if (reading != button.lastButtonState) {
    button.lastDebounceTime = now;
  }
  
  if ((now - button.lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
    if (reading != button.currentButtonState) {
      button.currentButtonState = reading;
      
      if (button.currentButtonState == LOW) {
        // Button pressed (LOW because of pull-up and button to GND)
        button.pressStartTime = now;
        button.buttonPressed = true;
        button.state = BUTTON_PRESSED;
        DEBUG_BUTTON("Button pressed");
      } else {
        // Button released
        if (button.buttonPressed) {
          unsigned long pressDuration = now - button.pressStartTime;
          
          if (pressDuration < SHORT_PRESS_MAX_MS) {
            // Short press (< 3 seconds)
            handleShortPress();
          } else if (pressDuration >= LONG_PRESS_MIN_MS && pressDuration <= LONG_PRESS_MAX_MS) {
            // Long press (3-10 seconds)
            handleLongPress();
          } else if (pressDuration > LONG_PRESS_MAX_MS) {
            DEBUG_BUTTON("Press too long (>10s), ignored");
            flashLed(500);
          }
          
          button.buttonPressed = false;
          button.state = BUTTON_IDLE;
        }
      }
    }
  }
  
  // Handle continuous press feedback (for long press indication)
  if (button.buttonPressed && (now - button.pressStartTime) > 2000) {
    if ((now - button.lastFeedbackTime) > BUTTON_HOLD_FEEDBACK_MS) {
      button.lastFeedbackTime = now;
      flashLed(20);  // Quick flash as holding feedback
    }
  }
  
  button.lastButtonState = reading;
}

// ================================================================================================
// @section     HELPER FUNCTIONS (Core System)
// ================================================================================================

/**
 * @brief   Adds timestamped entry to failure log with circular buffer management
 * @param   message - Description of the failure/protection event
 */
void addFailureLogEntry(String message) {
  String timestamp = String(millis() / 1000);
  failureLog.insert(failureLog.begin(), "[" + timestamp + "s] " + message);
  while (failureLog.size() > MAX_LOG_ENTRIES) failureLog.pop_back();
  
  #if ENABLE_DEBUG_LOGGING
    Serial.println(message);
  #endif
}

/**
 * @brief   Generates HTML representation of failure log (reverse chronological)
 * @return  String containing HTML divs for each log entry
 */
String getFailureLogHTML() {
  if (failureLog.empty()) return "<div class='terminal-entry'>No failure events logged</div>";
  String html = "";
  for (size_t i = 0; i < failureLog.size() && i < 50; i++)
    html += "<div class='terminal-entry'>" + failureLog[i] + "</div>";
  return html;
}

/**
 * @brief   Converts 6-byte MAC address to human-readable string
 * @param   mac - Pointer to 6-byte MAC array
 * @return  Formatted MAC string (e.g., "AA:BB:CC:DD:EE:FF")
 */
String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", 
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

/**
 * @brief   Converts MAC string to byte array with validation
 * @param   macStr - Input MAC string (format: "AA:BB:CC:DD:EE:FF")
 * @param   mac    - Output 6-byte array
 * @return  true if parsing successful and valid, false otherwise
 */
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

/**
 * @brief   Safe float comparison with epsilon tolerance
 * @param   a, b - Float values to compare
 * @param   epsilon - Tolerance threshold (default: FLOAT_EPSILON)
 * @return  true if |a-b| < epsilon
 */
bool floatEqual(float a, float b, float epsilon = FLOAT_EPSILON) {
  return fabs(a - b) < epsilon;
}

// ================================================================================================
// @section     EEPROM CONFIGURATION MANAGEMENT (With Sanitization)
// ================================================================================================

/**
 * @brief   Validates configuration integrity and sane value ranges
 * @return  true if magic number matches AND thresholds are logically valid
 */
bool isConfigValid() {
  if (config.magic != 0xDEADBEEF) return false;
  if (config.low_threshold < 0 || config.low_threshold > 100) return false;
  if (config.high_threshold < 0 || config.high_threshold > 100) return false;
  if (config.low_threshold >= config.high_threshold - FLOAT_EPSILON) return false;
  return true;
}

/**
 * @brief   Saves current configuration to EEPROM (debounced via queueConfigSave)
 * @note    Actual EEPROM write happens in processPendingConfigSave() after debounce
 */
void saveConfig() { 
  EEPROM.begin(512);
  config.magic = 0xDEADBEEF;
  EEPROM.put(0, config);
  EEPROM.commit();
  EEPROM.end();
  #if ENABLE_DEBUG_LOGGING
    Serial.println("[EEPROM] Configuration saved");
  #endif
}

/**
 * @brief   Loads configuration from EEPROM, applies defaults and sanitization if invalid
 * @note    All numeric values constrained to safe ranges using constrain()
 */
void loadConfig() { 
  EEPROM.begin(512);
  EEPROM.get(0, config);
  EEPROM.end();
  
  if (!isConfigValid()) {
    Config defaultConfig;
    config = defaultConfig;
    saveConfig();
    #if ENABLE_DEBUG_LOGGING
      Serial.println("[EEPROM] Invalid config detected - restored defaults");
    #endif
  }
  
  // Sanitize ALL configuration values with constrain()
  config.low_threshold = constrain(config.low_threshold, 0.0, 99.0);
  config.high_threshold = constrain(config.high_threshold, 1.0, 100.0);
  if (config.low_threshold >= config.high_threshold - FLOAT_EPSILON) {
    config.low_threshold = 30.0;
    config.high_threshold = 80.0;
  }
  
  config.min_power_threshold = constrain(config.min_power_threshold, 1.0, 100.0);
  config.pump_protection_time = constrain(config.pump_protection_time, 5, 300);
  config.max_power_threshold = constrain(config.max_power_threshold, 10.0, 2000.0);
  config.overload_cooldown_seconds = constrain(config.overload_cooldown_seconds, 0, 300);
  config.inrush_tolerance_ms = constrain(config.inrush_tolerance_ms, 100, 10000);
  config.espnow_channel = constrain(config.espnow_channel, 1, 13);
  config.sensor_timeout = constrain(config.sensor_timeout, 10000, 600000);
  config.cyclic_on_duration = constrain(config.cyclic_on_duration, 1000, 600000);
  config.cyclic_off_duration = constrain(config.cyclic_off_duration, 1000, 7200000);
  config.sensor_failure_strategy = constrain(config.sensor_failure_strategy, 0, 3);
  config.button_auto_cancel_timeout = constrain(config.button_auto_cancel_timeout, 60000, 3600000);
}

/**
 * @brief   Performs factory reset by erasing EEPROM and restoring defaults
 * @note    Triggers device reboot after reset completion
 */
void factoryReset() {
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++) EEPROM.write(i, 0xFF);
  EEPROM.commit();
  EEPROM.end();
  
  Config defaultConfig;
  config = defaultConfig;
  saveConfig();
  
  #if ENABLE_DEBUG_LOGGING
    Serial.println("[FACTORY] Reset complete - rebooting");
  #endif
  delay(1000);
  ESP.restart();
}

/**
 * @brief   Reboots the device after short delay for graceful shutdown
 */
void rebootDevice() { 
  #if ENABLE_DEBUG_LOGGING
    Serial.println("[SYSTEM] Reboot requested");
  #endif
  delay(500); 
  ESP.restart(); 
}

// ================================================================================================
// @section     POWER QUALITY CALCULATIONS (With Safety Checks)
// ================================================================================================

/**
 * @brief   Updates power quality variables based on current S31 readings
 * @note    Handles edge cases: relay OFF, very low apparent power, negative PF
 */
void updatePowerQuality() {
  float voltage = s31.getVoltage();
  float current = s31.getCurrent();
  float realPower = s31.getPower();
  bool relayState = s31.getRelayState();
  
  apparentPower = voltage * current;
  
  if (!relayState || realPower < 0.5) {
    powerFactor = 0.0;
    apparentPower = 0.0;
    reactivePower = 0.0;
  } else {
    if (apparentPower > 0.01) {
      powerFactor = realPower / apparentPower;
      powerFactor = constrain(powerFactor, 0.0, 1.0);
    } else {
      powerFactor = 0.0;
    }
    
    float vaSquared = apparentPower * apparentPower;
    float wSquared = realPower * realPower;
    reactivePower = (vaSquared > wSquared) ? sqrt(vaSquared - wSquared) : 0.0;
  }
}

// ================================================================================================
// @section     SENSOR HEALTH MONITORING (With Warning Hysteresis)
// ================================================================================================

/**
 * @brief   Updates sensor health status based on time since last valid ESP-NOW data
 * @note    Implements hysteresis to prevent rapid healthy↔warning↔dead oscillation
 */
void updateSensorHealth() {
  unsigned long now = millis();
  unsigned long timeSinceLastData = (lastEspNowData > 0) ? (now - lastEspNowData) : config.sensor_timeout + 10000;
  
  if (timeSinceLastData < ESP_NOW_DATA_TIMEOUT) {
    if (sensorIsDead) { 
      sensorIsDead = false; 
      sensorWarningIssued = false; 
      addFailureLogEntry("✅ Sensor RECOVERED - Data received"); 
    }
  } else if (timeSinceLastData < config.sensor_timeout) {
    if (!sensorWarningIssued) { 
      sensorWarningIssued = true; 
      addFailureLogEntry("⚠️ Sensor WARNING: No data for " + String(timeSinceLastData/1000) + "s"); 
    }
    sensorIsDead = false;
  } else {
    if (!sensorIsDead) { 
      sensorIsDead = true; 
      sensorDeadStartTime = now; 
      addFailureLogEntry("❌ SENSOR DEAD: No data for " + String(timeSinceLastData/1000) + "s"); 
    }
  }
}

/**
 * @brief   Determines pump state based on configured sensor failure strategy
 * @param   currentState - Current pump relay state (true=ON, false=OFF)
 * @return  Desired pump state per safety strategy
 */
bool handleSensorFailurePumpControl(bool currentState) {
  if (!sensorIsDead) return currentState;
  
  unsigned long now = millis();
  unsigned long timeSinceDead = now - sensorDeadStartTime;
  
  switch (config.sensor_failure_strategy) {
    case 0:
      if (currentState) addFailureLogEntry("🛑 Emergency STOP - Sensor failure (Strategy 0)");
      return false;
    case 1:
      return currentState;
    case 2:
      if (timeSinceDead > config.sensor_timeout) { 
        addFailureLogEntry("⚠️ FORCE ON - Risk of overflow! Sensor dead > timeout"); 
        return true; 
      }
      return currentState;
    case 3: {
      if (timeSinceDead > 3600000) {
        addFailureLogEntry("🚨 CYCLIC MODE: Emergency stop - Sensor dead for >1 hour");
        return false;
      }
      
      if (cyclicLastSwitchTime == 0) { 
        cyclicLastSwitchTime = now; 
        cyclicPumpState = false; 
      }
      
      unsigned long cycleDuration = cyclicPumpState ? config.cyclic_on_duration : config.cyclic_off_duration;
      if (now - cyclicLastSwitchTime >= cycleDuration) {
        cyclicPumpState = !cyclicPumpState;
        cyclicLastSwitchTime = now;
        addFailureLogEntry("🔄 Cyclic [" + String(timeSinceDead/1000) + "s dead]: Pump " + 
                         String(cyclicPumpState ? "ON" : "OFF"));
      }
      return cyclicPumpState;
    }
    default: 
      return false;
  }
}

// ================================================================================================
// @section     INTELLIGENT OVERLOAD PROTECTION (v4.5.3 FIXED)
// ================================================================================================

/**
 * @brief   Detects pump start events via edge detection with boot safety
 * @param   currentState - Current pump relay state
 * @return  true if a start event was detected, false otherwise
 */
bool detectPumpStartEvent(bool currentState) {
  if (!overload.bootInitialized) return false;
  
  bool startDetected = false;
  
  if (currentState && !overload.lastRelayState) {
    startDetected = true;
    DEBUG_OVERLOAD("Pump START detected (edge transition)");
  }
  
  if (currentState && !overload.inrushActive && overload.pumpStartTime == 0) {
    if (millis() < 5000) {
      startDetected = true;
      DEBUG_OVERLOAD("Pump START detected (boot supplemental)");
    }
  }
  
  return startDetected;
}

/**
 * @brief   Updates inrush protection state based on pump start/stop transitions
 * @param   currentState - Current pump relay state
 */
void updateInrushState(bool currentState) {
  unsigned long now = millis();
  
  if (detectPumpStartEvent(currentState)) {
    overload.pumpStartTime = now;
    overload.inrushActive = true;
    overload.manualStartPending = !config.auto_mode;
    
    DEBUG_OVERLOAD("Inrush protection ACTIVE for " + String(config.inrush_tolerance_ms) + "ms");
    addFailureLogEntry("🚀 Pump STARTED - Inrush tolerance active (" + 
                       String(config.inrush_tolerance_ms) + "ms) - Mode: " + 
                       String(config.auto_mode ? "AUTO" : "MANUAL"));
  }
  
  if (overload.inrushActive && currentState) {
    unsigned long runtime = now - overload.pumpStartTime;
    if (runtime >= config.inrush_tolerance_ms) {
      overload.inrushActive = false;
      DEBUG_OVERLOAD("Inrush period ended - Normal monitoring active");
      addFailureLogEntry("✅ Inrush period ended - Normal overload monitoring active");
    }
  }
  
  if (!currentState && overload.inrushActive) {
    overload.inrushActive = false;
    DEBUG_OVERLOAD("Inrush protection cancelled - Pump stopped");
  }
  
  overload.lastRelayState = currentState;
}

/**
 * @brief   Handles cooldown period logic with user feedback
 * @param   currentState - Current pump relay state
 * @return  true if in cooldown (pump must remain OFF), false otherwise
 */
bool handleCooldownPeriod(bool currentState) {
  unsigned long now = millis();
  
  if (overload.cooldownUntil > now) {
    unsigned long remaining = (overload.cooldownUntil - now) / 1000;
    
    if (currentState) {
      s31.setRelay(false);
      DEBUG_OVERLOAD("Cooldown active - forced pump OFF, " + String(remaining) + "s remaining");
      
      if (remaining > 0 && (now - overload.lastCooldownLogTime) > 30000) {
        overload.lastCooldownLogTime = now;
        addFailureLogEntry("⏱️ Cooldown active: " + String(remaining) + "s remaining");
      }
    }
    return true;
  }
  
  if (overload.active && now >= overload.cooldownUntil) {
    overload.active = false;
    addFailureLogEntry("✅ Cooldown period ended - Normal operation resumed");
  }
  
  return false;
}

/**
 * @brief   Checks for sustained overload condition (post-inrush)
 * @param   currentPower - Current power reading in Watts
 * @param   currentState - Current pump state
 * @return  true if overload detected and handled, false otherwise
 */
bool checkSustainedOverload(float currentPower, bool currentState) {
  unsigned long now = millis();
  
  if (!config.pump_load_protection_enabled || !currentState) return false;
  
  if (overload.inrushActive) {
    unsigned long runtime = now - overload.pumpStartTime;
    if (currentPower > config.max_power_threshold * 3) {
      DEBUG_OVERLOAD("Extreme inrush: " + String(currentPower, 1) + 
                    "W at " + String(runtime) + "ms - Possible fault");
    }
    return false;
  }
  
  if (currentPower > config.max_power_threshold && !overload.active) {
    unsigned long runtime = now - overload.pumpStartTime;
    
    if (runtime > 500) {
      overload.active = true;
      overload.cooldownUntil = now + (config.overload_cooldown_seconds * 1000);
      overload.overloadCount++;
      overload.lastOverloadPower = currentPower;
      
      addFailureLogEntry("╔════════════════════════════════════════════════════════════════╗");
      addFailureLogEntry("║  ⚠️⚠️⚠️  SUSTAINED OVERLOAD DETECTED!  ⚠️⚠️⚠️  ║");
      addFailureLogEntry("╠════════════════════════════════════════════════════════════════╣");
      addFailureLogEntry("║  Power: " + String(currentPower, 1) + "W > Limit: " + 
                         String(config.max_power_threshold, 0) + "W");
      addFailureLogEntry("║  Runtime before overload: " + String(runtime / 1000) + " seconds");
      addFailureLogEntry("║  Mode: " + String(config.auto_mode ? "AUTO" : "MANUAL"));
      addFailureLogEntry("║  Cooldown: " + String(config.overload_cooldown_seconds) + " seconds");
      addFailureLogEntry("║  Overload Event #" + String(overload.overloadCount));
      addFailureLogEntry("║  ACTION: IMMEDIATE POWER CUT - Pump STOPPED for safety");
      addFailureLogEntry("╚════════════════════════════════════════════════════════════════╝");
      
      s31.setRelay(false);
      return true;
    }
  }
  
  return false;
}

/**
 * @brief   Main overload protection handler - integrates all protection features
 * @param   currentPower - Current pump power consumption in Watts
 * @param   currentState - Current pump relay state
 * @return  true if pump is allowed to run, false if protection triggered
 */
bool checkOverloadProtection(float currentPower, bool currentState) {
  updateInrushState(currentState);
  if (handleCooldownPeriod(currentState)) return false;
  if (checkSustainedOverload(currentPower, currentState)) return false;
  return true;
}

// ================================================================================================
// @section     DRY RUN PROTECTION (Configurable & Coordinated)
// ================================================================================================

/**
 * @brief   Checks dry run condition with inrush coordination
 * @param   currentPower - Current power reading
 * @param   currentState - Current pump state
 * @return  true if dry run protection should stop the pump
 */
bool checkDryRunProtection(float currentPower, bool currentState) {
  unsigned long now = millis();
  
  if (!config.dry_run_enabled) {
    if (dryRun.lowPowerStartTime != 0) {
      DEBUG_DRYRUN("Dry run protection is DISABLED - timer reset");
      dryRun.lowPowerStartTime = 0;
      dryRun.protectionTriggered = false;
    }
    return false;
  }
  
  if (!currentState || sensorIsDead) {
    if (dryRun.lowPowerStartTime != 0) {
      DEBUG_DRYRUN("Dry run timer reset - Pump stopped or sensor dead");
      dryRun.lowPowerStartTime = 0;
      dryRun.protectionTriggered = false;
    }
    return false;
  }
  
  if (overload.inrushActive) return false;
  
  if (currentPower < config.min_power_threshold) {
    if (dryRun.lowPowerStartTime == 0) {
      dryRun.lowPowerStartTime = now;
      dryRun.protectionTriggered = false;
      addFailureLogEntry("⚠️ Low power detected (" + String(currentPower,1) + 
                       "W < " + String(config.min_power_threshold,0) + 
                       "W) - Dry run timer started");
    }
    
    unsigned long lowPowerDuration = (now - dryRun.lowPowerStartTime) / 1000;
    
    if (lowPowerDuration > 0 && (now - dryRun.lastLogTime) > 5000) {
      dryRun.lastLogTime = now;
      addFailureLogEntry("💧 Dry run in progress: " + String(lowPowerDuration) + 
                       "s / " + String(config.pump_protection_time) + "s");
    }
    
    if (lowPowerDuration >= config.pump_protection_time && !dryRun.protectionTriggered) {
      dryRun.protectionTriggered = true;
      addFailureLogEntry("💧 DRY RUN PROTECTION TRIGGERED! Pump stopped after " + 
                       String(lowPowerDuration) + " seconds of low power");
      return true;
    }
  } 
  else {
    if (dryRun.lowPowerStartTime != 0) {
      addFailureLogEntry("✅ Power restored (" + String(currentPower,1) + 
                       "W) - Dry run timer reset");
      dryRun.lowPowerStartTime = 0;
      dryRun.protectionTriggered = false;
    }
  }
  
  return false;
}

// ================================================================================================
// @section     PUMP CONTROL LOGIC (Complete Protection Integration)
// ================================================================================================

/**
 * @brief   Updates pump runtime statistics with overflow-safe calculations
 * @param   currentState - Current pump relay state
 */
void updatePumpStatistics(bool currentState) {
  unsigned long now = millis();
  
  if (currentState && !pumpStats.wasRunning) {
    pumpStats.pumpCycles++;
    pumpStats.lastPumpOnTime = now;
    pumpStats.wasRunning = true;
    pumpStats.totalEnergyKwh = s31.getEnergy();
    
    unsigned long hours = pumpStats.totalRuntimeSeconds / 3600;
    unsigned long minutes = (pumpStats.totalRuntimeSeconds % 3600) / 60;
    snprintf(pumpStats.lastStartStr, sizeof(pumpStats.lastStartStr), "%02lu:%02lu", hours, minutes);
    
    DEBUG_LOG("Pump cycle #" + String(pumpStats.pumpCycles) + " started");
  } 
  else if (!currentState && pumpStats.wasRunning) {
    unsigned long runDuration = (now - pumpStats.lastPumpOnTime) / 1000;
    pumpStats.totalRuntimeSeconds += runDuration;
    pumpStats.wasRunning = false;
    DEBUG_LOG("Pump stopped after " + String(runDuration) + " seconds runtime");
  }
}

/**
 * @brief   Determines desired pump state in auto mode based on water level hysteresis
 * @param   currentState - Current pump state
 * @return  Desired pump state (true=ON, false=OFF)
 */
bool getAutoModePumpState(bool currentState) {
  bool dataValid = (espnow_initialized && espnowDataValid && 
                    millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
  
  if (!dataValid || sensorIsDead) {
    return handleSensorFailurePumpControl(currentState);
  }
  
  if (currentWaterLevel <= config.low_threshold) return true;
  else if (currentWaterLevel >= config.high_threshold) return false;
  
  return currentState;
}

/**
 * @brief   Main pump control routine - integrates all protection features
 * @note    Protection priority: Overload > Dry Run > Water Level Control
 */
void controlPump() {
  updateSensorHealth();
  updatePowerQuality();
  
  if (millis() - lastControlCheck < CONTROL_INTERVAL) return;
  lastControlCheck = millis();
  
  bool currentState = s31.getRelayState();
  float currentPower = s31.getPower();
  
  if (!checkOverloadProtection(currentPower, currentState)) {
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  if (checkDryRunProtection(currentPower, currentState)) {
    s31.setRelay(false);
    updatePumpStatistics(s31.getRelayState());
    return;
  }
  
  bool shouldPumpOn = currentState;
  bool effectiveAutoMode = getEffectiveAutoMode();
  
  if (effectiveAutoMode) {
    shouldPumpOn = getAutoModePumpState(currentState);
  }
  
  if (shouldPumpOn != currentState) {
    s31.setRelay(shouldPumpOn);
    
    if (shouldPumpOn) {
      addFailureLogEntry("Pump ON - " + String(effectiveAutoMode ? "Auto" : "Manual") + 
                       (button.autoModeCancelled ? " (Temp Manual)" : "") + 
                       " mode, Level: " + String(currentWaterLevel,1) + "%");
    } else {
      addFailureLogEntry("Pump OFF - " + String(effectiveAutoMode ? "Auto" : "Manual") + 
                       (button.autoModeCancelled ? " (Temp Manual)" : "") + 
                       " mode, Level: " + String(currentWaterLevel,1) + "%");
    }
  }
  
  updatePumpStatistics(s31.getRelayState());
}

// ================================================================================================
// @section     ESP-NOW COMMUNICATION (With Retry Logic)
// ================================================================================================

/**
 * @brief   Adds formatted entry to ESP-NOW message log
 */
void addEspNowLogEntry(String mac, String rawData, float d, float l, float v, float b) {
  EspNowLogEntry entry;
  entry.timestamp = millis();
  entry.mac = mac;
  entry.rawData = rawData;
  entry.distance = d;
  entry.level = l;
  entry.volume = v;
  entry.battery = b;
  entry.valid = (d > 0 || l > 0 || v > 0 || b > 0);
  
  espnow_log.insert(espnow_log.begin(), entry);
  while (espnow_log.size() > MAX_LOG_ENTRIES) espnow_log.pop_back();
  espnow_msg_counter++;
}

/**
 * @brief   Generates HTML table of ESP-NOW message log (reverse chronological)
 */
String getEspNowLogHTML() {
  if (espnow_log.empty()) return "<div class='terminal-entry'>No ESP-NOW messages received yet...</div>";
  
  String html = "";
  for (size_t i = 0; i < espnow_log.size() && i < 50; i++) {
    EspNowLogEntry& e = espnow_log[i];
    unsigned long age = (millis() - e.timestamp) / 1000;
    
    html += "<div class='terminal-entry " + String(e.valid ? "terminal-valid" : "terminal-invalid") + "'>";
    html += "<div><span class='terminal-time'>[" + String(age) + "s ago]</span> ";
    html += "<span class='terminal-mac'>📡 MAC: " + e.mac + "</span></div>";
    html += "<div><span class='terminal-data'>📨 Raw: " + e.rawData + "</span></div>";
    
    if (e.valid) {
      html += "<div><span class='terminal-parsed'>📊 Parsed: D=" + String(e.distance, 1) + "cm, ";
      html += "L=" + String(e.level, 1) + "%, ";
      html += "V=" + String(e.volume, 1) + "L, ";
      html += "B=" + String(e.battery, 2) + "V</span></div>";
    } else {
      html += "<div><span class='terminal-parsed'>⚠️ Failed to parse message</span></div>";
    }
    html += "</div>";
  }
  return html;
}

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {}
void OnDataRecv(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len != sizeof(EspNowPacket)) { 
    addEspNowLogEntry(macToString(mac), "Invalid packet size: " + String(len), 0,0,0,0); 
    return; 
  }
  
  memcpy(&incoming, data, sizeof(incoming));
  String senderMac = macToString(mac);
  String rawResponse = String(incoming.msg);
  
  float d=0, l=0, v=0, b=0;
  bool parseSuccess = false;
  
  int dIndex = rawResponse.indexOf("\"d\":");
  if (dIndex != -1) { 
    int start = dIndex+4; 
    int end = rawResponse.indexOf(",", start); 
    if(end==-1) end=rawResponse.indexOf("}", start); 
    if(end!=-1){ d=rawResponse.substring(start,end).toFloat(); parseSuccess=true; } 
  }
  
  int lIndex = rawResponse.indexOf("\"l\":");
  if (lIndex != -1) { 
    int start = lIndex+4; 
    int end = rawResponse.indexOf(",", start); 
    if(end==-1) end=rawResponse.indexOf("}", start); 
    if(end!=-1){ l=rawResponse.substring(start,end).toFloat(); parseSuccess=true; } 
  }
  
  int vIndex = rawResponse.indexOf("\"v\":");
  if (vIndex != -1) { 
    int start = vIndex+4; 
    int end = rawResponse.indexOf(",", start); 
    if(end==-1) end=rawResponse.indexOf("}", start); 
    if(end!=-1){ v=rawResponse.substring(start,end).toFloat(); parseSuccess=true; } 
  }
  
  int bIndex = rawResponse.indexOf("\"b\":");
  if (bIndex != -1) { 
    int start = bIndex+4; 
    int end = rawResponse.indexOf(",", start); 
    if(end==-1) end=rawResponse.indexOf("}", start); 
    if(end!=-1){ b=rawResponse.substring(start,end).toFloat(); parseSuccess=true; } 
  }
  
  if (parseSuccess) {
    currentDistance = d; 
    currentWaterLevel = l; 
    currentVolume = v; 
    batteryVoltage = b;
    lastEspNowData = millis(); 
    espnowDataValid = true;
    
    if (sensorIsDead) { 
      sensorIsDead = false; 
      sensorWarningIssued = false; 
      addFailureLogEntry("✅ Sensor RECOVERED - Data received"); 
    }
  }
  
  addEspNowLogEntry(senderMac, rawResponse, d, l, v, b);
}

void initEspNow() {
  if (!config.use_espnow) { espnow_initialized = false; return; }
  
  WiFi.mode(WIFI_AP_STA);
  
  if (esp_now_init() != 0) { 
    espnow_initialized = false; 
    return; 
  }
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  uint8_t* peerMac = broadcastMac;
  if (config.peer_mac[0] != 0xFF && config.peer_mac[0] != 0x00) peerMac = config.peer_mac;
  
  if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_COMBO, config.espnow_channel, NULL, 0) != 0) { 
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
    outgoing.msg[sizeof(outgoing.msg)-1] = '\0';
    
    uint8_t* peerMac = broadcastMac;
    if (config.peer_mac[0] != 0xFF && config.peer_mac[0] != 0x00) peerMac = config.peer_mac;
    
    if (esp_now_send(peerMac, (uint8_t *)&outgoing, sizeof(outgoing)) == 0) return;
    
    delay(ESP_NOW_RETRY_DELAY_MS * (retry + 1));
  }
}

void requestSensorData() {
  if (!espnow_initialized) return;
  if (millis() - lastEspNowSend >= ESP_NOW_SEND_INTERVAL) {
    lastEspNowSend = millis();
    sendEspNowCommand("get_measure");
  }
}

void manualRequestSensorData() {
  if (!espnow_initialized) { 
    server.send(400, "text/plain", "ESP-NOW not initialized"); 
    return; 
  }
  sendEspNowCommand("get_measure");
  server.send(200, "text/plain", "Sent 'get_measure' request");
}

// ================================================================================================
// @section     WEB SERVER ENDPOINTS (Complete Implementation)
// ================================================================================================

void setupAPMode() {
  String apSSID = "SmartPump-" + String(ESP.getChipId() & 0xFFFF, HEX);
  WiFi.softAP(apSSID.c_str(), "12345678");
  Serial.println("AP Mode: " + apSSID + " | Password: 12345678");
}

void setupArduinoOTA() {
  if (!ENABLE_OTA) return;
  
  ArduinoOTA.setHostname(deviceName.c_str());
  
  ArduinoOTA.onStart([]() { Serial.println("OTA Update Starting..."); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA Update Complete"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (progress % (total/10) == 0) {
      Serial.printf("OTA Progress: %u%%\r\n", (progress * 100) / total);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

void setupWebServer() {
  // UI Pages
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", simple_html); });
  server.on("/engmode", HTTP_GET, []() { server.send_P(200, "text/html", engineering_html); });
  
  // Button status endpoint
  server.on("/button/status", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["enabled"] = config.button_enabled;
    doc["short_press_count"] = button.shortPressCount;
    doc["long_press_count"] = button.longPressCount;
    doc["auto_cancelled"] = button.autoModeCancelled;
    if (button.autoModeCancelled) {
      doc["auto_cancel_remaining"] = (config.button_auto_cancel_timeout - (millis() - button.autoModeCancelTime)) / 1000;
    }
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Button config endpoint
  server.on("/config/button", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<256> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("button_enabled")) config.button_enabled = doc["button_enabled"];
      if (doc.containsKey("button_auto_cancel")) config.button_auto_cancel = doc["button_auto_cancel"];
      if (doc.containsKey("button_auto_cancel_timeout")) {
        config.button_auto_cancel_timeout = constrain(doc["button_auto_cancel_timeout"].as<unsigned long>(), 60000, 3600000);
      }
      queueConfigSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  // Reset button stats
  server.on("/button/resetstats", HTTP_GET, []() {
    button.shortPressCount = 0;
    button.longPressCount = 0;
    server.send(200, "text/plain", "OK");
  });
  
  // Real-time data endpoint
  server.on("/data", HTTP_GET, []() {
    updatePowerQuality();
    updateSensorHealth();
    
    StaticJsonDocument<2048> doc;
    
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
    doc["autoMode"] = getEffectiveAutoMode();
    doc["autoModeOriginal"] = config.auto_mode;
    doc["autoCancelled"] = button.autoModeCancelled;
    doc["espnowActive"] = espnow_initialized;
    doc["espnowDataValid"] = espnowDataValid;
    doc["espnowMsgCount"] = espnow_msg_counter;
    doc["lastSeq"] = espnow_seq;
    doc["sensorHealthy"] = !sensorIsDead && (millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
    doc["sensorWarning"] = !sensorIsDead && (millis() - lastEspNowData >= ESP_NOW_DATA_TIMEOUT);
    doc["overloadProtectionActive"] = overload.active;
    doc["inInrushPeriod"] = overload.inrushActive;
    doc["buttonEnabled"] = config.button_enabled;
    doc["buttonShortPressCount"] = button.shortPressCount;
    doc["buttonLongPressCount"] = button.longPressCount;
    
    if (overload.inrushActive) {
      unsigned long elapsed = millis() - overload.pumpStartTime;
      if (config.inrush_tolerance_ms > elapsed) doc["inrushRemaining"] = config.inrush_tolerance_ms - elapsed;
      else { doc["inrushRemaining"] = 0; doc["inInrushPeriod"] = false; }
    } else doc["inrushRemaining"] = 0;
    
    doc["inCooldown"] = (millis() < overload.cooldownUntil);
    if (millis() < overload.cooldownUntil) doc["cooldownRemaining"] = (overload.cooldownUntil - millis()) / 1000;
    else doc["cooldownRemaining"] = 0;
    doc["overloadCount"] = overload.overloadCount;
    doc["dryRunEnabled"] = config.dry_run_enabled;
    doc["dryRunActive"] = (dryRun.lowPowerStartTime != 0);
    if (dryRun.lowPowerStartTime != 0) doc["dryRunTime"] = (millis() - dryRun.lowPowerStartTime) / 1000;
    else doc["dryRunTime"] = 0;
    
    if (lastEspNowData > 0) doc["sensorLastSeen"] = String((millis() - lastEspNowData)/1000) + "s ago";
    else doc["sensorLastSeen"] = "Never";
    
    if (espnow_initialized && espnowDataValid && millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT) 
      doc["dataSource"] = "ESP-NOW";
    else doc["dataSource"] = "No Data";
    
    String strategyName;
    switch(config.sensor_failure_strategy) {
      case 0: strategyName = "🛑 Stop Pump"; break;
      case 1: strategyName = "⚖️ Maintain"; break;
      case 2: strategyName = "⚠️ Force ON"; break;
      case 3: strategyName = "🔄 Cyclic"; break;
      default: strategyName = "Stop Pump";
    }
    doc["activeStrategy"] = strategyName;
    
    String reason = "";
    bool effectiveAuto = getEffectiveAutoMode();
    if (!effectiveAuto) reason = "Manual Control" + String(button.autoModeCancelled ? " (Temp - Auto cancelled)" : "");
    else if (overload.active) reason = "🛡️ OVERLOAD PROTECTION - Pump stopped";
    else if (millis() < overload.cooldownUntil) reason = "⏱️ Cooldown period - " + String((overload.cooldownUntil - millis())/1000) + "s remaining";
    else if (overload.inrushActive) reason = "⚡ Pump starting - Inrush tolerance active";
    else if (config.dry_run_enabled && dryRun.lowPowerStartTime != 0) {
      unsigned long dryRunTime = (millis() - dryRun.lowPowerStartTime) / 1000;
      reason = "💧 DRY RUN DETECTION - Low power for " + String(dryRunTime) + "s";
    }
    else if (s31.getRelayState()) reason = "Water level low (" + String(currentWaterLevel,1) + "%)";
    else if (currentWaterLevel >= config.high_threshold) reason = "Tank full";
    else reason = "Level OK";
    doc["pumpReason"] = reason;
    
    String response;
    if (serializeJson(doc, response) == 0) {
      server.send(500, "text/plain", "Internal error");
      return;
    }
    server.send(200, "application/json", response);
  });
  
  // Configuration endpoints
  server.on("/config", HTTP_GET, []() {
    StaticJsonDocument<1024> doc;
    doc["low_threshold"] = config.low_threshold;
    doc["high_threshold"] = config.high_threshold;
    doc["dry_run_enabled"] = config.dry_run_enabled;
    doc["min_power"] = config.min_power_threshold;
    doc["dry_run_protection"] = config.pump_protection_time;
    doc["use_espnow"] = config.use_espnow;
    doc["peer_mac"] = macToString(config.peer_mac);
    doc["espnow_channel"] = config.espnow_channel;
    doc["auto_mode"] = config.auto_mode;
    doc["hostname"] = deviceName;
    doc["sensor_timeout"] = config.sensor_timeout / 1000;
    doc["failure_strategy"] = config.sensor_failure_strategy;
    doc["cyclic_on_duration"] = config.cyclic_on_duration / 1000;
    doc["cyclic_off_duration"] = config.cyclic_off_duration / 1000;
    doc["pump_load_protection_enabled"] = config.pump_load_protection_enabled;
    doc["max_power_threshold"] = config.max_power_threshold;
    doc["inrush_tolerance_ms"] = config.inrush_tolerance_ms;
    doc["overload_cooldown_seconds"] = config.overload_cooldown_seconds;
    doc["button_enabled"] = config.button_enabled;
    doc["button_auto_cancel"] = config.button_auto_cancel;
    doc["button_auto_cancel_timeout"] = config.button_auto_cancel_timeout / 1000;
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/config/espnow", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("use_espnow")) config.use_espnow = doc["use_espnow"];
      if (doc.containsKey("peer_mac")) { 
        String mac = doc["peer_mac"].as<String>(); 
        if (stringToMac(mac, config.peer_mac)) {
          mac.toCharArray(config.peer_mac_str, 18);
        }
      }
      if (doc.containsKey("espnow_channel")) {
        config.espnow_channel = constrain(doc["espnow_channel"].as<int>(), 1, 13);
      }
      queueConfigSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/safety", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("sensor_timeout")) {
        config.sensor_timeout = constrain(doc["sensor_timeout"].as<unsigned long>() * 1000, 10000, 600000);
      }
      if (doc.containsKey("failure_strategy")) {
        config.sensor_failure_strategy = constrain(doc["failure_strategy"].as<uint8_t>(), 0, 3);
      }
      if (doc.containsKey("cyclic_on_duration")) {
        config.cyclic_on_duration = constrain(doc["cyclic_on_duration"].as<unsigned long>() * 1000, 1000, 600000);
      }
      if (doc.containsKey("cyclic_off_duration")) {
        config.cyclic_off_duration = constrain(doc["cyclic_off_duration"].as<unsigned long>() * 1000, 1000, 7200000);
      }
      queueConfigSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/pump", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("low_threshold")) config.low_threshold = constrain(doc["low_threshold"].as<float>(), 0.0, 99.0);
      if (doc.containsKey("high_threshold")) config.high_threshold = constrain(doc["high_threshold"].as<float>(), 1.0, 100.0);
      if (doc.containsKey("dry_run_enabled")) config.dry_run_enabled = doc["dry_run_enabled"];
      if (doc.containsKey("min_power")) config.min_power_threshold = constrain(doc["min_power"].as<float>(), 1.0, 100.0);
      if (doc.containsKey("dry_run_protection")) config.pump_protection_time = constrain(doc["dry_run_protection"].as<unsigned long>(), 5, 300);
      if (doc.containsKey("pump_load_protection_enabled")) config.pump_load_protection_enabled = doc["pump_load_protection_enabled"];
      if (doc.containsKey("max_power_threshold")) config.max_power_threshold = constrain(doc["max_power_threshold"].as<float>(), 10.0, 2000.0);
      if (doc.containsKey("inrush_tolerance_ms")) config.inrush_tolerance_ms = constrain(doc["inrush_tolerance_ms"].as<unsigned long>(), 100, 10000);
      if (doc.containsKey("overload_cooldown_seconds")) config.overload_cooldown_seconds = constrain(doc["overload_cooldown_seconds"].as<unsigned long>(), 0, 300);
      if (doc.containsKey("button_enabled")) config.button_enabled = doc["button_enabled"];
      if (doc.containsKey("button_auto_cancel")) config.button_auto_cancel = doc["button_auto_cancel"];
      if (doc.containsKey("button_auto_cancel_timeout")) config.button_auto_cancel_timeout = constrain(doc["button_auto_cancel_timeout"].as<unsigned long>(), 60000, 3600000);
      queueConfigSave();
      server.send(200, "text/plain", "OK");
    }
  });
  
  // Pump control endpoints
  server.on("/mode", HTTP_GET, []() { 
    if(server.hasArg("mode")){ 
      bool newMode = (server.arg("mode") == "auto");
      if (config.auto_mode != newMode) {
        config.auto_mode = newMode; 
        queueConfigSave();
        button.autoModeCancelled = false;
        if(!config.auto_mode) s31.setRelay(false);
        addFailureLogEntry("Mode changed to: " + String(config.auto_mode ? "AUTO" : "MANUAL"));
      }
    } 
    server.send(200,"text/plain","OK"); 
  });
  
  server.on("/toggle", HTTP_GET, []() { 
    bool effectiveAuto = getEffectiveAutoMode();
    if(!effectiveAuto) {
      if (millis() < overload.cooldownUntil) {
        server.send(403, "text/plain", "Pump in cooldown period");
        return;
      }
      bool newState = !s31.getRelayState();
      s31.setRelay(newState);
      addFailureLogEntry("Manual toggle: Pump " + String(newState ? "ON" : "OFF") + 
                        (button.autoModeCancelled ? " (Temp Manual)" : ""));
    }
    server.send(200,"text/plain","OK"); 
  });
  
  // Statistics endpoints
  server.on("/stats", HTTP_GET, []() { 
    StaticJsonDocument<512> doc; 
    doc["total_runtime"] = pumpStats.totalRuntimeSeconds/60; 
    doc["total_energy"] = pumpStats.totalEnergyKwh; 
    doc["pump_cycles"] = pumpStats.pumpCycles; 
    doc["last_start"] = pumpStats.lastStartStr; 
    doc["overload_stop_count"] = overload.overloadCount;
    doc["button_short_press"] = button.shortPressCount;
    doc["button_long_press"] = button.longPressCount;
    String response; 
    serializeJson(doc,response); 
    server.send(200,"application/json",response); 
  });
  
  server.on("/resetstats", HTTP_GET, []() { 
    pumpStats.totalRuntimeSeconds = 0; 
    pumpStats.totalEnergyKwh = 0; 
    pumpStats.pumpCycles = 0; 
    overload.overloadCount = 0;
    addFailureLogEntry("Statistics reset");
    server.send(200,"text/plain","OK"); 
  });
  
  // Log endpoints
  server.on("/failurelog", HTTP_GET, []() { 
    if (server.hasArg("format") && server.arg("format") == "html") {
      server.send(200, "text/html", getFailureLogHTML());
    } else {
      server.send(200, "application/json", "{\"logs\":[]}");
    }
  });
  
  server.on("/failurelog/clear", HTTP_POST, []() { failureLog.clear(); server.send(200, "text/plain", "OK"); });
  server.on("/espnow/log", HTTP_GET, []() { server.send(200, "text/html", getEspNowLogHTML()); });
  server.on("/espnow/clear", HTTP_POST, []() { espnow_log.clear(); espnow_msg_counter = 0; server.send(200, "text/plain", "OK"); });
  server.on("/espnow/export", HTTP_GET, []() {
    String csv = "Timestamp,Age(s),MAC,RawData,Distance_cm,Level_Percent,Volume_Liters,Battery_Voltage,Valid\n";
    for (size_t i = 0; i < espnow_log.size(); i++) {
      EspNowLogEntry& e = espnow_log[i];
      unsigned long age = (millis() - e.timestamp) / 1000;
      csv += String(e.timestamp) + "," + String(age) + "," + e.mac + ",\"" + e.rawData + "\",";
      csv += String(e.distance,1) + "," + String(e.level,1) + "," + String(e.volume,1) + "," + String(e.battery,2) + "," + String(e.valid?"Yes":"No") + "\n";
    }
    server.send(200, "text/csv", csv);
  });
  server.on("/espnow/request", HTTP_GET, []() { manualRequestSensorData(); });
  server.on("/exportstats", HTTP_GET, []() { 
    String csv = "Timestamp,Runtime(min),Energy(kWh),Cycles,LastStart,OverloadEvents,ButtonShortPress,ButtonLongPress\n"; 
    csv += String(millis()/1000) + "," + String(pumpStats.totalRuntimeSeconds/60) + "," + String(pumpStats.totalEnergyKwh) + "," + String(pumpStats.pumpCycles) + "," + String(pumpStats.lastStartStr) + "," + String(overload.overloadCount) + "," + String(button.shortPressCount) + "," + String(button.longPressCount) + "\n"; 
    server.send(200,"text/csv",csv); 
  });
  
  // System endpoints
  server.on("/factoryreset", HTTP_GET, []() { server.send(200,"text/plain","Factory resetting..."); delay(100); factoryReset(); });
  server.on("/reboot", HTTP_GET, []() { server.send(200,"text/plain","Rebooting..."); delay(100); rebootDevice(); });
  server.on("/info", HTTP_GET, []() { 
    String json = "{\"mac\":\"" + WiFi.macAddress() + "\",\"ap_ssid\":\"" + WiFi.softAPSSID() + "\",\"ip\":\"192.168.4.1\",\"version\":\"" FIRMWARE_VERSION "\"}"; 
    server.send(200,"application/json",json); 
  });
  
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
}

// ================================================================================================
// @section     ARDUINO SETUP & LOOP
// ================================================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  
  Serial.println("\n\n╔════════════════════════════════════════════════════════════════╗");
  Serial.println("║     SMART PUMP CONTROLLER v" FIRMWARE_VERSION " - BOOTING                  ║");
  Serial.println("╠════════════════════════════════════════════════════════════════╣");
  Serial.println("║  NEW: Physical button control (GPIO0)                         ║");
  Serial.println("║  • Short press: Toggle pump (Auto-cancel if in Auto)         ║");
  Serial.println("║  • Long press: Force Manual mode                              ║");
  Serial.println("║  • LED feedback for all button actions                        ║");
  Serial.println("╚════════════════════════════════════════════════════════════════╝");
  
  loadConfig();
  uint32_t chipId = ESP.getChipId();
  deviceName = "s31-pump-" + String(chipId & 0xFFFF, HEX);
  strncpy(config.hostname, deviceName.c_str(), sizeof(config.hostname)-1);
  
  s31.begin();
  WiFi.mode(WIFI_AP_STA);
  setupAPMode();
  initEspNow();
  setupWebServer();
  setupArduinoOTA();
  initButton();
  
  if (ENABLE_MDNS) {
    MDNS.begin(deviceName.c_str());
    MDNS.addService("http", "tcp", 80);
  }
  
  server.begin();
  
  dryRun.lowPowerStartTime = 0;
  dryRun.protectionTriggered = false;
  overload.lastRelayState = s31.getRelayState();
  overload.pumpStartTime = overload.lastRelayState ? millis() : 0;
  overload.inrushActive = false;
  overload.active = false;
  overload.cooldownUntil = 0;
  overload.bootInitialized = true;
  
  Serial.println("✓ System Ready");
  Serial.println("✓ AP: SmartPump-" + String(chipId & 0xFFFF, HEX));
  Serial.println("✓ IP: 192.168.4.1");
  Serial.println("✓ Version: " FIRMWARE_VERSION);
  Serial.println("✓ Simple UI: http://192.168.4.1");
  Serial.println("✓ Engineering UI: http://192.168.4.1/engmode");
  Serial.println("✓ Button: GPIO0 - Short press=toggle, Long press=Force Manual");
  Serial.println("✓ Dry Run: " + String(config.dry_run_enabled ? "ENABLED" : "DISABLED"));
  Serial.println("✓ Overload Protection: " + String(config.pump_load_protection_enabled ? "ACTIVE" : "DISABLED"));
  Serial.println("✓ EEPROM Save: Debounced (" + String(EEPROM_SAVE_DEBOUNCE_MS) + "ms)");
  Serial.println("════════════════════════════════════════════════════════════════");
}

void loop() {
  unsigned long currentMillis = millis();
  
  ESP.wdtFeed();
  handleButton();
  
  if (currentMillis - lastS31Update >= S31_UPDATE_INTERVAL) { 
    lastS31Update = currentMillis; 
    s31.update(); 
  }
  
  if (ENABLE_OTA && currentMillis - lastOTA >= OTA_INTERVAL) { 
    lastOTA = currentMillis; 
    ArduinoOTA.handle(); 
  }
  
  if (currentMillis - lastWebServer >= WEB_SERVER_INTERVAL) { 
    lastWebServer = currentMillis; 
    server.handleClient(); 
  }
  
  requestSensorData();
  controlPump();
  
  if (ENABLE_MDNS && currentMillis - lastMDNS >= MDNS_INTERVAL) { 
    lastMDNS = currentMillis; 
    MDNS.update(); 
  }
  
  processPendingConfigSave();
  
  delay(1);
}

// ================================================================================================
// @section     END OF CODE - v4.5.3 with Physical Button + Original UI
// ================================================================================================