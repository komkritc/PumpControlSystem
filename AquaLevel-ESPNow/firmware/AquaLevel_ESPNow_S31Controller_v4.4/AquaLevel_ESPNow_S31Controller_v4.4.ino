/**
 * ================================================================================================
 * SMART PUMP CONTROLLER - DUAL UI MODE v4.4.0
 * ================================================================================================
 * @file        AquaLevel_ESPNow_S31Controller_v4.4.ino
 * @version     4.4.0
 * @author      Smart Pump Controller
 * @license     MIT
 * 
 * @brief       Complete pump controller for Sonoff S31 with ESP-NOW, Dual Web Interface,
 *              comprehensive pump protection features, and SENSOR BYPASS / OVERRIDE MODE.
 * 
 * ================================================================================================
 * @section     FEATURE OVERVIEW
 * ================================================================================================
 * ✓ Dual UI: User-friendly interface + Engineering mode (full technical controls)
 * ✓ ESP-NOW wireless sensor communication with automatic data request every 15 seconds
 * ✓ Automatic pump control based on water level thresholds (low/high)
 * ✓ Dry-run protection (stops pump if low power consumption detected)
 * ✓ INTELLIGENT OVERLOAD PROTECTION:
 *   - Handles inrush current (2-7x normal for 1-3 seconds at startup)
 *   - Triggers only on SUSTAINED overload (prevents false trips)
 *   - Works in BOTH Auto AND Manual modes
 *   - Configurable inrush tolerance period and cooldown
 * ✓ SENSOR BYPASS / OVERRIDE MODE (NEW in v4.4.0):
 *   - Force pump ON/OFF when sensor is dead or for testing
 *   - Password-protected access (default: 1234)
 *   - Auto-expires after 1 hour for safety
 *   - Works independently of sensor status
 *   - Visual indicators in both UIs
 * ✓ Sensor failure handling with 4 configurable strategies
 * ✓ Real-time power monitoring (Watts, Volts, Amps, Power Factor)
 * ✓ Pump statistics tracking (runtime, cycles, energy, overload events)
 * ✓ OTA updates support
 * ✓ Persistent configuration storage in EEPROM
 * 
 * ================================================================================================
 * @section     UI ACCESS
 * ================================================================================================
 * Normal users:    http://192.168.4.1/        -> Simple, clean interface
 * Engineers:       http://192.168.4.1/engmode  -> Full technical interface
 * 
 * Override API:
 *   Force ON:   http://192.168.4.1/override?password=1234&state=on
 *   Force OFF:  http://192.168.4.1/override?password=1234&state=off
 *   Toggle:     http://192.168.4.1/override?password=1234&state=toggle
 *   Deactivate: http://192.168.4.1/override/off?password=1234
 *   Status:     http://192.168.4.1/override/status
 * 
 * Default AP Credentials:
 *   SSID: SmartPump-XXXX (where XXXX is chip ID)
 *   Password: 12345678
 *   IP: 192.168.4.1
 * 
 * ================================================================================================
 * @section     OVERRIDE MODE (SENSOR BYPASS)
 * ================================================================================================
 * 
 * PURPOSE: Allow pump control when sensor is dead, failed, or for manual testing
 * 
 * FEATURES:
 *   1. Password Protection - Default: "1234" (configurable via API/UI)
 *   2. Auto Timeout - Automatically disables after 1 hour for safety
 *   3. Visual Feedback - Badges indicate when override is active
 *   4. API Access - REST endpoints for automation/integration
 *   5. Persistent Password - Stored in EEPROM across reboots
 * 
 * USE CASES:
 *   - Sensor hardware failure
 *   - Emergency water pumping
 *   - System testing and maintenance
 *   - Manual override of automation
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

// ================================================================================================
// @section     FEATURE FLAGS
// ================================================================================================
#define ENABLE_MDNS             true    /**< Enable mDNS for easy discovery */
#define ENABLE_OTA              true    /**< Enable Over-The-Air firmware updates */

// ================================================================================================
// @section     SYSTEM TIMING CONSTANTS
// ================================================================================================
#define S31_UPDATE_INTERVAL     100     /**< Sonoff power monitoring update interval (ms) */
#define CONTROL_INTERVAL        250     /**< Pump control logic interval (ms) */
#define WEB_SERVER_INTERVAL     10      /**< Web server request handling interval (ms) */
#define OTA_INTERVAL            50      /**< OTA update handler interval (ms) */
#define MDNS_INTERVAL           1000    /**< mDNS service announcement interval (ms) */

// ================================================================================================
// @section     ESP-NOW TIMING CONSTANTS
// ================================================================================================
#define ESP_NOW_SEND_INTERVAL   15000   /**< Auto-send "get_measure" every 15 seconds */
#define ESP_NOW_DATA_TIMEOUT    30000   /**< Consider sensor data stale after 30 seconds */
#define SENSOR_HEARTBEAT_TIMEOUT 120000 /**< 2 minutes - declare sensor dead after this period */
#define MAX_LOG_ENTRIES         50      /**< Maximum ESP-NOW terminal log entries to store */

// ================================================================================================
// @section     OVERRIDE TIMING CONSTANTS
// ================================================================================================
#define OVERRIDE_TIMEOUT_MS     3600000 /**< Override auto-expires after 1 hour (ms) */
#define MAX_PASSWORD_LENGTH     5       /**< Maximum password length (including null terminator) */

// ================================================================================================
// @section     PIN DEFINITIONS
// ================================================================================================
#define RELAY_PIN   12    /**< Sonoff S31 built-in relay control pin (GPIO12) */

// ================================================================================================
// @struct      Config
// @brief       System configuration structure stored in EEPROM (persistent across reboots)
// ================================================================================================
struct Config {
  uint32_t magic = 0xDEADBEEF;                      /**< Validation magic number */
  
  // Water level control settings
  float low_threshold = 30.0;                      /**< Level % below which pump turns ON */
  float high_threshold = 80.0;                     /**< Level % above which pump turns OFF */
  bool auto_mode = true;                           /**< true = Auto, false = Manual control */
  
  // Dry run protection settings (Auto mode only)
  float min_power_threshold = 5.0;                 /**< Minimum power (W) for dry run detection */
  unsigned long pump_protection_time = 300;        /**< Seconds of low power before dry run stop */
  
  // Overload protection settings (BOTH Auto AND Manual modes)
  float max_power_threshold = 500.0;               /**< Max safe power (W) - overload trigger */
  bool pump_load_protection_enabled = true;        /**< Enable/disable overload protection */
  unsigned long overload_cooldown_seconds = 30;    /**< Cooldown period after overload (seconds) */
  unsigned long inrush_tolerance_ms = 2000;        /**< Time to ignore high current at start (ms) */
  
  // ESP-NOW settings
  bool use_espnow = true;                          /**< Enable/disable ESP-NOW communication */
  uint8_t peer_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  /**< Sensor MAC address */
  char peer_mac_str[18] = "FF:FF:FF:FF:FF:FF";     /**< Human-readable MAC string */
  int espnow_channel = 1;                          /**< WiFi channel for ESP-NOW (1-13) */
  
  // Sensor failure settings
  uint8_t sensor_failure_strategy = 0;             /**< 0=Stop,1=Maintain,2=ForceON,3=Cyclic */
  unsigned long sensor_timeout = SENSOR_HEARTBEAT_TIMEOUT;  /**< Time before sensor declared dead */
  bool sensor_emergency_stop = true;               /**< Emergency stop on sensor failure */
  unsigned long cyclic_on_duration = 300000;       /**< Cyclic mode ON duration (ms) */
  unsigned long cyclic_off_duration = 1800000;     /**< Cyclic mode OFF duration (ms) */
  
  // Override / Sensor Bypass settings (NEW in v4.4.0)
  bool sensor_bypass_mode = false;                 /**< Force pump control without sensor */
  char override_password[MAX_PASSWORD_LENGTH] = "1234";  /**< Password for override access */
  bool override_active = false;                    /**< Currently in override mode? */
  bool manual_override_state = false;              /**< Manual override pump state */
} config;

// ================================================================================================
// @struct      OverloadProtectionState
// @brief       Runtime state for intelligent overload protection with inrush handling
// ================================================================================================
struct OverloadProtectionState {
  bool active = false;                             /**< Protection currently triggered? */
  unsigned long cooldownUntil = 0;                 /**< Timestamp when cooldown period ends */
  unsigned long pumpStartTime = 0;                 /**< When pump was last started (millis) */
  bool waitingForStableOperation = false;          /**< Currently in inrush tolerance period? */
  unsigned long overloadCount = 0;                 /**< Total number of overload events */
  float lastOverloadPower = 0;                     /**< Power reading from last overload event */
  bool lastRelayState = false;                     /**< Previous relay state for edge detection */
} overload;

// ================================================================================================
// @section     OVERRIDE RUNTIME VARIABLES
// ================================================================================================
unsigned long overrideStartTime = 0;               /**< When override was activated (millis) */

// ================================================================================================
// @section     GLOBAL OBJECTS
// ================================================================================================
SonoffS31 s31(RELAY_PIN);                          /**< Sonoff power monitoring and relay control */
ESP8266WebServer server(80);                       /**< Web server instance for UI */
String deviceName = "s31-pump";                    /**< Device hostname for mDNS and OTA */

// ================================================================================================
// @section     TIMING VARIABLES
// ================================================================================================
unsigned long lastS31Update = 0;
unsigned long lastControlCheck = 0;
unsigned long lastWebServer = 0;
unsigned long lastOTA = 0;
unsigned long lastMDNS = 0;

// ================================================================================================
// @section     SENSOR DATA VARIABLES
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
// @section     CYCLIC MODE VARIABLES
// ================================================================================================
unsigned long cyclicLastSwitchTime = 0;
bool cyclicPumpState = false;

// ================================================================================================
// @section     POWER QUALITY VARIABLES
// ================================================================================================
float powerFactor = 0.0;
float apparentPower = 0.0;
float reactivePower = 0.0;

// ================================================================================================
// @struct      EspNowPacket
// @brief       Structure for ESP-NOW data packets exchanged with sensor
// ================================================================================================
typedef struct __attribute__((packed)) {
  uint32_t seq;                                    /**< Sequence number for packet tracking */
  uint32_t timestamp;                              /**< Microsecond timestamp */
  char msg[64];                                    /**< Message payload (command or sensor data) */
} EspNowPacket;

EspNowPacket outgoing;
EspNowPacket incoming;

// ================================================================================================
// @struct      EspNowLogEntry
// @brief       Structured log entry for ESP-NOW message history
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
// @section     ESP-NOW GLOBALS
// ================================================================================================
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool espnow_initialized = false;
unsigned long lastEspNowSend = 0;
uint32_t espnow_seq = 0;
uint32_t espnow_msg_counter = 0;
std::vector<EspNowLogEntry> espnow_log;

// ================================================================================================
// @struct      PumpStats
// @brief       Pump operational statistics for monitoring and maintenance
// ================================================================================================
struct PumpStats {
  unsigned long totalRuntimeSeconds = 0;           /**< Total pump ON time (seconds) */
  float totalEnergyKwh = 0;                        /**< Total energy consumed (kWh) */
  int pumpCycles = 0;                              /**< Number of times pump started */
  char lastStartStr[32] = "Never";                 /**< Human-readable last start time */
  unsigned long lastPumpOnTime = 0;                /**< Millis() when pump was last turned ON */
  bool wasRunning = false;                         /**< Previous pump state for runtime calculation */
} pumpStats;

std::vector<String> failureLog;                    /**< Circular buffer of failure events */

// ================================================================================================
// @section     USER-FRIENDLY HTML UI (Simple Mode)
// @brief       Clean, minimal interface for everyday users with override controls
// ================================================================================================
const char simple_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>Smart Pump | Easy Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Roboto, system-ui, sans-serif; background: #f1f5f9; padding: 16px; color: #0f172a; }
        .container { max-width: 550px; margin: 0 auto; }
        .card { background: white; border-radius: 32px; padding: 20px 18px; margin-bottom: 18px; box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05); }
        .header { text-align: center; margin-bottom: 8px; }
        .header h1 { font-size: 1.7rem; font-weight: 600; background: linear-gradient(135deg, #0f172a, #2563eb); background-clip: text; -webkit-background-clip: text; color: transparent; }
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
        .pump-btn.override { background: #f59e0b; border: 3px solid #dc2626; animation: pulse 0.5s infinite; }
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
        .override-card { border: 2px solid #f59e0b; background: #fffbeb; }
        .override-badge-active { background: #dc2626; color: white; animation: pulse 1s infinite; }
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>💧 AquaPro S31 Controller</h1>
        <div class="badge">ESP-NOW | Offline mode</div>
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
        </div>
        <div class="card">
            <div class="mode-row"><span class="mode-text">🤖 Auto mode</span><label class="toggle-switch"><input type="checkbox" id="autoModeToggle" onchange="toggleAutoMode()"><span class="slider"></span></label><span class="mode-text">👆 Manual</span></div>
            <button id="pumpActionBtn" class="pump-btn" onclick="manualPumpToggle()">PUMP OFF</button>
            <div id="pumpHint" style="font-size: 0.7rem; text-align: center;">✅ Auto mode handles pump</div>
            <div id="protectionNote" style="font-size: 0.7rem; text-align: center; color: #d97706; margin-top: 4px;">🛡️ Overload protection ACTIVE in BOTH modes (handles inrush current)</div>
            <hr>
            <div class="flex-between"><span>📦 Total runtime</span><strong><span id="totalRunMinutes">0</span> min</strong></div>
            <div class="flex-between"><span>🔄 Cycles count</span><strong><span id="cyclesCount">0</span></strong></div>
            <div class="flex-between"><span>⚠️ Overload events</span><strong><span id="overloadCount">0</span></strong></div>
        </div>
        
        <!-- OVERRIDE CONTROL CARD - NEW in v4.4.0 -->
        <div class="card override-card" id="overrideCard">
            <div class="flex-between">
                <span>🔓 SENSOR BYPASS / OVERRIDE MODE</span>
                <span id="overrideBadge" class="protection-badge override-badge-active" style="display: none;">⚠️ OVERRIDE ACTIVE</span>
            </div>
            <div id="overridePanel">
                <div class="small-note" style="margin-bottom: 12px;">
                    ⚡ Use when sensor is dead or for manual testing<br>
                    <strong>🔐 Password required: 1234 (default)</strong>
                </div>
                <div style="display: flex; gap: 10px; margin-bottom: 10px;">
                    <input type="password" id="overridePassword" placeholder="Password" style="flex:1; padding: 10px; border-radius: 20px; border: 1px solid #e2e8f0;">
                    <button onclick="overridePump('on')" style="background:#10b981; border:none; padding:10px 20px; border-radius:20px; color:white; font-weight:bold;">🔓 FORCE ON</button>
                    <button onclick="overridePump('off')" style="background:#ef4444; border:none; padding:10px 20px; border-radius:20px; color:white; font-weight:bold;">🔒 FORCE OFF</button>
                </div>
                <div style="display: flex; gap: 10px;">
                    <button onclick="overridePump('toggle')" style="background:#f59e0b; border:none; padding:10px 20px; border-radius:20px; color:white; font-weight:bold; flex:1;">🔄 TOGGLE</button>
                    <button onclick="deactivateOverride()" style="background:#6b7280; border:none; padding:10px 20px; border-radius:20px; color:white; font-weight:bold; flex:1;">🔒 EXIT OVERRIDE</button>
                </div>
                <div id="overrideStatus" style="margin-top: 10px; font-size: 0.7rem; text-align: center; color: #d97706;"></div>
            </div>
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
            <div class="setting-group"><label>⚠️ Dry run protection (seconds)</label><input type="number" id="dryRunProtection" step="30" min="10"><div class="small-note">If pump runs with low power, stop after this many seconds (Auto mode only)</div></div>
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
            <div class="setting-group"><label>🔐 Override Password (max 4 chars)</label><input type="text" id="overridePasswordSetting" maxlength="4" placeholder="1234"><div class="small-note">Password for sensor bypass/override mode</div></div>
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
            document.getElementById('dryRunProtection').value = cfg.dry_run_protection || 300;
            document.getElementById('loadProtectionToggle').checked = cfg.pump_load_protection_enabled !== false;
            document.getElementById('maxPowerThreshold').value = cfg.max_power_threshold || 500;
            document.getElementById('inrushTolerance').value = cfg.inrush_tolerance_ms || 2000;
            document.getElementById('cooldownPeriod').value = cfg.overload_cooldown_seconds || 30;
            document.getElementById('overridePasswordSetting').value = cfg.override_password || '1234';
        } 
    }
    
    async function savePumpSettings() {
        const low = parseFloat(document.getElementById('lowThreshold').value);
        const high = parseFloat(document.getElementById('highThreshold').value);
        const dry = parseInt(document.getElementById('dryRunProtection').value);
        const loadProtection = document.getElementById('loadProtectionToggle').checked;
        const maxPower = parseFloat(document.getElementById('maxPowerThreshold').value);
        const inrush = parseInt(document.getElementById('inrushTolerance').value);
        const cooldown = parseInt(document.getElementById('cooldownPeriod').value);
        const overridePass = document.getElementById('overridePasswordSetting').value;
        
        if(low >= high) { alert("Low threshold must be less than High threshold"); return; }
        if(maxPower < 10) { alert("Max power must be at least 10 watts"); return; }
        if(inrush < 100) { alert("Inrush tolerance must be at least 100ms"); return; }
        if(overridePass.length < 1 || overridePass.length > 4) { alert("Password must be 1-4 characters"); return; }
        
        const response = await fetch('/config/pump', { 
            method: 'POST', 
            headers: { 'Content-Type': 'application/json' }, 
            body: JSON.stringify({ 
                low_threshold: low, 
                high_threshold: high, 
                dry_run_protection: dry,
                pump_load_protection_enabled: loadProtection,
                max_power_threshold: maxPower,
                inrush_tolerance_ms: inrush,
                overload_cooldown_seconds: cooldown,
                override_password: overridePass
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
            btn.disabled = false; 
            btn.style.opacity = "1"; 
            hint.innerText = "✋ Manual mode: tap to turn pump ON/OFF (Overload protection still active)"; 
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
    
    async function overridePump(state) {
        const password = document.getElementById('overridePassword').value || '1234';
        const response = await fetch(`/override?password=${password}&state=${state}`);
        const result = await response.text();
        if(result.startsWith("ERROR")) {
            alert(result);
        } else {
            alert(result);
            refreshDashboard();
            updateOverrideStatus();
        }
    }
    
    async function deactivateOverride() {
        const password = document.getElementById('overridePassword').value || '1234';
        const response = await fetch(`/override/off?password=${password}`);
        const result = await response.text();
        if(result.startsWith("ERROR")) {
            alert(result);
        } else {
            alert(result);
            refreshDashboard();
            updateOverrideStatus();
        }
    }
    
    async function updateOverrideStatus() {
        const status = await fetchJSON('/override/status');
        if(status) {
            const badge = document.getElementById('overrideBadge');
            const statusDiv = document.getElementById('overrideStatus');
            const btn = document.getElementById('pumpActionBtn');
            if(status.override_active) {
                badge.style.display = 'inline-block';
                statusDiv.innerHTML = `🔓 OVERRIDE ACTIVE - Time remaining: ${status.override_time_remaining}s | Pump: ${status.manual_override_state ? 'ON' : 'OFF'}`;
                if(btn && !document.getElementById('autoModeToggle').checked) {
                    btn.classList.add('override');
                }
            } else {
                badge.style.display = 'none';
                if(btn) btn.classList.remove('override');
                if(status.sensor_dead) {
                    statusDiv.innerHTML = '⚠️ SENSOR DEAD - Use override to control pump manually (password: ' + (status.override_password || '1234') + ')';
                } else {
                    statusDiv.innerHTML = '✅ Sensor OK - Override available if needed';
                }
            }
        }
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
        
        const btn = document.getElementById('pumpActionBtn');
        if(d.override_active) {
            btn.innerText = "🔓 OVERRIDE: " + (d.manual_override_state ? "PUMP ON" : "PUMP OFF");
            btn.classList.add('override');
            btn.classList.remove('running', 'cooldown', 'inrush');
        } else if(d.inCooldown) {
            btn.innerText = "⏱️ COOLDOWN - " + d.cooldownRemaining + "s";
            btn.classList.add('cooldown');
            btn.classList.remove('running', 'inrush', 'override');
            btn.disabled = true;
        } else if(d.inInrushPeriod) {
            btn.innerText = "⚡ STARTING - Inrush mode";
            btn.classList.add('inrush');
            btn.classList.remove('running', 'cooldown', 'override');
        } else if(d.pumpState) { 
            btn.innerText = "💧 PUMP RUNNING"; 
            btn.classList.add('running');
            btn.classList.remove('inrush', 'cooldown', 'override');
            if(!document.getElementById('autoModeToggle').checked) btn.disabled = false;
        } else { 
            btn.innerText = "⏹️ PUMP OFF"; 
            btn.classList.remove('running', 'inrush', 'cooldown', 'override');
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
        
        const sensorSpan = document.getElementById('sensorBadge');
        if(d.sensorHealthy) sensorSpan.innerHTML = '<span class="led led-green"></span> ✅ Healthy';
        else if(d.sensorWarning) sensorSpan.innerHTML = '<span class="led led-yellow"></span> ⚠️ Weak signal';
        else sensorSpan.innerHTML = '<span class="led led-red"></span> ❌ No sensor data';
        document.getElementById('lastSeenText').innerText = d.sensorLastSeen || 'never';
        const toggle = document.getElementById('autoModeToggle');
        if(toggle && toggle.checked !== (d.autoMode === true)) { toggle.checked = (d.autoMode === true); updatePumpButtonAccess(); }
        const stats = await fetchJSON('/stats');
        if(stats) { document.getElementById('totalRunMinutes').innerText = stats.total_runtime || 0; document.getElementById('cyclesCount').innerText = stats.pump_cycles || 0; }
        await updateOverrideStatus();
    }
    
    setInterval(refreshDashboard, 1000);
    window.onload = () => { refreshDashboard(); loadPumpSettings(); updatePumpButtonAccess(); };
</script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     ENGINEERING HTML UI (Full Technical Interface)
// @brief       Complete professional interface with override tab
// ================================================================================================
const char engineering_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Smart Pump Controller - Engineering Mode</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; border-radius: 20px; padding: 25px; margin-bottom: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.2); }
        h1 { text-align: center; color: #333; margin-bottom: 5px; font-size: 24px; }
        .version { text-align: center; color: #999; font-size: 11px; margin-bottom: 5px; }
        .eng-badge { text-align: center; background: #FEE2E2; color: #991B1B; padding: 8px; border-radius: 10px; font-size: 12px; margin-bottom: 15px; font-weight: bold; }
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
        .pump-btn.inrush { background: #8b5cf6; }
        .pump-btn.override { background: #f59e0b; border: 3px solid #dc2626; }
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
        .override-section { border: 2px solid #f59e0b; background: #fffbeb; }
        @media (max-width: 600px) { .stats-grid { grid-template-columns: 1fr; } .nav-buttons { flex-direction: column; } .strategy-selector { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>🔧 Smart Pump Controller - Engineering Mode</h1>
            <div class="version">v4.4.0 | Full Technical Interface with Override</div>
            <div class="eng-badge">⚙️ ENGINEERING MODE - Overload protection + Sensor Bypass + Override</div>
            
            <div class="nav-buttons">
                <button class="nav-btn active" onclick="showSection('dashboard')">Dashboard</button>
                <button class="nav-btn" onclick="showSection('espnow')">ESP-NOW</button>
                <button class="nav-btn" onclick="showSection('pump')">Pump Settings</button>
                <button class="nav-btn" onclick="showSection('override')">🔓 Override</button>
                <button class="nav-btn" onclick="showSection('safety')">⚡ Safety</button>
                <button class="nav-btn" onclick="showSection('stats')">Statistics</button>
                <button class="nav-btn" onclick="showSection('system')">System</button>
            </div>
            
            <div id="dashboardSection">
                <div class="config-section" style="margin-bottom:15px">
                    <div>📡 Sensor: <span id="sensorHealth" class="sensor-health healthy">Checking...</span>
                    <span id="sensorLastSeen" style="font-size:11px;margin-left:10px"></span></div>
                    <div>🛡️ Overload Protection: <span id="overloadStatus" style="font-weight:bold">Active</span> <span id="overloadBadge" class="overload-badge" style="display:none">⚠️ TRIGGERED!</span></div>
                    <div>⚡ Inrush Period: <span id="inrushStatus">None</span></div>
                    <div>⏱️ Cooldown: <span id="cooldownStatus">None</span></div>
                    <div>🔓 Override: <span id="overrideStatusEng" style="font-weight:bold">Inactive</span></div>
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
                <div class="config-group"><label>Minimum Power Detection (Watts)</label><input type="number" id="minPower" step="1" placeholder="5"><div class="info-text">Pump stops if power below this for dry run protection (Auto mode only)</div></div>
                <div class="config-group"><label>Dry Run Protection (seconds)</label><input type="number" id="dryRunProtection" step="30" placeholder="300"></div>
                <hr>
                <div style="background:#FEF3C7; padding:10px; border-radius:10px; margin-bottom:15px;">
                    <strong>🛡️ OVERLOAD PROTECTION WITH INRUSH HANDLING (WORKS IN BOTH MODES)</strong>
                </div>
                <div class="config-group"><label class="checkbox-label"><input type="checkbox" id="loadProtectionToggle"> ✅ Enable Overload Protection</label><div class="info-text">★ Stops pump for SUSTAINED overload AFTER inrush period</div></div>
                <div class="config-group"><label>⚡ Max Power Limit (Watts)</label><input type="number" id="maxPower" step="10" placeholder="500"><div class="info-text">Pump stops if power exceeds this value AFTER startup inrush</div></div>
                <div class="config-group"><label>⚡ Inrush Tolerance (milliseconds)</label><input type="number" id="inrushTolerance" step="100" min="100" max="10000" placeholder="2000"><div class="info-text">Time to ignore high current at startup (2-7x normal). Critical for motor pumps!</div></div>
                <div class="config-group"><label>⏱️ Cooldown Period (seconds)</label><input type="number" id="cooldownPeriod" step="5" placeholder="30"><div class="info-text">Time to wait before allowing pump restart after overload</div></div>
                <hr>
                <div class="config-group"><label>🔐 Override Password (max 4 chars)</label><input type="text" id="overridePasswordSetting" maxlength="4" placeholder="1234"><div class="info-text">Password for sensor bypass/override mode</div></div>
                <button onclick="savePumpSettings()">Save Settings</button>
            </div>
            
            <div id="overrideSection" style="display:none" class="override-section">
                <h3>🔓 Sensor Bypass & Override Mode</h3>
                <div class="config-section">
                    <div class="config-group">
                        <label>🔐 Override Password</label>
                        <input type="text" id="overridePasswordInput" placeholder="1234" maxlength="4">
                        <div class="info-text">Password required for override operations (max 4 chars)</div>
                    </div>
                    <div style="background:#FEF3C7; padding:15px; border-radius:10px; margin-top:15px;">
                        <strong>⚠️ Manual Override Controls</strong><br>
                        <div style="display:flex; gap:10px; margin-top:10px; flex-wrap:wrap;">
                            <button onclick="manualOverride('on')" style="background:#10b981;">🔓 FORCE ON</button>
                            <button onclick="manualOverride('off')" style="background:#ef4444;">🔒 FORCE OFF</button>
                            <button onclick="manualOverride('toggle')" style="background:#f59e0b;">🔄 TOGGLE</button>
                            <button onclick="manualOverride('exit')" style="background:#6b7280;">🚪 EXIT OVERRIDE</button>
                        </div>
                        <div id="overrideStatusEngDetail" style="margin-top:10px; font-size:12px;"></div>
                    </div>
                    <div class="info-text" style="margin-top:15px">
                        💡 Override bypasses ALL sensor logic and water level control.<br>
                        🔐 Password is required for ALL override operations.<br>
                        ⏰ Override automatically expires after 1 hour for safety.<br>
                        📡 Works even when sensor is dead or disconnected.
                    </div>
                </div>
            </div>
            
            <div id="safetySection" style="display:none">
                <h3>🔒 Sensor Failure Protection</h3>
                <div class="info-text" style="margin-bottom:15px">⚠️ Strategy 0 (Stop Pump) works in BOTH Auto and Manual modes for maximum safety!</div>
                <div class="config-group"><label>Sensor Timeout (seconds)</label><input type="number" id="sensorTimeout" step="10" placeholder="120"></div>
                <div class="config-group"><label>📋 Failure Strategy</label>
                    <div class="strategy-selector">
                        <div class="strategy-card" data-strategy="0" onclick="selectStrategy(0)"><div class="strategy-title">🛑 Stop Pump</div><div class="strategy-desc">Immediately stop pump for maximum safety (Auto & Manual)</div></div>
                        <div class="strategy-card" data-strategy="1" onclick="selectStrategy(1)"><div class="strategy-title">⚖️ Maintain State</div><div class="strategy-desc">Keep pump in last known safe state</div></div>
                        <div class="strategy-card" data-strategy="2" onclick="selectStrategy(2)"><div class="strategy-title">⚠️ Force ON</div><div class="strategy-desc">Force pump ON (RISKY - potential overflow!)</div></div>
                        <div class="strategy-card" data-strategy="3" onclick="selectStrategy(3)"><div class="strategy-title">🔄 Cyclic Mode</div><div class="strategy-desc">Run pump in cycles (e.g., 5min ON, 30min OFF)</div></div>
                    </div>
                </div>
                <div id="cyclicSettings" style="display:none"><div class="config-group"><label>ON Duration (seconds)</label><input type="number" id="cyclicOnTime" step="60" placeholder="300"></div><div class="config-group"><label>OFF Duration (seconds)</label><input type="number" id="cyclicOffTime" step="60" placeholder="1800"></div></div>
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
            document.getElementById('overrideSection').style.display = s==='override'?'block':'none';
            document.getElementById('safetySection').style.display = s==='safety'?'block':'none';
            document.getElementById('statsSection').style.display = s==='stats'?'block':'none';
            document.getElementById('systemSection').style.display = s==='system'?'block':'none';
            document.querySelectorAll('.nav-btn').forEach(btn=>btn.classList.remove('active'));
            event.target.classList.add('active');
            if(s==='espnow'){loadEspNowSettings();refreshEspNowLog();}
            if(s==='pump')loadPumpSettings();
            if(s==='safety'){loadSafetySettings();loadFailureLog();}
            if(s==='system')loadSystemSettings();
            if(s==='stats')loadStats();
            if(s==='override')loadOverrideStatus();
        }
        
        async function toggleMode(){const mode=document.getElementById('modeToggle').checked?'auto':'manual';await fetch('/mode?mode='+mode);fetchData();updatePumpButtonState();}
        function updateModeUI(autoMode){const toggle=document.getElementById('modeToggle');const modeStatus=document.getElementById('modeStatus');const pumpBtn=document.getElementById('pumpBtn');if(autoMode){toggle.checked=true;modeStatus.innerHTML='🤖 Auto Mode';modeStatus.className='mode-status auto';pumpBtn.classList.add('disabled');}else{toggle.checked=false;modeStatus.innerHTML='👆 Manual Mode';modeStatus.className='mode-status manual';pumpBtn.classList.remove('disabled');}}
        function updatePumpButtonState(){const toggle=document.getElementById('modeToggle');const pumpBtn=document.getElementById('pumpBtn');if(toggle&&!toggle.checked)pumpBtn.classList.remove('disabled');else if(toggle&&toggle.checked)pumpBtn.classList.add('disabled');}
        function selectStrategy(s){selectedStrategy=s;document.querySelectorAll('.strategy-card').forEach(c=>c.classList.remove('selected'));document.querySelector(`.strategy-card[data-strategy="${s}"]`).classList.add('selected');document.getElementById('cyclicSettings').style.display=s===3?'block':'none';}
        
        async function manualGetMeasure(){const btn=document.getElementById('manualRequestBtn');const ot=btn.innerHTML;btn.innerHTML='⏳ Sending...';btn.disabled=true;document.getElementById('sendStatus').innerHTML='⏳ Sending...';try{const r=await fetch('/espnow/request');const t=await r.text();document.getElementById('sendStatus').innerHTML='✅ '+t;setTimeout(()=>document.getElementById('sendStatus').innerHTML='',3000);refreshEspNowLog();}catch(e){document.getElementById('sendStatus').innerHTML='❌ Failed';}finally{btn.innerHTML=ot;btn.disabled=false;}}
        
        async function manualOverride(action) {
            const password = document.getElementById('overridePasswordInput').value;
            if(!password) {
                alert("Please enter override password");
                return;
            }
            if(action === 'exit') {
                const response = await fetch(`/override/off?password=${password}`);
                const result = await response.text();
                alert(result);
            } else {
                const response = await fetch(`/override?password=${password}&state=${action}`);
                const result = await response.text();
                alert(result);
            }
            fetchData();
            loadOverrideStatus();
        }
        
        async function loadOverrideStatus() {
            const status = await fetchJSON('/override/status');
            if(status) {
                const statusDiv = document.getElementById('overrideStatusEngDetail');
                const statusSpan = document.getElementById('overrideStatusEng');
                if(status.override_active) {
                    statusSpan.innerHTML = `🔴 ACTIVE - ${status.override_time_remaining}s remaining`;
                    statusSpan.style.color = '#dc2626';
                    if(statusDiv) {
                        statusDiv.innerHTML = `🔴 OVERRIDE ACTIVE | Time: ${status.override_time_remaining}s | Pump: ${status.manual_override_state ? 'ON' : 'OFF'}<br>
                        ⚠️ Sensor bypass is active - Pump controlled manually`;
                        statusDiv.style.color = '#d97706';
                        statusDiv.style.fontWeight = 'bold';
                    }
                } else {
                    statusSpan.innerHTML = `⚪ Inactive`;
                    statusSpan.style.color = '#10b981';
                    if(statusDiv) {
                        statusDiv.innerHTML = `✅ Normal mode | Sensor: ${status.sensor_dead ? 'DEAD' : 'OK'}<br>
                        💡 Use override controls above to force pump ON/OFF if needed`;
                        statusDiv.style.color = '#10b981';
                    }
                }
            }
        }
        
        function getPFClass(pf,running){if(!running)return'pf-idle';if(pf>=0.95)return'pf-good';if(pf>=0.8)return'pf-warning';return'pf-bad';}
        function getPFQuality(pf,running){if(!running)return'⏸ Idle (PF=0)';if(pf>=0.95)return'✓ Excellent';if(pf>=0.9)return'✓ Good';if(pf>=0.8)return'⚠ Fair';return'⚠ Poor';}
        
        async function loadSafetySettings(){const cfg=await fetchConfig();document.getElementById('sensorTimeout').value=cfg.sensor_timeout||120;selectedStrategy=cfg.failure_strategy||0;selectStrategy(selectedStrategy);document.getElementById('cyclicOnTime').value=cfg.cyclic_on_duration||300;document.getElementById('cyclicOffTime').value=cfg.cyclic_off_duration||1800;}
        async function loadEspNowSettings(){const cfg=await fetchConfig();document.getElementById('useEspNow').value=cfg.use_espnow?'true':'false';document.getElementById('peerMac').value=cfg.peer_mac||'FF:FF:FF:FF:FF:FF';document.getElementById('espnowChannel').value=cfg.espnow_channel||1;const r=await fetch('/info');const i=await r.json();document.getElementById('deviceMac').innerHTML=i.mac||'Unknown';document.getElementById('apSsid').innerHTML=i.ap_ssid||'Unknown';}
        async function loadPumpSettings(){const cfg=await fetchConfig();document.getElementById('lowThreshold').value=cfg.low_threshold||30;document.getElementById('highThreshold').value=cfg.high_threshold||80;document.getElementById('minPower').value=cfg.min_power||5;document.getElementById('dryRunProtection').value=cfg.dry_run_protection||300;document.getElementById('loadProtectionToggle').checked=cfg.pump_load_protection_enabled!==false;document.getElementById('maxPower').value=cfg.max_power_threshold||500;document.getElementById('inrushTolerance').value=cfg.inrush_tolerance_ms||2000;document.getElementById('cooldownPeriod').value=cfg.overload_cooldown_seconds||30;document.getElementById('overridePasswordSetting').value=cfg.override_password||'1234';document.getElementById('overridePasswordInput').value=cfg.override_password||'1234';}
        async function loadSystemSettings(){const cfg=await fetchConfig();document.getElementById('hostname').value=cfg.hostname||'s31-pump';}
        async function fetchConfig(){const r=await fetch('/config');return await r.json();}
        async function fetchJSON(url){try{const r=await fetch(url);return await r.json();}catch(e){return null;}}
        async function refreshEspNowLog(){const r=await fetch('/espnow/log');const h=await r.text();document.getElementById('espnowTerminal').innerHTML=h;}
        async function clearEspNowLog(){await fetch('/espnow/clear',{method:'POST'});refreshEspNowLog();}
        async function exportEspNowLog(){window.location.href='/espnow/export';}
        async function loadFailureLog(){const r=await fetch('/failurelog');const h=await r.text();document.getElementById('failureLog').innerHTML=h;}
        async function clearFailureLog(){await fetch('/failurelog/clear',{method:'POST'});loadFailureLog();}
        
        async function fetchData(){try{const r=await fetch('/data');const d=await r.json();document.getElementById('waterLevel').innerText=d.waterLevel.toFixed(1);document.getElementById('distance').innerText=d.distance.toFixed(1);document.getElementById('volume').innerText=d.volume.toFixed(0);document.getElementById('battery').innerText=d.batteryVoltage.toFixed(2);document.getElementById('levelBar').style.width=d.waterLevel+'%';document.getElementById('levelBar').innerHTML=d.waterLevel.toFixed(0)+'%';document.getElementById('power').innerText=d.power.toFixed(1);document.getElementById('voltage').innerText=d.voltage.toFixed(1);document.getElementById('current').innerText=d.current.toFixed(2);document.getElementById('energy').innerText=d.energy.toFixed(2);const isRunning=d.pumpState;const pf=isRunning?d.powerFactor:0;document.getElementById('powerFactor').innerText=pf.toFixed(3);document.getElementById('powerFactor').className=getPFClass(pf,isRunning);document.getElementById('pfQuality').innerHTML=getPFQuality(pf,isRunning);const pumpBtn=document.getElementById('pumpBtn');const pumpText=document.getElementById('pumpText');if(d.override_active){pumpBtn.className='pump-btn override';pumpText.innerText='🔓 OVERRIDE: '+(d.manual_override_state?'ON':'OFF');pumpBtn.disabled=false;}else if(d.inCooldown){pumpBtn.className='pump-btn cooldown';pumpText.innerText='COOLDOWN';pumpBtn.disabled=true;}else if(d.inInrushPeriod){pumpBtn.className='pump-btn inrush';pumpText.innerText='STARTING';if(!d.autoMode)pumpBtn.disabled=false;}else if(isRunning){pumpBtn.className='pump-btn running';pumpText.innerText='PUMP ON';if(!d.autoMode)pumpBtn.disabled=false;}else{pumpBtn.className='pump-btn stopped';pumpText.innerText='PUMP OFF';if(!d.autoMode)pumpBtn.disabled=false;}document.getElementById('pumpReason').innerText=d.pumpReason;updateModeUI(d.autoMode);const sH=document.getElementById('sensorHealth');if(d.sensorHealthy){sH.className='sensor-health healthy';sH.innerHTML='✅ Sensor Healthy';}else if(d.sensorWarning){sH.className='sensor-health warning';sH.innerHTML='⚠️ Sensor Warning';}else{sH.className='sensor-health dead';sH.innerHTML='❌ Sensor Dead';}document.getElementById('sensorLastSeen').innerHTML=d.sensorLastSeen||'';document.getElementById('espnowStatus').innerHTML=d.espnowActive?'Active':'Disabled';document.getElementById('espnowLed').className='status-led '+(d.espnowActive?(d.espnowDataValid?'status-online':'status-warning'):'status-offline');document.getElementById('dataSource').innerHTML=d.dataSource||'No Data';document.getElementById('activeStrategy').innerHTML=d.activeStrategy||'-';document.getElementById('lastUpdate').innerHTML='Updated: '+new Date().toLocaleTimeString();document.getElementById('overloadStatus').innerHTML=d.overloadProtectionActive?'⚠️ TRIGGERED!':'Active';document.getElementById('overloadBadge').style.display=d.overloadProtectionActive?'inline-block':'none';document.getElementById('inrushStatus').innerHTML=d.inInrushPeriod?'Active ('+d.inrushRemaining+'ms)':'None';document.getElementById('cooldownStatus').innerHTML=d.inCooldown?d.cooldownRemaining+'s remaining':'None';if(currentSection==='espnow'){document.getElementById('espnowMsgCount').innerText=d.espnowMsgCount||0;document.getElementById('espnowLastTime').innerText=d.espnowLastTime||'Never';document.getElementById('espnowStatusText').innerHTML=d.espnowActive?(d.espnowDataValid?'✅ Receiving':'⚠️ No Data'):'❌ Disabled';document.getElementById('lastSeq').innerText=d.lastSeq||'-';}}catch(e){console.error(e);}}
        
        async function loadStats(){try{const r=await fetch('/stats');const s=await r.json();document.getElementById('totalRuntime').innerText=s.total_runtime+' min';document.getElementById('totalEnergy').innerText=s.total_energy+' kWh';document.getElementById('pumpCycles').innerText=s.pump_cycles;document.getElementById('lastStart').innerText=s.last_start||'Never';document.getElementById('overloadEvents').innerText=s.overload_stop_count||0;}catch(e){}}
        
        async function saveEspNow(){const s={use_espnow:document.getElementById('useEspNow').value==='true',peer_mac:document.getElementById('peerMac').value,espnow_channel:parseInt(document.getElementById('espnowChannel').value)};await fetch('/config/espnow',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Saved! Rebooting...');setTimeout(()=>location.reload(),3000);}
        async function saveSafetySettings(){const s={sensor_timeout:parseInt(document.getElementById('sensorTimeout').value),failure_strategy:selectedStrategy,cyclic_on_duration:parseInt(document.getElementById('cyclicOnTime').value)||300,cyclic_off_duration:parseInt(document.getElementById('cyclicOffTime').value)||1800};await fetch('/config/safety',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Safety settings saved!');}
        async function savePumpSettings(){const s={low_threshold:parseFloat(document.getElementById('lowThreshold').value),high_threshold:parseFloat(document.getElementById('highThreshold').value),min_power:parseFloat(document.getElementById('minPower').value),dry_run_protection:parseInt(document.getElementById('dryRunProtection').value),pump_load_protection_enabled:document.getElementById('loadProtectionToggle').checked,max_power_threshold:parseFloat(document.getElementById('maxPower').value),inrush_tolerance_ms:parseInt(document.getElementById('inrushTolerance').value),overload_cooldown_seconds:parseInt(document.getElementById('cooldownPeriod').value),override_password:document.getElementById('overridePasswordSetting').value};await fetch('/config/pump',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Pump settings saved!');}
        async function saveSystem(){const s={hostname:document.getElementById('hostname').value};await fetch('/config/system',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(s)});alert('Saved! Rebooting...');setTimeout(()=>location.reload(),3000);}
        async function togglePump(){const cfg=await fetchConfig();if(cfg.auto_mode){alert('Switch to Manual Mode first');return;}const d=await fetchJSON('/data');if(d&&d.inCooldown){alert('Pump in cooldown! Please wait.');return;}await fetch('/toggle');fetchData();}
        async function factoryReset(){if(confirm('FACTORY RESET - Erase ALL settings?')){await fetch('/factoryreset');}}
        async function rebootNow(){if(confirm('Reboot device?')){await fetch('/reboot');alert('Rebooting...');}}
        async function resetStats(){if(confirm('Reset statistics?')){await fetch('/resetstats');loadStats();}}
        async function exportStats(){window.location.href='/exportstats';}
        
        setInterval(fetchData,1000);setInterval(loadStats,10000);setInterval(loadOverrideStatus,2000);
        if(currentSection==='safety')setInterval(loadFailureLog,5000);
        fetchData();loadStats();
    </script>
</body>
</html>
)rawliteral";

// ================================================================================================
// @section     HELPER FUNCTIONS
// ================================================================================================

/**
 * @brief   Adds a timestamped entry to the failure log
 * @param   message - Description of the failure event
 */
void addFailureLogEntry(String message) {
  failureLog.insert(failureLog.begin(), message);
  while (failureLog.size() > 50) failureLog.pop_back();
}

/**
 * @brief   Generates HTML representation of the failure log
 * @return  String containing HTML divs for each log entry
 */
String getFailureLogHTML() {
  if (failureLog.empty()) return "<div class='terminal-entry'>No failure events logged</div>";
  String html = "";
  for (size_t i = 0; i < failureLog.size() && i < 20; i++)
    html += "<div class='terminal-entry'>" + failureLog[i] + "</div>";
  return html;
}

/**
 * @brief   Converts a 6-byte MAC address to human-readable string
 * @param   mac - Pointer to 6-byte MAC array
 * @return  Formatted MAC string (e.g., "AA:BB:CC:DD:EE:FF")
 */
String macToString(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

/**
 * @brief   Converts MAC string to byte array
 * @param   macStr - Input MAC string (format: "AA:BB:CC:DD:EE:FF")
 * @param   mac    - Output 6-byte array
 * @return  true if parsing successful, false otherwise
 */
bool stringToMac(const String& macStr, uint8_t* mac) {
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)values[i];
    return true;
  }
  return false;
}

// ================================================================================================
// @section     EEPROM CONFIGURATION MANAGEMENT
// ================================================================================================

/**
 * @brief   Validates if stored configuration is intact and has sane values
 * @return  true if magic number matches and thresholds are valid
 */
bool isConfigValid() {
  return (config.magic == 0xDEADBEEF && 
          config.low_threshold >= 0 && 
          config.high_threshold <= 100 && 
          config.low_threshold < config.high_threshold);
}

/**
 * @brief   Saves current configuration to EEPROM
 */
void saveConfig() { 
  EEPROM.begin(512);
  config.magic = 0xDEADBEEF;
  EEPROM.put(0, config);
  EEPROM.commit();
  EEPROM.end();
}

/**
 * @brief   Loads configuration from EEPROM, applies defaults if invalid
 */
void loadConfig() { 
  EEPROM.begin(512);
  EEPROM.get(0, config);
  EEPROM.end();
  
  if (!isConfigValid()) {
    Config defaultConfig;
    config = defaultConfig;
    saveConfig();
  }
  
  // Sanitize all configuration values
  if (config.low_threshold >= config.high_threshold) { 
    config.low_threshold = 30.0; 
    config.high_threshold = 80.0; 
  }
  if (config.espnow_channel < 1 || config.espnow_channel > 13) config.espnow_channel = 1;
  if (config.sensor_timeout < 10000) config.sensor_timeout = SENSOR_HEARTBEAT_TIMEOUT;
  if (config.max_power_threshold < 10) config.max_power_threshold = 500.0;
  if (config.overload_cooldown_seconds < 0) config.overload_cooldown_seconds = 30;
  if (config.inrush_tolerance_ms < 100) config.inrush_tolerance_ms = 2000;
  if (strlen(config.override_password) == 0) strcpy(config.override_password, "1234");
}

/**
 * @brief   Performs factory reset by erasing EEPROM and restoring defaults
 */
void factoryReset() {
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++) EEPROM.write(i, 0xFF);
  EEPROM.commit();
  EEPROM.end();
  Config defaultConfig;
  config = defaultConfig;
  saveConfig();
  delay(1000);
  ESP.restart();
}

/**
 * @brief   Reboots the device after a short delay
 */
void rebootDevice() { delay(500); ESP.restart(); }

// ================================================================================================
// @section     POWER QUALITY CALCULATIONS
// ================================================================================================

/**
 * @brief   Updates power quality variables based on current S31 readings
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
    if (apparentPower > 0.01) powerFactor = realPower / apparentPower;
    else powerFactor = 0.0;
    
    if (powerFactor < 0) powerFactor = 0;
    if (powerFactor > 1) powerFactor = 1;
    
    float vaSquared = apparentPower * apparentPower;
    float wSquared = realPower * realPower;
    reactivePower = (vaSquared > wSquared) ? sqrt(vaSquared - wSquared) : 0;
  }
}

// ================================================================================================
// @section     SENSOR HEALTH MONITORING
// ================================================================================================

/**
 * @brief   Updates sensor health status based on time since last data reception
 */
void updateSensorHealth() {
  unsigned long now = millis();
  unsigned long timeSinceLastData = (lastEspNowData > 0) ? (now - lastEspNowData) : config.sensor_timeout + 10000;
  
  if (timeSinceLastData < ESP_NOW_DATA_TIMEOUT) {
    if (sensorIsDead) { 
      sensorIsDead = false; 
      sensorWarningIssued = false; 
      addFailureLogEntry("✅ Sensor RECOVERED"); 
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

// ================================================================================================
// @section     SENSOR FAILURE HANDLING
// ================================================================================================

/**
 * @brief   Determines pump state based on configured sensor failure strategy
 * @param   currentState - Current pump relay state (true=ON, false=OFF)
 * @return  Desired pump state for safety strategy
 */
bool handleSensorFailurePumpControl(bool currentState) {
  if (!sensorIsDead) return currentState;
  
  unsigned long now = millis();
  unsigned long timeSinceDead = now - sensorDeadStartTime;
  
  switch (config.sensor_failure_strategy) {
    case 0:  // Stop Pump - SAFEST
      if (currentState) addFailureLogEntry("🛑 Emergency STOP - Sensor failure");
      return false;
      
    case 1:  // Maintain State
      return currentState;
      
    case 2:  // Force ON (RISKY)
      if (timeSinceDead > config.sensor_timeout) { 
        addFailureLogEntry("⚠️ FORCE ON - Risk of overflow!"); 
        return true; 
      }
      return currentState;
      
    case 3: {  // Cyclic Mode
      if (cyclicLastSwitchTime == 0) { 
        cyclicLastSwitchTime = now; 
        cyclicPumpState = false; 
      }
      unsigned long cycleDuration = cyclicPumpState ? config.cyclic_on_duration : config.cyclic_off_duration;
      if (now - cyclicLastSwitchTime >= cycleDuration) {
        cyclicPumpState = !cyclicPumpState;
        cyclicLastSwitchTime = now;
        addFailureLogEntry("🔄 Cyclic: Pump " + String(cyclicPumpState ? "ON" : "OFF"));
      }
      if (cyclicPumpState && (now - sensorDeadStartTime) > config.sensor_timeout) {
        addFailureLogEntry("⚠️ Cyclic emergency stop - timeout exceeded");
        return false;
      }
      return cyclicPumpState;
    }
      
    default: 
      return false;
  }
}

// ================================================================================================
// @section     OVERRIDE FUNCTIONS (Sensor Bypass Mode)
// @brief       Password-protected manual pump control when sensor is dead
// ================================================================================================

/**
 * @brief   Checks if override password is valid
 * @param   password - Input password string
 * @return  true if password matches configured password
 */
bool isOverridePasswordValid(String password) {
  return password == String(config.override_password);
}

/**
 * @brief   Activates manual override mode with password
 * @param   password - Password for authentication
 * @param   state - Desired pump state ("on", "off", or "toggle")
 * @return  String response message
 */
String activateOverride(String password, String state) {
  if (!isOverridePasswordValid(password)) {
    addFailureLogEntry("❌ Override failed: Invalid password attempt from IP: " + server.client().remoteIP().toString());
    return "ERROR: Invalid password";
  }
  
  config.override_active = true;
  config.sensor_bypass_mode = true;
  overrideStartTime = millis();
  
  if (state == "on") {
    config.manual_override_state = true;
    s31.setRelay(true);
    addFailureLogEntry("🔓 OVERRIDE ACTIVE: Pump forced ON (Password authorized)");
    return "OVERRIDE ACTIVE: Pump turned ON (will auto-expire in 1 hour)";
  } 
  else if (state == "off") {
    config.manual_override_state = false;
    s31.setRelay(false);
    addFailureLogEntry("🔓 OVERRIDE ACTIVE: Pump forced OFF (Password authorized)");
    return "OVERRIDE ACTIVE: Pump turned OFF (will auto-expire in 1 hour)";
  }
  else if (state == "toggle") {
    config.manual_override_state = !s31.getRelayState();
    s31.setRelay(config.manual_override_state);
    addFailureLogEntry("🔓 OVERRIDE ACTIVE: Pump toggled to " + String(config.manual_override_state ? "ON" : "OFF"));
    return "OVERRIDE ACTIVE: Pump toggled to " + String(config.manual_override_state ? "ON" : "OFF");
  }
  
  return "ERROR: Invalid state. Use 'on', 'off', or 'toggle'";
}

/**
 * @brief   Deactivates override mode and returns to normal operation
 * @param   password - Password for authentication
 * @return  String response message
 */
String deactivateOverride(String password) {
  if (!isOverridePasswordValid(password)) {
    addFailureLogEntry("❌ Override deactivation failed: Invalid password");
    return "ERROR: Invalid password";
  }
  
  config.override_active = false;
  config.sensor_bypass_mode = false;
  addFailureLogEntry("🔒 OVERRIDE DEACTIVATED: Returning to normal operation");
  return "OVERRIDE DEACTIVATED: Normal operation resumed";
}

/**
 * @brief   Checks if override should auto-expire after timeout period
 */
void checkOverrideTimeout() {
  if (config.override_active && (millis() - overrideStartTime) > OVERRIDE_TIMEOUT_MS) {
    config.override_active = false;
    config.sensor_bypass_mode = false;
    addFailureLogEntry("⏰ Override auto-expired after 1 hour for safety");
  }
}

// ================================================================================================
// @section     INTELLIGENT OVERLOAD PROTECTION (WITH INRUSH HANDLING)
// ================================================================================================

/**
 * @brief   Checks for overload condition with intelligent inrush current handling
 * @param   currentPower - Current pump power consumption in Watts
 * @param   currentState - Current pump relay state (true=ON, false=OFF)
 * @return  true if pump is allowed to run, false if overload protection triggered
 */
bool checkOverloadProtection(float currentPower, bool currentState) {
  unsigned long now = millis();
  
  // ==============================================================================================
  // DETECT PUMP START EVENTS (RISING EDGE DETECTION)
  // ==============================================================================================
  if (currentState && !overload.lastRelayState) {
    overload.pumpStartTime = now;
    overload.waitingForStableOperation = true;
    addFailureLogEntry("🚀 Pump STARTED - Inrush tolerance active for " + 
                       String(config.inrush_tolerance_ms) + "ms");
  }
  overload.lastRelayState = currentState;
  
  // ==============================================================================================
  // INRUSH TOLERANCE PERIOD - IGNORE HIGH CURRENT
  // ==============================================================================================
  if (overload.waitingForStableOperation && currentState) {
    unsigned long runtime = now - overload.pumpStartTime;
    if (runtime < config.inrush_tolerance_ms) {
      if (currentPower > config.max_power_threshold * 2) {
        addFailureLogEntry("📈 INRUSH CURRENT: " + String(currentPower, 1) + 
                           "W (" + String(runtime) + "ms) - Normal for startup, protection bypassed");
      }
      return true;
    } else {
      overload.waitingForStableOperation = false;
      addFailureLogEntry("✅ Inrush period ended - Normal overload monitoring active");
    }
  }
  
  // ==============================================================================================
  // COOLDOWN PERIOD CHECK
  // ==============================================================================================
  if (overload.cooldownUntil > now) {
    unsigned long remaining = (overload.cooldownUntil - now) / 1000;
    if (currentState) {
      s31.setRelay(false);
      if (remaining > 0 && remaining % 5 == 0) {
        addFailureLogEntry("⏱️ Cooldown active: " + String(remaining) + "s remaining - Pump blocked");
      }
    }
    return false;
  }
  
  if (overload.active && now >= overload.cooldownUntil) {
    overload.active = false;
    addFailureLogEntry("✅ Cooldown period ended - Normal operation resumed");
  }
  
  // ==============================================================================================
  // SUSTAINED OVERLOAD DETECTION
  // ==============================================================================================
  if (config.pump_load_protection_enabled && currentState && currentPower > config.max_power_threshold) {
    unsigned long runtime = now - overload.pumpStartTime;
    
    if (runtime < config.inrush_tolerance_ms) {
      addFailureLogEntry("📈 Inrush current detected: " + String(currentPower, 1) + 
                         "W (normal for startup) - Protection bypassed");
      return true;
    }
    
    if (!overload.active) {
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
      addFailureLogEntry("║  Cooldown: " + String(config.overload_cooldown_seconds) + " seconds");
      addFailureLogEntry("║  Overload Event #" + String(overload.overloadCount));
      addFailureLogEntry("║  ACTION: IMMEDIATE POWER CUT - Pump STOPPED for safety");
      addFailureLogEntry("╚════════════════════════════════════════════════════════════════╝");
      
      s31.setRelay(false);
      return false;
    }
    return false;
  }
  
  return true;
}

// ================================================================================================
// @section     PUMP CONTROL LOGIC (WITH OVERRIDE SUPPORT)
// ================================================================================================

/**
 * @brief   Main pump control routine with intelligent overload protection AND OVERRIDE
 * @note    Override mode takes precedence over all other control logic
 * @note    Overload protection still works even in override mode
 */
void controlPump() {
  updateSensorHealth();
  updatePowerQuality();
  
  if (millis() - lastControlCheck < CONTROL_INTERVAL) return;
  lastControlCheck = millis();
  
  // Check if override should auto-expire
  checkOverrideTimeout();
  
  bool currentState = s31.getRelayState();
  float currentPower = s31.getPower();
  
  // ==============================================================================================
  // ★ INTELLIGENT OVERLOAD PROTECTION - HIGHEST PRIORITY ★
  // ==============================================================================================
  bool overloadAllowed = checkOverloadProtection(currentPower, currentState);
  
  if (!overloadAllowed) {
    if (config.override_active) {
      addFailureLogEntry("⚠️ Overload protection overrides manual override - Pump stopped");
      config.manual_override_state = false;
    }
    return;
  }
  
  // ==============================================================================================
  // ★ OVERRIDE MODE - Manual control with password (BYPASSES ALL SENSOR LOGIC) ★
  // ==============================================================================================
  if (config.override_active || config.sensor_bypass_mode) {
    if (config.manual_override_state != currentState) {
      s31.setRelay(config.manual_override_state);
    }
    return;
  }
  
  // ==============================================================================================
  // MANUAL MODE HANDLING
  // ==============================================================================================
  if (!config.auto_mode) {
    if (sensorIsDead && config.sensor_failure_strategy == 0) {
      if (s31.getRelayState()) {
        s31.setRelay(false);
        addFailureLogEntry("🚨 MANUAL MODE: Emergency stop - Sensor dead");
      }
    }
    return;
  }
  
  // ==============================================================================================
  // AUTO MODE - Water level based control
  // ==============================================================================================
  bool shouldPumpOn = false;
  bool dataValid = (espnow_initialized && espnowDataValid && 
                    millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
  
  if (!dataValid || sensorIsDead) {
    shouldPumpOn = handleSensorFailurePumpControl(currentState);
  } else {
    if (currentWaterLevel <= config.low_threshold) {
      shouldPumpOn = true;
    } else if (currentWaterLevel >= config.high_threshold) {
      shouldPumpOn = false;
    } else {
      shouldPumpOn = currentState;
    }
  }
  
  // Pump statistics tracking
  if (currentState && !pumpStats.wasRunning) {
    pumpStats.pumpCycles++;
    pumpStats.lastPumpOnTime = millis();
    pumpStats.wasRunning = true;
    pumpStats.totalEnergyKwh = s31.getEnergy();
    
    unsigned long hours = pumpStats.totalRuntimeSeconds / 3600;
    unsigned long minutes = (pumpStats.totalRuntimeSeconds % 3600) / 60;
    snprintf(pumpStats.lastStartStr, sizeof(pumpStats.lastStartStr), "%02lu:%02lu", hours, minutes);
  } else if (!currentState && pumpStats.wasRunning) {
    pumpStats.totalRuntimeSeconds += (millis() - pumpStats.lastPumpOnTime) / 1000;
    pumpStats.wasRunning = false;
  }
  
  // Dry run protection
  if (currentState && currentPower < config.min_power_threshold && !sensorIsDead) {
    unsigned long runtime = (millis() - pumpStats.lastPumpOnTime) / 1000;
    if (runtime > max(config.pump_protection_time, config.inrush_tolerance_ms / 1000)) {
      shouldPumpOn = false;
      addFailureLogEntry("💧 Dry run protection - Pump stopped (power: " + 
                         String(currentPower, 1) + "W)");
    }
  }
  
  if (shouldPumpOn != currentState) {
    s31.setRelay(shouldPumpOn);
  }
}

// ================================================================================================
// @section     ESP-NOW COMMUNICATION
// ================================================================================================

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

String getEspNowLogHTML() {
  if (espnow_log.empty()) return "<div class='terminal-entry'>No ESP-NOW messages received yet...</div>";
  
  String html = "";
  for (size_t i = 0; i < espnow_log.size(); i++) {
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
  if (!config.use_espnow) { 
    espnow_initialized = false; 
    return; 
  }
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  
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
  
  outgoing.seq = espnow_seq++;
  outgoing.timestamp = micros();
  strncpy(outgoing.msg, command, sizeof(outgoing.msg)-1);
  outgoing.msg[sizeof(outgoing.msg)-1] = '\0';
  
  uint8_t* peerMac = broadcastMac;
  if (config.peer_mac[0] != 0xFF && config.peer_mac[0] != 0x00) peerMac = config.peer_mac;
  
  esp_now_send(peerMac, (uint8_t *)&outgoing, sizeof(outgoing));
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
// @section     SYSTEM SETUP
// ================================================================================================

void setupAPMode() {
  String apSSID = "SmartPump-" + String(ESP.getChipId() & 0xFFFF, HEX);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID.c_str(), "12345678");
}

void setupArduinoOTA() {
  if (!ENABLE_OTA) return;
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  ArduinoOTA.onError([](ota_error_t error) {});
  ArduinoOTA.begin();
}

// ================================================================================================
// @section     WEB SERVER ENDPOINTS
// ================================================================================================

void setupWebServer() {
  // UI Pages
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", simple_html); });
  server.on("/engmode", HTTP_GET, []() { server.send_P(200, "text/html", engineering_html); });
  
  // Real-time data endpoint
  server.on("/data", HTTP_GET, []() {
    updatePowerQuality();
    updateSensorHealth();
    
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
    doc["autoMode"] = config.auto_mode;
    doc["espnowActive"] = espnow_initialized;
    doc["espnowDataValid"] = espnowDataValid;
    doc["espnowMsgCount"] = espnow_msg_counter;
    doc["lastSeq"] = espnow_seq;
    doc["sensorHealthy"] = !sensorIsDead && (millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT);
    doc["sensorWarning"] = !sensorIsDead && (millis() - lastEspNowData >= ESP_NOW_DATA_TIMEOUT);
    
    // Overload protection status
    doc["overloadProtectionActive"] = overload.active;
    doc["inInrushPeriod"] = overload.waitingForStableOperation;
    
    if (overload.waitingForStableOperation) {
      unsigned long elapsed = millis() - overload.pumpStartTime;
      if (config.inrush_tolerance_ms > elapsed) {
        doc["inrushRemaining"] = config.inrush_tolerance_ms - elapsed;
      } else {
        doc["inrushRemaining"] = 0;
      }
    } else {
      doc["inrushRemaining"] = 0;
    }
    
    doc["inCooldown"] = (millis() < overload.cooldownUntil);
    if (millis() < overload.cooldownUntil) {
      doc["cooldownRemaining"] = (overload.cooldownUntil - millis()) / 1000;
    } else {
      doc["cooldownRemaining"] = 0;
    }
    doc["overloadCount"] = overload.overloadCount;
    
    // Override status
    doc["override_active"] = config.override_active;
    doc["sensor_bypass_mode"] = config.sensor_bypass_mode;
    doc["manual_override_state"] = config.manual_override_state;
    doc["override_time_remaining"] = config.override_active ? 
      (OVERRIDE_TIMEOUT_MS - (millis() - overrideStartTime)) / 1000 : 0;
    doc["override_password"] = String(config.override_password);
    
    if (lastEspNowData > 0) doc["sensorLastSeen"] = String((millis() - lastEspNowData)/1000) + "s ago";
    else doc["sensorLastSeen"] = "Never";
    
    if (lastEspNowData > 0) doc["espnowLastTime"] = String((millis() - lastEspNowData)/1000) + "s ago";
    else doc["espnowLastTime"] = "Never";
    
    if (espnow_initialized && espnowDataValid && millis() - lastEspNowData < ESP_NOW_DATA_TIMEOUT) 
      doc["dataSource"] = "ESP-NOW";
    else 
      doc["dataSource"] = "No Data";
    
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
    if (config.override_active) reason = "🔓 OVERRIDE MODE - Manual control active";
    else if (!config.auto_mode) reason = "Manual Control";
    else if (overload.active) reason = "🛡️ OVERLOAD PROTECTION - Pump stopped";
    else if (millis() < overload.cooldownUntil) reason = "⏱️ Cooldown period - " + 
            String((overload.cooldownUntil - millis())/1000) + "s remaining";
    else if (overload.waitingForStableOperation) reason = "⚡ Pump starting - Inrush tolerance active";
    else if (s31.getRelayState()) reason = "Water level low (" + String(currentWaterLevel,1) + "%)";
    else if (currentWaterLevel >= config.high_threshold) reason = "Tank full";
    else reason = "Level OK";
    doc["pumpReason"] = reason;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // Configuration endpoints
  server.on("/config", HTTP_GET, []() {
    StaticJsonDocument<512> doc;
    doc["low_threshold"] = config.low_threshold;
    doc["high_threshold"] = config.high_threshold;
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
    doc["override_password"] = String(config.override_password);
    
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
        stringToMac(mac, config.peer_mac); 
        strcpy(config.peer_mac_str, mac.c_str()); 
      }
      if (doc.containsKey("espnow_channel")) config.espnow_channel = doc["espnow_channel"];
      saveConfig();
      server.send(200, "text/plain", "OK");
      delay(500);
      ESP.restart();
    }
  });
  
  server.on("/config/safety", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("sensor_timeout")) config.sensor_timeout = doc["sensor_timeout"].as<unsigned long>() * 1000;
      if (doc.containsKey("failure_strategy")) config.sensor_failure_strategy = doc["failure_strategy"];
      if (doc.containsKey("cyclic_on_duration")) config.cyclic_on_duration = doc["cyclic_on_duration"].as<unsigned long>() * 1000;
      if (doc.containsKey("cyclic_off_duration")) config.cyclic_off_duration = doc["cyclic_off_duration"].as<unsigned long>() * 1000;
      saveConfig();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/pump", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("low_threshold")) config.low_threshold = doc["low_threshold"];
      if (doc.containsKey("high_threshold")) config.high_threshold = doc["high_threshold"];
      if (doc.containsKey("min_power")) config.min_power_threshold = doc["min_power"];
      if (doc.containsKey("dry_run_protection")) config.pump_protection_time = doc["dry_run_protection"];
      if (doc.containsKey("pump_load_protection_enabled")) config.pump_load_protection_enabled = doc["pump_load_protection_enabled"];
      if (doc.containsKey("max_power_threshold")) config.max_power_threshold = doc["max_power_threshold"];
      if (doc.containsKey("inrush_tolerance_ms")) config.inrush_tolerance_ms = doc["inrush_tolerance_ms"];
      if (doc.containsKey("overload_cooldown_seconds")) config.overload_cooldown_seconds = doc["overload_cooldown_seconds"];
      if (doc.containsKey("override_password")) {
        String pwd = doc["override_password"].as<String>();
        if (pwd.length() > 0 && pwd.length() <= 4) {
          strcpy(config.override_password, pwd.c_str());
        }
      }
      saveConfig();
      server.send(200, "text/plain", "OK");
    }
  });
  
  server.on("/config/system", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      StaticJsonDocument<512> doc;
      deserializeJson(doc, server.arg("plain"));
      if (doc.containsKey("hostname")) deviceName = doc["hostname"].as<String>();
      saveConfig();
      server.send(200, "text/plain", "OK");
      delay(500);
      ESP.restart();
    }
  });
  
  // ==============================================================================================
  // OVERRIDE ENDPOINTS (NEW in v4.4.0)
  // ==============================================================================================
  server.on("/override", HTTP_GET, []() {
    if (server.hasArg("password") && server.hasArg("state")) {
      String password = server.arg("password");
      String state = server.arg("state");
      String response = activateOverride(password, state);
      server.send(200, "text/plain", response);
    } else {
      server.send(400, "text/plain", "Usage: /override?password=1234&state=on|off|toggle");
    }
  });
  
  server.on("/override/off", HTTP_GET, []() {
    if (server.hasArg("password")) {
      String password = server.arg("password");
      String response = deactivateOverride(password);
      server.send(200, "text/plain", response);
    } else {
      server.send(400, "text/plain", "Usage: /override/off?password=1234");
    }
  });
  
  server.on("/override/status", HTTP_GET, []() {
    StaticJsonDocument<256> doc;
    doc["override_active"] = config.override_active;
    doc["sensor_bypass_mode"] = config.sensor_bypass_mode;
    doc["manual_override_state"] = config.manual_override_state;
    doc["override_time_remaining"] = config.override_active ? 
      (OVERRIDE_TIMEOUT_MS - (millis() - overrideStartTime)) / 1000 : 0;
    doc["sensor_dead"] = sensorIsDead;
    doc["sensor_bypass_available"] = true;
    doc["override_password"] = String(config.override_password);
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  // ESP-NOW management endpoints
  server.on("/espnow/request", HTTP_GET, []() { manualRequestSensorData(); });
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
  
  // Failure log endpoints
  server.on("/failurelog", HTTP_GET, []() { server.send(200, "text/html", getFailureLogHTML()); });
  server.on("/failurelog/clear", HTTP_POST, []() { failureLog.clear(); server.send(200, "text/plain", "OK"); });
  
  // Pump control endpoints
  server.on("/mode", HTTP_GET, []() { 
    if(server.hasArg("mode")){ 
      config.auto_mode = (server.arg("mode") == "auto"); 
      saveConfig(); 
      if(!config.auto_mode) s31.setRelay(false); 
    } 
    server.send(200,"text/plain","OK"); 
  });
  
  server.on("/toggle", HTTP_GET, []() { 
    if(!config.auto_mode && !config.override_active) {
      if (millis() < overload.cooldownUntil) {
        server.send(403, "text/plain", "Pump in cooldown period");
        return;
      }
      s31.toggleRelay(); 
    } else if (config.override_active) {
      server.send(403, "text/plain", "Cannot toggle in override mode. Use override API.");
      return;
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
    String response; 
    serializeJson(doc,response); 
    server.send(200,"application/json",response); 
  });
  
  server.on("/resetstats", HTTP_GET, []() { 
    pumpStats.totalRuntimeSeconds = 0; 
    pumpStats.totalEnergyKwh = 0; 
    pumpStats.pumpCycles = 0; 
    overload.overloadCount = 0;
    server.send(200,"text/plain","OK"); 
  });
  
  server.on("/exportstats", HTTP_GET, []() { 
    String csv="Timestamp,Runtime(min),Energy(kWh),Cycles,LastStart,OverloadEvents\n"; 
    csv+=String(millis()/1000)+","+String(pumpStats.totalRuntimeSeconds/60)+","+String(pumpStats.totalEnergyKwh)+","+String(pumpStats.pumpCycles)+","+String(pumpStats.lastStartStr)+","+String(overload.overloadCount)+"\n"; 
    server.send(200,"text/csv",csv); 
  });
  
  // System endpoints
  server.on("/factoryreset", HTTP_GET, []() { 
    server.send(200,"text/plain","Factory resetting..."); 
    delay(100); 
    factoryReset(); 
  });
  
  server.on("/reboot", HTTP_GET, []() { 
    server.send(200,"text/plain","Rebooting..."); 
    delay(100); 
    rebootDevice(); 
  });
  
  server.on("/info", HTTP_GET, []() { 
    String json = "{\"mac\":\"" + WiFi.macAddress() + "\",\"ap_ssid\":\"" + WiFi.softAPSSID() + "\",\"ip\":\"192.168.4.1\"}"; 
    server.send(200,"application/json",json); 
  });
}

// ================================================================================================
// @section     ARDUINO SETUP & LOOP
// ================================================================================================

void setup() {
  loadConfig();
  uint32_t chipId = ESP.getChipId();
  deviceName = "s31-pump-" + String(chipId & 0xFFFF, HEX);
  
  s31.begin();
  setupAPMode();
  initEspNow();
  setupWebServer();
  setupArduinoOTA();
  
  if (ENABLE_MDNS) {
    MDNS.begin(deviceName.c_str());
    MDNS.addService("http", "tcp", 80);
  }
  
  server.begin();
}

void loop() {
  unsigned long currentMillis = millis();
  
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
  
  delay(1);
}