/**
 * ================================================================================================
 * POWR316D - Professional Energy Monitoring & Control System
 * ================================================================================================
 * 
 * @project     POWR316D Energy Monitor
 * @version     1.0
 * @author      Smart Energy Solutions
 * @date        2024
 * @license     MIT
 * @repository  https://github.com/yourusername/POWR316D-Energy-Monitor
 * 
 * @section DESCRIPTION
 * Complete energy monitoring and control system for POWR316D hardware platform.
 * Features real-time power monitoring using CSE7759B energy meter IC,
 * TM1621 LCD display driver, relay control, and web-based management interface.
 * 
 * @section HARDWARE_REQUIREMENTS
 * - ESP32-WROOM-32 Microcontroller
 * - TM1621 LCD Driver (4-digit 7-segment display)
 * - CSE7759B Energy Meter IC (UART interface)
 * - 5V Relay Module for load control (active high)
 * - Tactile Button on GPIO0 for manual relay toggle
 * - Status LEDs (WiFi and System)
 * 
 * @section CSE7759B_PROTOCOL Energy Meter Communication Protocol
 * 
 * The CSE7759B is a high-precision energy metering IC that measures voltage,
 * current, and power. It communicates via UART at 4800 baud, 8 data bits,
 * even parity, 1 stop bit (8E1).
 * 
 * ┌────────────────────────────────────────────────────────────────────────────┐
 * │                        CSE7759B 24-Byte Packet Structure                   │
 * ├─────┬─────────┬──────────────┬──────────────┬──────────────┬──────┬────────┤
 * │Byte │   0     │      1       │    2-4       │    5-7       │ 8-10 │ 11-13  │
 * ├─────┼─────────┼──────────────┼──────────────┼──────────────┼──────┼────────┤
 * │Field│ 0x55    │   Status     │  Voltage     │  Voltage     │Curr  │Current │
 * │     │ Header  │   Flags      │  Coefficient │    Cycle     │Coeff │ Cycle  │
 * ├─────┼─────────┼──────────────┼──────────────┼──────────────┼──────┼────────┤
 * │Byte │ 14-16   │   17-19      │   20-21      │     22       │  23  │        │
 * ├─────┼─────────┼──────────────┼──────────────┼──────────────┼──────┼────────┤
 * │Field│ Power   │    Power     │    Adj.      │   CF High    │  CF  │Checksum│
 * │     │ Coeff   │    Cycle     │              │              │ Low  │        │
 * └─────┴─────────┴──────────────┴──────────────┴──────────────┴──────┴────────┘
 * 
 * Calculations:
 * - Voltage (V) = Voltage_Coefficient / Voltage_Cycle
 * - Current (A) = Current_Coefficient / Current_Cycle
 * - Power (W)   = Power_Coefficient / Power_Cycle
 * - Checksum    = Sum of bytes 2-22 (mod 256)
 * 
 * @section TM1621_LCD_DRIVER LCD Display Controller
 * 
 * The TM1621 is a 32-segment LCD driver with 4 common terminals.
 * It communicates via a 3-wire serial interface (DATA, WR, CS).
 * 
 * Display Buffer Layout (8 bytes total):
 * ┌────────────────────────────────────────────────────────────────────────────┐
 * │                           LCD Buffer Memory Map                            │
 * ├──────────┬─────────────────────────────────────────────────────────────────┤
 * │ Buffer 0 │ Top Row - Digit 1 (Thousands)                                   │
 * │ Buffer 1 │ Top Row - Digit 2 (Hundreds)                                    │
 * │ Buffer 2 │ Top Row - Digit 3 (Tens) + Top Decimal Point (Bit 7 = 0x80)     │
 * │ Buffer 3 │ Top Row - Digit 4 (Units) + °C Symbol (Bit 7 = 0x80)            │
 * ├──────────┼─────────────────────────────────────────────────────────────────┤
 * │ Buffer 4 │ Bottom Row - Digit 4 (Units) + W/kWh Symbol (Bit 3 = 0x08)      │
 * │ Buffer 5 │ Bottom Row - Digit 3 (Tens) + Bottom Decimal Point (Bit 3=0x08) │
 * │ Buffer 6 │ Bottom Row - Digit 2 (Hundreds) + %RH Symbol (Bit 3 = 0x08)     │
 * │ Buffer 7 │ Bottom Row - Digit 1 (Thousands) + V/A Symbol (Bit 3 = 0x08)    │
 * └──────────┴─────────────────────────────────────────────────────────────────┘
 * 
 * 7-Segment Display Pattern (Common Cathode):
 *      A (bit6)
 *     ┌───┐
 *   F │   │ B (bit5)
 *     │ G │
 *     └───┘
 *   E │   │ C (bit4)
 *     │   │
 *     └───┘  • DP (bit7)
 *      D (bit3)
 * 
 * @section DISPLAY_FORMATTING_RULES 4-Digit Display with Leading Zeros
 * 
 * Voltage:
 * - 220V     → "0220V"  (4 digits, no decimal)
 * - 221.3V   → "221.3V" (decimal on 3rd digit)
 * 
 * Current:
 * - 3A       → "0003A"  (4 digits, no decimal)
 * - 2.2A     → "002.2A" (1 decimal, leading zeros)
 * - 15.5A    → "155.5A" (1 decimal, 3 digits)
 * 
 * Power (v1.0):
 * - 8.5W     → "008.5W" (1 decimal, leading zeros)
 * - 75.3W    → "075.3W" (1 decimal, leading zero)
 * - 123.2W   → "123.2W" (1 decimal, no leading zero)
 * - 958W     → "0958W"  (integer, leading zero)
 * - 1500W    → "1500W"  (integer, 4 digits)
 * - 9999W    → "9999W"  (max display)
 * 
 * Energy:
 * - 5.6kWh   → "005.6kWh" (1 decimal, leading zeros)
 * - 12.34kWh → "012.3kWh" (4 digits, 1 decimal)
 * - 100kWh   → "0100kWh"  (integer, leading zero)
 * - 1500kWh  → "1500kWh"  (integer)
 * 
 * @section FEATURES
 * - Real-time Voltage, Current, Power, Power Factor monitoring
 * - Energy accumulation (kWh) with non-volatile storage
 * - Calibration support for all measurements
 * - Web-based control panel (offline-capable via captive portal)
 * - Manual GPIO0 button for relay control (with debouncing)
 * - Manual LCD bit tester for hardware debugging
 * - OTA (Over-The-Air) firmware updates
 * - Persistent storage (WiFi credentials, calibration, energy data)
 * - Automatic screen cycling (Voltage/Current ↔ Energy/Power)
 * - Symbol control (show/hide V/A, W/kWh symbols for testing)
 * 
 * @section WEB_ACCESS
 * Access Point Mode (Default - No WiFi configuration needed):
 * - SSID:     "POWR316D_AP"
 * - Password: "12345678"
 * - URL:      http://192.168.4.1
 * 
 * Client Mode (After Configuration):
 * - Connect to your WiFi network
 * - Check serial monitor or router DHCP for IP address
 * - Access via web browser
 * 
 * @section BUTTON_CONTROL
 * GPIO0 Button (connected to ground, internal pull-up enabled):
 * - Short press (< 2 seconds): Toggle relay ON/OFF
 * - Debounced to prevent false triggers
 * - Visual feedback via relay LED indicator
 * 
 * @section CALIBRATION_PROCEDURE
 * 1. Connect a known accurate multimeter in parallel with load
 * 2. Read actual values from multimeter
 * 3. Enter values in Calibration tab of web interface
 * 4. System calculates and stores multipliers automatically
 * 
 * Formula: Multiplier = Actual_Value / Raw_Value
 * 
 * @section DEPENDENCIES
 * - Arduino core for ESP32 (version 2.0.14 or newer)
 * - ArduinoJson library (version 6.19.4 or newer)
 * - Built-in libraries: WiFi, WebServer, DNSServer, LittleFS, ArduinoOTA
 * 
 * @section INSTALLATION
 * 1. Install ESP32 board support in Arduino IDE
 * 2. Install ArduinoJson library via Library Manager
 * 3. Connect hardware according to pin definitions
 * 4. Select ESP32 Dev Module board
 * 5. Set Flash Size to 4MB, Partition Scheme to "Default 4MB with spiffs"
 * 6. Upload code to ESP32
 * 7. Connect to "POWR316D_AP" WiFi network
 * 8. Open browser to http://192.168.4.1
 * 
 * @section TROUBLESHOOTING
 * - LCD not working: Check 5V power to TM1621 module
 * - No energy data: Verify CSE7759B TX connected to GPIO16
 * - WiFi issues: Check AP password is "12345678"
 * - Relay not toggling: Verify GPIO13 connection and 5V supply
 * 
 * @section SAFETY_WARNING
 * ⚠️ WARNING: This device handles AC mains voltage (110V/230V)!
 * - Only qualified personnel should install and maintain
 * - Always disconnect power before wiring
 * - Use proper insulation and enclosure
 * - Follow local electrical codes
 * - Use appropriate fuses and protection
 * - Keep away from children and moisture
 * 
 * ================================================================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <math.h>

/* ================================================================================================
 * VERSION & FEATURE CONFIGURATION
 * ================================================================================================ */

#define FIRMWARE_VERSION       "1.0"           /**< Current firmware version */
#define FIRMWARE_DATE          "2024-12-01"    /**< Release date */
#define ENABLE_CAPTIVE_PORTAL   true            /**< Enable DNS captive portal for easy setup */
#define ENABLE_OTA              true            /**< Enable Over-The-Air updates */
#define ENABLE_SERIAL_DEBUG     true            /**< Enable serial debug output */

/* ================================================================================================
 * SYSTEM TIMING CONSTANTS (milliseconds)
 * ================================================================================================ */

#define WIFI_CONNECT_TIMEOUT    15000UL         /**< WiFi connection timeout (15 seconds) */
#define LCD_UPDATE_INTERVAL     3000UL          /**< LCD screen cycling interval (3 seconds) */
#define ENERGY_UPDATE_INTERVAL   1000UL         /**< Energy accumulation interval (1 second) */
#define DEBOUNCE_DELAY           50UL           /**< Button debounce delay (50ms) */
#define BUTTON_HOLD_TIME        2000UL          /**< Button hold time for future features (2 seconds) */
#define SERIAL_BAUD_RATE        115200UL        /**< Serial debug baud rate */

/* ================================================================================================
 * NETWORK CONFIGURATION
 * ================================================================================================ */

const char* AP_SSID           = "POWR316D_AP";        /**< Access Point SSID (max 32 chars) */
const char* AP_PASSWORD       = "12345678";           /**< Access Point password (min 8 chars) */
const IPAddress AP_IP         (192, 168, 4, 1);       /**< Access Point static IP */
const IPAddress AP_SUBNET     (255, 255, 255, 0);     /**< Access Point subnet mask */

#define DEFAULT_WIFI_ENABLED   false                  /**< WiFi client disabled by default */
#define DEFAULT_WIFI_SSID      "YourWiFi"             /**< Default WiFi SSID placeholder */
#define DEFAULT_WIFI_PASSWORD  "YourPassword"         /**< Default WiFi password placeholder */

/* ================================================================================================
 * HARDWARE PIN DEFINITIONS
 * ================================================================================================ */

#define PIN_BUTTON             0     /**< Button input pin (GPIO0, active LOW, internal pull-up) */
#define PIN_RELAY             13    /**< Relay control pin (active HIGH to turn ON) */
#define PIN_WIFI_LED           5     /**< WiFi status LED (active HIGH) */
#define PIN_STATUS_LED        18    /**< System status LED (active HIGH) */
#define PIN_CSE_RX            16    /**< CSE7759B UART RX pin (connect to TX of CSE7759B) */

/* ================================================================================================
 * LCD PIN DEFINITIONS (TM1621 Driver)
 * ================================================================================================ */

#define PIN_LCD_DATA          14    /**< TM1621 serial data pin (DIO) */
#define PIN_LCD_WR            27    /**< TM1621 write clock pin (WR) */
#define PIN_LCD_RD            26    /**< TM1621 read clock pin (RD) - not used, set HIGH */
#define PIN_LCD_CS            25    /**< TM1621 chip select pin (CS) */

/* ================================================================================================
 * LCD BIT MASKS - DISCOVERED HARDWARE MAPPING (via Bit Tester Tool)
 * ================================================================================================ */

/**
 * @brief TM1621 LCD Bit Mapping
 * 
 * These masks control specific LCD segments. Use the Bit Tester in the web interface
 * to discover mapping for different hardware revisions.
 */
#define LCD_TOP_DECIMAL       0x80    /**< Top line decimal point: Buffer[2] bit 7 */
#define LCD_BOTTOM_DECIMAL    0x08    /**< Bottom line decimal point: Buffer[5] bit 3 */
#define LCD_VA_SYMBOL         0x08    /**< V/A symbol (Voltage/Amps): Buffer[7] bit 3 (top line) */
#define LCD_WKWH_SYMBOL       0x08    /**< W/kWh symbol (Watts/kiloWatt-hour): Buffer[4] bit 3 (bottom line) */
#define LCD_PERCENT_SYMBOL    0x08    /**< %RH symbol (Humidity): Buffer[6] bit 3 */
#define LCD_CELSIUS_SYMBOL    0x80    /**< °C symbol (Temperature): Buffer[3] bit 7 */

/* ================================================================================================
 * CSE7759B ENERGY METER CONFIGURATION
 * ================================================================================================ */

#define CSE_BAUD              4800    /**< CSE7759B UART baud rate (fixed by hardware) */
#define CSE_UART_CONFIG       SERIAL_8E1  /**< 8 data bits, Even parity, 1 stop bit */
#define CSE_PACKET_LEN        24      /**< Fixed packet length in bytes */
#define CSE_HEADER1           0x55    /**< First header byte (always 0x55) */
#define CSE_HEADER2           0x5A    /**< Second header byte (always 0x5A) - not used in checksum */
#define CSE_MIN_VALID_CYCLE   100     /**< Minimum valid cycle count (prevents division by zero) */

/* ================================================================================================
 * VALIDATION LIMITS (for sanity checking measured values)
 * ================================================================================================ */

#define MIN_VOLTAGE           80.0f   /**< Minimum valid voltage (V) - below this indicates error */
#define MAX_VOLTAGE           300.0f  /**< Maximum valid voltage (V) - above this indicates error */
#define MIN_CURRENT           0.0f    /**< Minimum valid current (A) */
#define MAX_CURRENT           100.0f  /**< Maximum valid current (A) - above this indicates error */
#define MIN_POWER             0.0f    /**< Minimum valid power (W) */
#define MAX_POWER             25000.0f /**< Maximum valid power (W) - 25kW max for typical loads */

/* ================================================================================================
 * DEFAULT CALIBRATION MULTIPLIERS
 * ================================================================================================ */

/**
 * @brief Calibration multipliers
 * 
 * These values adjust raw readings from CSE7759B to match actual measurements.
 * Formula: Actual = Raw × Multiplier
 * Default 1.0 means no adjustment.
 */
#define DEFAULT_VOLTAGE_MULTIPLIER    1.0f    /**< Voltage calibration factor */
#define DEFAULT_CURRENT_MULTIPLIER    1.0f    /**< Current calibration factor */
#define DEFAULT_POWER_MULTIPLIER      1.0f    /**< Power calibration factor */

/* ================================================================================================
 * CSE7759B PACKET OFFSETS
 * ================================================================================================ */

/**
 * @brief CSE7759B data packet byte offsets
 * 
 * The energy meter sends 24 bytes of data. These constants define the position
 * of each value within the packet for easy access.
 */
enum CSEPacketOffsets {
    OFFSET_HEADER           = 0,    /**< Packet header (0x55) */
    OFFSET_STATUS           = 1,    /**< Status flags (validity indicators) */
    OFFSET_VOLTAGE_COEFF    = 2,    /**< Voltage coefficient (3 bytes, big-endian) */
    OFFSET_VOLTAGE_CYCLE    = 5,    /**< Voltage cycle count (3 bytes, big-endian) */
    OFFSET_CURRENT_COEFF    = 8,    /**< Current coefficient (3 bytes, big-endian) */
    OFFSET_CURRENT_CYCLE    = 11,   /**< Current cycle count (3 bytes, big-endian) */
    OFFSET_POWER_COEFF      = 14,   /**< Power coefficient (3 bytes, big-endian) */
    OFFSET_POWER_CYCLE      = 17,   /**< Power cycle count (3 bytes, big-endian) */
    OFFSET_ADJ              = 20,   /**< Adjustment value (not used) */
    OFFSET_CF_HIGH          = 21,   /**< CF pulse counter high byte */
    OFFSET_CF_LOW           = 22,   /**< CF pulse counter low byte */
    OFFSET_CHECKSUM         = 23    /**< Packet checksum (sum of bytes 2-22) */
};

/* ================================================================================================
 * TM1621 LCD DRIVER CONSTANTS
 * ================================================================================================ */

#define TM1621_PULSE_WIDTH   10UL    /**< Pulse width in microseconds (10us = 100kHz) */
#define TM1621_PULSE_HALF    5UL     /**< Half pulse width for timing */

/* TM1621 Command Codes */
#define TM1621_CMD_SYS_EN    0x01    /**< System enable - turns on oscillator */
#define TM1621_CMD_LCD_ON    0x03    /**< LCD display on - enables output */
#define TM1621_CMD_TIMER_DIS 0x04    /**< Disable internal timer */
#define TM1621_CMD_WDT_DIS   0x05    /**< Disable watchdog timer */
#define TM1621_CMD_TONE_OFF  0x08    /**< Turn off tone generator (not used) */
#define TM1621_CMD_BIAS      0x29    /**< Set bias configuration (1/3 bias, 4 commons) */
#define TM1621_CMD_IRQ_DIS   0x80    /**< Disable interrupt output */

/** TM1621 initialization command sequence (must be sent in this order) */
const uint8_t tm1621_commands[] = {
    TM1621_CMD_SYS_EN,      // Step 1: Enable system oscillator
    TM1621_CMD_LCD_ON,      // Step 2: Turn on LCD display
    TM1621_CMD_BIAS,        // Step 3: Configure bias voltage
    TM1621_CMD_TIMER_DIS,   // Step 4: Disable timer (not needed)
    TM1621_CMD_WDT_DIS,     // Step 5: Disable watchdog (not needed)
    TM1621_CMD_TONE_OFF,    // Step 6: Disable tone generator
    TM1621_CMD_IRQ_DIS      // Step 7: Disable interrupts
};

/**
 * @brief 7-segment digit mapping for common cathode display
 * 
 * Each byte controls 8 segments in the following order (bit7 to bit0):
 * bit7: DP (decimal point)
 * bit6: A (top)
 * bit5: B (top-right)
 * bit4: C (bottom-right)
 * bit3: D (bottom)
 * bit2: E (bottom-left)
 * bit1: F (top-left)
 * bit0: G (middle)
 * 
 * The values are pre-calculated for digits 0-9 and special characters.
 */
const uint8_t digitMap[] = {
    0x00, /**< 0: Blank (all segments off) */
    0x5F, /**< 1: "0" - segments A, B, C, D, E, F */
    0x50, /**< 2: "1" - segments B, C */
    0x3D, /**< 3: "2" - segments A, B, D, E, G */
    0x79, /**< 4: "3" - segments A, B, C, D, G */
    0x72, /**< 5: "4" - segments B, C, F, G */
    0x6B, /**< 6: "5" - segments A, C, D, F, G */
    0x6F, /**< 7: "6" - segments A, C, D, E, F, G */
    0x51, /**< 8: "7" - segments A, B, C */
    0x7F, /**< 9: "8" - all segments */
    0x7B, /**< 10: "9" - segments A, B, C, D, F, G */
    0x20, /**< 11: "-" (minus sign) - segment G only */
    0x2F  /**< 12: Error indicator - segments A, B, C, D, E, F (no G) */
};

/* ================================================================================================
 * GLOBAL VARIABLES
 * ================================================================================================ */

WebServer    server(80);                     /**< HTTP web server on port 80 */
DNSServer    dnsServer;                      /**< DNS server for captive portal */

// Relay Control
bool         relayState         = false;     /**< Current relay state (true=ON, false=OFF) */
bool         lastButtonState    = HIGH;      /**< Previous button state (HIGH=released, LOW=pressed) */
unsigned long lastDebounceTime  = 0;         /**< Timestamp of last button state change */
unsigned long buttonPressTime   = 0;         /**< Timestamp when button was pressed (for hold detection) */
bool         buttonWasPressed   = false;     /**< Flag to track if button was pressed (for hold detection) */

// WiFi Configuration
char         wifi_ssid[64]      = DEFAULT_WIFI_SSID;      /**< WiFi SSID (max 63 chars + null) */
char         wifi_password[64]  = DEFAULT_WIFI_PASSWORD;  /**< WiFi password (max 63 chars + null) */
bool         wifi_sta_enabled   = DEFAULT_WIFI_ENABLED;   /**< WiFi client mode enabled flag */
bool         wifiConnected      = false;                   /**< Current WiFi connection status */

// System State
bool         rebootPending      = false;     /**< Reboot scheduled flag */
unsigned long rebootTime        = 0;         /**< Reboot timestamp */

// CSE7759B Statistics
struct {
    uint32_t validFrames   = 0;              /**< Count of successfully parsed packets */
    uint32_t invalidFrames = 0;              /**< Count of failed packets (checksum error) */
} cseStats;

// Energy Accumulation
float        energyKWh        = 0;           /**< Total accumulated energy in kilowatt-hours */
unsigned long lastEnergyTime  = 0;           /**< Last energy update timestamp (millis) */

// Calibration Multipliers
float        voltageMultiplier = DEFAULT_VOLTAGE_MULTIPLIER;  /**< Voltage calibration factor */
float        currentMultiplier = DEFAULT_CURRENT_MULTIPLIER;  /**< Current calibration factor */
float        powerMultiplier   = DEFAULT_POWER_MULTIPLIER;    /**< Power calibration factor */

// Measured Values (after calibration)
float        voltage          = 0;           /**< RMS Voltage (Volts) */
float        current          = 0;           /**< RMS Current (Amperes) */
float        power            = 0;           /**< Active Power (Watts) */
float        powerFactor      = 1.0f;        /**< Power Factor (0-1, 1=resistive load) */

// Raw Values (from CSE7759B, before calibration)
float        rawVoltage       = 0;           /**< Raw voltage reading from CSE7759B */
float        rawCurrent       = 0;           /**< Raw current reading from CSE7759B */
float        rawPower         = 0;           /**< Raw power reading from CSE7759B */

// LCD Control
uint8_t      lcdBuffer[8];                    /**< LCD display buffer (8 bytes, one per buffer) */
bool         lcdScreenToggle  = false;        /**< Screen cycling toggle (alternates displays) */
unsigned long lastLcdUpdate   = 0;            /**< Last LCD update timestamp */

// LCD Test Mode
bool         testModeActive   = false;        /**< Test mode active flag (bypasses normal updates) */
int          testTopValue     = 0;            /**< Test value for top line (0-9999) */
int          testBottomValue  = 0;            /**< Test value for bottom line (0-9999) */
bool         testTopHasDecimal = false;       /**< Top line decimal point flag in test mode */
bool         testBottomHasDecimal = false;    /**< Bottom line decimal point flag in test mode */
bool         testShowTopSymbol = true;        /**< Show top symbol (V/A) in test mode */
bool         testShowBottomSymbol = true;     /**< Show bottom symbol (W/kWh) in test mode */

// Manual Bit Tester
bool         manualTesterActive = false;      /**< Manual bit tester active flag */

// System Logging
String       webLog           = "";           /**< Web interface log buffer */
const int    MAX_LOG_LINES    = 20;           /**< Maximum number of log lines to keep in buffer */

// CSE7759B Serial Port
HardwareSerial* cseSerial     = nullptr;      /**< Pointer to UART1 for CSE7759B communication */

/* ================================================================================================
 * UTILITY FUNCTIONS
 * ================================================================================================ */

/**
 * @brief Add a timestamped message to the system log
 * 
 * This function adds messages to both the serial console (if debug enabled)
 * and the web interface log buffer. The log is automatically trimmed to
 * prevent memory issues.
 * 
 * @param msg Message to add to the log
 */
void addLog(const String& msg) {
    String timestamp = String(millis() / 1000);
    String logLine = "[" + timestamp + "] " + msg;
    
    // Add to web log buffer
    if (webLog.length() > 0) webLog += "\n";
    webLog += logLine;
    
    // Trim log if too large (keep last MAX_LOG_LINES)
    int lines = 1;
    for (int i = 0; i < webLog.length(); i++) {
        if (webLog[i] == '\n') lines++;
    }
    if (lines > MAX_LOG_LINES) {
        int firstNewline = webLog.indexOf('\n');
        if (firstNewline >= 0) {
            webLog = webLog.substring(firstNewline + 1);
        }
    }
    
    // Output to serial if enabled
    #ifdef ENABLE_SERIAL_DEBUG
        Serial.println(msg);
    #endif
}

/* ================================================================================================
 * LCD VALUE FORMATTING FUNCTIONS
 * 
 * These functions convert raw float measurements into 4-digit integers suitable
 * for display on the 7-segment LCD. They handle decimal point placement and
 * leading zeros according to the display rules.
 * ================================================================================================ */

/**
 * @brief Format voltage for 4-digit LCD display
 * 
 * @param voltage Input voltage value (80-300V)
 * @param displayValue Output: 4-digit integer (0-9999) to display
 * @param showDecimal Output: true if decimal point should be shown on 3rd digit
 * 
 * @details Formatting rules:
 * - Voltage >= 100V with fractional part > 0.05: Show 1 decimal (e.g., 221.3 → 2213)
 * - Otherwise: Show as integer with leading zeros (e.g., 220 → 0220)
 * 
 * @note The decimal point can only be placed on the 3rd digit (tens place)
 */
void formatVoltage(float voltage, int& displayValue, bool& showDecimal) {
    if (voltage >= 100.0f) {
        float fractional = voltage - (int)voltage;
        if (fractional > 0.05f && voltage < 1000.0f) {
            displayValue = (int)(voltage * 10);  // Convert to 1 decimal place
            showDecimal = true;
        } else {
            displayValue = (int)voltage;          // Integer value
            showDecimal = false;
        }
    } else {
        displayValue = (int)voltage;
        showDecimal = false;
    }
}

/**
 * @brief Format current for 4-digit LCD display
 * 
 * @param current Input current value (0-100A)
 * @param displayValue Output: 4-digit integer (0-9999) to display
 * @param showDecimal Output: true if decimal point should be shown on 3rd digit
 * 
 * @details Formatting rules:
 * - Current < 10A with fractional part > 0.05: Show 1 decimal (e.g., 2.2 → 0022)
 * - Current < 10A integer: Show as 4 digits with leading zeros (e.g., 3 → 0003)
 * - Current >= 10A: Show as integer (e.g., 15.5 → 1550 with decimal, or 15 → 0015)
 */
void formatCurrent(float current, int& displayValue, bool& showDecimal) {
    if (current < 10.0f) {
        float fractional = current - (int)current;
        if (fractional > 0.05f) {
            displayValue = (int)(current * 10);
            showDecimal = true;
        } else {
            displayValue = (int)current;
            showDecimal = false;
        }
    } else {
        if (current < 100.0f) {
            // Check if current has decimal part
            float fractional = current - (int)current;
            if (fractional > 0.05f) {
                displayValue = (int)(current * 10);
                showDecimal = true;
            } else {
                displayValue = (int)current;
                showDecimal = false;
            }
        } else {
            displayValue = 9999;  // Overflow - max display
            showDecimal = false;
        }
    }
}

/**
 * @brief Format power for 4-digit LCD display (UPDATED v1.0)
 * 
 * @param power Input power value (0-25000W)
 * @param displayValue Output: 4-digit integer (0-9999) to display
 * @param showDecimal Output: true if decimal point should be shown on 3rd digit
 * 
 * @details Formatting rules (v1.0):
 * - Power < 100W: Show with 1 decimal (e.g., 8.5W → 0085, decimal=true)
 * - Power 100-999W: Show as integer with leading zero (e.g., 958W → 0958, decimal=false)
 * - Power ≥ 1000W: Show as integer (e.g., 1500W → 1500, decimal=false)
 * - Values above 9999W capped at 9999
 */
void formatPower(float power, int& displayValue, bool& showDecimal) {
    if (power < 100.0f) {
        // Show with 1 decimal for powers under 100W
        // Examples: 8.5W -> 85, then padded to 0085 with decimal
        //           75.3W -> 753, padded to 0753 with decimal
        displayValue = (int)(power * 10);
        showDecimal = true;
        
        // Ensure we don't exceed 4 digits (max 999.9W = 9999)
        if (displayValue > 9999) displayValue = 9999;
    } 
    else if (power < 1000.0f) {
        // Show as 4-digit integer (e.g., 958W -> 0958)
        displayValue = (int)power;
        showDecimal = false;
    }
    else {
        // Show as integer for power >= 1000W
        displayValue = (int)power;
        if (displayValue > 9999) displayValue = 9999;
        showDecimal = false;
    }
}

/**
 * @brief Format energy for 4-digit LCD display
 * 
 * @param energy Input energy in kWh (0-99999 kWh)
 * @param displayValue Output: 4-digit integer (0-9999) to display
 * @param showDecimal Output: true if decimal point should be shown on 3rd digit
 * 
 * @details Formatting rules:
 * - Energy < 100 kWh: Show 1 decimal (e.g., 12.34 → 0123)
 * - Energy >= 100 kWh: Show as integer (e.g., 100 → 0100, 1500 → 1500)
 */
void formatEnergy(float energy, int& displayValue, bool& showDecimal) {
    if (energy < 100.0f) {
        displayValue = (int)(energy * 10);  // Show 1 decimal for <100 kWh
        showDecimal = true;
    } else if (energy < 1000.0f) {
        displayValue = (int)energy;
        showDecimal = false;
    } else {
        displayValue = (int)energy;
        if (displayValue > 9999) displayValue = 9999;
        showDecimal = false;
    }
}

/* ================================================================================================
 * CSE7759B ENERGY METER DRIVER
 * 
 * This section handles communication with the CSE7759B energy metering IC.
 * The chip sends continuous 24-byte packets at 4800 baud. Each packet contains
 * voltage, current, and power measurements in coefficient/cycle format.
 * 
 * PROTOCOL DETAILS:
 * - UART: 4800 baud, 8 data bits, Even parity, 1 stop bit (8E1)
 * - Packet length: 24 bytes fixed
 * - Header: 0x55 at byte 0
 * - Checksum: Sum of bytes 2-22 should equal byte 23 (mod 256)
 * - All multi-byte values are big-endian (MSB first)
 * 
 * CALCULATION FORMULAS:
 * - Voltage (V) = Voltage_Coefficient / Voltage_Cycle
 * - Current (A) = Current_Coefficient / Current_Cycle
 * - Power (W)   = Power_Coefficient / Power_Cycle
 * - Power Factor = Real Power / (Voltage × Current)
 * 
 * ================================================================================================ */

/**
 * @brief Parse a complete CSE7759B data packet and update measurements
 * 
 * @param buffer 24-byte packet received from CSE7759B
 * 
 * @details This function validates the packet checksum, extracts the 24-bit
 * coefficient and cycle values, calculates actual readings using the formula
 * Value = Coefficient / Cycle, and applies calibration multipliers.
 * 
 * The raw values are stored separately to allow calibration without re-communication.
 * 
 * @note This function is called automatically when a complete packet is received
 */
void parseCSEPacket(uint8_t* buffer) {
    static uint32_t packetCount = 0;
    
    // Verify first header byte (must be 0x55)
    if (buffer[OFFSET_HEADER] != CSE_HEADER1) {
        cseStats.invalidFrames++;
        return;
    }
    
    // Verify checksum (sum of bytes 2 through 22 should equal byte 23)
    uint8_t checksum = 0;
    for (int i = 2; i < 23; i++) {
        checksum += buffer[i];
    }
    
    if (checksum != buffer[OFFSET_CHECKSUM]) {
        cseStats.invalidFrames++;
        return;
    }
    
    // Extract 24-bit big-endian values
    // Voltage: 3 bytes (coefficient) / 3 bytes (cycle)
    uint32_t voltage_coeff = ((uint32_t)buffer[OFFSET_VOLTAGE_COEFF]     << 16) |
                             ((uint32_t)buffer[OFFSET_VOLTAGE_COEFF + 1] << 8)  |
                             (uint32_t)buffer[OFFSET_VOLTAGE_COEFF + 2];
    
    uint32_t voltage_cycle = ((uint32_t)buffer[OFFSET_VOLTAGE_CYCLE]     << 16) |
                             ((uint32_t)buffer[OFFSET_VOLTAGE_CYCLE + 1] << 8)  |
                             (uint32_t)buffer[OFFSET_VOLTAGE_CYCLE + 2];
    
    // Current: 3 bytes (coefficient) / 3 bytes (cycle)
    uint32_t current_coeff = ((uint32_t)buffer[OFFSET_CURRENT_COEFF]     << 16) |
                             ((uint32_t)buffer[OFFSET_CURRENT_COEFF + 1] << 8)  |
                             (uint32_t)buffer[OFFSET_CURRENT_COEFF + 2];
    
    uint32_t current_cycle = ((uint32_t)buffer[OFFSET_CURRENT_CYCLE]     << 16) |
                             ((uint32_t)buffer[OFFSET_CURRENT_CYCLE + 1] << 8)  |
                             (uint32_t)buffer[OFFSET_CURRENT_CYCLE + 2];
    
    // Power: 3 bytes (coefficient) / 3 bytes (cycle)
    uint32_t power_coeff = ((uint32_t)buffer[OFFSET_POWER_COEFF]     << 16) |
                           ((uint32_t)buffer[OFFSET_POWER_COEFF + 1] << 8)  |
                           (uint32_t)buffer[OFFSET_POWER_COEFF + 2];
    
    uint32_t power_cycle = ((uint32_t)buffer[OFFSET_POWER_CYCLE]     << 16) |
                           ((uint32_t)buffer[OFFSET_POWER_CYCLE + 1] << 8)  |
                           (uint32_t)buffer[OFFSET_POWER_CYCLE + 2];
    
    // Debug output for first 10 packets
    if (packetCount < 10) {
        packetCount++;
        addLog(String("Pkt") + packetCount + ": V coeff=" + String(voltage_coeff) +
               " cycle=" + String(voltage_cycle));
    }
    
    // Calculate Voltage (V = coeff / cycle)
    if (voltage_cycle > CSE_MIN_VALID_CYCLE && voltage_cycle < 100000) {
        rawVoltage = (float)voltage_coeff / (float)voltage_cycle;
        if (rawVoltage >= MIN_VOLTAGE && rawVoltage <= MAX_VOLTAGE) {
            voltage = rawVoltage * voltageMultiplier;
        }
    }
    
    // Calculate Current (A = coeff / cycle)
    if (current_cycle > CSE_MIN_VALID_CYCLE && current_cycle < 500000) {
        rawCurrent = (float)current_coeff / (float)current_cycle;
        if (rawCurrent >= MIN_CURRENT && rawCurrent <= MAX_CURRENT) {
            current = rawCurrent * currentMultiplier;
        }
    } else {
        current = 0;  // No load or invalid reading
    }
    
    // Calculate Power (W = coeff / cycle)
    if (power_cycle > CSE_MIN_VALID_CYCLE && power_cycle < 20000000) {
        rawPower = (float)power_coeff / (float)power_cycle;
        if (rawPower >= MIN_POWER && rawPower <= MAX_POWER) {
            power = rawPower * powerMultiplier;
        }
    } else {
        power = 0;
    }
    
    // Calculate Power Factor (PF = Real Power / Apparent Power)
    float apparentPower = voltage * current;
    if (apparentPower > 1.0f && power > 0.5f) {
        powerFactor = power / apparentPower;
        if (powerFactor > 1.0f) powerFactor = 1.0f;
        if (powerFactor < 0.0f) powerFactor = 0.0f;
    } else if (power < 0.5f) {
        powerFactor = 0.0f;  // No power draw
    }
    
    cseStats.validFrames++;
    
    // Periodic debug output (every 5 seconds)
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug > 5000) {
        lastDebug = millis();
        addLog(String("V=") + String(voltage, 1) + "V I=" + String(current, 3) +
               "A P=" + String(power, 1) + "W PF=" + String(powerFactor, 3));
    }
}

/**
 * @brief Read and process incoming CSE7759B serial data
 * 
 * This function must be called frequently in the main loop. It buffers incoming
 * bytes until a complete 24-byte packet is received, then calls parseCSEPacket().
 * 
 * @note Uses static variables to maintain packet state between calls
 */
void handleCSESerial() {
    static uint8_t rxBuffer[24];    // Packet buffer
    static int byteCounter = 0;     // Current position in buffer
    
    while (cseSerial->available()) {
        uint8_t byteIn = cseSerial->read();
        
        // Look for start of packet (0x55)
        if (byteCounter == 0 && byteIn != CSE_HEADER1) {
            continue;  // Discard bytes until we find header
        }
        
        rxBuffer[byteCounter++] = byteIn;
        
        // Process complete packet
        if (byteCounter == CSE_PACKET_LEN) {
            parseCSEPacket(rxBuffer);
            byteCounter = 0;  // Reset for next packet
        }
        
        // Safety: prevent buffer overflow
        if (byteCounter > CSE_PACKET_LEN) {
            byteCounter = 0;
        }
    }
}

/**
 * @brief Update accumulated energy based on current power
 * 
 * This function integrates power over time to calculate energy in kWh.
 * It uses the formula: Energy (kWh) = Power (W) × Time (s) / 3600000
 * 
 * @note Energy only accumulates when relay is ON and power is positive
 */
void updateEnergy() {
    unsigned long now = millis();
    
    if (lastEnergyTime == 0) {
        lastEnergyTime = now;
        return;
    }
    
    float dt = (now - lastEnergyTime) / 1000.0f;  // Time delta in seconds
    lastEnergyTime = now;
    
    // Limit time delta to prevent spikes (max 2 seconds)
    if (dt > 2.0f) dt = 2.0f;
    
    // Accumulate energy only when relay is ON and power is positive
    if (relayState && power > 0) {
        float delta_kWh = (power / 1000.0f) * (dt / 3600.0f);
        energyKWh += delta_kWh;
        
        // Rollover at 99999 kWh (max display)
        if (energyKWh > 99999.0f) energyKWh = 0.0f;
    }
}

/**
 * @brief Initialize the CSE7759B energy meter
 * 
 * Sets up the UART communication with the correct baud rate and parity.
 * Flushes any pending data and sets initial values.
 */
void initEnergyMeter(void) {
    cseSerial = new HardwareSerial(1);
    cseSerial->begin(CSE_BAUD, CSE_UART_CONFIG, PIN_CSE_RX, -1, false);
    
    // Clear any pending data from buffer
    while (cseSerial->available()) {
        cseSerial->read();
    }
    
    delay(500);  // Allow CSE7759B to stabilize
    
    memset(&cseStats, 0, sizeof(cseStats));
    lastEnergyTime = 0;
    
    // Set safe default values
    voltage = 230.0f;
    current = 0;
    power = 0;
    
    addLog("CSE7759B: Initialized v1.0");
}

/**
 * @brief Read energy meter data (call frequently in loop)
 * 
 * This function processes incoming serial data and updates energy accumulation
 * at 1-second intervals.
 */
void readEnergyData(void) {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    
    handleCSESerial();  // Process incoming packets
    
    // Update energy calculation once per second
    if (now - lastUpdate >= ENERGY_UPDATE_INTERVAL) {
        lastUpdate = now;
        updateEnergy();
    }
}

/* ================================================================================================
 * CALIBRATION FUNCTIONS
 * 
 * These functions adjust the measurement multipliers based on external reference readings.
 * Calibration values are stored in LittleFS and persist across reboots.
 * ================================================================================================ */

/**
 * @brief Calibrate voltage measurement
 * 
 * @param actualVoltage Measured voltage from external reference meter (80-280V)
 * 
 * @details Calculates new multiplier = actual / raw, then applies to future readings.
 * Raw value must be valid (received from CSE7759B) before calibration.
 */
void calibrateVoltage(float actualVoltage) {
    if (rawVoltage > 0 && actualVoltage >= MIN_VOLTAGE && actualVoltage <= MAX_VOLTAGE) {
        float newMultiplier = actualVoltage / rawVoltage;
        voltageMultiplier = newMultiplier;
        addLog(String("Voltage calibrated: ") + String(voltageMultiplier, 6));
        saveCalibrationToFile();
    } else {
        addLog("Voltage calibration failed: Invalid raw reading or actual value");
    }
}

/**
 * @brief Calibrate current measurement
 * 
 * @param actualCurrent Measured current from external reference meter (0-100A)
 */
void calibrateCurrent(float actualCurrent) {
    if (rawCurrent > 0 && actualCurrent >= MIN_CURRENT && actualCurrent <= MAX_CURRENT) {
        float newMultiplier = actualCurrent / rawCurrent;
        currentMultiplier = newMultiplier;
        addLog(String("Current calibrated: ") + String(currentMultiplier, 6));
        saveCalibrationToFile();
    }
}

/**
 * @brief Calibrate power measurement
 * 
 * @param actualPower Measured power from external reference meter (1-25000W)
 */
void calibratePower(float actualPower) {
    if (rawPower > 0 && actualPower >= MIN_POWER && actualPower <= MAX_POWER) {
        float newMultiplier = actualPower / rawPower;
        powerMultiplier = newMultiplier;
        addLog(String("Power calibrated: ") + String(powerMultiplier, 6));
        saveCalibrationToFile();
    }
}

/**
 * @brief Save calibration multipliers and energy to LittleFS
 * 
 * Uses JSON format for easy parsing and human readability.
 * File: /calibration.json
 */
void saveCalibrationToFile(void) {
    StaticJsonDocument<256> doc;
    doc["v_mult"] = voltageMultiplier;
    doc["i_mult"] = currentMultiplier;
    doc["p_mult"] = powerMultiplier;
    doc["energy_kwh"] = energyKWh;
    
    File file = LittleFS.open("/calibration.json", "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        addLog("Calibration saved to flash");
    } else {
        addLog("ERROR: Failed to save calibration file");
    }
}

/**
 * @brief Load calibration multipliers and energy from LittleFS
 * 
 * If file doesn't exist, uses default values.
 */
void loadCalibrationFromFile(void) {
    if (!LittleFS.begin()) {
        addLog("LittleFS mount failed, formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    
    if (!LittleFS.exists("/calibration.json")) {
        addLog("No calibration file, using defaults");
        voltageMultiplier = DEFAULT_VOLTAGE_MULTIPLIER;
        currentMultiplier = DEFAULT_CURRENT_MULTIPLIER;
        powerMultiplier = DEFAULT_POWER_MULTIPLIER;
        return;
    }
    
    File file = LittleFS.open("/calibration.json", "r");
    if (!file) return;
    
    StaticJsonDocument<256> doc;
    deserializeJson(doc, file);
    file.close();
    
    voltageMultiplier = doc["v_mult"] | DEFAULT_VOLTAGE_MULTIPLIER;
    currentMultiplier = doc["i_mult"] | DEFAULT_CURRENT_MULTIPLIER;
    powerMultiplier = doc["p_mult"] | DEFAULT_POWER_MULTIPLIER;
    energyKWh = doc["energy_kwh"] | 0.0f;
    
    addLog("Calibration loaded from flash");
}

/**
 * @brief Reset all calibration multipliers to default values (1.0)
 */
void resetCalibrationToDefault(void) {
    voltageMultiplier = DEFAULT_VOLTAGE_MULTIPLIER;
    currentMultiplier = DEFAULT_CURRENT_MULTIPLIER;
    powerMultiplier = DEFAULT_POWER_MULTIPLIER;
    saveCalibrationToFile();
    addLog("Calibration reset to defaults");
}

/* ================================================================================================
 * TM1621 LCD DRIVER - LOW LEVEL FUNCTIONS
 * 
 * These functions implement the communication protocol for the TM1621 LCD controller.
 * The protocol is bit-banged because the chip doesn't support standard SPI/I2C.
 * 
 * COMMUNICATION PROTOCOL:
 * - CS (Chip Select) active LOW
 * - Data is clocked on WR rising edge
 * - MSB first transmission
 * - Commands: 12-bit (1 start bit + 8 command + 3 stop bits)
 * - Address: 9-bit (1 start + 6 address + 2 stop bits)
 * - Data: 8-bit per common (COM0-COM3)
 * 
 * TIMING REQUIREMENTS:
 * - Pulse width: 10μs minimum
 * - Setup time: 2.5μs minimum
 * - Hold time: 2.5μs minimum
 * 
 * ================================================================================================ */

/**
 * @brief TM1621 stop sequence - end communication
 * 
 * Raises CS and DATA lines to high to release the bus.
 */
void TM1621StopSequence(void) {
    digitalWrite(PIN_LCD_CS, HIGH);
    delayMicroseconds(TM1621_PULSE_HALF);
    digitalWrite(PIN_LCD_DATA, HIGH);
}

/**
 * @brief Send a command to TM1621
 * 
 * @param command 8-bit command code (actual command is 9 bits including prefix)
 * 
 * @details TM1621 commands are 9 bits: 1 start bit (0) + 8 command bits.
 * The command is shifted out MSB first.
 */
void TM1621SendCmnd(uint16_t command) {
    uint16_t full_command = (0x0400 | command) << 5;
    digitalWrite(PIN_LCD_CS, LOW);
    delayMicroseconds(TM1621_PULSE_HALF);
    
    for (uint32_t i = 0; i < 12; i++) {
        digitalWrite(PIN_LCD_WR, LOW);
        digitalWrite(PIN_LCD_DATA, (full_command & 0x8000) ? HIGH : LOW);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        digitalWrite(PIN_LCD_WR, HIGH);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        full_command <<= 1;
    }
    TM1621StopSequence();
}

/**
 * @brief Send address to TM1621
 * 
 * @param address 6-bit address (0-63) for display memory
 * 
 * @details Address is sent with a command prefix.
 */
void TM1621SendAddress(uint16_t address) {
    uint16_t full_address = (address | 0x0140) << 7;
    digitalWrite(PIN_LCD_CS, LOW);
    delayMicroseconds(TM1621_PULSE_HALF);
    
    for (uint32_t i = 0; i < 9; i++) {
        digitalWrite(PIN_LCD_WR, LOW);
        digitalWrite(PIN_LCD_DATA, (full_address & 0x8000) ? HIGH : LOW);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        digitalWrite(PIN_LCD_WR, HIGH);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        full_address <<= 1;
    }
}

/**
 * @brief Send common data to TM1621
 * 
 * @param common 8-bit segment data to display
 * 
 * @details Each common (COM) line controls one digit position.
 */
void TM1621SendCommon(uint8_t common) {
    for (uint32_t i = 0; i < 8; i++) {
        digitalWrite(PIN_LCD_WR, LOW);
        digitalWrite(PIN_LCD_DATA, (common & 1) ? HIGH : LOW);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        digitalWrite(PIN_LCD_WR, HIGH);
        delayMicroseconds(TM1621_PULSE_WIDTH);
        common >>= 1;
    }
}

/**
 * @brief Convert row1 format to row2 format (for bottom line)
 * 
 * @param row1 8-bit value in row1 (top line) format
 * @return 8-bit value in row2 (bottom line) format
 * 
 * @details The TM1621 uses different segment mapping for top and bottom rows.
 * This function remaps bits to the correct positions for bottom row display.
 * 
 * Bit mapping transformation:
 * Row1 bits: 0,1,2,3,4,5,6,7
 * Row2 bits: 6,5,4,7,3,2,1,0
 */
uint32_t TM1621Row2(uint32_t row1) {
    uint32_t row2 = 0;
    bitWrite(row2, 0, bitRead(row1, 6));
    bitWrite(row2, 1, bitRead(row1, 5));
    bitWrite(row2, 2, bitRead(row1, 4));
    bitWrite(row2, 3, bitRead(row1, 7));
    bitWrite(row2, 4, bitRead(row1, 3));
    bitWrite(row2, 5, bitRead(row1, 2));
    bitWrite(row2, 6, bitRead(row1, 1));
    bitWrite(row2, 7, bitRead(row1, 0));
    return row2;
}

/**
 * @brief Initialize TM1621 LCD driver
 * 
 * Configures GPIO pins, sends initialization command sequence,
 * and clears the display buffer.
 * 
 * Initialization sequence:
 * 1. Hardware reset (CS low, RD low, WR low)
 * 2. Send system enable command
 * 3. Send LCD on command
 * 4. Configure bias
 * 5. Disable unused features
 * 6. Clear display memory
 */
void TM1621Init(void) {
    // Configure pins
    pinMode(PIN_LCD_DATA, OUTPUT);
    pinMode(PIN_LCD_WR, OUTPUT);
    pinMode(PIN_LCD_RD, OUTPUT);
    pinMode(PIN_LCD_CS, OUTPUT);
    
    // Set default states
    digitalWrite(PIN_LCD_DATA, HIGH);
    digitalWrite(PIN_LCD_CS, HIGH);
    digitalWrite(PIN_LCD_RD, HIGH);
    digitalWrite(PIN_LCD_WR, HIGH);
    
    // Hardware reset sequence
    digitalWrite(PIN_LCD_CS, LOW);
    delayMicroseconds(80);
    digitalWrite(PIN_LCD_RD, LOW);
    delayMicroseconds(15);
    digitalWrite(PIN_LCD_WR, LOW);
    delayMicroseconds(25);
    digitalWrite(PIN_LCD_DATA, LOW);
    delayMicroseconds(TM1621_PULSE_WIDTH);
    digitalWrite(PIN_LCD_DATA, HIGH);
    
    // Send configuration commands
    for (uint32_t cmd = 0; cmd < sizeof(tm1621_commands); cmd++) {
        TM1621SendCmnd(tm1621_commands[cmd]);
    }
    
    // Clear all display memory (16 segments × 2 addresses)
    TM1621SendAddress(0x00);
    for (uint32_t segment = 0; segment < 16; segment++) {
        TM1621SendCommon(0);
    }
    TM1621StopSequence();
    
    memset(lcdBuffer, 0, sizeof(lcdBuffer));
    addLog("TM1621: LCD initialized");
}

/**
 * @brief Send current LCD buffer to display
 * 
 * Transfers all 8 bytes of display buffer to the TM1621.
 * Each buffer position corresponds to one COM line.
 */
void TM1621SendRows(void) {
    TM1621SendAddress(0x10);  // Start at address 0x10 (display RAM)
    for (uint32_t i = 0; i < 8; i++) {
        TM1621SendCommon(lcdBuffer[i]);
    }
    TM1621StopSequence();
}

/**
 * @brief Clear the LCD display buffer (doesn't update hardware)
 */
void clearBuffer(void) {
    memset(lcdBuffer, 0, sizeof(lcdBuffer));
}

/**
 * @brief Get segment pattern for a digit
 * 
 * @param digit Digit 0-9
 * @return 8-bit segment pattern from digitMap
 */
uint8_t getDigit(int digit) {
    if (digit < 0 || digit > 9) return 0x00;
    return digitMap[digit + 1];
}

/* ================================================================================================
 * LCD DISPLAY FUNCTIONS - HIGH LEVEL
 * 
 * These functions handle the actual display of numbers and symbols on the LCD.
 * They automatically handle digit placement, decimal points, and symbols.
 * ================================================================================================ */

/**
 * @brief Set top line (4 digits) with optional decimal point
 * 
 * @param value 0-9999 integer value to display (will be padded to 4 digits)
 * @param decimal true to show decimal point on 3rd digit
 * 
 * @note Preserves V/A symbol in Buffer[7] (top line symbol)
 */
void setTopNumber(int value, bool decimal = false) {
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%04d", value);
    
    // Preserve V/A symbol that might be in Buffer[7]
    uint8_t va_symbol = lcdBuffer[7] & LCD_VA_SYMBOL;
    
    lcdBuffer[0] = getDigit(buffer[0] - '0');  // Digit 1 (thousands)
    lcdBuffer[1] = getDigit(buffer[1] - '0');  // Digit 2 (hundreds)
    
    if (decimal) {
        // Decimal point on 3rd digit (tens place)
        lcdBuffer[2] = getDigit(buffer[2] - '0') | LCD_TOP_DECIMAL;
    } else {
        lcdBuffer[2] = getDigit(buffer[2] - '0');
    }
    
    lcdBuffer[3] = getDigit(buffer[3] - '0');  // Digit 4 (units)
    
    // Restore V/A symbol to Buffer[7]
    lcdBuffer[7] = va_symbol;
}

/**
 * @brief Set bottom line (4 digits) with optional decimal point
 * 
 * @param value 0-9999 integer value to display (will be padded to 4 digits)
 * @param decimal true to show decimal point on 3rd digit
 * 
 * @note Preserves W/kWh symbol in Buffer[4] (bottom line symbol)
 */
void setBottomNumber(int value, bool decimal = false) {
    char buffer[5];
    snprintf(buffer, sizeof(buffer), "%04d", value);
    
    // Convert to row2 format for bottom line
    uint8_t d0 = TM1621Row2(getDigit(buffer[0] - '0'));  // Digit 1 -> Buffer[7]
    uint8_t d1 = TM1621Row2(getDigit(buffer[1] - '0'));  // Digit 2 -> Buffer[6]
    uint8_t d2 = TM1621Row2(getDigit(buffer[2] - '0'));  // Digit 3 -> Buffer[5]
    uint8_t d3 = TM1621Row2(getDigit(buffer[3] - '0'));  // Digit 4 -> Buffer[4]
    
    // Preserve W/kWh symbol that might be in Buffer[4]
    uint8_t wkwh_symbol = lcdBuffer[4] & LCD_WKWH_SYMBOL;
    
    if (decimal) {
        d2 |= LCD_BOTTOM_DECIMAL;
    }
    
    lcdBuffer[7] = d0;      // Bottom digit 1 (no top symbol here!)
    lcdBuffer[6] = d1;      // Bottom digit 2
    lcdBuffer[5] = d2;      // Bottom digit 3 (with possible decimal)
    lcdBuffer[4] = d3 | wkwh_symbol;  // Bottom digit 4 with W/kWh symbol
}

/**
 * @brief Set top V/A symbol (for voltage display)
 * 
 * @param show true to display symbol, false to hide
 * 
 * @details Controls the V/A symbol on the top line (Buffer[7] Bit 3)
 */
void setTopVASymbol(bool show) {
    if (show) {
        lcdBuffer[7] |= LCD_VA_SYMBOL;
    } else {
        lcdBuffer[7] &= ~LCD_VA_SYMBOL;
    }
}

/**
 * @brief Set bottom W/kWh symbol (for power/current display)
 * 
 * @param show true to display symbol, false to hide
 * 
 * @details Controls the W/kWh symbol on the bottom line (Buffer[4] Bit 3)
 */
void setBottomWKWhSymbol(bool show) {
    if (show) {
        lcdBuffer[4] |= LCD_WKWH_SYMBOL;
    } else {
        lcdBuffer[4] &= ~LCD_WKWH_SYMBOL;
    }
}

/**
 * @brief Show voltage on top line, current on bottom line
 * 
 * @param voltage Voltage value (80-300V)
 * @param current Current value (0-100A)
 * 
 * @details Displays voltage with V symbol and current with A symbol.
 * This is the primary measurement screen.
 */
void showVoltageCurrent(float voltage, float current) {
    clearBuffer();
    
    int voltageValue;
    bool voltageDecimal;
    formatVoltage(voltage, voltageValue, voltageDecimal);
    setTopNumber(voltageValue, voltageDecimal);
    
    int currentValue;
    bool currentDecimal;
    formatCurrent(current, currentValue, currentDecimal);
    setBottomNumber(currentValue, currentDecimal);
    
    setTopVASymbol(true);      // Show V/A symbol on top
        
    TM1621SendRows();
}

/**
 * @brief Show energy on top line, power on bottom line
 * 
 * @param energy Energy in kWh (0-99999 kWh)
 * @param power Power in Watts (0-25000W)
 * 
 * @details Displays energy on top (kWh symbol on bottom) and power on bottom (W symbol).
 * This is the secondary screen that cycles with the primary screen.
 */
void showEnergyPower(float energy, float power) {
    clearBuffer();
    
    int energyValue;
    bool energyDecimal;
    formatEnergy(energy, energyValue, energyDecimal);
    setTopNumber(energyValue, energyDecimal);
    setTopVASymbol(false);     // No symbol on top for energy
    
    int powerValue;
    bool powerDecimal;
    formatPower(power, powerValue, powerDecimal);
    setBottomNumber(powerValue, powerDecimal);
    setBottomWKWhSymbol(true); // Show W symbol on bottom
    
    TM1621SendRows();
}

/**
 * @brief Show custom test values on LCD (for test mode)
 * 
 * @param topValue 0-9999 value for top line
 * @param bottomValue 0-9999 value for bottom line
 * @param topHasDecimal Show decimal on top line
 * @param bottomHasDecimal Show decimal on bottom line
 * @param showTopSym Show V/A symbol
 * @param showBottomSym Show W/kWh symbol
 */
void showTestLCD(int topValue, int bottomValue, bool topHasDecimal, bool bottomHasDecimal, 
                  bool showTopSym, bool showBottomSym) {
    clearBuffer();
    setTopNumber(topValue, topHasDecimal);
    setBottomNumber(bottomValue, bottomHasDecimal);
    
    if (showTopSym) setTopVASymbol(true);
    if (showBottomSym) setBottomWKWhSymbol(true);
    
    TM1621SendRows();
}

/**
 * @brief Update LCD display (alternates between V/I and kW/P screens)
 * 
 * This function is called periodically from the main loop.
 * It alternates between showing Voltage/Current and Energy/Power.
 * 
 * @note Skips update if test mode or manual tester is active
 */
void updateLCDDisplay(void) {
    if (testModeActive || manualTesterActive) {
        return;  // Don't override test mode
    }
    
    if (millis() - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        lastLcdUpdate = millis();
        if (!lcdScreenToggle) {
            showVoltageCurrent(voltage, current);
        } else {
            showEnergyPower(energyKWh, power);
        }
        lcdScreenToggle = !lcdScreenToggle;
    }
}

/* ================================================================================================
 * MANUAL BIT TESTER FUNCTIONS
 * 
 * These functions allow manual testing of individual LCD segments.
 * Useful for discovering mapping on new hardware or troubleshooting.
 * ================================================================================================ */

/**
 * @brief Start manual bit tester mode
 * 
 * Shows "TEST" pattern on LCD and enables bit testing mode.
 */
void startManualTester() {
    manualTesterActive = true;
    testModeActive = false;
    clearBuffer();
    
    // Show "8888" pattern for reference
    lcdBuffer[0] = getDigit(8);
    lcdBuffer[1] = getDigit(8);
    lcdBuffer[2] = getDigit(8);
    lcdBuffer[3] = getDigit(8);
    lcdBuffer[4] = TM1621Row2(getDigit(8));
    lcdBuffer[5] = TM1621Row2(getDigit(8));
    lcdBuffer[6] = TM1621Row2(getDigit(8));
    lcdBuffer[7] = TM1621Row2(getDigit(8));
    TM1621SendRows();
    
    addLog("Manual Bit Tester Started");
}

/**
 * @brief Stop manual bit tester mode
 * 
 * Clears LCD and returns to normal operation.
 */
void stopManualTester() {
    manualTesterActive = false;
    clearBuffer();
    TM1621SendRows();
    addLog("Manual Bit Tester Stopped");
}

/**
 * @brief Test a single bit on LCD
 * 
 * @param bufferPos Buffer position (0-7)
 * @param bitPos Bit position (0-7)
 * @param withReference Show reference pattern (8888)
 * 
 * @details Sets the specified bit in the specified buffer and updates the LCD.
 * Also displays buffer:bit position on the top left for reference.
 */
void testSingleBit(int bufferPos, int bitPos, bool withReference) {
    if (!manualTesterActive) {
        startManualTester();
    }
    
    clearBuffer();
    
    if (withReference) {
        // Show all 8's as reference pattern
        for (int i = 0; i < 4; i++) {
            lcdBuffer[i] = getDigit(8);
        }
        lcdBuffer[4] = TM1621Row2(getDigit(8));
        lcdBuffer[5] = TM1621Row2(getDigit(8));
        lcdBuffer[6] = TM1621Row2(getDigit(8));
        lcdBuffer[7] = TM1621Row2(getDigit(8));
    }
    
    // Apply the test bit
    lcdBuffer[bufferPos] |= (1 << bitPos);
    
    // Show test info on LCD (buffer:bit)
    lcdBuffer[0] = getDigit(bufferPos);
    lcdBuffer[1] = getDigit(bitPos);
    
    TM1621SendRows();
    addLog(String("Test: buffer[") + bufferPos + "] bit " + bitPos + " = 0x" + String((1 << bitPos), HEX));
}

/* ================================================================================================
 * LCD TEST FUNCTIONS
 * 
 * These functions handle the test mode where users can manually enter values
 * to display on the LCD for testing purposes.
 * ================================================================================================ */

/**
 * @brief Parse user input and display on top line
 * 
 * @param input String like "1234" or "123.4"
 * @return true if parsing successful
 * 
 * @details Supports both integer (0-9999) and decimal (0-999.9) formats.
 * Decimal values are converted to 4-digit integers with decimal flag set.
 */
bool parseAndDisplayTop(String input) {
    input.trim();
    if (input.length() == 0) return false;
    
    if (input.indexOf('.') >= 0) {
        float num = input.toFloat();
        if (num >= 0 && num <= 999.9) {
            int intValue = (int)(num * 10 + 0.5);  // Convert to 4-digit integer
            if (intValue >= 0 && intValue <= 9999) {
                testTopValue = intValue;
                testTopHasDecimal = true;
                addLog(String("Top parsed: ") + String(num) + " -> " + String(intValue) + " decimal=true");
                return true;
            }
        }
    } else {
        int intValue = input.toInt();
        if (intValue >= 0 && intValue <= 9999) {
            testTopValue = intValue;
            testTopHasDecimal = false;
            addLog(String("Top parsed: ") + String(intValue) + " decimal=false");
            return true;
        }
    }
    return false;
}

/**
 * @brief Parse user input and display on bottom line
 * 
 * @param input String like "1234" or "123.4"
 * @return true if parsing successful
 */
bool parseAndDisplayBottom(String input) {
    input.trim();
    if (input.length() == 0) return false;
    
    if (input.indexOf('.') >= 0) {
        float num = input.toFloat();
        if (num >= 0 && num <= 999.9) {
            int intValue = (int)(num * 10 + 0.5);
            if (intValue >= 0 && intValue <= 9999) {
                testBottomValue = intValue;
                testBottomHasDecimal = true;
                addLog(String("Bottom parsed: ") + String(num) + " -> " + String(intValue) + " decimal=true");
                return true;
            }
        }
    } else {
        int intValue = input.toInt();
        if (intValue >= 0 && intValue <= 9999) {
            testBottomValue = intValue;
            testBottomHasDecimal = false;
            addLog(String("Bottom parsed: ") + String(intValue) + " decimal=false");
            return true;
        }
    }
    return false;
}

/**
 * @brief Activate LCD test mode with current test values
 */
void activateTestMode() {
    if (manualTesterActive) stopManualTester();
    testModeActive = true;
    showTestLCD(testTopValue, testBottomValue, testTopHasDecimal, testBottomHasDecimal,
                 testShowTopSymbol, testShowBottomSymbol);
    addLog("LCD Test mode ACTIVE");
}

/**
 * @brief Exit LCD test mode and return to normal operation
 */
void exitTestMode() {
    if (testModeActive) {
        testModeActive = false;
        lastLcdUpdate = 0;
        lcdScreenToggle = false;
        addLog("LCD Test mode ENDED");
        updateLCDDisplay();  // Immediately show normal display
    }
}

/* ================================================================================================
 * RELAY & LED CONTROL FUNCTIONS
 * 
 * Hardware control for relay and status LEDs.
 * ================================================================================================ */

/**
 * @brief Set WiFi status LED
 * 
 * @param state true = ON, false = OFF
 */
void setWiFiLED(bool state) {
    digitalWrite(PIN_WIFI_LED, state ? HIGH : LOW);
}

/**
 * @brief Set system status LED
 * 
 * @param state true = ON, false = OFF
 */
void setStatusLED(bool state) {
    digitalWrite(PIN_STATUS_LED, state ? HIGH : LOW);
}

/**
 * @brief Set relay state (controls AC load)
 * 
 * @param state true = ON (load connected), false = OFF (load disconnected)
 * 
 * @details Also updates WiFi LED and provides visual feedback by blinking status LED.
 */
void setRelay(bool state) {
    relayState = state;
    digitalWrite(PIN_RELAY, state ? HIGH : LOW);
    addLog(String("Relay: ") + (state ? "ON" : "OFF"));
    
    // Visual feedback: blink status LED and update WiFi LED
    setWiFiLED(state);
    setStatusLED(true);
    delay(100);
    setStatusLED(false);
}

/**
 * @brief Toggle relay state (ON→OFF or OFF→ON)
 * 
 * This function can be called from button press or web interface.
 */
void toggleRelay(void) {
    setRelay(!relayState);
}

/* ================================================================================================
 * CONFIGURATION MANAGEMENT
 * 
 * Functions for saving/loading configuration to/from LittleFS flash storage.
 * ================================================================================================ */

/**
 * @brief Save WiFi configuration to file
 * 
 * Stores WiFi credentials and relay state to /config.json
 */
void saveConfigToFile(void) {
    StaticJsonDocument<512> doc;
    doc["wifi_sta_enabled"] = wifi_sta_enabled;
    doc["wifi_ssid"] = String(wifi_ssid);
    doc["wifi_password"] = String(wifi_password);
    doc["relay_state"] = relayState;
    
    File file = LittleFS.open("/config.json", "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        addLog("Configuration saved to flash");
    } else {
        addLog("ERROR: Failed to save configuration");
    }
}

/**
 * @brief Load WiFi configuration from file
 * 
 * If file doesn't exist, creates default configuration.
 */
void loadConfigFromFile(void) {
    if (!LittleFS.begin()) {
        addLog("LittleFS mount failed, formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    
    if (!LittleFS.exists("/config.json")) {
        addLog("No configuration file, using defaults");
        wifi_sta_enabled = DEFAULT_WIFI_ENABLED;
        strcpy(wifi_ssid, DEFAULT_WIFI_SSID);
        strcpy(wifi_password, DEFAULT_WIFI_PASSWORD);
        saveConfigToFile();
        return;
    }
    
    File file = LittleFS.open("/config.json", "r");
    if (!file) return;
    
    StaticJsonDocument<512> doc;
    deserializeJson(doc, file);
    file.close();
    
    wifi_sta_enabled = doc["wifi_sta_enabled"] | DEFAULT_WIFI_ENABLED;
    String ssid = doc["wifi_ssid"].as<String>();
    String pwd = doc["wifi_password"].as<String>();
    ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
    pwd.toCharArray(wifi_password, sizeof(wifi_password));
    relayState = doc["relay_state"] | false;
    
    // Apply relay state
    digitalWrite(PIN_RELAY, relayState ? HIGH : LOW);
    
    addLog("Configuration loaded from flash");
}

/**
 * @brief Factory reset - erase all configuration and calibration
 * 
 * Removes config and calibration files, then reboots.
 */
void factoryReset(void) {
    addLog("Factory reset initiated...");
    if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
    if (LittleFS.exists("/calibration.json")) LittleFS.remove("/calibration.json");
    delay(500);
    ESP.restart();
}

/* ================================================================================================
 * WIFI FUNCTIONS
 * 
 * Network setup including Access Point mode and client mode connection.
 * ================================================================================================ */

/**
 * @brief Setup Access Point mode
 * 
 * Creates a WiFi network for initial configuration and fallback operation.
 */
void setupAPMode(void) {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_IP, AP_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    addLog("AP Mode: " + String(AP_SSID) + " @ " + WiFi.softAPIP().toString());
}

/**
 * @brief Connect to configured WiFi network (client mode)
 * 
 * Attempts to connect to the stored SSID/password.
 * Uses AP+STA mode to maintain access point while connecting.
 */
void connectToWiFi(void) {
    if (!wifi_sta_enabled || strlen(wifi_ssid) == 0) {
        addLog("WiFi client disabled");
        return;
    }
    
    addLog("Connecting to " + String(wifi_ssid));
    WiFi.mode(WIFI_AP_STA);  // Keep AP active while connecting
    WiFi.begin(wifi_ssid, wifi_password);
    
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT) {
        delay(500);
        setStatusLED(!digitalRead(PIN_STATUS_LED));  // Blink while connecting
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        addLog("WiFi connected: " + WiFi.localIP().toString());
        setWiFiLED(true);
    } else {
        addLog("WiFi connection failed - check credentials");
        setWiFiLED(false);
    }
}

/* ================================================================================================
 * BUTTON HANDLER
 * 
 * Handles physical button on GPIO0 for relay control with debouncing.
 * ================================================================================================ */

/**
 * @brief Initialize button pin
 * 
 * GPIO0 has internal pull-up, button connects to ground.
 */
void initButton(void) {
    pinMode(PIN_BUTTON, INPUT_PULLUP);
}

/**
 * @brief Handle button press with debouncing
 * 
 * Detects button press (HIGH→LOW transition), debounces, and toggles relay.
 * 
 * @note Short press (< 2 seconds): Toggle relay
 * @note Long press (> 2 seconds): Reserved for future features
 */
void handleButton(void) {
    bool currentState = digitalRead(PIN_BUTTON);
    
    // Button pressed (LOW)
    if (currentState == LOW && lastButtonState == HIGH) {
        if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
            lastDebounceTime = millis();
            buttonPressTime = millis();
            buttonWasPressed = true;
        }
    }
    
    // Button released (HIGH)
    if (currentState == HIGH && lastButtonState == LOW && buttonWasPressed) {
        unsigned long pressDuration = millis() - buttonPressTime;
        
        // Short press: toggle relay
        if (pressDuration < BUTTON_HOLD_TIME) {
            toggleRelay();
            addLog("Button pressed - Relay toggled");
        } else {
            // Long press: reserved for future features
            addLog("Button long press detected");
        }
        
        buttonWasPressed = false;
    }
    
    lastButtonState = currentState;
}

/* ================================================================================================
 * WEB SERVER - HTML (Complete offline-capable UI)
 * 
 * The HTML/CSS/JS is embedded in PROGMEM (flash memory) to save RAM.
 * All resources are inline so the UI works completely offline.
 * ================================================================================================ */

// HTML content is stored in PROGMEM to save RAM
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>POWR316D v1.0 - Energy Monitor</title>
    <style>
        *{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
        body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Helvetica Neue,Arial,sans-serif;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);min-height:100vh;padding:16px}
        .container{max-width:600px;margin:0 auto}
        .card{background:rgba(255,255,255,0.95);backdrop-filter:blur(10px);border-radius:28px;padding:20px;margin-bottom:16px;box-shadow:0 8px 32px rgba(0,0,0,0.1);transition:transform 0.2s}
        .card:hover{transform:translateY(-2px)}
        .header{text-align:center;background:linear-gradient(135deg,#667eea,#764ba2);color:white}
        .header h1{font-size:24px;font-weight:600;margin-bottom:4px}
        .version{font-size:12px;opacity:0.9;font-family:monospace}
        .status-card{background:linear-gradient(135deg,#667eea,#764ba2);color:white}
        .relay-status{text-align:center;padding:20px}
        .relay-indicator{width:100px;height:100px;margin:0 auto 20px;border-radius:50%;background:#f44336;display:flex;align-items:center;justify-content:center;transition:all 0.3s ease;box-shadow:0 4px 20px rgba(0,0,0,0.2)}
        .relay-indicator.on{background:#4CAF50;box-shadow:0 0 30px rgba(76,175,80,0.5)}
        .relay-indicator span{font-size:48px;font-weight:bold}
        .relay-text{font-size:28px;font-weight:bold;margin-bottom:20px}
        .button-group{display:flex;gap:12px;justify-content:center;flex-wrap:wrap}
        button{padding:12px 24px;border-radius:50px;font-size:16px;font-weight:600;cursor:pointer;border:none;transition:all 0.2s ease;font-family:inherit}
        button:hover{transform:scale(1.02);opacity:0.95}
        button:active{transform:scale(0.98)}
        .btn-primary{background:#4CAF50;color:white}
        .btn-danger{background:#f44336;color:white}
        .btn-warning{background:#ff9800;color:white}
        .btn-info{background:#2196F3;color:white}
        .btn-success{background:#28a745;color:white}
        .btn-test{background:#9c27b0;color:white}
        .btn-exit{background:#607d8b;color:white}
        .btn-bit{background:#795548;color:white;padding:8px 12px;font-size:14px}
        .info-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;margin:20px 0}
        .info-item{background:#f8f9fa;padding:16px;border-radius:16px;text-align:center}
        .info-label{font-size:12px;color:#666;margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px}
        .info-value{font-size:28px;font-weight:bold;color:#333}
        .log{background:#1e1e1e;color:#0f0;padding:12px;border-radius:12px;font-family:'Courier New',monospace;font-size:11px;max-height:200px;overflow-y:auto}
        .tab-buttons{display:flex;gap:8px;margin-bottom:20px;flex-wrap:wrap}
        .tab-btn{flex:1;background:#e5e7eb;padding:10px;border:none;border-radius:12px;cursor:pointer;font-weight:600;transition:all 0.2s;min-width:70px;font-size:14px}
        .tab-btn.active{background:linear-gradient(135deg,#667eea,#764ba2);color:white}
        .tab-pane{display:none;animation:fadeIn 0.3s}
        .tab-pane.active{display:block}
        @keyframes fadeIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
        .config-group{margin-bottom:20px}
        .config-group label{display:block;font-weight:600;margin-bottom:8px;color:#555}
        .config-group input{width:100%;padding:12px;border:2px solid #ddd;border-radius:12px;font-size:16px;transition:border-color 0.2s}
        .config-group input:focus{outline:none;border-color:#667eea}
        .lcd-test-group{display:flex;gap:12px;margin-bottom:16px;flex-wrap:wrap}
        .lcd-test-input{flex:2;min-width:150px}
        .lcd-test-input input{width:100%;padding:12px;border:2px solid #ddd;border-radius:12px;font-size:16px;text-align:center}
        .lcd-test-button{flex:1;min-width:100px}
        .lcd-test-button button{width:100%;padding:12px}
        .lcd-test-note{background:#e8f4f8;padding:12px;border-radius:12px;margin-bottom:16px;font-size:13px;border-left:4px solid #2196F3}
        .lcd-test-note code{background:#fff;padding:2px 6px;border-radius:6px;font-family:monospace}
        .lcd-test-status{margin-top:16px;padding:12px;border-radius:12px;text-align:center;font-weight:600}
        .lcd-test-status.test-active{background:#fff3cd;color:#856404}
        .lcd-test-status.test-inactive{background:#d4edda;color:#155724}
        .symbol-buttons{display:grid;grid-template-columns:repeat(2,1fr);gap:12px;margin-bottom:16px}
        .decimal-buttons{display:flex;gap:12px;margin-bottom:16px}
        .bit-tester{background:#f5f5f5;padding:16px;border-radius:16px;margin-top:16px}
        .bit-buttons{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}
        .buffer-selector{display:flex;gap:8px;flex-wrap:wrap;margin:12px 0}
        .buffer-btn{padding:8px 12px;background:#ddd;border:none;border-radius:8px;cursor:pointer;font-size:12px;transition:all 0.2s}
        .buffer-btn.active{background:#667eea;color:white}
        .calib-group{display:flex;gap:10px;margin-bottom:12px}
        .calib-group input{flex:2}
        .calib-group button{flex:1}
        .calib-info{background:#e8f4f8;padding:12px;border-radius:12px;margin-top:12px;font-size:13px}
        .calib-info span{font-weight:bold;color:#2196F3}
        .format-example{background:#e8f4f8;padding:12px;border-radius:12px;margin-top:12px;font-size:12px}
        .format-example code{background:#fff;padding:2px 6px;border-radius:4px;font-family:monospace}
        hr{margin:16px 0;border:none;border-top:1px solid #eee}
        .toast{position:fixed;bottom:20px;left:20px;right:20px;background:#333;color:white;padding:12px;border-radius:12px;text-align:center;z-index:1000;animation:slideUp 0.3s ease}
        @keyframes slideUp{from{transform:translateY(100px);opacity:0}to{transform:translateY(0);opacity:1}}
        @media (max-width:480px){body{padding:12px}.card{padding:16px}.info-value{font-size:22px}button{padding:10px 16px;font-size:14px}}
    </style>
</head>
<body>
<div class="container">
    <div class="card header">
        <h1>⚡ POWR316D Controller</h1>
        <div class="version">v1.0 | Smart Energy Monitor | GPIO0 Button Control</div>
    </div>
    
    <div class="card status-card">
        <div class="relay-status">
            <div class="relay-indicator" id="relayIndicator"><span>O</span></div>
            <div class="relay-text" id="relayText">OFF</div>
        </div>
        <div class="button-group">
            <button class="btn-primary" onclick="setRelay(1)">🔴 ON</button>
            <button class="btn-warning" onclick="toggleRelay()">🔄 TOGGLE</button>
            <button class="btn-danger" onclick="setRelay(0)">⚫ OFF</button>
        </div>
        <div style="text-align:center;margin-top:12px;font-size:12px;opacity:0.8">💡 GPIO0 button also toggles relay</div>
    </div>
    
    <div class="card">
        <div class="tab-buttons">
            <button class="tab-btn active" onclick="showTab('status')">📊 Status</button>
            <button class="tab-btn" onclick="showTab('lcdtest')">🖥️ LCD Test</button>
            <button class="tab-btn" onclick="showTab('bittester')">🔧 Bit Tester</button>
            <button class="tab-btn" onclick="showTab('calibrate')">🎯 Calibrate</button>
            <button class="tab-btn" onclick="showTab('wifi')">📡 WiFi</button>
            <button class="tab-btn" onclick="showTab('advanced')">⚙️ Advanced</button>
        </div>
        
        <div id="statusTab" class="tab-pane active">
            <div class="info-grid">
                <div class="info-item"><div class="info-label">⚡ Voltage</div><div class="info-value" id="voltage">0.0</div></div>
                <div class="info-item"><div class="info-label">🔌 Current</div><div class="info-value" id="current">0.000</div></div>
                <div class="info-item"><div class="info-label">💪 Power</div><div class="info-value" id="power">0.0</div></div>
                <div class="info-item"><div class="info-label">🔋 Energy</div><div class="info-value" id="energy">0.000</div></div>
            </div>
            <div class="info-item" style="margin-top:10px"><div class="info-label">📐 Power Factor</div><div class="info-value" id="powerfactor">1.00</div></div>
            <div class="format-example"><strong>📱 LCD Display Format (4-digit with leading zeros):</strong><br>
            <code>220V → "0220V"</code> | <code>221.3V → "221.3V"</code><br>
            <code>3A → "0003A"</code> | <code>2.2A → "002.2A"</code><br>
            <code>8.5W → "008.5W"</code> | <code>123.2W → "123.2W"</code> | <code>958W → "0958W"</code><br>
            <code>5.6kWh → "005.6kWh"</code> | <code>12.34kWh → "012.3kWh"</code></div>
        </div>
        
        <div id="lcdtestTab" class="tab-pane">
            <div class="config-group">
                <label>🎨 LCD Test Panel</label>
                <div class="lcd-test-note"><strong>✨ Decimal Support:</strong> Enter numbers like <code>222.8</code> or <code>220</code><br>
                <strong>📝 Format:</strong> 0-9999 or 0-999.9 (decimal auto-placed on 3rd digit)</div>
                <div class="symbol-buttons">
                    <button class="btn-info" onclick="toggleTopSymbol()" id="btnTopSymbol">🅅 Toggle Top Symbol (V/A)</button>
                    <button class="btn-info" onclick="toggleBottomSymbol()" id="btnBottomSymbol">🅆 Toggle Bottom Symbol (W/kWh)</button>
                </div>
                <div class="decimal-buttons">
                    <button class="btn-info" onclick="toggleAllSymbols(true)" style="flex:1">✅ Show All Symbols</button>
                    <button class="btn-info" onclick="toggleAllSymbols(false)" style="flex:1">❌ Hide All Symbols</button>
                </div>
                <div id="symbolStatus" style="font-size:12px;color:#666;text-align:center;margin-bottom:16px;padding:8px;background:#f0f0f0;border-radius:8px">Top Symbol: OFF | Bottom Symbol: OFF</div>
                <hr>
                <div class="lcd-test-group"><div class="lcd-test-input"><input type="text" id="topLineValue" placeholder="Top Line (e.g., 222.8)" value="222.8"></div><div class="lcd-test-button"><button class="btn-test" onclick="sendTopLine()">📤 Send Top Line</button></div></div>
                <div class="lcd-test-group"><div class="lcd-test-input"><input type="text" id="bottomLineValue" placeholder="Bottom Line (e.g., 2.2)" value="2.2"></div><div class="lcd-test-button"><button class="btn-test" onclick="sendBottomLine()">📤 Send Bottom Line</button></div></div>
                <div class="lcd-test-group"><div class="lcd-test-input"><input type="text" id="bothLinesValue" placeholder="Both (top/bottom)" value="222.8/2.2"></div><div class="lcd-test-button"><button class="btn-primary" onclick="sendBothLines()">📤 Send Both</button></div></div>
                <div class="lcd-test-group"><div class="lcd-test-input"><button class="btn-success" onclick="setTestPattern()" style="width:100%">🔢 Test Pattern (8888/8888)</button></div><div class="lcd-test-button"><button class="btn-exit" onclick="exitTestMode()">🚪 Exit Test Mode</button></div></div>
                <div class="lcd-test-status" id="testModeStatus">Status: Normal Mode</div>
            </div>
        </div>
        
        <div id="bittesterTab" class="tab-pane">
            <div class="config-group">
                <label>🔧 Manual LCD Bit Tester</label>
                <div class="lcd-test-note"><strong>📍 TM1621 Buffer Mapping:</strong><br>
                <code>Buffer[0]</code> Top Digit 1 | <code>Buffer[1]</code> Top Digit 2 | <code>Buffer[2]</code> Top Digit 3 + Top Decimal<br>
                <code>Buffer[3]</code> Top Digit 4 | <code>Buffer[4]</code> Bottom Digit 4 + W/kWh Symbol<br>
                <code>Buffer[5]</code> Bottom Digit 3 + Bottom Decimal | <code>Buffer[6]</code> Bottom Digit 2<br>
                <code>Buffer[7]</code> Bottom Digit 1 + V/A Symbol</div>
                <div class="bit-tester">
                    <div class="buffer-selector">
                        <button class="buffer-btn" onclick="selectBuffer(0)">Buf[0]</button><button class="buffer-btn" onclick="selectBuffer(1)">Buf[1]</button>
                        <button class="buffer-btn" onclick="selectBuffer(2)">Buf[2]</button><button class="buffer-btn" onclick="selectBuffer(3)">Buf[3]</button>
                        <button class="buffer-btn" onclick="selectBuffer(4)">Buf[4]</button><button class="buffer-btn" onclick="selectBuffer(5)">Buf[5]</button>
                        <button class="buffer-btn" onclick="selectBuffer(6)">Buf[6]</button><button class="buffer-btn" onclick="selectBuffer(7)">Buf[7]</button>
                    </div>
                    <div id="selectedBuffer" style="margin:10px 0;font-weight:bold;text-align:center">Selected: Buffer[0]</div>
                    <div class="bit-buttons">
                        <button class="btn-bit" onclick="testBit(0)">Bit0 (0x01)</button><button class="btn-bit" onclick="testBit(1)">Bit1 (0x02)</button>
                        <button class="btn-bit" onclick="testBit(2)">Bit2 (0x04)</button><button class="btn-bit" onclick="testBit(3)">Bit3 (0x08)</button>
                        <button class="btn-bit" onclick="testBit(4)">Bit4 (0x10)</button><button class="btn-bit" onclick="testBit(5)">Bit5 (0x20)</button>
                        <button class="btn-bit" onclick="testBit(6)">Bit6 (0x40)</button><button class="btn-bit" onclick="testBit(7)">Bit7 (0x80)</button>
                    </div>
                    <div style="margin:12px 0"><label><input type="checkbox" id="showReference" checked> Show reference pattern (8888)</label></div>
                    <div class="bit-buttons"><button class="btn-warning" onclick="startTester()">▶️ Start Tester</button><button class="btn-exit" onclick="stopTester()">⏹️ Stop Tester</button><button class="btn-primary" onclick="clearLCD()">🗑️ Clear LCD</button></div>
                </div>
            </div>
        </div>
        
        <div id="calibrateTab" class="tab-pane">
            <div class="config-group">
                <label>🎯 Calibration</label>
                <div class="calib-group"><input type="number" id="calVoltage" step="1" placeholder="Actual Voltage (V)"><button onclick="calibrateVoltage()" class="btn-success">Calibrate V</button></div>
                <div class="calib-group"><input type="number" id="calCurrent" step="0.01" placeholder="Actual Current (A)"><button onclick="calibrateCurrent()" class="btn-success">Calibrate I</button></div>
                <div class="calib-group"><input type="number" id="calPower" step="1" placeholder="Actual Power (W)"><button onclick="calibratePower()" class="btn-success">Calibrate P</button></div>
                <div class="calib-info">📊 Multipliers: V=<span id="voltageMult">1.0000</span> I=<span id="currentMult">1.0000</span> P=<span id="powerMult">1.0000</span></div>
            </div>
            <button onclick="resetCalibration()" class="btn-warning" style="width:100%;margin-top:10px">🔄 Reset to Defaults</button>
        </div>
        
        <div id="wifiTab" class="tab-pane">
            <div class="config-group"><label><input type="checkbox" id="wifiEnabled" onchange="onWifiChange()"> Enable WiFi Client</label></div>
            <div id="wifiConfig"><div class="config-group"><label>📡 SSID</label><input type="text" id="wifiSsid" placeholder="WiFi Name"></div>
            <div class="config-group"><label>🔒 Password</label><input type="password" id="wifiPassword" placeholder="Password"></div>
            <button onclick="saveWifi()" class="btn-info" style="width:100%">💾 Save and Reboot</button></div>
        </div>
        
        <div id="advancedTab" class="tab-pane">
            <button onclick="factoryReset()" class="btn-danger" style="width:100%;margin-bottom:10px">🔄 Factory Reset</button>
            <button onclick="reboot()" class="btn-warning" style="width:100%">🔄 Reboot Device</button>
            <div class="lcd-test-note" style="margin-top:16px"><strong>ℹ️ System Information:</strong><br>Firmware: v1.0<br>GPIO0 Button: Toggles relay (debounced)<br>LCD Buffer: 8 bytes<br>Energy Meter: CSE7759B @ 4800 baud 8E1</div>
        </div>
    </div>
    
    <div class="card"><h3 style="margin-bottom:12px">📋 System Log</h3><div id="log" class="log">Loading...</div></div>
</div>

<script>
let selectedBufferPos=0,testTopSymbolVisible=true,testBottomSymbolVisible=true;
function showToast(msg){let t=document.createElement('div');t.className='toast';t.textContent=msg;document.body.appendChild(t);setTimeout(()=>t.remove(),3000);}
function showTab(t){document.querySelectorAll('.tab-pane').forEach(p=>p.classList.remove('active'));document.querySelectorAll('.tab-btn').forEach(b=>b.classList.remove('active'));document.getElementById(t+'Tab').classList.add('active');event.target.classList.add('active');}
function selectBuffer(p){selectedBufferPos=p;document.getElementById('selectedBuffer').innerHTML=`Selected: Buffer[${p}]`;document.querySelectorAll('.buffer-btn').forEach(b=>b.classList.remove('active'));event.target.classList.add('active');}
async function testBit(b){const r=document.getElementById('showReference').checked;try{await fetch('/api/lcd/testbit',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({buffer:selectedBufferPos,bit:b,reference:r})});showToast(`Testing Buffer[${selectedBufferPos}] Bit ${b}`);}catch(e){showToast('Error');}}
async function startTester(){await fetch('/api/lcd/tester/start',{method:'POST'});showToast('Bit Tester Started');}
async function stopTester(){await fetch('/api/lcd/tester/stop',{method:'POST'});showToast('Bit Tester Stopped');fetchStatus();}
async function clearLCD(){await fetch('/api/lcd/clear',{method:'POST'});showToast('LCD Cleared');}
async function toggleTopSymbol(){testTopSymbolVisible=!testTopSymbolVisible;await fetch('/api/lcd/symbol/top',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({visible:testTopSymbolVisible})});updateSymbolButtons();showToast(`Top Symbol: ${testTopSymbolVisible?'ON':'OFF'}`);}
async function toggleBottomSymbol(){testBottomSymbolVisible=!testBottomSymbolVisible;await fetch('/api/lcd/symbol/bottom',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({visible:testBottomSymbolVisible})});updateSymbolButtons();showToast(`Bottom Symbol: ${testBottomSymbolVisible?'ON':'OFF'}`);}
async function toggleAllSymbols(s){testTopSymbolVisible=s;testBottomSymbolVisible=s;await fetch('/api/lcd/symbol/both',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({top:s,bottom:s})});updateSymbolButtons();showToast(s?'All Symbols ON':'All Symbols OFF');}
function updateSymbolButtons(){const tb=document.getElementById('btnTopSymbol'),bb=document.getElementById('btnBottomSymbol');if(tb)tb.style.background=testTopSymbolVisible?'#4CAF50':'#2196F3';if(bb)bb.style.background=testBottomSymbolVisible?'#4CAF50':'#2196F3';document.getElementById('symbolStatus').innerHTML=`Top Symbol: ${testTopSymbolVisible?'🟢 ON':'⚫ OFF'} | Bottom Symbol: ${testBottomSymbolVisible?'🟢 ON':'⚫ OFF'}`;}
async function fetchStatus(){try{const r=await fetch('/api/status'),d=await r.json();const ind=document.getElementById('relayIndicator'),txt=document.getElementById('relayText');if(d.relay){ind.classList.add('on');ind.querySelector('span').innerHTML='I';txt.innerHTML='ON';}else{ind.classList.remove('on');ind.querySelector('span').innerHTML='O';txt.innerHTML='OFF';}
document.getElementById('voltage').innerHTML=d.voltage.toFixed(1);document.getElementById('current').innerHTML=d.current.toFixed(3);document.getElementById('power').innerHTML=d.power.toFixed(1);document.getElementById('energy').innerHTML=d.energy.toFixed(3);document.getElementById('powerfactor').innerHTML=d.pf.toFixed(3);
document.getElementById('voltageMult').innerHTML=d.v_mult.toFixed(6);document.getElementById('currentMult').innerHTML=d.i_mult.toFixed(6);document.getElementById('powerMult').innerHTML=d.p_mult.toFixed(6);
document.getElementById('wifiEnabled').checked=d.wifiEnabled;onWifiChange();
if(d.testTopSymbolVisible!==undefined){testTopSymbolVisible=d.testTopSymbolVisible;testBottomSymbolVisible=d.testBottomSymbolVisible;updateSymbolButtons();}
const sd=document.getElementById('testModeStatus');if(d.testMode){sd.innerHTML='🔧 TEST MODE ACTIVE - LCD frozen for testing';sd.className='lcd-test-status test-active';}else{sd.innerHTML='✅ Normal Mode - LCD updates every 3 seconds';sd.className='lcd-test-status test-inactive';}}catch(e){console.error(e);}}
async function sendTopLine(){const v=document.getElementById('topLineValue').value.trim();if(!v){showToast('Enter value');return;}await fetch('/api/lcd/top',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:v})});fetchStatus();}
async function sendBottomLine(){const v=document.getElementById('bottomLineValue').value.trim();if(!v){showToast('Enter value');return;}await fetch('/api/lcd/bottom',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:v})});fetchStatus();}
async function sendBothLines(){const v=document.getElementById('bothLinesValue').value.trim();if(!v){showToast('Enter values');return;}const p=v.split('/');if(p.length!==2){showToast('Use top/bottom format');return;}await fetch('/api/lcd/both',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({top:p[0],bottom:p[1]})});fetchStatus();}
async function setTestPattern(){await fetch('/api/lcd/testpattern',{method:'POST'});showToast('Test pattern 8888/8888');fetchStatus();}
async function exitTestMode(){await fetch('/api/lcd/exit',{method:'POST'});showToast('Exited test mode');fetchStatus();}
async function calibrateVoltage(){const v=document.getElementById('calVoltage').value;if(!v){showToast('Enter voltage');return;}await fetch('/api/calibrate/voltage',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:parseFloat(v)})});fetchStatus();}
async function calibrateCurrent(){const v=document.getElementById('calCurrent').value;if(!v){showToast('Enter current');return;}await fetch('/api/calibrate/current',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:parseFloat(v)})});fetchStatus();}
async function calibratePower(){const v=document.getElementById('calPower').value;if(!v){showToast('Enter power');return;}await fetch('/api/calibrate/power',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({value:parseFloat(v)})});fetchStatus();}
async function resetCalibration(){if(confirm('Reset all calibration?')){await fetch('/api/calibrate/reset',{method:'POST'});fetchStatus();}}
async function setRelay(s){await fetch('/api/relay/'+s,{method:'POST'});fetchStatus();}
async function toggleRelay(){await fetch('/api/relay/toggle',{method:'POST'});fetchStatus();}
async function saveWifi(){const e=document.getElementById('wifiEnabled').checked,s=document.getElementById('wifiSsid').value.trim(),p=document.getElementById('wifiPassword').value;if(e&&!s){showToast('Enter SSID');return;}await fetch('/config/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({wifi_enabled:e,wifi_ssid:s,wifi_password:p})});showToast('WiFi saved, rebooting...');setTimeout(()=>location.reload(),3000);}
async function loadLog(){try{const r=await fetch('/log'),d=await r.json();document.getElementById('log').innerHTML=d.log.replace(/\n/g,'<br>');}catch(e){console.error(e);}}
function reboot(){if(confirm('Reboot device?'))fetch('/reboot');}
function factoryReset(){if(confirm('FACTORY RESET - ALL DATA LOST?'))fetch('/factoryreset');}
function onWifiChange(){const e=document.getElementById('wifiEnabled').checked;document.getElementById('wifiConfig').style.opacity=e?'1':'0.5';}
setInterval(fetchStatus,1000);setInterval(loadLog,2000);fetchStatus();loadLog();
</script>
</body>
</html>
)rawliteral";

/* ================================================================================================
 * WEB SERVER SETUP
 * ================================================================================================ */

void setupWebServer(void) {
    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", index_html); });
    
    server.on("/api/status", HTTP_GET, []() {
        readEnergyData();
        StaticJsonDocument<512> doc;
        doc["relay"] = relayState;
        doc["voltage"] = voltage;
        doc["current"] = current;
        doc["power"] = power;
        doc["energy"] = energyKWh;
        doc["pf"] = powerFactor;
        doc["v_mult"] = voltageMultiplier;
        doc["i_mult"] = currentMultiplier;
        doc["p_mult"] = powerMultiplier;
        doc["wifiEnabled"] = wifi_sta_enabled;
        doc["testMode"] = testModeActive;
        doc["testTopSymbolVisible"] = testShowTopSymbol;
        doc["testBottomSymbolVisible"] = testShowBottomSymbol;
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    server.on("/api/lcd/top", HTTP_POST, []() {
        if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));String v=d["value"];if(parseAndDisplayTop(v)){activateTestMode();server.send(200,"text/plain","OK");}else server.send(400,"text/plain","Invalid format");}
    });
    
    server.on("/api/lcd/bottom", HTTP_POST, []() {
        if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));String v=d["value"];if(parseAndDisplayBottom(v)){activateTestMode();server.send(200,"text/plain","OK");}else server.send(400,"text/plain","Invalid format");}
    });
    
    server.on("/api/lcd/both", HTTP_POST, []() {
        if(server.hasArg("plain")){StaticJsonDocument<192> d;deserializeJson(d,server.arg("plain"));String t=d["top"],b=d["bottom"];if(parseAndDisplayTop(t)&&parseAndDisplayBottom(b)){activateTestMode();server.send(200,"text/plain","OK");}else server.send(400,"text/plain","Invalid format");}
    });
    
    server.on("/api/lcd/testpattern", HTTP_POST, []() { testTopValue=8888;testBottomValue=8888;testTopHasDecimal=false;testBottomHasDecimal=false;activateTestMode();server.send(200,"text/plain","OK"); });
    server.on("/api/lcd/exit", HTTP_POST, []() { exitTestMode();server.send(200,"text/plain","OK"); });
    server.on("/api/lcd/clear", HTTP_POST, []() { clearBuffer();TM1621SendRows();server.send(200,"text/plain","OK"); });
    
    server.on("/api/lcd/symbol/top", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));testShowTopSymbol=d["visible"];if(testModeActive)showTestLCD(testTopValue,testBottomValue,testTopHasDecimal,testBottomHasDecimal,testShowTopSymbol,testShowBottomSymbol);server.send(200,"text/plain","OK");} });
    
    server.on("/api/lcd/symbol/bottom", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));testShowBottomSymbol=d["visible"];if(testModeActive)showTestLCD(testTopValue,testBottomValue,testTopHasDecimal,testBottomHasDecimal,testShowTopSymbol,testShowBottomSymbol);server.send(200,"text/plain","OK");} });
    
    server.on("/api/lcd/symbol/both", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));testShowTopSymbol=d["top"];testShowBottomSymbol=d["bottom"];if(testModeActive)showTestLCD(testTopValue,testBottomValue,testTopHasDecimal,testBottomHasDecimal,testShowTopSymbol,testShowBottomSymbol);server.send(200,"text/plain","OK");} });
    
    server.on("/api/lcd/tester/start", HTTP_POST, []() { startManualTester();server.send(200,"text/plain","OK"); });
    server.on("/api/lcd/tester/stop", HTTP_POST, []() { stopManualTester();server.send(200,"text/plain","OK"); });
    
    server.on("/api/lcd/testbit", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<64> d;deserializeJson(d,server.arg("plain"));int bp=d["buffer"],bit=d["bit"];bool ref=d["reference"]|true;testSingleBit(bp,bit,ref);server.send(200,"text/plain","OK");} });
    
    server.on("/api/calibrate/voltage", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));calibrateVoltage(d["value"]);server.send(200,"text/plain","OK");} });
    server.on("/api/calibrate/current", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));calibrateCurrent(d["value"]);server.send(200,"text/plain","OK");} });
    server.on("/api/calibrate/power", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<128> d;deserializeJson(d,server.arg("plain"));calibratePower(d["value"]);server.send(200,"text/plain","OK");} });
    server.on("/api/calibrate/reset", HTTP_POST, []() { resetCalibrationToDefault();server.send(200,"text/plain","OK"); });
    
    server.on("/api/relay/1", HTTP_POST, []() { setRelay(true);server.send(200,"text/plain","OK"); });
    server.on("/api/relay/0", HTTP_POST, []() { setRelay(false);server.send(200,"text/plain","OK"); });
    server.on("/api/relay/toggle", HTTP_POST, []() { toggleRelay();server.send(200,"text/plain","OK"); });
    
    server.on("/log", HTTP_GET, []() { StaticJsonDocument<1024> d;d["log"]=webLog;String r;serializeJson(d,r);server.send(200,"application/json",r); });
    
    server.on("/config/wifi", HTTP_POST, []() { if(server.hasArg("plain")){StaticJsonDocument<256> d;deserializeJson(d,server.arg("plain"));wifi_sta_enabled=d["wifi_enabled"]|false;String ssid=d["wifi_ssid"].as<String>(),pwd=d["wifi_password"].as<String>();ssid.toCharArray(wifi_ssid,sizeof(wifi_ssid));pwd.toCharArray(wifi_password,sizeof(wifi_password));saveConfigToFile();server.send(200,"text/plain","OK");if(wifi_sta_enabled&&strlen(wifi_ssid)>0)connectToWiFi();} });
    
    server.on("/reboot", HTTP_GET, []() { server.send(200,"text/plain","Rebooting...");ESP.restart(); });
    server.on("/factoryreset", HTTP_GET, []() { server.send(200,"text/plain","Factory reset...");factoryReset(); });
    
    server.onNotFound([]() { if(ENABLE_CAPTIVE_PORTAL){server.sendHeader("Location","http://192.168.4.1/",true);server.send(302,"text/plain","Redirect");} });
    
    server.begin();
    addLog("Web server started on port 80");
}

/* ================================================================================================
 * SYSTEM INITIALIZATION
 * ================================================================================================ */

void setup(void) {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    
    Serial.println();
    Serial.println("=================================================================================");
    Serial.println(" POWR316D COMPLETE CONTROLLER v" FIRMWARE_VERSION);
    Serial.println(" Professional Energy Monitoring System");
    Serial.println("=================================================================================");
    Serial.println(" LCD: TM1621 4-digit 7-segment display");
    Serial.println(" Meter: CSE7759B @ 4800 baud, 8E1");
    Serial.println(" Relay: GPIO13 (Active HIGH) | Button: GPIO0 (Active LOW)");
    Serial.println("=================================================================================");
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_WIFI_LED, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    
    setRelay(false);
    setWiFiLED(false);
    setStatusLED(false);
    
    TM1621Init();
    clearBuffer();
    showVoltageCurrent(230.0, 0.0);
    delay(1500);
    showEnergyPower(0.0, 0.0);
    delay(1500);
    clearBuffer();
    TM1621SendRows();
    
    loadConfigFromFile();
    loadCalibrationFromFile();
    
    setupAPMode();
    if(wifi_sta_enabled && strlen(wifi_ssid) > 0) connectToWiFi();
    
    initEnergyMeter();
    initButton();
    setupWebServer();
    
    if(ENABLE_OTA){
        ArduinoOTA.setHostname("POWR316D");
        ArduinoOTA.setPassword("admin123");
        ArduinoOTA.begin();
        addLog("OTA updates enabled - use admin123 password");
    }
    
    if(ENABLE_CAPTIVE_PORTAL){
        dnsServer.start(53, "*", AP_IP);
        addLog("DNS captive portal started");
    }
    
    for(int i=0;i<3;i++){setStatusLED(true);delay(200);setStatusLED(false);delay(200);}
    
    addLog("=================================================================================");
    addLog("System Ready - v" FIRMWARE_VERSION);
    addLog("AP Mode: " + String(AP_SSID) + " @ " + WiFi.softAPIP().toString());
    addLog("Web Interface: http://" + WiFi.softAPIP().toString());
    addLog("GPIO0 Button: Short press toggles relay");
    addLog("Power Display: <100W shows decimal (e.g., 8.5W → 008.5W)");
    addLog("=================================================================================");
}

/* ================================================================================================
 * MAIN LOOP
 * ================================================================================================ */

void loop(void) {
    if(ENABLE_OTA) ArduinoOTA.handle();
    server.handleClient();
    if(ENABLE_CAPTIVE_PORTAL) dnsServer.processNextRequest();
    readEnergyData();
    updateLCDDisplay();
    handleButton();
    delay(10);
}

/* ================================================================================================
 * END OF CODE
 * ================================================================================================ */