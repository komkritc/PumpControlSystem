/**
 * ===============================================================
 *  POWR316D COMPLETE CONTROLLER v1.0
 *  ===============================================================
 *  
 *  A professional-grade smart relay controller with energy monitoring
 *  designed for the POWR316D hardware platform.
 *
 *  HARDWARE SUPPORT:
 *  - CSE7759B / CSE7766 Energy Meter IC (Sonoff POW R2/Elite/POWR316D)
 *  - TM1621 LCD Display (4-digit with symbols)
 *  - Relay Control (GPIO13)
 *  - Button Input (GPIO0)
 *  - WiFi LED (GPIO5)
 *  - Status LED (GPIO18)
 *
 *  FEATURES:
 *  - Real-time Voltage, Current, Power monitoring
 *  - Energy accumulation (kWh) with persistence
 *  - Web-based control and monitoring interface
 *  - Full calibration for V, I, P via web interface
 *  - WiFi configuration with live save functionality
 *  - OTA firmware updates
 *  - Captive portal for easy setup
 *  - Factory reset capability
 *  - Automatic byte order detection for CSE7759B
 *
 *  COMMUNICATION PROTOCOLS:
 *  - CSE7759B: 4800 baud, 8E1, RX only on GPIO16
 *  - Web Server: HTTP on port 80
 *  - OTA: Custom port for firmware updates
 *  - DNS: Captive portal on port 53
 *
 *  API ENDPOINTS:
 *  - GET  /                - Web interface
 *  - GET  /api/status      - Real-time energy data
 *  - POST /api/relay/[0/1] - Relay control
 *  - POST /api/calibrate/voltage - Voltage calibration
 *  - POST /api/calibrate/current - Current calibration
 *  - POST /api/calibrate/power   - Power calibration
 *  - POST /config/wifi     - WiFi configuration
 *  - GET  /reboot          - Reboot device
 *  - GET  /factoryreset    - Factory reset
 *  - GET  /log             - System log
 *  - GET  /api/debug       - Debug information
 *
 *  ===============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

/* ===============================================================
 *  VERSION & FEATURE CONFIGURATION
 *  =============================================================== */

#define FIRMWARE_VERSION       "1.0"
#define ENABLE_CAPTIVE_PORTAL  true
#define ENABLE_OTA             true

/* ===============================================================
 *  SYSTEM TIMING CONSTANTS
 *  =============================================================== */

#define WIFI_CONNECT_TIMEOUT    15000UL
#define REBOOT_COUNTDOWN_SECONDS 5UL
#define MIN_SAVE_INTERVAL_MS    2000UL
#define ENERGY_READ_INTERVAL    100UL
#define LCD_UPDATE_INTERVAL     3000UL

/* ===============================================================
 *  NETWORK CONFIGURATION
 *  =============================================================== */

const char* ap_ssid = "POWR316D_AP";
const char* ap_password = "12345678";
const IPAddress apIP(192, 168, 4, 1);

#define DEFAULT_WIFI_ENABLED    false
#define DEFAULT_WIFI_SSID       "redmi4xx2"
#define DEFAULT_WIFI_PASSWORD   "komkritc"

/* ===============================================================
 *  HARDWARE PIN DEFINITIONS
 *  =============================================================== */

#define PIN_BUTTON       0
#define PIN_RELAY        13
#define PIN_WIFI_LED     5
#define PIN_STATUS_LED   18
#define PIN_CSE_RX       16

/* ===============================================================
 *  LCD PIN DEFINITIONS
 *  =============================================================== */

#define PIN_LCD_DATA     14
#define PIN_LCD_WR       27
#define PIN_LCD_RD       26
#define PIN_LCD_CS       25

/* ===============================================================
 *  CSE7759B ENERGY METER CONSTANTS
 *  =============================================================== */

#define CSE_BAUD                 4800UL
#define CSE_UART_CONFIG          SERIAL_8E1
#define CSE_BUFFER_SIZE          25
#define CSE_MAX_INVALID_POWER    128

#define CSE_PREF                 1000UL
#define CSE_UREF                 100UL

// Frame Detection
#define CSE_FRAME_HEADER1        0x55
#define CSE_FRAME_HEADER2        0x5A
#define CSE_ABNORMAL_HEADER      0xF2

// Validation Bounds
#define CSE_MIN_VOLTAGE          80
#define CSE_MAX_VOLTAGE          300
#define CSE_MIN_CURRENT          0.0f
#define CSE_MAX_CURRENT          100.0f
#define CSE_MIN_POWER            0
#define CSE_MAX_POWER            25000

// Frame Offsets (24-byte frame)
#define OFFSET_HEADER            0
#define OFFSET_VOLTAGE_COEFF     2
#define OFFSET_VOLTAGE_CYCLE     5
#define OFFSET_CURRENT_COEFF     8
#define OFFSET_CURRENT_CYCLE     11
#define OFFSET_POWER_COEFF       14
#define OFFSET_POWER_CYCLE       17
#define OFFSET_STATUS            20
#define OFFSET_CF_PULSES_HIGH    21
#define OFFSET_CF_PULSES_LOW     22
#define OFFSET_CHECKSUM          23

// Status Flags
#define STATUS_VOLTAGE_VALID     0x40
#define STATUS_CURRENT_VALID     0x20
#define STATUS_POWER_VALID       0x10

// Default Calibration
#define DEFAULT_VOLTAGE_COEFF    28545UL
#define DEFAULT_CURRENT_COEFF    15440UL
#define DEFAULT_POWER_COEFF      4972000UL

/* ===============================================================
 *  TM1621 LCD CONSTANTS
 *  =============================================================== */

#define TM1621_PULSE_WIDTH      10UL
#define TM1621_PULSE_HALF       5UL

#define TM1621_SYS_EN           0x01
#define TM1621_LCD_ON           0x03
#define TM1621_TIMER_DIS        0x04
#define TM1621_WDT_DIS          0x05
#define TM1621_TONE_OFF         0x08
#define TM1621_BIAS             0x29
#define TM1621_IRQ_DIS          0x80

const uint8_t tm1621_commands[] = { 
    TM1621_SYS_EN, TM1621_LCD_ON, TM1621_BIAS, 
    TM1621_TIMER_DIS, TM1621_WDT_DIS, TM1621_TONE_OFF, TM1621_IRQ_DIS 
};

const uint8_t digitMap[] = {
    0x00, 0x5F, 0x50, 0x3D, 0x79, 0x72, 0x6B, 0x6F, 0x51, 0x7F, 0x7B, 0x20, 0x2F
};

#define SYMBOL_V_POS      0x08
#define SYMBOL_A_POS      0x08
#define SYMBOL_KWH_POS    0x08
#define SYMBOL_W_POS      0x08

/* ===============================================================
 *  GLOBAL VARIABLES
 *  =============================================================== */

WebServer server(80);
DNSServer dnsServer;

bool relayState = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

char wifi_ssid[64] = DEFAULT_WIFI_SSID;
char wifi_password[64] = DEFAULT_WIFI_PASSWORD;
bool wifi_sta_enabled = DEFAULT_WIFI_ENABLED;
bool wifiConnected = false;

bool rebootPending = false;
unsigned long rebootTime = 0;
bool saveInProgress = false;
unsigned long lastSaveTime = 0;
bool pendingSave = false;

// CSE7759B Data Structure
struct {
  uint32_t voltage_cycle = 0;
  uint32_t current_cycle = 0;
  uint32_t power_cycle = 0;
  uint32_t power_cycle_first = 0;
  uint32_t cf_pulses = 0;
  int32_t cf_pulses_last_time = -1;
  
  int byte_counter = 0;
  uint8_t rx_buffer[CSE_BUFFER_SIZE];
  uint8_t power_invalid = 0;
  bool received = false;
  bool initialized = false;
  
  uint32_t voltage_coefficient = DEFAULT_VOLTAGE_COEFF;
  uint32_t current_coefficient = DEFAULT_CURRENT_COEFF;
  uint32_t power_coefficient = DEFAULT_POWER_COEFF;
  
  uint32_t valid_frames = 0;
  uint32_t invalid_frames = 0;
} cse;

// Processed Values
float energyVoltage = 0;
float energyCurrent = 0;
float energyPower = 0;
float energyKWh = 0;
bool energyDataValid = false;

// LCD
uint8_t lcdBuffer[8];
bool lcdScreenToggle = false;
unsigned long lastLcdUpdate = 0;

// Web Log
String webLog = "";
const int MAX_LOG_LINES = 20;

HardwareSerial* cseSerial = nullptr;

/* ===============================================================
 *  UTILITY FUNCTIONS
 *  =============================================================== */

void addLog(const String& msg) {
    String timestamp = String(millis() / 1000);
    String logLine = "[" + timestamp + "] " + msg;
    
    if (webLog.length() > 0) webLog += "\n";
    webLog += logLine;
    
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
    
    Serial.println(msg);
}

/* ===============================================================
 *  CSE7759B ENERGY METER DRIVER
 *  =============================================================== */

uint32_t extract24Bit(uint8_t offset) {
    return ((uint32_t)cse.rx_buffer[offset] << 16) | 
           ((uint32_t)cse.rx_buffer[offset + 1] << 8) | 
           (uint32_t)cse.rx_buffer[offset + 2];
}

uint32_t extract16Bit(uint8_t offset) {
    return ((uint32_t)cse.rx_buffer[offset + 1] << 8) | 
           (uint32_t)cse.rx_buffer[offset + 2];
}

void cseReceived(void) {
    uint8_t header = cse.rx_buffer[OFFSET_HEADER];
    
    if ((header & 0xFC) == 0xFC) {
        return;
    }
    
    if (!cse.initialized) {
        if (header != 0xAA) {
            cse.voltage_coefficient = extract24Bit(OFFSET_VOLTAGE_COEFF);
            cse.current_coefficient = extract24Bit(OFFSET_CURRENT_COEFF);
            cse.power_coefficient = extract24Bit(OFFSET_POWER_COEFF);
            
            if (cse.voltage_coefficient < 10000 || cse.voltage_coefficient > 50000) {
                cse.voltage_coefficient = DEFAULT_VOLTAGE_COEFF;
            }
            if (cse.current_coefficient < 10000 || cse.current_coefficient > 30000) {
                cse.current_coefficient = DEFAULT_CURRENT_COEFF;
            }
            if (cse.power_coefficient < 1000000 || cse.power_coefficient > 10000000) {
                cse.power_coefficient = DEFAULT_POWER_COEFF;
            }
            
            addLog(String("CSE: Vcoeff=") + String(cse.voltage_coefficient) +
                   " Icoeff=" + String(cse.current_coefficient) +
                   " Pcoeff=" + String(cse.power_coefficient));
        }
        cse.initialized = true;
    }
    
    uint8_t status = cse.rx_buffer[OFFSET_STATUS];
    
    cse.voltage_cycle = extract16Bit(OFFSET_VOLTAGE_CYCLE);
    cse.current_cycle = extract16Bit(OFFSET_CURRENT_CYCLE);
    cse.power_cycle = extract16Bit(OFFSET_POWER_CYCLE);
    cse.cf_pulses = ((uint32_t)cse.rx_buffer[OFFSET_CF_PULSES_HIGH] << 8) | 
                    (uint32_t)cse.rx_buffer[OFFSET_CF_PULSES_LOW];
    
    cse.valid_frames++;
    
    if (status & STATUS_VOLTAGE_VALID) {
        if (cse.voltage_cycle > 0 && cse.voltage_cycle < 100000) {
            float calculatedVoltage = (float)(cse.voltage_coefficient * CSE_UREF) / (float)cse.voltage_cycle;
            if (calculatedVoltage >= CSE_MIN_VOLTAGE && calculatedVoltage <= CSE_MAX_VOLTAGE) {
                energyVoltage = calculatedVoltage;
                energyDataValid = true;
            }
        }
    }
    
    if (relayState && (status & STATUS_POWER_VALID)) {
        cse.power_invalid = 0;
        
        if (cse.power_cycle_first == 0) {
            cse.power_cycle_first = cse.power_cycle;
        }
        
        if (cse.power_cycle_first != cse.power_cycle && cse.power_cycle > 0) {
            cse.power_cycle_first = (uint32_t)-1;
            float calculatedPower = (float)(cse.power_coefficient * CSE_PREF) / (float)cse.power_cycle;
            if (calculatedPower >= CSE_MIN_POWER && calculatedPower <= CSE_MAX_POWER) {
                energyPower = calculatedPower;
            }
        }
    } else if (!relayState) {
        cse.power_cycle_first = 0;
        energyPower = 0;
        energyCurrent = 0;
    }
    
    if ((status & STATUS_CURRENT_VALID) && energyPower > 0.1f) {
        if (cse.current_cycle > 0) {
            float calculatedCurrent = (float)cse.current_coefficient / (float)cse.current_cycle;
            if (calculatedCurrent >= CSE_MIN_CURRENT && calculatedCurrent <= CSE_MAX_CURRENT) {
                energyCurrent = calculatedCurrent;
            }
        }
    } else if (energyPower < 0.1f) {
        energyCurrent = 0;
    }
    
    static uint32_t logCounter = 0;
    if (++logCounter >= 100 && energyDataValid) {
        logCounter = 0;
        addLog(String("Power: ") + String(energyVoltage,1) + "V, " +
               String(energyCurrent,3) + "A, " + String(energyPower,1) + "W");
    }
}

void cseSerialInput(void) {
    while (cseSerial->available()) {
        uint8_t byte = cseSerial->read();
        
        if (cse.received) {
            cse.rx_buffer[cse.byte_counter++] = byte;
            
            if (24 == cse.byte_counter) {
                uint8_t checksum = 0;
                for (int i = 2; i < 23; i++) {
                    checksum += cse.rx_buffer[i];
                }
                
                if (checksum == cse.rx_buffer[OFFSET_CHECKSUM]) {
                    cseReceived();
                } else {
                    cse.invalid_frames++;
                }
                
                cse.received = false;
                cse.byte_counter = 0;
            }
        } else {
            if ((CSE_FRAME_HEADER2 == byte) && (1 == cse.byte_counter)) {
                cse.received = true;
            } else {
                cse.byte_counter = 0;
            }
            cse.rx_buffer[cse.byte_counter++] = byte;
        }
    }
}

void cseEverySecond(void) {
    if (cse.cf_pulses_last_time == -1) {
        cse.cf_pulses_last_time = (int32_t)cse.cf_pulses;
        return;
    }
    
    uint32_t delta = 0;
    if (cse.cf_pulses < (uint32_t)cse.cf_pulses_last_time) {
        delta = (0x10000 - (uint32_t)cse.cf_pulses_last_time) + cse.cf_pulses;
    } else {
        delta = cse.cf_pulses - (uint32_t)cse.cf_pulses_last_time;
    }
    
    if (delta > 0 && energyPower > 0 && relayState) {
        float delta_kWh = (delta * (cse.power_coefficient / 1000.0f)) / 3600000.0f;
        energyKWh += delta_kWh;
        cse.cf_pulses_last_time = (int32_t)cse.cf_pulses;
        if (energyKWh > 99999.0f) energyKWh = 0.0f;
    }
}

void initEnergyMeter(void) {
    cseSerial = new HardwareSerial(1);
    cseSerial->begin(CSE_BAUD, CSE_UART_CONFIG, PIN_CSE_RX, -1);
    
    memset(cse.rx_buffer, 0, CSE_BUFFER_SIZE);
    cse.byte_counter = 0;
    cse.received = false;
    cse.initialized = false;
    cse.valid_frames = 0;
    cse.invalid_frames = 0;
    cse.power_cycle_first = 0;
    cse.cf_pulses_last_time = -1;
    
    addLog("CSE7759B: Initialized at 4800 baud, 8E1 on GPIO" + String(PIN_CSE_RX));
}

void readEnergyData(void) {
    static unsigned long lastSecond = 0;
    unsigned long now = millis();
    
    if (cseSerial) {
        cseSerialInput();
    }
    
    if (now - lastSecond >= 1000) {
        lastSecond = now;
        cseEverySecond();
    }
}

/* ===============================================================
 *  CALIBRATION FUNCTIONS
 *  =============================================================== */

void calibrateVoltage(float actualVoltage) {
    if (cse.voltage_cycle > 0 && actualVoltage >= 100 && actualVoltage <= 280) {
        uint32_t newCoeff = (uint32_t)(((float)cse.voltage_cycle * actualVoltage) / CSE_UREF);
        addLog(String("Voltage Calibration: Old=") + String(cse.voltage_coefficient) + 
               " New=" + String(newCoeff) + " Target=" + String(actualVoltage,1) + "V");
        cse.voltage_coefficient = newCoeff;
        saveCalibrationToFile();
    }
}

void calibrateCurrent(float actualCurrent) {
    if (cse.current_cycle > 0 && actualCurrent >= 0.1 && actualCurrent <= 100) {
        uint32_t newCoeff = (uint32_t)((float)cse.current_cycle * actualCurrent);
        addLog(String("Current Calibration: Old=") + String(cse.current_coefficient) + 
               " New=" + String(newCoeff) + " Target=" + String(actualCurrent,2) + "A");
        cse.current_coefficient = newCoeff;
        saveCalibrationToFile();
    }
}

void calibratePower(float actualPower) {
    if (cse.power_cycle > 0 && actualPower >= 10 && actualPower <= 25000) {
        uint32_t newCoeff = (uint32_t)(((float)cse.power_cycle * actualPower) / CSE_PREF);
        addLog(String("Power Calibration: Old=") + String(cse.power_coefficient) + 
               " New=" + String(newCoeff) + " Target=" + String(actualPower,1) + "W");
        cse.power_coefficient = newCoeff;
        saveCalibrationToFile();
    }
}

void saveCalibrationToFile(void) {
    StaticJsonDocument<256> doc;
    doc["v_coeff"] = cse.voltage_coefficient;
    doc["i_coeff"] = cse.current_coefficient;
    doc["p_coeff"] = cse.power_coefficient;
    
    File file = LittleFS.open("/calibration.json", "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        addLog("Calibration saved to flash");
    }
}

void loadCalibrationFromFile(void) {
    if (LittleFS.exists("/calibration.json")) {
        File file = LittleFS.open("/calibration.json", "r");
        if (file) {
            StaticJsonDocument<256> doc;
            deserializeJson(doc, file);
            cse.voltage_coefficient = doc["v_coeff"] | DEFAULT_VOLTAGE_COEFF;
            cse.current_coefficient = doc["i_coeff"] | DEFAULT_CURRENT_COEFF;
            cse.power_coefficient = doc["p_coeff"] | DEFAULT_POWER_COEFF;
            file.close();
            addLog("Calibration loaded from flash");
        }
    }
}

/* ===============================================================
 *  TM1621 LCD DRIVER
 *  =============================================================== */

void TM1621StopSequence(void) {
    digitalWrite(PIN_LCD_CS, HIGH);
    delayMicroseconds(TM1621_PULSE_HALF);
    digitalWrite(PIN_LCD_DATA, HIGH);
}

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

void TM1621Init(void) {
    pinMode(PIN_LCD_DATA, OUTPUT);
    pinMode(PIN_LCD_WR, OUTPUT);
    pinMode(PIN_LCD_RD, OUTPUT);
    pinMode(PIN_LCD_CS, OUTPUT);
    
    digitalWrite(PIN_LCD_DATA, HIGH);
    digitalWrite(PIN_LCD_CS, HIGH);
    digitalWrite(PIN_LCD_RD, HIGH);
    digitalWrite(PIN_LCD_WR, HIGH);
    
    digitalWrite(PIN_LCD_CS, LOW);
    delayMicroseconds(80);
    digitalWrite(PIN_LCD_RD, LOW);
    delayMicroseconds(15);
    digitalWrite(PIN_LCD_WR, LOW);
    delayMicroseconds(25);
    digitalWrite(PIN_LCD_DATA, LOW);
    delayMicroseconds(TM1621_PULSE_WIDTH);
    digitalWrite(PIN_LCD_DATA, HIGH);
    
    for (uint32_t cmd = 0; cmd < sizeof(tm1621_commands); cmd++) {
        TM1621SendCmnd(tm1621_commands[cmd]);
    }
    
    TM1621SendAddress(0x00);
    for (uint32_t segment = 0; segment < 16; segment++) {
        TM1621SendCommon(0);
    }
    TM1621StopSequence();
    
    memset(lcdBuffer, 0, sizeof(lcdBuffer));
    addLog("TM1621: LCD initialized");
}

void TM1621SendRows(void) {
    TM1621SendAddress(0x10);
    for (uint32_t i = 0; i < 8; i++) {
        TM1621SendCommon(lcdBuffer[i]);
    }
    TM1621StopSequence();
}

void clearBuffer(void) { memset(lcdBuffer, 0, sizeof(lcdBuffer)); }

uint8_t getDigit(int digit) {
    if (digit < 0 || digit > 9) return 0x00;
    return digitMap[digit + 1];
}

void setTopNumber(int value) {
    lcdBuffer[0] = getDigit((value / 1000) % 10);
    lcdBuffer[1] = getDigit((value / 100) % 10);
    lcdBuffer[2] = getDigit((value / 10) % 10);
    lcdBuffer[3] = getDigit(value % 10);
}

void setBottomNumber(int value) {
    lcdBuffer[7] = TM1621Row2(getDigit((value / 1000) % 10));
    lcdBuffer[6] = TM1621Row2(getDigit((value / 100) % 10));
    lcdBuffer[5] = TM1621Row2(getDigit((value / 10) % 10));
    lcdBuffer[4] = TM1621Row2(getDigit(value % 10));
}

void setVoltageSymbol(bool on) { if (on) lcdBuffer[7] |= SYMBOL_V_POS; else lcdBuffer[7] &= ~SYMBOL_V_POS; }
void setCurrentSymbol(bool on) { if (on) lcdBuffer[7] |= SYMBOL_A_POS; else lcdBuffer[7] &= ~SYMBOL_A_POS; }
void setPowerSymbol(bool on)   { if (on) lcdBuffer[4] |= SYMBOL_W_POS; else lcdBuffer[4] &= ~SYMBOL_W_POS; }
void setEnergySymbol(bool on)  { if (on) lcdBuffer[4] |= SYMBOL_KWH_POS; else lcdBuffer[4] &= ~SYMBOL_KWH_POS; }

void showVoltageCurrent(int voltage, float current) {
    clearBuffer();
    setTopNumber(voltage);
    setVoltageSymbol(true);
    setBottomNumber((int)(current * 100));
    setCurrentSymbol(true);
    TM1621SendRows();
}

void showEnergyPower(float energy, int power) {
    clearBuffer();
    setTopNumber((int)(energy * 100));
    setEnergySymbol(true);
    setBottomNumber(power);
    setPowerSymbol(true);
    TM1621SendRows();
}

void updateLCDDisplay(void) {
    if (millis() - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
        lastLcdUpdate = millis();
        if (!lcdScreenToggle) {
            showVoltageCurrent((int)energyVoltage, energyCurrent);
        } else {
            showEnergyPower(energyKWh, (int)energyPower);
        }
        lcdScreenToggle = !lcdScreenToggle;
    }
}

/* ===============================================================
 *  RELAY & LED CONTROL
 *  =============================================================== */

void setWiFiLED(bool state) { digitalWrite(PIN_WIFI_LED, state ? HIGH : LOW); }
void setStatusLED(bool state) { digitalWrite(PIN_STATUS_LED, state ? HIGH : LOW); }

void setRelay(bool state) {
    relayState = state;
    digitalWrite(PIN_RELAY, state ? HIGH : LOW);
    addLog(String("Relay: ") + (state ? "ON" : "OFF"));
    setWiFiLED(state);
    setStatusLED(true);
    delay(100);
    setStatusLED(false);
}

void toggleRelay(void) { setRelay(!relayState); }

/* ===============================================================
 *  CONFIGURATION MANAGEMENT
 *  =============================================================== */

void saveConfigToFile(void) {
    if (saveInProgress) { pendingSave = true; return; }
    
    unsigned long now = millis();
    if (now - lastSaveTime < MIN_SAVE_INTERVAL_MS) { pendingSave = true; return; }
    
    saveInProgress = true;
    lastSaveTime = now;
    
    StaticJsonDocument<512> doc;
    doc["wifi_sta_enabled"] = wifi_sta_enabled;
    doc["wifi_ssid"] = String(wifi_ssid);
    doc["wifi_password"] = String(wifi_password);
    doc["relay_state"] = relayState;
    doc["energy_kwh"] = energyKWh;
    
    File file = LittleFS.open("/config.json", "w");
    if (file) {
        serializeJson(doc, file);
        file.close();
        addLog("Config saved");
    }
    
    saveInProgress = false;
    pendingSave = false;
}

void loadConfigFromFile(void) {
    if (!LittleFS.begin()) {
        addLog("LittleFS: Mount failed, formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    
    if (!LittleFS.exists("/config.json")) {
        addLog("Config: No file, using defaults");
        wifi_sta_enabled = DEFAULT_WIFI_ENABLED;
        strcpy(wifi_ssid, DEFAULT_WIFI_SSID);
        strcpy(wifi_password, DEFAULT_WIFI_PASSWORD);
        saveConfigToFile();
        return;
    }
    
    File file = LittleFS.open("/config.json", "r");
    if (!file) return;
    
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        addLog("Config: Parse error, using defaults");
        wifi_sta_enabled = DEFAULT_WIFI_ENABLED;
        strcpy(wifi_ssid, DEFAULT_WIFI_SSID);
        strcpy(wifi_password, DEFAULT_WIFI_PASSWORD);
        return;
    }
    
    wifi_sta_enabled = doc["wifi_sta_enabled"] | DEFAULT_WIFI_ENABLED;
    String ssid = doc["wifi_ssid"].as<String>();
    String pwd = doc["wifi_password"].as<String>();
    ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
    pwd.toCharArray(wifi_password, sizeof(wifi_password));
    relayState = doc["relay_state"] | false;
    energyKWh = doc["energy_kwh"] | 0.0;
    
    addLog("Config loaded: WiFi " + String(wifi_sta_enabled ? "ENABLED" : "DISABLED"));
    if (strlen(wifi_ssid) > 0) {
        addLog("SSID: " + String(wifi_ssid));
    }
}

void factoryReset(void) {
    addLog("Factory reset...");
    if (LittleFS.exists("/config.json")) LittleFS.remove("/config.json");
    if (LittleFS.exists("/calibration.json")) LittleFS.remove("/calibration.json");
    delay(500);
    ESP.restart();
}

void rebootWithCountdown(void) {
    rebootPending = true;
    rebootTime = millis() + (REBOOT_COUNTDOWN_SECONDS * 1000);
    addLog(String("Reboot in ") + REBOOT_COUNTDOWN_SECONDS + "s");
}

void checkRebootCountdown(void) {
    if (rebootPending && millis() >= rebootTime) {
        rebootPending = false;
        ESP.restart();
    }
}

/* ===============================================================
 *  WIFI FUNCTIONS
 *  =============================================================== */

void setupAPMode(void) {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ap_ssid, ap_password);
    addLog("AP Mode: " + String(ap_ssid) + " @ " + WiFi.softAPIP().toString());
}

void connectToWiFi(void) {
    if (!wifi_sta_enabled || strlen(wifi_ssid) == 0) {
        addLog("WiFi client disabled");
        return;
    }
    
    addLog("Connecting to " + String(wifi_ssid));
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    unsigned long start = millis();
    
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT) {
        delay(500);
        setStatusLED(!digitalRead(PIN_STATUS_LED));
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        addLog("WiFi connected: " + WiFi.localIP().toString());
        setStatusLED(true);
        delay(200);
        setStatusLED(false);
    } else {
        wifiConnected = false;
        addLog("WiFi connection failed");
    }
}

/* ===============================================================
 *  BUTTON HANDLER
 *  =============================================================== */

void initButton(void) { pinMode(PIN_BUTTON, INPUT_PULLUP); }

void handleButton(void) {
    bool currentState = digitalRead(PIN_BUTTON);
    if (lastButtonState == HIGH && currentState == LOW) {
        if (millis() - lastDebounceTime > debounceDelay) {
            lastDebounceTime = millis();
            toggleRelay();
        }
    }
    lastButtonState = currentState;
}

/* ===============================================================
 *  WEB SERVER WITH COMPLETE CALIBRATION
 *  =============================================================== */

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>POWR316D v1.0</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'Segoe UI',Arial,sans-serif;background:#f1f5f9;padding:16px}
.container{max-width:550px;margin:0 auto}
.card{background:white;border-radius:32px;padding:20px;margin-bottom:18px;box-shadow:0 4px 12px rgba(0,0,0,0.05)}
.header{text-align:center}
h1{color:#333;margin-bottom:5px;font-size:24px}
.version{color:#666;font-size:12px}
.status-card{background:linear-gradient(135deg,#667eea,#764ba2);color:white}
.relay-status{text-align:center;padding:20px}
.relay-indicator{width:100px;height:100px;margin:0 auto 20px;border-radius:50%;background:#f44336;display:flex;align-items:center;justify-content:center;transition:all 0.3s}
.relay-indicator.on{background:#4CAF50}
.relay-indicator span{font-size:48px;font-weight:bold}
.relay-text{font-size:28px;font-weight:bold}
.button-group{display:flex;gap:15px;justify-content:center;margin:20px 0}
button{padding:12px 24px;border-radius:50px;font-size:16px;font-weight:600;cursor:pointer;border:none;transition:opacity 0.2s}
button:hover{opacity:0.9}
.btn-primary{background:#4CAF50;color:white}
.btn-danger{background:#f44336;color:white}
.btn-warning{background:#ff9800;color:white}
.btn-info{background:#2196F3;color:white}
.btn-success{background:#28a745;color:white}
.info-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:15px;margin:20px 0}
.info-item{background:#f8f9fa;padding:15px;border-radius:12px;text-align:center}
.info-label{font-size:12px;color:#666;margin-bottom:5px}
.info-value{font-size:28px;font-weight:bold;color:#333}
.log{background:#f8f9fa;padding:10px;border-radius:12px;font-family:monospace;font-size:11px;max-height:200px;overflow-y:auto}
.tab-buttons{display:flex;gap:10px;margin-bottom:20px;flex-wrap:wrap}
.tab-btn{flex:1;background:#e5e7eb;padding:12px;border:none;border-radius:10px;cursor:pointer;font-weight:bold;transition:all 0.2s;min-width:80px}
.tab-btn.active{background:#667eea;color:white}
.tab-pane{display:none}
.tab-pane.active{display:block}
.config-group{margin-bottom:20px}
.config-group label{display:block;font-weight:bold;margin-bottom:8px;color:#555}
.config-group input{width:100%;padding:12px;border:2px solid #ddd;border-radius:8px;font-size:16px}
.config-group input:focus{outline:none;border-color:#667eea}
.calib-group{display:flex;gap:10px;margin-bottom:10px}
.calib-group input{flex:2}
.calib-group button{flex:1}
.calib-info{background:#e8f4f8;padding:10px;border-radius:8px;margin-top:10px;font-size:13px}
.calib-info span{font-weight:bold;color:#2196F3}
.debug-info{background:#f8f9fa;padding:10px;border-radius:8px;font-family:monospace;font-size:11px;margin-top:10px;white-space:pre-wrap;word-break:break-all}
hr{margin:15px 0;border:none;border-top:1px solid #eee}
</style>
</head>
<body>
<div class="container">
<div class="card header">
<h1>POWR316D Controller</h1>
<div class="version">v1.0 | Professional Energy Monitor</div>
</div>

<div class="card status-card">
<div class="relay-status">
<div class="relay-indicator" id="relayIndicator"><span>O</span></div>
<div class="relay-text" id="relayText">OFF</div>
</div>
<div class="button-group">
<button class="btn-primary" onclick="setRelay(1)">ON</button>
<button class="btn-warning" onclick="toggleRelay()">TOGGLE</button>
<button class="btn-danger" onclick="setRelay(0)">OFF</button>
</div>
</div>

<div class="card">
<div class="tab-buttons">
<button class="tab-btn active" onclick="showTab('status')">Status</button>
<button class="tab-btn" onclick="showTab('calibrate')">Calibrate</button>
<button class="tab-btn" onclick="showTab('wifi')">WiFi</button>
<button class="tab-btn" onclick="showTab('advanced')">Advanced</button>
</div>

<div id="statusTab" class="tab-pane active">
<div class="info-grid">
<div class="info-item"><div class="info-label">Voltage</div><div class="info-value" id="voltage">0</div></div>
<div class="info-item"><div class="info-label">Current</div><div class="info-value" id="current">0.00</div></div>
<div class="info-item"><div class="info-label">Power</div><div class="info-value" id="power">0</div></div>
<div class="info-item"><div class="info-label">Energy</div><div class="info-value" id="energy">0.0</div></div>
</div>
<div class="info-item" style="margin-top:10px">
<div class="info-label">LCD Display</div>
<div class="info-value" style="font-size:14px">Top: V / kWh | Bottom: A / W</div>
</div>
</div>

<div id="calibrateTab" class="tab-pane">
<div class="config-group">
<label>Voltage Calibration (V)</label>
<div class="calib-group">
<input type="number" id="calVoltage" step="1" placeholder="Actual voltage (e.g., 220)">
<button onclick="calibrateVoltage()" class="btn-success">Calibrate V</button>
</div>
<div class="calib-info">Current reading: <span id="currentVoltage">0</span> V | Cycle: <span id="voltageCycle">0</span> | Coeff: <span id="voltageCoeff">0</span></div>
</div>

<div class="config-group">
<label>Current Calibration (A)</label>
<div class="calib-group">
<input type="number" id="calCurrent" step="0.1" placeholder="Actual current (e.g., 2.5)">
<button onclick="calibrateCurrent()" class="btn-success">Calibrate I</button>
</div>
<div class="calib-info">Current reading: <span id="currentCurrent">0</span> A | Cycle: <span id="currentCycle">0</span> | Coeff: <span id="currentCoeff">0</span></div>
</div>

<div class="config-group">
<label>Power Calibration (W)</label>
<div class="calib-group">
<input type="number" id="calPower" step="10" placeholder="Actual power (e.g., 500)">
<button onclick="calibratePower()" class="btn-success">Calibrate P</button>
</div>
<div class="calib-info">Power reading: <span id="currentPower">0</span> W | Cycle: <span id="powerCycle">0</span> | Coeff: <span id="powerCoeff">0</span></div>
</div>

<hr>
<div class="config-group">
<label>Debug Information</label>
<button onclick="showDebug()" class="btn-info">Show Debug Info</button>
<div id="debugInfo" class="debug-info" style="display:none"></div>
</div>
</div>

<div id="wifiTab" class="tab-pane">
<div class="config-group">
<label><input type="checkbox" id="wifiEnabled" onchange="onWifiChange()"> Enable WiFi Client</label>
</div>
<div id="wifiConfig">
<div class="config-group"><label>SSID</label><input type="text" id="wifiSsid" placeholder="WiFi Name" autocomplete="off"></div>
<div class="config-group"><label>Password</label><input type="password" id="wifiPassword" placeholder="Password" autocomplete="off"></div>
<button onclick="saveWifi()" class="btn-info">Save and Reboot</button>
</div>
<div id="rebootMsg" style="display:none;margin-top:15px;padding:10px;background:#fef3c7;border-radius:10px;text-align:center">Rebooting in <span id="countdown">5</span> seconds...</div>
</div>

<div id="advancedTab" class="tab-pane">
<button onclick="factoryReset()" class="btn-danger" style="width:100%;margin-bottom:10px">Factory Reset</button>
<button onclick="reboot()" class="btn-warning" style="width:100%">Reboot Device</button>
</div>
</div>

<div class="card">
<h3>System Log</h3>
<div id="log" class="log">Loading...</div>
</div>
</div>

<script>
let countdownInterval = null;

function showTab(tabName) {
    document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(tabName + 'Tab').classList.add('active');
    event.target.classList.add('active');
}

function onWifiChange() {
    const enabled = document.getElementById('wifiEnabled').checked;
    document.getElementById('wifiConfig').style.opacity = enabled ? '1' : '0.5';
}

async function fetchStatus() {
    try {
        const res = await fetch('/api/status');
        const d = await res.json();
        
        const ind = document.getElementById('relayIndicator');
        const txt = document.getElementById('relayText');
        if(d.relay) {
            ind.classList.add('on');
            ind.querySelector('span').innerHTML = 'I';
            txt.innerHTML = 'ON';
        } else {
            ind.classList.remove('on');
            ind.querySelector('span').innerHTML = 'O';
            txt.innerHTML = 'OFF';
        }
        
        document.getElementById('voltage').innerHTML = d.voltage.toFixed(1);
        document.getElementById('current').innerHTML = d.current.toFixed(3);
        document.getElementById('power').innerHTML = Math.round(d.power);
        document.getElementById('energy').innerHTML = d.energy.toFixed(2);
        
        document.getElementById('currentVoltage').innerHTML = d.voltage.toFixed(1);
        document.getElementById('voltageCycle').innerHTML = d.voltage_cycle;
        document.getElementById('voltageCoeff').innerHTML = d.voltage_coeff;
        
        document.getElementById('currentCurrent').innerHTML = d.current.toFixed(3);
        document.getElementById('currentCycle').innerHTML = d.current_cycle;
        document.getElementById('currentCoeff').innerHTML = d.current_coeff;
        
        document.getElementById('currentPower').innerHTML = Math.round(d.power);
        document.getElementById('powerCycle').innerHTML = d.power_cycle;
        document.getElementById('powerCoeff').innerHTML = d.power_coeff;
        
        document.getElementById('wifiEnabled').checked = d.wifiEnabled;
        document.getElementById('wifiSsid').value = d.wifiSsid;
        onWifiChange();
    } catch(e) {
        console.error('Fetch status error:', e);
    }
}

async function calibrateVoltage() {
    const voltage = document.getElementById('calVoltage').value;
    if (!voltage) { alert('Enter actual voltage'); return; }
    try {
        const res = await fetch('/api/calibrate/voltage', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({value: parseFloat(voltage)})
        });
        if (res.ok) { alert('Voltage calibrated!'); fetchStatus(); }
        else { alert('Calibration failed'); }
    } catch(e) { alert('Error: ' + e.message); }
}

async function calibrateCurrent() {
    const current = document.getElementById('calCurrent').value;
    if (!current) { alert('Enter actual current'); return; }
    try {
        const res = await fetch('/api/calibrate/current', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({value: parseFloat(current)})
        });
        if (res.ok) { alert('Current calibrated!'); fetchStatus(); }
        else { alert('Calibration failed'); }
    } catch(e) { alert('Error: ' + e.message); }
}

async function calibratePower() {
    const power = document.getElementById('calPower').value;
    if (!power) { alert('Enter actual power'); return; }
    try {
        const res = await fetch('/api/calibrate/power', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({value: parseFloat(power)})
        });
        if (res.ok) { alert('Power calibrated!'); fetchStatus(); }
        else { alert('Calibration failed'); }
    } catch(e) { alert('Error: ' + e.message); }
}

async function showDebug() {
    try {
        const res = await fetch('/api/debug');
        const d = await res.json();
        const debugDiv = document.getElementById('debugInfo');
        debugDiv.style.display = 'block';
        debugDiv.innerHTML = JSON.stringify(d, null, 2);
    } catch(e) { alert('Error: ' + e.message); }
}

async function setRelay(s) { 
    try {
        await fetch('/api/relay/'+s, {method:'POST'});
        fetchStatus();
    } catch(e) { console.error(e); }
}

async function toggleRelay() { 
    try {
        await fetch('/api/relay/toggle', {method:'POST'});
        fetchStatus();
    } catch(e) { console.error(e); }
}

async function saveWifi() {
    const enabled = document.getElementById('wifiEnabled').checked;
    const ssid = document.getElementById('wifiSsid').value.trim();
    const pwd = document.getElementById('wifiPassword').value;
    
    if(enabled && !ssid) { alert('Enter SSID'); return; }
    
    try {
        const res = await fetch('/config/wifi', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({wifi_enabled: enabled, wifi_ssid: ssid, wifi_password: pwd})
        });
        
        if(res.ok) {
            let cnt=5;
            const div=document.getElementById('rebootMsg');
            const span=document.getElementById('countdown');
            div.style.display='block';
            if(countdownInterval) clearInterval(countdownInterval);
            countdownInterval=setInterval(()=>{
                cnt--;
                span.innerText=cnt;
                if(cnt<0){
                    clearInterval(countdownInterval);
                    location.reload();
                }
            },1000);
        } else { 
            const errorText = await res.text();
            alert('Save failed: ' + errorText);
        }
    } catch(e) { 
        alert('Error: ' + e.message);
    }
}

async function loadLog() {
    try {
        const res = await fetch('/log');
        const d = await res.json();
        document.getElementById('log').innerHTML = d.log.replace(/\n/g,'<br>');
    } catch(e) { console.error(e); }
}

function reboot() { if(confirm('Reboot device?')) fetch('/reboot'); }
function factoryReset() { if(confirm('FACTORY RESET? All settings will be lost!')) fetch('/factoryreset'); }

setInterval(fetchStatus, 1000);
setInterval(loadLog, 2000);
fetchStatus();
loadLog();
</script>
</body>
</html>
)rawliteral";

void setupWebServer(void) {
    server.on("/", HTTP_GET, []() { 
        server.send_P(200, "text/html", index_html); 
    });
    
    server.on("/api/status", HTTP_GET, []() {
        readEnergyData();
        StaticJsonDocument<1024> doc;
        doc["relay"] = relayState;
        doc["voltage"] = energyVoltage;
        doc["current"] = energyCurrent;
        doc["power"] = energyPower;
        doc["energy"] = energyKWh;
        doc["wifiEnabled"] = wifi_sta_enabled;
        doc["wifiSsid"] = String(wifi_ssid);
        doc["voltage_cycle"] = cse.voltage_cycle;
        doc["voltage_coeff"] = cse.voltage_coefficient;
        doc["current_cycle"] = cse.current_cycle;
        doc["current_coeff"] = cse.current_coefficient;
        doc["power_cycle"] = cse.power_cycle;
        doc["power_coeff"] = cse.power_coefficient;
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    server.on("/api/calibrate/voltage", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            StaticJsonDocument<128> doc;
            deserializeJson(doc, server.arg("plain"));
            if (doc.containsKey("value")) {
                calibrateVoltage(doc["value"]);
                server.send(200, "text/plain", "OK");
            } else {
                server.send(400, "text/plain", "Missing value");
            }
        } else {
            server.send(400, "text/plain", "No data");
        }
    });
    
    server.on("/api/calibrate/current", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            StaticJsonDocument<128> doc;
            deserializeJson(doc, server.arg("plain"));
            if (doc.containsKey("value")) {
                calibrateCurrent(doc["value"]);
                server.send(200, "text/plain", "OK");
            } else {
                server.send(400, "text/plain", "Missing value");
            }
        } else {
            server.send(400, "text/plain", "No data");
        }
    });
    
    server.on("/api/calibrate/power", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            StaticJsonDocument<128> doc;
            deserializeJson(doc, server.arg("plain"));
            if (doc.containsKey("value")) {
                calibratePower(doc["value"]);
                server.send(200, "text/plain", "OK");
            } else {
                server.send(400, "text/plain", "Missing value");
            }
        } else {
            server.send(400, "text/plain", "No data");
        }
    });
    
    server.on("/api/debug", HTTP_GET, []() {
        StaticJsonDocument<512> doc;
        doc["voltage_cycle"] = cse.voltage_cycle;
        doc["current_cycle"] = cse.current_cycle;
        doc["power_cycle"] = cse.power_cycle;
        doc["voltage_coeff"] = cse.voltage_coefficient;
        doc["current_coeff"] = cse.current_coefficient;
        doc["power_coeff"] = cse.power_coefficient;
        doc["voltage"] = energyVoltage;
        doc["current"] = energyCurrent;
        doc["power"] = energyPower;
        doc["energy"] = energyKWh;
        doc["valid_frames"] = cse.valid_frames;
        doc["invalid_frames"] = cse.invalid_frames;
        doc["relay"] = relayState;
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    server.on("/api/relay/1", HTTP_POST, []() { setRelay(true); server.send(200, "text/plain", "OK"); });
    server.on("/api/relay/0", HTTP_POST, []() { setRelay(false); server.send(200, "text/plain", "OK"); });
    server.on("/api/relay/toggle", HTTP_POST, []() { toggleRelay(); server.send(200, "text/plain", "OK"); });
    
    server.on("/log", HTTP_GET, []() {
        StaticJsonDocument<1024> doc;
        doc["log"] = webLog;
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response);
    });
    
    server.on("/config/wifi", HTTP_POST, []() {
        if (server.hasArg("plain")) {
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, server.arg("plain"));
            
            if (!error) {
                if (doc.containsKey("wifi_enabled")) {
                    wifi_sta_enabled = doc["wifi_enabled"];
                }
                if (doc.containsKey("wifi_ssid")) {
                    String ssid = doc["wifi_ssid"].as<String>();
                    ssid.toCharArray(wifi_ssid, sizeof(wifi_ssid));
                    addLog("WiFi SSID set to: " + String(wifi_ssid));
                }
                if (doc.containsKey("wifi_password")) {
                    String pwd = doc["wifi_password"].as<String>();
                    pwd.toCharArray(wifi_password, sizeof(wifi_password));
                }
                
                saveConfigToFile();
                server.send(200, "text/plain", "OK");
                
                if (wifi_sta_enabled && strlen(wifi_ssid) > 0) {
                    connectToWiFi();
                    delay(500);
                    rebootWithCountdown();
                }
            } else {
                server.send(400, "text/plain", "Invalid JSON");
            }
        } else {
            server.send(400, "text/plain", "No data");
        }
    });
    
    server.on("/reboot", HTTP_GET, []() { 
        server.send(200, "text/plain", "Rebooting..."); 
        delay(100); 
        ESP.restart(); 
    });
    
    server.on("/factoryreset", HTTP_GET, []() { 
        server.send(200, "text/plain", "Factory resetting..."); 
        delay(100); 
        factoryReset(); 
    });
    
    server.onNotFound([]() {
        if (ENABLE_CAPTIVE_PORTAL) {
            server.sendHeader("Location", "http://192.168.4.1/", true);
            server.send(302, "text/plain", "Redirecting...");
        } else {
            server.send(404, "text/plain", "Not found");
        }
    });
    
    server.begin();
    addLog("Web server started on http://192.168.4.1");
}

/* ===============================================================
 *  SYSTEM INITIALIZATION
 *  =============================================================== */

void setup(void) {
    Serial.begin(115200);
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_WIFI_LED, OUTPUT);
    pinMode(PIN_STATUS_LED, OUTPUT);
    
    setRelay(false);
    setWiFiLED(false);
    setStatusLED(false);
    
    TM1621Init();
    
    clearBuffer();
    showVoltageCurrent(8888, 8.88);
    delay(1500);
    showEnergyPower(88.8, 888);
    delay(1500);
    clearBuffer();
    
    loadConfigFromFile();
    loadCalibrationFromFile();
    
    setupAPMode();
    if (wifi_sta_enabled && strlen(wifi_ssid) > 0) {
        connectToWiFi();
    }
    
    initEnergyMeter();
    initButton();
    setupWebServer();
    
    if (ENABLE_OTA) {
        ArduinoOTA.setHostname("POWR316D");
        ArduinoOTA.setPassword("admin123");
        ArduinoOTA.begin();
        addLog("OTA updates enabled");
    }
    
    if (ENABLE_CAPTIVE_PORTAL) {
        dnsServer.start(53, "*", apIP);
        addLog("DNS captive portal started");
    }
    
    for(int i = 0; i < 3; i++) {
        setStatusLED(true);
        delay(200);
        setStatusLED(false);
        delay(200);
    }
    
    addLog("=== System Ready v" FIRMWARE_VERSION " ===");
    addLog("AP Mode: " + String(ap_ssid) + " @ " + WiFi.softAPIP().toString());
    if (WiFi.status() == WL_CONNECTED) {
        addLog("STA Mode: " + WiFi.localIP().toString());
    }
    addLog("CSE7759B: 4800 baud, 8E1 on GPIO" + String(PIN_CSE_RX));
    addLog("TM1621: LCD initialized");
}

/* ===============================================================
 *  MAIN LOOP
 *  =============================================================== */

void loop(void) {
    if (ENABLE_OTA) ArduinoOTA.handle();
    server.handleClient();
    if (ENABLE_CAPTIVE_PORTAL) dnsServer.processNextRequest();
    
    readEnergyData();
    updateLCDDisplay();
    handleButton();
    checkRebootCountdown();
    
    if (pendingSave && !saveInProgress && millis() - lastSaveTime >= MIN_SAVE_INTERVAL_MS) {
        saveConfigToFile();
    }
    
    delay(10);
}

/* ===============================================================
 *  END OF CODE
 *  =============================================================== */