#ifndef SonoffS31_h
#define SonoffS31_h

#include <Arduino.h>

class SonoffS31 {
public:
    // Constructor
    SonoffS31(int relayPin = 12, int serialBaudRate = 4800);
    
    // Initialization
    void begin();
    void update();  // Call this in loop()
    
    // Relay control
    void setRelay(bool state);
    bool getRelayState();
    void toggleRelay();
    
    // Power measurements
    float getVoltage();     // Returns voltage in volts (V)
    float getCurrent();     // Returns current in amps (A)
    float getPower();       // Returns power in watts (W)
    float getEnergy();      // Returns accumulated energy in kilowatt-hours (kWh)
    float getPowerFactor(); // Returns power factor (0.0 to 1.0)
    float getApparentPower(); // Returns apparent power in VA (Volt-Amps)
    
    // Energy management
    void resetEnergy();
    
    // Calibration (optional - chip auto-calibrates by default)
    void setVoltageCalibration(long cal);
    void setCurrentCalibration(long cal);
    void setPowerCalibration(long cal);
    void enableAutoCalibration(bool enable);
    
    // Callback for power updates (now includes power factor)
    typedef std::function<void(float power, float voltage, float current, float powerFactor)> PowerUpdateCallback;
    void onPowerUpdate(PowerUpdateCallback callback);
    
    // Callback for relay changes
    typedef std::function<void(bool state)> RelayChangeCallback;
    void onRelayChange(RelayChangeCallback callback);

private:
    // Hardware
    int _relayPin;
    int _serialBaudRate;
    bool _relayState;
    
    // Measurements
    float _voltage;
    float _current;
    float _power;
    float _energy;
    float _accumulatedEnergy;
    float _lastPower;
    float _powerFactor;
    unsigned long _lastEnergyCalc;
    
    // CSE7766 specific
    uint8_t _rxBuffer[25];
    int _byteCounter;
    bool _receiveFlag;
    long _voltageCycle;
    long _currentCycle;
    long _powerCycle;
    long _powerCycleFirst;
    uint8_t _powerInvalidCount;
    unsigned long _lastPowerTime;
    
    // Calibration
    long _voltageCal;
    long _currentCal;
    long _powerCal;
    bool _autoCalibrate;
    
    // Constants
    static const uint8_t CSE_HEADER1 = 0x55;
    static const uint8_t CSE_HEADER2 = 0x5A;
    static const uint8_t CSE_PACKET_LEN = 24;
    static const uint8_t CSE_MAX_INVALID_POWER = 20;
    static const int CSE_PREF = 1000;
    static const int CSE_UREF = 100;
    
    // Callbacks
    PowerUpdateCallback _powerCallback;
    RelayChangeCallback _relayCallback;
    
    // Internal methods
    void _processByte(uint8_t b);
    void _cseReceived();
    bool _cseSerialInput(uint8_t byte);
    void _calculateEnergy();
    void _calculatePowerFactor();
};

#endif