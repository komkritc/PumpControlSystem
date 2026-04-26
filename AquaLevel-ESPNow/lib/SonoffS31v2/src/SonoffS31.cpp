#include "SonoffS31.h"

SonoffS31::SonoffS31(int relayPin, int serialBaudRate) {
    _relayPin = relayPin;
    _serialBaudRate = serialBaudRate;
    _relayState = false;
    
    // Initialize measurements
    _voltage = 0.0;
    _current = 0.0;
    _power = 0.0;
    _energy = 0.0;
    _accumulatedEnergy = 0.0;
    _lastPower = 0.0;
    _powerFactor = 0.0;  // Default to 1.0 (perfect power factor)
    _lastEnergyCalc = 0;
    
    // Initialize CSE7766 variables
    _byteCounter = 0;
    _receiveFlag = false;
    _voltageCycle = 0;
    _currentCycle = 0;
    _powerCycle = 0;
    _powerCycleFirst = 0;
    _powerInvalidCount = 0;
    _lastPowerTime = 0;
    
    // Default calibration values (will be overwritten by chip)
    _voltageCal = 1912;
    _currentCal = 16140;
    _powerCal = 5364;
    _autoCalibrate = true;
    
    // Callbacks
    _powerCallback = nullptr;
    _relayCallback = nullptr;
    
    // Clear buffer
    memset(_rxBuffer, 0, sizeof(_rxBuffer));
}

void SonoffS31::begin() {
    // Initialize serial for CSE7766
    Serial.begin(_serialBaudRate, SERIAL_8E1);
    
    // Setup relay pin
    pinMode(_relayPin, OUTPUT);
    digitalWrite(_relayPin, LOW);
    _relayState = false;
    
    _lastEnergyCalc = millis();
    _lastPowerTime = millis();
    _powerInvalidCount = 0;
}

void SonoffS31::update() {
    // Read CSE7766 data
    while (Serial.available()) {
        int rb = Serial.read();
        if (rb != -1) {
            _cseSerialInput((uint8_t)rb);
        }
    }
}

void SonoffS31::setRelay(bool state) {
    _relayState = state;
    digitalWrite(_relayPin, _relayState ? HIGH : LOW);
    
    if (!_relayState) {
        // Reset readings when turning off
        _power = 0;
        _current = 0;
        _voltage = 0;
        _powerFactor = 0.0;
        _powerCycleFirst = 0;
        _powerInvalidCount = 0;
    }
    
    // Call callback if registered
    if (_relayCallback) {
        _relayCallback(_relayState);
    }
}

bool SonoffS31::getRelayState() {
    return _relayState;
}

void SonoffS31::toggleRelay() {
    setRelay(!_relayState);
}

float SonoffS31::getVoltage() {
    return _voltage;
}

float SonoffS31::getCurrent() {
    return _current;
}

float SonoffS31::getPower() {
    return _power;
}

float SonoffS31::getEnergy() {
    return _energy;
}

float SonoffS31::getPowerFactor() {
    return _powerFactor;
}

float SonoffS31::getApparentPower() {
    return _voltage * _current;
}

void SonoffS31::resetEnergy() {
    _accumulatedEnergy = 0;
    _energy = 0;
}

void SonoffS31::setVoltageCalibration(long cal) {
    _voltageCal = cal;
    _autoCalibrate = false;
}

void SonoffS31::setCurrentCalibration(long cal) {
    _currentCal = cal;
    _autoCalibrate = false;
}

void SonoffS31::setPowerCalibration(long cal) {
    _powerCal = cal;
    _autoCalibrate = false;
}

void SonoffS31::enableAutoCalibration(bool enable) {
    _autoCalibrate = enable;
}

void SonoffS31::onPowerUpdate(PowerUpdateCallback callback) {
    _powerCallback = callback;
}

void SonoffS31::onRelayChange(RelayChangeCallback callback) {
    _relayCallback = callback;
}

void SonoffS31::_calculatePowerFactor() {
    // Power Factor = Real Power / (Voltage × Current)
    // Clamped between 0 and 1
    float apparentPower = _voltage * _current;
    
    if (apparentPower > 0.01) {  // Avoid division by zero and very small values
        float calculatedPF = _power / apparentPower;
        // Clamp power factor between 0 and 1
        if (calculatedPF < 0) calculatedPF = 0;
        if (calculatedPF > 1) calculatedPF = 1;
        _powerFactor = calculatedPF;
    } else {
        // No load, power factor defaults to 0.0
        _powerFactor = 0.0;
    }
}

void SonoffS31::_cseReceived() {
    uint8_t header = _rxBuffer[0];
    
    if ((header & 0xFC) == 0xFC) {
        // Abnormal hardware
        return;
    }
    
    // Auto-calibration from chip
    if (_autoCalibrate) {
        if (_voltageCal == 1912 && header != 0xAA) {
            long voltage_coefficient = (_rxBuffer[2] << 16) | (_rxBuffer[3] << 8) | _rxBuffer[4];
            _voltageCal = voltage_coefficient / CSE_UREF;
        }
        
        if (_currentCal == 16140 && header != 0xAA) {
            long current_coefficient = (_rxBuffer[8] << 16) | (_rxBuffer[9] << 8) | _rxBuffer[10];
            _currentCal = current_coefficient;
        }
        
        if (_powerCal == 5364 && header != 0xAA) {
            long power_coefficient = (_rxBuffer[14] << 16) | (_rxBuffer[15] << 8) | _rxBuffer[16];
            _powerCal = power_coefficient / CSE_PREF;
        }
    }
    
    uint8_t adjustment = _rxBuffer[20];
    _voltageCycle = (_rxBuffer[5] << 16) | (_rxBuffer[6] << 8) | _rxBuffer[7];
    _currentCycle = (_rxBuffer[11] << 16) | (_rxBuffer[12] << 8) | _rxBuffer[13];
    _powerCycle = (_rxBuffer[17] << 16) | (_rxBuffer[18] << 8) | _rxBuffer[19];
    
    if (_relayState) {
        // Update voltage
        if ((adjustment & 0x40) && _voltageCycle > 0) {
            float newVoltage = (float)(_voltageCal * CSE_UREF) / (float)_voltageCycle;
            if (newVoltage > 50 && newVoltage < 300) {
                _voltage = newVoltage;
            }
        }
        
        // Update power
        if (adjustment & 0x10) {
            // Valid power reading
            _powerInvalidCount = 0;
            _lastPowerTime = millis();
            
            if ((header & 0xF2) == 0xF2) {
                _power = 0;
            } else {
                if (_powerCycleFirst == 0) {
                    _powerCycleFirst = _powerCycle;
                }
                if (_powerCycleFirst != _powerCycle) {
                    _powerCycleFirst = -1;
                    float newPower = (float)(_powerCal * CSE_PREF) / (float)_powerCycle;
                    if (newPower >= 0 && newPower < 5000) {
                        if (newPower > 0.5 || _power > 0) {
                            _power = newPower;
                        }
                    }
                }
            }
        } else {
            // Invalid power reading
            if (_powerInvalidCount < CSE_MAX_INVALID_POWER) {
                _powerInvalidCount++;
            }
            
            // Check timeout (2 seconds without valid reading)
            if (_powerInvalidCount >= CSE_MAX_INVALID_POWER || 
                (_lastPowerTime > 0 && millis() - _lastPowerTime > 2000)) {
                _powerCycleFirst = 0;
                _power = 0;
                _current = 0;
            }
        }
        
        // Update current
        if ((adjustment & 0x20) && _currentCycle > 0 && _power > 0.1) {
            float newCurrent = (float)_currentCal / (float)_currentCycle;
            if (newCurrent >= 0 && newCurrent < 20) {
                _current = newCurrent;
            }
        } else if (_power < 0.5) {
            _current = 0;
        }
        
        // Calculate power factor
        _calculatePowerFactor();
        
        // Calculate energy
        _calculateEnergy();
        
        // Call power callback if registered (now includes power factor)
        if (_powerCallback) {
            _powerCallback(_power, _voltage, _current, _powerFactor);
        }
        
    } else {
        // Relay is OFF
        _powerCycleFirst = 0;
        _powerInvalidCount = 0;
        _power = 0;
        _current = 0;
        _voltage = 0;
        _powerFactor = 0.0;
    }
}

bool SonoffS31::_cseSerialInput(uint8_t byte) {
    if (_receiveFlag) {
        _rxBuffer[_byteCounter++] = byte;
        if (CSE_PACKET_LEN == _byteCounter) {
            uint8_t checksum = 0;
            for (uint8_t i = 2; i < 23; i++) {
                checksum += _rxBuffer[i];
            }
            
            if (checksum == _rxBuffer[23]) {
                _cseReceived();
                _receiveFlag = false;
                _byteCounter = 0;
                return true;
            } else {
                // Try to re-sync
                for (int i = 1; i < _byteCounter; i++) {
                    if (_rxBuffer[i] == 0x5A) {
                        memmove(_rxBuffer, _rxBuffer + i, _byteCounter - i);
                        _byteCounter -= i;
                        _receiveFlag = true;
                        return false;
                    }
                }
                _receiveFlag = false;
                _byteCounter = 0;
            }
        }
    } else {
        if ((0x5A == byte) && (1 == _byteCounter)) {
            _receiveFlag = true;
        } else {
            _byteCounter = 0;
        }
        _rxBuffer[_byteCounter++] = byte;
    }
    return false;
}

void SonoffS31::_calculateEnergy() {
    unsigned long now = millis();
    if (_lastEnergyCalc > 0 && _power > 0.5) {
        float hours_elapsed = (now - _lastEnergyCalc) / 3600000.0;
        if (hours_elapsed > 0 && hours_elapsed < 1.0) {
            float avg_power = (_power + _lastPower) / 2.0;
            _accumulatedEnergy += (avg_power / 1000.0) * hours_elapsed;
            _energy = _accumulatedEnergy;
        }
    }
    _lastPower = _power;
    _lastEnergyCalc = now;
}