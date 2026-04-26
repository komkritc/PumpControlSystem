#include <SonoffS31.h>

SonoffS31 s31(12);  // Relay on GPIO12

void setup() {
    Serial.begin(115200);
    s31.begin();
    
    // Updated callback now includes power factor
    s31.onPowerUpdate([](float power, float voltage, float current, float powerFactor) {
        Serial.printf("Power: %.1fW, Voltage: %.1fV, Current: %.3fA, PF: %.3f\n", 
                      power, voltage, current, powerFactor);
    });
    
    s31.onRelayChange([](bool state) {
        Serial.printf("Relay changed to: %s\n", state ? "ON" : "OFF");
    });
    
    // Turn on relay
    s31.setRelay(true);
    
    Serial.println("Sonoff S31 Ready!");
    Serial.println("Power monitoring with Power Factor enabled\n");
}

void loop() {
    s31.update();  // Must call this regularly
    
    // Read measurements anytime
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        Serial.println("=== Current Readings ===");
        Serial.printf("  Real Power:     %.1f W\n", s31.getPower());
        Serial.printf("  Apparent Power: %.1f VA\n", s31.getApparentPower());
        Serial.printf("  Voltage:        %.1f V\n", s31.getVoltage());
        Serial.printf("  Current:        %.3f A\n", s31.getCurrent());
        Serial.printf("  Power Factor:   %.3f\n", s31.getPowerFactor());
        Serial.printf("  Energy:         %.3f kWh\n", s31.getEnergy());
        
        // Calculate efficiency or reactive power
        float reactivePower = sqrt(pow(s31.getApparentPower(), 2) - pow(s31.getPower(), 2));
        Serial.printf("  Reactive Power: %.1f VAR\n", reactivePower);
        
        // Power factor explanation
        if (s31.getPowerFactor() < 0.9 && s31.getPower() > 10) {
            Serial.println("  ⚠ Warning: Low power factor indicates reactive load!");
        } else if (s31.getPowerFactor() > 0.95) {
            Serial.println("  ✓ Good power factor");
        }
        
        Serial.println();
        lastPrint = millis();
    }
    
    // Example: Toggle relay every 15 seconds
    static unsigned long lastToggle = 0;
    if (millis() - lastToggle > 15000) {
        s31.toggleRelay();
        lastToggle = millis();
    }
    
    delay(10);
}