#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <SonoffS31.h>

// WiFi Configuration
const char* ssid = "redmi4xx";
const char* password = "komkritc";

// Device name
String deviceName = "s31";

// Create SonoffS31 instance (relay on GPIO12)
SonoffS31 s31(12);

// Web Server
ESP8266WebServer server(80);

// HTML Dashboard with Power Factor support
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <title>Sonoff S31</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        
        .card {
            background: white;
            border-radius: 20px;
            padding: 25px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        
        h1 {
            text-align: center;
            color: #333;
            margin-bottom: 5px;
            font-size: 24px;
        }
        
        .subtitle {
            text-align: center;
            color: #666;
            font-size: 11px;
            margin-bottom: 25px;
            padding-bottom: 10px;
            border-bottom: 1px solid #eee;
        }
        
        .power-display {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 15px;
            padding: 20px;
            text-align: center;
            margin-bottom: 20px;
            color: white;
        }
        
        .power-label {
            font-size: 14px;
            opacity: 0.9;
            margin-bottom: 5px;
        }
        
        .power-value {
            font-size: 48px;
            font-weight: bold;
        }
        
        .power-unit {
            font-size: 20px;
        }
        
        .stats {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-bottom: 20px;
        }
        
        .stat {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 12px;
            text-align: center;
        }
        
        .stat-label {
            font-size: 11px;
            color: #666;
            margin-bottom: 5px;
            text-transform: uppercase;
        }
        
        .stat-value {
            font-size: 22px;
            font-weight: bold;
            color: #333;
        }
        
        .stat-unit {
            font-size: 11px;
            color: #666;
        }
        
        .pf-card {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 12px;
            text-align: center;
            margin-bottom: 20px;
        }
        
        .pf-label {
            font-size: 12px;
            color: #666;
            margin-bottom: 5px;
        }
        
        .pf-value {
            font-size: 28px;
            font-weight: bold;
        }
        
        .pf-good { color: #10B981; }
        .pf-warning { color: #F59E0B; }
        .pf-bad { color: #EF4444; }
        
        .apparent-card {
            background: #f5f5f5;
            padding: 15px;
            border-radius: 12px;
            text-align: center;
            margin-bottom: 20px;
        }
        
        .apparent-label {
            font-size: 12px;
            color: #666;
            margin-bottom: 5px;
        }
        
        .apparent-value {
            font-size: 24px;
            font-weight: bold;
            color: #333;
        }
        
        .relay-btn {
            width: 120px;
            height: 120px;
            border-radius: 60px;
            border: none;
            font-size: 22px;
            font-weight: bold;
            cursor: pointer;
            transition: all 0.3s;
            margin: 10px auto 20px;
            display: block;
            box-shadow: 0 5px 20px rgba(0,0,0,0.2);
        }
        
        .relay-btn.on {
            background: #10B981;
            color: white;
            box-shadow: 0 5px 20px rgba(16,185,129,0.4);
        }
        
        .relay-btn.off {
            background: #EF4444;
            color: white;
            box-shadow: 0 5px 20px rgba(239,68,68,0.4);
        }
        
        .relay-btn:active {
            transform: scale(0.95);
        }
        
        .info {
            background: #f8f9fa;
            padding: 12px;
            border-radius: 10px;
            font-size: 11px;
            color: #666;
            text-align: center;
            margin-top: 20px;
        }
        
        .status {
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 4px;
            margin-right: 5px;
        }
        
        .status.online { background: #10B981; animation: pulse 2s infinite; }
        .status.offline { background: #EF4444; }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        
        .refresh {
            font-size: 10px;
            color: #999;
            margin-top: 10px;
            text-align: center;
        }
        
        @media (max-width: 480px) {
            .power-value { font-size: 36px; }
            .stat-value { font-size: 18px; }
            .relay-btn { width: 100px; height: 100px; font-size: 18px; }
            .stats { grid-template-columns: repeat(3, 1fr); gap: 10px; }
            .stat { padding: 10px; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>Sonoff S31</h1>
            <div class="subtitle" id="deviceId">Loading...</div>
            
            <div class="power-display">
                <div class="power-label">Real Power</div>
                <div class="power-value">
                    <span id="power">0</span><span class="power-unit">W</span>
                </div>
            </div>
            
            <div class="stats">
                <div class="stat">
                    <div class="stat-label">Voltage</div>
                    <div class="stat-value">
                        <span id="voltage">0</span><span class="stat-unit">V</span>
                    </div>
                </div>
                <div class="stat">
                    <div class="stat-label">Current</div>
                    <div class="stat-value">
                        <span id="current">0</span><span class="stat-unit">A</span>
                    </div>
                </div>
                <div class="stat">
                    <div class="stat-label">Energy</div>
                    <div class="stat-value">
                        <span id="energy">0</span><span class="stat-unit">kWh</span>
                    </div>
                </div>
            </div>
            
            <div class="apparent-card">
                <div class="apparent-label">Apparent Power (VA)</div>
                <div class="apparent-value">
                    <span id="apparentPower">0</span> <span class="stat-unit">VA</span>
                </div>
            </div>
            
            <div class="pf-card" id="pfCard">
                <div class="pf-label">Power Factor</div>
                <div class="pf-value" id="pfValue">
                    <span id="powerFactor">0.00</span>
                </div>
                <div id="pfQuality" style="font-size: 11px; margin-top: 5px;"></div>
            </div>
            
            <div class="stats">
                <div class="stat">
                    <div class="stat-label">Relay</div>
                    <div class="stat-value">
                        <span id="relayStatus">OFF</span>
                    </div>
                </div>
                <div class="stat">
                    <div class="stat-label">Reactive Power</div>
                    <div class="stat-value">
                        <span id="reactivePower">0</span><span class="stat-unit">VAR</span>
                    </div>
                </div>
                <div class="stat">
                    <div class="stat-label">Efficiency</div>
                    <div class="stat-value">
                        <span id="efficiency">0</span><span class="stat-unit">%</span>
                    </div>
                </div>
            </div>
            
            <button class="relay-btn off" id="relayBtn" onclick="toggleRelay()">
                <span id="relayText">OFF</span>
            </button>
            
            <div style="display: flex; gap: 10px; margin-top: 10px;">
                <button onclick="resetEnergy()" style="flex:1; padding:10px; background:#6B7280; color:white; border:none; border-radius:10px; cursor:pointer;">Reset Energy</button>
            </div>
            
            <div class="info">
                <div><span class="status" id="statusLed"></span> <span id="statusText">Updating...</span></div>
                <div id="updateTime" class="refresh"></div>
            </div>
        </div>
    </div>
    
    <script>
        function getPFClass(pf) {
            if (pf >= 0.95) return 'pf-good';
            if (pf >= 0.8) return 'pf-warning';
            return 'pf-bad';
        }
        
        function getPFQuality(pf) {
            if (pf >= 0.95) return '✓ Excellent - Near unity';
            if (pf >= 0.9) return '✓ Good - Efficient';
            if (pf >= 0.8) return '⚠ Fair - Some reactive power';
            return '✗ Poor - High reactive power';
        }
        
        async function fetchData() {
            try {
                const response = await fetch('/data');
                const data = await response.json();
                
                document.getElementById('power').innerText = data.power.toFixed(1);
                document.getElementById('voltage').innerText = data.voltage.toFixed(1);
                document.getElementById('current').innerText = data.current.toFixed(3);
                document.getElementById('energy').innerText = data.energy.toFixed(3);
                document.getElementById('apparentPower').innerText = data.apparentPower.toFixed(1);
                document.getElementById('reactivePower').innerText = data.reactivePower.toFixed(1);
                document.getElementById('efficiency').innerText = (data.powerFactor * 100).toFixed(1);
                
                const pfElement = document.getElementById('powerFactor');
                pfElement.innerText = data.powerFactor.toFixed(3);
                pfElement.className = getPFClass(data.powerFactor);
                
                document.getElementById('pfQuality').innerHTML = getPFQuality(data.powerFactor);
                document.getElementById('pfCard').style.borderLeft = '4px solid ' + 
                    (data.powerFactor >= 0.95 ? '#10B981' : (data.powerFactor >= 0.8 ? '#F59E0B' : '#EF4444'));
                
                const relayStatus = document.getElementById('relayStatus');
                const relayBtn = document.getElementById('relayBtn');
                const relayText = document.getElementById('relayText');
                
                if (data.relayState) {
                    relayStatus.innerText = 'ON';
                    relayBtn.className = 'relay-btn on';
                    relayText.innerText = 'ON';
                } else {
                    relayStatus.innerText = 'OFF';
                    relayBtn.className = 'relay-btn off';
                    relayText.innerText = 'OFF';
                }
                
                document.getElementById('statusLed').className = 'status online';
                document.getElementById('statusText').innerHTML = 'Connected';
                document.getElementById('updateTime').innerHTML = 'Updated: ' + new Date().toLocaleTimeString();
                
            } catch(e) {
                document.getElementById('statusLed').className = 'status offline';
                document.getElementById('statusText').innerHTML = 'Reconnecting...';
            }
        }
        
        async function toggleRelay() {
            try {
                const response = await fetch('/toggle');
                const result = await response.text();
                setTimeout(fetchData, 100);
            } catch(e) {
                console.error('Toggle error:', e);
            }
        }
        
        async function resetEnergy() {
            if (confirm('Reset energy counter?')) {
                try {
                    await fetch('/reset');
                    setTimeout(fetchData, 100);
                } catch(e) {
                    console.error('Reset error:', e);
                }
            }
        }
        
        async function getDeviceInfo() {
            try {
                const res = await fetch('/info');
                const info = await res.json();
                document.getElementById('deviceId').innerHTML = info.hostname + '.local';
            } catch(e) {}
        }
        
        setInterval(fetchData, 1000);
        setInterval(getDeviceInfo, 30000);
        
        fetchData();
        getDeviceInfo();
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);  // For debugging
  
  // Initialize the S31 library
  s31.begin();
  
  // Updated callback with Power Factor (4 parameters now)
  s31.onPowerUpdate([](float power, float voltage, float current, float powerFactor) {
    // Optional: Log power updates with PF
    // Serial.printf("Power: %.1fW, Voltage: %.1fV, Current: %.3fA, PF: %.3f\n", 
    //               power, voltage, current, powerFactor);
  });
  
  s31.onRelayChange([](bool state) {
    Serial.printf("Relay changed to: %s\n", state ? "ON" : "OFF");
  });
  
  // Connect to WiFi
  WiFi.hostname(deviceName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Setup mDNS
  MDNS.begin(deviceName.c_str());
  MDNS.addService("http", "tcp", 80);
  
  // Setup OTA
  ArduinoOTA.setHostname(deviceName.c_str());
  ArduinoOTA.setPassword("admin123");
  ArduinoOTA.begin();
  
  // Web server routes
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });
  
  server.on("/data", HTTP_GET, []() {
    float power = s31.getPower();
    float voltage = s31.getVoltage();
    float current = s31.getCurrent();
    float powerFactor = s31.getPowerFactor();
    float apparentPower = s31.getApparentPower();
    
    // Calculate reactive power: VAR = sqrt(VA² - W²)
    float reactivePower = sqrt(pow(apparentPower, 2) - pow(power, 2));
    if (isnan(reactivePower)) reactivePower = 0;
    
    String json = "{";
    json += "\"voltage\":" + String(voltage, 1) + ",";
    json += "\"current\":" + String(current, 3) + ",";
    json += "\"power\":" + String(power, 1) + ",";
    json += "\"powerFactor\":" + String(powerFactor, 3) + ",";
    json += "\"apparentPower\":" + String(apparentPower, 1) + ",";
    json += "\"reactivePower\":" + String(reactivePower, 1) + ",";
    json += "\"energy\":" + String(s31.getEnergy(), 3) + ",";
    json += "\"relayState\":" + String(s31.getRelayState() ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/toggle", HTTP_GET, []() {
    s31.toggleRelay();
    server.send(200, "text/plain", "OK");
  });
  
  server.on("/info", HTTP_GET, []() {
    String json = "{";
    json += "\"hostname\":\"" + deviceName + "\",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.on("/reset", HTTP_GET, []() {
    s31.resetEnergy();
    server.send(200, "text/plain", "Energy reset");
  });
  
  server.begin();
  Serial.println("Web server started");
  Serial.println("\n=== Sonoff S31 Ready ===");
  Serial.printf("Power Factor monitoring enabled\n");
  Serial.printf("Visit: http://%s.local or http://%s\n", deviceName.c_str(), WiFi.localIP().toString().c_str());
}

void loop() {
  // Update the S31 library (reads power data)
  s31.update();
  
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Handle mDNS
  MDNS.update();
  
  // Handle web server
  server.handleClient();
  
  delay(5);
}