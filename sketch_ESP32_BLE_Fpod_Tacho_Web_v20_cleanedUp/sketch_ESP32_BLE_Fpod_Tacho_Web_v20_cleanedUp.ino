#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <TimeLib.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// ============================================================================
// CONFIGURATION & CONSTANTS
// ============================================================================
const char* WIFI_SSID = "chicago";
const char* WIFI_PASSWORD = "Neukirch20211!";
const char* BLE_DEVICE_NAME = "Treadmill_Rieger";

// Hardware pins
const int INTERRUPT_PIN = 3;
const int LED_PIN = 2;

// Treadmill mechanics
const long BELT_DISTANCE_MM = 142;  // Belt distance per revolution in mm
const long DEBOUNCE_THRESHOLD_US = 300;  // Minimum time between valid interrupts
const long MAX_REVOLUTION_TIME_US = 1000000;  // Max time for valid revolution (1 second)

// BLE UUIDs
#define RSC_SERVICE_UUID BLEUUID((uint16_t)0x1814)
#define RSC_MEASUREMENT_UUID BLEUUID((uint16_t)0x2A53)
#define SENSOR_POSITION_UUID BLEUUID((uint16_t)0x2A5D)

// Update intervals
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;  // 1 second
const unsigned long WEB_UPDATE_INTERVAL = 1000;      // 1 second

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
struct TreadmillMetrics {
    volatile unsigned long startTime = 0;
    volatile unsigned int revCount = 0;
    volatile unsigned int totalRevCount = 0;
    volatile long accumulatorInterval = 0;
    volatile unsigned long longPauseTime = 0;
    
    float rpm = 0;
    float mps = 0;
    float mpsOffset = 0;
    long workoutDistance = 0;  // in mm
    unsigned long workoutStartTime = 0;
    unsigned long lastUpdate = 0;
};

struct BLEData {
    bool clientConnected = false;
    uint16_t instSpeed = 40;
    uint8_t instCadence = 1;
    uint16_t instStrideLength = 1;
    uint32_t totalDistance = 10;
    byte fakePos[1] = {1};
};

TreadmillMetrics metrics;
BLEData bleData;

// BLE objects
BLEServer* pServer = nullptr;
BLECharacteristic* pRSCMeasurement = nullptr;
BLECharacteristic* pSensorPosition = nullptr;
AsyncWebServer server(80);

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================
void IRAM_ATTR tachInterrupt() {
    unsigned long usNow = micros();
    long elapsed = usNow - metrics.startTime;
    
    // Debounce check
    if (elapsed < DEBOUNCE_THRESHOLD_US) {
        return;
    }
    
    // Reset if too much time has passed (treadmill stopped)
    if (elapsed > MAX_REVOLUTION_TIME_US) {
        metrics.startTime = usNow;
        metrics.longPauseTime = 0;
        return;
    }
    
    // Valid revolution detected
    if (elapsed > metrics.longPauseTime / 2) {
        metrics.startTime = usNow;
        metrics.longPauseTime = elapsed;
        metrics.revCount++;
        metrics.totalRevCount++;
        metrics.workoutDistance += BELT_DISTANCE_MM;
        metrics.accumulatorInterval += elapsed;
    }
}

// ============================================================================
// BLE CALLBACKS AND FUNCTIONS
// ============================================================================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        bleData.clientConnected = true;
        Serial.println("BLE client connected");
    }
    
    void onDisconnect(BLEServer* pServer) override {
        bleData.clientConnected = false;
        Serial.println("BLE client disconnected");
        // Restart advertising
        pServer->getAdvertising()->start();
    }
};

void initBLE() {
    BLEDevice::init(BLE_DEVICE_NAME);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    BLEService* pRSCService = pServer->createService(RSC_SERVICE_UUID);
    
    // RSC Measurement Characteristic
    pRSCMeasurement = pRSCService->createCharacteristic(
        RSC_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pRSCMeasurement->addDescriptor(new BLE2902());
    
    // Sensor Position Characteristic
    pSensorPosition = pRSCService->createCharacteristic(
        SENSOR_POSITION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    pRSCService->start();
    pServer->getAdvertising()->addServiceUUID(RSC_SERVICE_UUID);
    pServer->getAdvertising()->start();
    
    Serial.println("BLE service started");
}

void sendBLEData() {
    if (!bleData.clientConnected || !pRSCMeasurement) {
        return;
    }
    
    bleData.instSpeed = (metrics.mps + metrics.mpsOffset) * 256;
    bleData.totalDistance = metrics.workoutDistance / 100;  // Convert mm to cm
    
    byte dataPacket[10] = {
        3,  // Flags
        (byte)bleData.instSpeed, (byte)(bleData.instSpeed >> 8),
        bleData.instCadence,
        (byte)bleData.instStrideLength, (byte)(bleData.instStrideLength >> 8),
        (byte)bleData.totalDistance, (byte)(bleData.totalDistance >> 8),
        (byte)(bleData.totalDistance >> 16), (byte)(bleData.totalDistance >> 24)
    };
    
    pRSCMeasurement->setValue(dataPacket, 10);
    pRSCMeasurement->notify();
    pSensorPosition->setValue(bleData.fakePos, 1);
}

// ============================================================================
// CALCULATION FUNCTIONS
// ============================================================================
void calculateRPM() {
    if (metrics.revCount > 0) {
        metrics.rpm = 60000000.0 / (metrics.accumulatorInterval / metrics.revCount);
        metrics.accumulatorInterval = 0;
    } else {
        metrics.rpm = 0;
    }
    metrics.revCount = 0;
}

void calculateSpeed() {
    // Speed = distance_per_rev * rev_per_min * min_per_sec * m_per_mm
    metrics.mps = (BELT_DISTANCE_MM * metrics.rpm) / (60.0 * 1000.0);
}

void updateMetrics() {
    calculateRPM();
    calculateSpeed();
}

// ============================================================================
// WEB SERVER FUNCTIONS
// ============================================================================
String getSpeed() { return String(metrics.mps * 3.6, 1); }  // km/h with 1 decimal
String getDistance() { return String(metrics.workoutDistance / 1000.0, 0); }  // meters
String getRPM() { return String((int)metrics.rpm); }
String getHour() { return String(hour()); }
String getMinute() { return String(minute()); }
String getSecond() { return String(second()); }
String getOffset() { return String(metrics.mpsOffset * 3.6, 1); }  // km/h
String getTotalRevs() { return String(metrics.totalRevCount); }

void adjustOffsetUp() {
    metrics.mpsOffset += 0.139;  // ~0.5 km/h increment
    Serial.printf("Offset UP: %.1f km/h\n", metrics.mpsOffset * 3.6);
}

void adjustOffsetDown() {
    if (metrics.mpsOffset >= 0.139) {
        metrics.mpsOffset -= 0.139;
    } else {
        metrics.mpsOffset = 0;
    }
    Serial.printf("Offset DOWN: %.1f km/h\n", metrics.mpsOffset * 3.6);
}

void resetWorkout() {
    metrics.workoutDistance = 0;
    metrics.totalRevCount = 0;
    setTime(0, 0, 0, 0, 0, 0);
    Serial.println("Workout reset");
}

const char* HTML_TEMPLATE = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.7.2/css/all.css">
  <style>
    html { font-family: Arial; display: inline-block; margin: 0px auto; text-align: center; }
    h2 { font-size: 3.0rem; }
    p { font-size: 3.0rem; }
    .units { font-size: 1.2rem; }
    .button { border: none; color: white; padding: 15px 32px; text-align: center; 
              text-decoration: none; display: inline-block; font-size: 16px; 
              margin: 4px 2px; cursor: pointer; }
    .button1 { background-color: #4CAF50; }
    .button2 { background-color: #008CBA; }
    .button3 { background-color: #cc0000; }
  </style>
</head>
<body>
  <h2>Treadmill Monitor</h2>
  
  <p><i class="fas fa-running" style="color:#059e8a;"></i> 
     <span id="speed">%SPEED%</span><sup class="units">km/h</sup></p>
  
  <p><i class="fas fa-shoe-prints" style="color:#2d0000;"></i> 
     <span id="distance">%DISTANCE%</span><sup class="units">m</sup></p>
  
  <p><i class="fas fa-stopwatch" style="color:#059e8a;"></i> 
     <span id="hour">%HOUR%</span>:<span id="minute">%MINUTE%</span>:<span id="second">%SECOND%</span></p>
  
  <p><i class="fas fa-arrows-alt-h" style="color:#059e8a;"></i> 
     <span id="offset">%OFFSET%</span><sup class="units">km/h offset</sup><br>
     <a href="/offset/up"><button class="button button1">Up</button></a>
     <a href="/offset/down"><button class="button button2">Down</button></a></p>
  
  <p><i class="fas fa-tachometer-alt" style="color:#00add6;"></i> 
     <span id="rpm">%RPM%</span><sup class="units">rpm</sup></p>
  
  <p><i class="fas fa-chart-line" style="color:#00add6;"></i> 
     <span id="revs">%REVS%</span><sup class="units">total revs</sup></p>
  
  <p><a href="/reset"><button class="button button3">Reset Workout</button></a></p>

<script>
function updateValue(id, endpoint) {
  fetch('/' + endpoint)
    .then(response => response.text())
    .then(data => document.getElementById(id).innerHTML = data)
    .catch(error => console.error('Error:', error));
}

setInterval(() => {
  updateValue('speed', 'api/speed');
  updateValue('distance', 'api/distance');
  updateValue('hour', 'api/hour');
  updateValue('minute', 'api/minute');
  updateValue('second', 'api/second');
  updateValue('offset', 'api/offset');
  updateValue('rpm', 'api/rpm');
  updateValue('revs', 'api/revs');
}, 1000);
</script>
</body>
</html>)rawliteral";

String processTemplate(const String& var) {
    if (var == "SPEED") return getSpeed();
    if (var == "DISTANCE") return getDistance();
    if (var == "HOUR") return getHour();
    if (var == "MINUTE") return getMinute();
    if (var == "SECOND") return getSecond();
    if (var == "OFFSET") return getOffset();
    if (var == "RPM") return getRPM();
    if (var == "REVS") return getTotalRevs();
    return String();
}

void initWebServer() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  // LED on when connected
    
    // Main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });
    
    // API endpoints
    server.on("/api/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getSpeed());
    });
    server.on("/api/distance", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getDistance());
    });
    server.on("/api/rpm", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getRPM());
    });
    server.on("/api/hour", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getHour());
    });
    server.on("/api/minute", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getMinute());
    });
    server.on("/api/second", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getSecond());
    });
    server.on("/api/offset", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getOffset());
    });
    server.on("/api/revs", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getTotalRevs());
    });
    
    // Control endpoints
    server.on("/offset/up", HTTP_GET, [](AsyncWebServerRequest *request) {
        adjustOffsetUp();
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });
    server.on("/offset/down", HTTP_GET, [](AsyncWebServerRequest *request) {
        adjustOffsetDown();
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        resetWorkout();
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });
    
    server.begin();
    Serial.println("Web server started");
}

void initTachometer() {
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), tachInterrupt, FALLING);
    setTime(0, 0, 0, 0, 0, 0);
    Serial.println("Tachometer initialized");
}

void printMetrics() {
    Serial.printf("RPM: %d | Speed: %.1f km/h | Distance: %.0fm | Time: %02d:%02d:%02d | Offset: %.1f km/h\n",
        (int)metrics.rpm,
        metrics.mps * 3.6,
        metrics.workoutDistance / 1000.0,
        hour(), minute(), second(),
        metrics.mpsOffset * 3.6
    );
}

void checkWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, reconnecting...");
        digitalWrite(LED_PIN, LOW);
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        digitalWrite(LED_PIN, HIGH);
        Serial.println("\nWiFi reconnected");
    }
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== Treadmill Monitor Starting ===");
    
    initBLE();
    initTachometer();
    initWebServer();
    
    metrics.lastUpdate = millis();
    
    Serial.println("=== Setup Complete ===\n");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Update metrics every second
    if (currentTime - metrics.lastUpdate >= DISPLAY_UPDATE_INTERVAL) {
        metrics.lastUpdate = currentTime;
        
        updateMetrics();
        printMetrics();
        sendBLEData();
        checkWiFiConnection();
    }
    
    // Small delay to prevent excessive CPU usage
    delay(10);
}