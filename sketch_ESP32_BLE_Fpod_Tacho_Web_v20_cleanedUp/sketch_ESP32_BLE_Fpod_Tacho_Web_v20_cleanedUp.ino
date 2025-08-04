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
const int MAX_WIFI_RECONNECT_TRY = 2;
const char* BLE_DEVICE_NAME = "Treadmill_Rieger";

// Hardware pins
const int INTERRUPT_PIN = 4;
const int LED_PIN = 2;

// Treadmill mechanics
const long BELT_DISTANCE_MM = 191;  // Belt distance per revolution in mm
const long DEBOUNCE_THRESHOLD_US = 30;  // Minimum time between valid interrupts
const long MAX_REVOLUTION_TIME_US = 1000000;  // Max time for valid revolution (1 second)

// DUAL BLE SERVICES - RSC for data + FTMS for control
#define RSC_SERVICE_UUID BLEUUID((uint16_t)0x1814)
#define RSC_MEASUREMENT_UUID BLEUUID((uint16_t)0x2A53)
#define SENSOR_POSITION_UUID BLEUUID((uint16_t)0x2A5D)

#define FTMS_SERVICE_UUID BLEUUID((uint16_t)0x1826)
#define FTMS_FEATURE_UUID BLEUUID((uint16_t)0x2ACC)
#define TREADMILL_DATA_UUID BLEUUID((uint16_t)0x2ACD)
#define FTMS_CONTROL_POINT_UUID BLEUUID((uint16_t)0x2AD9)
#define FTMS_STATUS_UUID BLEUUID((uint16_t)0x2ADA)

// FTMS Control Point Operation Codes
#define FTMS_REQUEST_CONTROL        0x00
#define FTMS_RESET                  0x01
#define FTMS_SET_TARGET_SPEED       0x02
#define FTMS_SET_TARGET_INCLINATION 0x03
#define FTMS_START_RESUME           0x07
#define FTMS_STOP_PAUSE             0x08

// FTMS Response Codes
#define FTMS_SUCCESS                0x01
#define FTMS_NOT_SUPPORTED          0x02
#define FTMS_INVALID_PARAMETER      0x03

// Update intervals - FASTER UPDATES
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // 2 times per second 

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
struct TreadmillMetrics {
    volatile unsigned long startTime = 0;
    volatile unsigned int revCount = 0;
    volatile unsigned int totalRevCount = 0;
    volatile long accumulatorInterval = 0;
    volatile unsigned long longPauseTime = 0;
    volatile unsigned long sensorPulsPerSecond = 0;
    
    float rpm = 0;
    float mps = 0;
    float mpsOffset = 0;
    long workoutDistance = 0;  // in mm
    unsigned long workoutStartTime = 0;
    unsigned long lastUpdate = 0;
    
    // FTMS control metrics
    float targetSpeed = 0.0;        // km/h set by MyWoosh
    float targetInclination = 0.0;  // percentage 
    float currentInclination = 0.0; // percentage
    bool isRunning = false;
    bool isPaused = false;
    bool controlRequested = false;
    unsigned long sessionStartTime = 0;
};

struct BLEData {
    bool clientConnected = false;
    uint16_t instSpeed = 0;
    uint8_t instCadence = 0;
    uint16_t instStrideLength = 1;
    uint32_t totalDistance = 0;
    byte fakePos[1] = {1};
};

TreadmillMetrics metrics;
BLEData bleData;

// DUAL BLE objects
BLEServer* pServer = nullptr;
// RSC characteristics
BLECharacteristic* pRSCMeasurement = nullptr;
BLECharacteristic* pSensorPosition = nullptr;
// FTMS characteristics
BLECharacteristic* pFTMSTreadmillData = nullptr;
BLECharacteristic* pFTMSControlPoint = nullptr;
BLECharacteristic* pFTMSStatus = nullptr;
BLECharacteristic* pFTMSFeature = nullptr;

AsyncWebServer server(80);

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================
void IRAM_ATTR tachInterrupt() {
    unsigned long usNow = micros();
    long elapsed = usNow - metrics.startTime;
    static unsigned long lastSecondTime = 0;
    
    if (elapsed < DEBOUNCE_THRESHOLD_US) {
        return;
    }
    
    if (elapsed > MAX_REVOLUTION_TIME_US) {
        metrics.startTime = usNow;
        metrics.longPauseTime = 0;
        return;
    }

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
// FTMS CONTROL FUNCTIONS
// ============================================================================
void sendFTMSStatusUpdate(uint8_t statusCode) {
    if (!bleData.clientConnected || !pFTMSStatus) return;
    
    uint8_t statusData[2] = {0x01, statusCode};
    pFTMSStatus->setValue(statusData, 2);
    pFTMSStatus->notify();
    
    Serial.printf("FTMS Status: 0x%02X ", statusCode);
    switch(statusCode) {
        case 0x01: Serial.println("(Control Granted)"); break;
        case 0x02: Serial.println("(Reset Complete)"); break;
        case 0x04: Serial.println("(Started/Resumed)"); break;
        case 0x05: Serial.println("(Stopped)"); break;
        case 0x06: Serial.println("(Paused)"); break;
        default: Serial.println("(Unknown)"); break;
    }
}

void resetWorkout() {
    metrics.workoutDistance = 0;
    metrics.totalRevCount = 0;
    metrics.targetSpeed = 5;
    metrics.targetInclination = 0;
    metrics.isRunning = false;
    metrics.isPaused = false;
    metrics.sessionStartTime = millis();
    setTime(0, 0, 0, 0, 0, 0);
    Serial.println("Workout reset via FTMS command");
}

// FTMS Control Point Callbacks
class FTMSControlPointCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
        uint8_t* data = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();
        
        if (length < 1) return;
        
        uint8_t opCode = data[0];
        uint8_t response[3] = {0x80, opCode, FTMS_SUCCESS};
        
        Serial.printf("=== FTMS TRAINING COMMAND: 0x%02X ===\n", opCode);
        
        switch (opCode) {
            case FTMS_REQUEST_CONTROL:
                Serial.println("MyWoosh requesting control - GRANTED!");
                metrics.controlRequested = true;
                sendFTMSStatusUpdate(0x01);
                break;
                
            case FTMS_RESET:
                Serial.println("MyWoosh reset command");
                resetWorkout();
                sendFTMSStatusUpdate(0x02);
                break;
                
            case FTMS_SET_TARGET_SPEED:
                if (length >= 3) {
                    uint16_t speed = (data[2] << 8) | data[1];
                    float speedKmh = speed * 0.01;
                    Serial.printf("🎯 MyWoosh TARGET SPEED: %.2f km/h\n", speedKmh);
                    metrics.targetSpeed = speedKmh;
                    
                    // Automatically adjust offset to match target
                    float currentSpeedKmh = (metrics.mps + metrics.mpsOffset) * 3.6;
                    float adjustment = (speedKmh - currentSpeedKmh) / 3.6;
                    metrics.mpsOffset += adjustment;
                    
                    Serial.printf("Speed adjusted: Current %.1f -> Target %.1f (Offset: %.1f km/h)\n", 
                                  currentSpeedKmh, speedKmh, metrics.mpsOffset * 3.6);
                } else {
                    response[2] = FTMS_INVALID_PARAMETER;
                }
                break;
                
            case FTMS_SET_TARGET_INCLINATION:
                if (length >= 3) {
                    int16_t incline = (data[2] << 8) | data[1];
                    float inclinePercent = incline * 0.1;
                    Serial.printf("🏔️ MyWoosh TARGET INCLINE: %.1f%%\n", inclinePercent);
                    metrics.targetInclination = inclinePercent;
                    metrics.currentInclination = inclinePercent;
                } else {
                    response[2] = FTMS_INVALID_PARAMETER;
                }
                break;
                
            case FTMS_START_RESUME:
                Serial.println("▶️ MyWoosh START/RESUME command");
                if (!metrics.isRunning) {
                    metrics.isRunning = true;
                    metrics.isPaused = false;
                    metrics.sessionStartTime = millis();
                    sendFTMSStatusUpdate(0x04);
                } else if (metrics.isPaused) {
                    metrics.isPaused = false;
                    sendFTMSStatusUpdate(0x04);
                }
                break;
                
            case FTMS_STOP_PAUSE:
                if (length >= 2) {
                    if (data[1] == 0x01) {
                        Serial.println("⏹️ MyWoosh STOP command");
                        metrics.isRunning = false;
                        metrics.isPaused = false;
                        sendFTMSStatusUpdate(0x05);
                    } else if (data[1] == 0x02) {
                        Serial.println("⏸️ MyWoosh PAUSE command");
                        metrics.isPaused = true;
                        sendFTMSStatusUpdate(0x06);
                    }
                }
                break;
                
            default:
                Serial.printf("❌ Unsupported FTMS command: 0x%02X\n", opCode);
                response[2] = FTMS_NOT_SUPPORTED;
                break;
        }
        
        // Send response
        pFTMSControlPoint->setValue(response, 3);
        pFTMSControlPoint->indicate();
        
        Serial.println("=== FTMS COMMAND COMPLETE ===");
    }
};

// ============================================================================
// BLE CALLBACKS AND FUNCTIONS
// ============================================================================
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        bleData.clientConnected = true;
        Serial.println("=== DUAL SERVICE BLE CLIENT CONNECTED ===");
        //Serial.println("RSC: Ready for speed/distance data");
        //Serial.println("FTMS: Ready for training control");
        
        // Auto-grant FTMS control
        metrics.controlRequested = true;
        delay(200);
        sendFTMSStatusUpdate(0x01);
    }
    
    void onDisconnect(BLEServer* pServer) override {
        bleData.clientConnected = false;
        metrics.controlRequested = false;
        Serial.println("=== BLE CLIENT DISCONNECTED ===");
        
        // Restart advertising
        pServer->getAdvertising()->start();
    }
};

void initBLE() {
    BLEDevice::init(BLE_DEVICE_NAME);
    
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    
    // ========== RSC SERVICE (for data transmission) ==========
    BLEService* pRSCService = pServer->createService(RSC_SERVICE_UUID);
    
    pRSCMeasurement = pRSCService->createCharacteristic(
        RSC_MEASUREMENT_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pRSCMeasurement->addDescriptor(new BLE2902());
    
    pSensorPosition = pRSCService->createCharacteristic(
        SENSOR_POSITION_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    pRSCService->start();
    
    // ========== FTMS SERVICE (for control) ==========
    BLEService* pFTMSService = pServer->createService(FTMS_SERVICE_UUID);
    
    // FTMS Feature
    pFTMSFeature = pFTMSService->createCharacteristic(
        FTMS_FEATURE_UUID,
        BLECharacteristic::PROPERTY_READ
    );
    
    uint8_t features[8] = {0};
    uint32_t featureFlags = 0x00000001 | 0x00000004; // Speed + Distance
    memcpy(features, &featureFlags, 4);
    uint32_t targetFlags = 0x00000001 | 0x00000002; // Target Speed + Inclination
    memcpy(features + 4, &targetFlags, 4);
    pFTMSFeature->setValue(features, 8);
    
    // FTMS Treadmill Data (backup)
    pFTMSTreadmillData = pFTMSService->createCharacteristic(
        TREADMILL_DATA_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pFTMSTreadmillData->addDescriptor(new BLE2902());
    
    // FTMS Control Point
    pFTMSControlPoint = pFTMSService->createCharacteristic(
        FTMS_CONTROL_POINT_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE
    );
    pFTMSControlPoint->addDescriptor(new BLE2902());
    pFTMSControlPoint->setCallbacks(new FTMSControlPointCallbacks());
    
    // FTMS Status
    pFTMSStatus = pFTMSService->createCharacteristic(
        FTMS_STATUS_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pFTMSStatus->addDescriptor(new BLE2902());
    
    pFTMSService->start();
    
    // Start advertising BOTH services
    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(RSC_SERVICE_UUID);
    pAdvertising->addServiceUUID(FTMS_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    
    Serial.println("🔥 DUAL BLE SERVICES STARTED:");
    Serial.println("RSC (0x1814) - Data transmission");
    Serial.println("FTMS (0x1826) - Training control");
    Serial.println("Compatible with MyWoosh training programs!");
}

void sendBLEData() {
    if (!bleData.clientConnected) return;
    
    // ========== SEND RSC DATA (primary) ==========
    if (pRSCMeasurement) {
        bleData.instSpeed = (metrics.mps + metrics.mpsOffset) * 256;
        bleData.totalDistance = metrics.workoutDistance / 100;
        
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
    
    // ========== SEND FTMS DATA (backup) ==========
    if (pFTMSTreadmillData && metrics.controlRequested) {
        uint8_t ftmsData[10] = {0};
        size_t index = 0;
        
        uint16_t flags = 0x0001 | 0x0004; // Speed + Distance
        ftmsData[index++] = flags & 0xFF;
        ftmsData[index++] = (flags >> 8) & 0xFF;
        
        float speedKmh = (metrics.mps + metrics.mpsOffset) * 3.6;
        uint16_t speed = (uint16_t)(speedKmh * 100);
        ftmsData[index++] = speed & 0xFF;
        ftmsData[index++] = (speed >> 8) & 0xFF;
        
        uint32_t distanceM = metrics.workoutDistance / 1000;
        ftmsData[index++] = distanceM & 0xFF;
        ftmsData[index++] = (distanceM >> 8) & 0xFF;
        ftmsData[index++] = (distanceM >> 16) & 0xFF;
        
        pFTMSTreadmillData->setValue(ftmsData, index);
        pFTMSTreadmillData->notify();
    }
    
    // Debug output
    //float speedKmh = (metrics.mps + metrics.mpsOffset) * 3.6;
    //Serial.printf("📡 Data sent - Speed: %.1f km/h, Distance: %u m | Target: %.1f km/h, %.1f%%\n", 
                 // speedKmh, metrics.workoutDistance / 1000,
                 // metrics.targetSpeed, metrics.targetInclination);
}

// ============================================================================
// CALCULATION FUNCTIONS
// ============================================================================
void calculateRPM() {
    static unsigned long lastValidRPMTime = 0;
    unsigned long currentTime = millis();
    
    if (metrics.revCount > 0 && metrics.accumulatorInterval !=0) {
        metrics.rpm = 60000000.0 / (metrics.accumulatorInterval / metrics.revCount);
        metrics.sensorPulsPerSecond = (metrics.revCount * 60 *1000000) / metrics.accumulatorInterval; // 1 Second
        metrics.accumulatorInterval = 0;
        lastValidRPMTime = currentTime;
        metrics.revCount = 0;
    } else {
        // Nur auf 0 setzen, wenn längere Zeit keine Impulse
        if (currentTime - lastValidRPMTime > 2000) { // 2 Sekunden
            metrics.rpm = 0;
        }
        // Sonst: letzte RPM beibehalten
    }
}


void calculateSpeed() {
    metrics.mps = (BELT_DISTANCE_MM * metrics.rpm) / (60.0 * 1000.0);
}

void updateMetrics() {
    calculateRPM();
    calculateSpeed();
}

// ============================================================================
// WEB SERVER FUNCTIONS
// ============================================================================
String getSpeed() { return String((metrics.mps + metrics.mpsOffset) * 3.6, 1); }
String getSensorInterupts() { return String(metrics.sensorPulsPerSecond); }  // Pure sensor speed
String getDistance() { return String(metrics.workoutDistance / 1000.0, 0); }
String getRPM() { return String((int)metrics.rpm); }
String getHour() { return String(hour()); }
String getMinute() { return String(minute()); }
String getSecond() { return String(second()); }
String getOffset() { return String(metrics.mpsOffset * 3.6, 1); }
String getTotalRevs() { return String(metrics.totalRevCount); }

String getBLEStatus() { 
    if (!bleData.clientConnected) return "Disconnected";
    if (!metrics.controlRequested) return "Connected - Data Only";
    return "Connected - Full Control";
}
String getBLEClientCount() { return String(bleData.clientConnected ? 1 : 0); }
String getTargetSpeed() { return String(metrics.targetSpeed, 1); }
String getTargetIncline() { return String(metrics.targetInclination, 1); }
String getTrainingStatus() {
    if (!metrics.controlRequested) return "No Control";
    if (metrics.isPaused) return "Paused";
    if (metrics.isRunning) return "Running";
    return "Ready";
}

void adjustOffsetUp() {
    metrics.mpsOffset += 0.139;
    Serial.printf("Manual Offset UP: %.1f km/h\n", metrics.mpsOffset * 3.6);
}

void adjustOffsetDown() {
    if (metrics.mpsOffset >= 0.139) {
        metrics.mpsOffset -= 0.139;
    } else {
        metrics.mpsOffset = 0;
    }
    Serial.printf("Manual Offset DOWN: %.1f km/h\n", metrics.mpsOffset * 3.6);
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
    .sensor-speed { color: #666; font-size: 2.0rem; }
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
  
  <p class="sensor-speed"><i class="fas fa-cog" style="color:#059e8a;"></i> 
     Sensor interupts: <span id="sensorpulse">%SENSORPULSE%</span><sup class="units">interupts/s</sup></p>
  
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
  updateValue('sensorpulse', 'api/sensorpulse');
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
    if (var == "SENSORPULSE") return getSensorInterupts();
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
    int reconnectCont = 0;
    while (WiFi.status() != WL_CONNECTED && reconnectCont < MAX_WIFI_RECONNECT_TRY) {
        delay(3000);
        Serial.print(".");
        reconnectCont++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    Serial.println();
    Serial.printf("WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });
    
    // API endpoints - keeping original simple set
    server.on("/api/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getSpeed());
    });
    server.on("/api/sensorpulse", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", getSensorInterupts());
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
    Serial.printf("RPM: %d | Speed: %.1f km/h | Distance: %.0fm | Time: %02d:%02d:%02d | Offset: %.1f km/h | BLE: %s\n",
        (int)metrics.rpm,
        (metrics.mps + metrics.mpsOffset) * 3.6,
        metrics.workoutDistance / 1000.0,
        hour(), minute(), second(),
        metrics.mpsOffset * 3.6,
        bleData.clientConnected ? "Connected" : "Disconnected"
    );
    
    if (bleData.clientConnected && metrics.controlRequested) {
        Serial.printf("Training: Speed %.1f->%.1f km/h, Incline %.1f%%, Status: %s\n",
                      (metrics.mps + metrics.mpsOffset) * 3.6,
                      metrics.targetSpeed,
                      metrics.targetInclination,
                      metrics.isRunning ? (metrics.isPaused ? "Paused" : "Running") : "Stopped");
    }
}

void checkWiFiConnection() {
    int reconnectCont = 0;
    if (WiFi.status() != WL_CONNECTED ) {
        Serial.println("WiFi disconnected, reconnecting...");
        digitalWrite(LED_PIN, LOW);
        WiFi.reconnect();
        while (WiFi.status() != WL_CONNECTED && reconnectCont < MAX_WIFI_RECONNECT_TRY) {
            delay(3000);
            Serial.print("|");
            reconnectCont++;
        }

        if (WiFi.status() == WL_CONNECTED) {
          digitalWrite(LED_PIN, HIGH);
          Serial.println("\nWiFi reconnected");
        }
    }
}

// ============================================================================
// MAIN FUNCTIONS
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== SMART BIDIRECTIONAL TREADMILL ===");
    Serial.println("Features:");
    Serial.println("RSC Service - Reliable data transmission");
    Serial.println("FTMS Service - Training program control");
    Serial.println("Full bidirectional MyWoosh/Zwift support");
    
    initBLE();
    initTachometer();
    initWebServer();
    
    metrics.lastUpdate = millis();
    
    Serial.println("Connect MyWoosh for training programs!");
    Serial.printf("Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println();
    Serial.printf("BLE Device: %s\n", BLE_DEVICE_NAME);
    Serial.println("\n=== SETUP COMPLETE ===");
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - metrics.lastUpdate >= DISPLAY_UPDATE_INTERVAL) {
        metrics.lastUpdate = currentTime;
        updateMetrics();
        
        // Print metrics every 4th update (once per second) to avoid spam
        static int updateCounter = 0;
        updateCounter++;
        if (updateCounter >= 2) {
            //printMetrics(); // only logging
            updateCounter = 0;
        }

        sendBLEData(); // Send both RSC and FTMS data
        
        // Check WiFi less frequently
        static int wifiCounter = 0;
        wifiCounter++;
        if (wifiCounter >= 100 * DISPLAY_UPDATE_INTERVAL) { // Check WiFi every 5 seconds
            checkWiFiConnection();
            wifiCounter = 0;
        }
    }
    
    delay(10); // Reduced delay for more responsive updates
}