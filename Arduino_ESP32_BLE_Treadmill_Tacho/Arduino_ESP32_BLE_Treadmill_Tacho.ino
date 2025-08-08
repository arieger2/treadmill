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
const char* BLE_DEVICE_NAME = "Rieger_Treatmill";
const bool enableRSC = true;


// Hardware pins
const int INTERRUPT_PIN = 4;
const int LED_PIN = 2;

// Treadmill mechanics
const long BELT_DISTANCE_MM = 191;  // Belt distance per revolution in mm
const long DEBOUNCE_THRESHOLD_US = 30;  // Minimum time between valid interrupts
const long MAX_REVOLUTION_TIME_US = 1000000;  // Max time for valid revolution (1 second)

// DUAL BLE SERVICES - RSC for data for control
#define RSC_SERVICE_UUID BLEUUID((uint16_t)0x1814)
#define RSC_MEASUREMENT_UUID BLEUUID((uint16_t)0x2A53)
#define SENSOR_POSITION_UUID BLEUUID((uint16_t)0x2A5D)
// DUAL BLE SERVICES -  FTMS for control
#define FTMS_SERVICE_UUID BLEUUID((uint16_t)0x1826)
#define FTMS_FEATURE_UUID BLEUUID((uint16_t)0x2ACC)
#define TREADMILL_DATA_UUID BLEUUID((uint16_t)0x2ACD)
#define FTMS_CONTROL_POINT_UUID BLEUUID((uint16_t)0x2AD9)
#define FTMS_STATUS_UUID BLEUUID((uint16_t)0x2ADA)
#define SUPPORTED_SPEED_RANGE_UUID BLEUUID((uint16_t)0x2AD4)
#define SUPPORTED_INCLINATION_RANGE_UUID BLEUUID((uint16_t)0x2AD5)

// FTMS Control Point Operation Codes
#define FTMS_REQUEST_CONTROL        0x00
#define FTMS_RESET                  0x01
#define FTMS_SET_TARGET_SPEED       0x02
#define FTMS_SET_TARGET_INCLINATION 0x03
#define FTMS_START_RESUME           0x07
#define FTMS_STOP_PAUSE             0x08

// FTMS Response Codes
#define FTMS_SUPPORTED              0x01
#define FTMS_SUCCESS                0x01
#define FTMS_NOT_SUPPORTED          0x02
#define FTMS_INVALID_PARAMETER      0x03

// Update intervals - FASTER UPDATES
const unsigned long DISPLAY_UPDATE_INTERVAL = 500;  // 2 times per second 

static bool testdata = false;

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
    volatile uint16_t fmtsSupport = FTMS_NOT_SUPPORTED;
    
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
    uint8_t instCadence = 85;
    uint16_t instStrideLength = 80;
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
BLECharacteristic* pFTMSFitnessFeature = nullptr;
BLECharacteristic* pFTMSSupportedSpeedRange = nullptr;
BLECharacteristic* pFTMSSupportedInclinationRange = nullptr;


AsyncWebServer server(80);

// ============================================================================
// INTERRUPT HANDLER
// ============================================================================
void IRAM_ATTR tachInterrupt() {
    unsigned long usNow = micros();

    // Prevent overflow in elapsed calculation
    if (usNow < metrics.startTime) {
        // Handle micros() overflow (happens every ~70 minutes)
        metrics.startTime = usNow;
        return;
    }

    long elapsed = usNow - metrics.startTime;
    
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
        // Add bounds checking to prevent overflow
        if (metrics.revCount < UINT_MAX) {
            metrics.revCount++;
        }
        if (metrics.totalRevCount < UINT_MAX) {
            metrics.totalRevCount++;
        }
        // Prevent distance overflow
        if (metrics.workoutDistance < (LONG_MAX - BELT_DISTANCE_MM)) {
            metrics.workoutDistance += BELT_DISTANCE_MM;
        }
        
        // Prevent accumulator overflow
        if (metrics.accumulatorInterval < (LONG_MAX - elapsed)) {
            metrics.accumulatorInterval += elapsed;
        }
    }
}

// ============================================================================
// FTMS CONTROL FUNCTIONS
// ============================================================================

bool isNotificationEnabled(BLECharacteristic* characteristic) {
    if (characteristic == nullptr || !bleData.clientConnected) {
        return false;
    }
    
    // Check CCCD descriptor for treadmill notification status
    try {
        BLEDescriptor* cccdDescriptor = characteristic->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        if (cccdDescriptor != nullptr) {
            uint8_t* cccdValue = cccdDescriptor->getValue();
            if (cccdDescriptor->getLength() >= 2) {
                uint16_t cccd = cccdValue[0] | (cccdValue[1] << 8);
                return (cccd & 0x0001) != 0; // Check notification bit for treadmill data
            }
        }
    } catch (...) {
        // Fallback for treadmill clients that don't allow CCCD reading
    }
    
    // Fallback: assume treadmill notifications enabled if connected
    return bleData.clientConnected;
}


void sendFTMSStatusUpdate(uint8_t statusCode) {
    if (!bleData.clientConnected || !pFTMSStatus) {
        Serial.println("Cannot send FTMS status - not connected or invalid characteristic");
        return;
    }
    
    // ✅ CRITICAL: Check if client has enabled notifications
    if (!isNotificationEnabled(pFTMSStatus)) {
        Serial.println("⚠️  Client hasn't enabled FTMS status notifications yet - queuing status");
        // Could store status to send later when notifications are enabled
        return;
    }

    uint8_t statusData[2] = {0x01, statusCode};
    pFTMSStatus->setValue(statusData, 2);
    
    // Add error checking for notify
    try {
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
    } catch (...) {
        Serial.println("Error sending FTMS status notification");
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
    void onWrite(BLECharacteristic* pCharacteristic) override {
        uint8_t* data = pCharacteristic->getData();
        size_t length = pCharacteristic->getLength();

        if (data == nullptr || length == 0) {
            Serial.println("FTMS write received with invalid or empty data");
            return;
        }

        uint8_t opCode = data[0];
        // Handle loopback response packets safely
        if (opCode == 0x80) {
            Serial.println("Ignoring FTMS response packet (loopback)");
            return;
        }

        Serial.printf("FTMS write received (%d bytes): ", length);
        for (size_t i = 0; i < length; i++) {
            Serial.printf("0x%02X ", data[i]);
        }
        Serial.println();

        uint8_t response[3] = {0};
        response[0] = 0x80;
        response[1] = (length > 0) ? opCode : 0x00;
        response[2] = FTMS_SUCCESS;
        
        switch (opCode) {
            case FTMS_REQUEST_CONTROL:
                Serial.println("requesting control - GRANTED!");
                metrics.controlRequested = true;
                sendFTMSStatusUpdate(0x01);
                break;
                
            case FTMS_RESET:
                Serial.println("reset command");
                resetWorkout();
                sendFTMSStatusUpdate(0x02);
                break;
                
            case FTMS_SET_TARGET_SPEED:
                if (length >= 3) {
                    uint16_t speed = (data[2] << 8) | data[1];
                    float speedKmh = speed * 0.01;
                    Serial.printf("TARGET SPEED: %.2f km/h\n", speedKmh);
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
                    Serial.printf("TARGET INCLINE: %.1f%%\n", inclinePercent);
                    metrics.targetInclination = inclinePercent;
                    metrics.currentInclination = inclinePercent;
                } else {
                    response[2] = FTMS_INVALID_PARAMETER;
                }
                break;
                
            case FTMS_START_RESUME:
                Serial.println("▶️ START/RESUME command");
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
                        Serial.println("STOP command");
                        metrics.isRunning = false;
                        metrics.isPaused = false;
                        sendFTMSStatusUpdate(0x05);
                    } else if (data[1] == 0x02) {
                        Serial.println("PAUSE command");
                        metrics.isPaused = true;
                        sendFTMSStatusUpdate(0x06);
                    }
                }
                break;
                
            default:
                Serial.printf("Unsupported FTMS command: 0x%02X\n", opCode);
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
        Serial.println("BLE client connected - initializing...");
    }
    
    void onDisconnect(BLEServer* pServer) override {
        bleData.clientConnected = false;
        metrics.controlRequested = false;
        Serial.println("=== BLE CLIENT DISCONNECTED ===");
        
        // Safely restart advertising
        if (pServer && pServer->getAdvertising()) {
            pServer->getAdvertising()->start();
        }
    }
};

// ============================================================================
// ALTERNATIVE: Monitor CCCD writes to know when notifications are enabled
// ============================================================================
// Optional: Custom CCCD callback to track when client enables/disables notifications
class CCCDCallbacks : public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor* pDescriptor) override {
        uint8_t* data = pDescriptor->getValue();
        if (pDescriptor->getLength() >= 2) {
            uint16_t cccdValue = data[0] | (data[1] << 8);
            
            BLECharacteristic* pChar = pDescriptor->getCharacteristic();
            String charUUID = pChar->getUUID().toString().c_str();
            
            Serial.printf("CCCD Write - Char: %s, Value: 0x%04X ", charUUID.c_str(), cccdValue);
            
            if (cccdValue & 0x0001) {
                Serial.println("(Notifications ENABLED)");
            } else if (cccdValue & 0x0002) {
                Serial.println("(Indications ENABLED)");
            } else {
                Serial.println("(Notifications/Indications DISABLED)");
            }
        }
    }
};

void initBLE_RSC() {
    // must be after the FTMS initalisation !!!!!!!!!
    Serial.println("Initializing BLE for RSC...");

    try {
        if (enableRSC && pServer) {
            // ========== RSC SERVICE ==========
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
            //pServer->getAdvertising()->addServiceUUID(RSC_SERVICE_UUID);
            //pServer->getAdvertising()->start();
            
            Serial.println("RSC BLE service started - Compatible with MyWoosh/Zwift");
        } // ← THIS CLOSING BRACE WAS MISSING
    } catch (const std::exception& e) {
        Serial.printf("❌ RSC BLE initialization failed: %s\n", e.what());
    } catch (...) {
        Serial.println("❌ RSC BLE initialization failed with unknown error");
    }
} 

void initFTMS_BLE() {
    Serial.println("Initializing BLE...");

    try {
        BLEDevice::init(BLE_DEVICE_NAME);
        
        pServer = BLEDevice::createServer();
        if (!pServer) {
            Serial.println("Failed to create BLE server");
            return;
        }
        
        pServer->setCallbacks(new ServerCallbacks());
    
        // ========== FTMS SERVICE ==========
        BLEService* pFTMSService = pServer->createService(FTMS_SERVICE_UUID);
        if (!pFTMSService) {
            Serial.println("Failed to create FTMS service");
            return;
        }

        // Create Treadmill Data Characteristic (Read + Notify)
        pFTMSTreadmillData = pFTMSService->createCharacteristic(
                                TREADMILL_DATA_UUID,
                                BLECharacteristic::PROPERTY_READ |
                                BLECharacteristic::PROPERTY_NOTIFY
                            );
        
        // Create Fitness Feature Characteristic (Read only)
        pFTMSFitnessFeature = pFTMSService->createCharacteristic(
                                FTMS_FEATURE_UUID,
                                BLECharacteristic::PROPERTY_READ
                                );
        
        // Create Control Point Characteristic (Write + Indicate)
        pFTMSControlPoint = pFTMSService->createCharacteristic(
                                    FTMS_CONTROL_POINT_UUID,
                                    BLECharacteristic::PROPERTY_WRITE |
                                    BLECharacteristic::PROPERTY_INDICATE
                                );
        
        // Create Status Characteristic (Read + Notify)
        pFTMSStatus = pFTMSService->createCharacteristic(
                            FTMS_STATUS_UUID,
                            BLECharacteristic::PROPERTY_READ |
                            BLECharacteristic::PROPERTY_NOTIFY
                            );
        
        // Create Supported Speed Range Characteristic (Read only)
        pFTMSSupportedSpeedRange = pFTMSService->createCharacteristic(
                                    SUPPORTED_SPEED_RANGE_UUID,
                                    BLECharacteristic::PROPERTY_READ
                                    );
        
        // Create Supported Inclination Range Characteristic (Read only)
        pFTMSSupportedInclinationRange = pFTMSService->createCharacteristic(
                                            SUPPORTED_INCLINATION_RANGE_UUID,
                                            BLECharacteristic::PROPERTY_READ
                                        );
        
        if (pFTMSTreadmillData) {
            BLE2902* ftmsDataDescriptor = new BLE2902();
            ftmsDataDescriptor->setNotifications(true);
            ftmsDataDescriptor->setIndications(false);
            pFTMSTreadmillData->addDescriptor(ftmsDataDescriptor);
            Serial.println("✅ Added CCCD to FTMS Treadmill Data");
        }
        
        if (pFTMSControlPoint) {
            BLE2902* cpDescriptor = new BLE2902();
            cpDescriptor->setIndications(true);
            cpDescriptor->setNotifications(false);
            cpDescriptor->setCallbacks(new CCCDCallbacks());
            pFTMSControlPoint->addDescriptor(cpDescriptor);
            pFTMSControlPoint->setCallbacks(new FTMSControlPointCallbacks());
            Serial.println("✅ Added CCCD to FTMS Control Point");
        }
        
        if (pFTMSStatus) {
            BLE2902* statusDescriptor = new BLE2902();
            statusDescriptor->setNotifications(true);
            statusDescriptor->setIndications(false);
            statusDescriptor->setCallbacks(new CCCDCallbacks());
            pFTMSStatus->addDescriptor(statusDescriptor);
            Serial.println("✅ Added CCCD to FTMS Status");
        }

        uint32_t ftmsFeatures = 
            (1 << 0) |  // Average Speed Supported
            (1 << 1) |  // Total Distance Supported
            (1 << 2) |  // Instantaneous Cadence Supported (ADDED FOR CADENCE)
            (1 << 3) |  // Inclination Supported
            (1 << 8) |  // Target Speed Supported
            (1 << 9) |  // Target Inclination Supported
            (1 << 18);  // Heart Rate Target Supported

        // Pack features into 8-byte array (little-endian)
        uint8_t features[8] = {0};
        features[0] = (ftmsFeatures >> 0) & 0xFF;
        features[1] = (ftmsFeatures >> 8) & 0xFF;
        features[2] = (ftmsFeatures >> 16) & 0xFF;
        features[3] = (ftmsFeatures >> 24) & 0xFF;
        features[4] = 0; // Upper 32 bits
        features[5] = 0;
        features[6] = 0;
        features[7] = 0;
        
        pFTMSFitnessFeature->setValue(features, 8);
        // Set supported speed range
        uint8_t speedRange[6] = {0};
        uint16_t minSpeed = 10;  // 1.0 km/h
        uint16_t maxSpeed = 300; // 30.0 km/h  
        uint16_t speedIncrement = 1; // 0.1 km/h
        
        speedRange[0] = minSpeed & 0xFF;
        speedRange[1] = (minSpeed >> 8) & 0xFF;
        speedRange[2] = maxSpeed & 0xFF;
        speedRange[3] = (maxSpeed >> 8) & 0xFF;
        speedRange[4] = speedIncrement & 0xFF;
        speedRange[5] = (speedIncrement >> 8) & 0xFF;
        
        pFTMSSupportedSpeedRange->setValue(speedRange, 6);
        
        // Set supported inclination range
        uint8_t inclinationRange[6] = {0};
        int16_t minInclination = -150; // -15.0%
        int16_t maxInclination = 400;  // 40.0%
        uint16_t inclinationIncrement = 5; // 0.5%
        
        inclinationRange[0] = minInclination & 0xFF;
        inclinationRange[1] = (minInclination >> 8) & 0xFF;
        inclinationRange[2] = maxInclination & 0xFF;
        inclinationRange[3] = (maxInclination >> 8) & 0xFF;
        inclinationRange[4] = inclinationIncrement & 0xFF;
        inclinationRange[5] = (inclinationIncrement >> 8) & 0xFF;
        
        pFTMSSupportedInclinationRange->setValue(inclinationRange, 6);

        // Initialize status
        uint8_t status = 0x00; // Machine idle
        if (pFTMSStatus) {
            pFTMSStatus->setValue(&status, 1);
        }
      
        pFTMSService->start();
        Serial.println("✅ FTMS Service started");    
        
        // Start advertising
        BLEAdvertisementData advData;
        advData.setName("Horizon Elite T507");
        advData.setCompleteServices(BLEUUID((uint16_t)0x1826));
        advData.setFlags(0x06); // LE General Discoverable + BR/EDR not supported

        BLEAdvertising* pAdvertising = pServer->getAdvertising();
        pAdvertising->addServiceUUID(FTMS_SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->setMinPreferred(0x0);
        pAdvertising->setAdvertisementData(advData);
        pAdvertising->start();
        Serial.println("🚀 DUAL BLE SERVICES STARTED:");
        Serial.println("   RSC (0x1814) - Data transmission");
        Serial.println("   FTMS (0x1826) - Training control");
        Serial.println("✅ BLE initialization complete with explicit CCCD setup");
        
    } catch (const std::exception& e) {
        Serial.printf("❌ BLE initialization failed: %s\n", e.what());
    } catch (...) {
        Serial.println("❌ BLE initialization failed with unknown error");
    }
}

void sendRSC_BLE_Data() {
    if ( !pRSCMeasurement) {
        Serial.println("RSC Data NOT sent - Client disconnected or characteristic null");
        return;
    }
    
    // RSC CADENCE-ONLY DATA (FTMS handles speed separately)
    // MAIN FOCUS: Calculate cadence from treadmill belt RPM
    float stepsPerMinute = metrics.rpm * 2.0; // 2 steps per belt revolution
    bleData.instCadence = (uint8_t)(stepsPerMinute > 255 ? 255 : stepsPerMinute);
    
    // Minimal RSC fields (apps ignore these since FTMS provides real data)
    bleData.instSpeed = 256; // Minimal speed (apps use FTMS speed instead)
    bleData.totalDistance = 0; // No distance (apps use FTMS distance instead)
    bleData.instStrideLength = 800; // ~80cm average stride length
/*
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
*/  
    byte dataPacket[10] = {
        3,  // Flags: stride length + distance present (minimal required)
        (byte)bleData.instSpeed, (byte)(bleData.instSpeed >> 8),    // Minimal (ignored by apps)
        bleData.instCadence,                                        // CADENCE - THE ONLY IMPORTANT FIELD
        (byte)bleData.instStrideLength, (byte)(bleData.instStrideLength >> 8), // Stride
        (byte)bleData.totalDistance, (byte)(bleData.totalDistance >> 8),        // Minimal (ignored)
        (byte)(bleData.totalDistance >> 16), (byte)(bleData.totalDistance >> 24) // Minimal (ignored)
    };
    
    pRSCMeasurement->setValue(dataPacket, 10);
    pRSCMeasurement->notify();
    pSensorPosition->setValue(bleData.fakePos, 1);
    
    // Debug output focused on cadence only
    static int cadenceDebugCount = 0;
    if (++cadenceDebugCount % 40 == 0) { // Every 20 seconds
        Serial.printf("✅ RSC Cadence ONLY: %.0f spm (belt RPM: %.1f) - FTMS handles speed/distance\n", 
                    stepsPerMinute, metrics.rpm);
        Serial.println();
    }
}

void sendFTMS_BLE_Data() {
    if (!bleData.clientConnected) return;

    // ========== SEND FTMS TREADMILL DATA ==========
    sendRSC_BLE_Data();
    
    // ========== SEND FTMS TREADMILL DATA ==========
    if (pFTMSTreadmillData && metrics.controlRequested) {
        if (isNotificationEnabled(pFTMSTreadmillData)) {
            uint8_t ftmsData[20] = {0};
            size_t index = 0;
            const size_t MAX_FTMS_SIZE = 20;
            
            // CRITICAL: Based on official FTMS Treadmill Data specification ONLY
            // Reference: org.bluetooth.characteristic.treadmill_data.xml (UUID: 2ACD)
            
            // Calculate values with proper TREADMILL-specific resolutions
            float speedKmh = (metrics.mps + metrics.mpsOffset) * 3.6;
            uint16_t instantaneousSpeed = (uint16_t)(speedKmh * 100); // 0.01 km/h resolution
            uint16_t averageSpeed = instantaneousSpeed; // Same as instantaneous for simplicity
            uint32_t totalDistance = metrics.workoutDistance / 1000; // Convert mm to meters (1m resolution)
            int16_t inclination = (int16_t)(metrics.currentInclination * 10); // 0.1% resolution
            uint16_t elapsedTime = (millis() - metrics.sessionStartTime) / 1000; // 1s resolution
            
            // TREADMILL CADENCE: Steps per minute from belt RPM  
            // For treadmills, cadence represents STEPS PER MINUTE, not pedal RPM
            float stepsPerMinute = metrics.rpm * 2.0; // Assume 2 steps per belt revolution
            uint16_t instantaneousCadence = (uint16_t)(stepsPerMinute * 2); // 0.5 steps/min resolution
            
            // CRITICAL FIX: Treadmill FTMS flags - cadence may not be officially supported
            // Based on real-world treadmill implementations that work with apps
            uint16_t flags = 0x0000;
            flags |= (0 << 0);  // More Data = 0 (this packet is complete)
            flags |= (1 << 1);  // Average Speed Present = 1
            flags |= (1 << 2);  // Total Distance Present = 1
            // NOTE: Bit 3 (Instantaneous Cadence) may not be in official treadmill spec
            // flags |= (1 << 3);  // Instantaneous Cadence Present - REMOVED FOR TESTING
            flags |= (1 << 5);  // Inclination Present = 1 (skip bit 4)
            flags |= (1 << 6);  // Elapsed Time Present = 1
            
            // Build FTMS Treadmill Data packet in EXACT specification order:
            
            // 1. Flags field (2 bytes) - MANDATORY for all treadmills
            ftmsData[index++] = flags & 0xFF;
            ftmsData[index++] = (flags >> 8) & 0xFF;
            
            // 2. Instantaneous Speed (2 bytes) - ALWAYS PRESENT in treadmill data
            ftmsData[index++] = instantaneousSpeed & 0xFF;
            ftmsData[index++] = (instantaneousSpeed >> 8) & 0xFF;
            
            // 3. Average Speed (2 bytes) - Present if flag bit 1 = 1
            ftmsData[index++] = averageSpeed & 0xFF;
            ftmsData[index++] = (averageSpeed >> 8) & 0xFF;
            
            // 4. Total Distance (3 bytes) - Present if flag bit 2 = 1  
            ftmsData[index++] = totalDistance & 0xFF;
            ftmsData[index++] = (totalDistance >> 8) & 0xFF;
            ftmsData[index++] = (totalDistance >> 16) & 0xFF;
            
            // SKIP: Instantaneous Cadence - Not in official treadmill FTMS spec
            // Most treadmill apps get cadence from separate sensors or calculate it
            // The official FTMS treadmill spec may not include cadence field
            
            // 5. Inclination (2 bytes) - Present if flag bit 5 = 1
            ftmsData[index++] = inclination & 0xFF;
            ftmsData[index++] = (inclination >> 8) & 0xFF;
            
            // 6. Elapsed Time (2 bytes) - Present if flag bit 6 = 1
            ftmsData[index++] = elapsedTime & 0xFF;
            ftmsData[index++] = (elapsedTime >> 8) & 0xFF;
            
            // Send the properly formatted FTMS treadmill packet
            if (index <= MAX_FTMS_SIZE && index > 0) {
                try {
                    pFTMSTreadmillData->setValue(ftmsData, index);
                    pFTMSTreadmillData->notify();
                
                    // Enhanced debug output for treadmill-specific values
                    static int successCount = 0;
                    if (++successCount % 20 == 0) { // Every 10 seconds
                        Serial.printf("✅ TREADMILL FTMS: Speed=%.1fkm/h, Distance=%dm\n", 
                                    speedKmh, (int)totalDistance);
                        Serial.println();
                        Serial.printf("    Incline=%.1f%%, Time=%ds, Flags=0x%04X, Size=%d bytes (%d)\n", 
                                    metrics.currentInclination, elapsedTime, flags, index, successCount);
                        Serial.println();
                        Serial.printf("    NOTE: Cadence=%.0f spm not sent (not in treadmill FTMS spec)\n", stepsPerMinute);
                        Serial.println();
                        
                        // Debug hex dump for troubleshooting
                        if (successCount <= 60) { // First 30 seconds
                            Serial.print("    Treadmill packet: ");
                            for (size_t i = 0; i < index; i++) {
                                Serial.printf("%02X ", ftmsData[i]);
                            }
                            Serial.println();
                        }
                    }
                } catch (const std::exception& e) {
                    Serial.printf("❌ FTMS treadmill notify failed: %s\n", e.what());
                } catch (...) {
                    Serial.println("❌ FTMS treadmill notify failed with unknown error");
                }
            }
        } else {
            static int notEnabledCount = 0;
            if (++notEnabledCount % 200 == 0) {
                Serial.println("⚠️ FTMS Treadmill notifications not enabled by client yet");
            }
        }
    } else {
        static int noControlCount = 0;
        if (++noControlCount % 200 == 0) {
            if (!pFTMSTreadmillData) {
                Serial.println("⚠️ FTMS Treadmill Data characteristic is null");
            } else if (!metrics.controlRequested) {
                Serial.println("⚠️ FTMS Control not requested by treadmill client yet");
            }
        }
    }
}

// ============================================================================
// CALCULATION FUNCTIONS
// ============================================================================
void calculateRPM() {
    static unsigned long lastValidRPMTime = 0;
    unsigned long currentTime = millis();
    
    if (metrics.revCount > 0 && metrics.accumulatorInterval !=0) {
        metrics.rpm = 60000000.0 / (metrics.accumulatorInterval / metrics.revCount);
        metrics.sensorPulsPerSecond = (metrics.revCount * 60 *100000) / metrics.accumulatorInterval; // 1 Second
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
    //Serial.printf("Manual Offset UP: %.1f km/h\n", metrics.mpsOffset * 3.6);
}

void adjustOffsetDown() {
    if (metrics.mpsOffset >= 0.139) {
        metrics.mpsOffset -= 0.139;
    } else {
        metrics.mpsOffset = 0;
    }
    //Serial.printf("Manual Offset DOWN: %.1f km/h\n", metrics.mpsOffset * 3.6);
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

  <p><a href="/testdata"><button class="button button3">%TESTDATABUTTONTEXT%</button></a></p>

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

String getTestDataButtonText() {
    if (testdata) {
        return String("Turn Test Data OFF");
    } else {
        return String("Turn Test Data ON");
    }
}

String processTemplate(const String& var) {
    // Add bounds checking for all string operations
    if (var.length() == 0 || var.length() > 25) {  // Increased length limit for button text
        return String("Error");
    }
    
    if (var == "SPEED") return getSpeed();
    if (var == "SENSORPULSE") return getSensorInterupts();
    if (var == "DISTANCE") return getDistance();
    if (var == "HOUR") return getHour();
    if (var == "MINUTE") return getMinute();
    if (var == "SECOND") return getSecond();
    if (var == "OFFSET") return getOffset();
    if (var == "RPM") return getRPM();
    if (var == "REVS") return getTotalRevs();
    if (var == "TESTDATABUTTONTEXT") return getTestDataButtonText();
    return String("Unknown");
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
    Serial.println();

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
    
    server.on("/testdata", HTTP_GET, [](AsyncWebServerRequest *request) {
        // FIXED: Toggle state here, not in template function
        testdata = !testdata;
        if(testdata) {
            metrics.isRunning = true;
        } else {
            //metrics.isRunning = false;
        }
        Serial.printf("Test data toggled: %s\n", testdata ? "ON" : "OFF");
        Serial.println();
        request->send_P(200, "text/html", HTML_TEMPLATE, processTemplate);
    });

    server.begin();
    Serial.println("Web server started");
    testdata = false; // make sure testdata not generated on start
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
    Serial.println("Full bidirectional support");

    // PROPERLY INITIALIZE METRICS
    metrics.startTime = micros();
    metrics.workoutStartTime = millis();
    metrics.sessionStartTime = millis();
    metrics.lastUpdate = millis();
    
    //initBLE();
    initFTMS_BLE();
    initBLE_RSC();
    initTachometer();
    initWebServer();
    
    Serial.println("Connect for training programs!");
    Serial.printf("Web Interface: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println();
    Serial.printf("BLE Device: %s\n", BLE_DEVICE_NAME);
    Serial.println();
    Serial.println("\n=== SETUP COMPLETE ===");
    Serial.println();
}


// ============================================================================
// TESTING PROCEDURE FOR KINOMAP
// ============================================================================
// 2. Add this temporary test function for manual data generation
void generateTestData() {
    static unsigned long lastTestTime = 0;
    unsigned long currentTime = millis();
    tachInterrupt();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Handle delayed FTMS status update
    static bool statusSent = false;
    static unsigned long connectionTime = 0;
    
    if (bleData.clientConnected && !statusSent) {
        if (connectionTime == 0) {
            connectionTime = currentTime;
        }
        // Wait 1 second after connection before sending status
        else if (currentTime - connectionTime > 1000) {
            if (pFTMSStatus != nullptr) {
                sendFTMSStatusUpdate(0x01);
                statusSent = true;
                Serial.println("FTMS control granted after stabilization");
            }
        }
    }
    
    // Reset when disconnected
    if (!bleData.clientConnected) {
        statusSent = false;
        connectionTime = 0;
    }
    
    // Your existing loop logic
    if (currentTime - metrics.lastUpdate >= DISPLAY_UPDATE_INTERVAL) {
        metrics.lastUpdate = currentTime;
        updateMetrics();
        
        // ADD THIS LINE FOR TESTING:
        if (testdata) {
            generateTestData(); // Remove this after testing with real treadmill
        }

        // Send BLE data more frequently when client connected
        if (bleData.clientConnected) {
            sendFTMS_BLE_Data();
            
            // Debug output every few seconds
            static int debugCounter = 0;
            debugCounter++;
            if (debugCounter >= 60 && testdata) { // Every 3 seconds (500ms * 6)
                Serial.printf("Status: RPM=%d, Speed=%.1f km/h, Distance=%dm, Running=%s\n",
                            (int)metrics.rpm,
                            (metrics.mps + metrics.mpsOffset) * 3.6,
                            (int)(metrics.workoutDistance / 1000),
                            metrics.isRunning ? "YES" : "NO");
                Serial.println();
                debugCounter = 0;
            }
        }
        
        static int wifiCounter = 0;
        wifiCounter++;
        if (wifiCounter >= 100 * DISPLAY_UPDATE_INTERVAL) {
            checkWiFiConnection();
            wifiCounter = 0;
        }
    }
    
    delay(10);
}