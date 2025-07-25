/*
 * ESP32-S3 Super Mini ▸ MPU-6050 ▸ BLE ▸ LCD
 * Streams IMU data over BLE in JSON format compatible with Python client
 * Uses Classic ESP32 BLE library
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ---------- pins ---------- */
constexpr uint8_t PIN_SDA   = 8;
constexpr uint8_t PIN_SCL   = 9;
constexpr uint8_t LED_PIN   = 38;  // RGB LED on ESP32-S3 Super Mini

/* ---------- BLE UUIDs ---------- */
#define SERVICE_UUID      "12345678-1234-1234-1234-123456789012"
#define DATA_CHAR_UUID    "abcdef12-3456-789a-bcde-123456789abc"
#define CONTROL_CHAR_UUID "12345678-1234-1234-1234-123456789013"

/* ---------- timing ---------- */
volatile uint32_t imuInterval = 50;     // ms (default 20 Hz)
uint32_t lastImu   = 0;
uint32_t frameCnt  = 0;
bool displayOn     = true;

/* ---------- IMU ---------- */
Adafruit_MPU6050 mpu;

/* ---------- BLE ---------- */
BLEServer* pServer = nullptr;
BLECharacteristic* pDataChar = nullptr;
BLECharacteristic* pControlChar = nullptr;
bool deviceConnected = false;

/* ---------- BLE Server Callbacks ---------- */
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      digitalWrite(LED_PIN, LOW); // Turn on LED when connected
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      digitalWrite(LED_PIN, HIGH); // Turn off LED when disconnected
      BLEDevice::startAdvertising(); // Restart advertising
    }
};

/* ---------- Control Characteristic Callbacks ---------- */
class ControlCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string cmd = pCharacteristic->getValue();
      
      if (cmd.rfind("RATE:", 0) == 0) {            // RATE:xxx
        int hz = atoi(cmd.c_str() + 5);
        if (hz >= 1 && hz <= 500) {
          imuInterval = 1000 / hz;
        }
      }
      else if (cmd == "DISPLAY:ON") {
        displayOn = true;
      }
      else if (cmd == "DISPLAY:OFF") {
        displayOn = false;
      }
    }
};

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // LED off initially
  
  /* I²C + IMU */
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  delay(100);
  
  // Try multiple I2C addresses for MPU-6050
  bool mpu_found = false;
  uint8_t addresses[] = {0x68, 0x69};
  
  for (uint8_t addr : addresses) {
    if (mpu.begin(addr, &Wire)) {
      mpu_found = true;
      break;
    }
  }
  
  if (!mpu_found) {
    // Blink LED rapidly to show MPU error
    for(int i = 0; i < 10; i++) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
    while (true) { delay(1000); }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /* BLE Setup */
  BLEDevice::init("ESP32-IMU");
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Create Data Characteristic (for streaming IMU data)
  pDataChar = pService->createCharacteristic(
                DATA_CHAR_UUID,
                BLECharacteristic::PROPERTY_NOTIFY
              );
  pDataChar->addDescriptor(new BLE2902());
  
  // Create Control Characteristic (for receiving commands)
  pControlChar = pService->createCharacteristic(
                   CONTROL_CHAR_UUID,
                   BLECharacteristic::PROPERTY_WRITE
                 );
  pControlChar->setCallbacks(new ControlCallbacks());
  
  // Start the service
  pService->start();
  
  // Setup and start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  
  // Blink LED twice to show successful initialization
  for(int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
  digitalWrite(LED_PIN, HIGH); // LED off, ready for connections
}

/* ---------- LOOP ---------- */
void loop() {
  uint32_t now = millis();

  /* IMU streaming */
  if (now - lastImu >= imuInterval) {
    lastImu = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Build JSON payload */
    JsonDocument doc;
    doc["frame"]     = frameCnt++;
    doc["timestamp"] = now;
    JsonArray accel = doc["accel"].to<JsonArray>();
    accel.add(a.acceleration.x);
    accel.add(a.acceleration.y);
    accel.add(a.acceleration.z);
    JsonArray gyro = doc["gyro"].to<JsonArray>();
    gyro.add(g.gyro.x);
    gyro.add(g.gyro.y);
    gyro.add(g.gyro.z);

    char buffer[192];
    size_t len = serializeJson(doc, buffer);

    /* Send BLE notification if connected */
    if (deviceConnected && pDataChar) {
      pDataChar->setValue((uint8_t*)buffer, len);
      pDataChar->notify();
    }

    /* LED status indication */
    if (deviceConnected) {
      // Fast blink when connected and streaming
      digitalWrite(LED_PIN, (frameCnt % 4 < 2) ? LOW : HIGH);
    } else {
      // Slow blink when advertising
      digitalWrite(LED_PIN, (frameCnt % 40 < 20) ? LOW : HIGH);
    }
  }

  delay(1);
}