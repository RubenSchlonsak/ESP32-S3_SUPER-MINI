/**************************************************************************
 *  ESP32‑S3 ▸ MPU‑6050 (0x69) ▸ FastLED
 *  Blinks one NeoPixel, prints “Hello world” every 2 s,
 *  and streams IMU data at 20 Hz.
 *
 *  Board tested: ESP32‑S3‑DevKit‑C‑1
 *  Libraries   : FastLED 3.6.1, Adafruit_MPU6050 2.5.0
 **************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <FastLED.h>

/* ---------- pins ---------- */
constexpr uint8_t PIN_SDA   = 8;     // safe I²C pins on most ESP32‑S3 boards
constexpr uint8_t PIN_SCL   = 9;
constexpr uint8_t DATA_PIN  = 48;    // NeoPixel (any non‑boot pin)

/* ---------- LED config ---------- */
#define NUM_LEDS    1
#define BRIGHTNESS  32
CRGB leds[NUM_LEDS];

/* ---------- timing ---------- */
constexpr uint32_t BLINK_INTERVAL = 500;   // ms
constexpr uint32_t HELLO_INTERVAL = 2000;  // ms
constexpr uint32_t IMU_INTERVAL   = 50;    // ms → 20 Hz

uint32_t lastBlink  = 0;
uint32_t lastHello  = 0;
uint32_t lastImu    = 0;
bool      ledOn     = false;

/* ---------- IMU ---------- */
Adafruit_MPU6050 mpu;

/* ---------- SETUP ---------- */
void setup()
{
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // wait up to 2 s for terminal

  Serial.println(F("MPU‑6050 + FastLED demo starting"));

  /* LED */
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  /* I²C + IMU */
  Wire.begin(PIN_SDA, PIN_SCL, 400000);   // 400 kHz Fast mode
  // Adafruit driver: begin(addr, &Wire) — returns false if chip absent
  if (!mpu.begin(0x69, &Wire)) {
    Serial.println(F("ERROR: MPU‑6050 not found at 0x69!"));
    while (true) { delay(1); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // light filtering

  Serial.println(F("MPU‑6050 ready"));
}

/* ---------- LOOP ---------- */
void loop()
{
  uint32_t now = millis();

  /* 1) blink */
  if (now - lastBlink >= BLINK_INTERVAL) {
    lastBlink = now;
    ledOn = !ledOn;
    leds[0] = ledOn ? CRGB::Purple : CRGB::Black;
    FastLED.show();
  }

  /* 2) hello */
  if (now - lastHello >= HELLO_INTERVAL) {
    lastHello = now;
    Serial.println(F("Hello world"));
  }

  /* 3) IMU @ 20 Hz */
  if (now - lastImu >= IMU_INTERVAL) {
    lastImu = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.printf(
      "ax=%+.3f  ay=%+.3f  az=%+.3f [m/s²]  |  "
      "gx=%+.2f  gy=%+.2f  gz=%+.2f [°/s]\n",
      a.acceleration.x, a.acceleration.y, a.acceleration.z,
      g.gyro.x,        g.gyro.y,        g.gyro.z
    );
  }

  delay(1);     // tiny yield for Wi‑Fi/BLE/USB
}
