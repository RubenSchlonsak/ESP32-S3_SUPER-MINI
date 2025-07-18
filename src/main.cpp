/**
 * main.cpp – LED‑Test (WS2812/WS2818) auf GPIO 48
 *
 * Board:  ESP32‑S3 SuperMini
 * Pin:    GPIO 48  (Data‑In der adressierbaren LED)
 * Bibliothek: Adafruit_NeoPixel
 *
 * PlatformIO‑Beispiel:
 *   lib_deps = adafruit/Adafruit NeoPixel
 */

#include <FastLED.h>

#define NUM_LEDS    1
#define DATA_PIN    48
#define BRIGHTNESS  32   // Max = 255

CRGB leds[NUM_LEDS];

unsigned long lastHello = 0;
const unsigned long helloInterval = 2000;  // 2 Sekunden

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Bei USB-CDC auf Verbindung warten
  Serial.println("LED Test gestartet.");

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  // LED blinkt: An 500 ms, Aus 500 ms → 1s Zyklus
  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(500);
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);

  // Alle 2 Sekunden: Hello world (nicht blockierend!)
  if (millis() - lastHello >= helloInterval) {
    Serial.println("Hello world");
    lastHello = millis();
  }
}
