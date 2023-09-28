/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
// #define FASTLED_ALLOW_INTERRUPTS 0
#include "Arduino.h"
#include "FastLED.h"

#define NUM_LEDS 20
#define DATA_PIN 0
// #define LED_BUILTIN 13

CRGB leds[NUM_LEDS];

elapsedMillis tic;
uint8_t hue = 0;

// void setup() {
//   Serial.begin(115200);
//   Serial.println("FastLed Test");

//   FastLED.addLeds<WS2812, DATA_PIN, GBR>(leds, NUM_LEDS);
//   FastLED.setBrightness(255);
// }

// void loop() {
//     for (int i = 0; i < NUM_LEDS; i++) {
//     leds[i].setHue(((i * 8) + hue) & 255); 
//   }
//   FastLED.show();
//   hue++; 
//   delay(10);
// }


// void loop() {
//   FastLED.show();
// }

void setup() {
  Serial.begin(115200);
  Serial.println("FastLed Test");

  FastLED.addLeds<WS2812, DATA_PIN, GBR>(leds, NUM_LEDS); 
  FastLED.clear();
  FastLED.setBrightness(50);
  for (int i=0; i < NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
  }
}

void loop() {
  if (tic >= 333) {
    /* Shift prior colors down all the LEDs */
    for (int i=NUM_LEDS-1; i > 0; i--) {
      leds[i] = leds[i - 1];
    }
    /* Turn the first LED on with a random color */
    uint8_t red = random8();
    uint8_t green = random8();
    uint8_t blue = random8();
    Serial.printf("R=%3d G=%3d B=%3d\n", red, green, blue);
    leds[0].setRGB(red, green, blue);

    FastLED.show();
    tic = 0;
  }
}


// void setup() {
//     delay(2000);
//     FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // GRB ordering is typical
// }

// void loop() {
//     for(int whiteLed = 0; whiteLed < NUM_LEDS; whiteLed = whiteLed + 1) {
//         leds[whiteLed] = CRGB::White;
//         FastLED.show(100);
//         delay(100);
//         leds[whiteLed] = CRGB::Black;
//      }
// }

