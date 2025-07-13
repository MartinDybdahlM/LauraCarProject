#include "LedRing.h"

#define NUM_LEDS 12

LedRing::LedRing(int pin) : ring(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800) {
  speedValue = 0;
}

void LedRing::begin() {
  ring.begin();
  ring.show();
}

void LedRing::setSpeed(int speed) {
  speedValue = constrain(speed, 0, 255);
}

void LedRing::update() {
  ring.clear();
  int ledsOn = map(speedValue, 0, 255, 0, NUM_LEDS);

  for (int i = 0; i < ledsOn; i++) {
    uint32_t color;
    if (i < 4)
      color = ring.Color(255, 30, 30);
    else if (i < 8)
      color = ring.Color(180, 0, 0);
    else
      color = ring.Color(80, 0, 0);

    ring.setPixelColor(i, color);
  }

  ring.show();
}