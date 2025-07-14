#include "LedRing.h"

#define NUM_LEDS 12

// Predefined rainbow colors for each LED position - evenly spaced across spectrum
uint32_t rainbowColors[NUM_LEDS] = {
  0xFF0000,  // Red
  0xFF3300,  // Red-Orange
  0xFF6600,  // Orange
  0xFF9900,  // Yellow-Orange
  0xFFCC00,  // Golden Yellow
  0xFFFF00,  // Pure Yellow
  0x99FF00,  // Yellow-Green
  0x33FF00,  // Lime Green
  0x00FF33,  // Green
  0x00FF99,  // Cyan-Green
  0x0099FF,  // Sky Blue
  0x3300FF   // Blue-Purple
};

LedRing::LedRing(int pin) : ring(NUM_LEDS, pin, NEO_GRB + NEO_KHZ800) {
  speedValue = 0;
}

void LedRing::begin() {
  ring.begin();
  ring.show();
}

void LedRing::setSpeed(int speed) {
  speedValue = constrain(speed, 0, 254);
}

void LedRing::update() {
  ring.clear();
  int ledsOn = map(speedValue, 0, 254, 0, NUM_LEDS);

  for (int i = 0; i < ledsOn; i++) {
    ring.setPixelColor(i, rainbowColors[i]);
  }

  ring.show();
}