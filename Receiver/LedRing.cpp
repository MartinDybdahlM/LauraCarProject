#include "LedRing.h"

#define NUM_LEDS 12

uint32_t rainbowColors[NUM_LEDS] = {
  0xFF0000,  // Red
  0xFF4500,  // Orange Red
  0xFF8C00,  // Dark Orange
  0xFFD700,  // Gold
  0xFFFF00,  // Yellow
  0x9AFF9A,  // Light Green
  0x00FF00,  // Green
  0x00FF7F,  // Spring Green
  0x00FFFF,  // Cyan
  0x0080FF,  // Sky Blue
  0x0000FF,  // Blue
  0x8A2BE2   // Blue Violet
};

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
  int ledsOn = map(speedValue, 0, 254, 0, NUM_LEDS);

  for (int i = 0; i < ledsOn; i++) {
    ring.setPixelColor(i, rainbowColors[i]);
  }

  ring.show();
}