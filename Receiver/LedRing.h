#ifndef LED_RING_H
#define LED_RING_H

#include <Adafruit_NeoPixel.h>

class LedRing {
  private:
    Adafruit_NeoPixel ring;
    int speedValue;

  public:
    LedRing(int pin);
    void begin();
    void setSpeed(int speed);
    void update();
};

#endif