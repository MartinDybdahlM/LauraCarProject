#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <NewPing.h>

class DistanceSensor {
private:
    NewPing* sonar;

    bool isPlaying;
    unsigned long lastCheck;
    unsigned long checkInterval;

    int trigPin;
    int echoPin;
    int maxDistance;

public:
    DistanceSensor(int trig, int echo, int maxDist = 400);
    bool begin();
    void update();
    int getDistance();
    bool isProximityAlert(); // Check if proximity alert should be triggered
};

#endif
