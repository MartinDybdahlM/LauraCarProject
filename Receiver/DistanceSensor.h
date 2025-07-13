#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <NewPing.h>
#include "AudioPlayer.h"

class DistanceSensor {
private:
    NewPing* sonar;
    AudioPlayer* audioPlayer;

    bool isPlaying;
    unsigned long lastCheck;
    unsigned long checkInterval;

    int trigPin;
    int echoPin;
    int maxDistance;
    int dfRxPin;
    int dfTxPin;

public:
    DistanceSensor(int trig, int echo, int dfRx, int dfTx, int maxDist = 400);
    bool begin();
    void update();
    int getDistance();
};

#endif
