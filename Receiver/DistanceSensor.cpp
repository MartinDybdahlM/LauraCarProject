#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trig, int echo, int dfRx, int dfTx, int maxDist) {
    trigPin = trig;
    echoPin = echo;
    dfRxPin = dfRx;
    dfTxPin = dfTx;
    maxDistance = maxDist;

    sonar = new NewPing(trigPin, echoPin, maxDistance);
    audioPlayer = new AudioPlayer(dfRxPin, dfTxPin);

    isPlaying = false;
    lastCheck = 0;
    checkInterval = 100;
}

bool DistanceSensor::begin() {
    Serial.println("Initializing DistanceSensor...");

    if (!audioPlayer->begin()) {
        Serial.println("Error initializing AudioPlayer");
        return false;
    }

    Serial.println("DistanceSensor successfully initialized!");
    return true;
}

void DistanceSensor::update() {
    if (millis() - lastCheck > checkInterval) {
        lastCheck = millis();

        unsigned int distance = sonar->ping_cm();

        // Add distance reading to serial output for debugging
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");

        if (distance > 0 && distance < 20) {
            if (!isPlaying) {
                Serial.print("PROXIMITY ALERT! Distance: ");
                Serial.print(distance);
                Serial.println(" cm");

                audioPlayer->playSirenSound();
                isPlaying = true;
            }
        } else {
            if (isPlaying) {
                Serial.println("Distance > 20cm, stopping audio");
                audioPlayer->stop();
            }
            isPlaying = false;
        }
    }
}

int DistanceSensor::getDistance() {
    return sonar->ping_cm();
}

void DistanceSensor::setVolume(int volume) {
    audioPlayer->setVolume(volume);
}

void DistanceSensor::testAudio() {
    audioPlayer->runTest();
}
