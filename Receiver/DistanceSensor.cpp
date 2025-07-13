#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trig, int echo, int dfRx, int dfTx, int maxDist) {
    trigPin = trig;
    echoPin = echo;
    dfRxPin = dfRx;
    dfTxPin = dfTx;
    maxDistance = maxDist;

    sonar = new NewPing(trigPin, echoPin, maxDistance);
    serial = new HardwareSerial(1);
    dfPlayer = new DFRobotDFPlayerMini();

    isPlaying = false;
    lastCheck = 0;
    checkInterval = 100;
}

bool DistanceSensor::begin() {
    serial->begin(9600, SERIAL_8N1, dfRxPin, dfTxPin);

    Serial.println("Starting DFPlayer...");
    if (!dfPlayer->begin(*serial)) {
        Serial.println("DFPlayer could not be started.");
        return false;
    }

    Serial.println("DFPlayer successfully started!");
    dfPlayer->volume(25);
    return true;
}

void DistanceSensor::update() {
    if (millis() - lastCheck > checkInterval) {
        lastCheck = millis();

        unsigned int distance = sonar->ping_cm();

        if (distance > 0 && distance < 20) {
            if (!isPlaying) {
                Serial.print("PROXIMITY ALERT! Distance: ");
                Serial.print(distance);
                Serial.println(" cm");
                dfPlayer->play(2); // play "0002.mp3"
                isPlaying = true;
            }
        } else {
            isPlaying = false;
        }
    }
}

int DistanceSensor::getDistance() {
    return sonar->ping_cm();
}

void DistanceSensor::setVolume(int volume) {
    dfPlayer->volume(volume);
}
