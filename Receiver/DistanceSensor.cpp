#include "DistanceSensor.h"

DistanceSensor::DistanceSensor(int trig, int echo, int maxDist) {
    trigPin = trig;
    echoPin = echo;
    maxDistance = maxDist;

    sonar = new NewPing(trigPin, echoPin, maxDistance);

    isPlaying = false;
    lastCheck = 0;
    checkInterval = 100;
}

bool DistanceSensor::begin() {
    Serial.println("Initializing DistanceSensor...");
    Serial.println("DistanceSensor successfully initialized!");
    return true;
}

void DistanceSensor::update() {
    if (millis() - lastCheck > checkInterval) {
        lastCheck = millis();

        unsigned int distance = sonar->ping_cm();

        // Add distance reading to serial output for debugging
        //Serial.print("Distance: ");
        //Serial.print(distance);
        //Serial.println(" cm");
 
        if (distance < 0) { 
            Serial.print ("Distance under 0! Distance: ");
            Serial.print(distance);
            Serial.println(" cm");
            
        }
        if (distance < 20) {
            if (!isPlaying) {
                Serial.print("PROXIMITY ALERT! Distance: ");
                Serial.print(distance);
                Serial.println(" cm");
                isPlaying = true;
            }
        } else {
           if (isPlaying){
                Serial.println("Distance > 20cm, stopping audio");
                isPlaying = false;
           }

           
        }
    }
}

int DistanceSensor::getDistance() {
    return sonar->ping_cm();
}

bool DistanceSensor::isProximityAlert() {
    return isPlaying;
}
