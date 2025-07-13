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

    // Give the DFPlayer time to initialize
    delay(1000);

    Serial.println("Starting DFPlayer...");
    if (!dfPlayer->begin(*serial)) {
        Serial.println("DFPlayer could not be started.");
        Serial.println("Check:");
        Serial.println("1. Wiring connections");
        Serial.println("2. SD card is inserted");
        Serial.println("3. Audio file 0002.mp3 exists on SD card");
        return false;
    }

    Serial.println("DFPlayer successfully started!");

    // Wait for DFPlayer to be ready
    delay(200);

    dfPlayer->volume(25);
    Serial.print("Volume set to: 25");

    // Check if SD card is detected
    delay(100);
    int fileCount = dfPlayer->readFileCounts();
    Serial.print("Files found on SD card: ");
    Serial.println(fileCount);

    if (fileCount <= 0) {
        Serial.println("Warning: No files found on SD card or SD card not detected");
    }

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
                Serial.println("Attempting to play audio file 0002.mp3...");

                dfPlayer->play(2); // play "0002.mp3"
                isPlaying = true;

                // Give some time for the command to be processed
                delay(100);

                // Check if the player is actually playing
                if (dfPlayer->available()) {
                    uint8_t type = dfPlayer->readType();
                    int value = dfPlayer->read();
                    Serial.print("DFPlayer response - Type: ");
                    Serial.print(type);
                    Serial.print(", Value: ");
                    Serial.println(value);
                }
            }
        } else {
            if (isPlaying) {
                Serial.println("Distance > 20cm, stopping audio");
            }
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

void DistanceSensor::testAudio() {
    Serial.println("=== DF Player Audio Test ===");

    // Test volume setting
    Serial.println("Setting volume to 30...");
    dfPlayer->volume(30);
    delay(100);

    // Test file count
    int fileCount = dfPlayer->readFileCounts();
    Serial.print("Files on SD card: ");
    Serial.println(fileCount);

    if (fileCount > 0) {
        Serial.println("Attempting to play file 1 (0001.mp3)...");
        dfPlayer->play(1);
        delay(3000); // Play for 3 seconds

        Serial.println("Attempting to play file 2 (0002.mp3)...");
        dfPlayer->play(2);
        delay(3000); // Play for 3 seconds

        Serial.println("Stopping playback...");
        dfPlayer->stop();
    } else {
        Serial.println("No audio files found on SD card!");
        Serial.println("Make sure you have audio files named 0001.mp3, 0002.mp3, etc. on the SD card");
    }

    Serial.println("=== Test Complete ===");
}
