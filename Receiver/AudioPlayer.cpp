#include "AudioPlayer.h"

AudioPlayer::AudioPlayer(int dfRx, int dfTx) {
    rxPin = dfRx;
    txPin = dfTx;
    serial = new HardwareSerial(1);
    dfPlayer = new DFRobotDFPlayerMini();
    isInitialized = false;
}

AudioPlayer::~AudioPlayer() {
    delete dfPlayer;
    delete serial;
}

bool AudioPlayer::begin() {
    serial->begin(9600, SERIAL_8N1, rxPin, txPin);

    // Give the DFPlayer time to initialize
    delay(1000);

    Serial.println("Starting AudioPlayer...");
    if (!dfPlayer->begin(*serial)) {
        Serial.println("AudioPlayer could not be started.");
        Serial.println("Check:");
        Serial.println("1. Wiring connections");
        Serial.println("2. SD card is inserted");
        Serial.println("3. Audio files exist on SD card");
        return false;
    }

    Serial.println("AudioPlayer successfully started!");
    delay(200);

    // Set default volume
    setVolume(25);

    isInitialized = true;
    return true;
}

void AudioPlayer::setVolume(int volume) {
    if (isInitialized) {
        dfPlayer->volume(volume);
        Serial.print("Volume set to: ");
        Serial.println(volume);
    }
}

int AudioPlayer::getFileCount() {
    if (isInitialized) {
        delay(100);
        int fileCount = dfPlayer->readFileCounts();
        Serial.print("Files found on SD card: ");
        Serial.println(fileCount);
        return fileCount;
    }
    return 0;
}


void AudioPlayer::playStartingSound() {
    if (isInitialized) {
        Serial.println("Playing starting sound...");
        dfPlayer->play(1); // 0001.mp3 - starting sound
    }
}

void AudioPlayer::playSirenSound() {
    if (isInitialized) {
        Serial.println("Playing siren sound...");
        dfPlayer->play(2); // 0002.mp3 - siren sound
    }
}

void AudioPlayer::playDriftingSound() {
    if (isInitialized) {
        Serial.println("Playing drifting sound...");
        dfPlayer->play(3); // 0003.mp3 - drifting sound
    }
}

void AudioPlayer::stop() {
    if (isInitialized) {
        Serial.println("Stopping audio playback...");
        dfPlayer->stop();
    }
}

void AudioPlayer::playFile(int fileNumber) {
    if (isInitialized) {
        Serial.print("Playing file: ");
        Serial.println(fileNumber);
        dfPlayer->play(fileNumber);
    }
}

void AudioPlayer::runTest() {
    if (!isInitialized) {
        Serial.println("AudioPlayer not initialized!");
        return;
    }

    Serial.println("=== AudioPlayer Test ===");

    // Test volume setting
    Serial.println("Setting volume to 30...");
    setVolume(30);
    delay(100);

    // Test file count
    int fileCount = getFileCount();

    if (fileCount > 0) {
        Serial.println("Testing starting sound...");
        playStartingSound();
        delay(3000);

        if (fileCount >= 2) {
            Serial.println("Testing siren sound...");
            playSirenSound();
            delay(3000);

            if (fileCount >= 3) {
                Serial.println("Testing drifting sound...");
                playDriftingSound();
                delay(3000);
            }
        }
        stop();
    } else {
        Serial.println("No audio files found on SD card!");
        Serial.println("Make sure you have audio files named 0001.mp3, 0002.mp3, etc. on the SD card");
    }

    Serial.println("=== Test Complete ===");
}

bool AudioPlayer::isAvailable() {
    if (isInitialized) {
        return dfPlayer->available();
    }
    return false;
}
