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

