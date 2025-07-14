#ifndef AUDIOPLAYER_H
#define AUDIOPLAYER_H

#include "DFRobotDFPlayerMini.h"
#include <HardwareSerial.h>

class AudioPlayer {
private:
    DFRobotDFPlayerMini* dfPlayer;
    HardwareSerial* serial;
    int rxPin;
    int txPin;
    bool isInitialized;

public:
    AudioPlayer(int dfRx, int dfTx);
    ~AudioPlayer();

    bool begin();
    void setVolume(int volume);

    // Specific sound methods
    void playStartingSound();
    void playSirenSound();
    void playDriftingSound();

    // Generic play method
    void playFile(int fileNumber);
};

#endif
