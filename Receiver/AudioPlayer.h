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
    int getFileCount();

    // Specific sound methods
    void playStartingSound();
    void playSirenSound();
    void playDriftingSound();
    void stop();

    // Generic play method
    void playFile(int fileNumber);

    // Test functionality
    void runTest();

    // Check if player is available for commands
    bool isAvailable();
};

#endif
