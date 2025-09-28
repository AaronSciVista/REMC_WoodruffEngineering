#ifndef SAMPLE_COLLECTOR_H
#define SAMPLE_COLLECTOR_H

#include <Arduino.h>
#include <stddef.h>
#include "SharedRing.h"

class SampleCollector {
public:
    // Initialize the sample collector (call once in setup)
    static bool init(size_t storageCapacity = 250000);
    
    // Main processing function (call in main loop)
    static void update();
    
    // Control functions
    static void startGathering(int start, int stop);
    static void startGathering(); // Parameterless version using stored window
    static void stopGathering();
    static void sendAllSamples();
    
    // Window configuration functions
    static void setWindow(int start, int stop);
    
    // Status functions
    static size_t getSamplesStored();
    static bool isGathering();
    static size_t getStorageCapacity();
    
    // Debug functions
    static void printSampleDiagnostics(size_t count);
    
private:
    // Ring buffer storage system
    static Sample* ringBuffer;
    static volatile size_t ringCapacity;
    static volatile size_t ringHead;
    static volatile size_t totalSamplesReceived;
    
    // Dense gathering state
    static volatile bool denseGatheringActive;
    static volatile int gatheringStart;
    static volatile int gatheringStop;
    static volatile size_t samplesNeeded;
    static volatile size_t samplesCollected;
    static volatile size_t gatheringStartSampleCount;
    
    // Live feed state
    static const int liveFeedRatio = 10000;  // Send every 1/N samples as live feed
    static volatile size_t liveFeedCounter;
    
    // Window storage
    static volatile int windowStart;
    static volatile int windowStop;
    
    // Processing buffer
    static constexpr int MAX_FETCH = 1024;
    static Sample sampleBuffer[MAX_FETCH];
    
    // Rolling buffer for diagnostics
    static constexpr int WINDOW = 20;
    static Sample ringBuf[WINDOW];
    static int ringCount;
    static int ringIndex;
    
    // Helper functions
    static void storeSampleInRing(const Sample& sample);
    static void extractRequestedSamples();
    static size_t getRingIndex(int relativeIndex, size_t referenceSampleCount);
    static bool canSendNow();
};

#endif // SAMPLE_COLLECTOR_H
