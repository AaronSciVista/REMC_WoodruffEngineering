/*
  ---------------------------------------------------------------------------
  TimeMapper – NTP to HardwareTimer Time Mapping System
  ---------------------------------------------------------------------------

  This class maintains the mapping between NTP time (Unix microseconds) and
  HardwareTimer micros64 time. It provides bidirectional conversion methods
  and handles NTP re-synchronization.

  Key Features:
    - Convert HardwareTimer micros64 timestamps to NTP Unix time
    - Convert NTP Unix time to HardwareTimer micros64 timestamps
    - Automatic NTP re-synchronization management
    - Thread-safe singleton pattern for global access
    - Handles time drift compensation between sync periods

  Usage:
    TimeMapper::getInstance().begin();
    
    // Convert hardware timestamp to NTP time
    uint64_t ntpTime = TimeMapper::hardwareToNTP(hardwareTimestamp);
    
    // Convert NTP time to hardware timestamp
    uint64_t hwTime = TimeMapper::ntpToHardware(ntpTimestamp);
    
    // Force NTP sync
    TimeMapper::getInstance().syncNTP();

  The class automatically handles the offset calculation between the two
  time systems and maintains accuracy across NTP re-synchronization events.
  ---------------------------------------------------------------------------
*/

#pragma once
#include <Arduino.h>
#include "HardwareTimer.h"
#include "NTPClient.h"

class TimeMapper {
public:
    // Singleton access
    static TimeMapper& getInstance();
    
    // Static convenience methods for global access
    bool begin();
    static bool isReady();
    static void update();

    static uint64_t hardwareToNTP(uint64_t hardwareMicros);
    static uint64_t ntpToHardware(uint64_t ntpMicros);
    static uint64_t sampleToNTP(uint32_t t_us, uint32_t rollover_count);
    static void ntpToSample(uint64_t ntpMicros, uint32_t& t_us, uint32_t& rollover_count);
    
private:
    TimeMapper() = default;
    ~TimeMapper() = default;
    TimeMapper(const TimeMapper&) = delete;
    TimeMapper& operator=(const TimeMapper&) = delete;
    
    bool isReadyInstance() const;
    void updateInstance();
    bool syncNTPInstance();
    uint64_t hardwareToNTPInstance(uint64_t hardwareMicros) const;
    uint64_t ntpToHardwareInstance(uint64_t ntpMicros) const;



    // Singleton instance
    static TimeMapper* _instance;
    
    // Mapping state
    bool _initialized = false;

    // Auto-sync configuration
    static const uint32_t AUTO_SYNC_INTERVAL_MS = 10000; // 10 seconds
    uint32_t _lastAutoSyncMillis = 0;
};
