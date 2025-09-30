#include "TimeMapper.h"

// Initialize singleton instance pointer
TimeMapper* TimeMapper::_instance = nullptr;

TimeMapper& TimeMapper::getInstance() {
    if (_instance == nullptr) {
        _instance = new TimeMapper();
    }
    return *_instance;
}

// Static convenience methods
bool TimeMapper::isReady() {
    return getInstance().isReadyInstance();
}

void TimeMapper::update() {
    getInstance().updateInstance();
}

uint64_t TimeMapper::hardwareToNTP(uint64_t hardwareMicros) {
    return getInstance().hardwareToNTPInstance(hardwareMicros);
}

uint64_t TimeMapper::ntpToHardware(uint64_t ntpMicros) {
    return getInstance().ntpToHardwareInstance(ntpMicros);
}

uint64_t TimeMapper::sampleToNTP(uint32_t t_us, uint32_t rollover_count) {
    // Compose 64-bit hardware timestamp from Sample fields
    uint64_t hardwareTime = ((uint64_t)rollover_count << 32) | t_us;
    return hardwareToNTP(hardwareTime);
}

void TimeMapper::ntpToSample(uint64_t ntpMicros, uint32_t& t_us, uint32_t& rollover_count) {
    // Convert NTP time to hardware time
    uint64_t hardwareTime = ntpToHardware(ntpMicros);
    
    // Split into Sample fields
    t_us = (uint32_t)(hardwareTime & 0xFFFFFFFFULL);
    rollover_count = (uint32_t)(hardwareTime >> 32);
}

bool TimeMapper::begin() {
    if (_initialized) {
        return true;
    }
    
    Serial.println("[TimeMapper] Initializing...");
    
    // Check if HardwareTimer is ready
    if (!HardwareTimer::isInitialized()) {
        Serial.println("[TimeMapper] ERROR: HardwareTimer not initialized");
        return false;
    }
    
    // Check if NTP client is available
    if (!NTPClient::hasSynced()) {
        Serial.println("[TimeMapper] WARNING: NTP not yet synced, will sync on first update");
    }
    
    _initialized = true;
    _lastAutoSyncMillis = 0;
    
    return true;
}


uint64_t TimeMapper::hardwareToNTPInstance(uint64_t hardwareMicros) const {
    if (!NTPClient::hasSynced()) {
        Serial.println("[TimeMapper] WARNING: NTP Client not synced");
        return 0;
    }
    
    // Calculate the time difference in hardware time
    int64_t hardwareDelta = (int64_t)(hardwareMicros - NTPClient::localMicrosAtSync());
    
    // Apply the same delta to NTP time
    // This assumes the clocks run at the same rate (which they should over short periods)
    uint64_t ntpTime = NTPClient::serverMicrosAtSync() + hardwareDelta;
    
    return ntpTime;
}

uint64_t TimeMapper::ntpToHardwareInstance(uint64_t ntpMicros) const {
    if (!NTPClient::hasSynced()) {
        Serial.println("[TimeMapper] WARNING: NTP Client not synced");
        return 0;
    }

    // Calculate the time difference in NTP time
    int64_t ntpDelta = (int64_t)(ntpMicros - NTPClient::serverMicrosAtSync());
    
    // Apply the same delta to hardware time
    uint64_t hardwareTime = NTPClient::localMicrosAtSync() + ntpDelta;
    
    return hardwareTime;
}

bool TimeMapper::isReadyInstance() const {
    return _initialized && NTPClient::hasSynced();
}

void TimeMapper::updateInstance() {
    if (!_initialized) {
        return;
    }
    // Check if we need to auto-sync
    uint32_t currentMillis = millis();
    if (currentMillis - _lastAutoSyncMillis >= AUTO_SYNC_INTERVAL_MS) {
        Serial.println("[TimeMapper] Auto-sync triggered");
        NTPClient::requestTime();
        _lastAutoSyncMillis = currentMillis;
    }
}