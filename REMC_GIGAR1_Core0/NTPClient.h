#pragma once
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Dns.h>
#include "HardwareTimer.h"






class NTPClient {
public:
  // Singleton access
  static NTPClient& getInstance();
  
  // Static initialization method for setting UDP object
  static void initialize(EthernetUDP* udp);
  static bool begin(const char* server, uint16_t ntpPort = 123);
  static bool requestTime();
  static void update();
  static bool hasSynced();
  static uint64_t serverMicrosAtSync();
  static uint64_t serverMicrosNow();
  static uint64_t localMicrosAtSync();
  static void printMetricsStatistics();

  // You may pass a shared UDP instance if you want; otherwise default-construct one.
  explicit NTPClient(EthernetUDP* udp);

private:

  bool beginInstance(const char* server, uint16_t ntpPort = 123);
  bool requestTimeInstance();
  void updateInstance();

  bool hasSyncedInstance() const { return _synced; }
  uint64_t serverMicrosAtSyncInstance();
  uint64_t serverMicrosNowInstance();
  uint64_t localMicrosAtSyncInstance();

  // Was there at least one successful sync?

  bool resolveServerIP(const char* server);
  bool sendRequest();
  bool readResponse(uint64_t& microseconds);

  static uint64_t ntpFracToMicros(uint32_t frac) {
    // Convert 32-bit NTP fractional seconds to microseconds: frac * 1e6 / 2^32
    return ((uint64_t)frac * 1000000ULL) >> 32;
  }

  // Singleton instance
  static NTPClient* _instance;

  EthernetUDP _ownedUdp;        // used if user didn't supply one
  EthernetUDP* _udp = nullptr;  // active UDP instance

  IPAddress _serverIP;
  uint16_t _serverPort = 123;
  uint16_t _localPort = 0;

  bool _serverResolved = false;
  bool _interruptEnabled = false;

  // Sync anchors
  bool _synced = false;
  uint64_t _serverMicrosAtSync = 0;
  uint64_t _localMicrosAtSync = 0; 

  // Metrics collection
  struct NTPMetrics {
    uint64_t roundTripTime;           // Actual round trip time using interrupt timestamp
    uint64_t responseNowDifference;   // Difference between response received and 'now'
    uint64_t inaccurateRoundTripTime; // Round trip time if using 'now' instead of interrupt timestamp
  };

  static const size_t METRICS_BUFFER_SIZE = 1000;
  NTPMetrics _metricsBuffer[METRICS_BUFFER_SIZE];
  size_t _metricsIndex = 0;
  size_t _metricsCount = 0;

  // Statistics calculation methods
  void addMetric(const NTPMetrics& metric);
  void calculateStatistics(uint64_t& mean, uint64_t& min, uint64_t& max, double& stdev, 
                          const uint64_t* data, size_t count) const;
  void printMetricsStatisticsInstance() const;
};
