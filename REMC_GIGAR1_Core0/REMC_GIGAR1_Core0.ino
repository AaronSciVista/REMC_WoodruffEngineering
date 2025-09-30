#include <Arduino.h>
#include <RPC.h>
#include "SharedRing.h"
#include "UdpManager.h"
#include "SampleCollector.h"
#include "StateManager.h"
#include "ActuatorManager.h"
#include "SDRAM.h"
#include "HardwareTimer.h"
#include "NTPClient.h"
#include "TimeMapper.h"
#include "Config.h"

void setup() { 
  Serial.begin(115200);
  while (!Serial) { }    // wait (native-USB boards)
  Serial.println(F("\nREMC Switch Control Initializing (Dual Core)..."));

  // Initialize SDRAM hardware - important for storing 8MB worth of accumulated samples
  if (!SDRAM.begin()) {
    Serial.println("[Serial Core] ERROR: SDRAM init failed!");
    while (1);
  }
  // Initialize Sample Collector (includes SharedRing setup)
  if (!SampleCollector::init()) {
    Serial.println("[Serial Core] ERROR: SampleCollector init failed!");
    while(1); // Halt on initialization failure
  }

  // Initialize Actuator Manager
  Serial.println(F("[Serial Core] ActuatorManager init"));
  ActuatorManager::init();
  
  // Initialize State Manager
  Serial.println(F("[Serial Core] StateManager init"));
  StateManager::init();

  // Initialize UDP Manager
  Serial.println(F("[Serial Core] UdpManager init"));
  UdpManager::init();

  // Initialize the shared timer (only call from M7)
  if (HardwareTimer::begin()) {
      Serial.println("[Serial Core] Hardware timer initialized successfully");
  } else {
      Serial.println("[Serial Core] Failed to initialize hardware timer");
  }

  NTPClient::initialize(UdpManager::getNTPUdpObject());
  NTPClient::begin(Config::NTP_SERVER, Config::NTP_CLIENT_PORT);
  Serial.println("[Serial Core] NTP initialized.");

  // Initialize TimeMapper after both HardwareTimer and NTP are ready
  Serial.println("[Serial Core] TimeMapper beginning.");
  if (TimeMapper::getInstance().begin()) {
    Serial.println("[Serial Core] TimeMapper initialized successfully");
  } else {
    Serial.println("[Serial Core] TimeMapper initialization failed");
  }
  
  // Starts Sampling Core (1)
  RPC.begin();
  Serial.println(F("[Serial Core] Ready - call startGathering() to begin"));

}

void loop() {
  // Print RPC messages from M4 Core
  DEBUG_printRPCMessages();
  
  // Update State Manager (actuator control FSM)
  StateManager::update();
  
  // Process samples (main sample collection logic)
  SampleCollector::update();
  
  // Handle incoming UDP commands
  UdpManager::update();

  NTPClient::update();

  // Update TimeMapper (handles automatic NTP re-sync every 10 seconds)
  TimeMapper::update();

}

// ===== DEBUG FUNCTIONS FROM M4 CORE =====
void DEBUG_printRPCMessages() {
  // Pump any RPC messages into Serial
  String buffer = "";
  while (RPC.available()) {
    buffer += (char)RPC.read();
  }
  if (buffer.length() > 0) {
    Serial.print(buffer);
  }
}