#include "NTPClient.h"
#include "w5100mod.h"

static const uint32_t NTP_UNIX_EPOCH_DIFF = 2208988800UL; // seconds between 1900 and 1970
static const int NTP_PACKET_SIZE = 48;

// Initialize singleton instance pointer
NTPClient* NTPClient::_instance = nullptr;

NTPClient::NTPClient(EthernetUDP* udp) {
  if (udp) {
    _udp = udp;
  } else {
    _udp = &_ownedUdp;
  }
}

// Singleton implementation
NTPClient& NTPClient::getInstance() {
  return *_instance;
}

// Static initialization method
void NTPClient::initialize(EthernetUDP* udp) {
  // Create singleton instance if it doesn't exist
  if (_instance == nullptr) {
    _instance = new NTPClient(udp);
  }
}







// Static wrapper methods
uint64_t NTPClient::serverMicrosAtSync() {
  return getInstance().serverMicrosAtSyncInstance();
}
uint64_t NTPClient::localMicrosAtSync() {
  return getInstance().localMicrosAtSyncInstance();
}
bool NTPClient::hasSynced() {
  return getInstance().hasSyncedInstance();
}

bool NTPClient::requestTime() {
  return getInstance().requestTimeInstance();
}

bool NTPClient::begin(const char* server, uint16_t ntpPort) {
  return getInstance().beginInstance(server, ntpPort);
}

void NTPClient::update() {
  getInstance().updateInstance();
}
// -------------







// ISR Utility methods
void clearSIRs() { // After a socket IR, SnIR and SIR need to be reset
  // for (int i=0;i<8;i++) {
  //   W5100.writeSnIR(i,0xFF); // Clear socket i interrupt
  // }
  // W5100.writeSIR(0xFF); // Clear SIR
  W5100.writeSnIR(2,0xFF); // Clear socket i interrupt
  W5100.writeSIR(0x04); // Clear SIR
}

// disable interrupts for all sockets
inline void disableSIRs() {W5100.writeSIMR(0x00);}

// enable interrupts for socket 2
inline void enableSIRs() {W5100.writeSIMR(0x04);}

// Interrupt service routine
uint32_t requestsSent = 0;
uint32_t responsesReceived = 0;
uint64_t lastResponseTime = 0;
uint32_t responsesProcessed = 0;

void socketISR()
{
  uint64_t now = HardwareTimer::getMicros64();
  lastResponseTime = now;
  responsesReceived++;
}
// -------------


bool NTPClient::beginInstance(const char* server, uint16_t ntpPort) {
  _serverPort = ntpPort;
  
  Serial.print("[NTP] Resolving server: ");
  Serial.println(server);
  _serverResolved = resolveServerIP(server);
  
  if (_serverResolved) {
    Serial.print("[NTP] Server resolved to: ");
    Serial.println(_serverIP);
  } else {
    Serial.println("[NTP] ERROR: Failed to resolve server address");
  }
  
  // This is the interrupt pin for the W5500 shield
  pinMode(2, INPUT);
  for (int i = 0; i < 8; i++) {
    W5100.writeSnIMR(i,0x04);  // Socket IR mask: RECV for all sockets
  }
  enableSIRs();
  attachInterrupt(digitalPinToInterrupt(2), socketISR, FALLING);
  _interruptEnabled = true;

  return _serverResolved && _interruptEnabled;
}


bool NTPClient::resolveServerIP(const char* server) {
  // Try dotted-quad first
  IPAddress ip;
  if (ip.fromString(server)) {
    Serial.println("[NTP] Server is IP address - no DNS resolution needed");
    _serverIP = ip;
    return true;
  }
  
  Serial.println("[NTP] Server is hostname - attempting DNS resolution");
  
  // Fallback to DNS for hostnames
  DNSClient dns;
  IPAddress dnsServer = Ethernet.dnsServerIP();
  Serial.print("[NTP] DNS server from DHCP: ");
  Serial.println(dnsServer);
  
  if (dnsServer == IPAddress(0,0,0,0)) {
    // Try gateway as DNS if none configured
    dnsServer = Ethernet.gatewayIP();
    Serial.print("[NTP] Using gateway as DNS server: ");
    Serial.println(dnsServer);
  }
  
  if (dnsServer == IPAddress(0,0,0,0)) {
    Serial.println("[NTP] ERROR: No DNS server available");
    return false;
  }
  
  dns.begin(dnsServer);
  Serial.print("[NTP] Attempting DNS lookup for: ");
  Serial.println(server);

  int res = dns.getHostByName(server, _serverIP);
  if (res == 1) {
    Serial.print("[NTP] DNS resolution successful: ");
    Serial.println(_serverIP);
  } else {
    Serial.print("[NTP] DNS resolution failed with code: ");
    Serial.println(res);
  }
  
  return (res == 1);
}

bool NTPClient::sendRequest() {
  if (!_serverResolved) {
    Serial.println("[NTP] ERROR: Cannot send request - server not resolved");
    return false;
  }

  //Serial.print("[NTP] Sending NTP request to ");
  //Serial.print(_serverIP);
  //Serial.print(":");
  //Serial.println(_serverPort);

  uint8_t packet[NTP_PACKET_SIZE];
  memset(packet, 0, sizeof(packet));

  // LI = 0 (no warning), VN = 4, Mode = 3 (client)
  packet[0] = 0x23; // 0b0010_0011

  // Transmit Timestamp can be left 0; server will fill its own transmit time.

  if (_udp->beginPacket(_serverIP, _serverPort) != 1) {
    Serial.println("[NTP] ERROR: Failed to begin UDP packet");
    return false;
  }
  
  size_t written = _udp->write(packet, sizeof(packet));
  if (written != sizeof(packet)) {
    Serial.print("[NTP] ERROR: Incomplete packet write - wrote ");
    Serial.print(written);
    Serial.print(" of ");
    Serial.println(sizeof(packet));
    return false;
  }
  
  if (_udp->endPacket() != 1) {
    Serial.println("[NTP] ERROR: Failed to send UDP packet");
    return false;
  }
  
  //Serial.println("[NTP] NTP request sent successfully");
  return true;
}

bool NTPClient::readResponse(uint64_t& microseconds) {
  microseconds = 0;
  int size = _udp->parsePacket();
  if (size == 0) return false;  // No packet available
  
  if (size < NTP_PACKET_SIZE) {
    Serial.print("[NTP] WARNING: Received packet too small: ");
    Serial.print(size);
    Serial.print(" bytes (expected ");
    Serial.print(NTP_PACKET_SIZE);
    Serial.println(")");
    return false;
  }

  //Serial.print("[NTP] Received NTP response (");
  //Serial.print(size);
  //Serial.println(" bytes)");

  uint8_t buf[NTP_PACKET_SIZE];
  int bytesRead = _udp->read(buf, NTP_PACKET_SIZE);
  
  if (bytesRead != NTP_PACKET_SIZE) {
    Serial.print("[NTP] ERROR: Read incomplete - got ");
    Serial.print(bytesRead);
    Serial.print(" of ");
    Serial.println(NTP_PACKET_SIZE);
    return false;
  }

  // Check if it's a valid NTP response
  uint8_t mode = buf[0] & 0x07;
  if (mode != 4) {  // Server mode
    Serial.print("[NTP] ERROR: Invalid mode in response: ");
    Serial.println(mode);
    return false;
  }

  // Transmit Timestamp starts at byte 40 (big-endian): seconds (4), fraction (4)
  uint32_t secs = ((uint32_t)buf[40] << 24) |
                  ((uint32_t)buf[41] << 16) |
                  ((uint32_t)buf[42] << 8)  |
                  ((uint32_t)buf[43]);

  uint32_t frac = ((uint32_t)buf[44] << 24) |
                  ((uint32_t)buf[45] << 16) |
                  ((uint32_t)buf[46] << 8)  |
                  ((uint32_t)buf[47]);

  // Basic sanity: NTP time should be after Jan 1, 2000 (946684800 Unix)
  uint32_t unixSecs = secs - NTP_UNIX_EPOCH_DIFF;
  if (unixSecs < 946684800UL) {
    Serial.print("[NTP] ERROR: Timestamp sanity check failed - Unix seconds: ");
    Serial.println(unixSecs);
    return false;
  }

  // Convert to microseconds
  microseconds = ((uint64_t)unixSecs * 1000000ULL) + ntpFracToMicros(frac);
  return true;
}


bool NTPClient::requestTimeInstance() {
  if (!_serverResolved) {
    Serial.println("[NTP] ERROR: Cannot sync - server not resolved");
    return false;
  }

  // Flush any stale packets
  int flushed = 0;
  while (_udp->parsePacket() > 0) {
    uint8_t dump[64];
    _udp->read(dump, sizeof(dump));
    flushed++;
  }
  if (flushed > 0) {
    //Serial.print("[NTP] Flushed ");
    //Serial.print(flushed);
    //Serial.println(" stale packets");
  }

  if (!sendRequest()) {
    Serial.println("[NTP] ERROR: Failed to send request");
    return false;
  }
  return true;
}

void NTPClient::updateInstance() {
  if (responsesProcessed < responsesReceived) {
    uint64_t now = HardwareTimer::getMicros64();
    uint64_t serverMicroseconds = 0;
    if (readResponse(serverMicroseconds)) {
      responsesProcessed++;
      Serial.print("[NTP] Processing response, ");
      Serial.print("Responses received: ");
      Serial.print(responsesReceived);
      Serial.print(", Responses processed: ");
      Serial.println(responsesProcessed);

      _serverMicrosAtSync = serverMicroseconds;
      _localMicrosAtSync = now;
      _synced = true;
    }

    clearSIRs();
    enableSIRs();
  }
}


uint64_t NTPClient::serverMicrosAtSyncInstance() {
  if (!_synced) return 0ULL;

  uint64_t nowLocal = HardwareTimer::getMicros64();
  uint64_t elapsed = nowLocal - _localMicrosAtSync;
  return _serverMicrosAtSync + elapsed;
}

uint64_t NTPClient::localMicrosAtSyncInstance() {
  if (!_synced) return 0ULL;
  return _localMicrosAtSync;
}

