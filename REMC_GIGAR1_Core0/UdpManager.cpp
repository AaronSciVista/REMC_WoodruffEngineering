#include "UdpManager.h"
#include "SharedRing.h"  // For Sample struct definition only
#include "Config.h"
#include "StateManager.h"
#include "PinConfig.h"
#include "SampleCollector.h"
#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include "MD5.h"      // For schema hashing
#include <TimeLib.h>  // For timekeeping (needs external time source)
#include "TimeMapper.h"

// --- Network Configuration ---
static EthernetUDP cmdUdp;  // Multicast listener for commands
static EthernetUDP udp;
static EthernetUDP ntpUdp; // for NTP sync

static const IPAddress PC_MCAST = Config::TELEMETRY_IP;  // Destination IP for telemetry
static const uint16_t UDP_PORT = Config::TELEMETRY_PORT;

static const IPAddress CMD_MCAST = Config::COMMAND_MCAST_IP;
static const uint16_t CMD_PORT = Config::COMMAND_PORT;

static const uint16_t NTP_PORT = Config::NTP_PORT;

// Neutrino header constants
static const uint32_t MSG_ID = 1;  // Atomic
static const uint32_t FLAGS_NORMAL = 0;
static const uint32_t FLAGS_COLLECTED_SAMPLES = 1;  // Tag for collected samples
static const uint32_t FLAGS_BATCH_END = 2;  // Tag for end of batch
static const size_t FRAG_LEN = 16;
static const size_t HEADER_SIZE = 64;

// Telemetry schema - samples are bundled per loop iteration
static const char* schema =
  "node_name REMC \n"
  "c telem_period 100000\n"  // 100µs (in nanoseconds)
  "v switch_voltage f32 u:kV\n"
  "v switch_current f32 u:kA\n"
  "v output_voltage_a f32 u:kV\n"
  "v output_voltage_b f32 u:kV\n"
  "v temperature_1 f32 u:degC\n"
  "v armed_status u8\n"
  "v em_status u8\n"
  "v msw_a_status u8\n"
  "v msw_b_status u8\n"
  "v manual_mode_status u8\n"
  "v hold_mode_status u8\n"

  "\n\n\n\n\n\n\n\n\n\n";  // Pad to multiple of 16 bytes

// Dynamic bundling configuration - optimized for MTU
// Ethernet MTU=1500, IP=20, UDP=8 → max payload=1472
// Header=64, remaining=1408, sample=42 → max samples=33 (safe)
constexpr size_t MAX_SAMPLES_PER_BUNDLE = 33;  // Maximum samples per UDP packet (MTU optimized for 42-byte samples)

// Data sizes - variable samples per packet  
static const size_t DATA_SIZE_PER_SAMPLE = (5 * sizeof(float)) + sizeof(uint64_t) + 6 * sizeof(uint8_t) + sizeof(uint64_t);  // 42 bytes per sample (includes us_end)
static const size_t MAX_PACKET_SIZE = HEADER_SIZE + (DATA_SIZE_PER_SAMPLE * MAX_SAMPLES_PER_BUNDLE);
// Packet size check: 64 + (42 * 33) = 1450 bytes < 1472 MTU limit ✓

struct TelemetrySample {
  float sv, sc, ova, ovb, tm1;
  uint64_t us;  // NTP timestamp in microseconds (from TimeMapper)
  uint8_t ready, em, a, b, manual, hold;
  uint64_t us_end;  // NTP timestamp in microseconds (from TimeMapper)
};

// ADC conversion constants (copied from old TimerManager)
const float ADC_MAX_VALUE = 4095.0f;

// Switch current calibration:
const float SCALE_SWITCH_CURRENT_A = 1000.0f / ADC_MAX_VALUE;  // 1 count ≈ 0.244 A
const float OFFSET_SWITCH_CURRENT_A = -471.551f;               // raw = 0 → –500 A

// Switch voltage calibration:
const float SCALE_VOLTAGE_KV = 0.004449458233f;  // kV = 1 count
const float OFFSET_VOLTAGE_KV = -8.939881545f;   // raw = 0 → –8.94 kV

// Output A voltage calibration (kV):
const float SCALE_OUTPUT_A_KV = 0.004447667531f;  // kV = 1 count
const float OFFSET_OUTPUT_A_KV = -8.941615805f;   // raw = 0 → –8.94 kV

// Output B voltage calibration (kV):
const float SCALE_OUTPUT_B_KV = 0.004445948727f;  // kV = 1 count
const float OFFSET_OUTPUT_B_KV = -8.936364074f;   // raw = 0 → –8.94 kV

// Temperature calibration:
const float SCALE_TEMP_DEGC = 100.0f / ADC_MAX_VALUE;  // 1 count ≈ 0.0244 °C
const float OFFSET_TEMP_DEGC = -5.5f;                  // raw = 0 → 0 °C

namespace {
uint8_t schemaHash[16];
uint32_t schemaNumFrags = 0;
uint32_t currentSchemaFrag = 0;

// Dynamic bundling - collect samples from current loop iteration
static TelemetrySample s_sample_bundle[MAX_SAMPLES_PER_BUNDLE];
static size_t s_bundle_count = 0;
static bool s_sending_collected_samples = false;

void calcSchemaHash() {
  char* buf = (char*)malloc(strlen(schema) + 1);
  if (buf != NULL) {
    strcpy(buf, schema);
    unsigned char* digest = MD5::make_hash(buf);
    free(buf);
    if (digest) {
      memcpy(schemaHash, digest, 16);
      free(digest);
    } else {
      Serial.println(F("[UDP][ERR] Schema hash error"));
      memset(schemaHash, 0, 16);
    }
  } else {
    Serial.println(F("[UDP][ERR] Memory allocation error"));
    memset(schemaHash, 0, 16);
  }
  schemaNumFrags = (strlen(schema) + FRAG_LEN - 1) / FRAG_LEN;
  currentSchemaFrag = 0;
}

uint32_t htonl_custom(uint32_t h) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return __builtin_bswap32(h);
#else
  return h;
#endif
}

uint64_t htobe64_custom(uint64_t v) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return ((uint64_t)htonl_custom(v & 0xffffffff) << 32) | htonl_custom(v >> 32);
#else
  return v;
#endif
}


uint64_t getUnixTimeNanos() {
  // Get integer seconds since epoch
  time_t epochSecs = now();
  // Get microseconds since boot; take only the fractional part within the current second
  unsigned long us = micros() % 1000000UL;
  // Combine into nanoseconds
  return (uint64_t)epochSecs * 1000000000ULL + (uint64_t)us * 1000ULL;
}

}

namespace UdpManager {

// Forward declarations
void sendNeutrinoPacket();

void init() {
  Serial.println(F("UdpManager: Ethernet.begin..."));
  Ethernet.begin((byte*)Config::MAC_ADDRESS,
                 Config::LOCAL_IP,
                 Config::GATEWAY_IP,
                 Config::GATEWAY_IP,
                 Config::SUBNET_MASK);

  Serial.print(F("UdpManager: Binding UDP on port "));
  Serial.println(UDP_PORT);
  if (udp.begin(UDP_PORT) != 1) {
    Serial.println(F("UdpManager: UDP bind failed"));
  }

  // --- JOIN MULTICAST GROUP FOR TELEMETRY ---
  Serial.println(F("Joining telemtry multicast "));
  if (udp.beginMulticast(PC_MCAST, UDP_PORT) != 1) {
    Serial.println(F("UdpManager: Telemetry multicast join failed"));
  } else {
    Serial.print(F("UdpManager: Joined telemetry multicast "));
    Serial.print(PC_MCAST);
    Serial.print(F(":"));
    Serial.println(UDP_PORT);
  }

  // --- JOIN MULTICAST GROUP FOR COMMAND ---
  Serial.println(F("Joining command multicast "));
  if (cmdUdp.beginMulticast(CMD_MCAST, CMD_PORT) != 1) {
    Serial.println(F("UdpManager: Command multicast join failed"));
  } else {
    Serial.print(F("UdpManager: Joined command multicast "));
    Serial.print(CMD_MCAST);
    Serial.print(F(":"));
    Serial.println(CMD_PORT);
  }

  // NTP port
  Serial.print(F("UdpManager: Binding NTP on port "));
  Serial.println(NTP_PORT);
  if (ntpUdp.begin(NTP_PORT) != 1) {
    Serial.println(F("UdpManager: NTP bind failed"));
  }

  calcSchemaHash();

  Serial.println(F("[UDP] init complete."));
}

// Convert raw ADC values to physical units
float convertSwitchCurrentA(uint16_t raw) {
  return raw * SCALE_SWITCH_CURRENT_A + OFFSET_SWITCH_CURRENT_A;
}

float convertSwitchVoltageKV(uint16_t raw) {
  return raw * SCALE_VOLTAGE_KV + OFFSET_VOLTAGE_KV;
}

float convertOutputVoltageAKV(uint16_t raw) {
  return raw * SCALE_OUTPUT_A_KV + OFFSET_OUTPUT_A_KV;
}

float convertOutputVoltageBKV(uint16_t raw) {
  return raw * SCALE_OUTPUT_B_KV + OFFSET_OUTPUT_B_KV;
}

float convertTemp1DegC(uint16_t raw) {
  return raw * SCALE_TEMP_DEGC + OFFSET_TEMP_DEGC;
}

bool addSample(const Sample& sample) {
  // Check if bundle is full
  if (s_bundle_count >= MAX_SAMPLES_PER_BUNDLE) {
    // Bundle full - send current bundle and start new one
    flushSamples();
  }
  
  // Convert raw ADC values to physical units and add to bundle
  TelemetrySample& ts = s_sample_bundle[s_bundle_count];
  
  ts.sv = convertSwitchVoltageKV(sample.swV);
  ts.sc = convertSwitchCurrentA(sample.swI);
  ts.ova = convertOutputVoltageAKV(sample.outA);
  ts.ovb = convertOutputVoltageBKV(sample.outB);
  ts.tm1 = convertTemp1DegC(sample.t1);
  ts.us = TimeMapper::sampleToNTP(sample.t_us, sample.rollover_count);
  ts.us_end = TimeMapper::sampleToNTP(sample.t_us_end, sample.rollover_count_end);
  // Get these from state manager 
  ts.ready = StateManager::isReady() ? 1 : 0;
  ts.em = StateManager::isEmActActive() ? 1 : 0;
  ts.a = (digitalRead(PIN_MSW_POS_A) == LOW) ? 0 : 1;
  ts.b = (digitalRead(PIN_MSW_POS_B) == LOW) ? 0 : 1;
  ts.manual = StateManager::isManualModeActive() ? 1 : 0;
  ts.hold = StateManager::isHoldAfterFireModeActive() ? 1 : 0;
  
  s_bundle_count++;
  return true;
}

void flushSamples() {
  if (s_bundle_count == 0) return;  // Nothing to send
  
  sendNeutrinoPacket();
  s_bundle_count = 0;  // Reset bundle for next iteration
}

size_t getBufferUsage() {
  return s_bundle_count;
}

size_t getBufferCapacity() {
  return MAX_SAMPLES_PER_BUNDLE;
}

EthernetUDP* getUdpObject() {
  return &udp;
}
EthernetUDP* getNTPUdpObject() {
  return &ntpUdp;
}
// Legacy functions
bool isPacketReady() {
  return s_bundle_count > 0;
}

void sendPacketIfReady() {
  flushSamples();
}

void startSendingCollectedSamples() {
  s_sending_collected_samples = true;
}

void stopSendingCollectedSamples() {
  s_sending_collected_samples = false;
}

void sendBatchEndMarker() {
  // Send an empty packet with FLAGS_BATCH_END to signal batch completion
  if (s_bundle_count > 0) {
    flushSamples();  // Flush any remaining samples first
  }
  
  // Send empty packet with batch end flag
  uint8_t packet[HEADER_SIZE];
  memset(packet, 0, HEADER_SIZE);
  uint32_t* h = reinterpret_cast<uint32_t*>(packet);
  h[0] = htonl_custom(MSG_ID);
  h[1] = htonl_custom(FLAGS_BATCH_END);
  
  if (udp.beginPacket(PC_MCAST, UDP_PORT) == 1) {
    if (udp.write(packet, HEADER_SIZE) > 0) {
      udp.endPacket();
    }
  }
  Serial.println("[UDP] Sent batch end marker");
}

void onSampleTick(uint32_t irq_us) {
  // This function is now deprecated - use addSample() instead
  // Keeping for compatibility but it won't be called
}

void processIncoming() {
  int size = cmdUdp.parsePacket();
  if (size > 0) {
    const int BUF_SIZE = 128;
    uint8_t buf[BUF_SIZE];
    int len = cmdUdp.read(buf, min(size, BUF_SIZE));
    if (len > 64) {
      Serial.print(F("UdpManager: Received command: "));
      Serial.println(buf[64], HEX);
      uint8_t cmd = buf[64];
      switch (cmd) {
        // State management commands 
        case 0x01: StateManager::requestArm(); break;
        case 0x02: StateManager::triggerSoftwareActuate(); break;
        case 0x03: StateManager::requestDisarm(); break;
        case 0x04: 
          // Collect command with timing window - directly starts SampleCollector
          if (len >= 64 + 9) { // 1 byte command + 4 bytes start + 4 bytes stop
            // Parse range parameters (little-endian int32)
            int32_t start = *((int32_t*)(buf + 65));
            int32_t stop = *((int32_t*)(buf + 69));
            
            Serial.print(F("UdpManager: Collect command with range - start: "));
            Serial.print(start);
            Serial.print(F(", stop: "));
            Serial.println(stop);
            
            // Directly start sample collection with timing window
            SampleCollector::startGathering(start, stop);
          } else {
            Serial.println(F("UdpManager: Collect command missing range parameters"));
          }
          break;
        case 0x11: StateManager::manualActuatorControl(ACT_FWD); break;
        case 0x12: StateManager::manualActuatorControl(ACT_STOP); break;
        case 0x13: StateManager::manualActuatorControl(ACT_BWD); break;
        case 0x15: StateManager::manualEMEnable(); break;
        case 0x16: StateManager::manualEMDisable(); break;
        case 0x1F: StateManager::enableManualMode(); break;
        case 0x1E: StateManager::disableManualMode(); break;
        case 0x20: StateManager::enableHoldAfterFireMode(); break;
        case 0x21: StateManager::disableHoldAfterFireMode(); break;
        default: break;
      }
    }
  }
}

void update() {
  processIncoming();
}

void sendNeutrinoPacket() {
  if (s_bundle_count == 0) return;
  
  // Build one UDP datagram that contains the current bundle
  size_t packet_size = HEADER_SIZE + (DATA_SIZE_PER_SAMPLE * s_bundle_count);
  uint8_t packet[MAX_PACKET_SIZE];
  uint32_t* h = reinterpret_cast<uint32_t*>(packet);
  h[0] = htonl_custom(MSG_ID);
  h[1] = htonl_custom(s_sending_collected_samples ? FLAGS_COLLECTED_SAMPLES : FLAGS_NORMAL);
  h[2] = htonl_custom(schemaNumFrags);
  h[3] = htonl_custom(1);  // NUM_ATOMIC_FRAGS
  memcpy(packet + 16, schemaHash, 16);

  // Schema fragment cycles each packet
  memset(packet + 32, 0, FRAG_LEN);
  size_t schemaLen = strlen(schema);
  size_t offset = currentSchemaFrag * FRAG_LEN;
  if (offset < schemaLen) {
    size_t copyLen = min(FRAG_LEN, schemaLen - offset);
    memcpy(packet + 32, schema + offset, copyLen);
  }
  h[12] = htonl_custom(currentSchemaFrag);
  h[13] = htonl_custom(0);  // ATOMIC_IDX
  currentSchemaFrag = (currentSchemaFrag + 1) % schemaNumFrags;

  uint64_t t = htobe64_custom(getUnixTimeNanos());
  memcpy(packet + 56, &t, sizeof(uint64_t));

  // OPTIMIZED: Bulk copy payload data (faster than individual memcpy calls)
  uint8_t* d = packet + HEADER_SIZE;

  for (size_t i = 0; i < s_bundle_count; i++) {
    const TelemetrySample& sample = s_sample_bundle[i];
    
    // Copy all floats at once (20 bytes: 5 * sizeof(float))
    memcpy(d, &sample.sv, 20);
    d += 20;
    
    // Copy uint64_t NTP timestamp (start time)
    memcpy(d, &sample.us, sizeof(sample.us));
    d += sizeof(sample.us);

    // Copy all status bytes at once (6 bytes)
    *d++ = sample.ready;
    *d++ = sample.em;
    *d++ = sample.a;
    *d++ = sample.b;
    *d++ = sample.manual;
    *d++ = sample.hold;
    
    // Copy uint64_t NTP timestamp (end time)
    memcpy(d, &sample.us_end, sizeof(sample.us_end));
    d += sizeof(sample.us_end);
  }

  // PERFORMANCE: Remove error checking Serial.print calls to maximize speed
  // Send Packet - optimized for speed
  if (udp.beginPacket(PC_MCAST, UDP_PORT) == 1) {
   if (udp.write(packet, packet_size) > 0) {
     udp.endPacket();
   }
  }
}

}  // namespace UdpManager
