#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

namespace obd {

enum class KwpSession : uint8_t {
  Default = 0x81,
  FlashReprogramming = 0x85,
  Engineering = 0x86,
  Adjustment = 0x87,
  Standby = 0x89,
  Passive = 0x90,
  ExtendedDiagnostic = 0x92
};

enum class KwpResponseType : uint8_t {
  ResponseRequired = 0x01,
  ResponseNotRequired = 0x02
};

enum class Endianness : uint8_t {
  Little,
  Big
};

struct IsoTpConfig {
  uint16_t requestId = 0x7E0;
  uint16_t responseId = 0x7E8;
  uint8_t flowControlBlockSize = 0;
  uint8_t flowControlSeparationTimeMs = 0;
  uint32_t responseTimeoutMs = 250;
  uint32_t firstFrameTimeoutMs = 250;
  uint32_t consecutiveFrameTimeoutMs = 120;
};

struct KwpConfig {
  IsoTpConfig isotp;
  uint8_t csPin = 10;
  uint32_t spiClockHz = 8000000;
  CAN_SPEED canSpeed = CAN_500KBPS;
  CAN_CLOCK canClock = MCP_8MHZ;
  uint32_t keepAliveIntervalMs = 1200;
};

struct ParameterDefinition {
  const char *name;
  const char *unit;
  uint8_t localIdentifier;
  uint16_t position;
  uint8_t size;
  Endianness endianness;
  float scale;
  float offset;
  uint8_t precision;
  uint32_t bitMask;
  uint8_t bitShift;
};

struct DecodedParameter {
  const ParameterDefinition *definition = nullptr;
  float value = 0.0f;
  uint32_t raw = 0;
  bool valid = false;
};

class Mcp2515IsoTp {
 public:
  Mcp2515IsoTp(uint8_t csPin, SPIClass &spi = SPI);

  bool begin(const KwpConfig &config);
  bool sendAndReceive(const uint8_t *payload,
                      size_t payloadLength,
                      uint8_t *responseBuffer,
                      size_t &responseLength,
                      size_t responseCapacity);
  bool receive(uint8_t *responseBuffer, size_t &responseLength, size_t responseCapacity);
  bool readFrame(struct can_frame &frame, uint32_t timeoutMs);
  const char *lastError() const;

 private:
  bool sendSingleFrame(const uint8_t *payload, size_t payloadLength);
  bool receiveIsoTpPayload(uint8_t *responseBuffer,
                           size_t &responseLength,
                           size_t responseCapacity);
  bool sendFlowControlFrame();
  bool sendCanFrame(const struct can_frame &frame);
  static uint32_t millisSince(uint32_t startMs);

  MCP2515 mcp_;
  SPIClass *spi_ = nullptr;
  uint8_t csPin_ = 10;
  KwpConfig config_{};
  char lastError_[96] = {0};
};

class Kwp2000Client {
 public:
  Kwp2000Client(Mcp2515IsoTp &transport, const KwpConfig &config);

  void setConfig(const KwpConfig &config);
  bool begin();
  void loop();

  bool startCommunication();
  bool startDiagnosticSession(KwpSession session, int8_t baudrateIdentifier = -1);
  bool testerPresent(KwpResponseType responseType = KwpResponseType::ResponseRequired);
  bool readDataByLocalIdentifier(uint8_t localIdentifier,
                                 uint8_t *responseBuffer,
                                 size_t &responseLength,
                                 size_t responseCapacity);
  bool execute(uint8_t serviceId,
               const uint8_t *requestData,
               size_t requestLength,
               uint8_t *responseBuffer,
               size_t &responseLength,
               size_t responseCapacity);

  const char *lastError() const;

 private:
  bool buildAndSend(uint8_t serviceId,
                    const uint8_t *requestData,
                    size_t requestLength,
                    uint8_t *responseBuffer,
                    size_t &responseLength,
                    size_t responseCapacity);
  bool handleKwpResponse(uint8_t serviceId,
                         uint8_t *responseBuffer,
                         size_t &responseLength,
                         size_t responseCapacity);

  Mcp2515IsoTp &transport_;
  KwpConfig config_;
  uint32_t lastActivityMs_ = 0;
  char lastError_[96] = {0};
};

class HyundaiKwpLogger {
 public:
  explicit HyundaiKwpLogger(Kwp2000Client &client);

  bool begin();
  bool printCsvHeader(Stream &stream);
  bool poll(Stream &stream);
  bool poll(DecodedParameter *results, size_t &resultCount, size_t resultCapacity);

  const char *lastError() const;
  static const ParameterDefinition *definitions(size_t &count);

 private:
  bool fetchSource(uint8_t localIdentifier, uint8_t *payload, size_t &payloadLength, size_t payloadCapacity);
  static uint32_t readValue(const uint8_t *payload, const ParameterDefinition &definition);
  static float convertValue(uint32_t raw, const ParameterDefinition &definition);

  Kwp2000Client &client_;
  char lastError_[96] = {0};
};

}  // namespace obd
