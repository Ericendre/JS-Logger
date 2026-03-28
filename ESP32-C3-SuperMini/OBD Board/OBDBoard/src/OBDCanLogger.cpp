#include "OBDCanLogger.h"

#include <math.h>
#include <string.h>

namespace obd {

namespace {

constexpr uint8_t kServiceStartCommunication = 0x81;
constexpr uint8_t kServiceStartDiagnosticSession = 0x10;
constexpr uint8_t kServiceReadDataByLocalIdentifier = 0x21;
constexpr uint8_t kServiceTesterPresent = 0x3E;
constexpr uint8_t kNegativeResponseSid = 0x7F;
constexpr uint8_t kResponsePendingCode = 0x78;

constexpr size_t kMaxSourcePayload = 192;

ParameterDefinition kHyundaiDefinitions[] = {
    {"Battery voltage", "V", 0x01, 1, 1, Endianness::Little, 0.10159f, 0.0f, 2, 0, 0},
    {"Engine Coolant Temperature Sensor", "C", 0x01, 4, 1, Endianness::Little, 0.75f, -48.0f, 2, 0, 0},
    {"Oil Temperature Sensor", "C", 0x01, 6, 1, Endianness::Little, 1.0f, -40.0f, 2, 0, 0},
    {"Intake Air Temperature Sensor", "C", 0x01, 9, 1, Endianness::Little, 0.75f, -48.0f, 2, 0, 0},
    {"Throttle Position", "deg", 0x01, 11, 1, Endianness::Little, 0.468627f, 0.0f, 2, 0, 0},
    {"Adapted Throttle Position", "deg", 0x01, 12, 2, Endianness::Little, 0.001825f, 0.0f, 2, 0, 0},
    {"Cranking Signal", "", 0x01, 14, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x02, 1},
    {"Closed Throttle Position", "", 0x01, 14, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x04, 2},
    {"Part Load Status", "", 0x01, 14, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x08, 3},
    {"Air Flow Rate from Mass Air Flow Sensor", "kg/h", 0x01, 15, 2, Endianness::Little, 0.03125f, 0.0f, 2, 0, 0},
    {"Vehicle Speed", "km/h", 0x01, 30, 1, Endianness::Little, 1.0f, 0.0f, 0, 0, 0},
    {"Engine Speed", "RPM", 0x01, 31, 2, Endianness::Little, 1.0f, 0.0f, 0, 0, 0},
    {"Target Idle Speed", "RPM", 0x01, 33, 2, Endianness::Little, 1.0f, 0.0f, 0, 0, 0},
    {"Transaxle Range Switch", "", 0x01, 36, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x01, 0},
    {"A/C Switch", "", 0x01, 37, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x01, 0},
    {"A/C Pressure Switch", "", 0x01, 37, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x02, 1},
    {"A/C Relay", "", 0x01, 37, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x04, 2},
    {"Oxygen Sensor-Bank1/Sensor1", "mV", 0x01, 38, 2, Endianness::Little, 4.883f, 0.0f, 2, 0, 0},
    {"Oxygen Sensor-Bank1/Sensor2", "mV", 0x01, 40, 2, Endianness::Little, 4.883f, 0.0f, 2, 0, 0},
    {"Ignition Timing Advance for 1 Cylinder", "deg", 0x01, 58, 1, Endianness::Little, -0.325f, -72.0f, 2, 0, 0},
    {"Cylinder Injection Time-Bank1", "ms", 0x01, 76, 2, Endianness::Little, 0.004f, 0.0f, 3, 0, 0},
    {"Fuel System Status", "", 0x01, 84, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x01, 0},
    {"Long Term Fuel Trim-Idle Load", "ms", 0x01, 89, 2, Endianness::Little, 0.004f, 0.0f, 3, 0, 0},
    {"Long Term Fuel Trim-Part Load", "%", 0x01, 91, 2, Endianness::Little, 0.001529f, 0.0f, 3, 0, 0},
    {"Oxygen Sensor Heater Duty-Bank1/Sensor1", "%", 0x01, 93, 1, Endianness::Little, 0.390625f, 0.0f, 2, 0, 0},
    {"Oxygen Sensor Heater Duty-Bank1/Sensor2", "%", 0x01, 94, 1, Endianness::Little, 0.390625f, 0.0f, 2, 0, 0},
    {"Idle speed control actuator", "%", 0x01, 99, 2, Endianness::Little, 0.001529f, 0.0f, 3, 0, 0},
    {"EVAP Purge valve", "%", 0x01, 101, 2, Endianness::Little, 0.003052f, 0.0f, 3, 0, 0},
    {"Ignition dwell time", "ms", 0x01, 106, 2, Endianness::Little, 0.004f, 0.0f, 3, 0, 0},
    {"Camshaft Actual Position", "deg", 0x01, 142, 1, Endianness::Little, 0.375f, 60.0f, 2, 0, 0},
    {"Camshaft position target", "deg", 0x01, 143, 1, Endianness::Little, 0.375f, 60.0f, 2, 0, 0},
    {"CVVT Status", "", 0x01, 145, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x07, 1},
    {"CVVT Actuation Status", "", 0x01, 146, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x03, 1},
    {"CVVT Valve Duty", "%", 0x01, 156, 2, Endianness::Little, 0.001526f, 0.0f, 3, 0, 0},
    {"CVVT Duty Control Status", "", 0x01, 160, 1, Endianness::Little, 1.0f, 0.0f, 0, 0x03, 1},
};

void setError(char *buffer, size_t bufferSize, const char *message) {
  if (bufferSize == 0) {
    return;
  }
  strncpy(buffer, message, bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
}

void setErrorCode(char *buffer, size_t bufferSize, const char *prefix, uint8_t code) {
  snprintf(buffer, bufferSize, "%s 0x%02X", prefix, code);
}

float roundToPrecision(float value, uint8_t precision) {
  float factor = 1.0f;
  for (uint8_t i = 0; i < precision; ++i) {
    factor *= 10.0f;
  }
  return roundf(value * factor) / factor;
}

}  // namespace

Mcp2515IsoTp::Mcp2515IsoTp(uint8_t csPin, SPIClass &spi) : mcp_(csPin), spi_(&spi), csPin_(csPin) {}

bool Mcp2515IsoTp::begin(const KwpConfig &config) {
  config_ = config;
  csPin_ = config.csPin;

  spi_->begin();
  mcp_.reset();
  if (mcp_.setBitrate(config.canSpeed, config.canClock) != MCP2515::ERROR_OK) {
    setError(lastError_, sizeof(lastError_), "MCP2515 bitrate setup failed");
    return false;
  }
  if (mcp_.setNormalMode() != MCP2515::ERROR_OK) {
    setError(lastError_, sizeof(lastError_), "MCP2515 normal mode failed");
    return false;
  }

  struct can_frame filterFrame = {};
  filterFrame.can_id = config.isotp.responseId;
  filterFrame.can_dlc = 0;
  mcp_.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp_.setFilter(MCP2515::RXF0, false, filterFrame.can_id);
  mcp_.setFilter(MCP2515::RXF1, false, filterFrame.can_id);

  lastError_[0] = '\0';
  return true;
}

bool Mcp2515IsoTp::sendAndReceive(const uint8_t *payload,
                                  size_t payloadLength,
                                  uint8_t *responseBuffer,
                                  size_t &responseLength,
                                  size_t responseCapacity) {
  if (!sendSingleFrame(payload, payloadLength)) {
    return false;
  }
  return receiveIsoTpPayload(responseBuffer, responseLength, responseCapacity);
}

bool Mcp2515IsoTp::receive(uint8_t *responseBuffer, size_t &responseLength, size_t responseCapacity) {
  return receiveIsoTpPayload(responseBuffer, responseLength, responseCapacity);
}

bool Mcp2515IsoTp::sendSingleFrame(const uint8_t *payload, size_t payloadLength) {
  if (payloadLength > 7) {
    setError(lastError_, sizeof(lastError_), "Requests longer than 7 bytes are not implemented");
    return false;
  }

  struct can_frame frame = {};
  frame.can_id = config_.isotp.requestId;
  frame.can_dlc = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = static_cast<uint8_t>(payloadLength & 0x0F);
  memcpy(&frame.data[1], payload, payloadLength);
  return sendCanFrame(frame);
}

bool Mcp2515IsoTp::receiveIsoTpPayload(uint8_t *responseBuffer,
                                       size_t &responseLength,
                                       size_t responseCapacity) {
  responseLength = 0;

  struct can_frame frame = {};
  if (!readFrame(frame, config_.isotp.firstFrameTimeoutMs)) {
    return false;
  }

  const uint8_t pciType = frame.data[0] >> 4;
  if (pciType == 0x0) {
    const size_t payloadLength = frame.data[0] & 0x0F;
    if (payloadLength > 7 || payloadLength > responseCapacity) {
      setError(lastError_, sizeof(lastError_), "Single frame response too large");
      return false;
    }
    memcpy(responseBuffer, &frame.data[1], payloadLength);
    responseLength = payloadLength;
    return true;
  }

  if (pciType != 0x1) {
    setError(lastError_, sizeof(lastError_), "Unexpected ISO-TP frame type");
    return false;
  }

  const size_t totalLength = static_cast<size_t>((frame.data[0] & 0x0F) << 8) | frame.data[1];
  if (totalLength > responseCapacity) {
    setError(lastError_, sizeof(lastError_), "Response buffer too small");
    return false;
  }

  const size_t firstChunkLength = min(static_cast<size_t>(6), totalLength);
  memcpy(responseBuffer, &frame.data[2], firstChunkLength);
  responseLength = firstChunkLength;

  if (!sendFlowControlFrame()) {
    return false;
  }

  uint8_t expectedSequence = 1;
  while (responseLength < totalLength) {
    if (!readFrame(frame, config_.isotp.consecutiveFrameTimeoutMs)) {
      return false;
    }

    const uint8_t consecutiveType = frame.data[0] >> 4;
    const uint8_t sequence = frame.data[0] & 0x0F;
    if (consecutiveType != 0x2) {
      setError(lastError_, sizeof(lastError_), "Expected consecutive frame");
      return false;
    }
    if (sequence != expectedSequence) {
      setError(lastError_, sizeof(lastError_), "ISO-TP sequence mismatch");
      return false;
    }

    const size_t bytesLeft = totalLength - responseLength;
    const size_t chunkLength = min(static_cast<size_t>(7), bytesLeft);
    memcpy(&responseBuffer[responseLength], &frame.data[1], chunkLength);
    responseLength += chunkLength;
    expectedSequence = (expectedSequence + 1) & 0x0F;
  }

  return true;
}

bool Mcp2515IsoTp::sendFlowControlFrame() {
  struct can_frame frame = {};
  frame.can_id = config_.isotp.requestId;
  frame.can_dlc = 8;
  memset(frame.data, 0, sizeof(frame.data));
  frame.data[0] = 0x30;
  frame.data[1] = config_.isotp.flowControlBlockSize;
  frame.data[2] = config_.isotp.flowControlSeparationTimeMs;
  return sendCanFrame(frame);
}

bool Mcp2515IsoTp::sendCanFrame(const struct can_frame &frame) {
  if (mcp_.sendMessage(&frame) != MCP2515::ERROR_OK) {
    setError(lastError_, sizeof(lastError_), "CAN transmit failed");
    return false;
  }
  return true;
}

bool Mcp2515IsoTp::readFrame(struct can_frame &frame, uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  while (millisSince(startMs) < timeoutMs) {
    const auto error = mcp_.readMessage(&frame);
    if (error == MCP2515::ERROR_OK) {
      if ((frame.can_id & 0x7FF) == config_.isotp.responseId) {
        return true;
      }
    } else if (error != MCP2515::ERROR_NOMSG) {
      setError(lastError_, sizeof(lastError_), "CAN receive failed");
      return false;
    }
    delay(1);
  }

  setError(lastError_, sizeof(lastError_), "CAN receive timeout");
  return false;
}

const char *Mcp2515IsoTp::lastError() const { return lastError_; }

uint32_t Mcp2515IsoTp::millisSince(uint32_t startMs) { return millis() - startMs; }

Kwp2000Client::Kwp2000Client(Mcp2515IsoTp &transport, const KwpConfig &config)
    : transport_(transport), config_(config) {}

void Kwp2000Client::setConfig(const KwpConfig &config) { config_ = config; }

bool Kwp2000Client::begin() {
  if (!transport_.begin(config_)) {
    setError(lastError_, sizeof(lastError_), transport_.lastError());
    return false;
  }
  lastActivityMs_ = millis();
  lastError_[0] = '\0';
  return true;
}

void Kwp2000Client::loop() {
  if (config_.keepAliveIntervalMs == 0) {
    return;
  }
  const uint32_t elapsed = millis() - lastActivityMs_;
  if (elapsed >= config_.keepAliveIntervalMs) {
    testerPresent(KwpResponseType::ResponseRequired);
  }
}

bool Kwp2000Client::startCommunication() {
  uint8_t response[32];
  size_t responseLength = 0;
  return buildAndSend(kServiceStartCommunication, nullptr, 0, response, responseLength, sizeof(response));
}

bool Kwp2000Client::startDiagnosticSession(KwpSession session, int8_t baudrateIdentifier) {
  uint8_t request[2] = {static_cast<uint8_t>(session), 0};
  size_t requestLength = 1;
  if (baudrateIdentifier >= 0) {
    request[1] = static_cast<uint8_t>(baudrateIdentifier);
    requestLength = 2;
  }

  uint8_t response[32];
  size_t responseLength = 0;
  return buildAndSend(
      kServiceStartDiagnosticSession, request, requestLength, response, responseLength, sizeof(response));
}

bool Kwp2000Client::testerPresent(KwpResponseType responseType) {
  uint8_t request[1] = {static_cast<uint8_t>(responseType)};
  uint8_t response[32];
  size_t responseLength = 0;
  return buildAndSend(kServiceTesterPresent, request, sizeof(request), response, responseLength, sizeof(response));
}

bool Kwp2000Client::readDataByLocalIdentifier(uint8_t localIdentifier,
                                              uint8_t *responseBuffer,
                                              size_t &responseLength,
                                              size_t responseCapacity) {
  uint8_t request[1] = {localIdentifier};
  return buildAndSend(kServiceReadDataByLocalIdentifier,
                      request,
                      sizeof(request),
                      responseBuffer,
                      responseLength,
                      responseCapacity);
}

bool Kwp2000Client::execute(uint8_t serviceId,
                            const uint8_t *requestData,
                            size_t requestLength,
                            uint8_t *responseBuffer,
                            size_t &responseLength,
                            size_t responseCapacity) {
  return buildAndSend(serviceId, requestData, requestLength, responseBuffer, responseLength, responseCapacity);
}

const char *Kwp2000Client::lastError() const { return lastError_; }

bool Kwp2000Client::buildAndSend(uint8_t serviceId,
                                 const uint8_t *requestData,
                                 size_t requestLength,
                                 uint8_t *responseBuffer,
                                 size_t &responseLength,
                                 size_t responseCapacity) {
  uint8_t request[8] = {0};
  request[0] = serviceId;
  if (requestLength > sizeof(request) - 1) {
    setError(lastError_, sizeof(lastError_), "KWP request too large");
    return false;
  }
  if (requestLength > 0 && requestData != nullptr) {
    memcpy(&request[1], requestData, requestLength);
  }

  if (!transport_.sendAndReceive(request,
                                 requestLength + 1,
                                 responseBuffer,
                                 responseLength,
                                 responseCapacity)) {
    setError(lastError_, sizeof(lastError_), transport_.lastError());
    return false;
  }

  if (!handleKwpResponse(serviceId, responseBuffer, responseLength, responseCapacity)) {
    return false;
  }

  lastActivityMs_ = millis();
  lastError_[0] = '\0';
  return true;
}

bool Kwp2000Client::handleKwpResponse(uint8_t serviceId,
                                      uint8_t *responseBuffer,
                                      size_t &responseLength,
                                      size_t responseCapacity) {
  if (responseLength < 1) {
    setError(lastError_, sizeof(lastError_), "Empty KWP response");
    return false;
  }

  if (responseBuffer[0] == static_cast<uint8_t>(serviceId + 0x40)) {
    return true;
  }

  if (responseBuffer[0] == kNegativeResponseSid) {
    if (responseLength < 3) {
      setError(lastError_, sizeof(lastError_), "Malformed negative response");
      return false;
    }
    if (responseBuffer[2] == kResponsePendingCode) {
      if (!transport_.receive(responseBuffer, responseLength, responseCapacity)) {
        setError(lastError_, sizeof(lastError_), transport_.lastError());
        return false;
      }
      return handleKwpResponse(serviceId, responseBuffer, responseLength, responseCapacity);
    }
    setErrorCode(lastError_, sizeof(lastError_), "Negative response code", responseBuffer[2]);
    return false;
  }

  setErrorCode(lastError_, sizeof(lastError_), "Unexpected response SID", responseBuffer[0]);
  return false;
}

HyundaiKwpLogger::HyundaiKwpLogger(Kwp2000Client &client) : client_(client) {}

bool HyundaiKwpLogger::begin() {
  if (!client_.startCommunication()) {
    setError(lastError_, sizeof(lastError_), client_.lastError());
    return false;
  }
  if (!client_.startDiagnosticSession(KwpSession::Default)) {
    setError(lastError_, sizeof(lastError_), client_.lastError());
    return false;
  }
  lastError_[0] = '\0';
  return true;
}

bool HyundaiKwpLogger::printCsvHeader(Stream &stream) {
  size_t count = 0;
  const ParameterDefinition *defs = definitions(count);
  stream.print("timestamp_ms");
  for (size_t i = 0; i < count; ++i) {
    stream.print(',');
    stream.print(defs[i].name);
    if (defs[i].unit != nullptr && defs[i].unit[0] != '\0') {
      stream.print(" [");
      stream.print(defs[i].unit);
      stream.print(']');
    }
  }
  stream.println();
  return true;
}

bool HyundaiKwpLogger::poll(Stream &stream) {
  DecodedParameter results[sizeof(kHyundaiDefinitions) / sizeof(kHyundaiDefinitions[0])] = {};
  size_t resultCount = 0;
  if (!poll(results, resultCount, sizeof(results) / sizeof(results[0]))) {
    return false;
  }

  stream.print(millis());
  for (size_t i = 0; i < resultCount; ++i) {
    stream.print(',');
    if (!results[i].valid) {
      stream.print("nan");
      continue;
    }
    stream.print(results[i].value, results[i].definition->precision);
  }
  stream.println();
  return true;
}

bool HyundaiKwpLogger::poll(DecodedParameter *results, size_t &resultCount, size_t resultCapacity) {
  size_t definitionCount = 0;
  const ParameterDefinition *defs = definitions(definitionCount);
  if (resultCapacity < definitionCount) {
    setError(lastError_, sizeof(lastError_), "Result buffer too small");
    return false;
  }

  uint8_t payload[kMaxSourcePayload] = {0};
  size_t payloadLength = 0;
  if (!fetchSource(0x01, payload, payloadLength, sizeof(payload))) {
    return false;
  }

  for (size_t i = 0; i < definitionCount; ++i) {
    results[i].definition = &defs[i];
    results[i].valid = defs[i].position + defs[i].size <= payloadLength;
    if (!results[i].valid) {
      continue;
    }
    results[i].raw = readValue(payload, defs[i]);
    results[i].value = convertValue(results[i].raw, defs[i]);
  }

  resultCount = definitionCount;
  lastError_[0] = '\0';
  return true;
}

const char *HyundaiKwpLogger::lastError() const { return lastError_; }

const ParameterDefinition *HyundaiKwpLogger::definitions(size_t &count) {
  count = sizeof(kHyundaiDefinitions) / sizeof(kHyundaiDefinitions[0]);
  return kHyundaiDefinitions;
}

bool HyundaiKwpLogger::fetchSource(uint8_t localIdentifier,
                                   uint8_t *payload,
                                   size_t &payloadLength,
                                   size_t payloadCapacity) {
  if (!client_.readDataByLocalIdentifier(localIdentifier, payload, payloadLength, payloadCapacity)) {
    setError(lastError_, sizeof(lastError_), client_.lastError());
    return false;
  }
  if (payloadLength < 2 || payload[1] != localIdentifier) {
    setError(lastError_, sizeof(lastError_), "Unexpected local identifier response");
    return false;
  }
  lastError_[0] = '\0';
  return true;
}

uint32_t HyundaiKwpLogger::readValue(const uint8_t *payload, const ParameterDefinition &definition) {
  if (definition.bitMask != 0) {
    return (payload[definition.position] & definition.bitMask) >> definition.bitShift;
  }

  uint32_t value = 0;
  if (definition.endianness == Endianness::Little) {
    for (uint8_t i = 0; i < definition.size; ++i) {
      value |= static_cast<uint32_t>(payload[definition.position + i]) << (8 * i);
    }
  } else {
    for (uint8_t i = 0; i < definition.size; ++i) {
      value = (value << 8) | payload[definition.position + i];
    }
  }
  return value;
}

float HyundaiKwpLogger::convertValue(uint32_t raw, const ParameterDefinition &definition) {
  const float value = static_cast<float>(raw) * definition.scale + definition.offset;
  return roundToPrecision(value, definition.precision);
}

}  // namespace obd
