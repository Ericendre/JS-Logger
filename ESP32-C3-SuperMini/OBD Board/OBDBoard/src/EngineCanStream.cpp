#include "EngineCanStream.h"

#include <stdio.h>
#include <string.h>

namespace obd {

namespace {

constexpr uint16_t kIdDme1 = 0x316;
constexpr uint16_t kIdDme2 = 0x329;
constexpr uint16_t kIdDme4 = 0x545;
constexpr uint16_t kIdDme5 = 0x2A0;

void setError(char *buffer, size_t bufferSize, const char *message) {
  if (bufferSize == 0) {
    return;
  }
  strncpy(buffer, message, bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
}

}  // namespace

EngineCanStream::EngineCanStream(SPIClass &spi) : spi_(&spi) {}

void EngineCanStream::setConfig(const EngineCanConfig &config) {
  config_ = config;
  delete mcp_;
  mcp_ = new MCP2515(config.csPin);
}

void EngineCanStream::setWriter(EngineFieldWriter writer, void *context) {
  writer_ = writer;
  writerContext_ = context;
}

bool EngineCanStream::begin() {
  if (mcp_ == nullptr) {
    mcp_ = new MCP2515(config_.csPin);
  }

  spi_->begin(config_.sckPin, config_.misoPin, config_.mosiPin, config_.csPin);

  mcp_->reset();
  if (mcp_->setBitrate(config_.canSpeed, config_.canClock) != MCP2515::ERROR_OK) {
    setError(lastError_, sizeof(lastError_), "MCP2515 bitrate setup failed");
    return false;
  }

  mcp_->setFilterMask(MCP2515::MASK0, false, 0x7FF);
  mcp_->setFilterMask(MCP2515::MASK1, false, 0x7FF);
  mcp_->setFilter(MCP2515::RXF0, false, kIdDme1);
  mcp_->setFilter(MCP2515::RXF1, false, kIdDme2);
  mcp_->setFilter(MCP2515::RXF2, false, kIdDme4);
  mcp_->setFilter(MCP2515::RXF3, false, kIdDme5);

  if (mcp_->setNormalMode() != MCP2515::ERROR_OK) {
    setError(lastError_, sizeof(lastError_), "MCP2515 normal mode failed");
    return false;
  }

  lastError_[0] = '\0';
  return true;
}

void EngineCanStream::update() {
  const uint8_t frameBudget = max<uint8_t>(1, config_.maxFramesPerUpdate);
  for (uint8_t i = 0; i < frameBudget; ++i) {
    can_frame frame = {};
    const auto error = mcp_->readMessage(&frame);
    if (error == MCP2515::ERROR_NOMSG) {
      break;
    }
    if (error != MCP2515::ERROR_OK) {
      setError(lastError_, sizeof(lastError_), "CAN receive failed");
      break;
    }

    const uint16_t canId = frame.can_id & 0x7FF;
    if (canId == kIdDme1) {
      updateFromDme1(frame);
      dirty_ = true;
    } else if (canId == kIdDme2) {
      updateFromDme2(frame);
      dirty_ = true;
    } else if (canId == kIdDme4) {
      updateFromDme4(frame);
      dirty_ = true;
    } else if (canId == kIdDme5) {
      updateFromDme5(frame);
      dirty_ = true;
    }
  }

  publishIfDue();
}

bool EngineCanStream::ready() const { return data_.hasDme1 && data_.hasDme2; }

const EngineData &EngineCanStream::data() const { return data_; }

const char *EngineCanStream::lastError() const { return lastError_; }

bool EngineCanStream::publishIfDue() {
  if (!dirty_ || !ready() || writer_ == nullptr) {
    return false;
  }
  if (millis() - lastPublishMs_ < config_.publishIntervalMs) {
    return false;
  }

  dirty_ = false;
  lastPublishMs_ = millis();

  return publishUInt("key_on", data_.keyOn ? 1U : 0U) &&
         publishUInt("rpm_signal_error", data_.rpmSignalError ? 1U : 0U) &&
         publishUInt("tcs_ack", data_.tcsAck ? 1U : 0U) &&
         publishUInt("fuel_cut_active", data_.fuelCutActive ? 1U : 0U) &&
         publishUInt("ac_relay_active", data_.acRelayActive ? 1U : 0U) &&
         publishUInt("torque_calc_error", data_.torqueCalcError ? 1U : 0U) &&
         publishFloat("rpm", data_.rpm, 0) &&
         publishUInt("vehicle_speed", data_.vehicleSpeed) &&
         publishFloat("coolant_temp", data_.coolantTemp, 1) &&
         publishFloat("pedal_position", data_.pedalPosition, 1) &&
         publishUInt("atmospheric_pressure", data_.atmosphericPressure) &&
         publishFloat("indexed_engine_torque", data_.indexedEngineTorque, 1) &&
         publishFloat("indicated_engine_torque", data_.indicatedEngineTorque, 1) &&
         publishFloat("theoretical_engine_torque", data_.theoreticalEngineTorque, 1) &&
         publishFloat("torque_losses", data_.torqueLosses, 1) &&
         (!data_.hasDme4 ||
          (publishUInt("immobilizer_authenticated", data_.immobilizerAuthenticated ? 1U : 0U) &&
           publishUInt("mil_active", data_.milActive ? 1U : 0U) &&
           publishUInt("immobilizer_enabled", data_.immobilizerEnabled ? 1U : 0U) &&
           publishFloat("battery_voltage", data_.batteryVoltage, 2) &&
           publishFloat("fuel_consumption", data_.fuelConsumption, 2))) &&
         (!data_.hasDme5 || publishFloat("intake_air_temp", data_.intakeAirTemp, 1));
}

bool EngineCanStream::publishValue(const char *key, const char *value) {
  if (writer_ == nullptr) {
    return false;
  }
  return writer_(key, value, writerContext_);
}

bool EngineCanStream::publishUInt(const char *key, uint32_t value) {
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%lu", static_cast<unsigned long>(value));
  return publishValue(key, buffer);
}

bool EngineCanStream::publishFloat(const char *key, float value, uint8_t precision) {
  char buffer[20];
  dtostrf(value, 0, precision, buffer);
  return publishValue(key, buffer);
}

float EngineCanStream::decodeRpm(const can_frame &frame) {
  const uint16_t raw = static_cast<uint16_t>(frame.data[2]) |
                       (static_cast<uint16_t>(frame.data[3]) << 8);
  return raw * 0.15625f;
}

float EngineCanStream::decodeCoolantTemp(const can_frame &frame) {
  return static_cast<float>(frame.data[1]) * 0.75f - 48.0f;
}

float EngineCanStream::decodePedalPosition(const can_frame &frame) {
  return static_cast<float>(frame.data[5]) * 0.390625f;
}

uint16_t EngineCanStream::decodeAtmosphericPressure(const can_frame &frame) {
  return static_cast<uint16_t>(frame.data[2]) * 2U + 598U;
}

float EngineCanStream::decodeTorquePercent(uint8_t raw) { return static_cast<float>(raw) * 0.39f; }

float EngineCanStream::decodeBatteryVoltage(const can_frame &frame) {
  return static_cast<float>(frame.data[3]) * 0.1f;
}

float EngineCanStream::decodeFuelConsumption(const can_frame &frame) {
  const uint16_t raw = static_cast<uint16_t>(frame.data[1]) |
                       (static_cast<uint16_t>(frame.data[2]) << 8);
  return static_cast<float>(raw) * 0.128f;
}

float EngineCanStream::decodeIntakeAirTemp(const can_frame &frame) {
  return static_cast<float>(frame.data[1]) * 0.75f;
}

void EngineCanStream::updateFromDme1(const can_frame &frame) {
  data_.hasDme1 = true;
  data_.keyOn = (frame.data[0] & 0x01) != 0;
  data_.rpmSignalError = (frame.data[0] & 0x02) != 0;
  data_.tcsAck = (frame.data[0] & 0x04) != 0;
  data_.fuelCutActive = (frame.data[0] & 0x08) != 0;
  data_.acRelayActive = (frame.data[0] & 0x20) != 0;
  data_.torqueCalcError = (frame.data[0] & 0x40) != 0;
  data_.indexedEngineTorque = decodeTorquePercent(frame.data[1]);
  data_.rpm = decodeRpm(frame);
  data_.indicatedEngineTorque = decodeTorquePercent(frame.data[4]);
  data_.torqueLosses = decodeTorquePercent(frame.data[5]);
  data_.vehicleSpeed = frame.data[6];
  data_.theoreticalEngineTorque = decodeTorquePercent(frame.data[7]);
}

void EngineCanStream::updateFromDme2(const can_frame &frame) {
  data_.hasDme2 = true;
  data_.coolantTemp = decodeCoolantTemp(frame);
  data_.atmosphericPressure = decodeAtmosphericPressure(frame);
  data_.pedalPosition = decodePedalPosition(frame);
}

void EngineCanStream::updateFromDme4(const can_frame &frame) {
  data_.hasDme4 = true;
  data_.immobilizerAuthenticated = (frame.data[0] & 0x01) != 0;
  data_.milActive = (frame.data[0] & 0x02) != 0;
  data_.immobilizerEnabled = (frame.data[0] & 0x04) != 0;
  data_.batteryVoltage = decodeBatteryVoltage(frame);
  data_.fuelConsumption = decodeFuelConsumption(frame);
}

void EngineCanStream::updateFromDme5(const can_frame &frame) {
  data_.hasDme5 = true;
  data_.intakeAirTemp = decodeIntakeAirTemp(frame);
}

}  // namespace obd
