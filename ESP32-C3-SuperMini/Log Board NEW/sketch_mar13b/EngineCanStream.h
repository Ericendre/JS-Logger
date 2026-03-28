#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

namespace obd {

struct EngineCanConfig {
  uint8_t sckPin = 4;
  uint8_t misoPin = 5;
  uint8_t mosiPin = 6;
  uint8_t csPin = 7;
  CAN_SPEED canSpeed = CAN_500KBPS;
  CAN_CLOCK canClock = MCP_8MHZ;
  uint32_t publishIntervalMs = 100;
  uint8_t maxFramesPerUpdate = 2;
};

struct EngineData {
  bool hasDme1 = false;
  bool hasDme2 = false;
  bool hasDme4 = false;
  bool hasDme5 = false;
  bool keyOn = false;
  bool rpmSignalError = false;
  bool tcsAck = false;
  bool fuelCutActive = false;
  bool acRelayActive = false;
  bool torqueCalcError = false;
  float rpm = 0.0f;
  uint8_t vehicleSpeed = 0;
  float coolantTemp = 0.0f;
  float pedalPosition = 0.0f;
  uint16_t atmosphericPressure = 0;
  float indexedEngineTorque = 0.0f;
  float indicatedEngineTorque = 0.0f;
  float theoreticalEngineTorque = 0.0f;
  float torqueLosses = 0.0f;
  bool immobilizerAuthenticated = false;
  bool milActive = false;
  bool immobilizerEnabled = false;
  float batteryVoltage = 0.0f;
  float fuelConsumption = 0.0f;
  float intakeAirTemp = 0.0f;
};

using EngineFieldWriter = bool (*)(const char *key, const char *value, void *context);

class EngineCanStream {
 public:
  explicit EngineCanStream(SPIClass &spi = SPI);

  void setConfig(const EngineCanConfig &config);
  void setWriter(EngineFieldWriter writer, void *context = nullptr);

  bool begin();
  void update();

  bool ready() const;
  const EngineData &data() const;
  const char *lastError() const;

 private:
  bool publishIfDue();
  bool publishValue(const char *key, const char *value);
  bool publishUInt(const char *key, uint32_t value);
  bool publishFloat(const char *key, float value, uint8_t precision);

  static float decodeRpm(const can_frame &frame);
  static float decodeCoolantTemp(const can_frame &frame);
  static float decodePedalPosition(const can_frame &frame);
  static uint16_t decodeAtmosphericPressure(const can_frame &frame);
  static float decodeTorquePercent(uint8_t raw);
  static float decodeBatteryVoltage(const can_frame &frame);
  static float decodeFuelConsumption(const can_frame &frame);
  static float decodeIntakeAirTemp(const can_frame &frame);

  void updateFromDme1(const can_frame &frame);
  void updateFromDme2(const can_frame &frame);
  void updateFromDme4(const can_frame &frame);
  void updateFromDme5(const can_frame &frame);

  SPIClass *spi_ = nullptr;
  MCP2515 *mcp_ = nullptr;
  EngineCanConfig config_{};
  EngineFieldWriter writer_ = nullptr;
  void *writerContext_ = nullptr;
  EngineData data_{};
  bool dirty_ = false;
  uint32_t lastPublishMs_ = 0;
  char lastError_[96] = {0};
};

}  // namespace obd
