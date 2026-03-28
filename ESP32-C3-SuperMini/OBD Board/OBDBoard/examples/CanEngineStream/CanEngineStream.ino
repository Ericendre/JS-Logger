#include <SPI.h>
#include <mcp2515.h>

namespace {

constexpr uint8_t kCanSckPin = 4;
constexpr uint8_t kCanMisoPin = 5;
constexpr uint8_t kCanMosiPin = 6;
constexpr uint8_t kCanCsPin = 7;

constexpr CAN_SPEED kCanSpeed = CAN_500KBPS;
constexpr CAN_CLOCK kCanClock = MCP_8MHZ;

constexpr uint16_t kIdDme1 = 0x316;
constexpr uint16_t kIdDme2 = 0x329;

MCP2515 canController(kCanCsPin);

struct EngineData {
  bool hasDme1 = false;
  bool hasDme2 = false;

  bool keyOn = false;
  float rpm = 0.0f;
  uint8_t vehicleSpeed = 0;
  float coolantTemp = 0.0f;
  float pedalPosition = 0.0f;
  uint16_t atmosphericPressure = 0;
  float indexedEngineTorque = 0.0f;
  float indicatedEngineTorque = 0.0f;
  float theoreticalEngineTorque = 0.0f;
  float torqueLosses = 0.0f;
};

EngineData engineData;
uint32_t lastPrintMs = 0;

float decodeRpm(const can_frame &frame) {
  const uint16_t raw = static_cast<uint16_t>(frame.data[2]) |
                       (static_cast<uint16_t>(frame.data[3]) << 8);
  return raw * 0.15625f;
}

float decodeCoolantTemp(const can_frame &frame) {
  return static_cast<float>(frame.data[1]) * 0.75f - 48.0f;
}

float decodePedalPosition(const can_frame &frame) {
  return static_cast<float>(frame.data[5]) * 0.390625f;
}

uint16_t decodeAtmosphericPressure(const can_frame &frame) {
  return static_cast<uint16_t>(frame.data[2]) * 2U + 598U;
}

float decodeTorquePercent(uint8_t raw) {
  return static_cast<float>(raw) * 0.39f;
}

void updateFromDme1(const can_frame &frame) {
  engineData.hasDme1 = true;
  engineData.keyOn = (frame.data[0] & 0x01) != 0;
  engineData.indexedEngineTorque = decodeTorquePercent(frame.data[1]);
  engineData.rpm = decodeRpm(frame);
  engineData.indicatedEngineTorque = decodeTorquePercent(frame.data[4]);
  engineData.torqueLosses = decodeTorquePercent(frame.data[5]);
  engineData.vehicleSpeed = frame.data[6];
  engineData.theoreticalEngineTorque = decodeTorquePercent(frame.data[7]);
}

void updateFromDme2(const can_frame &frame) {
  engineData.hasDme2 = true;
  engineData.coolantTemp = decodeCoolantTemp(frame);
  engineData.atmosphericPressure = decodeAtmosphericPressure(frame);
  engineData.pedalPosition = decodePedalPosition(frame);
}

void printValue(const char *key, float value, uint8_t precision) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\"=");
  Serial.print(value, precision);
  Serial.print(';');
}

void printValue(const char *key, uint32_t value) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\"=");
  Serial.print(value);
  Serial.print(';');
}

void printValue(const char *key, uint16_t value) {
  printValue(key, static_cast<uint32_t>(value));
}

void printValue(const char *key, uint8_t value) {
  printValue(key, static_cast<uint32_t>(value));
}

void printValue(const char *key, bool value) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\"=");
  Serial.print(value ? 1 : 0);
  Serial.print(';');
}

void printStream() {
  printValue("key_on", engineData.keyOn);
  printValue("rpm", engineData.rpm, 0);
  printValue("vehicle_speed", engineData.vehicleSpeed);
  printValue("coolant_temp", engineData.coolantTemp, 1);
  printValue("pedal_position", engineData.pedalPosition, 1);
  printValue("atmospheric_pressure", engineData.atmosphericPressure);
  printValue("indexed_engine_torque", engineData.indexedEngineTorque, 1);
  printValue("indicated_engine_torque", engineData.indicatedEngineTorque, 1);
  printValue("theoretical_engine_torque", engineData.theoreticalEngineTorque, 1);
  printValue("torque_losses", engineData.torqueLosses, 1);
  Serial.println();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("CAN engine stream");
  Serial.println("SCK=GPIO4 MISO=GPIO5 MOSI=GPIO6 CS=GPIO7");

  SPI.begin(kCanSckPin, kCanMisoPin, kCanMosiPin, kCanCsPin);

  canController.reset();
  if (canController.setBitrate(kCanSpeed, kCanClock) != MCP2515::ERROR_OK) {
    Serial.println("setBitrate failed");
    return;
  }

  canController.setFilterMask(MCP2515::MASK0, false, 0x7FF);
  canController.setFilter(MCP2515::RXF0, false, kIdDme1);
  canController.setFilter(MCP2515::RXF1, false, kIdDme2);

  if (canController.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("setNormalMode failed");
    return;
  }

  Serial.println("listening...");
}

void loop() {
  can_frame frame = {};
  const auto error = canController.readMessage(&frame);
  if (error == MCP2515::ERROR_OK) {
    const uint16_t canId = frame.can_id & 0x7FF;
    if (canId == kIdDme1) {
      updateFromDme1(frame);
    } else if (canId == kIdDme2) {
      updateFromDme2(frame);
    }
  }

  if (millis() - lastPrintMs >= 200 && engineData.hasDme1 && engineData.hasDme2) {
    lastPrintMs = millis();
    printStream();
  }
}
