#include <SPI.h>
#include <mcp2515.h>

namespace {

constexpr uint8_t kCanSckPin = 4;
constexpr uint8_t kCanMisoPin = 5;
constexpr uint8_t kCanMosiPin = 6;
constexpr uint8_t kCanCsPin = 7;

constexpr CAN_SPEED kCanSpeed = CAN_500KBPS;
constexpr CAN_CLOCK kCanClock = MCP_8MHZ;

constexpr uint16_t kRequestId = 0x7E0;
constexpr uint16_t kResponseId = 0x7E8;

MCP2515 canController(kCanCsPin);

void printFrame(const can_frame &frame) {
  Serial.print("id=0x");
  Serial.print(frame.can_id & 0x7FF, HEX);
  Serial.print(" dlc=");
  Serial.print(frame.can_dlc);
  Serial.print(" data=");
  for (uint8_t i = 0; i < frame.can_dlc; ++i) {
    if (frame.data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(frame.data[i], HEX);
    if (i + 1 < frame.can_dlc) {
      Serial.print(' ');
    }
  }
  Serial.println();
}

bool sendSingleFrame(const uint8_t *payload, size_t payloadLength) {
  if (payloadLength > 7) {
    Serial.println("payload too large for single frame");
    return false;
  }

  can_frame frame = {};
  frame.can_id = kRequestId;
  frame.can_dlc = 8;
  frame.data[0] = static_cast<uint8_t>(payloadLength);
  for (uint8_t i = 0; i < payloadLength; ++i) {
    frame.data[i + 1] = payload[i];
  }

  const auto error = canController.sendMessage(&frame);
  if (error != MCP2515::ERROR_OK) {
    Serial.print("sendMessage failed, code=");
    Serial.println(static_cast<int>(error));
    return false;
  }

  Serial.print("tx ");
  printFrame(frame);
  return true;
}

bool waitForResponse(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    can_frame frame = {};
    const auto error = canController.readMessage(&frame);
    if (error == MCP2515::ERROR_OK) {
      if ((frame.can_id & 0x7FF) >= 0x7E8 && (frame.can_id & 0x7FF) <= 0x7EF) {
        Serial.print("rx ");
        printFrame(frame);
        return true;
      }
    }
    delay(2);
  }

  Serial.println("timeout waiting for 0x7E8");
  return false;
}

void sendProbe(const char *label, const uint8_t *payload, size_t payloadLength) {
  Serial.println();
  Serial.print("probe: ");
  Serial.println(label);
  if (!sendSingleFrame(payload, payloadLength)) {
    return;
  }
  waitForResponse(1000);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("KWP diagnostic probe");
  Serial.println("pins:");
  Serial.println("SCK=GPIO4 MISO=GPIO5 MOSI=GPIO6 CS=GPIO7");

  SPI.begin(kCanSckPin, kCanMisoPin, kCanMosiPin, kCanCsPin);

  canController.reset();
  if (canController.setBitrate(kCanSpeed, kCanClock) != MCP2515::ERROR_OK) {
    Serial.println("setBitrate failed");
    return;
  }
  if (canController.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("setNormalMode failed");
    return;
  }

  Serial.println("controller ready");
  Serial.println("sending GKFlasher-style probes to 0x7E0");

  const uint8_t startCommunication[] = {0x81};
  const uint8_t startFlashSession[] = {0x10, 0x85};
  const uint8_t readTimingLimits[] = {0x83, 0x00};
  const uint8_t testerPresent[] = {0x3E, 0x01};
  const uint8_t startDefaultSession[] = {0x10, 0x81};
  const uint8_t readLocalId01[] = {0x21, 0x01};
  const uint8_t obdMode01Pid00[] = {0x01, 0x00};

  sendProbe("KWP StartCommunication 0x81", startCommunication, sizeof(startCommunication));
  delay(150);
  sendProbe("KWP StartDiagnosticSession 0x10 0x85", startFlashSession, sizeof(startFlashSession));
  delay(150);
  sendProbe("KWP AccessTimingParameters 0x83 0x00", readTimingLimits, sizeof(readTimingLimits));
  delay(150);
  sendProbe("KWP TesterPresent 0x3E 0x01", testerPresent, sizeof(testerPresent));
  delay(150);
  sendProbe("KWP StartDiagnosticSession 0x10 0x81", startDefaultSession, sizeof(startDefaultSession));
  delay(150);
  sendProbe("KWP ReadDataByLocalIdentifier 0x21 0x01", readLocalId01, sizeof(readLocalId01));
  delay(150);
  sendProbe("OBD-II Mode 01 PID 00", obdMode01Pid00, sizeof(obdMode01Pid00));
}

void loop() {
  delay(1000);
}
