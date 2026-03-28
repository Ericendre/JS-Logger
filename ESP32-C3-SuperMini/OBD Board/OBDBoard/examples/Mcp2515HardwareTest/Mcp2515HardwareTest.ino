#include <SPI.h>
#include <mcp2515.h>

namespace {

constexpr uint8_t kCanSckPin = 4;
constexpr uint8_t kCanMisoPin = 5;
constexpr uint8_t kCanMosiPin = 6;
constexpr uint8_t kCanCsPin = 7;

constexpr CAN_SPEED kCanSpeed = CAN_500KBPS;
constexpr CAN_CLOCK kCanClock = MCP_8MHZ;

MCP2515 canController(kCanCsPin);

bool printFrame(const can_frame &frame) {
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
  return true;
}

bool testSpiAndMcp() {
  Serial.println("step 1: reset MCP2515");
  canController.reset();

  Serial.println("step 2: configure bitrate");
  if (canController.setBitrate(kCanSpeed, kCanClock) != MCP2515::ERROR_OK) {
    Serial.println("fail: setBitrate");
    return false;
  }

  Serial.println("step 3: enter loopback mode");
  if (canController.setLoopbackMode() != MCP2515::ERROR_OK) {
    Serial.println("fail: setLoopbackMode");
    return false;
  }

  can_frame txFrame = {};
  txFrame.can_id = 0x123;
  txFrame.can_dlc = 8;
  txFrame.data[0] = 0xDE;
  txFrame.data[1] = 0xAD;
  txFrame.data[2] = 0xBE;
  txFrame.data[3] = 0xEF;
  txFrame.data[4] = 0xCA;
  txFrame.data[5] = 0xFE;
  txFrame.data[6] = 0x12;
  txFrame.data[7] = 0x34;

  Serial.println("step 4: send loopback frame");
  if (canController.sendMessage(&txFrame) != MCP2515::ERROR_OK) {
    Serial.println("fail: sendMessage in loopback");
    return false;
  }

  Serial.println("step 5: read loopback frame");
  const uint32_t start = millis();
  while (millis() - start < 1000) {
    can_frame rxFrame = {};
    const auto error = canController.readMessage(&rxFrame);
    if (error == MCP2515::ERROR_OK) {
      Serial.println("ok: MCP2515 SPI and internal CAN logic work");
      printFrame(rxFrame);
      return true;
    }
    delay(5);
  }

  Serial.println("fail: no loopback frame received");
  return false;
}

void listenOnBus() {
  Serial.println("step 6: switch to normal mode");
  if (canController.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("fail: setNormalMode");
    return;
  }

  Serial.println("listening on CAN for 10 seconds");
  Serial.println("if frames appear here, MCP2515 + TJA1050 + wiring are alive");

  const uint32_t start = millis();
  uint32_t frameCount = 0;
  while (millis() - start < 10000) {
    can_frame frame = {};
    const auto error = canController.readMessage(&frame);
    if (error == MCP2515::ERROR_OK) {
      ++frameCount;
      printFrame(frame);
    }
    delay(2);
  }

  Serial.print("listen complete, frames=");
  Serial.println(frameCount);
  if (frameCount == 0) {
    Serial.println("no bus traffic seen");
    Serial.println("possible causes: wrong CAN speed, wiring issue, no active bus, or transceiver problem");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("MCP2515 hardware test");
  Serial.println("wiring:");
  Serial.println("ESP SCK=GPIO4 MISO=GPIO6 MOSI=GPIO7 CS=GPIO10");

  SPI.begin(kCanSckPin, kCanMisoPin, kCanMosiPin, kCanCsPin);

  if (!testSpiAndMcp()) {
    Serial.println("hardware self-test failed");
    Serial.println("check CS/SCK/MISO/MOSI, 3.3V on MCP2515 VDD, and common ground");
    return;
  }

  listenOnBus();
}

void loop() {
  delay(1000);
}
