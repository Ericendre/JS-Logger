#include "src/OBDCanLogger.h"

using namespace obd;

namespace {

constexpr uint8_t kCanSckPin = 4;
constexpr uint8_t kCanMisoPin = 5;
constexpr uint8_t kCanMosiPin = 6;
constexpr uint8_t kCanCsPin = 7;
constexpr uint32_t kPollIntervalMs = 100;

KwpConfig config;
Mcp2515IsoTp isotp(kCanCsPin);
Kwp2000Client kwp(isotp, config);
HyundaiKwpLogger logger(kwp);

uint32_t lastPollMs = 0;

void printFatal(const char *message) {
  Serial.print("fatal: ");
  Serial.println(message);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(kCanSckPin, kCanMisoPin, kCanMosiPin, kCanCsPin);

  config.csPin = kCanCsPin;
  config.canSpeed = CAN_500KBPS;
  config.canClock = MCP_8MHZ;
  config.isotp.requestId = 0x7E0;
  config.isotp.responseId = 0x7E8;
  config.keepAliveIntervalMs = 1200;
  kwp.setConfig(config);

  if (!kwp.begin()) {
    printFatal(kwp.lastError());
    return;
  }

  if (!logger.begin()) {
    printFatal(logger.lastError());
    return;
  }

  logger.printCsvHeader(Serial);
}

void loop() {
  kwp.loop();

  if (millis() - lastPollMs < kPollIntervalMs) {
    delay(5);
    return;
  }

  lastPollMs = millis();
  if (!logger.poll(Serial)) {
    Serial.print("poll_error: ");
    Serial.println(logger.lastError());
    delay(250);
  }
}
