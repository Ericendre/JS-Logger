#ifndef TM1637_MCP_H
#define TM1637_MCP_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#define MCP_A0 0
#define MCP_A1 1
#define MCP_A2 2
#define MCP_A3 3
#define MCP_A4 4
#define MCP_A5 5
#define MCP_A6 6
#define MCP_A7 7

#define MCP_B0 8
#define MCP_B1 9
#define MCP_B2 10
#define MCP_B3 11
#define MCP_B4 12
#define MCP_B5 13
#define MCP_B6 14
#define MCP_B7 15

class TM1637_MCP {
public:
  static const uint8_t MAX_DISPLAYS = 4;

  explicit TM1637_MCP(uint8_t i2cAddress = 0x20);

  bool begin(TwoWire &wirePort = Wire);
  void attachDisplay(uint8_t id, uint8_t clkPin, uint8_t dioPin);

  void setBrightness(uint8_t id, uint8_t brightness);
  uint8_t getBrightness(uint8_t id) const;

  void clear(uint8_t id);
  void showInt(uint8_t id, int value);
  void showFloat(uint8_t id, float value, uint8_t decimals = 1);
  void showText(uint8_t id, const char* text);

  // Optionnel si tu veux envoyer directement les 4 segments
  void showRaw(uint8_t id, const uint8_t data[4]);

  void pinModeLed(uint8_t pin, uint8_t mode);
  void digitalWriteLed(uint8_t pin, bool state);
  void setErrorLeds(bool state);

private:
  struct DisplayConfig {
    uint8_t clk = 255;
    uint8_t dio = 255;
    uint8_t brightness = 7;
    uint8_t lastData[4] = {0, 0, 0, 0};
    uint8_t lastBrightness = 255;
    bool attached = false;
    bool cacheValid = false;
  };

  Adafruit_MCP23X17 _mcp;
  TwoWire *_wire = nullptr;
  uint8_t _address;
  uint16_t _gpioState = 0xFFFF;   // sorties au repos à HIGH
  DisplayConfig _displays[MAX_DISPLAYS];

  static const uint8_t _digitMap[10];
  static const uint8_t _errorLedPins[7];
  static const uint8_t SEG_MINUS = 0x40;
  static const uint8_t SEG_BLANK = 0x00;
  static const uint8_t SEG_E = 0x79;
  static const uint8_t SEG_r = 0x50;
  static const uint8_t SEG_SPACE = 0x00;
  static const uint8_t SEG_DASH = 0x40;

  void pinModeOutputFast(uint8_t pin);
  void writeGPIOFast(uint8_t pin, bool level);
  void flushGPIO();

  void start(uint8_t clk, uint8_t dio);
  void stop(uint8_t clk, uint8_t dio);
  void writeByte(uint8_t clk, uint8_t dio, uint8_t value);
  void sendCommand(uint8_t clk, uint8_t dio, uint8_t cmd);
  void writeData4(uint8_t clk, uint8_t dio, const uint8_t data[4], uint8_t brightness);
  bool shouldSkipWrite(uint8_t id, const uint8_t data[4]) const;
  void updateDisplayCache(uint8_t id, const uint8_t data[4]);

  bool validId(uint8_t id) const;
  void formatIntToSegments(int value, uint8_t out[4]);
  void formatFloatToSegments(float value, uint8_t decimals, uint8_t out[4]);
};

#endif
