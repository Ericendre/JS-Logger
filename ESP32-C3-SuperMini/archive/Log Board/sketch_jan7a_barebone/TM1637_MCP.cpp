#include "TM1637_MCP.h"
#include <math.h>

const uint8_t TM1637_MCP::_digitMap[10] = {
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F  // 9
};

TM1637_MCP::TM1637_MCP(uint8_t i2cAddress) : _address(i2cAddress) {}

bool TM1637_MCP::begin(TwoWire &wirePort) {
  _wire = &wirePort;
  _wire->begin();

  if (!_mcp.begin_I2C(_address, _wire)) {
    return false;
  }

  // Tout le monde au repos à HIGH
  _gpioState = 0xFFFF;
  _mcp.writeGPIOAB(_gpioState);
  return true;
}

void TM1637_MCP::attachDisplay(uint8_t id, uint8_t clkPin, uint8_t dioPin) {
  if (id >= MAX_DISPLAYS) return;

  _displays[id].clk = clkPin;
  _displays[id].dio = dioPin;
  _displays[id].brightness = 7;
  _displays[id].attached = true;

  pinModeOutputFast(clkPin);
  pinModeOutputFast(dioPin);

  writeGPIOFast(clkPin, true);
  writeGPIOFast(dioPin, true);
}

void TM1637_MCP::setBrightness(uint8_t id, uint8_t brightness) {
  if (!validId(id)) return;
  if (brightness > 7) brightness = 7;
  _displays[id].brightness = brightness;
}

uint8_t TM1637_MCP::getBrightness(uint8_t id) const {
  if (!validId(id)) return 0;
  return _displays[id].brightness;
}

void TM1637_MCP::clear(uint8_t id) {
  if (!validId(id)) return;
  uint8_t blank[4] = {SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK};
  showRaw(id, blank);
}

void TM1637_MCP::showInt(uint8_t id, int value) {
  if (!validId(id)) return;

  uint8_t data[4];
  formatIntToSegments(value, data);
  writeData4(_displays[id].clk, _displays[id].dio, data, _displays[id].brightness);
}

void TM1637_MCP::showFloat(uint8_t id, float value, uint8_t decimals) {
  if (!validId(id)) return;

  if (decimals > 3) decimals = 3;

  uint8_t data[4];
  formatFloatToSegments(value, decimals, data);
  writeData4(_displays[id].clk, _displays[id].dio, data, _displays[id].brightness);
}

void TM1637_MCP::showRaw(uint8_t id, const uint8_t data[4]) {
  if (!validId(id)) return;
  writeData4(_displays[id].clk, _displays[id].dio, data, _displays[id].brightness);
}

bool TM1637_MCP::validId(uint8_t id) const {
  return (id < MAX_DISPLAYS) && _displays[id].attached;
}

void TM1637_MCP::pinModeOutputFast(uint8_t pin) {
  _mcp.pinMode(pin, OUTPUT);
}

void TM1637_MCP::flushGPIO() {
  _mcp.writeGPIOAB(_gpioState);
}

void TM1637_MCP::writeGPIOFast(uint8_t pin, bool level) {
  if (level) {
    _gpioState |= (1U << pin);
  } else {
    _gpioState &= ~(1U << pin);
  }
  flushGPIO();
}

void TM1637_MCP::start(uint8_t clk, uint8_t dio) {
  writeGPIOFast(dio, true);
  writeGPIOFast(clk, true);
  writeGPIOFast(dio, false);
}

void TM1637_MCP::stop(uint8_t clk, uint8_t dio) {
  writeGPIOFast(clk, false);
  writeGPIOFast(dio, false);
  writeGPIOFast(clk, true);
  writeGPIOFast(dio, true);
}

void TM1637_MCP::writeByte(uint8_t clk, uint8_t dio, uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    writeGPIOFast(clk, false);
    writeGPIOFast(dio, (value & 0x01) != 0);
    writeGPIOFast(clk, true);
    value >>= 1;
  }

  // ACK ignoré volontairement pour aller vite
  writeGPIOFast(clk, false);
  writeGPIOFast(dio, true);
  writeGPIOFast(clk, true);
}

void TM1637_MCP::sendCommand(uint8_t clk, uint8_t dio, uint8_t cmd) {
  start(clk, dio);
  writeByte(clk, dio, cmd);
  stop(clk, dio);
}

void TM1637_MCP::writeData4(uint8_t clk, uint8_t dio, const uint8_t data[4], uint8_t brightness) {
  if (brightness > 7) brightness = 7;

  // Data command: écriture auto-incrément
  sendCommand(clk, dio, 0x40);

  // Address command
  start(clk, dio);
  writeByte(clk, dio, 0xC0);
  writeByte(clk, dio, data[0]);
  writeByte(clk, dio, data[1]);
  writeByte(clk, dio, data[2]);
  writeByte(clk, dio, data[3]);
  stop(clk, dio);

  // Display control
  sendCommand(clk, dio, 0x88 | brightness);
}

void TM1637_MCP::formatIntToSegments(int value, uint8_t out[4]) {
  for (uint8_t i = 0; i < 4; i++) out[i] = SEG_BLANK;

  // Saturation sur ce qu'on peut afficher
  if (value > 9999) value = 9999;
  if (value < -999) value = -999;

  bool negative = (value < 0);
  int v = negative ? -value : value;

  if (!negative) {
    // Positif : blancs à gauche
    out[3] = _digitMap[v % 10];
    v /= 10;

    if (v > 0) {
      out[2] = _digitMap[v % 10];
      v /= 10;
    }
    if (v > 0) {
      out[1] = _digitMap[v % 10];
      v /= 10;
    }
    if (v > 0) {
      out[0] = _digitMap[v % 10];
    }

    // Cas 0
    if (value == 0) {
      out[3] = _digitMap[0];
    }
  } else {
    // Négatif : signe puis jusqu'à 3 chiffres
    out[0] = SEG_MINUS;
    out[3] = _digitMap[v % 10];
    v /= 10;

    if (v > 0) {
      out[2] = _digitMap[v % 10];
      v /= 10;
    }
    if (v > 0) {
      out[1] = _digitMap[v % 10];
    }
  }
}

void TM1637_MCP::formatFloatToSegments(float value, uint8_t decimals, uint8_t out[4]) {
  for (uint8_t i = 0; i < 4; i++) out[i] = SEG_BLANK;

  if (decimals > 3) decimals = 3;

  bool negative = (value < 0.0f);
  float absValue = negative ? -value : value;

  int mul = 1;
  for (uint8_t i = 0; i < decimals; i++) mul *= 10;

  long scaled = lroundf(absValue * mul);

  // Nombre de digits dispo
  // positif: 4 digits
  // négatif: 3 digits + "-"
  long maxScaled;
  if (negative) {
    if (decimals == 0) maxScaled = 999;
    else if (decimals == 1) maxScaled = 999;
    else if (decimals == 2) maxScaled = 99;
    else maxScaled = 9;
  } else {
    if (decimals == 0) maxScaled = 9999;
    else if (decimals == 1) maxScaled = 9999;
    else if (decimals == 2) maxScaled = 999;
    else maxScaled = 99;
  }

  if (scaled > maxScaled) {
    // Overflow simple: ----
    out[0] = SEG_MINUS;
    out[1] = SEG_MINUS;
    out[2] = SEG_MINUS;
    out[3] = SEG_MINUS;
    return;
  }

  if (!negative) {
    out[3] = _digitMap[scaled % 10];
    scaled /= 10;
    out[2] = (scaled > 0 || decimals >= 1) ? _digitMap[scaled % 10] : SEG_BLANK;
    scaled /= 10;
    out[1] = (scaled > 0 || decimals >= 2) ? _digitMap[scaled % 10] : SEG_BLANK;
    scaled /= 10;
    out[0] = (scaled > 0 || decimals >= 3) ? _digitMap[scaled % 10] : SEG_BLANK;
  } else {
    // signe à gauche
    out[0] = SEG_MINUS;
    out[3] = _digitMap[scaled % 10];
    scaled /= 10;
    out[2] = (scaled > 0 || decimals >= 1) ? _digitMap[scaled % 10] : SEG_BLANK;
    scaled /= 10;
    out[1] = (scaled > 0 || decimals >= 2) ? _digitMap[scaled % 10] : SEG_BLANK;
  }

  // Placement du point décimal
  // décimales = 1 -> XX.X -> point sur digit 2
  // décimales = 2 -> X.XX -> point sur digit 1
  // décimales = 3 -> .XXX -> point sur digit 0 (ou - .XX impossible proprement)
  if (decimals == 1) out[2] |= 0x80;
  else if (decimals == 2) out[1] |= 0x80;
  else if (decimals == 3) out[0] |= 0x80;

  // Corrige le cas négatif avec 3 décimales, qui n'est pas affichable proprement sur 4 digits
  if (negative && decimals == 3) {
    out[0] = SEG_MINUS;
    out[1] = SEG_MINUS;
    out[2] = SEG_MINUS;
    out[3] = SEG_MINUS;
  }
}