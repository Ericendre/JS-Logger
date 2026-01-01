#include <Adafruit_MAX31855.h>

// Ajuster les broches selon ton cablage ESP32-C3 Super Mini.
constexpr int PIN_SCK = 4;
constexpr int PIN_CS = 7;
constexpr int PIN_MISO = 5;
constexpr int PIN_CS_2 = 8;

Adafruit_MAX31855 thermocouple(PIN_SCK, PIN_CS, PIN_MISO);
Adafruit_MAX31855 thermocouple2(PIN_SCK, PIN_CS_2, PIN_MISO);

constexpr unsigned long LOG_INTERVAL_MS = 500;
constexpr unsigned long GUARD_TIME_MS = 50;

using TaskFn = void (*)();
struct Task {
  TaskFn fn;
  unsigned long intervalMs;
  unsigned long lastMs;
};

void logEgt();

Task tasks[] = {
  {logEgt, LOG_INTERVAL_MS, 0},
};
constexpr size_t TASK_COUNT = sizeof(tasks) / sizeof(tasks[0]);

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_CS_2, OUTPUT);
  digitalWrite(PIN_CS_2, HIGH);

  Serial.begin(115200);
  delay(250);
  Serial.println("MAX31855 init...");
}

void loop() {
  unsigned long now = millis();
  for (size_t i = 0; i < TASK_COUNT; ++i) {
    Task &t = tasks[i];
    if (now - t.lastMs >= t.intervalMs) {
      t.lastMs = now;
      t.fn();
    }
  }
}

void logEgt() {
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CS_2, HIGH);

  double celsius = thermocouple.readCelsius();
  if (isnan(celsius)) {
    uint8_t fault = thermocouple.readError();
    if (fault == 0x02) {
      return; // ignore short to GND
    }
    fault &= ~0x02;
    if (fault == 0) {
      return;
    }
    Serial.print("Fault MAX31855: 0x");
    Serial.println(fault, HEX);
  } else {
    Serial.print("EGT (C): ");
    Serial.println(celsius, 2);
  }

  delay(GUARD_TIME_MS);

  double celsius2 = thermocouple2.readCelsius();
  if (isnan(celsius2)) {
    uint8_t fault = thermocouple2.readError();
    if (fault == 0x02) {
      return; // ignore short to GND
    }
    fault &= ~0x02;
    if (fault == 0) {
      return;
    }
    Serial.print("Fault MAX31855 #2: 0x");
    Serial.println(fault, HEX);
  } else {
    Serial.print("EGT2 (C): ");
    Serial.println(celsius2, 2);
  }
}
