#include <Adafruit_MAX31855.h>

// Ajuster les broches selon ton cablage ESP32-C3 Super Mini.
constexpr int PIN_SCK = 4;
constexpr int PIN_CS = 7;
constexpr int PIN_MISO = 5;
constexpr int PIN_OIL_PRESSURE = A1;

Adafruit_MAX31855 thermocouple(PIN_SCK, PIN_CS, PIN_MISO);

constexpr unsigned long LOG_INTERVAL_MS = 500;

using TaskFn = void (*)();
struct Task {
  TaskFn fn;
  unsigned long intervalMs;
  unsigned long lastMs;
};

void logEgt();
void logOilPressure();

Task tasks[] = {
  {logEgt, LOG_INTERVAL_MS, 0},
  {logOilPressure, LOG_INTERVAL_MS, 0},
};
constexpr size_t TASK_COUNT = sizeof(tasks) / sizeof(tasks[0]);

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
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

}

void logOilPressure() {
  int oilPressureRaw = analogRead(PIN_OIL_PRESSURE);
  double vAdc = (static_cast<double>(oilPressureRaw) * 5) / 4095.0;
  double bar = (vAdc - 1.2)*3.5518;
  Serial.print("Oil pressure (bar): ");
  Serial.println(bar, 3);
}

