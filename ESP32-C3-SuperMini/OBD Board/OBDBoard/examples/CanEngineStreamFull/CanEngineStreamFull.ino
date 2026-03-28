#include <EngineCanStream.h>

namespace {

obd::EngineCanStream engineCan;
uint32_t lastPrintMs = 0;

void printValue(const char *key, uint32_t value) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\"=");
  Serial.print(value);
  Serial.print(';');
}

void printValue(const char *key, float value, uint8_t precision) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\"=");
  Serial.print(value, precision);
  Serial.print(';');
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("CAN engine stream (full)");
  Serial.println("Listening for 0x316, 0x329, 0x545, 0x2A0");

  obd::EngineCanConfig config;
  config.sckPin = 4;
  config.misoPin = 5;
  config.mosiPin = 6;
  config.csPin = 7;
  config.publishIntervalMs = 100;
  config.maxFramesPerUpdate = 4;

  engineCan.setConfig(config);

  if (!engineCan.begin()) {
    Serial.print("CAN init failed: ");
    Serial.println(engineCan.lastError());
    return;
  }

  Serial.println("listening...");
}

void loop() {
  engineCan.update();
  if (!engineCan.ready() || millis() - lastPrintMs < 100) {
    return;
  }

  lastPrintMs = millis();
  const obd::EngineData &data = engineCan.data();

  printValue("key_on", data.keyOn ? 1U : 0U);
  printValue("rpm_signal_error", data.rpmSignalError ? 1U : 0U);
  printValue("tcs_ack", data.tcsAck ? 1U : 0U);
  printValue("fuel_cut_active", data.fuelCutActive ? 1U : 0U);
  printValue("ac_relay_active", data.acRelayActive ? 1U : 0U);
  printValue("torque_calc_error", data.torqueCalcError ? 1U : 0U);
  printValue("rpm", data.rpm, 0);
  printValue("vehicle_speed", data.vehicleSpeed);
  printValue("coolant_temp", data.coolantTemp, 1);
  printValue("pedal_position", data.pedalPosition, 1);
  printValue("atmospheric_pressure", data.atmosphericPressure);
  printValue("indexed_engine_torque", data.indexedEngineTorque, 1);
  printValue("indicated_engine_torque", data.indicatedEngineTorque, 1);
  printValue("theoretical_engine_torque", data.theoreticalEngineTorque, 1);
  printValue("torque_losses", data.torqueLosses, 1);
  if (data.hasDme4) {
    printValue("immobilizer_authenticated", data.immobilizerAuthenticated ? 1U : 0U);
    printValue("mil_active", data.milActive ? 1U : 0U);
    printValue("immobilizer_enabled", data.immobilizerEnabled ? 1U : 0U);
    printValue("battery_voltage", data.batteryVoltage, 2);
    printValue("fuel_consumption", data.fuelConsumption, 2);
  }
  if (data.hasDme5) {
    printValue("intake_air_temp", data.intakeAirTemp, 1);
  }
  Serial.println();
}
