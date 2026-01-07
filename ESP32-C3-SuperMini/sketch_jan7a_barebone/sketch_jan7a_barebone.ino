#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Base code for ESP32-C3 Super Mini sensor sampling and serial output.
// Adjust SENSOR_READ_HZ and the sensor stubs for your hardware.

// ================== BLE UART (Nordic UART Service) ==================
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BLECharacteristic *g_tx_char = nullptr;
bool g_device_connected = false;

// Sampling frequency in Hz.
constexpr float SENSOR_READ_HZ = 5.0f;

// Computed sampling period in microseconds.
constexpr uint32_t SENSOR_READ_PERIOD_US =
    static_cast<uint32_t>(1000000.0f / SENSOR_READ_HZ);

// Simple data container for your sensors.
struct SensorData {
  float air_fuel_ratio;
};

SensorData g_latest_data;
uint32_t g_next_read_us = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    g_device_connected = true;
  }

  void onDisconnect(BLEServer *pServer) override {
    (void)pServer;
    g_device_connected = false;
    delay(200);
    BLEDevice::startAdvertising();
  }
};

static void bleSendLine(const String &line) {
  if (!g_device_connected || g_tx_char == nullptr) return;
  const int chunk_size = 200;
  for (int i = 0; i < (int)line.length(); i += chunk_size) {
    String chunk = line.substring(i, i + chunk_size);
    g_tx_char->setValue((uint8_t *)chunk.c_str(), chunk.length());
    g_tx_char->notify();
    delay(5);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  BLEDevice::init("ESP32C3-SuperMini");
  BLEDevice::setMTU(247);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);
  g_tx_char = service->createCharacteristic(
      TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  g_tx_char->addDescriptor(new BLE2902());

  service->createCharacteristic(
      RX_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->start();

  // TODO: initialize sensors here.
}

void loop() {
  const uint32_t now_us = micros();
  if (timeToRead(now_us)) {
    g_next_read_us = now_us + SENSOR_READ_PERIOD_US;
    lireCapteurs(g_latest_data);
    envoyerSerie(g_latest_data);
  }
}

bool timeToRead(uint32_t now_us) {
  // Handles micros() overflow safely with unsigned math.
  return static_cast<int32_t>(now_us - g_next_read_us) >= 0;
}

void lireCapteurs(SensorData &out) {
  const int raw = analogRead(A0);
  const float vref = 3.3f;
  const float voltage = (static_cast<float>(raw) / 4095.0f) * vref;
  const float afr_min = 10.0f;
  const float afr_max = 18.5f;
  const float v_min = 1.0f;
  const float v_max = 1.85f;

  float afr = afr_min +
              (voltage - v_min) * (afr_max - afr_min) / (v_max - v_min);
  if (afr < afr_min) afr = afr_min;
  if (afr > afr_max) afr = afr_max;
  out.air_fuel_ratio = afr;
}

void envoyerSerie(const SensorData &data) {
  // TODO: replace with your desired format.
  String line;
  line.reserve(48);
  line += "Air/Fuel Ratio: ";
  line += String(data.air_fuel_ratio, 2);
  line += "\n";

  Serial.print(line);
  bleSendLine(line);
}
