
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <TM1637_MCP.h>
#include "EngineCanStream.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BLECharacteristic *g_tx_char = nullptr;
BLECharacteristic *g_rx_char = nullptr;
bool g_device_connected = false;
Preferences g_preferences;

TM1637_MCP displays(0x20);
obd::EngineCanStream engineCan;
bool displaysAvailable = false;

bool testMode = true;

uint32_t lastDisplay = 0;
uint32_t lastLogic = 0;
uint32_t lastStream = 0;

uint32_t delayDisplay = 5;
uint32_t delayLogic = 10;
uint32_t delayStream = 100;

// Stockage de petits couples cle/valeur recus par ESP-NOW.
constexpr size_t MAX_REMOTE_FIELDS = 32;
constexpr size_t MAX_KEY_LEN = 16;
constexpr size_t MAX_VAL_LEN = 20;
constexpr size_t MAX_ESPNOW_MSG_LEN = 127;

struct KeyValue {
  char key[MAX_KEY_LEN];
  char value[MAX_VAL_LEN];
  bool used;
  uint32_t updatedAt = 0;
};

KeyValue g_remote_fields[MAX_REMOTE_FIELDS];
char g_pending_espnow_data[MAX_ESPNOW_MSG_LEN + 1] = {};
volatile int g_pending_espnow_len = 0;
volatile bool g_pending_espnow_ready = false;
portMUX_TYPE g_espnow_mux = portMUX_INITIALIZER_UNLOCKED;

// Timestamp de la dernière réception de données (pour détecter le timeout).
uint32_t lastDataReceived = 0;

// Flag pour savoir si on affiche ---- sur tous les afficheurs.
bool showingErr = false;
uint32_t lastErrorLedStep = 0;
uint8_t errorLedIndex = 0;
int8_t errorLedDirection = 1;
uint32_t lastWarningBlinkStep = 0;
bool warningBlinkState = false;
uint32_t lastTimeoutDisplayStep = 0;
uint8_t timeoutDisplayFrame = 0;
uint8_t timeoutDisplayIndex = 0;

// Permet d'accéder aux valeurs par clé en utilisant la syntaxe carData["rpm"].
// Exemple : Serial.print(carData["rpm"]);
// Si la clé n'existe pas, renvoie une chaîne vide.
static int findRemoteField(const char *key);  // déclaré avant usage ci-dessous
struct RemoteFieldsAccessor {
  const char *operator[](const char *key) const {
    int idx = findRemoteField(key);
    return (idx >= 0) ? g_remote_fields[idx].value : "";
  }
};
static const RemoteFieldsAccessor carData;
static int upsertRemoteField(const char *key, const char *value);
static bool onEngineField(const char *key, const char *value, void *context);
static bool tryGetFieldFloat(const char *key, float &value);
static bool tryGetAnyFieldFloat(float &value, const char *key1, const char *key2 = nullptr);
static void processPendingEspNowMessage();
static bool isFieldTimedOut(const char *key, uint32_t timeoutMs = 500);
static void updateTimeoutDisplayAnimation();
static void showTimeoutDisplayFrame(uint8_t displayId);

constexpr uint8_t WARNING_LED_COUNT = 7;
// Assigne ici chaque nom de warning a une LED.
const uint8_t warningLedPins[WARNING_LED_COUNT] = {
  MCP_A0, MCP_B0, MCP_A1, MCP_B1, MCP_A2, MCP_B2, MCP_A3
};
const char* warningLedNames[WARNING_LED_COUNT] = {
  "overheat",
  "oil_pressure",
  "air_temp",
  "afr",
  "egt",
  "none",
  "low_battery"
};
bool warningLedStates[WARNING_LED_COUNT] = {};

static int findWarningIndex(const char *name);
struct WarningAccessor {
  bool fallback = false;

  bool &operator[](const char *name) {
    const int idx = findWarningIndex(name);
    if (idx >= 0) {
      return warningLedStates[idx];
    }
    fallback = false;
    return fallback;
  }
};
static WarningAccessor warnings;

// Configuration des clés à afficher sur chaque afficheur (modifie dans setup()).
char displayKeyStorage[4][MAX_KEY_LEN] = {};
const char* displayKeys[4];
// Configuration des décimales pour chaque afficheur (0 à 3).
uint8_t displayDecimals[4];
static void trimWhitespace(char *s);
static bool setDisplayKey(uint8_t index, const char *key);
static bool applyBleCommand(char *command);
static void loadDisplayConfig();
static void saveDisplayConfig();
static void updateErrorLed(bool timeoutActive);
static void clearErrorLeds();
static void clearTimeoutDisplayAnimation();

// Callbacks BLE: gestion connexion/deconnexion d'un client.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    g_device_connected = true;
    Serial.println("BLE client connected");
  }

  void onDisconnect(BLEServer *pServer) override {
    (void)pServer;
    g_device_connected = false;
    Serial.println("BLE client disconnected");
    delay(200);
    BLEDevice::startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    char buffer[64];
    const size_t copyLength = value.length() < sizeof(buffer) - 1 ? value.length() : sizeof(buffer) - 1;
    memcpy(buffer, value.c_str(), copyLength);
    buffer[copyLength] = '\0';

    if (testMode) {
      Serial.print("BLE RX: ");
      Serial.println(buffer);
    }

    if (!applyBleCommand(buffer) && testMode) {
      Serial.println("BLE RX ignored");
    }
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

  if (testMode) {
    Serial.print("BLE TX: ");
    Serial.println(line);
  }
}

static bool tryGetFieldFloat(const char *key, float &value) {
  if (key == nullptr || key[0] == '\0') {
    return false;
  }

  const char *valueStr = carData[key];
  if (valueStr == nullptr || valueStr[0] == '\0') {
    return false;
  }

  value = atof(valueStr);
  return true;
}

static bool tryGetAnyFieldFloat(float &value, const char *key1, const char *key2) {
  return tryGetFieldFloat(key1, value) || tryGetFieldFloat(key2, value);
}

void setup()
{
    Serial.begin(115200);
    g_preferences.begin("displaycfg", false);

    obd::EngineCanConfig engineCanConfig;
    engineCanConfig.sckPin = 4;
    engineCanConfig.misoPin = 5;
    engineCanConfig.mosiPin = 6;
    engineCanConfig.csPin = 7;
    engineCanConfig.publishIntervalMs = 100;
    engineCanConfig.maxFramesPerUpdate = 2;
    engineCan.setConfig(engineCanConfig);
    engineCan.setWriter(onEngineField, nullptr);
    if (!engineCan.begin()) {
        Serial.print("CAN init failed: ");
        Serial.println(engineCan.lastError());
    }

    // Initialisation du serveur ESP-NOW.
    Serial.println("ESP-NOW debug: init WiFi STA");
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
    } else {
        Serial.println("ESP-NOW init OK");
        esp_now_register_recv_cb(onDataRecv);
    }
    Serial.print("STA MAC: ");
    Serial.println(WiFi.macAddress());

    // Initialisation du serveur BLE.
    BLEDevice::init("ESP32C3-SuperMini");
    BLEDevice::setMTU(247);

    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService *service = server->createService(SERVICE_UUID);
    g_tx_char = service->createCharacteristic(
        TX_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    g_tx_char->addDescriptor(new BLE2902());

    g_rx_char = service->createCharacteristic(
        RX_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    g_rx_char->setCallbacks(new RxCallbacks());

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);
    advertising->start();

    // Configuration des afficheurs : chaque slot affiche une clé spécifique.
    // Modifie ces chaînes pour changer ce qui s'affiche sur chaque afficheur.
    setDisplayKey(0, "air_pressure");
    setDisplayKey(1, "air_temp");
    setDisplayKey(2, "oil_pressure");
    setDisplayKey(3, "AFR");

    // Configuration des décimales pour chaque afficheur (0 = entier, 1 = 1 décimale, etc.).
    displayDecimals[0] = 2; 
    displayDecimals[1] = 1;
    displayDecimals[2] = 2;
    displayDecimals[3] = 1;
    loadDisplayConfig();

    displaysAvailable = displays.begin();
    if (!displaysAvailable) {
        Serial.println("MCP23017 non detecte, affichage desactive");
    } else {
        displays.attachDisplay(0, MCP_B7, MCP_A7);
        displays.attachDisplay(1, MCP_B6, MCP_A6);
        displays.attachDisplay(2, MCP_B5, MCP_A5);
        displays.attachDisplay(3, MCP_B4, MCP_A4);
        displays.setErrorLeds(false);

        displays.setBrightness(0, 7);
        displays.setBrightness(1, 7);
        displays.setBrightness(2, 7);
        displays.setBrightness(3, 7);

        displays.clear(0);
        displays.clear(1);
        displays.clear(2);
        displays.clear(3);

        clearTimeoutDisplayAnimation();
        for (uint8_t i = 0; i < 12; ++i) {
            updateTimeoutDisplayAnimation();
            delay(70);
        }
        clearTimeoutDisplayAnimation();
    }
    
}


void loop()
{
    engineCan.update();
    processPendingEspNowMessage();

    if (millis() - lastLogic >= delayLogic)// Gestion d'erreur et calcul de données
    {
        lastLogic = millis();
        dataManager_update();
    }

    if (millis() - lastStream >= delayStream)// Envoi des données au PC via BLE
    {
        lastStream = millis();
        stream_update();
    }

    if (millis() - lastDisplay >= delayDisplay)// Affichage des données sur les display + pixels
    {
        lastDisplay = millis();
        if (!displaysAvailable) {
            return;
        }

        // Vérifie le timeout : si pas de données depuis 1s, affiche ---- sur tous les afficheurs
        bool timeout = (millis() - lastDataReceived > 500);
        if (timeout && !showingErr) {
            clearTimeoutDisplayAnimation();
            displays.setErrorLeds(true);
            showingErr = true;
        }

        updateErrorLed(timeout);
        updateTimeoutDisplayAnimation();

        if (!timeout && showingErr) {
            clearTimeoutDisplayAnimation();
            clearErrorLeds();
            showingErr = false;
            // Ne reset pas, laisse display_update() reprendre le cycle normal
        }

        display_update();
    }
}

void dataManager_update()
{
    float rpm = 0.0f;
    float oil_pressure = 0.0f;
    float oil_temp = 0.0f;
    float coolant_temp = 0.0f;
    float air_temp = 0.0f;
    float air_pressure = 0.0f;
    float afr = 0.0f;
    float egt = 0.0f;
    float battery = 0.0f;
    float pedal_position = 0.0f;
    float vehicle_speed = 0.0f;


    const bool hasRpm = tryGetAnyFieldFloat(rpm, "rpm", "RPM");
    const bool hasOilPressure = tryGetAnyFieldFloat(oil_pressure, "oil_pressure", "OilPressure");
    const bool hasCoolantTemp = tryGetAnyFieldFloat(coolant_temp, "coolant_temp", "CoolantTemp");
    const bool hasOilTemp = tryGetAnyFieldFloat(oil_temp, "oil_temp", "OilTemp");
    const bool hasAirTemp = tryGetAnyFieldFloat(air_temp, "air_temp", "AirTemp");
    const bool hasAfr = tryGetAnyFieldFloat(afr, "AFR", "afr");
    const bool hasEgt = tryGetAnyFieldFloat(egt, "egt", "EGT");
    const bool hasBattery = tryGetAnyFieldFloat(battery, "battery", "voltage");
    const bool hasPedalPosition = tryGetAnyFieldFloat(pedal_position, "pedal_position", "PedalPosition");
    const bool hasVehicleSpeed = tryGetAnyFieldFloat(vehicle_speed, "vehicle_speed", "VehicleSpeed");

    warnings["overheat"] =
        (hasCoolantTemp && coolant_temp > 100.0f) ||
        (hasOilTemp && oil_temp > 120.0f);

    warnings["oil_pressure"] =
        hasRpm && hasOilPressure &&
        rpm > 2000.0f &&
        oil_pressure < 3.0f;

    warnings["air_temp"] =
        hasVehicleSpeed && hasAirTemp &&
        vehicle_speed > 20.0f && air_temp > 40.0f;

    warnings["afr"] =
        hasAfr &&
        hasPedalPosition &&
        pedal_position > 50.0f &&
        (afr < 12.0f || afr > 14.0f);

    warnings["egt"] =
        hasEgt &&
        egt > 900.0f;

    warnings["none"] = false;

    warnings["low_battery"] =
        hasBattery &&
        battery < 12.0f;
}

void stream_update()
{
    if (testMode) {
        Serial.println("stream_update tick");
    }

    if (!g_device_connected || g_tx_char == nullptr) {
        if (testMode) {
            Serial.println("BLE TX skipped: no client");
        }
        return;
    }

    const bool timeout = (millis() - lastDataReceived > 500);
    if (timeout) {
        bleSendLine(String("\"ts\"=") + millis() + ";\n");
        return;
    }

    String line;
    line.reserve(MAX_REMOTE_FIELDS * (MAX_KEY_LEN + MAX_VAL_LEN + 6));

    for (size_t i = 0; i < MAX_REMOTE_FIELDS; ++i) {
        if (!g_remote_fields[i].used || g_remote_fields[i].key[0] == '\0' ||
            g_remote_fields[i].value[0] == '\0') {
            continue;
        }

        line += '"';
        line += g_remote_fields[i].key;
        line += "\"=";
        line += g_remote_fields[i].value;
        line += ';';
    }

    if (line.length() == 0) {
        if (testMode) {
            Serial.println("BLE TX skipped: no fields");
        }
        return;
    }

    line += '\n';
    bleSendLine(line);
    // Envoi des données au PC via BLE
    
}

void display_update()
{
    static uint8_t displayIndex = 0;

    // Récupère la clé configurée pour cet afficheur
    const char* key = displayKeys[displayIndex];
    if (!key) {
        // Si pas de clé configurée, affiche ----
        displays.showText(displayIndex, "----");
    } else if (isFieldTimedOut(key)) {
        showTimeoutDisplayFrame(displayIndex);
    } else {
        // Récupère la valeur associée à la clé
        const char* valueStr = carData[key];
        if (valueStr && valueStr[0] != '\0') {
            // Convertit la chaîne en float et affiche
            float value = atof(valueStr);
            displays.showFloat(displayIndex, value, displayDecimals[displayIndex]);
        } else {
            // Si valeur non disponible, affiche ----
            displays.showText(displayIndex, "----");
        }
    }

    displayIndex++;
    if (displayIndex >= 4)
        displayIndex = 0;
}

// Cherche si une clé distante existe déjà dans le tableau.
static int findRemoteField(const char *key) 
{
  if (!key) return -1;
  for (size_t i = 0; i < MAX_REMOTE_FIELDS; ++i) {
    if (g_remote_fields[i].used &&
        strncmp(g_remote_fields[i].key, key, MAX_KEY_LEN) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

static bool isFieldTimedOut(const char *key, uint32_t timeoutMs) {
  const int idx = findRemoteField(key);
  if (idx < 0) {
    return true;
  }

  const uint32_t updatedAt = g_remote_fields[idx].updatedAt;
  if (updatedAt == 0) {
    return true;
  }

  return (millis() - updatedAt) > timeoutMs;
}

// Ajoute ou met à jour une valeur distante reçue via ESP-NOW.
static int upsertRemoteField(const char *key, const char *value) {
  if (!key || !value) return -1;
  int idx = findRemoteField(key);
  if (idx < 0) {
    for (size_t i = 0; i < MAX_REMOTE_FIELDS; ++i) {
      if (!g_remote_fields[i].used) {
        idx = static_cast<int>(i);
        g_remote_fields[i].used = true;
        strncpy(g_remote_fields[i].key, key, MAX_KEY_LEN - 1);
        g_remote_fields[i].key[MAX_KEY_LEN - 1] = '\0';
        break;
      }
    }
  }
  if (idx < 0) return -1;

  strncpy(g_remote_fields[idx].value, value, MAX_VAL_LEN - 1);
  g_remote_fields[idx].value[MAX_VAL_LEN - 1] = '\0';
  g_remote_fields[idx].updatedAt = millis();
  return idx;
}

// Callback appelé automatiquement à chaque réception ESP-NOW.
static void trimWhitespace(char *s) {
  if (s == nullptr) return;

  char *start = s;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
    start++;
  }
  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }

  size_t len = strlen(s);
  while (len > 0 &&
         (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\r' || s[len - 1] == '\n' ||
          s[len - 1] == ';')) {
    s[len - 1] = '\0';
    len--;
  }

  if (s[0] == '"') {
    memmove(s, s + 1, strlen(s));
  }

  len = strlen(s);
  while (len > 0 && s[len - 1] == '"') {
    s[len - 1] = '\0';
    len--;
  }
}

static bool setDisplayKey(uint8_t index, const char *key) {
  if (index >= 4 || key == nullptr || key[0] == '\0') {
    return false;
  }

  strncpy(displayKeyStorage[index], key, MAX_KEY_LEN - 1);
  displayKeyStorage[index][MAX_KEY_LEN - 1] = '\0';
  displayKeys[index] = displayKeyStorage[index];

  if (testMode) {
    Serial.print("Display ");
    Serial.print(index);
    Serial.print(" <- ");
    Serial.println(displayKeys[index]);
    Serial.print("Display key present: ");
    Serial.println(findRemoteField(displayKeys[index]) >= 0 ? "yes" : "no");
  }
  return true;
}

static bool applyBleCommand(char *command) {
  if (command == nullptr) {
    return false;
  }

  trimWhitespace(command);
  if (command[0] == '\0') {
    return false;
  }

  if ((command[0] == 'D' || command[0] == 'd') &&
      command[1] >= '0' && command[1] <= '3' &&
      command[2] == '=') {
    char *key = &command[3];
    trimWhitespace(key);
    const uint8_t index = static_cast<uint8_t>(command[1] - '0');
    char *separator = strchr(key, '/');
    if (separator != nullptr) {
      *separator = '\0';
      char *precisionText = separator + 1;
      trimWhitespace(key);
      trimWhitespace(precisionText);
      if (precisionText[0] >= '0' && precisionText[0] <= '3' && precisionText[1] == '\0') {
        displayDecimals[index] = static_cast<uint8_t>(precisionText[0] - '0');
      }
    }

    if (setDisplayKey(index, key)) {
      saveDisplayConfig();
      bleSendLine(String("OK D") + command[1] + "=" + displayKeys[index] + "/" + displayDecimals[index] + "\n");
      return true;
    }
  }

  return false;
}

static void loadDisplayConfig() {
  for (uint8_t i = 0; i < 4; ++i) {
    char keyName[4];
    snprintf(keyName, sizeof(keyName), "k%u", i);

    String storedKey = g_preferences.getString(keyName, "");
    storedKey.trim();
    if (storedKey.length() > 0) {
      setDisplayKey(i, storedKey.c_str());
    }

    char decName[4];
    snprintf(decName, sizeof(decName), "d%u", i);
    const uint8_t decimals = g_preferences.getUChar(decName, displayDecimals[i]);
    displayDecimals[i] = decimals <= 3 ? decimals : displayDecimals[i];
  }
}

static void saveDisplayConfig() {
  for (uint8_t i = 0; i < 4; ++i) {
    char keyName[4];
    snprintf(keyName, sizeof(keyName), "k%u", i);
    g_preferences.putString(keyName, displayKeys[i] != nullptr ? displayKeys[i] : "");

    char decName[4];
    snprintf(decName, sizeof(decName), "d%u", i);
    g_preferences.putUChar(decName, displayDecimals[i] <= 3 ? displayDecimals[i] : 0);
  }
}

static int findWarningIndex(const char *name) {
  if (name == nullptr || name[0] == '\0') {
    return -1;
  }

  for (uint8_t i = 0; i < WARNING_LED_COUNT; ++i) {
    if (warningLedNames[i] != nullptr && strcmp(warningLedNames[i], name) == 0) {
      return i;
    }
  }

  return -1;
}

static void updateErrorLed(bool timeoutActive) {
  constexpr uint32_t kWarningBlinkMs = 250;
  const uint32_t now = millis();

  if (timeoutActive) {
    return;
  }

  if (now - lastWarningBlinkStep >= kWarningBlinkMs) {
    lastWarningBlinkStep = now;
    warningBlinkState = !warningBlinkState;
  }

  for (uint8_t i = 0; i < WARNING_LED_COUNT; ++i) {
    displays.digitalWriteLed(warningLedPins[i], warningLedStates[i] && warningBlinkState);
  }
}

static void clearErrorLeds() {
  displays.setErrorLeds(false);
  errorLedIndex = 0;
  errorLedDirection = 1;
  lastErrorLedStep = 0;
}

static void updateTimeoutDisplayAnimation() {
  constexpr uint32_t kStepMs = 70;
  constexpr uint8_t kFrameCount = 12;

  const uint32_t now = millis();
  if (now - lastTimeoutDisplayStep < kStepMs) {
    return;
  }
  lastTimeoutDisplayStep = now;

  timeoutDisplayFrame++;
  if (timeoutDisplayFrame >= kFrameCount) {
    timeoutDisplayFrame = 0;
  }
}

static void showTimeoutDisplayFrame(uint8_t displayId) {
  constexpr uint8_t SEG_A = 0x01;
  constexpr uint8_t SEG_B = 0x02;
  constexpr uint8_t SEG_C = 0x04;
  constexpr uint8_t SEG_D = 0x08;
  constexpr uint8_t SEG_E = 0x10;
  constexpr uint8_t SEG_F = 0x20;

  uint8_t frame[4] = {0, 0, 0, 0};

  switch (timeoutDisplayFrame) {
    case 0: frame[0] = SEG_A; break;
    case 1: frame[1] = SEG_A; break;
    case 2: frame[2] = SEG_A; break;
    case 3: frame[3] = SEG_A; break;
    case 4: frame[3] = SEG_B; break;
    case 5: frame[3] = SEG_C; break;
    case 6: frame[3] = SEG_D; break;
    case 7: frame[2] = SEG_D; break;
    case 8: frame[1] = SEG_D; break;
    case 9: frame[0] = SEG_D; break;
    case 10: frame[0] = SEG_E; break;
    case 11: frame[0] = SEG_F; break;
    default: frame[0] = SEG_A; break;
  }

  displays.showRaw(displayId, frame);
}

static void clearTimeoutDisplayAnimation() {
  lastTimeoutDisplayStep = 0;
  timeoutDisplayFrame = 0;
  timeoutDisplayIndex = 0;
  if (!displaysAvailable) {
    return;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    displays.clear(i);
  }
}

static bool onEngineField(const char *key, const char *value, void *context) {
  (void)context;
  const int idx = upsertRemoteField(key, value);
  if (idx < 0) return false;
  lastDataReceived = millis();
  clearErrorLeds();
  showingErr = false;
  return true;
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (!data || len <= 0) {
    return;
  }

  const int safeLen = len > static_cast<int>(MAX_ESPNOW_MSG_LEN) ? static_cast<int>(MAX_ESPNOW_MSG_LEN) : len;
  portENTER_CRITICAL(&g_espnow_mux);
  memcpy(g_pending_espnow_data, data, safeLen);
  g_pending_espnow_data[safeLen] = '\0';
  g_pending_espnow_len = safeLen;
  g_pending_espnow_ready = true;
  portEXIT_CRITICAL(&g_espnow_mux);
}

static void processPendingEspNowMessage() {
  char localBuffer[MAX_ESPNOW_MSG_LEN + 1] = {};
  int localLen = 0;

  portENTER_CRITICAL(&g_espnow_mux);
  if (g_pending_espnow_ready) {
    localLen = g_pending_espnow_len;
    memcpy(localBuffer, g_pending_espnow_data, localLen + 1);
    g_pending_espnow_ready = false;
    g_pending_espnow_len = 0;
  }
  portEXIT_CRITICAL(&g_espnow_mux);

  if (localLen <= 0) {
    return;
  }

  if (testMode) {
    Serial.print("ESP-NOW RX payload: ");
    Serial.write(reinterpret_cast<const uint8_t *>(localBuffer), localLen);
    Serial.println();
  }

  parseRemoteMessage(reinterpret_cast<const uint8_t *>(localBuffer), localLen);
  lastDataReceived = millis();
  clearErrorLeds();
  showingErr = false;
}

// Décode un message texte ESP-NOW de la forme "rpm=3500;temp=87;gear=3", puis formate et envoie la sortie.
static void parseRemoteMessage(const uint8_t *data, int len) {
  if (!data || len <= 0 || len > 127) return;
  char buffer[64];
  size_t copy_len = (size_t)len < sizeof(buffer) - 1 ? (size_t)len : sizeof(buffer) - 1;
  memcpy(buffer, data, copy_len);
  buffer[copy_len] = '\0';

  char *saveptr = nullptr;
  char *token = strtok_r(buffer, ";", &saveptr);
  while (token) {
    char *eq = strchr(token, '=');
    if (eq) {
      *eq = '\0';
      char *key = token;
      char *value = eq + 1;
      if (*key && *value) {
        upsertRemoteField(key, value);
      }
    }
    token = strtok_r(nullptr, ";", &saveptr);
  }

  // Formatage et envoi de la sortie après parsing
  static char output[256];
  char *ptr = output;
  size_t remaining = sizeof(output) - 1;

  for (size_t i = 0; i < MAX_REMOTE_FIELDS && remaining > 0; ++i) {
    if (!g_remote_fields[i].used) continue;
    int written = snprintf(ptr, remaining, "%s: %s\n",
                           g_remote_fields[i].key, g_remote_fields[i].value);
    if (written > 0 && (size_t)written < remaining) {
      ptr += written;
      remaining -= written;
    } else {
      break;
    }
  }
  *ptr = '\0';

  if(testMode){
    Serial.print(output);
  }
  //bleSendLine(String(output));
}
