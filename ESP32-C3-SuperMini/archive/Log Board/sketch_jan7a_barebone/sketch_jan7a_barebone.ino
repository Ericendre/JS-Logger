// Ces includes "Network*" sont charges par certaines versions du core ESP32.
// Si ton sketch compile sans eux, tu peux probablement les supprimer.
#include <Network.h>
#include <NetworkClient.h>
#include <NetworkEvents.h>
#include <NetworkInterface.h>
#include <NetworkManager.h>
#include <NetworkServer.h>
#include <NetworkUdp.h>

// Bloc BLE: sert a exposer les donnees comme un "UART Bluetooth".
// A supprimer si tu ne veux pas envoyer les mesures vers un telephone/app BLE.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Bloc WiFi + ESP-NOW: sert a recevoir des donnees d'un autre ESP32.
// A supprimer si tu ne veux pas de communication radio entre cartes.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>


// Sketch de base pour:
// 1) recevoir des infos en ESP-NOW,
// 2) les afficher sur le port serie,
// 3) les envoyer en BLE.

// ================== BLE UART (Nordic UART Service) ==================
// UUID standard du service "Nordic UART Service" souvent utilise pour envoyer
// des lignes de texte entre un ESP32 et une app mobile BLE.
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID RX_CHAR_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID TX_CHAR_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

// Pointeur sur la caracteristique BLE de sortie.
BLECharacteristic *g_tx_char = nullptr;
// Indique si un client BLE est actuellement connecte.
bool g_device_connected = false;

// Stockage de petits couples cle/valeur recus par ESP-NOW.
// Exemple de message distant: "rpm=3500;temp=87"
constexpr size_t MAX_REMOTE_FIELDS = 12;
constexpr size_t MAX_KEY_LEN = 16;
constexpr size_t MAX_VAL_LEN = 20;

struct KeyValue {
  char key[MAX_KEY_LEN];
  char value[MAX_VAL_LEN];
  bool used;
};

KeyValue g_remote_fields[MAX_REMOTE_FIELDS];

// Affiche une adresse MAC proprement dans le moniteur serie.
static void printMac(const uint8_t *mac) {
  if (mac == nullptr) return;
  Serial.printf(
      "%02X:%02X:%02X:%02X:%02X:%02X",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Callbacks BLE: gestion connexion/deconnexion d'un client.
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

// Envoie une ligne en BLE.
// Le decoupage en morceaux evite de depasser la taille supportee par la pile BLE.
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
  // Port serie pour debug et affichage des mesures.
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Initialisation du serveur BLE.
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

  // Initialisation radio en mode station pour pouvoir utiliser ESP-NOW.
  // Le canal WiFi doit generalement correspondre a celui de l'emetteur.
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
}

void loop() {
  // Plus aucune lecture locale a faire.
  // Le sketch attend simplement les messages entrants.
  delay(20);
}

// Cherche si une cle distante existe deja dans le tableau.
static int findRemoteField(const char *key) {
  for (size_t i = 0; i < MAX_REMOTE_FIELDS; ++i) {
    if (g_remote_fields[i].used &&
        strncmp(g_remote_fields[i].key, key, MAX_KEY_LEN) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// Ajoute ou met a jour une valeur distante recue via ESP-NOW.
static int upsertRemoteField(const char *key, const char *value) {
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
  if (idx < 0) {
    return -1;
  }

  strncpy(g_remote_fields[idx].value, value, MAX_VAL_LEN - 1);
  g_remote_fields[idx].value[MAX_VAL_LEN - 1] = '\0';
  return idx;
}

// Retire les espaces/tabulations au debut et a la fin d'un texte.
static void trimWhitespace(char *s) {
  if (s == nullptr) return;
  char *start = s;
  while (*start == ' ' || *start == '\t') start++;
  if (start != s) memmove(s, start, strlen(start) + 1);
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[len - 1] = '\0';
    len--;
  }
}

// Decode un message texte ESP-NOW de la forme:
// "rpm=3500;temp=87;gear=3"
// puis le range dans g_remote_fields pour l'affichage final.
static void parseRemoteMessage(const uint8_t *data, int len) {
  if (data == nullptr || len <= 0) return;
  char buffer[128];
  size_t copy_len = (len < (int)(sizeof(buffer) - 1)) ? (size_t)len : (sizeof(buffer) - 1);
  memcpy(buffer, data, copy_len);
  buffer[copy_len] = '\0';

  char *saveptr = nullptr;
  char *token = strtok_r(buffer, ";", &saveptr);
  while (token != nullptr) {
    char *eq = strchr(token, '=');
    if (eq != nullptr) {
      *eq = '\0';
      char *key = token;
      char *value = eq + 1;
      trimWhitespace(key);
      trimWhitespace(value);
      if (key[0] != '\0' && value[0] != '\0') {
        upsertRemoteField(key, value);
      }
    }
    token = strtok_r(nullptr, ";", &saveptr);
  }
}

// Callback appele automatiquement a chaque reception ESP-NOW.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (info != nullptr) {
    Serial.print("ESP-NOW RX from ");
    printMac(info->src_addr);
    Serial.print(" len=");
    Serial.println(len);
  } else {
    Serial.print("ESP-NOW RX len=");
    Serial.println(len);
  }
  if (data != nullptr && len > 0) {
    Serial.print("ESP-NOW RX payload: ");
    for (int i = 0; i < len; ++i) {
      Serial.write(data[i]);
    }
    Serial.write('\n');
  }
  parseRemoteMessage(data, len);
  envoyerSerie();
}

// Construit la sortie texte finale.
// Actuellement:
// - les champs recus via ESP-NOW
//
// Si tu veux un sketch minimal:
// - garde seulement Serial.print(...)
// - supprime bleSendLine(...) si tu ne gardes pas le BLE
void envoyerSerie() {
  String line;
  line.reserve(200);

  for (size_t i = 0; i < MAX_REMOTE_FIELDS; ++i) {
    if (!g_remote_fields[i].used) continue;
    line += g_remote_fields[i].key;
    line += ": ";
    line += g_remote_fields[i].value;
    line += "\n";
  }

  Serial.print(line);
  bleSendLine(line);
}
