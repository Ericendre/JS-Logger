#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

Adafruit_ADS1115 ads;

const int pwmPin = 0;
const int pwmChannel = 0;
const int pwmFreq = 20000;
const int pwmRes = 10;

const float afrInputTable[] = {
  0.000, 0.125, 0.250, 0.375, 0.500, 0.625, 0.750, 0.875, 1.000, 1.125,
  1.250, 1.375, 1.500, 1.625, 1.750, 1.875, 2.000, 2.125, 2.250, 2.375,
  2.500, 2.625, 2.750, 2.875, 3.000, 3.125, 3.250, 3.375, 3.500, 3.625,
  3.750, 3.875, 4.000, 4.125, 4.250
};

const float afrOutputTable[] = {
  0.905, 0.904, 0.903, 0.902, 0.901, 0.900, 0.890, 0.880, 0.870, 0.860,
  0.850, 0.840, 0.830, 0.820, 0.800, 0.775, 0.750, 0.700, 0.600, 0.410,
  0.240, 0.170, 0.137, 0.113, 0.100, 0.091, 0.083, 0.077, 0.076, 0.075,
  0.074, 0.073, 0.072, 0.071, 0.070
};

const int afrTableSize = sizeof(afrInputTable) / sizeof(afrInputTable[0]);
const uint32_t sendIntervalMs = 100;

// MAC de l'ESP32 affichage
uint8_t receiverMAC[] = {0x10, 0x00, 0x3B, 0xB0, 0x51, 0xC0};
//10:00:3B:B0:51:C0
// paquet de donn??es
typedef struct {
  uint32_t timestamp;
  uint32_t packetID;
  
  float air_temp;
  float air_pressure;
  float oil_pressure;
  float AFR;

  //float rpm;
} SensorPacket;


// instance du paquet
SensorPacket packet;


// compteur de messages envoy??s
uint32_t packetCounter = 0;


// variable indiquant si le peer est connect??
bool peerConnected = false;
char g_last_msg[128];
size_t g_last_msg_len = 0;
uint32_t lastSendMs = 0;

float convertAfrVoltageToNarrowEmu(float inputVoltage);
void updateAfrAnalogOutput(float inputVoltage);

static void printMac(const uint8_t *mac) {
  if (!mac) return;
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void addPeer() // Ajoute le récepteur dans la table ESP-NOW
{
  // Si le peer existe déjà, on ne raffiche pas l'erreur "already exists".
  if (esp_now_is_peer_exist(receiverMAC)) {
    Serial.print("ESP-NOW peer already exists: ");
    printMac(receiverMAC);
    Serial.println();
    peerConnected = true;
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 6;     // canal wifi fixe
  peerInfo.encrypt = false; // pas de chiffrement

  esp_err_t peerErr = esp_now_add_peer(&peerInfo);
  if (peerErr == ESP_OK) {
    Serial.print("ESP-NOW peer added: ");
    printMac(receiverMAC);
    Serial.println();
    peerConnected = true;
  } else {
    Serial.print("ESP-NOW add peer failed: ");
    Serial.print(peerErr);
    Serial.print(" (");
    Serial.print(esp_err_to_name(peerErr));
    Serial.println(")");
    peerConnected = false;
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status)// Permet de savoir si le paquet est bien parti
{
  Serial.print("ESP-NOW TX ");
  if (info != nullptr) {
    Serial.print("to ");
    Serial.printf(
        "%02X:%02X:%02X:%02X:%02X:%02X",
        info->src_addr[0],
        info->src_addr[1],
        info->src_addr[2],
        info->src_addr[3],
        info->src_addr[4],
        info->src_addr[5]);
    Serial.print(" ");
  }

  if(status == ESP_NOW_SEND_SUCCESS)
  {
    Serial.println("OK");
  }
  else
  {
    Serial.print("FAIL ");
    Serial.println(status);
  }

  if(status != ESP_NOW_SEND_SUCCESS)
  {
    // si erreur on consid??re la liaison perdue
    peerConnected = false;
  }

}

void setup(void)
{
  Serial.begin(115200);

  //                                                                ADS1115
  //                                                                -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit =  0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 0.125mV
  //ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.0078125mV
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

   WiFi.mode(WIFI_STA);

  // Force la puissance TX au max (valeur 78 = +78 dBm sur ESP32).
  // Attention : dépassement de normes RF possibles, à utiliser seulement pour débug.
  esp_wifi_set_max_tx_power(40);

  // fixe le canal WiFi (plus stable)
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);


  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed");
    return;
  }

  Serial.println("ESP-NOW init OK");
  esp_now_register_send_cb(onDataSent);

  addPeer();

  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  ledcAttachPin(pwmPin, pwmChannel);
}

void loop(void)
{
  updateSensorData();

  uint32_t now = millis();
  if (now - lastSendMs >= sendIntervalMs) {
    lastSendMs = now;
    sendData();
  }

  if (g_last_msg_len > 0) {
    Serial.write((const uint8_t *)g_last_msg, g_last_msg_len);
    Serial.write('\n');
  }

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float convertAfrVoltageToNarrowEmu(float inputVoltage)
{
  if (inputVoltage <= afrInputTable[0]) {
    return afrOutputTable[0];
  }

  if (inputVoltage >= afrInputTable[afrTableSize - 1]) {
    return afrOutputTable[afrTableSize - 1];
  }

  for (int i = 0; i < afrTableSize - 1; i++) {
    if (inputVoltage >= afrInputTable[i] && inputVoltage <= afrInputTable[i + 1]) {
      float x0 = afrInputTable[i];
      float x1 = afrInputTable[i + 1];
      float y0 = afrOutputTable[i];
      float y1 = afrOutputTable[i + 1];

      return y0 + ((inputVoltage - x0) * (y1 - y0) / (x1 - x0));
    }
  }

  return afrOutputTable[afrTableSize - 1];
}

void updateAfrAnalogOutput(float inputVoltage)
{
  float outputVoltage = convertAfrVoltageToNarrowEmu(inputVoltage);
  int duty = (int)((outputVoltage / 3.3f) * ((1 << pwmRes) - 1) + 0.5f);

  if (duty < 0) {
    duty = 0;
  }

  if (duty > ((1 << pwmRes) - 1)) {
    duty = (1 << pwmRes) - 1;
  }

  ledcWrite(pwmChannel, duty);
}

float voltageToTemperature(float V)
{
  const float Vcc = 5.0;
  const float Rfixed = 2400.0;

  const float R0 = 2500.0;      // r??sistance ?? 20??C
  const float T0 = 293.15;      // 20??C en Kelvin
  const float B = 3950.0;       // coefficient Beta approx

  // calcul r??sistance NTC
  float Rntc = (V * Rfixed) / (Vcc - V);

  // formule Beta
  float tempK = 1.0 / ((1.0/T0) + (1.0/B) * log(Rntc/R0));

  // Kelvin -> Celsius
  float tempC = tempK - 273.15;

  return tempC;
}

void sendData()
{

  // si la liaison est perdue on tente de reconnecter
  if(!peerConnected)
  {
    addPeer();
  }


  // horodatage
  packet.timestamp = millis();

  // num??ro de paquet
  packet.packetID = packetCounter++;

  // envoi du paquet (format texte avec labels)
  char msg[128];
  int n = snprintf(
      msg,
      sizeof(msg),
      "air_temp=%.2f;air_pressure=%.2f;oil_pressure=%.2f;AFR=%.2f",
      packet.air_temp,
      packet.air_pressure,
      packet.oil_pressure,
      packet.AFR);
  if (n <= 0) {
    return;
  }
  size_t len = (n < (int)sizeof(msg)) ? (size_t)n : (sizeof(msg) - 1);
  esp_err_t err = esp_now_send(receiverMAC, (uint8_t *)msg, len);

  if (err != ESP_OK) {
    Serial.print("ESP-NOW send failed: ");
    Serial.print(err);
    Serial.print(" (");
    Serial.print(esp_err_to_name(err));
    Serial.println(")");
  }
  memcpy(g_last_msg, msg, len);
  g_last_msg_len = len;

}

void updateSensorData()
{
  // ici on met ?? jour les donn??es du paquet avec les mesures r??elles
  // par exemple :
  float afrVoltage = ads.computeVolts(ads.readADC_SingleEnded(0));

  packet.air_temp = voltageToTemperature(ads.computeVolts(ads.readADC_SingleEnded(3)));
  packet.air_pressure = mapfloat(ads.computeVolts(ads.readADC_SingleEnded(2)), 0.4, 4.65, 0.2, 3.0) - 1.0;
  packet.oil_pressure = mapfloat(ads.computeVolts(ads.readADC_SingleEnded(1)), 0.5, 4.5, 0, 7.0);
  packet.AFR = mapfloat(afrVoltage, 0.5, 4.25, 11.0, 18.5);

  updateAfrAnalogOutput(afrVoltage);
}
