/*
  Récepteur ESP-NOW pour MeteoGPS_V4_ESPNOW
  ESP32-C3 Supermini + LCD 2004 I2C (plus besoin de module LoRa)

  Trames acceptées :
  MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats,vent,dir   (station avec vent)
  MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats            (station sans vent)

  Core Arduino ESP32 3.x recommandé (RSSI disponible).
*/

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

/* ---------- ESP-NOW : mêmes réglages que l'émetteur ---------- */
#define ESPNOW_CHANNEL 1
#define USE_LONG_RANGE 1

/* ---------- LCD ---------- */
hd44780_I2Cexp disp(0x27, 20, 4);

/* ---------- Timeout ---------- */
const unsigned long SIGNAL_TIMEOUT = 45000;   // 3 trames manquées

/* ---------- Tampon partagé avec le callback ---------- */
// Le callback ESP-NOW tourne dans la tâche Wi-Fi : on y fait juste une copie,
// tout le traitement (parsing, LCD) se fait dans loop().
portMUX_TYPE   rxMux = portMUX_INITIALIZER_UNLOCKED;
char           rxBuf[251];
volatile bool  rxReady = false;
volatile int   rxRssi  = 0;

/* ---------- Données reçues ---------- */
struct MeteoData {
  long  cpt;
  int   qnh, temp, hum, dew, meteo;
  bool  hasFix;
  float lat, lon;
  int   alt, sats;
  bool  hasSpeed;
  float windMs;
  int   windDir;     // indice 0-15, -1 si inconnu
  int   rssi;
};

const char* const CARDINAUX[16] = {
  "N","NNE","NE","ENE","E","ESE","SE","SSE",
  "S","SSO","SO","OSO","O","ONO","NO","NNO"
};

bool          hasData   = false;
bool          lostShown = false;
unsigned long tLastRx   = 0;
long          prevCpt   = -1;
unsigned long lostCount = 0;

/* ---------- Callback de réception ---------- */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  int rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
#else
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  int rssi = 0;   // RSSI non fourni par le core 2.x
#endif
  if (len <= 0) return;
  if (len > (int)sizeof(rxBuf) - 1) len = sizeof(rxBuf) - 1;

  portENTER_CRITICAL(&rxMux);
  memcpy(rxBuf, data, len);
  rxBuf[len] = '\0';
  rxRssi  = rssi;
  rxReady = true;
  portEXIT_CRITICAL(&rxMux);
}

/* ---------- Libellés météo ---------- */
const char* meteoLabel(int m) {
  switch (m) {
    case 0: return "CLAIR";
    case 1: return "BRUME";
    case 2: return "PLUIE";
    case 3: return "VOL OK";
    case 4: return "HUMIDE";
    default: return "?";
  }
}

/* ---------- Découpage CSV qui conserve les champs vides ---------- */
int splitCSV(char* s, char* fields[], int maxFields) {
  int n = 0;
  fields[n++] = s;
  while (*s && n < maxFields) {
    if (*s == ',') {
      *s = '\0';
      fields[n++] = s + 1;
    }
    s++;
  }
  return n;
}

bool parseFrame(char* buf, MeteoData& d) {
  char* f[14];
  int n = splitCSV(buf, f, 14);
  if ((n != 11 && n != 13) || strcmp(f[0], "MTO") != 0) return false;

  d.cpt   = atol(f[1]);
  d.qnh   = atoi(f[2]);
  d.temp  = atoi(f[3]);
  d.hum   = atoi(f[4]);
  d.dew   = atoi(f[5]);
  d.meteo = atoi(f[6]);
  d.hasFix = (f[7][0] != '\0');
  d.lat  = d.hasFix ? atof(f[7]) : 0;
  d.lon  = d.hasFix ? atof(f[8]) : 0;
  d.alt  = d.hasFix ? atoi(f[9]) : 0;
  d.sats = atoi(f[10]);

  d.hasSpeed = (n == 13 && f[11][0] != '\0');
  d.windMs   = d.hasSpeed ? atof(f[11]) : 0;
  d.windDir  = (n == 13 && f[12][0] != '\0') ? atoi(f[12]) : -1;
  if (d.windDir > 15) d.windDir = -1;
  return true;
}

/* ---------- LCD ---------- */
void lcdLine(uint8_t row, const char* txt) {
  char line[21];
  snprintf(line, sizeof(line), "%-20s", txt);
  disp.setCursor(0, row);
  disp.print(line);
}

void showData(const MeteoData& d) {
  char l[32];
  snprintf(l, sizeof(l), "QNH %4d  T %3d%cC", d.qnh, d.temp, (char)223);
  lcdLine(0, l);

  snprintf(l, sizeof(l), "HR%3d%% Td%3d %s", d.hum, d.dew, meteoLabel(d.meteo));
  lcdLine(1, l);

  char sVent[8] = " --.-";
  if (d.hasSpeed) snprintf(sVent, sizeof(sVent), "%5.1f", d.windMs * 3.6f);
  snprintf(l, sizeof(l), "Vent %s km/h %s", sVent, d.windDir >= 0 ? CARDINAUX[d.windDir] : "--");
  lcdLine(2, l);

  // Alt/sats et RSSI partagent la dernière ligne pour laisser la place au vent
  if (d.hasFix) snprintf(l, sizeof(l), "%4dm S%2d R%4d P%lu", d.alt, d.sats, d.rssi, lostCount);
  else          snprintf(l, sizeof(l), "No fix R%4d P%lu", d.rssi, lostCount);
  lcdLine(3, l);
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  disp.init();
  disp.backlight();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
#if USE_LONG_RANGE
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
#endif
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Adresse MAC à recopier dans l'émetteur si tu veux un envoi ciblé
  String mac = WiFi.macAddress();
  Serial.print(F("MAC recepteur : "));
  Serial.println(mac);

  if (esp_now_init() != ESP_OK) {
    lcdLine(0, "ESP-NOW: ERREUR");
    Serial.println(F("ESP-NOW : init echouee"));
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  lcdLine(0, "Recepteur ESP-NOW");
  lcdLine(1, USE_LONG_RANGE ? "Canal 1  mode LR" : "Canal 1");
  lcdLine(2, mac.c_str());
  lcdLine(3, "En attente...");
  Serial.println(F("Recepteur pret"));
}

/* ---------- LOOP ---------- */
void loop() {
  if (rxReady) {
    char buf[251];
    int  rssi;

    portENTER_CRITICAL(&rxMux);
    memcpy(buf, rxBuf, sizeof(buf));
    rssi    = rxRssi;
    rxReady = false;
    portEXIT_CRITICAL(&rxMux);

    Serial.print(F("RX ["));
    Serial.print(rssi);
    Serial.print(F(" dBm] "));
    Serial.println(buf);

    MeteoData d;
    if (parseFrame(buf, d)) {
      d.rssi = rssi;
      if (prevCpt >= 0 && d.cpt > prevCpt + 1) lostCount += d.cpt - prevCpt - 1;
      prevCpt = d.cpt;

      hasData   = true;
      lostShown = false;
      tLastRx   = millis();
      showData(d);
    } else {
      Serial.println(F("  -> trame ignoree (format inconnu)"));
    }
  }

  if (hasData && !lostShown && millis() - tLastRx > SIGNAL_TIMEOUT) {
    lcdLine(3, "!! PERTE SIGNAL !!");
    lostShown = true;
    Serial.println(F("Perte de signal"));
  }
}
