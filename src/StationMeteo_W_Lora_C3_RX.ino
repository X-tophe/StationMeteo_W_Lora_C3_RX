/*
  Passerelle ESP-NOW -> réseau local (box Wi-Fi) pour MeteoGPS_V4_ESPNOW
  ESP32-C3 Supermini + LCD 2004 I2C

  - Se connecte à la box Wi-Fi (mode station)
  - Reçoit les trames ESP-NOW de la station (mode LR accepté)
  - Affiche les données sur le LCD
  - Sert une page web sur le réseau local :
        http://meteo.local/        (ou http://<IP affichée sur le LCD>/)
        http://meteo.local/json    (données brutes JSON, pour domotique...)

  IMPORTANT - canal radio :
  Une fois connecté à la box, l'ESP32 est obligé d'utiliser le CANAL DE LA BOX
  pour l'ESP-NOW. L'émetteur doit donc émettre sur ce même canal (il est
  affiché au démarrage sur le LCD et le moniteur série). Conseil : fixer le
  canal dans les réglages de la box (pas de canal "auto"), sinon la liaison
  sera perdue le jour où la box change de canal.

  Trames acceptées :
  MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats,vent,dir   (station avec vent)
  MTO,cpt,QNH,T,HR,Td,meteo,lat,lon,alt,sats            (station sans vent)

  Core Arduino ESP32 3.x recommandé.
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

/* ---------- CONFIG WI-FI ---------- */
#define WIFI_SSID     "NomDeTaBox"
#define WIFI_PASSWORD "MotDePasseWiFi"
#define HOSTNAME      "meteo"           // -> http://meteo.local
// Certains ESP32-C3 Supermini se connectent mal à pleine puissance
// (antenne mal adaptée). En cas d'échec de connexion, essayer 8.5 dBm.
#define WIFI_TX_POWER WIFI_POWER_19_5dBm

/* ---------- LCD ---------- */
hd44780_I2Cexp disp(0x27, 20, 4);

/* ---------- Serveur web ---------- */
WebServer server(80);

/* ---------- Timeout ---------- */
const unsigned long SIGNAL_TIMEOUT = 45000;   // 3 trames manquées

/* ---------- Tampon partagé avec le callback ---------- */
// Le callback ESP-NOW tourne dans la tâche Wi-Fi : on y fait juste une copie,
// tout le traitement (parsing, LCD, web) se fait dans loop().
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

MeteoData     last;
bool          hasData   = false;
bool          lostShown = false;
unsigned long tLastRx   = 0;
long          prevCpt   = -1;
unsigned long lostCount = 0;
unsigned long rxCount   = 0;

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

void showDataLine3(const MeteoData& d) {
  char l[32];
  if (d.hasFix) snprintf(l, sizeof(l), "%4dm S%2d R%4d P%lu", d.alt, d.sats, d.rssi, lostCount);
  else          snprintf(l, sizeof(l), "No fix R%4d P%lu", d.rssi, lostCount);
  lcdLine(3, l);
}

void showIpLine3() {
  char l[32];
  if (WiFi.status() == WL_CONNECTED)
    snprintf(l, sizeof(l), "IP %s", WiFi.localIP().toString().c_str());
  else
    snprintf(l, sizeof(l), "WiFi: hors ligne");
  lcdLine(3, l);
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

  showDataLine3(d);
}

/* ---------- WEB : JSON ---------- */
void handleJson() {
  char sWind[12] = "null", sDir[8] = "null", sDeg[8] = "null";
  char sLat[16] = "null", sLon[16] = "null", sAlt[8] = "null";

  if (hasData && last.hasSpeed) snprintf(sWind, sizeof(sWind), "%.1f", last.windMs * 3.6f);
  if (hasData && last.windDir >= 0) {
    snprintf(sDir, sizeof(sDir), "\"%s\"", CARDINAUX[last.windDir]);
    snprintf(sDeg, sizeof(sDeg), "%.1f", last.windDir * 22.5f);
  }
  if (hasData && last.hasFix) {
    snprintf(sLat, sizeof(sLat), "%.5f", last.lat);
    snprintf(sLon, sizeof(sLon), "%.5f", last.lon);
    snprintf(sAlt, sizeof(sAlt), "%d", last.alt);
  }

  long age = hasData ? (long)((millis() - tLastRx) / 1000) : -1;
  bool online = hasData && (millis() - tLastRx) <= SIGNAL_TIMEOUT;

  char json[600];
  snprintf(json, sizeof(json),
    "{\"data\":%s,\"online\":%s,\"age_s\":%ld,\"cpt\":%ld,"
    "\"qnh\":%d,\"temp\":%d,\"hum\":%d,\"dew\":%d,"
    "\"meteo\":%d,\"meteo_txt\":\"%s\","
    "\"wind_kmh\":%s,\"wind_dir\":%s,\"wind_deg\":%s,"
    "\"fix\":%s,\"lat\":%s,\"lon\":%s,\"alt\":%s,\"sats\":%d,"
    "\"rssi\":%d,\"rx\":%lu,\"lost\":%lu,\"channel\":%d,\"wifi_rssi\":%d}",
    hasData ? "true" : "false", online ? "true" : "false", age,
    hasData ? last.cpt : -1L,
    last.qnh, last.temp, last.hum, last.dew,
    last.meteo, hasData ? meteoLabel(last.meteo) : "",
    sWind, sDir, sDeg,
    (hasData && last.hasFix) ? "true" : "false", sLat, sLon, sAlt, last.sats,
    last.rssi, rxCount, lostCount, WiFi.channel(), (int)WiFi.RSSI());

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

/* ---------- WEB : page HTML ---------- */
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="fr"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Station météo</title>
<style>
:root{--bg:#f4f6f8;--card:#fff;--txt:#1d2733;--mut:#6b7785;--acc:#1f6feb;--bad:#d1242f}
@media(prefers-color-scheme:dark){:root{--bg:#0f1419;--card:#1a2129;--txt:#e6edf3;--mut:#8b98a5;--acc:#58a6ff;--bad:#ff7b72}}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,sans-serif;background:var(--bg);color:var(--txt)}
main{max-width:720px;margin:auto;padding:16px}
h1{font-size:1.3rem;margin:0 0 4px}#st{color:var(--mut);font-size:.9rem;margin-bottom:16px}
#st.off{color:var(--bad);font-weight:600}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
.c{background:var(--card);border-radius:12px;padding:14px;box-shadow:0 1px 3px #0002}
.l{color:var(--mut);font-size:.8rem;text-transform:uppercase;letter-spacing:.04em}
.v{font-size:1.9rem;font-weight:600;margin-top:4px}.u{font-size:1rem;color:var(--mut);font-weight:400}
#arr{display:inline-block;transition:transform .5s;color:var(--acc)}
.s{font-size:.85rem;color:var(--mut);margin-top:16px;line-height:1.6}
</style></head><body><main>
<h1>Station météo</h1><div id="st">Chargement…</div>
<div class="g">
<div class="c"><div class="l">Température</div><div class="v"><span id="t">--</span><span class="u"> °C</span></div></div>
<div class="c"><div class="l">QNH</div><div class="v"><span id="p">--</span><span class="u"> hPa</span></div></div>
<div class="c"><div class="l">Humidité</div><div class="v"><span id="h">--</span><span class="u"> %</span></div></div>
<div class="c"><div class="l">Point de rosée</div><div class="v"><span id="d">--</span><span class="u"> °C</span></div></div>
<div class="c"><div class="l">Vent</div><div class="v"><span id="w">--</span><span class="u"> km/h</span></div></div>
<div class="c"><div class="l">Direction</div><div class="v"><span id="arr">↓</span> <span id="wd">--</span></div></div>
<div class="c"><div class="l">Météo</div><div class="v" id="m">--</div></div>
</div>
<div class="s" id="x"></div>
</main><script>
const $=i=>document.getElementById(i);
async function up(){
 try{
  const j=await (await fetch('/json')).json();
  if(!j.data){$('st').textContent='En attente de la station…';return;}
  $('t').textContent=j.temp;$('p').textContent=j.qnh;$('h').textContent=j.hum;$('d').textContent=j.dew;
  $('w').textContent=j.wind_kmh===null?'--':j.wind_kmh.toFixed(1);
  $('wd').textContent=j.wind_dir===null?'--':j.wind_dir;
  if(j.wind_deg!==null)$('arr').style.transform='rotate('+j.wind_deg+'deg)';
  $('m').textContent=j.meteo_txt;
  $('st').textContent=j.online?'Dernière trame il y a '+j.age_s+' s':'PERTE SIGNAL depuis '+j.age_s+' s';
  $('st').className=j.online?'':'off';
  $('x').innerHTML='GPS : '+(j.fix?j.lat+', '+j.lon+' · '+j.alt+' m · '+j.sats+' sat':'pas de fix')+
   '<br>Liaison station : '+j.rssi+' dBm · trames reçues '+j.rx+' · perdues '+j.lost+
   '<br>Wi-Fi box : canal '+j.channel+' · '+j.wifi_rssi+' dBm · <a href="/json">JSON</a>';
 }catch(e){$('st').textContent='Passerelle injoignable';$('st').className='off';}
}
up();setInterval(up,5000);
</script></body></html>)HTML";

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}

/* ---------- WI-FI ---------- */
void setupWiFi() {
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // indispensable : sinon des trames ESP-NOW sont ratées
  WiFi.setAutoReconnect(true);
  // B/G/N pour la box + LR pour recevoir la station en longue portée
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);

  lcdLine(0, "Connexion WiFi...");
  lcdLine(1, WIFI_SSID);
  Serial.print(F("Connexion a "));
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_TX_POWER);

  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("WiFi OK  IP : "));
    Serial.print(WiFi.localIP());
    Serial.print(F("  canal : "));
    Serial.println(WiFi.channel());
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.println(F("http://" HOSTNAME ".local"));
    }
  } else {
    Serial.println(F("WiFi : echec (nouvel essai automatique en arriere-plan)"));
  }
}

/* ---------- SETUP ---------- */
void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin();
  disp.init();
  disp.backlight();

  setupWiFi();

  Serial.print(F("MAC passerelle : "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    lcdLine(0, "ESP-NOW: ERREUR");
    Serial.println(F("ESP-NOW : init echouee"));
    while (1) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();

  char l[32];
  lcdLine(0, "Passerelle ESP-NOW");
  snprintf(l, sizeof(l), "Canal box : %d", WiFi.channel());
  lcdLine(1, l);
  lcdLine(2, HOSTNAME ".local");
  showIpLine3();
  Serial.print(F("Regler l'emetteur sur le canal "));
  Serial.println(WiFi.channel());
}

/* ---------- LOOP ---------- */
void loop() {
  server.handleClient();

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

      last      = d;
      hasData   = true;
      lostShown = false;
      tLastRx   = millis();
      rxCount++;
      showData(d);
    } else {
      Serial.println(F("  -> trame ignoree (format inconnu)"));
    }
  }

  // Ligne 4 du LCD : alterne données / adresse IP toutes les 5 s
  static unsigned long tAlt = 0;
  static bool showIp = false;
  if (hasData && !lostShown && millis() - tAlt > 5000) {
    tAlt = millis();
    showIp = !showIp;
    if (showIp) showIpLine3(); else showDataLine3(last);
  }

  if (hasData && !lostShown && millis() - tLastRx > SIGNAL_TIMEOUT) {
    lcdLine(3, "!! PERTE SIGNAL !!");
    lostShown = true;
    Serial.println(F("Perte de signal"));
  }
}
