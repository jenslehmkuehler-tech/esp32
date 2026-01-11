#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

/* ================== WLAN ================== */
const char* WIFI_SSID  = "Hello WWW";
const char* WIFI_PASS  = "affe1234";

/* ================== INVERTER KONFIGURATION ================== */
#define MAX_INVERTERS 4

struct InverterConfig {
  const char* name;
  const char* displayName;
  const char* ssid;
  const char* password;
  const char* ip;
  bool enabled;
};

InverterConfig inverters[MAX_INVERTERS] = {
  {"inverter1", "Inverter 1", "Delta-O5720400067WN", "affe1234", "192.168.50.1", true},
  {"inverter2", "Inverter 2", "Delta-O5P20B01756WM", "affe1234", "192.168.50.1", true},
  {"inverter3", "Inverter 3", "Delta-O5720400067WN", "affe1234", "192.168.50.1", false},
  {"inverter4", "Inverter 4", "Delta-O5720400067WN", "affe1234", "192.168.50.1", false}
};

const uint16_t DELTA_PORT = 502;

/* ================== MQTT ================== */
const char* MQTT_SERVER = "192.168.1.178";
const int   MQTT_PORT   = 1883;
const char* MQTT_USER   = "esp32";
const char* MQTT_PASS   = "esp1234";

/* ================== OBJECTS ================== */
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

/* ================== STATE ================== */
bool deltaTrigger = false; // reserved for future use
int  activeInverterIndex = -1;
char statusText[64] = "Idle";

/* ================== DATA ================== */
struct InverterData {
  float inV1, inI1, inP1;
  float inV2, inI2, inP2;
  float outV, outI, outP, outF;
  float powerDay;
  bool hasData;
};

InverterData inverterData[MAX_INVERTERS];

/* ================== LOG (ring buffer, fixed-size) ================== */
#define LOG_SIZE 120
#define LOG_LINE_LEN 128
static char logBuf[LOG_SIZE][LOG_LINE_LEN];
static int logHead = 0; // next write index
static int logCount = 0; // number of stored entries

unsigned long startMillis = 0;

void logMsg(const char *fmt, ...){
  va_list ap;
  va_start(ap, fmt);
  char tmp[LOG_LINE_LEN-32];
  vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);

  unsigned long s = (millis()-startMillis)/1000;
  snprintf(logBuf[logHead], LOG_LINE_LEN, "[%lus] %s", s, tmp);
  Serial.println(logBuf[logHead]);
  logHead = (logHead + 1) % LOG_SIZE;
  if(logCount < LOG_SIZE) logCount++;
}

/* helper to retrieve logs (oldest-first) */
int getLogEntries(char out[][LOG_LINE_LEN], int maxEntries){
  int c = min(logCount, maxEntries);
  int oldest = (logHead - logCount + LOG_SIZE) % LOG_SIZE;
  for(int i=0;i<c;i++){
    strncpy(out[i], logBuf[(oldest + i) % LOG_SIZE], LOG_LINE_LEN-1);
    out[i][LOG_LINE_LEN-1] = '\0';
  }
  return c;
}

/* ================== WIFI ================== */
uint32_t lastWifiAttempt = 0;
const uint32_t WIFI_RETRY_INTERVAL = 5000; // ms

bool connectWiFiOnce(const char* ssid,const char* pass){
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,pass);

  uint32_t t = millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t<3000){
    ArduinoOTA.handle();
    delay(5);
  }
  if(WiFi.status()==WL_CONNECTED){
    logMsg("WLAN verbunden: %s IP %s", ssid, WiFi.localIP().toString().c_str());
    return true;
  }
  logMsg("WLAN Verbindung fehlgeschlagen: %s", ssid);
  return false;
}

void ensureWiFiConnected(){
  if(WiFi.status()==WL_CONNECTED) return;
  if(millis() - lastWifiAttempt < WIFI_RETRY_INTERVAL) return;
  lastWifiAttempt = millis();
  // try to connect to configured AP
  connectWiFiOnce(WIFI_SSID, WIFI_PASS);
}

void disconnectWiFi(){
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

/* ================== MQTT DISCOVERY (ArduinoJson) ================== */
String makeDiscoveryTopic(const char* inverterName, const char* uid){
  String topic = "homeassistant/sensor/esp32_" + String(inverterName) + "/" + String(uid) + "/config";
  return topic;
}

void sendDiscovery(
  const char* inverterName,
  const char* displayName,
  const char* name,
  const char* topic,
  const char* unit,
  const char* devClass,
  const char* uid,
  const char* stateClass
){
  StaticJsonDocument<512> doc;

  doc["name"] = String(displayName) + " " + String(name);
  doc["state_topic"] = topic;
  doc["availability_topic"] = "esp32/availability";
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  doc["unit_of_meas"] = unit;
  doc["device_class"] = devClass;
  doc["state_class"] = stateClass;
  doc["unique_id"] = String(inverterName) + "_" + String(uid);

  JsonObject device = doc.createNestedObject("device");
  device["identifiers"] = JsonArray();
  device["identifiers"][0] = String("esp32_") + String(inverterName);
  device["name"] = String("ESP32 ") + String(displayName);
  device["model"] = "Delta Solar Inverter";
  device["manufacturer"] = "Custom";

  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  String topicStr = makeDiscoveryTopic(inverterName, uid);
  if(!mqttClient.publish(topicStr.c_str(), buf, true)){
    logMsg("MQTT Discovery publish failed: %s_%s", inverterName, uid);
  }
}

void sendAllDiscovery(){
  for(int i=0;i<MAX_INVERTERS;i++){
    if(!inverters[i].enabled) continue;
    String base = "esp32/" + String(inverters[i].name);

    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH1 Voltage", (base+"/inV1/state").c_str(),"V","voltage","inV1","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH1 Current", (base+"/inI1/state").c_str(),"A","current","inI1","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH1 Power",   (base+"/inP1/state").c_str(),"kW","power","inP1","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH2 Voltage", (base+"/inV2/state").c_str(),"V","voltage","inV2","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH2 Current", (base+"/inI2/state").c_str(),"A","current","inI2","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Input CH2 Power",   (base+"/inP2/state").c_str(),"kW","power","inP2","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Output Voltage",    (base+"/outV/state").c_str(),"V","voltage","outV","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Output Current",    (base+"/outI/state").c_str(),"A","current","outI","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Output Power",      (base+"/outP/state").c_str(),"kW","power","outP","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Frequency",         (base+"/outF/state").c_str(),"Hz","frequency","outF","measurement");
    sendDiscovery(inverters[i].name,inverters[i].displayName,"Energy Today",      (base+"/powerDay/state").c_str(),"kWh","energy","powerDay","total_increasing");
  }
}

/* ================== MQTT ================== */
uint32_t lastMqttAttempt = 0;
const uint32_t MQTT_RETRY_INTERVAL = 5000; // ms

String makeClientId(){
  String mac = WiFi.macAddress();
  mac.replace(':','');
  return String("ESP32_SOLAR_") + mac;
}

void mqttCallback(char* topic, byte* payload, unsigned int length){
  // Handle incoming MQTT messages here if needed
}

void reconnectMQTTOnce(){
  if(mqttClient.connected()) return;
  if(millis() - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;
  lastMqttAttempt = millis();

  logMsg("MQTT verbinden...");
  String clientId = makeClientId();
  mqttClient.setBufferSize(3072); // set before connect
  mqttClient.setCallback(mqttCallback);
  if(mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, "esp32/availability", 0, true, "offline")){
    mqttClient.publish("esp32/availability","online",true);
    sendAllDiscovery();
    logMsg("MQTT verbunden als %s", clientId.c_str());
  } else {
    logMsg("MQTT Verbindung failed, rc=%d", mqttClient.state());
  }
}

/* ================== POLLING / PUBLISHING ================== */
uint32_t lastPoll = 0;
const uint32_t POLL_INTERVAL = 5000; // ms

// Placeholder: implement Modbus-TCP polling here. For now we simulate values.
void readInverterData(int idx){
  // If you want to add Modbus-TCP: use a Modbus TCP client library (ArduinoModbus, ModbusIP, etc.)
  // Connect to inverters[idx].ip:DELTA_PORT and read relevant registers, then scale to units.
  
  // Simulation (keeps last values stable if already has data)
  if(!inverterData[idx].hasData){
    inverterData[idx].inV1 = 230.0;
    inverterData[idx].inI1 = 0.5;
    inverterData[idx].inP1 = 0.115;
    inverterData[idx].inV2 = 0.0; inverterData[idx].inI2 = 0.0; inverterData[idx].inP2 = 0.0;
    inverterData[idx].outV = 230.0; inverterData[idx].outI = 0.5; inverterData[idx].outP = 0.115; inverterData[idx].outF = 50.0;
    inverterData[idx].powerDay = 1.23;
    inverterData[idx].hasData = true;
  } else {
    // small random jitter
    inverterData[idx].inP1 += ((rand() % 21) - 10) / 1000.0;
    inverterData[idx].outP += ((rand() % 21) - 10) / 1000.0;
    inverterData[idx].powerDay += ((rand() % 3) / 10000.0);
  }
}

void publishInverterData(int idx){
  if(!inverters[idx].enabled) return;
  String base = "esp32/" + String(inverters[idx].name);

  char buf[128];
  // publish numeric values as strings - Home Assistant will parse
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inV1); mqttClient.publish((base+"/inV1/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inI1); mqttClient.publish((base+"/inI1/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inP1); mqttClient.publish((base+"/inP1/state").c_str(), buf, true);

  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inV2); mqttClient.publish((base+"/inV2/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inI2); mqttClient.publish((base+"/inI2/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].inP2); mqttClient.publish((base+"/inP2/state").c_str(), buf, true);

  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].outV); mqttClient.publish((base+"/outV/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].outI); mqttClient.publish((base+"/outI/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].outP); mqttClient.publish((base+"/outP/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].outF); mqttClient.publish((base+"/outF/state").c_str(), buf, true);
  snprintf(buf, sizeof(buf), "%.3f", inverterData[idx].powerDay); mqttClient.publish((base+"/powerDay/state").c_str(), buf, true);
}

/* ================== SETUP / LOOP ================== */
void setup(){
  Serial.begin(115200);
  startMillis = millis();

  for(int i=0;i<MAX_INVERTERS;i++) inverterData[i].hasData=false;

  // try initial connect
  connectWiFiOnce(WIFI_SSID,WIFI_PASS);

  mqttClient.setServer(MQTT_SERVER,MQTT_PORT);

  ArduinoOTA.setHostname("esp32-solar");
  ArduinoOTA.setPassword("affe1234");
  ArduinoOTA.begin();

  server.on("/logs", [](){
    char out[LOG_SIZE][LOG_LINE_LEN];
    int c = getLogEntries(out, LOG_SIZE);
    String payload = "";
    for(int i=0;i<c;i++){
      payload += String(out[i]) + "\n";
    }
    server.send(200, "text/plain", payload);
  });

  server.begin();
  logMsg("ESP32 Solar Logger gestartet");
}

void loop(){
  server.handleClient();
  ArduinoOTA.handle();

  ensureWiFiConnected();
  reconnectMQTTOnce();

  if(mqttClient.connected()) mqttClient.loop();

  // Polling and publishing
  if(millis() - lastPoll >= POLL_INTERVAL){
    lastPoll = millis();
    for(int i=0;i<MAX_INVERTERS;i++){
      if(!inverters[i].enabled) continue;
      readInverterData(i);
      if(mqttClient.connected()) publishInverterData(i);
    }
  }
}

// End of sketch
