/**
 * ESP32 BambuAFE — Cloud MQTT Edition
 * -------------------------------------
 * On first boot (no config.json found):
 *   - Starts a Wi-Fi Access Point named "BambuAFE-Setup"
 *   - Hosts a web config portal at 192.168.4.1  (setup.html)
 *   - Saves submitted settings to /config.json in LittleFS
 *   - Reboots into normal operation
 *
 * On subsequent boots:
 *   - Reads config.json
 *   - Connects to Wi-Fi
 *   - Connects to Bambu Cloud MQTT using stored token
 *   - Connects to Bambu Cloud MQTT broker (us.mqtt.bambulab.com:8883)
 *   - Subscribes to both printer report topics
 *   - Parses print state and exhaust fan status to drive a shared PWM fan
 *   - Serves a password-protected status dashboard at / (index.html)
 *
 * To reset config: hold BOOT button (GPIO 0) for 3 seconds on startup
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>

// ─── Pin & timing constants ────────────────────────────────────────────────
#define RESET_BUTTON_PIN   0
#define RESET_HOLD_MS      3000
#define AP_SSID            "BambuAFE-Setup"
#define AP_PASSWORD        ""
#define CONFIG_FILE        "/config.json"
#define FIRMWARE_VERSION   "0.9.0"

#define FAN1_PIN           16
#define FAN2_PIN           17
#define PWM_FREQ           25000
#define PWM_RESOLUTION     8
#define PWM_CHANNEL_1      0
#define PWM_CHANNEL_2      1

#define BAMBU_CLOUD_PORT   8883
#define MQTT_RECONNECT_MS  15000

// ─── Global objects ────────────────────────────────────────────────────────
WebServer        server(80);
WiFiClientSecure tlsClient;
PubSubClient     mqttClient(tlsClient);

// ─── Config struct ─────────────────────────────────────────────────────────
struct Config {
  char wifi_ssid[64]      = "";
  char wifi_pass[64]      = "";
  char device_name[64]    = "BambuAFE-ESP32";
  char dashboard_pass[64] = "admin";
  char bambu_region[8]    = "us";       // us or cn
  char bambu_user_id[64]  = "";         // set via config page or PowerShell script
  char bambu_token[512]   = "";         // set via config page or PowerShell script
  char printer1_name[64]  = "";
  char printer1_serial[32]= "";
  int  printer1_gen       = 1;   // 1 = Gen 1 (X1C/P1S/A1), 2 = Gen 2 (H2C/H2S/H2D)
  char printer2_name[64]  = "";
  char printer2_serial[32]= "";
  int  printer2_gen       = 1;
  int  fan_speed_printing        = 10;
  int  fan_speed_one_exhausting  = 50;
  int  fan_speed_both_exhausting = 100;
};

Config cfg;

// ─── Printer runtime state ─────────────────────────────────────────────────
struct PrinterState {
  char  name[64]          = "";
  int   gen               = 1;   // 1 = Gen 1, 2 = Gen 2
  char  gcode_state[32]   = "offline";
  int   progress          = 0;
  int   remaining_min     = 0;
  float nozzle_temp       = 0.0f;
  float bed_temp          = 0.0f;
  float chamber_temp      = 0.0f;
  bool  exhausting        = false;
  int   exhaust_fan_speed = 0;
  bool  connected         = false;
  bool  pushall_sent      = false;
  unsigned long last_seen = 0;
};

PrinterState p1state;
PrinterState p2state;
int currentFanPct = 0;

// ══════════════════════════════════════════════════════════════════════════
//  PWM fan control
// ══════════════════════════════════════════════════════════════════════════

void initFans() {
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN1_PIN, PWM_CHANNEL_1);
  ledcAttachPin(FAN2_PIN, PWM_CHANNEL_2);
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
}

void setFanSpeed(int pct) {
  pct = constrain(pct, 0, 100);
  if (pct == currentFanPct) return;
  currentFanPct = pct;
  int duty = map(pct, 0, 100, 0, 255);
  ledcWrite(PWM_CHANNEL_1, duty);
  ledcWrite(PWM_CHANNEL_2, duty);
  Serial.printf("[Fan] Speed set to %d%% (duty %d)\n", pct, duty);
}

void updateFanSpeed() {
  bool p1printing   = p1state.connected && (strcmp(p1state.gcode_state, "RUNNING") == 0);
  bool p2printing   = p2state.connected && (strcmp(p2state.gcode_state, "RUNNING") == 0);
  bool p1exhausting = p1state.connected && (p1state.exhaust_fan_speed > 1);
  bool p2exhausting = p2state.connected && (p2state.exhaust_fan_speed > 1);

  // Normalise exhaust to 0-100% for each printer
  int p1pct = (p1state.gen == 1)
    ? (constrain(p1state.exhaust_fan_speed, 0, 15) * 100) / 15
    : constrain(p1state.exhaust_fan_speed, 0, 100);
  int p2pct = (p2state.gen == 1)
    ? (constrain(p2state.exhaust_fan_speed, 0, 15) * 100) / 15
    : constrain(p2state.exhaust_fan_speed, 0, 100);

  int targetPct = 0;

  if (p1exhausting && p2exhausting) {
    float avgExhaust = (p1pct + p2pct) / 2.0f;
    targetPct = (int)(cfg.fan_speed_printing + (avgExhaust / 100.0f) * (cfg.fan_speed_both_exhausting - cfg.fan_speed_printing));
  } else if (p1exhausting) {
    targetPct = (int)(cfg.fan_speed_printing + (p1pct / 100.0f) * (cfg.fan_speed_one_exhausting - cfg.fan_speed_printing));
  } else if (p2exhausting) {
    targetPct = (int)(cfg.fan_speed_printing + (p2pct / 100.0f) * (cfg.fan_speed_one_exhausting - cfg.fan_speed_printing));
  } else if (p1printing || p2printing) {
    targetPct = cfg.fan_speed_printing;
  }

  setFanSpeed(constrain(targetPct, 0, 100));
}

// ══════════════════════════════════════════════════════════════════════════
//  Bambu Cloud authentication — token is entered manually in config
//  To get your token and uid:
//  1. Log in to https://bambulab.com in a browser
//  2. Open DevTools (F12) → Application → Cookies → bambulab.com
//  3. Copy the value of "token" → paste as Bambu Token in config
//  4. Visit https://api.bambulab.com/v1/iot-service/api/user/bind in
//     the same browser (while logged in) and copy your uid from the JSON
//  Token expires after ~3 months and must be updated manually
// ══════════════════════════════════════════════════════════════════════════

bool hasValidToken() {
  if (!strlen(cfg.bambu_token) || !strlen(cfg.bambu_user_id)) {
    Serial.println("[Auth] No token or user ID configured");
    return false;
  }
  return true;
}

// ══════════════════════════════════════════════════════════════════════════
//  MQTT payload parser
// ══════════════════════════════════════════════════════════════════════════

void parseGen1Payload(JsonObject& print, PrinterState& state) {
  // Gen 1: X1C, P1S, P1P, A1, A1 Mini
  // Fields: gcode_state, mc_percent, mc_remaining_time, nozzle_temper,
  //         bed_temper, exhaust_fan_speed, big_fan2_speed
  const char* cmd = print["command"] | "";
  bool hasStatusData = !print["gcode_state"].isNull()      ||
                       !print["mc_percent"].isNull()        ||
                       !print["nozzle_temper"].isNull()     ||
                       !print["bed_temper"].isNull()        ||
                       !print["exhaust_fan_speed"].isNull() ||
                       !print["big_fan2_speed"].isNull();

  if (strlen(cmd) > 0 && !hasStatusData) {
    return;
  }

  if (!print["gcode_state"].isNull())       strlcpy(state.gcode_state, print["gcode_state"] | "IDLE", sizeof(state.gcode_state));
  if (!print["mc_percent"].isNull())        state.progress      = print["mc_percent"].as<int>();
  if (!print["mc_remaining_time"].isNull()) state.remaining_min = print["mc_remaining_time"].as<int>();
  if (!print["nozzle_temper"].isNull())     state.nozzle_temp   = print["nozzle_temper"].as<float>();
  if (!print["bed_temper"].isNull())        state.bed_temp      = print["bed_temper"].as<float>();
  if (!print["device"]["ctc"]["info"]["temp"].isNull())
    state.chamber_temp = print["device"]["ctc"]["info"]["temp"].as<float>();
  if (!print["exhaust_fan_speed"].isNull()) {
    state.exhaust_fan_speed = print["exhaust_fan_speed"].as<int>();
    state.exhausting        = (state.exhaust_fan_speed > 1);
  } else if (!print["big_fan2_speed"].isNull()) {
    // X1C reports exhaust fan as big_fan2_speed, scale 0-15
    int fanSpeed = String(print["big_fan2_speed"] | "0").toInt();
    state.exhaust_fan_speed = fanSpeed;
    state.exhausting        = (fanSpeed > 0);
  }
}

void parseGen2Payload(JsonObject& print, PrinterState& state) {
  // Gen 2: H2C, H2S, H2D
  // Fields are reported differently — status comes via device.extruder
  // gcode_state, mc_percent etc. may still appear in some messages
  const char* cmd = print["command"] | "";
  bool hasStatusData = !print["gcode_state"].isNull()        ||
                       !print["mc_percent"].isNull()          ||
                       !print["nozzle_temper"].isNull()       ||
                       !print["device"]["extruder"].isNull()  ||
                       !print["device"]["airduct"].isNull()   ||
                       !print["layer_num"].isNull()           ||
                       !print["3D"].isNull()                  ||
                       !print["ams"].isNull();

  if (strlen(cmd) > 0 && !hasStatusData) {
    return;
  }

  // Standard fields (present in some Gen 2 messages)
  if (!print["gcode_state"].isNull())
    strlcpy(state.gcode_state, print["gcode_state"] | "IDLE", sizeof(state.gcode_state));
  else if (strcmp(state.gcode_state, "offline") == 0)
    strlcpy(state.gcode_state, "IDLE", sizeof(state.gcode_state));
  if (!print["mc_percent"].isNull()) {
    state.progress = print["mc_percent"].as<int>();
    if (state.progress > 0 && state.progress < 100 &&
        (strcmp(state.gcode_state, "offline") == 0 || strcmp(state.gcode_state, "IDLE") == 0))
      strlcpy(state.gcode_state, "RUNNING", sizeof(state.gcode_state));
  }
  if (!print["mc_remaining_time"].isNull()) state.remaining_min = print["mc_remaining_time"].as<int>();

  // layer_num indicates active printing for Gen 2
  int layerNum = -1;
  if (!print["layer_num"].isNull())            layerNum = print["layer_num"].as<int>();
  else if (!print["3D"]["layer_num"].isNull()) layerNum = print["3D"]["layer_num"].as<int>();
  if (layerNum >= 0) {
    if (strcmp(state.gcode_state, "offline") == 0 || strcmp(state.gcode_state, "IDLE") == 0)
      strlcpy(state.gcode_state, "RUNNING", sizeof(state.gcode_state));
  }
  if (!print["nozzle_temper"].isNull())     state.nozzle_temp   = print["nozzle_temper"].as<float>();
  if (!print["bed_temper"].isNull())
    state.bed_temp = print["bed_temper"].as<float>();
  else if (!print["device"]["bed"]["info"]["temp"].isNull()) {
    int rawTemp = print["device"]["bed"]["info"]["temp"].as<int>();
    state.bed_temp = (rawTemp > 65536) ? (float)(rawTemp / 65536) : (float)rawTemp;
  }
  if (!print["device"]["ctc"]["info"]["temp"].isNull()) {
    int rawTemp = print["device"]["ctc"]["info"]["temp"].as<int>();
    state.chamber_temp = (rawTemp > 65536) ? (float)(rawTemp / 65536) : (float)rawTemp;
  }
  if (!print["exhaust_fan_speed"].isNull()) {
    state.exhaust_fan_speed = print["exhaust_fan_speed"].as<int>();
    state.exhausting        = (state.exhaust_fan_speed > 1);
  }

  // Gen 2 airduct — exhaust fan is func=2 in device.airduct.parts
  JsonArray airductParts = print["device"]["airduct"]["parts"];
  if (!airductParts.isNull()) {
    for (JsonObject part : airductParts) {
      if ((part["func"] | -1) == 2) {
        int fanState = part["state"] | 0;
        state.exhaust_fan_speed = fanState;
        state.exhausting        = (fanState > 1);
        break;
      }
    }
  }
  // info array: id=0 (right/inactive), id=1 (left/active)
  // temp = current nozzle temp (integer degrees)
  // htar = target temp, hnow = current heater
  // stat = extruder status flags
  JsonArray extruderInfo = print["device"]["extruder"]["info"];
  if (!extruderInfo.isNull()) {
    for (JsonObject ext : extruderInfo) {
      int id   = ext["id"]   | -1;
      int temp = ext["temp"] | 0;

      if (id == 1) {
        // Nozzle temp is packed as value / 65536
        if (temp > 65536)
          state.nozzle_temp = (float)(temp / 65536);
        else if (state.nozzle_temp == 0.0f && temp > 0 && temp < 500)
          state.nozzle_temp = (float)temp;
      }
    }
  }
}

void parseBambuPayload(const char* payload, PrinterState& state) {
  if (strlen(payload) < 3) return;  // skip empty {} heartbeats

  JsonDocument doc;
  if (deserializeJson(doc, payload)) return;

  state.connected = true;
  state.last_seen = millis();

  JsonObject print = doc["print"];
  if (print.isNull()) {
    // Gen 2 sometimes sends data at the root level without a "print" wrapper
    if (state.gen == 2 && (!doc["device"].isNull() || !doc["command"].isNull())) {
      JsonObject root = doc.as<JsonObject>();
      parseGen2Payload(root, state);
      updateFanSpeed();
    }
    return;
  }

  if (state.gen == 2)
    parseGen2Payload(print, state);
  else
    parseGen1Payload(print, state);

  updateFanSpeed();
}

// ══════════════════════════════════════════════════════════════════════════
//  MQTT callback — routes by topic serial number
// ══════════════════════════════════════════════════════════════════════════

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String msg((char*)payload, length);

  // Match topic to printer by serial number
  if (strlen(cfg.printer1_serial) && strstr(topic, cfg.printer1_serial))
    parseBambuPayload(msg.c_str(), p1state);
  else if (strlen(cfg.printer2_serial) && strstr(topic, cfg.printer2_serial))
    parseBambuPayload(msg.c_str(), p2state);
}

// ══════════════════════════════════════════════════════════════════════════
//  MQTT connection & loop
// ══════════════════════════════════════════════════════════════════════════

void requestPushAll(const char* serial, PrinterState& state) {
  char reqTopic[80];
  snprintf(reqTopic, sizeof(reqTopic), "device/%s/request", serial);
  const char* pushCmd = "{\"pushing\":{\"command\":\"pushall\"}}";
  bool ok = mqttClient.publish(reqTopic, pushCmd);
  state.pushall_sent = true;
}

bool connectMqtt() {
  if (!hasValidToken()) return false;

  tlsClient.setInsecure();
  const char* mqttHost = (strcmp(cfg.bambu_region, "cn") == 0)
                         ? "cn.mqtt.bambulab.com"
                         : "us.mqtt.bambulab.com";
  mqttClient.setServer(mqttHost, BAMBU_CLOUD_PORT);
  mqttClient.setBufferSize(8192);
  mqttClient.setCallback(onMqttMessage);

  // Cloud MQTT: username = u_{user_id}, password = token
  char mqttUser[80];
  char clientId[64];
  snprintf(mqttUser,  sizeof(mqttUser),  "u_%s", cfg.bambu_user_id);
  snprintf(clientId, sizeof(clientId), "bambuafe-%s", cfg.device_name);

  Serial.printf("[MQTT] Connecting to %s as %s...\n", mqttHost, mqttUser);
  if (!mqttClient.connect(clientId, mqttUser, cfg.bambu_token)) {
    Serial.printf("[MQTT] Connection failed, rc=%d\n", mqttClient.state());
    return false;
  }

  Serial.println("[MQTT] Connected to Bambu Cloud");

  // Subscribe to both printer report topics
  if (strlen(cfg.printer1_serial)) {
    char topic[80];
    snprintf(topic, sizeof(topic), "device/%s/report", cfg.printer1_serial);
    bool ok = mqttClient.subscribe(topic, 1);
    p1state.pushall_sent = false;
    Serial.printf("[MQTT] Subscribed: %s — %s\n", topic, ok ? "OK" : "FAILED");
  }
  if (strlen(cfg.printer2_serial)) {
    char topic[80];
    snprintf(topic, sizeof(topic), "device/%s/report", cfg.printer2_serial);
    bool ok = mqttClient.subscribe(topic, 1);
    p2state.pushall_sent = false;
    Serial.printf("[MQTT] Subscribed: %s — %s\n", topic, ok ? "OK" : "FAILED");
  }

  return true;
}

void initMqtt() {
  strlcpy(p1state.name, cfg.printer1_name, sizeof(p1state.name));
  p1state.gen = cfg.printer1_gen;
  strlcpy(p2state.name, cfg.printer2_name, sizeof(p2state.name));
  p2state.gen = cfg.printer2_gen;
  if (!hasValidToken()) {
    Serial.println("[MQTT] No token configured — skipping MQTT connect");
    return;
  }
  connectMqtt();
}

void reinitMqtt() {
  Serial.println("[MQTT] Reinitialising...");
  if (mqttClient.connected()) mqttClient.disconnect();
  p1state = PrinterState();
  p2state = PrinterState();
  updateFanSpeed();
  initMqtt();
}

void loopMqtt() {
  static unsigned long lastReconnect   = 0;
  static unsigned long lastPushall1    = 0;
  static unsigned long lastPushall2    = 0;

  if (mqttClient.connected()) {
    mqttClient.loop();

    // Send pushall on connect, then every 45s to keep status fresh
    if (strlen(cfg.printer1_serial)) {
      if (!p1state.pushall_sent || millis() - lastPushall1 > 45000) {
        lastPushall1 = millis();
        requestPushAll(cfg.printer1_serial, p1state);
      }
    }
    if (strlen(cfg.printer2_serial)) {
      if (!p2state.pushall_sent || millis() - lastPushall2 > 45000) {
        lastPushall2 = millis();
        requestPushAll(cfg.printer2_serial, p2state);
      }
    }
  } else if (millis() - lastReconnect > MQTT_RECONNECT_MS) {
    lastReconnect = millis();
    Serial.println("[MQTT] Reconnecting...");
    p1state.connected    = false;
    p2state.connected    = false;
    p1state.pushall_sent = false;
    p2state.pushall_sent = false;
    connectMqtt();
  }

  // Mark printer offline if no message received for 60s
  if (p1state.connected && p1state.last_seen && millis() - p1state.last_seen > 60000) {
    p1state.connected = false;
    strlcpy(p1state.gcode_state, "offline", sizeof(p1state.gcode_state));
    p1state.exhausting = false; p1state.exhaust_fan_speed = 0;
    updateFanSpeed();
  }
  if (p2state.connected && p2state.last_seen && millis() - p2state.last_seen > 60000) {
    p2state.connected = false;
    strlcpy(p2state.gcode_state, "offline", sizeof(p2state.gcode_state));
    p2state.exhausting = false; p2state.exhaust_fan_speed = 0;
    updateFanSpeed();
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  Filesystem helpers
// ══════════════════════════════════════════════════════════════════════════

bool configExists() { return LittleFS.exists(CONFIG_FILE); }

bool loadConfig() {
  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f) return false;
  JsonDocument doc;
  if (deserializeJson(doc, f)) { f.close(); return false; }
  f.close();

  strlcpy(cfg.wifi_ssid,       doc["wifi_ssid"]       | "",               sizeof(cfg.wifi_ssid));
  strlcpy(cfg.wifi_pass,       doc["wifi_pass"]       | "",               sizeof(cfg.wifi_pass));
  strlcpy(cfg.device_name,     doc["device_name"]     | "BambuAFE-ESP32", sizeof(cfg.device_name));
  strlcpy(cfg.dashboard_pass,  doc["dashboard_pass"]  | "admin",          sizeof(cfg.dashboard_pass));
  strlcpy(cfg.bambu_region,    doc["bambu_region"]    | "us",             sizeof(cfg.bambu_region));
  strlcpy(cfg.bambu_user_id,   doc["bambu_user_id"]   | "",               sizeof(cfg.bambu_user_id));
  strlcpy(cfg.bambu_token,     doc["bambu_token"]     | "",               sizeof(cfg.bambu_token));
  strlcpy(cfg.printer1_name,   doc["printer1_name"]   | "",               sizeof(cfg.printer1_name));
  strlcpy(cfg.printer1_serial, doc["printer1_serial"] | "",               sizeof(cfg.printer1_serial));
  cfg.printer1_gen              = doc["printer1_gen"]              | 1;
  strlcpy(cfg.printer2_name,   doc["printer2_name"]   | "",               sizeof(cfg.printer2_name));
  strlcpy(cfg.printer2_serial, doc["printer2_serial"] | "",               sizeof(cfg.printer2_serial));
  cfg.printer2_gen              = doc["printer2_gen"]              | 1;
  cfg.fan_speed_printing        = doc["fan_speed_printing"]        | 10;
  cfg.fan_speed_one_exhausting      = doc["fan_speed_one_exhausting"]      | 50;
  cfg.fan_speed_both_exhausting = doc["fan_speed_both_exhausting"] | 100;

  return true;
}

bool saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return false;
  JsonDocument doc;
  doc["wifi_ssid"]               = cfg.wifi_ssid;
  doc["wifi_pass"]               = cfg.wifi_pass;
  doc["device_name"]             = cfg.device_name;
  doc["dashboard_pass"]          = cfg.dashboard_pass;
  doc["bambu_region"]            = cfg.bambu_region;
  doc["bambu_user_id"]           = cfg.bambu_user_id;
  doc["bambu_token"]             = cfg.bambu_token;
  doc["printer1_name"]           = cfg.printer1_name;
  doc["printer1_serial"]         = cfg.printer1_serial;
  doc["printer1_gen"]            = cfg.printer1_gen;
  doc["printer2_name"]           = cfg.printer2_name;
  doc["printer2_serial"]         = cfg.printer2_serial;
  doc["printer2_gen"]            = cfg.printer2_gen;
  doc["fan_speed_printing"]      = cfg.fan_speed_printing;
  doc["fan_speed_one_exhausting"]    = cfg.fan_speed_one_exhausting;
  doc["fan_speed_both_exhausting"] = cfg.fan_speed_both_exhausting;
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

void deleteConfig() {
  if (LittleFS.exists(CONFIG_FILE)) { LittleFS.remove(CONFIG_FILE); Serial.println("[Config] Deleted"); }
}

// ══════════════════════════════════════════════════════════════════════════
//  HTML helpers
// ══════════════════════════════════════════════════════════════════════════

String loadHtmlFile(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) return "<h1>File not found: " + String(path) + "</h1>";
  String html = f.readString();
  f.close();
  return html;
}

// ══════════════════════════════════════════════════════════════════════════
//  Config Portal (AP mode)
// ══════════════════════════════════════════════════════════════════════════

void handlePortalRoot() { server.send(200, "text/html", loadHtmlFile("/setup.html")); }

void handlePortalSave() {
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return; }
  if (server.hasArg("wifi_ssid"))      strlcpy(cfg.wifi_ssid,      server.arg("wifi_ssid").c_str(),      sizeof(cfg.wifi_ssid));
  if (server.hasArg("wifi_pass"))      strlcpy(cfg.wifi_pass,      server.arg("wifi_pass").c_str(),      sizeof(cfg.wifi_pass));
  if (server.hasArg("device_name"))    strlcpy(cfg.device_name,    server.arg("device_name").c_str(),    sizeof(cfg.device_name));
  if (server.hasArg("dashboard_pass")) strlcpy(cfg.dashboard_pass, server.arg("dashboard_pass").c_str(), sizeof(cfg.dashboard_pass));
  if (saveConfig()) {
    server.send(200, "text/html", loadHtmlFile("/saved.html"));
    delay(2000); ESP.restart();
  } else {
    server.send(500, "text/plain", "Failed to save config.");
  }
}

void startConfigPortal() {
  Serial.println("\n[Portal] Starting config portal...");
  WiFi.mode(WIFI_AP);
  strlen(AP_PASSWORD) > 0 ? WiFi.softAP(AP_SSID, AP_PASSWORD) : WiFi.softAP(AP_SSID);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[Portal] AP: %s  |  Visit: http://%s\n", AP_SSID, ip.toString().c_str());

  server.on("/",     HTTP_GET,  handlePortalRoot);
  server.on("/save", HTTP_POST, handlePortalSave);
  server.on("/scan", HTTP_GET, []() {
    int n = WiFi.scanNetworks();
    String json = "[";
    for (int i = 0; i < n; i++) {
      if (i) json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) + "\","
              "\"rssi\":" + WiFi.RSSI(i) + ","
              "\"secured\":" + (WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
    }
    json += "]";
    server.send(200, "application/json", json);
  });
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();
  pinMode(2, OUTPUT);
  Serial.println("[Portal] Waiting for configuration...");
  while (true) { server.handleClient(); digitalWrite(2, !digitalRead(2)); delay(200); }
}

// ══════════════════════════════════════════════════════════════════════════
//  Normal mode HTTP handlers
// ══════════════════════════════════════════════════════════════════════════

bool checkAuth() {
  if (!server.authenticate("admin", cfg.dashboard_pass)) {
    server.requestAuthentication(BASIC_AUTH, "BambuAFE Dashboard", "Authorisation required");
    return false;
  }
  return true;
}

void handleDashboard()  { if (!checkAuth()) return; server.send(200, "text/html", loadHtmlFile("/index.html")); }
void handleConfigPage() { if (!checkAuth()) return; server.send(200, "text/html", loadHtmlFile("/config.html")); }

void handleStatus() {
  if (!checkAuth()) return;
  JsonDocument doc;
  doc["device_name"]      = cfg.device_name;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["uptime_s"]         = millis() / 1000;
  doc["free_heap"]        = ESP.getFreeHeap();
  doc["ip"]               = WiFi.localIP().toString();
  doc["fan_pct"]          = currentFanPct;
  doc["cloud_connected"]  = mqttClient.connected();
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handlePrinterStatus() {
  if (!checkAuth()) return;
  auto printerJson = [](JsonObject obj, const PrinterState& s, const char* name) {
    // Normalise exhaust_fan_speed to 0-100%
    // Gen1 X1C: big_fan2_speed uses 0-15 scale
    // Gen2 H2C: airduct state is already 0-100
    int exhaustPct;
    if (s.gen == 1) {
      exhaustPct = (constrain(s.exhaust_fan_speed, 0, 15) * 100) / 15;
    } else {
      exhaustPct = constrain(s.exhaust_fan_speed, 0, 100);
    }
    obj["name"]          = strlen(s.name) ? s.name : name;
    obj["connected"]     = s.connected;
    obj["gcode_state"]   = s.gcode_state;
    obj["progress"]      = s.progress;
    obj["remaining_min"] = s.remaining_min;
    obj["nozzle_temp"]   = serialized(String(s.nozzle_temp, 1));
    obj["bed_temp"]      = serialized(String(s.bed_temp, 1));
    obj["chamber_temp"]  = serialized(String(s.chamber_temp, 1));
    obj["exhausting"]    = s.exhausting;
    obj["exhaust_pct"]   = exhaustPct;
  };
  JsonDocument doc;
  doc["fan_pct"] = currentFanPct;
  if (strlen(cfg.printer1_serial)) printerJson(doc["printer1"].to<JsonObject>(), p1state, cfg.printer1_name);
  if (strlen(cfg.printer2_serial)) printerJson(doc["printer2"].to<JsonObject>(), p2state, cfg.printer2_name);
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleConfigGet() {
  if (!checkAuth()) return;
  JsonDocument doc;
  doc["has_token"]               = strlen(cfg.bambu_token) > 0;
  doc["bambu_user_id"]           = cfg.bambu_user_id;
  doc["bambu_region"]            = cfg.bambu_region;
  doc["printer1_name"]           = cfg.printer1_name;
  doc["printer1_serial"]         = cfg.printer1_serial;
  doc["printer1_gen"]            = cfg.printer1_gen;
  doc["printer2_name"]           = cfg.printer2_name;
  doc["printer2_serial"]         = cfg.printer2_serial;
  doc["printer2_gen"]            = cfg.printer2_gen;
  doc["fan_speed_printing"]      = cfg.fan_speed_printing;
  doc["fan_speed_one_exhausting"]    = cfg.fan_speed_one_exhausting;
  doc["fan_speed_both_exhausting"] = cfg.fan_speed_both_exhausting;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleConfigPost() {
  if (!checkAuth()) return;
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return; }

  if (server.hasArg("bambu_user_id"))
    strlcpy(cfg.bambu_user_id, server.arg("bambu_user_id").c_str(), sizeof(cfg.bambu_user_id));
  if (server.hasArg("bambu_token") && server.arg("bambu_token").length() > 0)
    strlcpy(cfg.bambu_token, server.arg("bambu_token").c_str(), sizeof(cfg.bambu_token));
  if (server.hasArg("bambu_region"))    strlcpy(cfg.bambu_region,    server.arg("bambu_region").c_str(),    sizeof(cfg.bambu_region));
  if (server.hasArg("printer1_name"))   strlcpy(cfg.printer1_name,   server.arg("printer1_name").c_str(),   sizeof(cfg.printer1_name));
  if (server.hasArg("printer1_serial")) strlcpy(cfg.printer1_serial, server.arg("printer1_serial").c_str(), sizeof(cfg.printer1_serial));
  if (server.hasArg("printer1_gen"))    cfg.printer1_gen = server.arg("printer1_gen").toInt();
  if (server.hasArg("printer2_name"))   strlcpy(cfg.printer2_name,   server.arg("printer2_name").c_str(),   sizeof(cfg.printer2_name));
  if (server.hasArg("printer2_serial")) strlcpy(cfg.printer2_serial, server.arg("printer2_serial").c_str(), sizeof(cfg.printer2_serial));
  if (server.hasArg("printer2_gen"))    cfg.printer2_gen = server.arg("printer2_gen").toInt();
  if (server.hasArg("fan_speed_printing")) {
    int v = server.arg("fan_speed_printing").toInt();
    cfg.fan_speed_printing = (v == 0) ? 0 : constrain(v, 10, 100);
  }
  if (server.hasArg("fan_speed_one_exhausting")) {
    int v = server.arg("fan_speed_one_exhausting").toInt();
    cfg.fan_speed_one_exhausting = (v == 0) ? 0 : constrain(v, 10, 100);
  }
  if (server.hasArg("fan_speed_both_exhausting")) {
    int v = server.arg("fan_speed_both_exhausting").toInt();
    cfg.fan_speed_both_exhausting = (v == 0) ? 0 : constrain(v, 10, 100);
  }

  if (saveConfig()) {
    reinitMqtt();
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"Save failed\"}");
  }
}

void handleResetPost() {
  if (!checkAuth()) return;
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return; }
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500); deleteConfig(); ESP.restart();
}

void handleReboot() {
  if (!checkAuth()) return;
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return; }
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500); ESP.restart();
}

void startNormalServer() {
  server.on("/",               HTTP_GET,  handleDashboard);
  server.on("/config.html",    HTTP_GET,  handleConfigPage);
  server.on("/status",         HTTP_GET,  handleStatus);
  server.on("/printer-status", HTTP_GET,  handlePrinterStatus);
  server.on("/config",         HTTP_GET,  handleConfigGet);
  server.on("/config",         HTTP_POST, handleConfigPost);
  server.on("/reset",          HTTP_POST, handleResetPost);
  server.on("/reboot",         HTTP_POST, handleReboot);
  server.on("/get_bambu_token.ps1", HTTP_GET, []() {
    if (!checkAuth()) return;
    File f = LittleFS.open("/get_bambu_token.ps1", "r");
    if (!f) { server.send(404, "text/plain", "Not found"); return; }
    String content = f.readString();
    f.close();
    // Replace the default hostname with the actual configured device name
    content.replace("BambuAFE-ESP32", cfg.device_name);
    server.sendHeader("Content-Disposition", "attachment; filename=\"get_bambu_token.ps1\"");
    server.send(200, "application/octet-stream", content);
  });
  server.onNotFound([]() {
    if (!checkAuth()) return;
    String path = server.uri();
    if (LittleFS.exists(path)) {
      String ct = "text/plain";
      if (path.endsWith(".html")) ct = "text/html";
      else if (path.endsWith(".js"))  ct = "application/javascript";
      else if (path.endsWith(".css")) ct = "text/css";
      else if (path.endsWith(".ico")) ct = "image/x-icon";
      else if (path.endsWith(".ps1")) ct = "application/octet-stream";
      File f = LittleFS.open(path, "r");
      server.streamFile(f, ct);
      f.close();
    } else {
      server.send(404, "text/plain", "Not found: " + path);
    }
  });
  server.begin();
  Serial.println("[Server] HTTP server started");
}

// ══════════════════════════════════════════════════════════════════════════
//  Wi-Fi
// ══════════════════════════════════════════════════════════════════════════

bool connectWiFi(unsigned long timeoutMs = 15000) {
  Serial.printf("[WiFi] Connecting to: %s\n", cfg.wifi_ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(cfg.device_name);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
  WiFi.setAutoReconnect(true);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) { Serial.println("[WiFi] Timed out"); return false; }
    delay(500); Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected  IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

// ══════════════════════════════════════════════════════════════════════════
//  Reset-button detection
// ══════════════════════════════════════════════════════════════════════════

bool resetButtonHeld() {
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  if (digitalRead(RESET_BUTTON_PIN) == HIGH) return false;
  Serial.println("[Reset] Button held — keep holding to wipe config...");
  unsigned long start = millis();
  while (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (millis() - start >= RESET_HOLD_MS) return true;
    delay(50);
  }
  return false;
}

// ══════════════════════════════════════════════════════════════════════════
//  setup() & loop()
// ══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n\n=== BambuAFE ESP32 ===");
  Serial.printf("[Boot] Firmware version: %s\n", FIRMWARE_VERSION);

  initFans();

  if (!LittleFS.begin(true)) { Serial.println("[FS] Mount failed!"); return; }
  Serial.println("[FS] LittleFS mounted");

  if (resetButtonHeld()) { Serial.println("[Reset] Wiping config"); deleteConfig(); }

  if (!configExists() || !loadConfig()) startConfigPortal();

  if (!connectWiFi()) {
    Serial.println("[Boot] Wi-Fi failed — falling back to config portal");
    deleteConfig(); ESP.restart();
  }

  startNormalServer();

  if (MDNS.begin(cfg.device_name)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] Reachable at http://%s.local/\n", cfg.device_name);
  }

  initMqtt();

  pinMode(2, OUTPUT);
  Serial.printf("[Boot] Running as: %s\n", cfg.device_name);
}

void loop() {
  server.handleClient();
  loopMqtt();
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink >= 1000) { lastBlink = millis(); digitalWrite(2, !digitalRead(2)); }
}
