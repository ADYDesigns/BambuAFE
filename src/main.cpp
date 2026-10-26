/**
 * ESP32 BambuAFE — Local (LAN) Edition
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
 *   - Connects DIRECTLY to each printer's own MQTT broker on the local network
 *     (TLS, port 8883, user "bblp", password = the printer's LAN access code).
 *     No Bambu Cloud account, token, Developer Mode or LAN-Only mode needed.
 *   - Finds printers with SSDP discovery ("Find printers" on the config page)
 *     and re-discovers a printer by serial number if its IP address changes
 *   - Parses print state and exhaust fan status to drive a shared PWM fan
 *   - Scales fan speed based on printer exhaust percentage
 *   - Tracks and saves fan runtime to /runtime.json every 10 minutes
 *   - Serves a password-protected status dashboard at / (index.html)
 *
 * To reset config: hold BOOT button (GPIO 0) for 3 seconds on startup
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <functional>
#include <ESPmDNS.h>

// ─── Pin & timing constants ────────────────────────────────────────────────
#define RESET_BUTTON_PIN   0
#define RESET_HOLD_MS      3000
#define AP_SSID            "BambuAFE-Setup"
#define AP_PASSWORD        ""
#define CONFIG_FILE        "/config.json"
#define FIRMWARE_VERSION   "1.1.0"

#define FAN1_PIN           16
#define FAN2_PIN           17
#define PWM_FREQ           25000
#define PWM_RESOLUTION     8
#define PWM_CHANNEL_1      0
#define PWM_CHANNEL_2      1
#define POT_PIN            35

#define PRINTER_COUNT      2

// Local MQTT — every Bambu printer runs its own broker
#define BAMBU_LOCAL_PORT   8883
#define BAMBU_LOCAL_USER   "bblp"
// A local H2C report is a FULL status message of ~25-30 KB, about once a
// second (measured on two H2Cs).  Holding that in RAM per printer, on top of
// ~40 KB per TLS connection, ran the ESP32 out of memory with two printers.
// So messages are never stored: the JSON is parsed straight off the TLS
// connection as it arrives, keeping only the handful of fields we need.
#define MQTT_KEEPALIVE_S         30
#define MQTT_READ_TIMEOUT_MS     3000   // max wait for the next bytes of a message in progress
#define TLS_CONNECT_TIMEOUT_S    4    // core default is 30 s — would freeze the loop if a printer is off
#define TLS_HANDSHAKE_TIMEOUT_S  10   // core default is 120 s
#define MQTT_RECONNECT_MIN_MS    15000
#define MQTT_RECONNECT_MAX_MS    60000  // back off to this while a printer stays unreachable
#define PUSHALL_INTERVAL_MS      300000 // full-status request every 5 min (P1/A1 printers dislike frequent pushall)
#define NO_DATA_WARN_MS          30000  // connected but silent this long = warn in the serial log
#define STALE_DATA_MS            360000 // connected but silent this long = drop and reconnect

// SSDP discovery — printers answer an M-SEARCH sent to this multicast group
#define SSDP_MULTICAST_IP        IPAddress(239, 255, 255, 250)
#define SSDP_PORT_PRIMARY        1990   // H2C answered from this port
#define SSDP_PORT_ALT            2021   // some other models/firmware use this one
#define SSDP_WAIT_MS             2500
#define SSDP_MAX_RESULTS         16     // printers listed per search (~136 bytes each, temporary)
#define REDISCOVER_AFTER_FAILS   3      // connection failures before searching for a new IP
#define REDISCOVER_MIN_GAP_MS    120000 // don't search more often than this per printer

// ─── Global objects ────────────────────────────────────────────────────────
WebServer server(80);

// ─── Config struct ─────────────────────────────────────────────────────────
struct PrinterConfig {
  char name[64]        = "";
  char ip[16]          = "";   // dotted IPv4; may be filled in automatically by discovery
  char serial[32]      = "";
  char access_code[32] = "";   // the 8-character LAN access code shown on the printer's screen
};

struct Config {
  char wifi_ssid[64]      = "";
  char wifi_pass[64]      = "";
  char device_name[64]    = "BambuAFE-ESP32";
  char dashboard_pass[64] = "admin";
  PrinterConfig printer[PRINTER_COUNT];
  int  fan_speed_printing        = 10;
  int  fan_speed_one_exhausting  = 50;
  int  fan_speed_both_exhausting = 100;
  int  fan_runon_s               = 0;      // keep the fans going this long after they would stop (0-180)
  bool manual_knob               = false;  // GPIO 35 floats with no knob attached, so only read it when enabled
};

#define FAN_RUNON_MAX_S    180

Config cfg;

// ─── Printer runtime state ─────────────────────────────────────────────────
struct PrinterState {
  char  gcode_state[32]   = "offline";
  int   progress          = 0;
  int   remaining_min     = 0;
  int   exhaust_pct       = 0;      // already normalised to 0-100, whatever the model
  bool  exhausting        = false;
  bool  has_data          = false;  // at least one report parsed since connecting
  unsigned long last_seen = 0;
};

// ══════════════════════════════════════════════════════════════════════════
//  Minimal MQTT 3.1.1 client for the printer's local broker
//
//  Why not PubSubClient?  It must hold each whole message in a buffer before
//  you can look at it — 40 KB per printer here.  This client hands the
//  message body to the JSON parser as a stream instead, 512 bytes at a time.
//  Only what Bambu needs is implemented: CONNECT, SUBSCRIBE, PUBLISH (QoS 0)
//  and PINGREQ.  An MQTT packet is: 1 type byte, a "remaining length" in
//  7-bit chunks, then the body.
// ══════════════════════════════════════════════════════════════════════════

// Feeds one message body to ArduinoJson.  ArduinoJson accepts any class with
// read() and readBytes() as a "custom reader".  It never reads past the end
// of this message, and drain() discards whatever the parser didn't consume.
class PayloadReader {
 public:
  PayloadReader(WiFiClientSecure& c, uint32_t len) : _c(c), _left(len) {}
  int read() {
    if (_pos >= _have && !fill()) return -1;
    return (uint8_t)_buf[_pos++];
  }
  size_t readBytes(char* out, size_t n) {
    size_t got = 0;
    while (got < n) {
      if (_pos >= _have && !fill()) break;
      size_t k = min(n - got, _have - _pos);
      memcpy(out + got, _buf + _pos, k);
      _pos += k; got += k;
    }
    return got;
  }
  bool drain() {                       // skip the rest of this message
    _pos = _have;
    while (_left) { if (!fill()) return false; _pos = _have; }
    return true;
  }
  bool failed() const { return _failed; }

 private:
  bool fill() {
    if (_left == 0) return false;
    unsigned long start = millis();
    while (true) {
      int avail = _c.available();
      if (avail > 0) {
        size_t want = min((size_t)_left, sizeof(_buf));
        want = min(want, (size_t)avail);
        int n = _c.read((uint8_t*)_buf, want);
        if (n > 0) { _have = n; _pos = 0; _left -= n; return true; }
      }
      if (!_c.connected() || millis() - start > MQTT_READ_TIMEOUT_MS) { _failed = true; return false; }
      delay(1);
    }
  }
  WiFiClientSecure& _c;
  uint32_t _left;
  char     _buf[512];
  size_t   _pos = 0, _have = 0;
  bool     _failed = false;
};

class LocalMqtt {
 public:
  WiFiClientSecure tls;
  int  lastRc = 0;     // same codes PubSubClient used: -2 can't connect, -4 timeout, 1-5 = broker refused
  int  index  = 0;     // printer number, for log messages
  std::function<void(PayloadReader&)> onPublish;

  bool connected() { return _up && tls.connected(); }

  bool connect(const char* host, uint16_t port, const char* clientId, const char* user, const char* pass) {
    _up = false;
    tls.stop();
    IPAddress ip;
    if (!ip.fromString(host) || !tls.connect(ip, port)) { lastRc = -2; tls.stop(); return false; }

    uint8_t body[256]; size_t n = 0;
    if (!putStr(body, n, sizeof(body), "MQTT")) return fail(-2);
    body[n++] = 4;                                   // protocol level 4 = MQTT 3.1.1
    body[n++] = 0xC2;                                // flags: username + password + clean session
    body[n++] = MQTT_KEEPALIVE_S >> 8; body[n++] = MQTT_KEEPALIVE_S & 0xFF;
    if (!putStr(body, n, sizeof(body), clientId) || !putStr(body, n, sizeof(body), user) ||
        !putStr(body, n, sizeof(body), pass)) return fail(-2);
    if (!sendPacket(0x10, body, n)) return fail(-2);

    uint8_t h, len, flags, rc;                       // CONNACK: 0x20 0x02 <flags> <return code>
    if (!readByte(h, 5000) || !readByte(len, 1000) || !readByte(flags, 1000) || !readByte(rc, 1000))
      return fail(-4);
    if ((h >> 4) != 2) return fail(-4);
    if (rc != 0) return fail(rc);
    lastRc = 0;
    _up = true;
    _lastIn = _lastOut = millis();
    return true;
  }

  void disconnect() {
    if (tls.connected()) { const uint8_t d[2] = { 0xE0, 0x00 }; tls.write(d, 2); }
    _up = false;
    tls.stop();
  }

  bool subscribe(const char* topic) {
    uint8_t body[128]; size_t n = 0;
    body[n++] = 0; body[n++] = 1;                    // packet id 1
    if (!putStr(body, n, sizeof(body), topic) || n >= sizeof(body)) return false;
    body[n++] = 0;                                   // QoS 0
    return sendPacket(0x82, body, n);
  }

  bool publish(const char* topic, const char* payload) {
    uint8_t body[256]; size_t n = 0;
    size_t plen = strlen(payload);
    if (!putStr(body, n, sizeof(body), topic) || n + plen > sizeof(body)) return false;
    memcpy(body + n, payload, plen); n += plen;
    return sendPacket(0x30, body, n);
  }

  // Keepalive, then read at most one packet.  A 30 KB report takes a few tens of ms.
  void loop() {
    if (!_up) return;
    if (!tls.connected()) { _up = false; tls.stop(); return; }
    unsigned long now = millis();
    if (now - _lastOut > MQTT_KEEPALIVE_S * 500UL) {          // ping at half the keepalive
      const uint8_t ping[2] = { 0xC0, 0x00 };
      if (tls.write(ping, 2) != 2) { disconnect(); return; }
      _lastOut = now;
    }
    if (now - _lastIn > MQTT_KEEPALIVE_S * 2000UL) {          // nothing at all, not even PINGRESP
      Serial.printf("[MQTT] Printer %d: no response from printer — dropping connection\n", index + 1);
      disconnect(); return;
    }
    if (tls.available() <= 0) return;

    uint8_t h; uint32_t rem;
    if (!readByte(h, MQTT_READ_TIMEOUT_MS) || !readRemLen(rem)) { disconnect(); return; }
    _lastIn = millis();
    uint8_t type = h >> 4;

    if (type == 3) {                                          // PUBLISH from the printer
      uint8_t a, b;
      if (rem < 2 || !readByte(a, MQTT_READ_TIMEOUT_MS) || !readByte(b, MQTT_READ_TIMEOUT_MS)) { disconnect(); return; }
      uint32_t topicLen = ((uint32_t)a << 8) | b;
      uint32_t hdr = 2 + topicLen + (((h >> 1) & 3) ? 2 : 0); // topic (+ packet id if QoS > 0)
      if (hdr > rem || !skip(hdr - 2)) { disconnect(); return; }
      PayloadReader in(tls, rem - hdr);                       // one subscription per link, so no topic check
      if (onPublish) onPublish(in);
      if (!in.drain()) { disconnect(); return; }
    } else if (type == 9) {                                   // SUBACK
      uint8_t buf[8]; uint32_t got = 0; bool ok = true;
      while (got < rem) { uint8_t c; if (!readByte(c, MQTT_READ_TIMEOUT_MS)) { ok = false; break; } if (got < sizeof(buf)) buf[got] = c; got++; }
      if (!ok) { disconnect(); return; }
      Serial.printf("[MQTT] Printer %d: subscription %s\n", index + 1,
                    (rem >= 3 && buf[2] != 0x80) ? "accepted" : "REFUSED");
    } else if (!skip(rem)) {                                  // PINGRESP and anything else
      disconnect();
    }
  }

 private:
  bool _up = false;
  unsigned long _lastIn = 0, _lastOut = 0;

  bool fail(int rc) { lastRc = rc; _up = false; tls.stop(); return false; }

  static bool putStr(uint8_t* buf, size_t& n, size_t cap, const char* s) {
    size_t len = strlen(s);
    if (n + 2 + len > cap) return false;
    buf[n++] = len >> 8; buf[n++] = len & 0xFF;
    memcpy(buf + n, s, len); n += len;
    return true;
  }

  bool sendPacket(uint8_t header, const uint8_t* body, size_t len) {
    uint8_t hdr[5]; size_t h = 0;
    hdr[h++] = header;
    size_t x = len;
    do { uint8_t d = x % 128; x /= 128; if (x) d |= 0x80; hdr[h++] = d; } while (x && h < 5);
    if (tls.write(hdr, h) != h || tls.write(body, len) != len) return false;
    _lastOut = millis();
    return true;
  }

  bool readByte(uint8_t& out, unsigned long timeoutMs) {
    unsigned long start = millis();
    while (tls.available() <= 0) {
      if (!tls.connected() || millis() - start > timeoutMs) return false;
      delay(1);
    }
    int c = tls.read();
    if (c < 0) return false;
    out = (uint8_t)c;
    return true;
  }

  bool readRemLen(uint32_t& len) {
    len = 0; uint32_t mult = 1;
    for (int i = 0; i < 4; i++) {
      uint8_t d;
      if (!readByte(d, MQTT_READ_TIMEOUT_MS)) return false;
      len += (d & 0x7F) * mult;
      if (!(d & 0x80)) return true;
      mult *= 128;
    }
    return false;                                             // malformed length
  }

  bool skip(uint32_t n) {
    uint8_t buf[64];
    while (n) {
      unsigned long start = millis();
      while (tls.available() <= 0) {
        if (!tls.connected() || millis() - start > MQTT_READ_TIMEOUT_MS) return false;
        delay(1);
      }
      int r = tls.read(buf, min((uint32_t)sizeof(buf), n));
      if (r <= 0) return false;
      n -= r;
    }
    return true;
  }
};

// One connection per printer — each printer is its own MQTT broker, so the
// single shared cloud connection of the old firmware becomes two links here.
struct PrinterLink {
  LocalMqtt        mqtt;
  PrinterState     state;
  bool          wasConnected   = false;
  unsigned long connectedAt    = 0;
  unsigned long lastAttempt    = 0;
  unsigned long retryDelay     = MQTT_RECONNECT_MIN_MS;
  unsigned long lastPushall    = 0;
  unsigned long lastDiscovery  = 0;
  int           failCount      = 0;
  int           lastRc         = 0;     // last MQTT result code on a failed connect
  bool          warnedNoData   = false;
};

PrinterLink links[PRINTER_COUNT];
int currentFanPct = 0;

// ─── Fan runtime tracking ──────────────────────────────────────────────────
unsigned long runtimeMinutes    = 0;   // total minutes fan has run above 5%
unsigned long lastRuntimeSave   = 0;   // millis of last save to LittleFS
unsigned long lastRuntimeTick   = 0;   // millis of last accumulator tick
unsigned long runtimeSecondAcc  = 0;   // accumulated milliseconds toward the next minute

#define RUNTIME_FILE       "/runtime.json"
#define RUNTIME_SAVE_MS    600000      // save every 10 minutes

bool saveConfig();  // forward declaration — used by the self-heal code before its definition

// ══════════════════════════════════════════════════════════════════════════
//  Potentiometer — manual fan override on GPIO 35
//  Returns 0-100 if pot is connected, -1 if not detected or not enabled
// ══════════════════════════════════════════════════════════════════════════

int readPotentiometer() {
  if (!cfg.manual_knob) return -1;   // nothing wired to GPIO 35 — its random readings would start the fans
  int raw = analogRead(POT_PIN);
  if (raw < 372) return -1;  // below ~0.3v threshold — treat as not connected
  return map(raw, 372, 4095, 0, 100);
}

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

bool printerConfigured(int i) {
  return strlen(cfg.printer[i].serial) > 0 && strlen(cfg.printer[i].access_code) > 0;
}

// "Live" = connected to the printer AND we have parsed real data from it
bool printerLive(int i) {
  return printerConfigured(i) && links[i].mqtt.connected() && links[i].state.has_data;
}

void updateFanSpeed() {
  // Check for manual potentiometer override first
  int potPct = readPotentiometer();
  if (potPct >= 0) {
    setFanSpeed(constrain(potPct, 0, 100));
    return;
  }

  bool printing[PRINTER_COUNT], exhausting[PRINTER_COUNT];
  int  pct[PRINTER_COUNT];
  for (int i = 0; i < PRINTER_COUNT; i++) {
    const PrinterState& s = links[i].state;
    bool live     = printerLive(i);
    printing[i]   = live && strcmp(s.gcode_state, "RUNNING") == 0;
    exhausting[i] = live && s.exhausting;
    pct[i]        = live ? s.exhaust_pct : 0;
  }

  int targetPct = 0;
  if (exhausting[0] && exhausting[1]) {
    float avgExhaust = (pct[0] + pct[1]) / 2.0f;
    targetPct = (int)(cfg.fan_speed_printing + (avgExhaust / 100.0f) * (cfg.fan_speed_both_exhausting - cfg.fan_speed_printing));
  } else if (exhausting[0] || exhausting[1]) {
    int p = exhausting[0] ? pct[0] : pct[1];
    targetPct = (int)(cfg.fan_speed_printing + (p / 100.0f) * (cfg.fan_speed_one_exhausting - cfg.fan_speed_printing));
  } else if (printing[0] || printing[1]) {
    targetPct = cfg.fan_speed_printing;
  }

  // Run-on: when the fans would stop, hold their last speed for fan_runon_s
  // seconds to clear what is left in the chamber and hose.
  static unsigned long lastNeededAt = 0;
  static int           lastAutoPct  = 0;
  if (targetPct > 0) {
    lastNeededAt = millis();
    lastAutoPct  = targetPct;
  } else if (lastAutoPct > 0 && millis() - lastNeededAt < cfg.fan_runon_s * 1000UL) {
    targetPct = lastAutoPct;
  } else {
    lastAutoPct = 0;
  }

  setFanSpeed(constrain(targetPct, 0, 100));
}

// ══════════════════════════════════════════════════════════════════════════
//  MQTT payload parser
//
//  One parser for every model.  Instead of asking the user which "generation"
//  a printer is, we look at which exhaust field the message actually contains:
//    1. print.device.airduct.parts[func==2].state  — 0-100   (H2 series and newer)
//    2. print.big_fan2_speed                       — 0-15    (X1/P1/A1; H2C sends it too)
//    3. print.exhaust_fan_speed                    — 0-15    (legacy, rarely seen)
//  The first one present wins, so a missing field never stops the fans.
// ══════════════════════════════════════════════════════════════════════════

// The filter tells ArduinoJson which fields to keep.  Everything else in the
// ~30 KB report is skipped while parsing, so the document stays a few hundred
// bytes instead of tens of KB.  The [0] entry of an array filter applies to
// every element of that array.
const JsonDocument& reportFilter() {
  static JsonDocument filter;
  static bool built = false;
  if (!built) {
    for (JsonObject p : { filter["print"].to<JsonObject>(), filter.as<JsonObject>() }) {
      p["gcode_state"]       = true;
      p["mc_percent"]        = true;
      p["mc_remaining_time"] = true;
      p["layer_num"]         = true;
      p["big_fan2_speed"]    = true;
      p["exhaust_fan_speed"] = true;
      JsonObject part = p["device"]["airduct"]["parts"][0].to<JsonObject>();
      part["func"]  = true;
      part["state"] = true;
    }
    built = true;
  }
  return filter;
}

// Bambu sends some numbers as strings ("4") and some as numbers (4).
// Handle both explicitly rather than trusting an implicit conversion.
int jsonInt(JsonVariantConst v) {
  if (v.is<const char*>()) return atoi(v.as<const char*>());
  return v.as<int>();
}

int scale0to15(int raw) {   // 0-15 fan steps -> 0-100%, same curve as before
  return (raw <= 0) ? 0 : 10 + (constrain(raw, 0, 15) * 90) / 15;
}

// TReader is the PayloadReader on the ESP32 (or a plain string in the host tests)
template <typename TReader>
void parseReport(TReader& in, PrinterState& s) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, in, DeserializationOption::Filter(reportFilter()));
  if (err) { Serial.printf("[MQTT] JSON parse error: %s\n", err.c_str()); return; }

  // Normally everything is under "print"; some messages put it at the root
  JsonObject p = doc["print"].is<JsonObject>() ? doc["print"].as<JsonObject>()
                                               : doc.as<JsonObject>();
  if (p.isNull() || p.size() == 0) return;   // nothing we care about (e.g. an info reply)

  s.last_seen = millis();
  s.has_data  = true;

  bool hadState = !p["gcode_state"].isNull();
  if (hadState)
    strlcpy(s.gcode_state, p["gcode_state"] | "IDLE", sizeof(s.gcode_state));
  else if (strcmp(s.gcode_state, "offline") == 0)
    strlcpy(s.gcode_state, "IDLE", sizeof(s.gcode_state));

  if (!p["mc_percent"].isNull())        s.progress      = jsonInt(p["mc_percent"]);
  if (!p["mc_remaining_time"].isNull()) s.remaining_min = jsonInt(p["mc_remaining_time"]);

  // Partial messages (P1/A1 send changes only) may omit gcode_state —
  // infer "printing" from progress/layer if we have nothing better.
  if (!hadState && strcmp(s.gcode_state, "IDLE") == 0) {
    bool midPrint = (s.progress > 0 && s.progress < 100) ||
                    (!p["layer_num"].isNull() && jsonInt(p["layer_num"]) > 0);
    if (midPrint) strlcpy(s.gcode_state, "RUNNING", sizeof(s.gcode_state));
  }

  int pct = -1;
  JsonArray parts = p["device"]["airduct"]["parts"];
  for (JsonObject part : parts) {
    if (jsonInt(part["func"]) == 2) { pct = constrain(jsonInt(part["state"]), 0, 100); break; }
  }
  if (pct < 0 && !p["big_fan2_speed"].isNull())    pct = scale0to15(jsonInt(p["big_fan2_speed"]));
  if (pct < 0 && !p["exhaust_fan_speed"].isNull()) pct = scale0to15(jsonInt(p["exhaust_fan_speed"]));

  if (pct >= 0) {
    s.exhaust_pct = pct;
    s.exhausting  = pct > 0;
  }
}

// ══════════════════════════════════════════════════════════════════════════
//  SSDP discovery — find Bambu printers on the local network
//
//  We send one M-SEARCH to the SSDP multicast group.  Each printer answers
//  with a small HTTP-style reply sent straight back (unicast) to the port we
//  sent from, so the same UDP socket both sends and listens.  Useful headers:
//    Location: <ip>   USN: <serial>   DevName.bambu.com: <name>
//    DevModel.bambu.com: <model code, e.g. O1C2 = H2C>
// ══════════════════════════════════════════════════════════════════════════

struct FoundPrinter {
  char ip[16]     = "";
  char serial[32] = "";
  char name[64]   = "";
  char model[24]  = "";
};

// Copy the value of "Header: value" if this line starts with that header (case-insensitive)
bool headerValue(const char* line, const char* header, char* out, size_t outLen) {
  size_t hl = strlen(header);
  if (strncasecmp(line, header, hl) != 0 || line[hl] != ':') return false;
  const char* v = line + hl + 1;
  while (*v == ' ') v++;
  strlcpy(out, v, outLen);
  return true;
}

int discoverPrinters(FoundPrinter* found, int maxFound, unsigned long waitMs = SSDP_WAIT_MS) {
  WiFiUDP udp;
  if (!udp.begin(0)) { Serial.println("[SSDP] Could not open UDP socket"); return 0; }   // 0 = any free port

  const char* msearch =
    "M-SEARCH * HTTP/1.1\r\n"
    "HOST: 239.255.255.250:1900\r\n"
    "MAN: \"ssdp:discover\"\r\n"
    "MX: 2\r\n"
    "ST: urn:bambulab-com:device:3dprinter:1\r\n\r\n";
  for (uint16_t port : { (uint16_t)SSDP_PORT_PRIMARY, (uint16_t)SSDP_PORT_ALT }) {
    udp.beginPacket(SSDP_MULTICAST_IP, port);
    udp.write((const uint8_t*)msearch, strlen(msearch));
    udp.endPacket();
  }

  int count = 0;
  char buf[1024];
  unsigned long start = millis();
  while (millis() - start < waitMs) {
    int len = udp.parsePacket();
    if (len <= 0) { delay(10); continue; }
    len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) continue;
    buf[len] = '\0';
    if (strncmp(buf, "M-SEARCH", 8) == 0) continue;   // someone else's search, not a printer

    FoundPrinter fp;
    bool isBambu = false;
    for (char* line = strtok(buf, "\r\n"); line; line = strtok(nullptr, "\r\n")) {
      headerValue(line, "Location", fp.ip, sizeof(fp.ip));
      headerValue(line, "USN", fp.serial, sizeof(fp.serial));
      headerValue(line, "DevName.bambu.com", fp.name, sizeof(fp.name));
      if (headerValue(line, "DevModel.bambu.com", fp.model, sizeof(fp.model))) isBambu = true;
    }
    if (!isBambu || !strlen(fp.serial)) continue;
    if (!strlen(fp.ip)) strlcpy(fp.ip, udp.remoteIP().toString().c_str(), sizeof(fp.ip));

    bool dup = false;
    for (int i = 0; i < count; i++) if (strcmp(found[i].serial, fp.serial) == 0) dup = true;
    if (!dup && count < maxFound) {
      found[count++] = fp;
      Serial.printf("[SSDP] Found %s (%s) at %s, model %s\n", fp.name, fp.serial, fp.ip, fp.model);
    }
  }
  udp.stop();
  return count;
}

// Self-heal: look for this printer's serial number and adopt its current IP.
// Returns true if the IP was found (changed or not).
bool rediscoverPrinter(int i) {
  PrinterLink& L = links[i];
  L.lastDiscovery = millis();
  Serial.printf("[SSDP] Searching for printer %d (%s)...\n", i + 1, cfg.printer[i].serial);

  FoundPrinter found[SSDP_MAX_RESULTS];
  int n = discoverPrinters(found, SSDP_MAX_RESULTS);
  for (int k = 0; k < n; k++) {
    if (strcasecmp(found[k].serial, cfg.printer[i].serial) != 0) continue;
    if (strcmp(found[k].ip, cfg.printer[i].ip) != 0) {
      Serial.printf("[SSDP] Printer %d moved: %s -> %s — saving new address\n",
                    i + 1, strlen(cfg.printer[i].ip) ? cfg.printer[i].ip : "(none)", found[k].ip);
      strlcpy(cfg.printer[i].ip, found[k].ip, sizeof(cfg.printer[i].ip));
      saveConfig();
    }
    return true;
  }
  Serial.printf("[SSDP] Printer %d not found — is it powered on and on this network?\n", i + 1);
  return false;
}

// ══════════════════════════════════════════════════════════════════════════
//  MQTT connection & loop — one link per printer
// ══════════════════════════════════════════════════════════════════════════

void requestPushAll(int i) {
  char topic[80];
  snprintf(topic, sizeof(topic), "device/%s/request", cfg.printer[i].serial);
  if (!links[i].mqtt.publish(topic, "{\"pushing\":{\"command\":\"pushall\"}}"))
    Serial.printf("[MQTT] pushall failed for printer %d\n", i + 1);
  links[i].lastPushall = millis();
}

void resetPrinterState(int i) {
  links[i].state = PrinterState();
  links[i].warnedNoData = false;
}

bool connectPrinter(int i) {
  PrinterLink& L = links[i];
  PrinterConfig& pc = cfg.printer[i];

  Serial.printf("[MQTT] Printer %d: connecting to %s...\n", i + 1, pc.ip);
  char clientId[80];
  snprintf(clientId, sizeof(clientId), "%s-p%d", cfg.device_name, i + 1);   // must be unique per connection

  if (!L.mqtt.connect(pc.ip, BAMBU_LOCAL_PORT, clientId, BAMBU_LOCAL_USER, pc.access_code)) {
    L.lastRc = L.mqtt.lastRc;
    const char* why = (L.lastRc == 4 || L.lastRc == 5) ? "access code rejected" :
                      (L.lastRc <= -2)                 ? "printer unreachable"  : "refused";
    Serial.printf("[MQTT] Printer %d: connect failed, rc=%d (%s)\n", i + 1, L.lastRc, why);
    return false;
  }

  L.lastRc = 0;
  char topic[80];
  snprintf(topic, sizeof(topic), "device/%s/report", pc.serial);
  bool ok = L.mqtt.subscribe(topic);
  Serial.printf("[MQTT] Printer %d: connected, subscribing to %s — %s\n", i + 1, topic, ok ? "sent" : "FAILED");
  Serial.printf("[Mem] Free heap %u bytes, largest block %u bytes\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  resetPrinterState(i);
  L.wasConnected = true;
  L.connectedAt  = millis();
  requestPushAll(i);
  return true;
}

void initPrinterLink(int i) {
  PrinterLink& L = links[i];
  L.mqtt.index = i;
  L.mqtt.tls.setInsecure();                    // printers use a self-signed certificate
  L.mqtt.tls.setTimeout(TLS_CONNECT_TIMEOUT_S);
  L.mqtt.tls.setHandshakeTimeout(TLS_HANDSHAKE_TIMEOUT_S);
  L.mqtt.onPublish = [i](PayloadReader& in) {
    parseReport(in, links[i].state);           // parsed as it streams in — the message is never stored
    updateFanSpeed();
  };

  L.wasConnected = false;
  L.failCount    = 0;
  L.lastRc       = 0;
  L.retryDelay   = MQTT_RECONNECT_MIN_MS;
  L.lastAttempt  = millis() - MQTT_RECONNECT_MIN_MS;   // try right away
  resetPrinterState(i);
}

void initPrinters() {
  for (int i = 0; i < PRINTER_COUNT; i++) {
    initPrinterLink(i);
    if (!printerConfigured(i)) Serial.printf("[MQTT] Printer %d not configured — skipping\n", i + 1);
  }
}

void reinitPrinters() {
  Serial.println("[MQTT] Reinitialising printer connections...");
  for (int i = 0; i < PRINTER_COUNT; i++)
    if (links[i].mqtt.connected()) links[i].mqtt.disconnect();
  initPrinters();
  updateFanSpeed();
}

void loopPrinter(int i) {
  PrinterLink& L = links[i];
  if (!printerConfigured(i)) return;
  unsigned long now = millis();

  if (L.mqtt.connected()) {
    L.mqtt.loop();
    if (!L.mqtt.connected()) return;           // dropped during loop() — handled on the next pass

    // Take a FRESH millis() here.  loop() may have just stamped last_seen, and
    // the "now" captured above is older — older minus newer on an unsigned
    // long wraps around to ~4.29 billion ms, which looked like 49 days of silence.
    unsigned long t = millis();
    if (t - L.lastPushall > PUSHALL_INTERVAL_MS) requestPushAll(i);

    unsigned long silentFor = t - (L.state.has_data ? L.state.last_seen : L.connectedAt);
    if (!L.state.has_data && !L.warnedNoData && silentFor > NO_DATA_WARN_MS) {
      L.warnedNoData = true;
      Serial.printf("[MQTT] Printer %d: connected but no report yet — check the serial number "
                    "(a wrong serial still subscribes fine)\n", i + 1);
    }
    if (silentFor > STALE_DATA_MS) {
      Serial.printf("[MQTT] Printer %d: no data for %lus — reconnecting\n", i + 1, silentFor / 1000);
      L.mqtt.disconnect();
    }
    return;
  }

  // Just lost the connection — clear the state so the fans stop following stale data
  if (L.wasConnected) {
    L.wasConnected = false;
    Serial.printf("[MQTT] Printer %d: disconnected\n", i + 1);
    resetPrinterState(i);
    updateFanSpeed();
  }

  if (now - L.lastAttempt < L.retryDelay) return;
  L.lastAttempt = now;

  // No IP yet (e.g. upgraded from the cloud firmware) — go straight to discovery
  if (!strlen(cfg.printer[i].ip)) {
    if (now - L.lastDiscovery >= REDISCOVER_MIN_GAP_MS || L.lastDiscovery == 0) rediscoverPrinter(i);
    if (!strlen(cfg.printer[i].ip)) return;
  }

  if (connectPrinter(i)) {
    L.failCount  = 0;
    L.retryDelay = MQTT_RECONNECT_MIN_MS;
    return;
  }

  L.failCount++;
  L.retryDelay = min((unsigned long)MQTT_RECONNECT_MAX_MS, L.retryDelay * 2);

  // Self-heal: the printer may simply have a new IP address from DHCP.
  // A wrong access code (rc 4/5) is not an addressing problem, so skip it.
  bool authProblem = (L.lastRc == 4 || L.lastRc == 5);
  if (!authProblem && L.failCount >= REDISCOVER_AFTER_FAILS &&
      (L.lastDiscovery == 0 || millis() - L.lastDiscovery >= REDISCOVER_MIN_GAP_MS)) {
    char oldIp[16];
    strlcpy(oldIp, cfg.printer[i].ip, sizeof(oldIp));
    if (rediscoverPrinter(i) && strcmp(oldIp, cfg.printer[i].ip) != 0) {
      L.failCount   = 0;
      L.retryDelay  = MQTT_RECONNECT_MIN_MS;
      L.lastAttempt = millis() - MQTT_RECONNECT_MIN_MS;   // reconnect on the next pass
    }
  }
}

void loopPrinters() {
  for (int i = 0; i < PRINTER_COUNT; i++) loopPrinter(i);
}

// ══════════════════════════════════════════════════════════════════════════
//  Filesystem helpers
// ══════════════════════════════════════════════════════════════════════════

bool configExists() { return LittleFS.exists(CONFIG_FILE); }

// Keys are printer1_name, printer1_ip, ... — name and serial keep the same
// keys as the cloud firmware, so they carry over after an upgrade.
String pkey(int i, const char* field) { return "printer" + String(i + 1) + "_" + field; }

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
  for (int i = 0; i < PRINTER_COUNT; i++) {
    PrinterConfig& pc = cfg.printer[i];
    strlcpy(pc.name,        doc[pkey(i, "name")]        | "", sizeof(pc.name));
    strlcpy(pc.ip,          doc[pkey(i, "ip")]          | "", sizeof(pc.ip));
    strlcpy(pc.serial,      doc[pkey(i, "serial")]      | "", sizeof(pc.serial));
    strlcpy(pc.access_code, doc[pkey(i, "access_code")] | "", sizeof(pc.access_code));
  }
  cfg.fan_speed_printing        = doc["fan_speed_printing"]        | 10;
  cfg.fan_speed_one_exhausting  = doc["fan_speed_one_exhausting"]  | 50;
  cfg.fan_speed_both_exhausting = doc["fan_speed_both_exhausting"] | 100;
  cfg.fan_runon_s               = constrain((int)(doc["fan_runon_s"] | 0), 0, FAN_RUNON_MAX_S);
  cfg.manual_knob               = doc["manual_knob"] | false;
  return true;
}

bool saveConfig() {
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f) return false;
  JsonDocument doc;
  doc["wifi_ssid"]      = cfg.wifi_ssid;
  doc["wifi_pass"]      = cfg.wifi_pass;
  doc["device_name"]    = cfg.device_name;
  doc["dashboard_pass"] = cfg.dashboard_pass;
  for (int i = 0; i < PRINTER_COUNT; i++) {
    doc[pkey(i, "name")]        = cfg.printer[i].name;
    doc[pkey(i, "ip")]          = cfg.printer[i].ip;
    doc[pkey(i, "serial")]      = cfg.printer[i].serial;
    doc[pkey(i, "access_code")] = cfg.printer[i].access_code;
  }
  doc["fan_speed_printing"]        = cfg.fan_speed_printing;
  doc["fan_speed_one_exhausting"]  = cfg.fan_speed_one_exhausting;
  doc["fan_speed_both_exhausting"] = cfg.fan_speed_both_exhausting;
  doc["fan_runon_s"]               = cfg.fan_runon_s;
  doc["manual_knob"]               = cfg.manual_knob;
  serializeJsonPretty(doc, f);
  f.close();
  return true;
}

void deleteConfig() {
  if (LittleFS.exists(CONFIG_FILE)) { LittleFS.remove(CONFIG_FILE); Serial.println("[Config] Deleted"); }
}

// ══════════════════════════════════════════════════════════════════════════
//  Runtime tracking helpers
// ══════════════════════════════════════════════════════════════════════════

void saveRuntime();  // forward declaration

void loadRuntime() {
  if (!LittleFS.exists(RUNTIME_FILE)) {
    saveRuntime();  // create file with default value on first boot
    return;
  }
  File f = LittleFS.open(RUNTIME_FILE, "r");
  if (!f) return;
  JsonDocument doc;
  if (!deserializeJson(doc, f)) runtimeMinutes = doc["runtime_minutes"] | 0;
  f.close();
}

void saveRuntime() {
  File f = LittleFS.open(RUNTIME_FILE, "w", true);  // true = create if not exists
  if (!f) { Serial.println("[Runtime] Failed to open runtime.json for writing"); return; }
  JsonDocument doc;
  doc["runtime_minutes"] = runtimeMinutes;
  serializeJson(doc, f);
  f.close();
  lastRuntimeSave = millis();
}

void resetRuntime() {
  runtimeMinutes   = 0;
  runtimeSecondAcc = 0;
  saveRuntime();
}

// ══════════════════════════════════════════════════════════════════════════
//  HTML helpers
// ══════════════════════════════════════════════════════════════════════════

// Stream the page straight from flash in small chunks instead of loading the
// whole file (up to ~18 KB) into a String first — RAM is tight with two
// printer connections open.
void sendHtmlFile(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) {
    Serial.printf("[Web] %s is missing — run Platform -> Upload Filesystem Image in PlatformIO\n", path);
    server.send(500, "text/html",
      "<h2>Web files missing</h2><p>The controller's web pages were not found.  In PlatformIO run "
      "<b>Platform &rarr; Upload Filesystem Image</b>, then restart the controller.</p>");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

// ══════════════════════════════════════════════════════════════════════════
//  Config Portal (AP mode)
// ══════════════════════════════════════════════════════════════════════════

void handlePortalRoot() { sendHtmlFile("/setup.html"); }

void handlePortalSave() {
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return; }
  if (server.hasArg("wifi_ssid"))      strlcpy(cfg.wifi_ssid,      server.arg("wifi_ssid").c_str(),      sizeof(cfg.wifi_ssid));
  if (server.hasArg("wifi_pass"))      strlcpy(cfg.wifi_pass,      server.arg("wifi_pass").c_str(),      sizeof(cfg.wifi_pass));
  if (server.hasArg("device_name"))    strlcpy(cfg.device_name,    server.arg("device_name").c_str(),    sizeof(cfg.device_name));
  if (server.hasArg("dashboard_pass")) strlcpy(cfg.dashboard_pass, server.arg("dashboard_pass").c_str(), sizeof(cfg.dashboard_pass));
  if (saveConfig()) {
    sendHtmlFile("/saved.html");
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
    // Built with ArduinoJson so names with quotes or backslashes stay valid JSON.
    // Mesh systems list the same name once per access point — keep the strongest.
    int n = WiFi.scanNetworks();
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; i++) {
      String ssid = WiFi.SSID(i);
      if (!ssid.length()) continue;                   // hidden network
      JsonObject seen;
      for (JsonObject o : arr) if (ssid == o["ssid"].as<const char*>()) { seen = o; break; }
      if (!seen.isNull()) {
        if (WiFi.RSSI(i) > seen["rssi"].as<int>()) seen["rssi"] = WiFi.RSSI(i);
        continue;
      }
      JsonObject o = arr.add<JsonObject>();
      o["ssid"]    = ssid;
      o["rssi"]    = WiFi.RSSI(i);
      o["secured"] = WiFi.encryptionType(i) != WIFI_AUTH_OPEN;
    }
    WiFi.scanDelete();
    String json; serializeJson(doc, json);
    server.send(200, "application/json", json);
  });
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();
  Serial.println("[Portal] Waiting for configuration...");
  while (true) { server.handleClient(); delay(200); }
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

// Every POST from our own pages carries this header.  The browser re-sends the
// saved login to the controller even from another website's form, but another
// website cannot add a custom header, so this blocks cross-site requests (CSRF).
#define CSRF_HEADER "X-BambuAFE"

bool checkPost() {
  if (server.method() != HTTP_POST) { server.send(405, "text/plain", "Method Not Allowed"); return false; }
  if (server.header(CSRF_HEADER) != "1") { server.send(403, "text/plain", "Forbidden"); return false; }
  return true;
}

void handleDashboard()  { if (!checkAuth()) return; sendHtmlFile("/index.html"); }
void handleConfigPage() { if (!checkAuth()) return; sendHtmlFile("/config.html"); }

const char* linkStatusText(int i) {
  if (!printerConfigured(i))            return "not configured";
  if (links[i].mqtt.connected())        return links[i].state.has_data ? "connected" : "connected, waiting for data";
  if (links[i].lastRc == 4 || links[i].lastRc == 5) return "access code rejected";
  if (!strlen(cfg.printer[i].ip))       return "searching for printer";
  return "reconnecting";
}

void handleStatus() {
  if (!checkAuth()) return;
  JsonDocument doc;
  doc["device_name"]      = cfg.device_name;
  doc["firmware_version"] = FIRMWARE_VERSION;
  doc["uptime_s"]         = millis() / 1000;
  doc["free_heap"]        = ESP.getFreeHeap();
  doc["max_alloc_heap"]   = ESP.getMaxAllocHeap();
  doc["ip"]               = WiFi.localIP().toString();
  doc["fan_pct"]          = currentFanPct;
  JsonArray arr = doc["printer_links"].to<JsonArray>();
  for (int i = 0; i < PRINTER_COUNT; i++) {
    JsonObject o = arr.add<JsonObject>();
    o["configured"] = printerConfigured(i);
    o["connected"]  = links[i].mqtt.connected();
    o["status"]     = linkStatusText(i);
    o["ip"]         = cfg.printer[i].ip;
  }
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handlePrinterStatus() {
  if (!checkAuth()) return;
  JsonDocument doc;
  doc["fan_pct"]         = currentFanPct;
  doc["manual_override"] = (readPotentiometer() >= 0);
  for (int i = 0; i < PRINTER_COUNT; i++) {
    if (!strlen(cfg.printer[i].serial)) continue;
    const PrinterState& s = links[i].state;
    JsonObject obj = doc["printer" + String(i + 1)].to<JsonObject>();
    obj["name"]        = strlen(cfg.printer[i].name) ? cfg.printer[i].name : (i == 0 ? "Printer 1" : "Printer 2");
    obj["connected"]   = printerLive(i);
    obj["gcode_state"] = s.gcode_state;
    obj["progress"]    = s.progress;
    bool noTimeState = (strcmp(s.gcode_state, "FINISH") == 0 ||
                        strcmp(s.gcode_state, "IDLE") == 0 ||
                        strcmp(s.gcode_state, "offline") == 0);
    obj["remaining_min"] = noTimeState ? 0 : s.remaining_min;
    obj["exhausting"]    = s.exhausting;
    obj["exhaust_pct"]   = s.exhaust_pct;
  }
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleConfigGet() {
  if (!checkAuth()) return;
  JsonDocument doc;
  for (int i = 0; i < PRINTER_COUNT; i++) {
    doc[pkey(i, "name")]            = cfg.printer[i].name;
    doc[pkey(i, "ip")]              = cfg.printer[i].ip;
    doc[pkey(i, "serial")]          = cfg.printer[i].serial;
    doc[pkey(i, "has_access_code")] = strlen(cfg.printer[i].access_code) > 0;   // never send the code itself back
  }
  doc["fan_speed_printing"]        = cfg.fan_speed_printing;
  doc["fan_speed_one_exhausting"]  = cfg.fan_speed_one_exhausting;
  doc["fan_speed_both_exhausting"] = cfg.fan_speed_both_exhausting;
  doc["fan_runon_s"]               = cfg.fan_runon_s;
  doc["manual_knob"]               = cfg.manual_knob;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleConfigPost() {
  if (!checkAuth() || !checkPost()) return;

  for (int i = 0; i < PRINTER_COUNT; i++) {
    PrinterConfig& pc = cfg.printer[i];
    String k;
    if (server.hasArg(k = pkey(i, "name")))   strlcpy(pc.name,   server.arg(k).c_str(), sizeof(pc.name));
    if (server.hasArg(k = pkey(i, "ip")))     strlcpy(pc.ip,     server.arg(k).c_str(), sizeof(pc.ip));
    if (server.hasArg(k = pkey(i, "serial"))) {
      // A different printer in this slot — the saved code belonged to the old one
      if (strcasecmp(pc.serial, server.arg(k).c_str()) != 0) pc.access_code[0] = '\0';
      strlcpy(pc.serial, server.arg(k).c_str(), sizeof(pc.serial));
    }
    // Blank = keep the saved code (the page never receives it).  "clear" via an empty serial instead.
    if (server.hasArg(k = pkey(i, "access_code")) && server.arg(k).length() > 0)
      strlcpy(pc.access_code, server.arg(k).c_str(), sizeof(pc.access_code));
    if (!strlen(pc.serial)) pc.access_code[0] = '\0';   // printer removed — forget its code too
  }
  if (server.hasArg("fan_speed_printing")) {
    int v = server.arg("fan_speed_printing").toInt();
    cfg.fan_speed_printing = (v == 0) ? 0 : constrain(v, 8, 100);
  }
  if (server.hasArg("fan_speed_one_exhausting")) {
    int v = server.arg("fan_speed_one_exhausting").toInt();
    cfg.fan_speed_one_exhausting = (v == 0) ? 0 : constrain(v, 8, 100);
  }
  if (server.hasArg("fan_speed_both_exhausting")) {
    int v = server.arg("fan_speed_both_exhausting").toInt();
    cfg.fan_speed_both_exhausting = (v == 0) ? 0 : constrain(v, 8, 100);
  }
  if (server.hasArg("fan_runon_s"))
    cfg.fan_runon_s = constrain((int)server.arg("fan_runon_s").toInt(), 0, FAN_RUNON_MAX_S);
  if (server.hasArg("manual_knob"))
    cfg.manual_knob = server.arg("manual_knob") == "1";

  if (saveConfig()) {
    server.send(200, "application/json", "{\"ok\":true}");   // reply first — reconnecting takes a few seconds
    reinitPrinters();
  } else {
    server.send(500, "application/json", "{\"ok\":false,\"error\":\"Save failed\"}");
  }
}

void handleDiscover() {
  if (!checkAuth()) return;
  FoundPrinter found[SSDP_MAX_RESULTS];
  int n = discoverPrinters(found, SSDP_MAX_RESULTS);
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int k = 0; k < n; k++) {
    JsonObject o = arr.add<JsonObject>();
    o["ip"]     = found[k].ip;
    o["serial"] = found[k].serial;
    o["name"]   = found[k].name;
    o["model"]  = found[k].model;
  }
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleResetPost() {
  if (!checkAuth() || !checkPost()) return;
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500); deleteConfig(); ESP.restart();
}

void handleReboot() {
  if (!checkAuth() || !checkPost()) return;
  server.send(200, "application/json", "{\"ok\":true}");
  delay(500); ESP.restart();
}

void handleRuntimeGet() {
  if (!checkAuth()) return;
  JsonDocument doc;
  doc["hours"]   = runtimeMinutes / 60;
  doc["minutes"] = runtimeMinutes % 60;
  doc["total_minutes"] = runtimeMinutes;
  String json; serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleRuntimeReset() {
  if (!checkAuth() || !checkPost()) return;
  resetRuntime();
  server.send(200, "application/json", "{\"ok\":true}");
}

void startNormalServer() {
  server.on("/",               HTTP_GET,  handleDashboard);
  server.on("/config.html",    HTTP_GET,  handleConfigPage);
  server.on("/status",         HTTP_GET,  handleStatus);
  server.on("/printer-status", HTTP_GET,  handlePrinterStatus);
  server.on("/config",         HTTP_GET,  handleConfigGet);
  server.on("/config",         HTTP_POST, handleConfigPost);
  server.on("/discover",       HTTP_GET,  handleDiscover);
  server.on("/reset",          HTTP_POST, handleResetPost);
  server.on("/reboot",         HTTP_POST, handleReboot);
  server.on("/runtime",        HTTP_GET,  handleRuntimeGet);
  server.on("/runtime/reset",  HTTP_POST, handleRuntimeReset);
  server.onNotFound([]() {
    if (!checkAuth()) return;
    // Only web page files.  config.json holds the Wi-Fi password and access
    // codes in plain text, so .json and anything else unknown is never served.
    String path = server.uri();
    const char* ct = nullptr;
    if      (path.endsWith(".html")) ct = "text/html";
    else if (path.endsWith(".js"))   ct = "application/javascript";
    else if (path.endsWith(".css"))  ct = "text/css";
    else if (path.endsWith(".ico"))  ct = "image/x-icon";
    if (ct && LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      server.streamFile(f, ct);
      f.close();
    } else {
      server.send(404, "text/plain", "Not found: " + path);
    }
  });
  const char* headers[] = { CSRF_HEADER };
  server.collectHeaders(headers, 1);
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
  pinMode(POT_PIN, INPUT);

  if (!LittleFS.begin(true)) { Serial.println("[FS] Mount failed!"); return; }
  Serial.println("[FS] LittleFS mounted");

  if (resetButtonHeld()) { Serial.println("[Reset] Wiping config"); deleteConfig(); }

  if (!configExists() || !loadConfig()) startConfigPortal();

  if (!connectWiFi()) {
    Serial.println("[Boot] Wi-Fi failed — restarting...");
    ESP.restart();
  }

  startNormalServer();

  if (MDNS.begin(cfg.device_name)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mDNS] Reachable at http://%s.local/\n", cfg.device_name);
  }

  initPrinters();
  loadRuntime();

  Serial.printf("[Boot] Running as: %s\n", cfg.device_name);
}

void loopRuntime() {
  unsigned long now = millis();

  // Accumulate milliseconds when fan is above 5%
  if (currentFanPct > 5) {
    if (lastRuntimeTick == 0) lastRuntimeTick = now;
    unsigned long elapsed = now - lastRuntimeTick;
    lastRuntimeTick = now;
    runtimeSecondAcc += elapsed;
    while (runtimeSecondAcc >= 60000) {
      runtimeMinutes++;
      runtimeSecondAcc -= 60000;
    }
  } else {
    lastRuntimeTick = 0;
  }

  // Save every 10 minutes
  if (lastRuntimeSave == 0) lastRuntimeSave = now;
  if (now - lastRuntimeSave >= RUNTIME_SAVE_MS) saveRuntime();
}

void loop() {
  static unsigned long lastFanCheck = 0;

  server.handleClient();
  loopPrinters();
  loopRuntime();

  // Re-evaluate the fans regularly, not only when a printer message arrives —
  // otherwise the manual-override knob does nothing when no printer is talking.
  if (millis() - lastFanCheck >= 250) {
    lastFanCheck = millis();
    updateFanSpeed();
  }
}
