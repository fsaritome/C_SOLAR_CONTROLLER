/**
 * ESP8266 Solar Monitor & Inverter Controller
 * 
 * This program monitors a solar system's status by querying metrics from another ESP device
 * and controls an inverter based on battery and MPPT status.
 * It also provides a web server with endpoints for metrics and inverter control.
 * Control (POST /on, /off, /unlock) needs ?token=<API_TOKEN from secrets.h>; telnet is read-only.
 * Optional AUTO_TOKEN (secrets.h) is accepted only by POST /on: an automation (e.g. a mains-outage handler) can
 * switch on through a SOFT lock but can never unlock, lock, or pass a HARD lock. Without AUTO_TOKEN it is disabled.
 *
 * Locks (inverter_lock metric): 0 NONE, 1 SOFT, 2 HARD.
 *  - HARD = a safety condition holds (battery alarm, MPPT error/fault, low battery, no fresh solar2 data, Wi-Fi
 *    lost). The inverter is off and nothing switches it on. When the condition clears it drops to SOFT, never NONE.
 *  - SOFT = after boot, after /off, after a hard lock cleared. Operator /on is refused until /unlock;
 *    the AUTO token may switch on.
 *  - NONE = after /unlock (operator token). Operator /on works.
 *  - POST /hardlock (operator token): inverter off + HARD lock "operator hard lock" (maintenance / tests); only
 *    POST /hardunlock (operator token) lifts it, not conditions clearing. Kept across restarts: a flag file on
 *    LittleFS, read at boot before anything else. If the file system can't be mounted the controller boots HARD
 *    ("lock storage error"); /hardunlock clears that too (operator decision). Unlock lifts the RAM lock only after
 *    the file is gone (fail-safe), so a failed delete keeps the lock.
 */

// --- Includes ---
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Ticker.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <TelnetStream.h>
#include <LittleFS.h>
#include "secrets.h"  // Wi-Fi and OTA passwords live here, outside git (see secrets.example.h)
#ifndef AUTO_TOKEN
#define AUTO_TOKEN ""  // empty = the on-only automation token is disabled
#endif

// Where solar2's Prometheus metrics are read. Set it in secrets.h to solar2's fixed IP, so a DNS outage can't
// stop the readings (and with them the inverter); the name below is only the fallback.
#ifndef SOLAR2_METRICS_URL
#define SOLAR2_METRICS_URL "http://solar2/metrics"
#endif

// --- Pin Definitions ---
#define INVERTER_PIN D7  // GPIO pin connected to the inverter control

// --- Network Configuration ---
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const uint16_t SERVER_PORT = 8664;

// --- Global Variables ---
enum : uint8_t { LOCK_NONE = 0, LOCK_SOFT = 1, LOCK_HARD = 2 };
uint8_t lockLevel = LOCK_HARD;        // boot: hard until the first fresh solar2 data, then soft
String lockReason = "boot: no solar2 data yet";
bool lowBatteryLatched = false;       // set below SOC/voltage shutdown, cleared above the recover thresholds
bool safetyAlarm = false;             // battery alarm or MPPT error/fault in the last solar2 read
volatile bool wifiLost = false;       // Wi-Fi down for WIFI_TIMEOUT (set by the ticker)
bool operatorHardLock = false;        // POST /hardlock; only POST /hardunlock clears it (persistent, see below)
const char *HARDLOCK_FILE = "/hardlock";
bool lockStoreOk = false;             // LittleFS mounted
bool lockStoreError = false;          // mount failed at boot: HARD until the operator clears it
bool hardlockStored = false;          // the flag file exists
volatile bool pendingHardlockWrite = false;  // set by /hardlock, done in loop() (no flash writes in web callbacks)
volatile bool pendingHardlockClear = false;  // set by /hardunlock, done in loop()
bool lastOnAuto = false;              // true if the last successful /on came with AUTO_TOKEN (automation)
String lastBatteryHasAlarm = "0";     // Last battery alarm value from solar2
int solar2ConnectionStatus = 0;       // Solar2 connection status (1 = reachable, 0 = unreachable)
int solar2ReadFailCount = 0;          // Counter for consecutive failed reads
const int MAX_CONSECUTIVE_FAILS = 10; // 10 x 500ms = 5 seconds max lag before shutdown
String lastMpptHasError = "0";        // Last MPPT has_error value from solar2
String lastMpptFaultState = "0";      // Last MPPT is_in_fault_state value
String lastMpptState = "";            // Last MPPT state text (e.g. "Float")
String lastBatteryVoltage = "0";      // Last battery voltage (V)
String lastBatteryCurrent = "0";      // Last battery current (A)
String lastStateOfCharge = "0";       // Last state of charge (%)
String lastPvPower = "0";             // Last MPPT PV power (W)
unsigned long lastSuccessfulRead = 0; // Timestamp of last successful solar2 data read

// WiFi monitoring variables
Ticker wifiCheckTicker;              // Ticker for periodic WiFi checks
unsigned long wifiDisconnectTime = 0; // Time when WiFi disconnection was detected
const unsigned long WIFI_TIMEOUT = 10000;       // 10s WiFi disconnect timeout before shutdown
const unsigned long DATA_STALE_TIMEOUT = 30000; // 30s without fresh solar2 data → shutdown

// Low-battery cutoff thresholds
const float SOC_SHUTDOWN_THRESHOLD    = 20.0f;  // Shut down inverter below this state of charge (%)
const float SOC_RECOVER_THRESHOLD     = 25.0f;  // Alarm clears only when SoC rises above this (%)
const float VOLTAGE_SHUTDOWN_THRESHOLD = 11.8f; // Shut down below this battery voltage (V)
const float VOLTAGE_RECOVER_THRESHOLD  = 12.2f; // Alarm clears only when voltage rises above this (V)

// --- Web Server ---
AsyncWebServer server(SERVER_PORT);

// --- Function Prototypes ---
void printWiFiNetworks();
void checkBatteryAlarmAndShutdown();
void setupWebServer();

/**
 * Helper function to log messages to both Serial and Telnet
 */
void log(const String &message) {
  Serial.println(message);
  TelnetStream.print(message);
  TelnetStream.print("\r\n"); // Proper telnet line ending
}

void logf(const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
  TelnetStream.print(buffer);
  if (strchr(buffer, '\n')) { // If the message contains a newline
    TelnetStream.print("\r"); // Add carriage return for proper telnet formatting
  }
}

/**
 * Scans and prints available WiFi networks to serial console
 * Helpful for debugging connection issues
 */
void printWiFiNetworks() {
  int n = WiFi.scanNetworks();
  Serial.println("Found WiFi networks:");
  for (int i = 0; i < n; ++i) {
    Serial.printf("  %2d: %s (RSSI: %d) %s\n", 
                  i + 1, 
                  WiFi.SSID(i).c_str(), 
                  WiFi.RSSI(i), 
                  WiFi.encryptionType(i) == ENC_TYPE_NONE ? "OPEN" : "SECURED");
  }
}

/**
 * Reads solar2's metrics once (called every 500 ms while Wi-Fi is up). Updates the readings and the safety flags.
 */
void readSolar2(unsigned long now) {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2000);
  http.begin(client, SOLAR2_METRICS_URL);
  yield();
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    http.end();
    if (solar2ReadFailCount < 1000) solar2ReadFailCount++;
    solar2ConnectionStatus = 0;
    return;
  }

  // Successful HTTP read
  solar2ReadFailCount = 0;
  solar2ConnectionStatus = 1;
  lastSuccessfulRead = now;
  String payload = http.getString();
  http.end();

  auto rawValue = [&](const String &key, String &dest) -> bool {
    int i = payload.indexOf(key);
    if (i == -1) return false;
    int vs = payload.indexOf('}', i);
    if (vs == -1) return false;
    vs++;
    while (vs < (int)payload.length() && isspace(payload[vs])) vs++;
    int ve = vs;
    while (ve < (int)payload.length() && !isspace(payload[ve])) ve++;
    dest = payload.substring(vs, ve);
    return true;
  };

  bool batteryAlarm = false, mpptError = false;
  if (rawValue("esphome_binary_sensor_value{id=\"battery_has_alarm\"", lastBatteryHasAlarm))
    batteryAlarm = lastBatteryHasAlarm.toFloat() >= 0.5f;
  if (rawValue("esphome_binary_sensor_value{id=\"mppt_has_error\"", lastMpptHasError))
    mpptError = lastMpptHasError.toFloat() >= 0.5f;
  if (rawValue("esphome_binary_sensor_value{id=\"mppt_is_in_fault_state\"", lastMpptFaultState))
    mpptError = mpptError || lastMpptFaultState.toFloat() >= 0.5f;
  safetyAlarm = batteryAlarm || mpptError;

  rawValue("esphome_sensor_value{id=\"battery_voltage\"", lastBatteryVoltage);
  rawValue("esphome_sensor_value{id=\"battery_current\"", lastBatteryCurrent);
  rawValue("esphome_sensor_value{id=\"state_of_charge\"", lastStateOfCharge);
  rawValue("esphome_sensor_value{id=\"mppt_pv_power\"", lastPvPower);

  // MPPT state text
  int si = payload.indexOf("esphome_text_sensor_value{id=\"mppt_state\"");
  if (si != -1) {
    int valAttr = payload.indexOf("value=\"", si);
    if (valAttr != -1) {
      valAttr += 7;
      int valEnd = payload.indexOf('"', valAttr);
      if (valEnd != -1) lastMpptState = payload.substring(valAttr, valEnd);
    }
  }

  // Low battery latch with hysteresis (0 = no reading: ignored, like before)
  float soc = lastStateOfCharge.toFloat();
  float voltage = lastBatteryVoltage.toFloat();
  bool lowNow = (soc > 0.0f && soc < SOC_SHUTDOWN_THRESHOLD) ||
                (voltage > 0.0f && voltage < VOLTAGE_SHUTDOWN_THRESHOLD);
  bool recovered = (soc <= 0.0f || soc >= SOC_RECOVER_THRESHOLD) &&
                   (voltage <= 0.0f || voltage >= VOLTAGE_RECOVER_THRESHOLD);
  if (lowNow) lowBatteryLatched = true;
  else if (lowBatteryLatched && recovered) lowBatteryLatched = false;
}

/**
 * The current safety condition, or "" if none. Any condition = HARD lock.
 */
String safetyReason(unsigned long now) {
  if (operatorHardLock) return "operator hard lock";
  if (lockStoreError) return "lock storage error";
  if (wifiLost) return "wifi lost";
  if (lastSuccessfulRead == 0) return "no solar2 data yet";
  if (now - lastSuccessfulRead >= DATA_STALE_TIMEOUT) return "solar2 data stale";
  if (solar2ReadFailCount >= MAX_CONSECUTIVE_FAILS) return "solar2 unreachable";
  if (lastBatteryHasAlarm.toFloat() >= 0.5f) return "battery alarm";
  if (safetyAlarm) return "mppt error or fault";
  if (lowBatteryLatched) return "low battery";  // fixed text: a changing reason would re-log and churn metric labels
  return "";
}

/**
 * Enforces the lock every 500 ms: a safety condition switches the inverter off and holds a HARD lock; when it
 * clears, the lock drops to SOFT (a person or the automation decides what happens next).
 */
void checkBatteryAlarmAndShutdown() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();
  if (now - lastCheck < 500) return;
  lastCheck = now;

  if (WiFi.status() == WL_CONNECTED) readSolar2(now);
  now = millis();

  String reason = safetyReason(now);
  if (reason.length() > 0) {
    digitalWrite(INVERTER_PIN, LOW);
    if (lockLevel != LOCK_HARD || !reason.equals(lockReason)) {
      logf("HARD LOCK: %s - inverter off\n", reason.c_str());
    }
    lockLevel = LOCK_HARD;
    lockReason = reason;
  } else if (lockLevel == LOCK_HARD) {
    logf("Hard lock cleared (%s) -> soft lock: /unlock needed for the buttons\n", lockReason.c_str());
    lockLevel = LOCK_SOFT;
    lockReason = "after hard lock: " + lockReason;
  }
}

/**
 * Setup the web server endpoints for metrics and control
 */
/**
 * Control endpoints need the API token from secrets.h as query parameter: POST /on?token=...
 * (a query parameter keeps the browser request "simple", so no CORS preflight is needed).
 * Compared in constant time.
 */
bool tokenMatches(AsyncWebServerRequest *request, const char *expected) {
  if (!request->hasParam("token")) return false;
  const String &given = request->getParam("token")->value();
  size_t n = strlen(expected);
  if (n < 16 || given.length() != n) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < n; i++) diff |= (uint8_t)given[i] ^ (uint8_t)expected[i];
  return diff == 0;
}

// The operator token: all control endpoints.
bool authorized(AsyncWebServerRequest *request) { return tokenMatches(request, API_TOKEN); }

// The automation token: POST /on only (never /unlock or /off).
bool authorizedAuto(AsyncWebServerRequest *request) { return tokenMatches(request, AUTO_TOKEN); }

void addCors(AsyncWebServerRequest *request, AsyncWebServerResponse *response) {
  String origin = request->header("Origin");
  if (origin.length() > 0) {
    response->addHeader("Access-Control-Allow-Origin", origin);
  }
  response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
  response->addHeader("Access-Control-Allow-Headers", "*");
  response->addHeader("Access-Control-Allow-Credentials", "true");
}

void rejectUnauthorized(AsyncWebServerRequest *request, const char *what) {
  AsyncWebServerResponse *response = request->beginResponse(401, "text/plain", "token missing or wrong");
  addCors(request, response);
  request->send(response);
  logf("Rejected %s from %s: token missing or wrong\n", what, request->client()->remoteIP().toString().c_str());
}

void setupWebServer() {
  // Prometheus metrics endpoint
  server.on("/metrics", HTTP_GET, [](AsyncWebServerRequest *request) {
    String content = "# HELP inverter_status shows the status of GPIO pin inverter\n";
    content += "# TYPE inverter_status gauge\n";
    int inverterStatus = digitalRead(INVERTER_PIN);
    content += "inverter_status " + String(inverterStatus) + "\n";
    
    content += "# HELP battery_has_alarm shows the last fetched battery alarm status\n";
    content += "# TYPE battery_has_alarm gauge\n";
    content += "battery_has_alarm " + lastBatteryHasAlarm + "\n";
    
    content += "# HELP solar2_connection_status shows if solar2 is reachable (1=reachable, 0=not)\n";
    content += "# TYPE solar2_connection_status gauge\n";
    content += "solar2_connection_status " + String(solar2ConnectionStatus) + "\n";

    content += "# HELP solar2_consecutive_fails consecutive failed HTTP attempts to reach solar2\n";
    content += "# TYPE solar2_consecutive_fails gauge\n";
    content += "solar2_consecutive_fails " + String(solar2ReadFailCount) + "\n";

    content += "# HELP mppt_has_error last MPPT has_error flag\n";
    content += "# TYPE mppt_has_error gauge\n";
    content += "mppt_has_error " + lastMpptHasError + "\n";

    content += "# HELP mppt_fault_state last MPPT is_in_fault_state flag\n";
    content += "# TYPE mppt_fault_state gauge\n";
    content += "mppt_fault_state " + lastMpptFaultState + "\n";

    content += "# HELP alarm_active 1 if any alarm is blocking the inverter\n";
    content += "# TYPE alarm_active gauge\n";
    content += "alarm_active " + String(lockLevel == LOCK_HARD ? 1 : 0) + "\n";

    content += "# HELP inverter_lock 0 none, 1 soft (operator /on needs /unlock; automation may switch on), 2 hard (safety: nothing switches on)\n";
    content += "# TYPE inverter_lock gauge\n";
    content += "inverter_lock " + String(lockLevel) + "\n";

    content += "# HELP inverter_lock_info why the inverter is locked (label reason)\n";
    content += "# TYPE inverter_lock_info gauge\n";
    {
      String r = lockLevel == LOCK_NONE ? String("") : lockReason;
      r.replace("\\", "/");
      r.replace("\"", "'");
      content += "inverter_lock_info{reason=\"" + r + "\"} 1\n";
    }

    content += "# HELP inverter_hardlock_persistent 1 if the operator hard lock is stored on flash (survives a restart)\n";
    content += "# TYPE inverter_hardlock_persistent gauge\n";
    content += "inverter_hardlock_persistent " + String(hardlockStored ? 1 : 0) + "\n";

    content += "# HELP lock_store_ok 1 if the lock storage (LittleFS) is mounted\n";
    content += "# TYPE lock_store_ok gauge\n";
    content += "lock_store_ok " + String(lockStoreOk ? 1 : 0) + "\n";

    content += "# HELP inverter_locked 1 if any lock (soft or hard) is set; kept for compatibility\n";
    content += "# TYPE inverter_locked gauge\n";
    content += "inverter_locked " + String(lockLevel != LOCK_NONE ? 1 : 0) + "\n";

    content += "# HELP inverter_last_on_auto 1 if the last successful /on came from the automation token\n";
    content += "# TYPE inverter_last_on_auto gauge\n";
    content += "inverter_last_on_auto " + String(lastOnAuto ? 1 : 0) + "\n";

    content += "# HELP solar2_data_age_ms milliseconds since last successful solar2 read\n";
    content += "# TYPE solar2_data_age_ms gauge\n";
    unsigned long dataAge = (lastSuccessfulRead > 0) ? millis() - lastSuccessfulRead : 0;
    content += "solar2_data_age_ms " + String(dataAge) + "\n";

    content += "# HELP battery_voltage battery voltage in V\n";
    content += "# TYPE battery_voltage gauge\n";
    content += "battery_voltage " + lastBatteryVoltage + "\n";

    content += "# HELP battery_current battery current in A\n";
    content += "# TYPE battery_current gauge\n";
    content += "battery_current " + lastBatteryCurrent + "\n";

    content += "# HELP state_of_charge battery state of charge in %\n";
    content += "# TYPE state_of_charge gauge\n";
    content += "state_of_charge " + lastStateOfCharge + "\n";

    content += "# HELP mppt_pv_power MPPT PV input power in W\n";
    content += "# TYPE mppt_pv_power gauge\n";
    content += "mppt_pv_power " + lastPvPower + "\n";

    content += "# HELP soc_shutdown_threshold SoC level below which inverter shuts down (%)\n";
    content += "# TYPE soc_shutdown_threshold gauge\n";
    content += "soc_shutdown_threshold " + String(SOC_SHUTDOWN_THRESHOLD) + "\n";

    content += "# HELP voltage_shutdown_threshold Voltage below which inverter shuts down (V)\n";
    content += "# TYPE voltage_shutdown_threshold gauge\n";
    content += "voltage_shutdown_threshold " + String(VOLTAGE_SHUTDOWN_THRESHOLD) + "\n";

    content += "# HELP wifi_rssi WiFi signal strength in dBm\n";
    content += "# TYPE wifi_rssi gauge\n";
    content += "wifi_rssi " + String(WiFi.RSSI()) + "\n";

    content += "# HELP free_heap_bytes free heap memory in bytes\n";
    content += "# TYPE free_heap_bytes gauge\n";
    content += "free_heap_bytes " + String(ESP.getFreeHeap()) + "\n";

    request->send(200, "text/plain", content);
  });

  // Unlock (operator token only): SOFT -> NONE. Refused while a HARD lock holds.
  server.on("/unlock", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!authorized(request)) { rejectUnauthorized(request, "/unlock"); return; }
    AsyncWebServerResponse *response;
    if (lockLevel == LOCK_HARD) {
      response = request->beginResponse(409, "text/plain", "Cannot unlock: hard lock (" + lockReason + ")");
      logf("Unlock rejected: hard lock (%s)\n", lockReason.c_str());
    } else {
      lockLevel = LOCK_NONE;
      lockReason = "";
      response = request->beginResponse(200, "text/plain", "Inverter unlocked.");
      log("Inverter unlocked.");
    }
    addCors(request, response);
    request->send(response);
  });

  // Turn inverter ON: HARD refuses everyone (409); SOFT refuses the operator token (404, "locked"), the
  // AUTO token may pass it; NONE: on.
  server.on("/on", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool manual = authorized(request);
    bool automatic = !manual && authorizedAuto(request);
    if (!manual && !automatic) { rejectUnauthorized(request, "/on"); return; }
    const char *by = automatic ? "auto" : "operator";
    String ip = request->client()->remoteIP().toString();
    AsyncWebServerResponse *response;
    if (lockLevel == LOCK_HARD) {
      response = request->beginResponse(409, "text/plain", "inverter blocked: hard lock (" + lockReason + ")");
      logf("Inverter ON (%s, %s) rejected: hard lock (%s)\n", by, ip.c_str(), lockReason.c_str());
    } else if (lockLevel == LOCK_SOFT && manual) {
      response = request->beginResponse(404, "text/plain", "inverter locked (soft): unlock first");
      logf("Inverter ON (%s, %s) rejected - soft lock\n", by, ip.c_str());
    } else {
      digitalWrite(INVERTER_PIN, HIGH);
      lastOnAuto = automatic;
      response = request->beginResponse(200, "text/plain", "inverter turned on");
      logf("Inverter turned ON (%s, %s, lock was %d)!\n", by, ip.c_str(), lockLevel);
    }
    addCors(request, response);
    request->send(response);
  });

  // Operator hard lock (operator token only): off now + HARD, until /hardunlock.
  server.on("/hardlock", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!authorized(request)) { rejectUnauthorized(request, "/hardlock"); return; }
    operatorHardLock = true;
    pendingHardlockWrite = true;
    digitalWrite(INVERTER_PIN, LOW);
    lockLevel = LOCK_HARD;
    lockReason = "operator hard lock";
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain",
      lockStoreOk ? "inverter off, hard lock set (stored, survives a restart)"
                  : "inverter off, hard lock set (NOT stored: lock storage error)");
    addCors(request, response);
    request->send(response);
    logf("HARD LOCK by operator (%s) - inverter off\n", request->client()->remoteIP().toString().c_str());
  });

  // Lift the operator hard lock (operator token only): -> SOFT, or stays HARD if a real condition holds.
  server.on("/hardunlock", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!authorized(request)) { rejectUnauthorized(request, "/hardunlock"); return; }
    pendingHardlockClear = true;   // loop() deletes the file, then lifts the lock
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain",
      "operator hard lock release requested (soft lock within a second unless a safety condition holds)");
    addCors(request, response);
    request->send(response);
    logf("Operator hard lock released (%s)\n", request->client()->remoteIP().toString().c_str());
  });

  // Turn inverter OFF (operator token only): off + SOFT lock (a HARD lock stays HARD).
  server.on("/off", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (!authorized(request)) { rejectUnauthorized(request, "/off"); return; }
    digitalWrite(INVERTER_PIN, LOW);
    if (lockLevel != LOCK_HARD) { lockLevel = LOCK_SOFT; lockReason = "operator off"; }
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "inverter turned off");
    addCors(request, response);
    request->send(response);
    log("Inverter turned OFF (soft lock).");
  });
}

/**
 * Checks WiFi connectivity and disables inverter if disconnected for too long
 */
void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiDisconnectTime == 0) {
      // First detection of disconnect
      wifiDisconnectTime = millis();
      Serial.println("WiFi disconnected!");
    } else if (millis() - wifiDisconnectTime >= WIFI_TIMEOUT) {
      // WiFi has been disconnected for too long: off now; the loop holds the HARD lock ("wifi lost")
      digitalWrite(INVERTER_PIN, LOW);
      if (!wifiLost) Serial.println("Inverter shutdown due to prolonged WiFi disconnection!");
      wifiLost = true;
    }
  } else {
    // WiFi is connected, reset disconnect timer
    wifiDisconnectTime = 0;
    wifiLost = false;
  }
}

/**
 * Sets up Over The Air (OTA) updates
 */
void setupOTA() {
  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("solar_inverter");

  // Set password for OTA updates
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nUpdate End");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
}

/**
 * Handle interactive telnet commands
 */
void handleTelnetMenu() {
  if (!TelnetStream.available()) return;

  String cmd = TelnetStream.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "help") {
    TelnetStream.println("\r\n=== Solar Inverter Telnet Menu ===");
    TelnetStream.println("  help    - Show this menu");
    TelnetStream.println("  status  - Inverter state, lock, alarm, battery");
    TelnetStream.println("  wifi    - SSID, RSSI, IP, MAC");
    TelnetStream.println("  scan    - Scan nearby WiFi networks");
    TelnetStream.println("  exit    - Close this session");
    TelnetStream.println("  (read-only: switching needs the token via HTTP)\r\n");

  } else if (cmd == "status") {
    TelnetStream.println("\r\n=== Status ===");
    TelnetStream.printf("  Inverter pin  : %s\r\n", digitalRead(INVERTER_PIN) ? "ON" : "OFF");
    TelnetStream.printf("  Hard lock file: %s (storage %s)\r\n", hardlockStored ? "yes" : "no", lockStoreOk ? "ok" : "ERROR");
    TelnetStream.printf("  Lock          : %s %s\r\n",
                        lockLevel == LOCK_HARD ? "HARD" : lockLevel == LOCK_SOFT ? "SOFT" : "NONE", lockReason.c_str());
    TelnetStream.printf("  Last ON by    : %s\r\n", lastOnAuto ? "automation" : "operator");
    TelnetStream.printf("  Battery alarm : %s\r\n", lastBatteryHasAlarm.c_str());
    TelnetStream.printf("  Battery V     : %s V\r\n", lastBatteryVoltage.c_str());
    TelnetStream.printf("  Battery A     : %s A\r\n", lastBatteryCurrent.c_str());
    TelnetStream.printf("  SoC           : %s %%\r\n", lastStateOfCharge.c_str());
    TelnetStream.printf("  PV Power      : %s W\r\n", lastPvPower.c_str());
    TelnetStream.printf("  MPPT state    : %s\r\n", lastMpptState.c_str());
    TelnetStream.printf("  MPPT error    : %s\r\n", lastMpptHasError.c_str());
    TelnetStream.printf("  MPPT fault    : %s\r\n", lastMpptFaultState.c_str());
    TelnetStream.printf("  Solar2 status : %s\r\n", solar2ConnectionStatus ? "Connected" : "Disconnected");
    TelnetStream.printf("  Read fails    : %d\r\n", solar2ReadFailCount);
    unsigned long dataAge = (lastSuccessfulRead > 0) ? (millis() - lastSuccessfulRead) / 1000 : 0;
    TelnetStream.printf("  Data age      : %lu s\r\n", dataAge);
    TelnetStream.printf("  Uptime        : %lu s\r\n", millis() / 1000);
    TelnetStream.printf("  Free heap     : %u bytes\r\n", ESP.getFreeHeap());
    TelnetStream.println();

  } else if (cmd == "wifi") {
    TelnetStream.println("\r\n=== WiFi ===");
    TelnetStream.printf("  Status   : %s\r\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    TelnetStream.printf("  SSID     : %s\r\n", WiFi.SSID().c_str());
    TelnetStream.printf("  BSSID    : %s\r\n", WiFi.BSSIDstr().c_str());
    TelnetStream.printf("  RSSI     : %d dBm\r\n", WiFi.RSSI());
    TelnetStream.printf("  IP       : %s\r\n", WiFi.localIP().toString().c_str());
    TelnetStream.printf("  Gateway  : %s\r\n", WiFi.gatewayIP().toString().c_str());
    TelnetStream.printf("  DNS      : %s\r\n", WiFi.dnsIP().toString().c_str());
    TelnetStream.printf("  MAC      : %s\r\n", WiFi.macAddress().c_str());
    TelnetStream.printf("  Hostname : %s\r\n", WiFi.hostname().c_str());
    TelnetStream.printf("  Channel  : %d\r\n", WiFi.channel());
    TelnetStream.println();

  } else if (cmd == "scan") {
    TelnetStream.println("\r\nScanning WiFi networks...");
    int n = WiFi.scanNetworks();
    if (n == 0) {
      TelnetStream.println("  No networks found.");
    } else {
      TelnetStream.printf("  Found %d networks:\r\n", n);
      for (int i = 0; i < n; i++) {
        TelnetStream.printf("  %2d: %-32s  %d dBm  ch%d  %s\r\n",
          i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
          WiFi.encryptionType(i) == ENC_TYPE_NONE ? "OPEN" : "SECURED");
      }
    }
    WiFi.scanDelete();
    TelnetStream.println();

  } else if (cmd == "exit" || cmd == "quit") {
    TelnetStream.println("Bye!");
    TelnetStream.flush();
    TelnetStream.stop();

  } else {
    TelnetStream.printf("Unknown command: '%s'. Type 'help' for available commands.\r\n", cmd.c_str());
  }
}

/**
 * Arduino setup function - runs once at startup
 */
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize GPIO pins
  pinMode(INVERTER_PIN, OUTPUT);
  digitalWrite(INVERTER_PIN, LOW);  // Start with inverter OFF

  // Persistent operator hard lock: read before anything can switch the inverter on
  lockStoreOk = LittleFS.begin();
  if (!lockStoreOk) {
    lockStoreError = true;
    lockLevel = LOCK_HARD;
    lockReason = "lock storage error";
  } else if (LittleFS.exists(HARDLOCK_FILE)) {
    operatorHardLock = true;
    hardlockStored = true;
    lockLevel = LOCK_HARD;
    lockReason = "operator hard lock";
  }
  
  log("\nESP8266 Solar Monitor & Inverter Controller starting...");
  
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);                // Don't cache settings in flash (avoids 11b brick repeat)
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);   // Force 802.11n
  WiFi.setOutputPower(20.5);             // Max TX power (20.5 dBm)
  WiFi.setSleepMode(WIFI_NONE_SLEEP);    // Keep radio always on
  WiFi.hostname("solar_inverter");       // Set hostname before connect
  
  // Static IP configuration — faster reconnect, no DHCP dependency
  IPAddress staticIP(192, 168, 178, 122);
  IPAddress gateway(192, 168, 178, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(192, 168, 178, 75);
  WiFi.config(staticIP, gateway, subnet, dns);
  
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  
  Serial.print("Connecting to WiFi...");
  TelnetStream.print("Connecting to WiFi...\r\n");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    TelnetStream.print(".\r\n");
  }
  Serial.println();
  TelnetStream.print("\r\n");
  logf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  
  // Initialize MDNS responder
  if (MDNS.begin("solar_inverter")) {
    MDNS.addService("http", "tcp", SERVER_PORT);
    log("MDNS responder started");
    log("You can now access the device at http://solar_inverter.local");
  }
  
  // Setup OTA updates
  setupOTA();
  
  // Setup the webserver endpoints
  setupWebServer();

  // Start the web server
  server.begin();
  logf("Web server started on port %d\n", SERVER_PORT);
  
  // Start WiFi monitoring ticker (check every second)
  wifiCheckTicker.attach(1.0, checkWiFiConnection);
  log("WiFi monitoring started");
  
  // Start Telnet server for remote monitoring
  TelnetStream.begin();
  log("Telnet server started");
  log("Connect using 'telnet solar_inverter.local' from command line");
}

// --- Arduino Loop ---
unsigned long lastCheckTime = 0;
const unsigned long CHECK_INTERVAL = 500;  // Check every 500ms

/**
 * Main program loop - runs continuously
 */
/**
 * Flash writes for the operator hard lock, outside the web callbacks. Fail-safe: the lock is lifted only after
 * the flag file is really gone.
 */
void handleHardlockStore() {
  if (pendingHardlockWrite) {
    pendingHardlockWrite = false;
    if (lockStoreOk) {
      File f = LittleFS.open(HARDLOCK_FILE, "w");
      bool ok = f && f.print("1") == 1;
      if (f) f.close();
      hardlockStored = ok && LittleFS.exists(HARDLOCK_FILE);
      logf("Operator hard lock %s\n", hardlockStored ? "stored" : "NOT stored (write failed)");
    }
  }
  if (pendingHardlockClear) {
    pendingHardlockClear = false;
    if (pendingHardlockWrite) return;  // a new /hardlock came in meanwhile: it wins
    bool gone = true;
    if (lockStoreOk && LittleFS.exists(HARDLOCK_FILE)) {
      LittleFS.remove(HARDLOCK_FILE);
      gone = !LittleFS.exists(HARDLOCK_FILE);
    }
    if (gone) {
      operatorHardLock = false;
      lockStoreError = false;
      hardlockStored = false;
      log("Operator hard lock released.");
    } else {
      log("Operator hard lock NOT released: could not delete the stored lock.");
    }
  }
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Store / clear the operator hard lock (flash writes stay out of the web callbacks)
  handleHardlockStore();
  
  // Check battery/MPPT status from solar2 and enforce shutdown if needed
  checkBatteryAlarmAndShutdown();
  
  // Handle interactive telnet commands
  handleTelnetMenu();
  
  // Handle MDNS
  MDNS.update();
  
  // Give time to background tasks
  yield();
}
