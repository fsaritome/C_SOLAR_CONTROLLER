/**
 * ESP8266 Solar Monitor & Inverter Controller
 * 
 * This program monitors a solar system's status by querying metrics from another ESP device
 * and controls an inverter based on battery and MPPT status.
 * It also provides a web server with endpoints for metrics and inverter control.
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

// --- Pin Definitions ---
#define INVERTER_PIN D7  // GPIO pin connected to the inverter control

// --- Network Configuration ---
const char *ssid = "IOT-Hadal";
const char *password = "Chr0nowirelessisdabest";
const uint16_t SERVER_PORT = 8664;

// --- Global Variables ---
bool locked = true;                   // Manual inverter lock (operator-controlled)
bool alarmActive = false;             // Safety lock: set on alarm, blocks /unlock and /on until alarm clears
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
 * Polls solar2.local/metrics every 500ms.
 * - Shuts down inverter after MAX_CONSECUTIVE_FAILS (10 × 500ms = 5s) HTTP failures
 * - Shuts down if no successful read for DATA_STALE_TIMEOUT (30s)
 * - Sets alarmActive on battery/MPPT alarm; only clears it when both read as clean
 */
void checkBatteryAlarmAndShutdown() {
  static unsigned long lastCheck = 0;
  unsigned long now = millis();

  if (now - lastCheck < 500) {
    return;
  }
  lastCheck = now;

  // Data staleness safety: if we had at least one successful read but data went stale, shut down.
  // NOTE: do NOT return here — polling must continue so recovery is possible when solar2 comes back.
  if (lastSuccessfulRead > 0 && (now - lastSuccessfulRead) >= DATA_STALE_TIMEOUT) {
    if (!alarmActive) {
      digitalWrite(INVERTER_PIN, LOW);
      locked = true;
      alarmActive = true;
      logf("Inverter shutdown: no solar2 data for %lu seconds!\n", (now - lastSuccessfulRead) / 1000UL);
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(2000);
  http.begin(client, "http://solar2.powdersplash.com/metrics");
  yield();
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    http.end();
    solar2ReadFailCount++;
    solar2ConnectionStatus = 0;
    if (solar2ReadFailCount >= MAX_CONSECUTIVE_FAILS) {
      digitalWrite(INVERTER_PIN, LOW);
      locked = true;
      alarmActive = true;
      logf("Inverter shutdown: %d consecutive solar2 failures (HTTP %d)!\n",
           solar2ReadFailCount, httpCode);
    }
    return;
  }

  // Successful HTTP read
  solar2ReadFailCount = 0;
  solar2ConnectionStatus = 1;
  lastSuccessfulRead = now;
  String payload = http.getString();
  http.end();

  bool batteryAlarm = false;
  bool mpptError = false;

  // --- Check Battery Alarm ---
  String search = "esphome_binary_sensor_value{id=\"battery_has_alarm\"";
  int idx = payload.indexOf(search);
  if (idx != -1) {
    int valueStart = payload.indexOf('}', idx);
    if (valueStart != -1) {
      valueStart++;
      while (valueStart < (int)payload.length() && isspace(payload[valueStart])) valueStart++;
      int valueEnd = valueStart;
      while (valueEnd < (int)payload.length() && !isspace(payload[valueEnd])) valueEnd++;
      String valueStr = payload.substring(valueStart, valueEnd);
      lastBatteryHasAlarm = valueStr;
      if (valueStr == "1.000000") batteryAlarm = true;
    }
  }

  // --- Check MPPT has_error ---
  {
    String mpptSearch = "esphome_binary_sensor_value{id=\"mppt_has_error\"";
    int mpptIdx = payload.indexOf(mpptSearch);
    if (mpptIdx != -1) {
      int valueStart = payload.indexOf('}', mpptIdx);
      if (valueStart != -1) {
        valueStart++;
        while (valueStart < (int)payload.length() && isspace(payload[valueStart])) valueStart++;
        int valueEnd = valueStart;
        while (valueEnd < (int)payload.length() && !isspace(payload[valueEnd])) valueEnd++;
        String valueStr = payload.substring(valueStart, valueEnd);
        lastMpptHasError = valueStr;
        if (valueStr == "1.000000") mpptError = true;
      }
    }
  }

  // --- Check MPPT is_in_fault_state ---
  {
    String faultSearch = "esphome_binary_sensor_value{id=\"mppt_is_in_fault_state\"";
    int faultIdx = payload.indexOf(faultSearch);
    if (faultIdx != -1) {
      int valueStart = payload.indexOf('}', faultIdx);
      if (valueStart != -1) {
        valueStart++;
        while (valueStart < (int)payload.length() && isspace(payload[valueStart])) valueStart++;
        int valueEnd = valueStart;
        while (valueEnd < (int)payload.length() && !isspace(payload[valueEnd])) valueEnd++;
        String valueStr = payload.substring(valueStart, valueEnd);
        lastMpptFaultState = valueStr;
        if (valueStr == "1.000000") mpptError = true;
      }
    }
  }

  // --- Parse numeric sensors ---
  auto parseValue = [&](const String& id, String& dest) {
    String key = "esphome_sensor_value{id=\"" + id + "\"";
    int i = payload.indexOf(key);
    if (i != -1) {
      int vs = payload.indexOf('}', i);
      if (vs != -1) {
        vs++;
        while (vs < (int)payload.length() && isspace(payload[vs])) vs++;
        int ve = vs;
        while (ve < (int)payload.length() && !isspace(payload[ve])) ve++;
        dest = payload.substring(vs, ve);
      }
    }
  };
  parseValue("battery_voltage",  lastBatteryVoltage);
  parseValue("battery_current",  lastBatteryCurrent);
  parseValue("state_of_charge",  lastStateOfCharge);
  parseValue("mppt_pv_power",    lastPvPower);

  // --- Parse MPPT state text ---
  {
    String stateSearch = "esphome_text_sensor_value{id=\"mppt_state\"";
    int si = payload.indexOf(stateSearch);
    if (si != -1) {
      int valAttr = payload.indexOf("value=\"", si);
      if (valAttr != -1) {
        valAttr += 7;
        int valEnd = payload.indexOf('"', valAttr);
        if (valEnd != -1) {
          lastMpptState = payload.substring(valAttr, valEnd);
        }
      }
    }
  }

  if (batteryAlarm || mpptError) {
    if (!alarmActive) {
      alarmActive = true;
      logf("ALARM ACTIVE: battery=%s mppt=%s — inverter shutdown!\n",
           lastBatteryHasAlarm.c_str(), lastMpptHasError.c_str());
    }
    digitalWrite(INVERTER_PIN, LOW);
    locked = true;
  } else {
    // Check low SoC
    float soc     = lastStateOfCharge.toFloat();
    float voltage = lastBatteryVoltage.toFloat();
    bool lowSoc     = (soc > 0.0f)     && (soc     < SOC_SHUTDOWN_THRESHOLD);
    bool lowVoltage = (voltage > 0.0f) && (voltage < VOLTAGE_SHUTDOWN_THRESHOLD);

    if (lowSoc || lowVoltage) {
      if (!alarmActive) {
        alarmActive = true;
        logf("LOW BATTERY SHUTDOWN: SoC=%.1f%% Voltage=%.2fV\n", soc, voltage);
      }
      digitalWrite(INVERTER_PIN, LOW);
      locked = true;
    } else if (alarmActive) {
      // Only clear alarm when BOTH SoC and voltage are above recovery thresholds
      bool socRecovered     = (soc     <= 0.0f || soc     >= SOC_RECOVER_THRESHOLD);
      bool voltageRecovered = (voltage <= 0.0f || voltage >= VOLTAGE_RECOVER_THRESHOLD);
      if (socRecovered && voltageRecovered) {
        alarmActive = false;
        log("Alarms cleared. Alarm lock released. Use /unlock to re-enable inverter.");
      }
    }
  }
}

/**
 * Setup the web server endpoints for metrics and control
 */
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
    
    content += "# HELP solar2_connection_status shows if solar2.powdersplash.com is reachable (1=reachable, 0=not)\n";
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
    content += "alarm_active " + String(alarmActive ? 1 : 0) + "\n";

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

  // Unlock the inverter (blocked while alarm is active)
  server.on("/unlock", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response;
    if (alarmActive) {
      response = request->beginResponse(409, "text/plain",
        "Cannot unlock: alarm active (battery=" + lastBatteryHasAlarm + " mppt=" + lastMpptHasError + ")");
      logf("Unlock rejected: alarm active (battery=%s mppt=%s)\n",
           lastBatteryHasAlarm.c_str(), lastMpptHasError.c_str());
    } else {
      locked = false;
      response = request->beginResponse(200, "text/plain", "Inverter unlocked.");
      log("Inverter unlocked.");
    }

    String origin = request->header("Origin");
    if (origin.length() > 0) {
      response->addHeader("Access-Control-Allow-Origin", origin);
    }
    response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "*");
    response->addHeader("Access-Control-Allow-Credentials", "true");
    request->send(response);
  });
  
  // Turn inverter ON endpoint
  server.on("/on", HTTP_POST, [](AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response;
    if (alarmActive) {
      response = request->beginResponse(409, "text/plain",
        "inverter blocked: alarm active (battery=" + lastBatteryHasAlarm + " mppt=" + lastMpptHasError + ")");
      logf("Inverter ON rejected: alarm active (battery=%s mppt=%s)\n",
           lastBatteryHasAlarm.c_str(), lastMpptHasError.c_str());
    } else if (locked) {
      response = request->beginResponse(404, "text/plain", "inverter locked");
      log("Inverter ON request rejected - locked!");
    } else {
      digitalWrite(INVERTER_PIN, HIGH);
      response = request->beginResponse(200, "text/plain", "inverter turned on");
      log("Inverter turned ON!");
    }
    
    // Add CORS headers
    String origin = request->header("Origin");
    if (origin.length() > 0) {
      response->addHeader("Access-Control-Allow-Origin", origin);
    }
    response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "*");
    response->addHeader("Access-Control-Allow-Credentials", "true");
    
    request->send(response);
  });

  // Turn inverter OFF endpoint
  server.on("/off", HTTP_POST, [](AsyncWebServerRequest *request) {
    digitalWrite(INVERTER_PIN, LOW);
    locked = true;  // Always lock after turning off
    
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "inverter turned off");
    
    // Add CORS headers
    String origin = request->header("Origin");
    if (origin.length() > 0) {
      response->addHeader("Access-Control-Allow-Origin", origin);
    }
    response->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "*");
    response->addHeader("Access-Control-Allow-Credentials", "true");
    
    request->send(response);
    Serial.println("Inverter turned OFF and locked!");
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
      // WiFi has been disconnected for too long
      digitalWrite(INVERTER_PIN, LOW);
      locked = true;
      Serial.println("Inverter shutdown due to prolonged WiFi disconnection!");
    }
  } else {
    // WiFi is connected, reset disconnect timer
    wifiDisconnectTime = 0;
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
  ArduinoOTA.setPassword("solar123");

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
    TelnetStream.println("  on      - Turn inverter ON");
    TelnetStream.println("  off     - Turn inverter OFF");
    TelnetStream.println("  lock    - Lock inverter");
    TelnetStream.println("  unlock  - Unlock inverter");
    TelnetStream.println("  reboot  - Restart ESP\r\n");

  } else if (cmd == "status") {
    TelnetStream.println("\r\n=== Status ===");
    TelnetStream.printf("  Inverter pin  : %s\r\n", digitalRead(INVERTER_PIN) ? "ON" : "OFF");
    TelnetStream.printf("  Locked        : %s\r\n", locked ? "YES" : "NO");
    TelnetStream.printf("  Alarm active  : %s\r\n", alarmActive ? "YES" : "NO");
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

  } else if (cmd == "on") {
    if (alarmActive) {
      TelnetStream.println("Cannot turn on: alarm active!");
    } else if (locked) {
      TelnetStream.println("Cannot turn on: inverter is locked. Use 'unlock' first.");
    } else {
      digitalWrite(INVERTER_PIN, HIGH);
      TelnetStream.println("Inverter turned ON.");
      log("[Telnet] Inverter turned ON");
    }

  } else if (cmd == "off") {
    digitalWrite(INVERTER_PIN, LOW);
    locked = true;
    TelnetStream.println("Inverter turned OFF and locked.");
    log("[Telnet] Inverter turned OFF");

  } else if (cmd == "lock") {
    locked = true;
    TelnetStream.println("Inverter locked.");
    log("[Telnet] Inverter locked");

  } else if (cmd == "unlock") {
    if (alarmActive) {
      TelnetStream.println("Cannot unlock: alarm active!");
    } else {
      locked = false;
      TelnetStream.println("Inverter unlocked.");
      log("[Telnet] Inverter unlocked");
    }

  } else if (cmd == "exit" || cmd == "quit") {
    TelnetStream.println("Bye!");
    TelnetStream.flush();
    TelnetStream.stop();

  } else if (cmd == "reboot") {
    TelnetStream.println("Rebooting...");
    TelnetStream.flush();
    delay(100);
    ESP.restart();

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
void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Check battery/MPPT status from solar2 and enforce shutdown if needed
  checkBatteryAlarmAndShutdown();
  
  // Handle interactive telnet commands
  handleTelnetMenu();
  
  // Handle MDNS
  MDNS.update();
  
  // Give time to background tasks
  yield();
}
