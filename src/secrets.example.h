// Copy this file to secrets.h and fill in your own values. secrets.h is ignored by git.
#pragma once
#define WIFI_SSID "your-wifi-name"
#define WIFI_PASSWORD "your-wifi-password"
#define OTA_PASSWORD "choose-an-ota-password"
// Needed for POST /on, /off, /unlock as ?token=...; at least 16 characters, letters and digits.
#define API_TOKEN "choose-a-long-random-token"
// Optional: a second token accepted ONLY by POST /on (an automation may switch on but never unlock or lock).
// Leave it out to disable that path. At least 16 characters, different from API_TOKEN.
#define AUTO_TOKEN "choose-another-long-random-token"
// solar2's metrics by fixed IP (no DNS dependency), e.g. "http://192.168.1.50/metrics"
#define SOLAR2_METRICS_URL "http://192.168.1.50/metrics"
// Optional static address (otherwise DHCP), e.g.:
#define STATIC_IP "192.168.1.60"
#define GATEWAY_IP "192.168.1.1"
#define DNS_IP "192.168.1.2"
