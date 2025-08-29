#include "comms.h"

namespace Comms {
static bool g_paired = false;
static ThrustCommand lastCmd = {0};
const uint8_t BroadcastMac[6] = {0xff,0xff,0xff,0xff,0xff,0xff};

void init(const char *ssid, const char *password, int tcpPort) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.softAP(ssid, password);
    WiFi.begin(ssid, password);
    esp_now_init();
    g_paired = true; // placeholder for real pairing
}

bool receiveCommand(ThrustCommand &cmd) {
    cmd = lastCmd; // placeholder - real implementation would read from ESP-NOW
    return g_paired;
}

bool paired() {
    return g_paired;
}
}

