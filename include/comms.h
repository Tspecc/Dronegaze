#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

namespace Comms {
struct ThrustCommand {
    uint32_t magic;
    uint16_t throttle;
    int8_t pitchAngle;
    int8_t rollAngle;
    int8_t yawAngle;
    bool arm_motors;
} __attribute__((packed));

struct TelemetryPacket {
    uint32_t magic;
    float pitch, roll, yaw;
    float pitchCorrection, rollCorrection, yawCorrection;
    uint16_t throttle;
    int8_t pitchAngle, rollAngle, yawAngle;
    float altitude, verticalAcc;
    uint32_t commandAge;
} __attribute__((packed));

enum PairingType : uint8_t {
    SCAN_REQUEST = 0x01,
    DRONE_IDENTITY = 0x02,
    ILITE_IDENTITY = 0x03,
    DRONE_ACK = 0x04,
};

struct IdentityMessage {
    uint8_t type;
    char identity[16];
    uint8_t mac[6];
} __attribute__((packed));

void init(const char *ssid, const char *password, int tcpPort);
bool receiveCommand(ThrustCommand &cmd);
bool paired();
extern const uint8_t BroadcastMac[6];
}

