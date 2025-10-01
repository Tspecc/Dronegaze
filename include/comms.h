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
  uint32_t magic;                // Should be PACKET_MAGIC
  float pitch, roll, yaw;        // Orientation in degrees
  float pitchCorrection, rollCorrection, yawCorrection; // PID outputs
  float verticalCorrection;      // Vertical PID output
  uint16_t throttle;             // Current throttle command
  int8_t pitchAngle, rollAngle, yawAngle; // Commanded angles
  uint8_t stabilizationMask;     // Bitmask of enabled stabilization axes
  uint8_t reserved[3];           // Reserved for future use
  float verticalAcc;             // Vertical acceleration in m/s^2
  uint32_t commandAge;           // Age of last command in ms
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

enum class MessageType : uint8_t {
    MSG_PAIR_REQ = 0x01,
    MSG_IDENTITY_REPLY = 0x02,
    MSG_PAIR_CONFIRM = 0x03,
    MSG_PAIR_ACK = 0x04,
    MSG_COMMAND = 0x06,
};

struct DiscoveryIdentity {
    char deviceType[12];
    char platform[16];
    char customId[32];
    uint8_t mac[6];
} __attribute__((packed));

struct CommandHeader {
    uint8_t version;
    MessageType type;
    DiscoveryIdentity id;
    uint32_t monotonicMs;
    uint32_t reserved;
} __attribute__((packed));

struct CommandPacket {
    CommandHeader header;
    char command[48];
} __attribute__((packed));

constexpr uint8_t COMMAND_PROTOCOL_VERSION = 1;

bool init(const char *ssid, const char *password, int tcpPort);
bool init(const char *ssid, const char *password, int tcpPort, esp_now_recv_cb_t recvCallback);
bool receiveCommand(ThrustCommand &cmd);
bool paired();
extern const uint8_t BroadcastMac[6];
}

