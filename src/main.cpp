#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <EEPROM.h>
#include <freertos/queue.h>
#include <cstring>
#include "comms.h"
#include "commands.h"
#include "pid.h"
#include "control.h"
#include "imu.h"
#include "motor.h"

// ==================== BOARD CONFIGURATION ====================
// Select pin mappings and task sizes based on target board

#define CONFIG_IDF_TARGET_ESP32C3



#if defined(CONFIG_IDF_TARGET_ESP32C3)
// ESP32-C3 Super Mini (RISC-V single core)
const int PIN_MFL = 1; // MTL
const int PIN_MFR = 2; // MTR
const int PIN_MBL = 0; // MBL
const int PIN_MBR = 3;  // MBR
const int BUZZER_PIN = 6;
const uint32_t CPU_FREQ_MHZ = 160;

const int PWM_RESOLUTION = 16;

// Reduced stack sizes for smaller RAM
const uint16_t FAST_TASK_STACK = 2048*2;
const uint16_t COMM_TASK_STACK = 4096*2;
const uint16_t FAILSAFE_TASK_STACK = 2048*2;
const uint16_t TELEMETRY_TASK_STACK = 2048*2;
const uint16_t OTA_TASK_STACK = 2048*2;
const uint16_t BUZZER_TASK_STACK = 1024*2;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreate(fn, name, stack, NULL, prio, handle)
#else
// Default ESP32 (e.g., NodeMCU-32S)
const int PIN_MFL = 14;
const int PIN_MFR = 27;
const int PIN_MBL = 26;
const int PIN_MBR = 25;
const int BUZZER_PIN = -1; // No buzzer by default
const uint32_t CPU_FREQ_MHZ = 240;
const int PWM_RESOLUTION = 16;
const uint16_t FAST_TASK_STACK = 4096;
const uint16_t COMM_TASK_STACK = 8192;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t TELEMETRY_TASK_STACK = 4096;
const uint16_t OTA_TASK_STACK = 2048;
const uint16_t BUZZER_TASK_STACK = 1024;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)
#endif


// Use LEDC channel 5 for the buzzer. Channels are paired by timer on the
// ESP32‑C3 (0/1, 2/3, 4/5), so channel 5 uses timer2 which none of the motors
// occupy. This prevents the buzzer from changing the motors' 50 Hz PWM.

const int BUZZER_CHANNEL = 5;


/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Dronegaze Telemetry port";
const char *WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;

// Motor and control constants
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int CORRECTION_LIMIT = 400;
const unsigned long FAILSAFE_TIMEOUT = 200;  // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms
const float FLIP_ANGLE = 70.0f; // degrees; beyond this we cut motors
const float ARMING_ANGLE_LIMIT = 15.0f; // max tilt allowed to arm
const int ARMING_THROTTLE = THROTTLE_MIN + 50; // throttle must stay below to arm/disarm
const unsigned long DISARM_DELAY = 1000; // ms throttle-low before disarm

// IMU constants
const float GYRO_SCALE = 131.0; // LSB/°/s for ±250°/s
const float rad_to_deg = 180.0 / PI;
const float VERTICAL_ACC_GAIN = 20.0f; // throttle units per m/s^2

bool failsafe_enable = 1;
bool isArmed = 0;

bool enableFilters = false; // Enable or disable filters
bool enableQuadFilters = false;

const char *DRONE_ID = "DrongazeA1";
const uint32_t PACKET_MAGIC = 0xA1B2C3D4;

struct BuzzerCommand { uint16_t freq; uint16_t duration; };

// ==================== GLOBAL VARIABLES ====================
// Hardware
WiFiServer server(TCP_PORT);
WiFiClient client;
Comms::ThrustCommand command = {PACKET_MAGIC, THROTTLE_MIN, 0, 0, 0, false};
Motor::Outputs currentOutputs{MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
Motor::Outputs targetOutputs{MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
float pitch = 0, roll = 0, yaw = 0;
float pitchCorrection = 0, rollCorrection = 0, yawCorrection = 0;
float verticalCorrection = 0;
float pitchSetpoint = 0, rollSetpoint = 0, yawSetpoint = 0; // angle targets
bool yawControlEnabled = false;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetry = 0;
String incomingCommand = "";
QueueHandle_t buzzerQueue = nullptr;

// Legacy PID controllers retained for OLED tuning interface (unused)
PIDController pitchPID, rollPID, yawPID;
PIDController verticalAccelPID(VERTICAL_ACC_GAIN, 0.0, 5.0);

// ==================== COMMUNICATION FUNCTIONS ====================
// Time in ms before we consider the controller disconnected
const unsigned long CONNECTION_TIMEOUT = 1000;
// Minimum delay between handshake responses to avoid spamming
const unsigned long HANDSHAKE_COOLDOWN = 500;
unsigned long lastHandshakeSent = 0;
bool telemetryEnabled = false; // Serial/TCP telemetry disabled by default
String messageBuffer = "";
const int MAX_MESSAGE_LENGTH = 256;
bool serialActive = false; // Tracks if a Serial session is currently open

// Dynamic ESP-NOW pairing
uint8_t iliteMac[6];
uint8_t selfMac[6];
bool ilitePaired = false;
uint8_t commandPeer[6];
bool commandPeerSet = false;
unsigned long lastDiscoveryTime = 0;


// ==================== IMPROVED COMMUNICATION FUNCTIONS ====================

void beep(uint16_t freq, uint16_t duration)
{
    if (BUZZER_PIN < 0 || !buzzerQueue)
        return;
    BuzzerCommand cmd{freq, duration};
    xQueueSend(buzzerQueue, &cmd, 0);
}

void BuzzerTask(void *pvParameters)
{
    BuzzerCommand cmd;
    while (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY))
    {
        ledcWriteTone(BUZZER_CHANNEL, cmd.freq);
        vTaskDelay(pdMS_TO_TICKS(cmd.duration));
        ledcWrite(BUZZER_CHANNEL, 0);
    }
}

void streamTelemetry()
{
    if (millis() - lastTelemetry < TELEMETRY_INTERVAL)
        return;

    lastTelemetry = millis();

    Comms::TelemetryPacket packet = {
        PACKET_MAGIC,
        pitch, roll, yaw,
        pitchCorrection, rollCorrection, yawCorrection,
        (uint16_t)command.throttle,
        command.pitchAngle, command.rollAngle, command.yawAngle,
        IMU::verticalAcc(),
        (uint32_t)(millis() - lastCommandTime)
    };

    // Send telemetry to ILITE ground station if paired
    if (ilitePaired)
    {
        esp_now_send(iliteMac, (uint8_t *)&packet, sizeof(packet));
    }

    // Also send to the last command peer if different
    if (commandPeerSet && (!ilitePaired || memcmp(commandPeer, iliteMac, 6) != 0))
    {
        esp_now_send(commandPeer, (uint8_t *)&packet, sizeof(packet));
    }

    bool tcpActive = client && client.connected();
    if (!(telemetryEnabled && (serialActive || tcpActive)))
        return;

    String telemetry = "DB:" + String(pitch, 2) + " " + String(roll, 2) + " " + String(yaw, 2) + " " +
                       String(pitchCorrection, 2) + " " + String(rollCorrection, 2) + " " + String(yawCorrection, 2) + " " +
                       String(command.throttle) + " " + String(command.pitchAngle) + " " +
                       String(command.rollAngle) + " " + String(command.yawAngle) + " " +
                       String(IMU::verticalAcc(), 2) + " " + String(millis() - lastCommandTime);

    if (serialActive)
        Serial.println(telemetry);
    if (tcpActive)
    {
        client.println(telemetry);
        client.flush();
    }
}

void handleIncomingData()
{
    // Update serial connection status based on host presence
#if ARDUINO_USB_CDC_ON_BOOT
    serialActive = Serial; // DTR not available on some boards
#else
    // Without USB-CDC we can't detect port status; start once data arrives
    if (Serial.available())
        serialActive = true;
#endif

    // Handle TCP client connection
    if (!client || !client.connected())
    {
        WiFiClient newClient = server.available();
        if (newClient)
        {
            if (client)
                client.stop(); // Close old connection
            client = newClient;
            Commands::sendLine("ACK: TCP client connected");
        }
    }

    // Process TCP data with buffering
    while (client && client.available())
    {
        char c = client.read();
        if (c == '\n' || c == '\r')
        {
            if (messageBuffer.length() > 0)
            {
                Commands::handleCommand(messageBuffer);
                messageBuffer = "";
            }
        }
        else if (messageBuffer.length() < MAX_MESSAGE_LENGTH)
        {
            messageBuffer += c;
        }
        else
        {
            // Buffer overflow protection
            messageBuffer = "";
            Commands::sendLine("ERROR: Message too long");
        }
    }

    // Process Serial data with buffering
    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (messageBuffer.length() > 0)
            {
                Commands::handleCommand(messageBuffer);
                messageBuffer = "";
            }
        }
        else if (messageBuffer.length() < MAX_MESSAGE_LENGTH)
        {
            messageBuffer += c;
        }
        else
        {
            // Buffer overflow protection
            messageBuffer = "";
            Commands::sendLine("ERROR: Message too long");
        }
    }
}

// Detect dropped connections and cleanup peers
void monitorConnection() {
    if (ilitePaired && millis() - lastCommandTime > CONNECTION_TIMEOUT) {
        ilitePaired = false;
        commandPeerSet = false;
        esp_now_del_peer(iliteMac);
        memset(iliteMac, 0, sizeof(iliteMac));
    }
}

// ==================== ESP-NOW CALLBACK ====================
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(Comms::IdentityMessage))
    {
        Comms::IdentityMessage msg;
        memcpy(&msg, incomingData, sizeof(msg));
        unsigned long now = millis();
        if (msg.type == Comms::SCAN_REQUEST)
        {
            if (now - lastHandshakeSent > HANDSHAKE_COOLDOWN)
            {
                if (!esp_now_is_peer_exist(mac))
                {
                    esp_now_peer_info_t peerInfo = {};
                    memcpy(peerInfo.peer_addr, mac, 6);
                    peerInfo.channel = 0;
                    peerInfo.encrypt = false;
                    esp_now_add_peer(&peerInfo);
                }
                Comms::IdentityMessage resp = {};
                resp.type = Comms::DRONE_IDENTITY;
                strncpy(resp.identity, DRONE_ID, sizeof(resp.identity));
                memcpy(resp.mac, selfMac, 6);
                esp_now_send(mac, (uint8_t *)&resp, sizeof(resp));
                lastHandshakeSent = now;
            }
        }


        else if (msg.type == Comms::ILITE_IDENTITY && !ilitePaired)
        {
            memcpy(iliteMac, msg.mac, 6);
            ilitePaired = true;
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, msg.mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (!esp_now_is_peer_exist(msg.mac))
            {
                esp_now_add_peer(&peerInfo);
            }
            Comms::IdentityMessage ack = {};
            ack.type = Comms::DRONE_ACK;
            strncpy(ack.identity, DRONE_ID, sizeof(ack.identity));
            memcpy(ack.mac, selfMac, 6);
            esp_now_send(msg.mac, (uint8_t *)&ack, sizeof(ack));
            lastHandshakeSent = now;
            if (BUZZER_PIN >= 0)
            {
                beep(2000, 200); // short beep on pairing
            }
        }

        return;
    }

    if (len == sizeof(Comms::ThrustCommand))
    {
        Comms::ThrustCommand incoming;
        memcpy(&incoming, incomingData, sizeof(incoming));

        // Ignore commands from unknown devices or with wrong magic
        if (!ilitePaired || memcmp(mac, iliteMac, 6) != 0 || incoming.magic != PACKET_MAGIC)
        {
            return;
        }

        command = incoming;
        lastCommandTime = millis();

        if (!commandPeerSet || memcmp(commandPeer, mac, 6) != 0)
        {
            memcpy(commandPeer, mac, 6);
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            if (!esp_now_is_peer_exist(mac))
            {
                esp_now_add_peer(&peerInfo);
            }
            commandPeerSet = true;
        }

        // Update angle setpoints from command
        pitchSetpoint = command.pitchAngle;
        rollSetpoint = command.rollAngle;
        yawSetpoint = command.yawAngle;
    }
}

void checkFailsafe() {
    static unsigned long lastAlarmTime = 0;
    static unsigned long disarmStart = 0;
    if (!failsafe_enable) return;

    // Connection-loss failsafe
    if (millis() - lastCommandTime > FAILSAFE_TIMEOUT) {
        command.throttle = THROTTLE_MIN;
        command.pitchAngle = 0;
        command.rollAngle = 0;
        command.yawAngle = yaw; // hold current yaw on failsafe
        pitchSetpoint = rollSetpoint = 0;
        yawSetpoint = yaw;
        yawControlEnabled = false;

        targetOutputs.MFL = targetOutputs.MFR = targetOutputs.MBL = targetOutputs.MBR = MOTOR_MIN;
        isArmed = false;

        if (BUZZER_PIN >= 0 && millis() - lastAlarmTime > 5000) {
            beep(800, 200); // periodic short alarm
            lastAlarmTime = millis();
        }

        static unsigned long lastFailsafeMessage = 0;
        if (millis() - lastFailsafeMessage > 5000) {
            Commands::sendLine("FAILSAFE: No commands received for " + String(millis() - lastCommandTime) + "ms");
            lastFailsafeMessage = millis();
        }
        return;
    }

    // Arming / disarming logic
    if (!isArmed) {
        if (ilitePaired && command.arm_motors &&
            fabs(pitch) < ARMING_ANGLE_LIMIT &&
            fabs(roll) < ARMING_ANGLE_LIMIT) {
            isArmed = true;
            beep(1200, 100);
        }
    } else {
        bool keepArmed = command.arm_motors;
        if (!keepArmed) {
            if (disarmStart == 0) disarmStart = millis();
            if (millis() - disarmStart > DISARM_DELAY) {
                isArmed = false;
                targetOutputs.MFL = targetOutputs.MFR = targetOutputs.MBL = targetOutputs.MBR = MOTOR_MIN;
                beep(600, 200);
            }
        } else {
            disarmStart = 0;
        }
    }
}

// ==================== SETUP ====================

void FastTask(void *pvParameters) {
    while (true) {
        IMU::update();
        pitch = IMU::pitch();
       roll = IMU::roll();
       yaw = IMU::yaw();
        // If the craft tilts beyond the safe angle while armed, immediately disarm
        if (isArmed && (fabs(pitch) > FLIP_ANGLE || fabs(roll) > FLIP_ANGLE)) {
            isArmed = false;
            beep(1000, 200);
            Motor::update(false, currentOutputs, targetOutputs);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ControlOutputs ctrlOut;
        computeCorrections(pitchSetpoint, rollSetpoint, yawSetpoint,
                           pitch, roll, yaw,
                           IMU::gyroX(), IMU::gyroY(), IMU::gyroZ(),
                           IMU::verticalAcc(), yawControlEnabled, ctrlOut);
        rollCorrection = ctrlOut.roll;
        pitchCorrection = ctrlOut.pitch;
        yawCorrection = ctrlOut.yaw;
        verticalCorrection = ctrlOut.vertical;
        int base = constrain(command.throttle, THROTTLE_MIN, THROTTLE_MAX);
        base = constrain(base + verticalCorrection, THROTTLE_MIN, THROTTLE_MAX);
        int pitchCorr = constrain((int)pitchCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        int rollCorr = constrain((int)rollCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        int yawCorr = constrain((int)yawCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        Motor::mix(base, pitchCorr, rollCorr, yawCorr, targetOutputs);
        Motor::update(isArmed, currentOutputs, targetOutputs);
        vTaskDelay(pdMS_TO_TICKS(5)); // ~100 Hz (adjust to 1 kHz if needed)
    }
}

void CommTask(void *pvParameters) {
    while (true) {
        handleIncomingData();
        monitorConnection();
        if (!ilitePaired && millis() - lastDiscoveryTime > 1000) {
            Comms::IdentityMessage msg = {};
            msg.type = Comms::DRONE_IDENTITY;
            strncpy(msg.identity, DRONE_ID, sizeof(msg.identity));
            memcpy(msg.mac, selfMac, 6);
            esp_now_send(Comms::BroadcastMac, (uint8_t *)&msg, sizeof(msg));
            lastDiscoveryTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz for responsiveness
    }
}

void FailsafeTask(void *pvParameters) {
    while (true) {
        checkFailsafe();
        vTaskDelay(pdMS_TO_TICKS(10)); // ~10 Hz
    }
}

void TelemetryTask(void *pvParameters) {
    while (true) {
        streamTelemetry();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void OTATask(void *pvParameters) {
    while (true) {
        ArduinoOTA.handle();  // OTA can be run slowly
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


void setup()
{
    Serial.begin(115200);
    Serial.println("Flight Controller Starting...");
    if (BUZZER_PIN >= 0) {
        // Use a standard 2 kHz buzzer tone with 8-bit resolution to avoid
        // disturbing the motor PWM timers.
        ledcSetup(BUZZER_CHANNEL, 2000, 8);
        ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
        buzzerQueue = xQueueCreate(5, sizeof(BuzzerCommand));
        CREATE_TASK(
            BuzzerTask,
            "BuzzerTask",
            BUZZER_TASK_STACK,
            1,
            NULL,
            1
        );
        beep(1000, 200);
        beep(1180, 200);
    } //50:78:7D:45:D9:F0 new mac

    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    // Initialize motor outputs
    Motor::init(PIN_MFL, PIN_MFR, PIN_MBL, PIN_MBR, PWM_RESOLUTION);
    Motor::calibrate();
    Comms::init(WIFI_SSID, WIFI_PASSWORD, TCP_PORT);
    ArduinoOTA.begin();
    server.begin();
    WiFi.macAddress(selfMac);

    esp_now_register_recv_cb(onReceive);
    Serial.println("ESP-NOW initialized");
    Serial.println("OTA service started");

    // Initialize IMU
    IMU::init();
    pitch = IMU::pitch();
    roll = IMU::roll();
    yaw = IMU::yaw();
    lastCommandTime = millis();

    // Initialize setpoints to current orientation
    pitchSetpoint = 0;
    rollSetpoint = 0;
    yawSetpoint = 0;

    Serial.println("System ready for flight!");
    delay(300);
    ControlOutputs ctrlOut;
    computeCorrections(pitchSetpoint, rollSetpoint, yawSetpoint,
                       pitch, roll, yaw,
                       IMU::gyroX(), IMU::gyroY(), IMU::gyroZ(),
                       IMU::verticalAcc(), yawControlEnabled, ctrlOut);
    rollCorrection = ctrlOut.roll;
    pitchCorrection = ctrlOut.pitch;
    yawCorrection = ctrlOut.yaw;
    verticalCorrection = ctrlOut.vertical;

    CREATE_TASK(
        FastTask,
        "FastTask",
        FAST_TASK_STACK,
        3,
        NULL,
        1
    );

    CREATE_TASK(
        CommTask,
        "CommTask",
        COMM_TASK_STACK,
        4,
        NULL,
        1 // Core 0 — use Core 0 for Wi-Fi tasks to avoid conflicts
    );

    CREATE_TASK(
        FailsafeTask,
        "FailsafeTask",
        FAILSAFE_TASK_STACK,
        1,
        NULL,
        1
    );

    CREATE_TASK(
        TelemetryTask,
        "TelemetryTask",
        TELEMETRY_TASK_STACK,
        2,
        NULL,
        1
    );

    CREATE_TASK(
        OTATask,
        "OTATask",
        OTA_TASK_STACK,
        1,
        NULL,
        0
    );

}
// ==================== MAIN LOOP ====================

void loop() {
    vTaskDelete(NULL); // Kill the default task
}
