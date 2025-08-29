#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include <EEPROM.h>
#include <cstring>
#include "comms.h"
#include "pid.h"
#include "imu.h"
#include "motor.h"

// ==================== BOARD CONFIGURATION ====================
// Select pin mappings and task sizes based on target board

#define CONFIG_IDF_TARGET_ESP32C3

#if defined(CONFIG_IDF_TARGET_ESP32C3)
// ESP32-C3 Super Mini (RISC-V single core)
const int PIN_MFL = 1; // MTL
const int PIN_MFR = 2; // MTR
const int PIN_MBL = 10; // MBL
const int PIN_MBR = 3;  // MBR
const int BUZZER_PIN = 6;
const uint32_t CPU_FREQ_MHZ = 160;

const int PWM_RESOLUTION = 14;

// Reduced stack sizes for smaller RAM
const uint16_t FAST_TASK_STACK = 2048*2;
const uint16_t COMM_TASK_STACK = 4096*2;
const uint16_t FAILSAFE_TASK_STACK = 2048*2;
const uint16_t OTA_TASK_STACK = 2048*2;
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
const uint16_t OTA_TASK_STACK = 2048;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)
#endif

const int BUZZER_CHANNEL = 4;


/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Dronegaze Telemetry port";
const char *WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;
const int EEPROM_SIZE = 512*6;
const int EEPROM_ADDR = 0x0;

// Motor and control constants
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int THROTTLE_HOVER = (THROTTLE_MIN + THROTTLE_MAX) / 2;
const int THROTTLE_DEADBAND = 20;
const int CORRECTION_LIMIT = 150;
const int EASING_RATE = 2000; // respond almost instantly
const unsigned long FAILSAFE_TIMEOUT = 200;  // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms
const float ALTITUDE_ACC_GAIN = 20.0f; // throttle units per m/s^2

// IMU constants
const float GYRO_SCALE = 131.0; // LSB/°/s for ±250°/s
const float rad_to_deg = 180.0 / PI;

// ESC calibration is disabled by default to prevent unintended motor spin-ups.
// Set to true when you explicitly want to calibrate ESCs on the next boot.
const bool ENABLE_ESC_CALIBRATION = false;

bool failsafe_enable = 1;
bool isArmed = 0;

bool enableFilters = false; // Enable or disable filters
bool enableQuadFilters = false;

const char *DRONE_ID = "DrongazeA1";
const uint32_t PACKET_MAGIC = 0xA1B2C3D4;

// ==================== GLOBAL VARIABLES ====================
// Hardware
WiFiServer server(TCP_PORT);
WiFiClient client;
PIDController pitchPID, rollPID, yawPID;
PIDController altitudePID(2.0, 0.0, 0.5); // PID for altitude hold
PIDController verticalAccelPID(ALTITUDE_ACC_GAIN, 0.0, 5.0); // PID to damp vertical acceleration
Comms::ThrustCommand command = {PACKET_MAGIC, THROTTLE_MIN, 0, 0, 0, false};
Motor::Outputs currentOutputs{MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
Motor::Outputs targetOutputs{MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
float pitch = 0, roll = 0, yaw = 0;
float pitchCorrection = 0, rollCorrection = 0, yawCorrection = 0;
float altitudeCorrection = 0;
float pitchSetpoint = 0, rollSetpoint = 0, yawSetpoint = 0; // angle targets
float altitudeSetpoint = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetry = 0;
String incomingCommand = "";
unsigned long buzzerOffTime = 0;

// ==================== EEPROM FUNCTIONS ====================

// Structure to save only the PID parameters, not runtime state
struct PIDParams
{
    float Kp, Ki, Kd;
    uint32_t checksum; // Simple validation
};

// Calculate simple checksum for validation
uint32_t calculateChecksum(const PIDParams &params)
{
    uint32_t sum = 0;
    sum += (uint32_t)(params.Kp * 1000);
    sum += (uint32_t)(params.Ki * 1000);
    sum += (uint32_t)(params.Kd * 1000);
    return sum;
}
const uint32_t EEPROM_MAGIC = 0xABCD1234; // Signature to verify EEPROM is initialized
const int EEPROM_MAGIC_ADDR = EEPROM_ADDR + EEPROM_SIZE - sizeof(uint32_t); // Store it at the end

void savePIDToEEPROM()
{
    EEPROM.put(EEPROM_ADDR, pitchPID);
    EEPROM.put(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.put(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID);

    EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);  // Write the magic number
    EEPROM.commit();
    Serial.println(" PID values saved to EEPROM");
}


void loadPIDFromEEPROM()
{
    uint32_t storedMagic = 0;
    EEPROM.get(EEPROM_MAGIC_ADDR, storedMagic);

    if (storedMagic != EEPROM_MAGIC)
    {
        Serial.println("EEPROM not initialized or corrupted. Using defaults.");
        return; // Don’t load anything; system will use default constructed PIDs
    }

    EEPROM.get(EEPROM_ADDR, pitchPID);
    EEPROM.get(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.get(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID);

    pitchPID.reset();
    rollPID.reset();
    yawPID.reset();
    verticalAccelPID.reset();

    Serial.println("PID values loaded from EEPROM");
}

// ==================== COMMUNICATION FUNCTIONS ====================
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; // 1 second
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

void sendLine(const String &line)
{
    // Always send to Serial
    Serial.println(line);

    // Send to TCP client if connected
    if (client && client.connected())
    {
        client.println(line);
        client.flush(); // Ensure data is sent immediately
    }
}

void sendHeartbeat()
{
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL)
    {
        lastHeartbeat = millis();
        sendLine("HEARTBEAT:" + String(millis()));
    }
}

void handleCommand(const String &cmd)
{
    String trimmed = cmd;
    trimmed.trim();

    if (trimmed.length() == 0)
        return;

    if (trimmed == "status")
    {
        sendLine("Pitch PID: Kp=" + String(pitchPID.Kp, 3) + " Ki=" + String(pitchPID.Ki, 3) + " Kd=" + String(pitchPID.Kd, 3));
        sendLine("Roll  PID: Kp=" + String(rollPID.Kp, 3) + " Ki=" + String(rollPID.Ki, 3) + " Kd=" + String(rollPID.Kd, 3));
        sendLine("Yaw   PID: Kp=" + String(yawPID.Kp, 3) + " Ki=" + String(yawPID.Ki, 3) + " Kd=" + String(yawPID.Kd, 3)); // ✅ Show yaw PID
        sendLine("System: " + String(millis()) + "ms uptime");
        sendLine("Last Command: " + String(millis() - lastCommandTime) + "ms ago");
        sendLine("Yaw Setpoint: " + String(yawSetpoint, 2) + "°"); // ✅ Show yaw setpoint
    }else
    if(trimmed == "failsafe on"){failsafe_enable=1; sendLine("Enabled failsafe mode");}else if (trimmed == "failsafe off"){failsafe_enable=0; sendLine("Disabled failsafe mode");}
    else if(trimmed == "filters on"){enableFilters = true; sendLine("Enabled filters");}
    else if(trimmed == "filters off"){enableFilters = false; sendLine("Disabled filters");}
    else if(trimmed == "quadfilters on"){enableQuadFilters = true; sendLine("Enabled quad filters");}
    else if(trimmed == "quadfilters off"){enableQuadFilters = false; sendLine("Disabled quad filters");}
    else if (trimmed == "save")
    {
        savePIDToEEPROM();
        sendLine("ACK: PID values saved to EEPROM");
    }
    else if (trimmed == "reset")
    {
        pitchPID.reset();
        rollPID.reset();
        yawPID.reset(); // ✅ Reset yaw PID
        verticalAccelPID.reset();
        sendLine("ACK: PID controllers reset");
    }
    else if (trimmed == "telemetry on")
    {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    }
    else if (trimmed == "telemetry off")
    {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    }
    else if (trimmed == "help")
    {
        sendLine("Available commands:");
        sendLine("status - Show current PID values and system status");
        sendLine("set [pitch|roll|yaw] [kp|ki|kd] [value] - Set PID parameters");
        sendLine("yaw [setpoint] - Set yaw setpoint for heading hold");
        sendLine("save - Save PID values to EEPROM");
        sendLine("reset - Reset PID controllers");
        sendLine("telemetry on/off - Enable or disable telemetry");
        sendLine("ping - Check connection");
        sendLine("arm - Arm motors");
        sendLine("disarm - Disarm motors");
        sendLine("setfilter [pitch|roll|yaw] [freq|q] [value] - Set filter parameters");
    }
    else if (trimmed.startsWith("set "))
    {
        // Parse: set [pitch|roll|yaw] [kp|ki|kd] [value]
        int firstSpace = trimmed.indexOf(' ', 4);
        int secondSpace = trimmed.indexOf(' ', firstSpace + 1);

        if (firstSpace > 0 && secondSpace > 0)
        {
            String axis = trimmed.substring(4, firstSpace);
            String param = trimmed.substring(firstSpace + 1, secondSpace);
            float value = trimmed.substring(secondSpace + 1).toFloat();

            PIDController *pid = nullptr;
            if (axis == "pitch")
                pid = &pitchPID;
            else if (axis == "roll")
                pid = &rollPID;
            else if (axis == "yaw") // ✅ Add yaw PID tuning
                pid = &yawPID;

            if (pid != nullptr)
            {
                if (param == "kp")
                    pid->Kp = value;
                else if (param == "ki")
                    pid->Ki = value;
                else if (param == "kd")
                    pid->Kd = value;
                else
                {
                    sendLine("ERROR: Invalid parameter. Use kp, ki, or kd");
                    return;
                }
                sendLine("ACK: Updated " + axis + " " + param + " to " + String(value, 3));
            }
            else
            {
                sendLine("ERROR: Invalid axis. Use pitch, roll, or yaw"); // ✅ Update error message
            }
        }
        else
        {
            sendLine("ERROR: Invalid format. Use: set [pitch|roll|yaw] [kp|ki|kd] [value]"); // ✅ Update help
        }
    }
    else if (trimmed.startsWith("yaw ")) // ✅ Add yaw setpoint command
    {
        float newYawSetpoint = trimmed.substring(4).toFloat();
        yawSetpoint = newYawSetpoint;
        sendLine("ACK: Yaw setpoint set to " + String(yawSetpoint, 2) + "°");
    }
    else if (trimmed == "save")
    {
        savePIDToEEPROM();
        sendLine("ACK: PID values saved to EEPROM");
    }
    else if (trimmed == "reset")
    {
        pitchPID.reset();
        rollPID.reset();
        yawPID.reset(); // ✅ Reset yaw PID
        verticalAccelPID.reset();
        sendLine("ACK: PID controllers reset");
    }
    else if (trimmed == "telemetry on")
    {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    }
    else if (trimmed == "telemetry off")
    {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    }
    else if (trimmed == "ping")
    {
        sendLine("PONG");
    }
    else if (trimmed == "arm")
    {
        isArmed = true;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        delay(1000);
        sendLine("ACK: Motors armed");
    }
    else if (trimmed == "disarm")
    {
        isArmed = false;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        sendLine("ACK: Motors disarmed");
    }
    else
    {
        sendLine("ERROR: Unknown command");
        sendLine("Commands: status | set [pitch|roll|yaw] [kp|ki|kd] [value] | yaw [setpoint] | save | reset | telemetry [on|off] | ping"); // ✅ Update help
    }
}


void updateBuzzer()
{
    if (BUZZER_PIN >= 0 && buzzerOffTime && millis() > buzzerOffTime)
    {
        ledcWrite(BUZZER_CHANNEL, 0);
        buzzerOffTime = 0;
    }
}

void beep(uint16_t freq, uint16_t duration)
{
    if (BUZZER_PIN < 0)
        return;
    ledcWriteTone(BUZZER_CHANNEL, freq);
    buzzerOffTime = millis() + duration;

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
        IMU::altitude(), IMU::verticalAcc(),
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
                       String(command.rollAngle) + " " + String(command.yawAngle) + " " + String(IMU::altitude(), 2) + " " +
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
            sendLine("ACK: TCP client connected");
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
                handleCommand(messageBuffer);
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
            sendLine("ERROR: Message too long");
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
                handleCommand(messageBuffer);
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
            sendLine("ERROR: Message too long");
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


        else if (msg.type == Comms::ILITE_IDENTITY && !ilitePaired && now - lastHandshakeSent > HANDSHAKE_COOLDOWN)
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

        int throttleDelta = command.throttle - THROTTLE_HOVER;
        if (abs(throttleDelta) > THROTTLE_DEADBAND)
        {
            altitudeSetpoint += throttleDelta * 0.01f;
        }
    }
}

void checkFailsafe()
{
    static unsigned long lastAlarmTime = 0;
    updateBuzzer();
    if (failsafe_enable)
    {
        // Only arm when paired controller explicitly arms
        isArmed = ilitePaired && command.arm_motors;
        if (millis() - lastCommandTime > FAILSAFE_TIMEOUT)
        {
            command.throttle = THROTTLE_MIN;
            command.pitchAngle = 0;
            command.rollAngle = 0;
            command.yawAngle = yaw; // hold current yaw on failsafe
            pitchSetpoint = rollSetpoint = 0;
            yawSetpoint = yaw;

            // Emergency stop
            targetOutputs.MFL = targetOutputs.MFR = targetOutputs.MBL = targetOutputs.MBR = MOTOR_MIN;
            isArmed = 0;

            if (BUZZER_PIN >= 0 && millis() - lastAlarmTime > 5000)
            {
                beep(800, 200); // periodic short alarm
                lastAlarmTime = millis();
            }

            // Only print failsafe message once per second to avoid spam
            static unsigned long lastFailsafeMessage = 0;
            if (millis() - lastFailsafeMessage > 5000)
            {
                sendLine("FAILSAFE: No commands received for " + String(millis() - lastCommandTime) + "ms");
                lastFailsafeMessage = millis();
            }
        }
        else
        {
            if (BUZZER_PIN >= 0)
            {
                beep(0,1);
            }
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
        PIDOutputs pidOut;
        updatePIDControllers(pitchSetpoint, rollSetpoint, yawSetpoint, altitudeSetpoint,
                              pitch, roll, yaw, IMU::altitude(), IMU::verticalAcc(),
                              pitchPID, rollPID, yawPID, altitudePID, verticalAccelPID, pidOut);
        rollCorrection = pidOut.roll;
        pitchCorrection = pidOut.pitch;
        yawCorrection = pidOut.yaw;
        altitudeCorrection = pidOut.altitude;
        int base = constrain(command.throttle, THROTTLE_MIN, THROTTLE_MAX);
        base = constrain(base + altitudeCorrection, THROTTLE_MIN, THROTTLE_MAX);
        int pitchCorr = constrain((int)pitchCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        int rollCorr = constrain((int)rollCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        int yawCorr = constrain((int)yawCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
        Motor::mix(base, pitchCorr, rollCorr, yawCorr, targetOutputs);
        Motor::update(isArmed, currentOutputs, targetOutputs);
        vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz (adjust to 1 kHz if needed)
    }
}

void CommTask(void *pvParameters) {
    while (true) {
        handleIncomingData();
        streamTelemetry();
        monitorConnection();
        if (!ilitePaired && millis() - lastDiscoveryTime > 1000) {
            Comms::IdentityMessage msg = {};
            msg.type = Comms::DRONE_IDENTITY;
            strncpy(msg.identity, DRONE_ID, sizeof(msg.identity));
            memcpy(msg.mac, selfMac, 6);
            esp_now_send(Comms::BroadcastMac, (uint8_t *)&msg, sizeof(msg));
            lastDiscoveryTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // ~20 Hz
    }
}

void FailsafeTask(void *pvParameters) {
    while (true) {
        checkFailsafe();
        vTaskDelay(pdMS_TO_TICKS(10)); // ~10 Hz
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
        ledcSetup(BUZZER_CHANNEL, 1000, PWM_RESOLUTION);
        ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
        beep(1000, 200);
        delay(200);
        updateBuzzer();
        beep(1180, 200);
        delay(200);
        updateBuzzer();
    } //50:78:7D:45:D9:F0 new mac

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    loadPIDFromEEPROM();
    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    // Initialize motor outputs
    Motor::init(PIN_MFL, PIN_MFR, PIN_MBL, PIN_MBR, PWM_RESOLUTION);
    if (ENABLE_ESC_CALIBRATION) {
        // ESC calibration not implemented in modular version
    }
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

    // Initialize setpoints to current orientation/altitude
    pitchSetpoint = 0;
    rollSetpoint = 0;
    yawSetpoint = 0;

    Serial.println("System ready for flight!");
    delay(300);
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset(); // ✅ Reset yaw PID
    verticalAccelPID.reset();
    PIDOutputs pidOut;
    updatePIDControllers(pitchSetpoint, rollSetpoint, yawSetpoint, altitudeSetpoint,
                          pitch, roll, yaw, IMU::altitude(), IMU::verticalAcc(),
                          pitchPID, rollPID, yawPID, altitudePID, verticalAccelPID, pidOut);
    rollCorrection = pidOut.roll;
    pitchCorrection = pidOut.pitch;
    yawCorrection = pidOut.yaw;
    altitudeCorrection = pidOut.altitude;

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
