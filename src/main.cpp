#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include "ESC.h"
#include <bandpass.h>
#include <EEPROM.h>
#include "QuadFilter.h"

/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Dronegaze Telemetry port";
const char *WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;
const int EEPROM_SIZE = 512 * 32;
const int EEPROM_ADDR = 0x0;

// Motor and control constants
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int BIAS_LIMIT = 100;
const int CORRECTION_LIMIT = 150;
const int EASING_RATE = 50;
const unsigned long FAILSAFE_TIMEOUT = 200;  // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms

// IMU constants
const float GYRO_SCALE = 131.0; // LSB/°/s for ±250°/s
const float rad_to_deg = 180.0 / PI;

bool failsafe_enable = 1;
bool isArmed = 0;

// ==================== STRUCTURES ====================

CascadedFilter pitchQuadFilter(2, 1000.0, 10.0, 0.707, FilterType::LOW_PASS);
CascadedFilter rollQuadFilter(2, 1000.0, 10.0, 0.707, FilterType::LOW_PASS);
CascadedFilter yawQuadFilter(2, 1000.0, 10.0, 0.707, FilterType::LOW_PASS);


struct PIDController
{
    float Kp, Ki, Kd;
    float errorSum;
    float lastError;
    bool initialized;

    PIDController(float p = 2.0, float i = 0.0, float d = 0.5)
        : Kp(p), Ki(i), Kd(d), errorSum(0), lastError(0), initialized(false) {}

    float compute(float error, float dt)
    {
        static unsigned long lastErrorMsg = 0;
        unsigned long now = millis();
        if (isnan(error) || isnan(dt) || dt <= 0)
        {
            if (now - lastErrorMsg > 2000)
            {
                Serial.println("ERROR: Invalid PID inputs - error:" + String(error) + " dt:" + String(dt));
                lastErrorMsg = now;
            }
            return 0;
        }
        if (!initialized)
        {
            lastError = error;
            initialized = true;
        }
        errorSum += error * dt;
        errorSum = constrain(errorSum, -100, 100);
        float derivative = (error - lastError) / dt;
        if (isnan(derivative))
            derivative = 0;
        lastError = error;
        float output = Kp * error + Ki * errorSum + Kd * derivative;
        if (isnan(output))
            return 0;
        return output;
    }

    void reset()
    {
        errorSum = 0;
        lastError = 0;
        initialized = false;
    }
};

struct ThrustCommand
{
    uint16_t throttle;
    int8_t pitchBias;
    int8_t rollBias;
    int8_t yawBias;
    bool arm_motors;
}; //dummy packet

struct MotorOutputs
{
    int MFL, MFR, MBL, MBR;

    MotorOutputs() : MFL(MOTOR_MIN), MFR(MOTOR_MIN), MBL(MOTOR_MIN), MBR(MOTOR_MIN) {}

    void constrainAll()
    {
        MFL = constrain(MFL, MOTOR_MIN, MOTOR_MAX);
        MFR = constrain(MFR, MOTOR_MIN, MOTOR_MAX);
        MBL = constrain(MBL, MOTOR_MIN, MOTOR_MAX);
        MBR = constrain(MBR, MOTOR_MIN, MOTOR_MAX);
    }
};

// ==================== GLOBAL VARIABLES ====================
// Hardware
MPU6050 mpu;
ESC escFL(14, 0), escFR(27, 1), escBL(26, 2), escBR(25, 3);
WiFiServer server(TCP_PORT);
WiFiClient client;
KalmanFilter kalmanX, kalmanY, kalmanZ;
BandPass pitchFilter(0.01, 0.98);
BandPass rollFilter(0.01, 0.98);
BandPass yawFilter(0.01, 0.98); // ✅ Add yaw filter
PIDController pitchPID, rollPID, yawPID;
ThrustCommand command = {THROTTLE_MIN, 0, 0, 0};
MotorOutputs currentOutputs, targetOutputs;
float pitch = 0, roll = 0, yaw = 0;
float pitchCorrection = 0, rollCorrection = 0, yawCorrection = 0;
float yawSetpoint = 0; // ✅ Add yaw setpoint for heading hold
unsigned long lastUpdate = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetry = 0;
String incomingCommand = "";

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

void savePIDToEEPROM()
{
    EEPROM.put(EEPROM_ADDR, pitchPID);
    EEPROM.put(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.put(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID); // ✅ Load yaw PID
    EEPROM.put(EEPROM_ADDR+ 3 * sizeof(PIDController) , pitchQuadFilter );
    EEPROM.put(EEPROM_ADDR+ 3 * sizeof(PIDController) + sizeof(CascadedFilter), rollQuadFilter );
    EEPROM.put(EEPROM_ADDR+ 3 * sizeof(PIDController) + 2 * sizeof(CascadedFilter), yawQuadFilter );
    EEPROM.commit();
    Serial.println("PID values saved to EEPROM");
}

void loadPIDFromEEPROM()
{
    EEPROM.get(EEPROM_ADDR, pitchPID);
    EEPROM.get(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.get(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID); // ✅ Load yaw PID
    EEPROM.get(EEPROM_ADDR+ 3 * sizeof(PIDController) , pitchQuadFilter );
    EEPROM.get(EEPROM_ADDR+ 3 * sizeof(PIDController) + sizeof(CascadedFilter), rollQuadFilter );
    EEPROM.get(EEPROM_ADDR+ 3 * sizeof(PIDController) + 2 * sizeof(CascadedFilter), yawQuadFilter );
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset(); // ✅ Reset yaw PID
    Serial.println("PID values loaded from EEPROM");
}

// ==================== COMMUNICATION FUNCTIONS ====================
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; // 1 second
bool telemetryEnabled = true;
String messageBuffer = "";
const int MAX_MESSAGE_LENGTH = 256;

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
        escFL.arm();
        escFR.arm();
        escBL.arm();
        escBR.arm();
        delay(1000);
        sendLine("ACK: Motors armed");
    }
    else if (trimmed == "disarm")
    {
        isArmed = false;
        escFL.writeMicroseconds(1000);
        escFR.writeMicroseconds(1000);
        escBL.writeMicroseconds(1000);
        escBR.writeMicroseconds(1000);
        sendLine("ACK: Motors disarmed");
    }
    else if (trimmed.startsWith("setfilter "))
    {
        // Format: setfilter [pitch|roll|yaw] [freq|q] [value]
        int firstSpace = trimmed.indexOf(' ', 10);
        int secondSpace = trimmed.indexOf(' ', firstSpace + 1);

        if (firstSpace > 0 && secondSpace > 0)
        {
            String axis = trimmed.substring(10, firstSpace);
            String param = trimmed.substring(firstSpace + 1, secondSpace);
            float value = trimmed.substring(secondSpace + 1).toFloat();

            CascadedFilter *filter = nullptr;
            if (axis == "pitch")
                filter = &pitchQuadFilter;
            else if (axis == "roll")
                filter = &rollQuadFilter;
            else if (axis == "yaw")
                filter = &yawQuadFilter;

            if (filter)
            {
                if (param == "freq")
                {
                    filter->setFrequency(value);
                    sendLine("ACK: " + axis + " filter frequency set to " + String(value));
                }
                else if (param == "q")
                {   
                    filter->setQ(value);
                    sendLine("ACK: " + axis + " filter Q set to " + String(value));
                }
                else
                {
                    sendLine("ERROR: Invalid filter parameter. Use freq or q");
                }
            }
            else
            {
                sendLine("ERROR: Invalid filter axis. Use pitch, roll, or yaw");
            }
        }
        else
        {
            sendLine("ERROR: Invalid format. Use: setfilter [pitch|roll|yaw] [freq|q] [value]");
        }
    }
    else

    {
        sendLine("ERROR: Unknown command");
        sendLine("Commands: status | set [pitch|roll|yaw] [kp|ki|kd] [value] | yaw [setpoint] | save | reset | telemetry [on|off] | ping"); // ✅ Update help
    }
}

void streamTelemetry()
{
    if (!telemetryEnabled)
        return;

    if (millis() - lastTelemetry >= TELEMETRY_INTERVAL)
    {
        lastTelemetry = millis();

        // Enhanced telemetry with yaw data
        String telemetry = "DB:" + String(pitch, 2) + " " + String(roll, 2) + " " + String(yaw, 2) + " " +
                           String(pitchCorrection, 2) + " " + String(rollCorrection, 2) + " " + String(yawCorrection, 2) + " " +
                           String(command.throttle) + " " + String(command.pitchBias) + " " +
                           String(command.rollBias) + " " + String(command.yawBias) + " " + String(millis() - lastCommandTime);
        sendLine(telemetry);
    }
}

void handleIncomingData()
{
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

// ==================== ESP-NOW CALLBACK ====================
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(ThrustCommand))
    {
        ThrustCommand prevCommand = command;
        memcpy(&command, incomingData, sizeof(ThrustCommand));
        lastCommandTime = millis();

        // ✅ Update yaw setpoint when yaw bias is applied
        if (command.yawBias > 15 || command.yawBias < -15)
        {
            yawSetpoint += command.yawBias * 0.05; // Adjust rate as needed
            // Keep yaw setpoint within -180 to 180 degrees
            if (yawSetpoint > 180)
                yawSetpoint -= 360;
            else if (yawSetpoint < -180)
                yawSetpoint += 360;
        }
    }
}

// ==================== MOTOR CONTROL ====================
int easeMotorOutput(int current, int target)
{
    if (current < target)
    {
        return min(current + EASING_RATE, target);
    }
    else
    {
        return max(current - EASING_RATE, target);
    }
}

void updateMotorOutputs()
{
    if (isArmed)
    {
        currentOutputs.MFL = easeMotorOutput(currentOutputs.MFL, targetOutputs.MFL);
        currentOutputs.MFR = easeMotorOutput(currentOutputs.MFR, targetOutputs.MFR);
        currentOutputs.MBL = easeMotorOutput(currentOutputs.MBL, targetOutputs.MBL);
        currentOutputs.MBR = easeMotorOutput(currentOutputs.MBR, targetOutputs.MBR);

        escFL.writeMicroseconds(currentOutputs.MFL);
        escFR.writeMicroseconds(currentOutputs.MFR);
        escBL.writeMicroseconds(currentOutputs.MBL);
        escBR.writeMicroseconds(currentOutputs.MBR);
    }
    else
    {
        // If not armed, set all outputs to minimum
        escFL.writeMicroseconds(MOTOR_MIN);
        escFR.writeMicroseconds(MOTOR_MIN);
        escBL.writeMicroseconds(MOTOR_MIN);
        escBR.writeMicroseconds(MOTOR_MIN);
    }
}

void calculateMotorMix()
{
    int base = command.throttle;
    int pitchBias = constrain(command.pitchBias, -BIAS_LIMIT, BIAS_LIMIT);
    int rollBias = constrain(command.rollBias, -BIAS_LIMIT, BIAS_LIMIT);

    int pitchCorr = constrain(pitchCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
    int rollCorr = constrain(rollCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
    int yawCorr = constrain(yawCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT); // ✅ Add yaw correction

    // ✅ Standard quadcopter mixing with yaw control
    // Yaw is controlled by differential thrust between CW and CCW motor pairs
    targetOutputs.MFL = base - pitchCorr + rollCorr - pitchBias + rollBias - yawCorr; // CCW motor
    targetOutputs.MFR = base - pitchCorr - rollCorr - pitchBias - rollBias + yawCorr; // CW motor
    targetOutputs.MBL = base + pitchCorr + rollCorr + pitchBias + rollBias - yawCorr; // CCW motor
    targetOutputs.MBR = base + pitchCorr - rollCorr + pitchBias - rollBias + yawCorr; // CW motor

    targetOutputs.constrainAll();
}
void updateIMU();
void updatePIDControllers();
void checkFailsafe()
{
    if (failsafe_enable)
    {
         isArmed = command.arm_motors; // Use the command to arm/disarm motors
        if (millis() - lastCommandTime > FAILSAFE_TIMEOUT)
        {
            command.throttle = THROTTLE_MIN;
            command.pitchBias = 0;
            command.rollBias = 0;
            command.yawBias = 0; // ✅ Reset yaw bias in failsafe

            // Emergency stop
            targetOutputs.MFL = targetOutputs.MFR = targetOutputs.MBL = targetOutputs.MBR = MOTOR_MIN;
            isArmed=0;

            // Only print failsafe message once per second to avoid spam
            static unsigned long lastFailsafeMessage = 0;
            if (millis() - lastFailsafeMessage > 5000)
            {
                sendLine("FAILSAFE: No commands received for " + String(millis() - lastCommandTime) + "ms");
                lastFailsafeMessage = millis();
            }
        }
    }
}

// ==================== IMU FUNCTIONS ====================
void updateIMU()
{
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0;
    if (dt <= 0 || dt > 0.1)
    {
        lastUpdate = now;
        return;
    }
    lastUpdate = now;

    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    float gyroXrate = gx / GYRO_SCALE;
    float gyroYrate = gy / GYRO_SCALE;
    float gyroZrate = gz / GYRO_SCALE;

double filteredRoll = kalmanX.update(rollAcc, gyroXrate, dt);
double filteredPitch = kalmanY.update(pitchAcc, gyroYrate, dt);
double filteredYaw = kalmanZ.update(yaw, gyroZrate, dt);

roll = rollQuadFilter.process(filteredRoll);
pitch = pitchQuadFilter.process(filteredPitch);
yaw = yawQuadFilter.process(filteredYaw);

    // Keep yaw within -180 to 180 degrees
    if (yaw > 180)
        yaw -= 360;
    else if (yaw < -180)
        yaw += 360;
}

void updatePIDControllers()
{
    unsigned long now = millis();
    static unsigned long lastPIDUpdate = 0;
    static unsigned long lastDebugPrint = 0;
    const unsigned long DEBUG_INTERVAL = 500; // Print debug every 500ms

    if (lastPIDUpdate == 0)
    {
        lastPIDUpdate = now;
        Serial.println("DEBUG: PID first call - skipping");
        return;
    }

    float dt = (now - lastPIDUpdate) / 1000.0;
    lastPIDUpdate = now;

    // Rate-limited debug timing
    bool shouldDebug = (now - lastDebugPrint) >= DEBUG_INTERVAL;
    if (shouldDebug)
    {
        Serial.println("DEBUG: PID dt = " + String(dt, 6));
    }

    // Prevent invalid dt
    if (dt <= 0 || dt > 0.1)
    {
        // Only print this error occasionally
        static unsigned long lastDtError = 0;
        if (now - lastDtError > 1000)
        {
            Serial.println("DEBUG: Invalid dt detected: " + String(dt, 6));
            lastDtError = now;
        }
        return;
    }

    float pitchError = -pitch; // Target is 0°
    float rollError = -roll;

    // ✅ Calculate yaw error with wrap-around handling
    float yawError = yawSetpoint - yaw;
    // Handle wrap-around for yaw error
    // if (yawError > 180) yawError -= 360; //not for the time being. . .
    // else if (yawError < -180) yawError += 360;

    rollCorrection = rollPID.compute(rollError, dt);
    pitchCorrection = pitchPID.compute(pitchError, dt);
    yawCorrection = yawPID.compute(yawError, dt); // ✅ Compute yaw correction

    // Rate-limited debug output
    if (shouldDebug)
    {
        Serial.println("DEBUG: pitch=" + String(pitch, 2) + " err=" + String(pitchError, 2) +
                       " corr=" + String(pitchCorrection, 2));
        Serial.println("DEBUG: roll=" + String(roll, 2) + " err=" + String(rollError, 2) +
                       " corr=" + String(rollCorrection, 2));
        Serial.println("DEBUG: yaw=" + String(yaw, 2) + " setpoint=" + String(yawSetpoint, 2) +
                       " err=" + String(yawError, 2) + " corr=" + String(yawCorrection, 2)); // ✅ Debug yaw
        Serial.println("DEBUG: Pitch PID - Kp:" + String(pitchPID.Kp, 2) +
                       " Ki:" + String(pitchPID.Ki, 3) +
                       " Kd:" + String(pitchPID.Kd, 2));
        Serial.println("DEBUG: Yaw PID - Kp:" + String(yawPID.Kp, 2) +
                       " Ki:" + String(yawPID.Ki, 3) +
                       " Kd:" + String(yawPID.Kd, 2)); // ✅ Debug yaw PID
        lastDebugPrint = now;
    }

    // NaN errors should be immediate but rate-limited
    if (isnan(pitchCorrection) || isnan(rollCorrection) || isnan(yawCorrection))
    {
        static unsigned long lastNaNError = 0;
        if (now - lastNaNError > 1000)
        {
            Serial.println("ERROR: NaN in PID corrections");
            lastNaNError = now;
        }
        pitchCorrection = 0;
        rollCorrection = 0;
        yawCorrection = 0; // ✅ Reset yaw correction on NaN
    }
}

void setupKalmanFilters()
{
    kalmanX.Q_angle = 0.001f;
    kalmanX.Q_bias = 0.003f;
    kalmanX.R_measure = 0.03f;
    kalmanY.Q_angle = 0.001f;
    kalmanY.Q_bias = 0.003f;
    kalmanY.R_measure = 0.03f;
    kalmanZ.Q_angle = 0.005f; // tuned for yaw (gyro-only, no accel ref)
    kalmanZ.Q_bias = 0.003f;
    kalmanZ.R_measure = 0.05f;

    Serial.println("Priming Kalman filters...");
    for (int i = 0; i < 100; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        float rollAcc = atan2(ay, az) * RAD_TO_DEG;
        float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
        float gyroZrate = gz / GYRO_SCALE;

        if (!isnan(rollAcc) && !isnan(pitchAcc))
        {
            kalmanX.update(rollAcc, gx / GYRO_SCALE, 0.01);
            kalmanY.update(pitchAcc, gy / GYRO_SCALE, 0.01);
            yaw = kalmanZ.update(yaw, gyroZrate, 0.01); // prime yaw
        }
        delay(10);
    }
    Serial.println("Kalman filters primed");
}

// ==================== SETUP ====================
void setup()
{
    Serial.begin(115200);
    Serial.println("Flight Controller Starting...");

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    loadPIDFromEEPROM();

    // Initialize ESCs
    escFL.attach();
    escFR.attach();
    escBL.attach();
    escBR.attach();
    escFL.arm();
    escFR.arm();
    escBL.arm();
    escBR.arm();
    // delay(1000);
    // escFL.writeMicroseconds(2000);
    // escFR.writeMicroseconds(2000);
    // escBL.writeMicroseconds(2000);
    // escBR.writeMicroseconds(2000);
    // delay(10);
    // escFL.writeMicroseconds(1000);
    // escFR.writeMicroseconds(1000);
    // escBL.writeMicroseconds(1000);
    // escBR.writeMicroseconds(1000);
    // delay(2000);

    // Initialize WiFi
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    ArduinoOTA.begin();
    server.begin();
    Serial.println("WiFi AP and TCP server started");

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    Serial.println("ESP-NOW initialized");

    Serial.println("OTA service started");

    // Initialize IMU
    Wire.begin();
    mpu.initialize();
    mpu.setDLPFMode(3); // 44Hz low-pass filter

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed");
    }

    // Configure Kalman filters
    setupKalmanFilters();

    pitchQuadFilter.setFrequency(10.0);
    pitchQuadFilter.setQ(0.707);

    rollQuadFilter.setFrequency(10.0);
    rollQuadFilter.setQ(0.707);

    yawQuadFilter.setFrequency(10.0);
    yawQuadFilter.setQ(0.707);


    lastUpdate = millis();
    lastCommandTime = millis();

    // ✅ Initialize yaw setpoint to current yaw
    yawSetpoint = yaw;

    Serial.println("System ready for flight!");
    delay(2000);
    pitchPID.reset();
    rollPID.reset();
    yawPID.reset(); // ✅ Reset yaw PID
    updatePIDControllers();
}
unsigned long last_handle_time;
// ==================== MAIN LOOP ====================
void loop()
{
    ArduinoOTA.handle(); // Handle OTA updates
    if(millis()-last_handle_time>=10){
    checkFailsafe();
    updateIMU();
    updatePIDControllers();
    calculateMotorMix();
    updateMotorOutputs();
    handleIncomingData();
    streamTelemetry();
    // sendHeartbeat();
    }
    static unsigned long lastLoopTime = 0;
    // Watchdog - restart if system hangs
    if (millis() - lastLoopTime > 100)
    {
        // System might be hanging, but just log it
        if (millis() - lastLoopTime > 1000)
        {
            sendLine("WARNING: Long loop time: " + String(millis() - lastLoopTime) + "ms" + "free heap:" + String(ESP.getFreeHeap()));
        }
    last_handle_time = millis();
    }
    lastLoopTime = millis();
}