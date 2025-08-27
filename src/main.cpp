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

// ==================== BOARD CONFIGURATION ====================
// Select pin mappings and task sizes based on target board
#define CONFIG_IDF_TARGET_ESP32C3
#if defined(CONFIG_IDF_TARGET_ESP32C3)
// ESP32-C3 Super Mini (RISC-V single core)
const int PIN_MFL = 20; // MTL
const int PIN_MFR = 10; // MTR
const int PIN_MBL = 21; // MBL
const int PIN_MBR = 9;  // MBR
const int BUZZER_PIN = 6;
const uint32_t CPU_FREQ_MHZ = 160;
// Reduced stack sizes for smaller RAM
const uint16_t FAST_TASK_STACK = 2048;
const uint16_t COMM_TASK_STACK = 4096;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t OTA_TASK_STACK = 2048;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreate(fn, name, stack, NULL, prio, handle)
#else
// Default ESP32 (e.g., NodeMCU-32S)
const int PIN_MFL = 14;
const int PIN_MFR = 27;
const int PIN_MBL = 26;
const int PIN_MBR = 25;
const int BUZZER_PIN = -1; // No buzzer by default
const uint32_t CPU_FREQ_MHZ = 240;
const uint16_t FAST_TASK_STACK = 4096;
const uint16_t COMM_TASK_STACK = 8192;
const uint16_t FAILSAFE_TASK_STACK = 2048;
const uint16_t OTA_TASK_STACK = 2048;
#define CREATE_TASK(fn, name, stack, prio, handle, core) xTaskCreatePinnedToCore(fn, name, stack, NULL, prio, handle, core)
#endif

/// ==================== CONSTANTS ====================
const char *WIFI_SSID = "Dronegaze Telemetry port";
const char *WIFI_PASSWORD = "ASCEpec@2025";
IPAddress ip = {192,168,4,1};

const int TCP_PORT = 8000;
const int EEPROM_SIZE = 512*6;
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

bool enableFilters = false; // Enable or disable filters
bool enableQuadFilters = false;

// ==================== STRUCTURES ====================

CascadedFilter pitchQuadFilter(1, 100.0, 10.0, 0.707, FilterType::LOW_PASS);
CascadedFilter rollQuadFilter(1, 100.0, 10.0, 0.707, FilterType::LOW_PASS);
CascadedFilter yawQuadFilter(1, 100.0, 10.0, 0.707, FilterType::LOW_PASS);
CascadedFilter yawAntiDrift(1, 100.0, 10.0, 0.707, FilterType::LOW_PASS);

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
ESC escFL(PIN_MFL, 0), escFR(PIN_MFR, 1), escBL(PIN_MBL, 2), escBR(PIN_MBR, 3);
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
const uint32_t EEPROM_MAGIC = 0xABCD1234; // Signature to verify EEPROM is initialized
const int EEPROM_MAGIC_ADDR = EEPROM_ADDR + EEPROM_SIZE - sizeof(uint32_t); // Store it at the end
double yawQ,pitchQ,rollQ,yawF,pitchF,rollF; 

void savePIDToEEPROM()
{
    pitchF = pitchQuadFilter.fre;
    pitchQ = pitchQuadFilter.getQ();

    rollF= rollQuadFilter.fre;
    rollQ= rollQuadFilter.getQ();

    yawF=yawQuadFilter.fre;
    yawQ=yawQuadFilter.getQ();

    EEPROM.put(EEPROM_ADDR, pitchPID);
    EEPROM.put(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.put(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController), yawQ);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController) + sizeof(double)+1,  yawF);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController)+ 2*sizeof(double)+1, pitchQ);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController) + 3*sizeof(double)+1,  pitchF);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController)+ 4*sizeof(double)+1, rollQ);
    EEPROM.put(EEPROM_ADDR + 3 * sizeof(PIDController) + 5*sizeof(double)+1,  rollF);

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
    yawQ=0.707;
    yawF=10.0;

    pitchQ=0.707;
    pitchF=10.0;

    rollQ=0.707;
    rollF=10.0;
  
        Serial.println("EEPROM not initialized or corrupted. Using defaults.");
        return; // Don’t load anything; system will use default constructed PIDs
    }

    EEPROM.get(EEPROM_ADDR, pitchPID);
    EEPROM.get(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.get(EEPROM_ADDR + 2 * sizeof(PIDController), yawPID);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController), yawQ);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController) + sizeof(double)+1,  yawF);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController)+ 2*sizeof(double)+1, pitchQ);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController) + 3*sizeof(double)+1,  pitchF);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController)+ 4*sizeof(double)+1, rollQ);
    EEPROM.get(EEPROM_ADDR + 3 * sizeof(PIDController) + 5*sizeof(double)+1,  rollF);

    pitchPID.reset();
    rollPID.reset();
    yawPID.reset();

    pitchQuadFilter.setFrequency(pitchF);
    pitchQuadFilter.setQ(pitchQ);

    rollQuadFilter.setFrequency(rollF);
    rollQuadFilter.setQ(rollQ);

    yawQuadFilter.setFrequency(yawF);
    yawQuadFilter.setQ(yawQ);



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
        sendLine("PITCH_FILTER " + String(pitchQuadFilter.getQ(), 3) + " " + String(pitchQuadFilter.fre, 3));
        sendLine("ROLL_FILTER " + String(rollQuadFilter.getQ(), 3) + " " + String(rollQuadFilter.fre, 3));
        sendLine("YAW_FILTER " + String(yawQuadFilter.getQ(), 3) + " " + String(yawQuadFilter.fre, 3));
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
        yawQuadFilter.reset();
        rollQuadFilter.reset();
        pitchQuadFilter.reset();
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
            double value = trimmed.substring(secondSpace + 1).toDouble();

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
                pitchF=pitchQuadFilter.fre;
                pitchQ=pitchQuadFilter.getQ();

                rollF=rollQuadFilter.fre;
                rollQ=rollQuadFilter.getQ();

                yawF=yawQuadFilter.fre;
                yawQ=yawQuadFilter.getQ();
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
double filteredRoll ;
double filteredPitch;
double filteredYaw  ;

unsigned long lastIMUVerbose;
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

if(enableFilters){
 filteredRoll  = rollFilter.update(kalmanX.update(rollAcc, gyroXrate, dt));
 filteredPitch = pitchFilter.update(kalmanY.update(pitchAcc, gyroYrate, dt));
 filteredYaw   = yawFilter.update(kalmanZ.update(yaw, gyroZrate, dt));
}else
{
filteredRoll = kalmanX.update(rollAcc, gyroXrate, dt);
filteredPitch = kalmanY.update(pitchAcc, gyroYrate, dt);
filteredYaw = kalmanZ.update(yaw, gyroZrate, dt);
}

if(enableQuadFilters){
roll = rollQuadFilter.process(filteredRoll);
pitch = pitchQuadFilter.process(filteredPitch);
yaw = yawQuadFilter.process(filteredYaw);
}else
{
roll = filteredRoll;
pitch = filteredPitch;
yaw = yawAntiDrift.process(filteredYaw); // Use yawAntiDrift filter for yaw
}

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

    // Prevent invalid dt
    if (dt <= 0 || dt > 0.1)
    {
        // Only print this error occasionally
        static unsigned long lastDtError = 0;
        if (now - lastDtError > 1000)
        {
            // Serial.println("DEBUG: Invalid dt detected: " + String(dt, 6));
            lastDtError = now;
        }
        return;
    }

    float pitchError = -pitch; // Target is 0°
    float rollError = -roll;

    // ✅ Calculate yaw error with wrap-around handling
    float yawError = yawSetpoint - yaw;
    // Handle wrap-around for yaw error
     if (yawError > 180) yawError -= 360; //not for the time being. . .
     else if (yawError < -180) yawError += 360;

    rollCorrection =  rollPID.compute(rollError, dt);
    pitchCorrection = pitchPID.compute(pitchError, dt);
    yawCorrection = yawPID.compute(yawError, dt);


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
    kalmanX.Q_angle = 0.01f;
    kalmanX.Q_bias = 0.006f;
    kalmanX.R_measure = 0.1f;
    kalmanY.Q_angle = 0.01f;
    kalmanY.Q_bias = 0.006f;
    kalmanY.R_measure = 0.05f;
    kalmanZ.Q_angle = 0.01f; // tuned for yaw (gyro-only, no accel ref)
    kalmanZ.Q_bias = 0.006f;
    kalmanZ.R_measure = 0.1f;

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

void FastTask(void *pvParameters) {
    while (true) {
        updateIMU();
        updatePIDControllers();
        calculateMotorMix();
        updateMotorOutputs();
        vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz (adjust to 1 kHz if needed)
    }
}

void CommTask(void *pvParameters) {
    while (true) {
        handleIncomingData();
        streamTelemetry();
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
    delay(1000);
    if (BUZZER_PIN >= 0) {
        pinMode(BUZZER_PIN, OUTPUT);
       tone(BUZZER_PIN, 1000);
        delay(200);
       tone(BUZZER_PIN, 0);
    } //50:78:7D:45:D9:F0 new mac

    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    loadPIDFromEEPROM();
    Serial.println(3 * sizeof(PIDController) + 3 * sizeof(CascadedFilter));
    setCpuFrequencyMhz(CPU_FREQ_MHZ);
    // Initialize ESCs
    escFL.attach();
    escFR.attach();
    escBL.attach();
    escBR.attach();
    escFL.arm();
    escFR.arm();
    escBL.arm();
    escBR.arm();

    // Initialize WiFi

    WiFi.mode(WIFI_AP_STA);
    WiFi.config(ip,ip,{255,255,255,0});
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD,1,0,4);
    ArduinoOTA.begin();
    server.begin();
    Serial.println("WiFi AP and TCP server started");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("mac Address:");
    Serial.println(WiFi.macAddress());

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
        2,
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
