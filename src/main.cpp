#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <esp_now.h>
#include "ESC.h"
#include <bandpass.h>
#include <EEPROM.h>

// ==================== CONSTANTS ====================
const char* WIFI_SSID = "Dronegaze Telemetry port";
const char* WIFI_PASSWORD = "ASCEpec@2025";
const int TCP_PORT = 8000;
const int EEPROM_SIZE = 512*8;
const int EEPROM_ADDR = 0x0;

// Motor and control constants
const int MOTOR_MIN = 1000;
const int MOTOR_MAX = 2000;
const int THROTTLE_MIN = 1000;
const int THROTTLE_MAX = 2000;
const int BIAS_LIMIT = 100;
const int CORRECTION_LIMIT = 150;
const int EASING_RATE = 2;
const unsigned long FAILSAFE_TIMEOUT = 500; // ms
const unsigned long TELEMETRY_INTERVAL = 50; // ms

// IMU constants
const float GYRO_SCALE = 131.0; // LSB/°/s for ±250°/s
const float rad_to_deg = 180.0 / PI;

// ==================== STRUCTURES ====================
// ==================== IMPROVED PID CONTROLLER ====================
struct PIDController {
    float Kp, Ki, Kd;
    float errorSum;
    float lastError;
    bool initialized;
    
    PIDController(float p = 2.0, float i = 0.0, float d = 0.5) 
        : Kp(p), Ki(i), Kd(d), errorSum(0), lastError(0), initialized(false) {}
    
    float compute(float error, float dt) {
        // Rate-limited error messages
        static unsigned long lastErrorMsg = 0;
        unsigned long now = millis();
        
        if (isnan(error) || isnan(dt) || dt <= 0) {
            if (now - lastErrorMsg > 2000) {
                Serial.println("ERROR: Invalid PID inputs - error:" + String(error) + " dt:" + String(dt));
                lastErrorMsg = now;
            }
            return 0;
        }
        
        if (!initialized) {
            lastError = error;
            initialized = true;
            // Continue with calculation instead of returning 0
        }
        
        errorSum += error * dt;
        errorSum = constrain(errorSum, -100, 100);
        
        float derivative = initialized ? (error - lastError) / dt : 0;
        
        if (isnan(derivative)) {
            if (now - lastErrorMsg > 2000) {
                Serial.println("ERROR: NaN in derivative");
                lastErrorMsg = now;
            }
            derivative = 0;
        }
        
        lastError = error;
        
        float output = Kp * error + Ki * errorSum + Kd * derivative;
        
        if (isnan(output)) {
            if (now - lastErrorMsg > 2000) {
                Serial.println("ERROR: NaN in PID output");
                lastErrorMsg = now;
            }
            return 0;
        }
        
        return output;
    }
    
    void reset() {
        errorSum = 0;
        lastError = 0;
        initialized = false;
    }
};

struct ThrustCommand {
    uint16_t throttle;
    int8_t pitchBias;
    int8_t rollBias;
};

struct MotorOutputs {
    int MFL, MFR, MBL, MBR;
    
    MotorOutputs() : MFL(MOTOR_MIN), MFR(MOTOR_MIN), MBL(MOTOR_MIN), MBR(MOTOR_MIN) {}
    
    void constrainAll() {
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

// Filters
KalmanFilter kalmanX, kalmanY;
BandPass pitchFilter(0.01, 0.98);
BandPass rollFilter(0.01, 0.98);

// Control
PIDController pitchPID, rollPID;
ThrustCommand command = {THROTTLE_MIN, 0, 0};
MotorOutputs currentOutputs, targetOutputs;

// State variables
float pitch = 0, roll = 0;
float pitchCorrection = 0, rollCorrection = 0;
unsigned long lastUpdate = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetry = 0;
String incomingCommand = "";

// ==================== EEPROM FUNCTIONS ====================

// Structure to save only the PID parameters, not runtime state
struct PIDParams {
    float Kp, Ki, Kd;
    uint32_t checksum;  // Simple validation
};

// Calculate simple checksum for validation
uint32_t calculateChecksum(const PIDParams& params) {
    uint32_t sum = 0;
    sum += (uint32_t)(params.Kp * 1000);
    sum += (uint32_t)(params.Ki * 1000);
    sum += (uint32_t)(params.Kd * 1000);
    return sum;
}

void savePIDToEEPROM() {
    EEPROM.put(EEPROM_ADDR, pitchPID);
    EEPROM.put(EEPROM_ADDR + sizeof(PIDController), rollPID);
    EEPROM.commit();
    Serial.println("✅ PID values saved to EEPROM");
}

void loadPIDFromEEPROM() {
    EEPROM.get(EEPROM_ADDR, pitchPID);
    EEPROM.get(EEPROM_ADDR + sizeof(PIDController), rollPID);
    pitchPID.reset();
    rollPID.reset();
    Serial.println("📦 PID values loaded from EEPROM");
}

// ==================== COMMUNICATION FUNCTIONS ====================
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000; // 1 second
bool telemetryEnabled = true;
String messageBuffer = "";
const int MAX_MESSAGE_LENGTH = 256;

// ==================== IMPROVED COMMUNICATION FUNCTIONS ====================

void sendLine(const String& line) {
    // Always send to Serial
    Serial.println(line);
    
    // Send to TCP client if connected
    if (client && client.connected()) {
        client.println(line);
        client.flush(); // Ensure data is sent immediately
    }
}

void sendHeartbeat() {
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = millis();
        sendLine("HEARTBEAT:" + String(millis()));
    }
}

void handleCommand(const String& cmd) {
    String trimmed = cmd;
    trimmed.trim();
    
    if (trimmed.length() == 0) return;
    
    if (trimmed == "status") {
        sendLine("Pitch PID: Kp=" + String(pitchPID.Kp, 3) + " Ki=" + String(pitchPID.Ki, 3) + " Kd=" + String(pitchPID.Kd, 3));
        sendLine("Roll  PID: Kp=" + String(rollPID.Kp, 3) + " Ki=" + String(rollPID.Ki, 3) + " Kd=" + String(rollPID.Kd, 3));
        sendLine("System: " + String(millis()) + "ms uptime");
        sendLine("Last Command: " + String(millis() - lastCommandTime) + "ms ago");
    }
    else if (trimmed.startsWith("set ")) {
        // Parse: set [pitch|roll] [kp|ki|kd] [value]
        int firstSpace = trimmed.indexOf(' ', 4);
        int secondSpace = trimmed.indexOf(' ', firstSpace + 1);
        
        if (firstSpace > 0 && secondSpace > 0) {
            String axis = trimmed.substring(4, firstSpace);
            String param = trimmed.substring(firstSpace + 1, secondSpace);
            float value = trimmed.substring(secondSpace + 1).toFloat();
            
            PIDController* pid = nullptr;
            if (axis == "pitch") pid = &pitchPID;
            else if (axis == "roll") pid = &rollPID;
            
            if (pid != nullptr) {
                if (param == "kp") pid->Kp = value;
                else if (param == "ki") pid->Ki = value;
                else if (param == "kd") pid->Kd = value;
                else {
                    sendLine("ERROR: Invalid parameter. Use kp, ki, or kd");
                    return;
                }
                sendLine("ACK: Updated " + axis + " " + param + " to " + String(value, 3));
            } else {
                sendLine("ERROR: Invalid axis. Use pitch or roll");
            }
        } else {
            sendLine("ERROR: Invalid format. Use: set [pitch|roll] [kp|ki|kd] [value]");
        }
    }
    else if (trimmed == "save") {
        savePIDToEEPROM();
        sendLine("ACK: PID values saved to EEPROM");
    }
    else if (trimmed == "reset") {
        pitchPID.reset();
        rollPID.reset();
        sendLine("ACK: PID controllers reset");
    }
    else if (trimmed == "telemetry on") {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    }
    else if (trimmed == "telemetry off") {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    }
    else if (trimmed == "ping") {
        sendLine("PONG");
    }
    else {
        sendLine("ERROR: Unknown command");
        sendLine("Commands: status | set [pitch|roll] [kp|ki|kd] [value] | save | reset | telemetry [on|off] | ping");
    }
}

void streamTelemetry() {
    if (!telemetryEnabled) return;
    
    if (millis() - lastTelemetry >= TELEMETRY_INTERVAL) {
        lastTelemetry = millis();
        
        // Enhanced telemetry with more data
        String telemetry = "DB:" + String(pitch, 2) + " " + String(roll, 2) + " " + 
                          String(pitchCorrection, 2) + " " + String(rollCorrection, 2) + " " +
                          String(command.throttle) + " " + String(command.pitchBias) + " " + 
                          String(command.rollBias) + " " + String(millis() - lastCommandTime);
        sendLine(telemetry);
    }
}

void handleIncomingData() {
    // Handle TCP client connection
    if (!client || !client.connected()) {
        WiFiClient newClient = server.available();
        if (newClient) {
            if (client) client.stop(); // Close old connection
            client = newClient;
            sendLine("ACK: TCP client connected");
        }
    }
    
    // Process TCP data with buffering
    while (client && client.available()) {
        char c = client.read();
        if (c == '\n' || c == '\r') {
            if (messageBuffer.length() > 0) {
                handleCommand(messageBuffer);
                messageBuffer = "";
            }
        } else if (messageBuffer.length() < MAX_MESSAGE_LENGTH) {
            messageBuffer += c;
        } else {
            // Buffer overflow protection
            messageBuffer = "";
            sendLine("ERROR: Message too long");
        }
    }
    
    // Process Serial data with buffering
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (messageBuffer.length() > 0) {
                handleCommand(messageBuffer);
                messageBuffer = "";
            }
        } else if (messageBuffer.length() < MAX_MESSAGE_LENGTH) {
            messageBuffer += c;
        } else {
            // Buffer overflow protection
            messageBuffer = "";
            sendLine("ERROR: Message too long");
        }
    }
}


// ==================== ESP-NOW CALLBACK ====================
void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
    if (len == sizeof(ThrustCommand)) {
        memcpy(&command, incomingData, sizeof(ThrustCommand));
        lastCommandTime = millis();
    }
}

// ==================== MOTOR CONTROL ====================
int easeMotorOutput(int current, int target) {
    if (current < target) {
        return min(current + EASING_RATE, target);
    } else {
        return max(current - EASING_RATE, target);
    }
}

void updateMotorOutputs() {
    currentOutputs.MFL = easeMotorOutput(currentOutputs.MFL, targetOutputs.MFL);
    currentOutputs.MFR = easeMotorOutput(currentOutputs.MFR, targetOutputs.MFR);
    currentOutputs.MBL = easeMotorOutput(currentOutputs.MBL, targetOutputs.MBL);
    currentOutputs.MBR = easeMotorOutput(currentOutputs.MBR, targetOutputs.MBR);
    
    escFL.writeMicroseconds(currentOutputs.MFL);
    escFR.writeMicroseconds(currentOutputs.MFR);
    escBL.writeMicroseconds(currentOutputs.MBL);
    escBR.writeMicroseconds(currentOutputs.MBR);
}

void calculateMotorMix() {
    int base = command.throttle;
    int pitchBias = constrain(command.pitchBias, -BIAS_LIMIT, BIAS_LIMIT);
    int rollBias = constrain(command.rollBias, -BIAS_LIMIT, BIAS_LIMIT);
    
    int pitchCorr = constrain(pitchCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
    int rollCorr = constrain(rollCorrection, -CORRECTION_LIMIT, CORRECTION_LIMIT);
    
    // Standard quadcopter mixing
    targetOutputs.MFL = base - pitchCorr + rollCorr - pitchBias + rollBias;
    targetOutputs.MFR = base - pitchCorr - rollCorr - pitchBias - rollBias;
    targetOutputs.MBL = base + pitchCorr + rollCorr + pitchBias + rollBias;
    targetOutputs.MBR = base + pitchCorr - rollCorr + pitchBias - rollBias;
    
    targetOutputs.constrainAll();
}

void checkFailsafe() {
    if (millis() - lastCommandTime > FAILSAFE_TIMEOUT) {
        command.throttle = THROTTLE_MIN;
        command.pitchBias = 0;
        command.rollBias = 0;
        
        // Emergency stop
        targetOutputs.MFL = targetOutputs.MFR = targetOutputs.MBL = targetOutputs.MBR = MOTOR_MIN;
        
        // Only print failsafe message once per second to avoid spam
        static unsigned long lastFailsafeMessage = 0;
        if (millis() - lastFailsafeMessage > 1000) {
            sendLine("FAILSAFE: No commands received for " + String(millis() - lastCommandTime) + "ms");
            lastFailsafeMessage = millis();
        }
    }
}

// ==================== IMU FUNCTIONS ====================
void updateIMU() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0;
    
    // Prevent invalid dt values
    if (dt <= 0 || dt > 0.1) {  // Cap at 100ms
        lastUpdate = now;
        return;
    }
    
    lastUpdate = now;
    
    // Calculate accelerometer angles with NaN protection
    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    
    // Check for NaN in accelerometer calculations
    if (isnan(rollAcc) || isnan(pitchAcc)) {
        Serial.println("ERROR: NaN in accelerometer calculations");
        return;
    }
    
    // Convert gyro to °/s
    float gyroXrate = gx / GYRO_SCALE;
    float gyroYrate = gy / GYRO_SCALE;
    
    // Check for NaN in gyro calculations
    if (isnan(gyroXrate) || isnan(gyroYrate)) {
        Serial.println("ERROR: NaN in gyro calculations");
        return;
    }
    
    // Kalman filter fusion
    roll = kalmanX.update(rollAcc, gyroXrate, dt);
    pitch = kalmanY.update(pitchAcc, gyroYrate, dt);
    
    // Check Kalman filter outputs
    if (isnan(roll) || isnan(pitch)) {
        Serial.println("ERROR: NaN from Kalman filter");
        roll = 0;
        pitch = 0;
    }
}

void updatePIDControllers() {
    unsigned long now = millis();
    static unsigned long lastPIDUpdate = 0;
    static unsigned long lastDebugPrint = 0;
    const unsigned long DEBUG_INTERVAL = 500; // Print debug every 500ms
    
    if (lastPIDUpdate == 0) {
        lastPIDUpdate = now;
        Serial.println("DEBUG: PID first call - skipping");
        return;
    }
    
    float dt = (now - lastPIDUpdate) / 1000.0;
    lastPIDUpdate = now;
    
    // Rate-limited debug timing
    bool shouldDebug = (now - lastDebugPrint) >= DEBUG_INTERVAL;
    if (shouldDebug) {
        Serial.println("DEBUG: PID dt = " + String(dt, 6));
    }
    
    // Prevent invalid dt
    if (dt <= 0 || dt > 0.1) {
        // Only print this error occasionally
        static unsigned long lastDtError = 0;
        if (now - lastDtError > 1000) {
            Serial.println("DEBUG: Invalid dt detected: " + String(dt, 6));
            lastDtError = now;
        }
        return;
    }
    
    float pitchError = -pitch;  // Target is 0°
    float rollError = -roll;
    
    rollCorrection = rollPID.compute(rollError, dt);
    pitchCorrection = pitchPID.compute(pitchError, dt);

    
    // Rate-limited debug output
    if (shouldDebug) {
        Serial.println("DEBUG: pitch=" + String(pitch, 2) + " err=" + String(pitchError, 2) + 
                       " corr=" + String(pitchCorrection, 2));
        Serial.println("DEBUG: roll=" + String(roll, 2) + " err=" + String(rollError, 2) + 
                       " corr=" + String(rollCorrection, 2));
        Serial.println("DEBUG: Pitch PID - Kp:" + String(pitchPID.Kp, 2) + 
                       " Ki:" + String(pitchPID.Ki, 3) + 
                       " Kd:" + String(pitchPID.Kd, 2));
        lastDebugPrint = now;
    }
    
    // NaN errors should be immediate but rate-limited
    if (isnan(pitchCorrection) || isnan(rollCorrection)) {
        static unsigned long lastNaNError = 0;
        if (now - lastNaNError > 1000) {
            Serial.println("ERROR: NaN in PID corrections");
            lastNaNError = now;
        }
        pitchCorrection = 0;
        rollCorrection = 0;
    }
}

void setupKalmanFilters() {
    // Initialize Kalman filters with better values
    kalmanX.Q_angle = 0.001f;    // Reduced process noise
    kalmanX.Q_bias = 0.003f;
    kalmanX.R_measure = 0.03f;   // Reduced measurement noise
    
    kalmanY.Q_angle = 0.001f;
    kalmanY.Q_bias = 0.003f;
    kalmanY.R_measure = 0.03f;
    
    // Prime the filters with multiple readings
    Serial.println("Priming Kalman filters...");
    for (int i = 0; i < 100; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        float rollAcc = atan2(ay, az) * RAD_TO_DEG;
        float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
        
        // Check for valid readings
        if (!isnan(rollAcc) && !isnan(pitchAcc)) {
            kalmanX.update(rollAcc, gx / GYRO_SCALE, 0.01);
            kalmanY.update(pitchAcc, gy / GYRO_SCALE, 0.01);
        }
        
        delay(10);
    }
    Serial.println("Kalman filters primed");
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    Serial.println("🚁 Flight Controller Starting...");
    
    // Initialize EEPROM
    EEPROM.begin(EEPROM_SIZE);
    loadPIDFromEEPROM();
    
    // Initialize ESCs
    escFL.attach(); escFR.attach(); escBL.attach(); escBR.attach();
    escFL.arm(); escFR.arm(); escBL.arm(); escBR.arm();
    
    // Initialize WiFi
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    server.begin();
    Serial.println("📡 WiFi AP and TCP server started");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }
    esp_now_register_recv_cb(onReceive);
    Serial.println("✅ ESP-NOW initialized");
    
    // Initialize IMU
    Wire.begin();
    mpu.initialize();
    mpu.setDLPFMode(3);  // 44Hz low-pass filter
    
    if (!mpu.testConnection()) {
        Serial.println("❌ MPU6050 connection failed");
    }
    
    // Configure Kalman filters
    setupKalmanFilters();
    
    lastUpdate = millis();
    lastCommandTime = millis();
    
    Serial.println("✅ System ready for flight!");
    delay(2000);
    pitchPID.reset();
    rollPID.reset();
    updatePIDControllers();
}

// ==================== MAIN LOOP ====================
void loop() {
 updateIMU();
    updatePIDControllers();
    
    checkFailsafe();
    calculateMotorMix();
    updateMotorOutputs();
    
    handleIncomingData();
    streamTelemetry();
    //sendHeartbeat();
    
    // Watchdog - restart if system hangs
    static unsigned long lastLoopTime = 0;
    if (millis() - lastLoopTime > 100) {
        // System might be hanging, but just log it
        if (millis() - lastLoopTime > 1000) {
            sendLine("WARNING: Long loop time: " + String(millis() - lastLoopTime) + "ms");
        }
    }
    lastLoopTime = millis();
    
    delay(10);
}