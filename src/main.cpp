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
// Declare your filters
BandPass pitchFilter(0.01, 0.98);
BandPass rollFilter(0.01, 0.98);

const char* ssid = "ESP32_AP";       // or your router SSID
const char* password = "12345678";   // set accordingly

WiFiServer server(8000);             // same port as Processing
WiFiClient client;

struct PID {
  float Kp;
  float Ki;
  float Kd;
};

PID pitchPID = {2.0, 0.0, 0.5};
PID rollPID  = {2.0, 0.0, 0.5};

 float pitchCorrection;
 float rollCorrection;

// EEPROM address (must be large enough for 2 PID structs)
#define EEPROM_ADDR 0x0

void savePIDToEEPROM() {
  EEPROM.put(EEPROM_ADDR, pitchPID);
  EEPROM.put(EEPROM_ADDR + sizeof(PID), rollPID);
  EEPROM.commit();
  Serial.println("✅ PID values saved to EEPROM.");
}

void loadPIDFromEEPROM() {
  EEPROM.get(EEPROM_ADDR, pitchPID);
  EEPROM.get(EEPROM_ADDR + sizeof(PID), rollPID);
  Serial.println("📦 PID values loaded from EEPROM.");
}

void handleSerialCommand() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "status") {
      Serial.printf("Pitch PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", pitchPID.Kp, pitchPID.Ki, pitchPID.Kd);
      Serial.printf("Roll  PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", rollPID.Kp, rollPID.Ki, rollPID.Kd);
    }
    else if (line.startsWith("set ")) {
      // Format: set pitch Kp 2.5
      char target[10], param[10];
      float val;
      sscanf(line.c_str(), "set %s %s %f", target, param, &val);

      PID* pid = (String(target) == "pitch") ? &pitchPID : &rollPID;

      if (String(param) == "kp") pid->Kp = val;
      else if (String(param) == "ki") pid->Ki = val;
      else if (String(param) == "kd") pid->Kd = val;

      Serial.printf("Updated %s PID: Kp=%.2f Ki=%.2f Kd=%.2f\n",
                    target, pid->Kp, pid->Ki, pid->Kd);
    }
    else if (line == "save") {
      savePIDToEEPROM();
    }
    else {
      Serial.println("Commands: status | set [pitch|roll] [kp|ki|kd] [value] | save");
    }
  }
}

// ESC setup
ESC escFL(14, 0);
ESC escFR(27, 1);
ESC escBL(26, 2);
ESC escBR(25, 3);

// Command struct
struct ThrustCommand
{
    uint16_t throttle;
    int8_t pitchBias;
    int8_t rollBias;
};

ThrustCommand command = {1000, 0, 0};

// Eased output vars
int currentFL = 1000, currentFR = 1000, currentBL = 1000, currentBR = 1000;
const int easingRate = 2;

// MPU6050 setup
MPU6050 mpu;
KalmanFilter kalmanX, kalmanY;
unsigned long lastUpdate = 0;
String incomingCommand = "";
unsigned long lastTelemetry = 0;
// PID constants 
float Kp = 2.0, Ki = 0, Kd = 0;
float pitchErrorSum = constrain(pitchErrorSum, -100, 100);
float rollErrorSum = constrain(rollErrorSum, -100, 100);
float lastPitchError = 0, lastRollError = 0;

// Filtered output
float pitch = 0, roll = 0;

unsigned long lastCommandTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 500; // ms

void onReceive(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    if (len == sizeof(ThrustCommand))
    {
        memcpy(&command, incomingData, sizeof(ThrustCommand));
        lastCommandTime = millis(); // update on command receive
    }
}
void sendLine(const String& line) {
  Serial.println(line); // Local debug
  if (client && client.connected()) {
    client.println(line);  // GUI via Processing
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("status")) {
    String statusMsg1 = String("Pitch PID: Kp=") + pitchPID.Kp + " Ki=" + pitchPID.Ki + " Kd=" + pitchPID.Kd;
    String statusMsg2 = String("Roll  PID: Kp=") + rollPID.Kp + " Ki=" + rollPID.Ki + " Kd=" + rollPID.Kd;
    sendLine(statusMsg1);
    sendLine(statusMsg2);
  } else if (cmd.startsWith("set pitch kp")) {
    pitchPID.Kp = cmd.substring(13).toFloat();
  } else if (cmd.startsWith("set pitch ki")) {
    pitchPID.Ki = cmd.substring(13).toFloat();
  } else if (cmd.startsWith("set pitch kd")) {
    pitchPID.Kd = cmd.substring(13).toFloat();
  } else if (cmd.startsWith("set roll kp")) {
    rollPID.Kp = cmd.substring(12).toFloat();
  } else if (cmd.startsWith("set roll ki")) {
    rollPID.Ki = cmd.substring(12).toFloat();
  } else if (cmd.startsWith("set roll kd")) {
    rollPID.Kd = cmd.substring(12).toFloat();
  } else if (cmd == "save") {
    EEPROM.put(0, pitchPID);
    EEPROM.put(sizeof(pitchPID), rollPID);
    EEPROM.commit();
    sendLine("✅ Settings saved to EEPROM");
  }
}

void streamTelemetry() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 50) {
    lastSend = millis();

    String db = "DB:";
    db += String(pitch, 2) + " ";
    db += String(roll, 2) + " ";
    db += String(pitchCorrection, 2) + " ";
    db += String(rollCorrection, 2) + " ";
    db += String(command.throttle);

    sendLine(db);
  }
}

int ease(int current, int target)
{
    if (current < target)
        return min(current + easingRate, target);
    else
        return max(current - easingRate, target);
}

void setup()
{
    
    Serial.begin(115200);

    EEPROM.begin(512);  // Init EEPROM
    loadPIDFromEEPROM();

    // ESC
    escFL.attach();
    escFR.attach();
    escBL.attach();
    escBR.attach();
    escFL.arm();
    escFR.arm();
    escBL.arm();
    escBR.arm();

      // Set WiFi mode to allow both ESP-NOW and TCP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP started");

  // Start TCP server
  server.begin();
  Serial.println("TCP server started on port 8000");

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onReceive);
  Serial.println("✅ ESP-NOW ready");

    // MPU6050
    Wire.begin();
    mpu.initialize();
    mpu.setDLPFMode(3);  // Set to 3 (about 44Hz cutoff)
    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 failed to connect.");
        // while (1)
        //     ;
    }
    delay(1000); // stabilize

    // Prime filter
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    kalmanX.update(rollAcc, gx / 131.0, 0.01);
    kalmanY.update(pitchAcc, gy / 131.0, 0.01);
    lastUpdate = millis();

kalmanX.Q_angle     =0.005f;      // Trust gyro a bit more
kalmanX.Q_bias      =0.003f;       // Same bias estimation
kalmanX.R_measure   =0.1f;      // Less trust in noisy accelerometer

kalmanY.Q_angle     =0.005f;      // Trust gyro a bit more
kalmanY.Q_bias      =0.003f;       // Same bias estimation
kalmanY.R_measure   =0.1f;      // Less trust in noisy accelerometer

    Serial.println("System ready.");
}

void loop()
{
    
    // === MPU6050 angle ===
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    float gyroXrate = gx / 131.0;
    float gyroYrate = gy / 131.0;

    roll = kalmanX.update(rollAcc, gyroXrate, dt);
    pitch = kalmanY.update(pitchAcc, gyroYrate, dt);



// Then in your main loop:
    // pitch = pitchFilter.update(pitch);
    // roll  = rollFilter.update(roll);

    Serial.printf("Pitch: %.2f, Roll: %.2f\n", pitch, roll);

    float pitchError = -pitch; // target = 0
    float rollError = -roll;

    pitchErrorSum += pitchError * dt;
    rollErrorSum += rollError * dt;

    float pitchDeriv = (pitchError - lastPitchError) / dt;
    float rollDeriv = (rollError - lastRollError) / dt;

    pitchCorrection = pitchPID.Kp * pitchError + pitchPID.Ki * pitchErrorSum + pitchPID.Kd * pitchDeriv;
    rollCorrection = rollPID.Kp * rollError + rollPID.Ki * rollErrorSum + rollPID.Kd * rollDeriv;

    lastPitchError = pitchError;
    lastRollError = rollError;

    // === Thrust Mixer ===
int base = command.throttle;

int pitchBiasThrust = constrain(command.pitchBias, -100, 100);  
int rollBiasThrust = constrain(command.rollBias, -100, 100);

// Apply both PID corrections AND manual biases
int targetFL = constrain(base - constrain(pitchCorrection,-150,150) + constrain(rollCorrection,-150,150) 
                         - pitchBiasThrust + rollBiasThrust, 1000, 2000);
int targetFR = constrain(base - constrain(pitchCorrection,-150,150) - constrain(rollCorrection,-150,150) 
                         - pitchBiasThrust - rollBiasThrust, 1000, 2000);
int targetBL = constrain(base + constrain(pitchCorrection,-150,150) + constrain(rollCorrection,-150,150) 
                         + pitchBiasThrust + rollBiasThrust, 1000, 2000);
int targetBR = constrain(base + constrain(pitchCorrection,-150,150) - constrain(rollCorrection,-150,150) 
                         + pitchBiasThrust - rollBiasThrust, 1000, 2000);


        if (millis() - lastCommandTime > FAILSAFE_TIMEOUT)
    {
        command.throttle = 1000;
        command.pitchBias = 0;
        command.rollBias = 0;
    escFL.writeMicroseconds(1000);
    escFR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000);
    escBR.writeMicroseconds(1000);
        Serial.println("Failsafe activated, setting throttle to 1000.");
    }else{
    currentFL = ease(currentFL, targetFL);
    currentFR = ease(currentFR, targetFR);
    currentBL = ease(currentBL, targetBL);
    currentBR = ease(currentBR, targetBR);

    
    escFL.writeMicroseconds(currentFL);
    escFR.writeMicroseconds(currentFR);
    escBL.writeMicroseconds(currentBL);
    escBR.writeMicroseconds(currentBR);
    }
    Serial.printf("DB:%.2f %.2f %.2f %.2f %d\n", pitch, roll, pitchCorrection, rollCorrection, command.throttle);

if (!client || !client.connected()) {
    client = server.available();
  }

  // Non-blocking TCP read
  while (client && client.available()) {
    char c = client.read();
    if (c == '\n') {
      handleCommand(incomingCommand);
      incomingCommand = "";
    } else {
      incomingCommand += c;
    }
  }

  // Non-blocking Serial read
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(incomingCommand);
      incomingCommand = "";
    } else {
      incomingCommand += c;
    }
  }
    streamTelemetry();      // Optional
    delay(10);
}
