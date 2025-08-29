#include "imu.h"
#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"
#include <bandpass.h>
#include "QuadFilter.h"

namespace IMU {
static MPU6050 mpu;
static KalmanFilter kalmanX, kalmanY, kalmanZ;
static BandPass pitchFilter(0.1,0.9), rollFilter(0.1,0.9), yawFilter(0.1,0.9);
static CascadedFilter pitchQuad(1,200.0,20.0,0.707,FilterType::LOW_PASS);
static CascadedFilter rollQuad(1,200.0,20.0,0.707,FilterType::LOW_PASS);
static CascadedFilter yawQuad(1,200.0,20.0,0.707,FilterType::LOW_PASS);

static float g_pitch=0, g_roll=0, g_yaw=0;
static float g_altitude=0, g_verticalAcc=0, g_verticalVel=0;
static float pitchOffset=0, rollOffset=0, yawOffset=0;
static unsigned long lastUpdate=0;

void setupKalman() {
    kalmanX.Q_angle = 0.01f; kalmanX.Q_bias = 0.006f; kalmanX.R_measure = 0.1f;
    kalmanY.Q_angle = 0.01f; kalmanY.Q_bias = 0.006f; kalmanY.R_measure = 0.05f;
    kalmanZ.Q_angle = 0.01f; kalmanZ.Q_bias = 0.006f; kalmanZ.R_measure = 0.1f;
}

void init() {
    Wire.begin();
    mpu.initialize();
    setupKalman();
    zero();
}

void update() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    unsigned long now = millis();
    float dt = (now - lastUpdate)/1000.0f;
    if (dt <= 0 || dt > 0.1f) { lastUpdate = now; return; }
    lastUpdate = now;
    float rollAcc = atan2(ay, az) * RAD_TO_DEG;
    float pitchAcc = atan2(-ax, sqrt(ay*ay+az*az))*RAD_TO_DEG;
    float gyroXrate = gx/131.0f;
    float gyroYrate = gy/131.0f;
    float gyroZrate = gz/131.0f;
    float fr = rollFilter.update(kalmanX.update(rollAcc, gyroXrate, dt));
    float fp = pitchFilter.update(kalmanY.update(pitchAcc, gyroYrate, dt));
    float fy = yawFilter.update(kalmanZ.update(g_yaw, gyroZrate, dt));
    g_roll = rollQuad.process(fr) - rollOffset;
    g_pitch = pitchQuad.process(fp) - pitchOffset;
    g_yaw = yawQuad.process(fy) - yawOffset;
    if (g_yaw > 180) g_yaw -= 360; else if (g_yaw < -180) g_yaw += 360;
    const float ACC_SCALE=16384.0f;
    float ax_ms2 = (float)ax/ACC_SCALE*9.81f;
    float ay_ms2 = (float)ay/ACC_SCALE*9.81f;
    float az_ms2 = (float)az/ACC_SCALE*9.81f;
    float pitchRad = g_pitch*DEG_TO_RAD;
    float rollRad = g_roll*DEG_TO_RAD;
    float worldZ = cos(pitchRad)*cos(rollRad)*az_ms2+
                   sin(pitchRad)*cos(rollRad)*ax_ms2+
                   cos(pitchRad)*sin(rollRad)*ay_ms2;
    g_verticalAcc = worldZ - 9.81f;
    g_verticalVel += g_verticalAcc * dt;
    g_altitude += g_verticalVel * dt;
}

void zero() {
    update();
    rollOffset = g_roll;
    pitchOffset = g_pitch;
    yawOffset = g_yaw;
    g_altitude = 0; g_verticalVel = 0;
}

float pitch() { return g_pitch; }
float roll() { return g_roll; }
float yaw() { return g_yaw; }
float altitude() { return g_altitude; }
float verticalAcc() { return g_verticalAcc; }
}

