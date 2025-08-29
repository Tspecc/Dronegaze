#pragma once
#include <Arduino.h>

class PIDController {
public:
    PIDController(float p = 2.0, float i = 0.0, float d = 0.5);
    float compute(float error, float dt);
    void reset();

    float Kp, Ki, Kd;
private:
    float errorSum;
    float lastError;
    bool initialized;
};

struct PIDOutputs {
    float roll;
    float pitch;
    float yaw;
    float altitude;
};

void updatePIDControllers(float pitchSetpoint,
                          float rollSetpoint,
                          float yawSetpoint,
                          float altitudeSetpoint,
                          float pitch,
                          float roll,
                          float yaw,
                          float altitude,
                          float verticalAcc,
                          PIDController &pitchPID,
                          PIDController &rollPID,
                          PIDController &yawPID,
                          PIDController &altitudePID,
                          PIDController &verticalAccelPID,
                          PIDOutputs &out);
