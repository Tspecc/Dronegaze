#include "pid.h"

PIDController::PIDController(float p, float i, float d)
    : Kp(p), Ki(i), Kd(d), errorSum(0), lastError(0), initialized(false) {}

float PIDController::compute(float error, float dt) {
    if (dt <= 0) return 0;
    if (!initialized) {
        lastError = error;
        initialized = true;
    }
    errorSum += error * dt;
    errorSum = constrain(errorSum, -100, 100);
    float derivative = (error - lastError) / dt;
    lastError = error;
    return Kp * error + Ki * errorSum + Kd * derivative;
}

void PIDController::reset() {
    errorSum = 0;
    lastError = 0;
    initialized = false;
}

void updatePIDControllers(float pitchSetpoint,
                          float rollSetpoint,
                          float yawSetpoint,
                          float pitch,
                          float roll,
                          float yaw,
                          float verticalAcc,
                          bool yawEnabled,
                          PIDController &pitchPID,
                          PIDController &rollPID,
                          PIDController &yawPID,
                          PIDController &verticalAccelPID,
                          PIDOutputs &out) {
    float pitchError = pitchSetpoint - pitch;
    float rollError = rollSetpoint - roll;
    out.roll = rollPID.compute(rollError, 0.01f);
    out.pitch = pitchPID.compute(pitchError, 0.01f);
    if (yawEnabled) {
        float yawError = yawSetpoint - yaw;
        if (yawError > 180) yawError -= 360;
        else if (yawError < -180) yawError += 360;
        out.yaw = yawPID.compute(yawError, 0.01f);
    } else {
        yawPID.reset();
        out.yaw = 0;
    }
    out.vertical = verticalAccelPID.compute(verticalAcc, 0.01f);
}

