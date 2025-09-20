#pragma once
#include <Arduino.h>

struct ControlOutputs {
    float roll;
    float pitch;
    float yaw;
    float vertical;
};

namespace Control {

enum class Axis : uint8_t {
    Roll = 0,
    Pitch = 1,
    Yaw = 2,
    Vertical = 3,
};

struct Gains {
    float kp;
    float ki;
    float kd;
};

void init();

void computeCorrections(float pitchSetpoint,
                        float rollSetpoint,
                        float yawSetpoint,
                        float pitch,
                        float roll,
                        float yaw,
                        float gyroX,
                        float gyroY,
                        float gyroZ,
                        float verticalAcc,
                        bool throttleStable,
                        bool yawEnabled,
                        ControlOutputs &out);

bool pidEnabled();
void setPidEnabled(bool enabled);

bool filtersEnabled();
void setFiltersEnabled(bool enabled);

bool quadFiltersEnabled();
void setQuadFiltersEnabled(bool enabled);

Gains getGains(Axis axis);
void setGains(Axis axis, const Gains &gains);
void resetGains();

float filterAlpha();
void setFilterAlpha(float alpha);

float quadFilterAlpha();
void setQuadFilterAlpha(float alpha);

} // namespace Control
