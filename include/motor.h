#pragma once
#include <Arduino.h>
#include "ESC.h"

namespace Motor {
struct Outputs {
    int MFL, MFR, MBL, MBR;
    void constrainAll();
};

bool init(int pinFL, int pinFR, int pinBL, int pinBR, int pwmRes);
void mix(int base, int pitchCorr, int rollCorr, int yawCorr, Outputs &target);
void update(bool isArmed, Outputs &current, const Outputs &target);
void calibrate();
}

