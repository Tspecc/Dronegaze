#include "motor.h"

namespace Motor {

// Use timer-based PWM outputs for each motor at 50 Hz with 1–2 ms pulses.
static ESC escFL(0);
static ESC escFR(0);
static ESC escBL(0);
static ESC escBR(0);

void calibrate()
{
    // hold motors disarmed (<1 ms) for 3 s to ensure props are removed
    escFL.writeMicroseconds(900); escFR.writeMicroseconds(900);
    escBL.writeMicroseconds(900); escBR.writeMicroseconds(900);
    delay(3000);

    // standard ESC calibration: max then min throttle
    escFL.writeMicroseconds(2000); escFR.writeMicroseconds(2000);
    escBL.writeMicroseconds(2000); escBR.writeMicroseconds(2000);
    delay(2000);
    escFL.writeMicroseconds(1000); escFR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000); escBR.writeMicroseconds(1000);
    delay(2000);
}

void Outputs::constrainAll() {
    MFL = constrain(MFL, 1000, 2000);
    MFR = constrain(MFR, 1000, 2000);
    MBL = constrain(MBL, 1000, 2000);
    MBR = constrain(MBR, 1000, 2000);
}

void init(int pinFL, int pinFR, int pinBL, int pinBR, int pwmRes) {
    escFL = ESC(pinFL);
    escFR = ESC(pinFR);
    escBL = ESC(pinBL);
    escBR = ESC(pinBR);
    escFL.attach(); escFR.attach(); escBL.attach(); escBR.attach();
}

int ease(int current, int target) {
    const int EASING_RATE = 500;
    if (current < target) return min(current + EASING_RATE, target);
    else return max(current - EASING_RATE, target);
}

void mix(int base, int pitchCorr, int rollCorr, int yawCorr, Outputs &target) {
    target.MFL = base - pitchCorr + rollCorr - yawCorr;
    target.MFR = base - pitchCorr - rollCorr + yawCorr;
    target.MBL = base + pitchCorr + rollCorr - yawCorr;
    target.MBR = base + pitchCorr - rollCorr + yawCorr;
    target.constrainAll();
}

void update(bool isArmed, Outputs &current, const Outputs &target) {
    if (isArmed) {
        current.MFL = ease(current.MFL, target.MFL);
        current.MFR = ease(current.MFR, target.MFR);
        current.MBL = ease(current.MBL, target.MBL);
        current.MBR = ease(current.MBR, target.MBR);
        escFL.writeMicroseconds(current.MFL);
        escFR.writeMicroseconds(current.MFR);
        escBL.writeMicroseconds(current.MBL);
        escBR.writeMicroseconds(current.MBR);
    } else {
        escFL.writeMicroseconds(1000);
        escFR.writeMicroseconds(1000);
        escBL.writeMicroseconds(1000);
        escBR.writeMicroseconds(1000);
    }
}
}

