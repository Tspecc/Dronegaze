#include "motor.h"

namespace Motor {

// Use hardware LEDC channels for stable 50 Hz PWM pulses.
static ESC escFL(0, 0);
static ESC escFR(0, 1);
static ESC escBL(0, 2);
static ESC escBR(0, 3);

void calibrate()
{
    // Hold motors in a disarmed state to ensure props are removed
    escFL.writeMicroseconds(900); escFR.writeMicroseconds(900);
    escBL.writeMicroseconds(900); escBR.writeMicroseconds(900);
    delay(3000);

    // Standard ESC calibration: max then min throttle
    escFL.writeMicroseconds(2000); escFR.writeMicroseconds(2000);
    escBL.writeMicroseconds(2000); escBR.writeMicroseconds(2000);
    delay(1000);
    escFL.writeMicroseconds(1000); escFR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000); escBR.writeMicroseconds(1000);
    delay(1000);

    // Return to a safe disarmed level
    escFL.writeMicroseconds(900); escFR.writeMicroseconds(900);
    escBL.writeMicroseconds(900); escBR.writeMicroseconds(900);
    delay(1000);
}

void Outputs::constrainAll() {
    MFL = constrain(MFL, 1000, 2000);
    MFR = constrain(MFR, 1000, 2000);
    MBL = constrain(MBL, 1000, 2000);
    MBR = constrain(MBR, 1000, 2000);
}

bool init(int pinFL, int pinFR, int pinBL, int pinBR, int pwmRes) {
    escFL = ESC(pinFL, 0, 50, pwmRes);
    escFR = ESC(pinFR, 1, 50, pwmRes);
    escBL = ESC(pinBL, 2, 50, pwmRes);
    escBR = ESC(pinBR, 3, 50, pwmRes);
    bool ok = escFL.attach();
    ok = escFR.attach() && ok;
    ok = escBL.attach() && ok;
    ok = escBR.attach() && ok;
    return ok;
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
        escFL.writeMicroseconds(900);
        escFR.writeMicroseconds(900);
        escBL.writeMicroseconds(900);
        escBR.writeMicroseconds(900);
    }
}
}

