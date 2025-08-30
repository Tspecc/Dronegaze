#include "motor.h"

namespace Motor {

// On the ESP32‑C3, LEDC channels are paired by timer: channels 0/1 use timer0
// and 2/3 use timer1. Channels 4/5 use timer2. By keeping the motors on
// channels 0‑3 we occupy timers 0 and 1 only, leaving timer2 free for the
// buzzer on channel 5 so their frequencies cannot conflict.
static ESC escFL(0,0,50,16), escFR(0,1,50,16), escBL(0,2,50,16), escBR(0,3,50,16);

static int pwmResolution = 16;

void Outputs::constrainAll() {
    MFL = constrain(MFL, 1000, 2000);
    MFR = constrain(MFR, 1000, 2000);
    MBL = constrain(MBL, 1000, 2000);
    MBR = constrain(MBR, 1000, 2000);
}

void init(int pinFL, int pinFR, int pinBL, int pinBR, int pwmRes) {

    // Assign channels sequentially; this maps the motors to timers 0 and 1.
    // Timer2 remains unused and is dedicated to the buzzer (channel 5).

    escFL = ESC(pinFL,0,50,pwmRes);
    escFR = ESC(pinFR,2,50,pwmRes);
    escBL = ESC(pinBL,3,50,pwmRes);
    escBR = ESC(pinBR,4,50,pwmRes);
    pwmResolution = pwmRes;
    escFL.attach(); escFR.attach(); escBL.attach(); escBR.attach();
    // Verify all ESC channels operate at the expected 50 Hz. Any drift is logged
    if (escFL.frequency()!=50 || escFR.frequency()!=50 ||
        escBL.frequency()!=50 || escBR.frequency()!=50) {
        Serial.println("WARNING: PWM frequency mismatch detected");
    }
}

int ease(int current, int target) {
    const int EASING_RATE = 2000;
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

