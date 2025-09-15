#include "motor.h"

namespace Motor {

// One ESC per MCPWM generator: 0A/0B/1A/1B.
static ESC escFL(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, 50);
static ESC escFR(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, 50);
static ESC escBL(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, 50);
static ESC escBR(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, 50);

void calibrate()
{

//    escFL.detach(); escFR.detach(); escBL.detach(); escBR.detach();
  //  delay(20); // allow pins to float
    //escFL.attach(); escFR.attach(); escBL.attach(); escBR.attach();

    // Hold motors in a disarmed state to ensure props are removed
    //escFL.writeMicroseconds(900); escFR.writeMicroseconds(900);
    //escBL.writeMicroseconds(900); escBR.writeMicroseconds(900);
    //delay(3000);


    // // standard ESC calibration: max then min throttle
    // escFL.writeMicroseconds(1000); escFR.writeMicroseconds(1000);
    // escBL.writeMicroseconds(1000); escBR.writeMicroseconds(1000);
    // delay(3000);
}

void Outputs::constrainAll() {
    MFL = constrain(MFL, 1000, 2000);
    MFR = constrain(MFR, 1000, 2000);
    MBL = constrain(MBL, 1000, 2000);
    MBR = constrain(MBR, 1000, 2000);
}

// Attach ESCs to their pins and start the MCPWM timers.
bool init(int pinFL, int pinFR, int pinBL, int pinBR) {
    escFL = ESC(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, pinFL);
    escFR = ESC(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_B, pinFR);
    escBL = ESC(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_A, pinBL);
    escBR = ESC(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_GEN_B, pinBR);

    return escFL.attach()
        && escFR.attach()
        && escBL.attach()
        && escBR.attach();
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

// Ramp each motor toward its target when armed.
// Otherwise, send a neutral pulse to keep ESCs disarmed.
void update(bool armed, Outputs &current, const Outputs &target) {
    if (armed) {
        current.MFL = ease(current.MFL, target.MFL);
        current.MFR = ease(current.MFR, target.MFR);
        current.MBL = ease(current.MBL, target.MBL);
        current.MBR = ease(current.MBR, target.MBR);
    } else {
        current = Outputs{1000, 1000, 1000, 1000};
    }

    escFL.writeMicroseconds(current.MFL);
    escFR.writeMicroseconds(current.MFR);
    escBL.writeMicroseconds(current.MBL);
    escBR.writeMicroseconds(current.MBR);
}

}

