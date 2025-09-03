#pragma once
#include <Arduino.h>
#include <driver/mcpwm.h>

/*
 * ESC hardware PWM driver using MCPWM timers.
 * Replaces the previous LEDC implementation to guarantee a stable
 * 50 Hz PWM with 1–2 ms pulse width using dedicated motor-control
 * hardware timers. Sub‑1 ms pulses are allowed for safety routines
 * such as disarming before calibration.
 */

class ESC {
public:
    ESC(int pin, mcpwm_unit_t unit, mcpwm_timer_t timer,
        mcpwm_operator_t op, mcpwm_io_signals_t signal, uint32_t freq = 50);
    bool attach();
    void arm(int pulse = 1000);
    void writeMicroseconds(int pulse); // constrained to 900–2000 µs
    uint32_t frequency() const;

private:
    int _pin;
    mcpwm_unit_t _unit;
    mcpwm_timer_t _timer;
    mcpwm_operator_t _op;
    mcpwm_io_signals_t _signal;
    uint32_t _freq;
};

