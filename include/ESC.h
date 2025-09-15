#pragma once
#include <Arduino.h>
#include <driver/mcpwm.h>

/*
 * ESC driver using the ESP32 MCPWM hardware timers for stable 50 Hz output.
 * Each instance controls one MCPWM generator to produce 1–2 ms pulses
 * required by standard RC ESCs. Sub-1 ms pulses are permitted for
 * disarming and calibration.
 */
class ESC {
public:
    ESC(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t gen,
        int pin, uint32_t freq = 50);
    bool attach();
    void detach();
    void writeMicroseconds(int pulse); // constrained to 900–2000 µs
    void writeMicrosecondsUnconstrained(int pulse);

private:
    mcpwm_unit_t _unit;
    mcpwm_timer_t _timer;
    mcpwm_generator_t _gen;
    int _pin;
    uint32_t _freq;
};
