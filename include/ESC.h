#pragma once
#include <Arduino.h>

/*
 * ESC driver using hardware LEDC PWM for stable 50 Hz output.
 * Each instance controls one LEDC channel and timer to generate
 * 1–2 ms pulses required by standard RC ESCs. Sub-1 ms pulses
 * are permitted for disarming and calibration.
 */
class ESC {
public:
    ESC(int pin, int channel, uint32_t freq = 50, uint8_t resolution = 16);
    bool attach();
    void writeMicroseconds(int pulse); // constrained to 900–2000 µs
private:
    int _pin;
    int _channel;
    uint32_t _freq;
    uint8_t _resolution;
    uint32_t _period_us;
};
