#pragma once
#include <Arduino.h>
static const uint32_t PWM_PERIOD_US = 20000UL; // 1 / 50 Hz = 20 ms
class ESC {
public:
    ESC(int pin, int channel, uint32_t freq = 50, int resolution = 16);
    bool attach();
    void arm(int pulse = 1000);  // Default arm pulse
    void writeMicroseconds(int pulse); // 1000–2000 µs
    // Expose the real PWM frequency so callers can verify timing
    uint32_t frequency() const;
private:
    int _pin;
    int _channel;
    uint32_t _freq; // store actual PWM frequency returned by ledcSetup
    int _resolution;
};
