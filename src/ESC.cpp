#include "ESC.h"

ESC::ESC(int pin, int channel, uint32_t freq, int resolution)
    : _pin(pin), _channel(channel), _freq(freq), _resolution(resolution) {}

bool ESC::attach() {
    // Capture the real frequency returned by LEDC setup to account for
    // variations caused by CPU clock changes (e.g., on ESP32-C3).
    if(ledcSetup(_channel, _freq, _resolution))
    {}
    {
        return 0;
    }
    ledcAttachPin(_pin, _channel);
    return 1;
}

void ESC::arm(int pulse) {
    writeMicroseconds(pulse);
}

static inline uint32_t clampu32(uint32_t v, uint32_t lo, uint32_t hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void ESC::writeMicroseconds(int pulse) {
  // Constrain the input value to the standard servo range
  pulse = clampu32(pulse, 500, 2500);
  const uint32_t maxDuty = (1UL << _resolution) - 1UL;

  // Calculate the duty cycle value based on the chosen resolution
  // (2^resolution / (1/frequency*1000000)) * microseconds
   uint32_t duty = (pulse * maxDuty + (PWM_PERIOD_US / 2)) / PWM_PERIOD_US;
  
  // Write the new duty cycle to the LEDC channel
  ledcWrite(_channel, duty);
}

uint32_t ESC::frequency() const { return _freq; }
