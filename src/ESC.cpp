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

void ESC::writeMicroseconds(int pulse) {
  // Constrain the input value to the standard servo range
  pulse = constrain(pulse, 500, 2500);
  
  // Calculate the duty cycle value based on the chosen resolution
  // (2^resolution / (1/frequency*1000000)) * microseconds
  int dutyCycle = (long)(pow(2, _resolution) * pulse) / 20000;
  
  // Write the new duty cycle to the LEDC channel
  ledcWrite(_channel, dutyCycle);
}

uint32_t ESC::frequency() const { return _freq; }
