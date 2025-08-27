#include "ESC.h"

ESC::ESC(int pin, int channel, int freq, int resolution)
    : _pin(pin), _channel(channel), _freq(freq), _resolution(resolution) {}

void ESC::attach() {
    ledcSetup(_channel, _freq, _resolution);
    ledcAttachPin(_pin, _channel);
}

void ESC::arm(int pulse) {
    writeMicroseconds(pulse);
}

void ESC::writeMicroseconds(int pulse) {
    pulse = constrain(pulse, 1000, 2000);
    uint32_t maxDuty = (1 << _resolution) - 1;
    uint32_t duty = (uint64_t)maxDuty * pulse * _freq / 1000000; // scale to period
    ledcWrite(_channel, duty);
}
