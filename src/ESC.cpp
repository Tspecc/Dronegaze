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
    uint32_t duty = map(pulse, 1000, 2000, 3276, 6553); // 16-bit PWM
    ledcWrite(_channel, duty);
}
