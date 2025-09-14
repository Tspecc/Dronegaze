#include "ESC.h"

ESC::ESC(int pin, int channel, uint32_t freq, uint8_t resolution)
    : _pin(pin), _channel(channel), _freq(freq), _resolution(resolution) {
    _period_us = 1000000UL / _freq;
}

bool ESC::attach() {
    // Configure the LEDC channel and attach the pin. Return false on failure.
    if (ledcSetup(_channel, _freq, _resolution) == 0) {
        return false;
    }
    ledcAttachPin(_pin, _channel);
    writeMicroseconds(900); // ensure disarmed on start
    return true;
}

void ESC::writeMicroseconds(int pulse) {
    pulse = constrain(pulse, 900, 2000);
    uint32_t duty_max = (1UL << _resolution) - 1;
    uint32_t duty = (static_cast<uint32_t>(pulse) * duty_max) / _period_us;
    ledcWrite(_channel, duty);
}
