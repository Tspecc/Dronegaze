#include "ESC.h"

ESC::ESC(int pin, mcpwm_unit_t unit, mcpwm_timer_t timer,
         mcpwm_operator_t op, mcpwm_io_signals_t signal, uint32_t freq)
    : _pin(pin), _unit(unit), _timer(timer), _op(op), _signal(signal), _freq(freq) {}

bool ESC::attach() {
    mcpwm_gpio_init(_unit, _signal, _pin);
    mcpwm_config_t cfg{};
    cfg.frequency = _freq; // 50 Hz
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_unit, _timer, &cfg);
    return true;
}

void ESC::arm(int pulse) {
    writeMicroseconds(pulse);
}

static inline uint32_t clampu32(uint32_t v, uint32_t lo, uint32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void ESC::writeMicroseconds(int pulse) {
    pulse = clampu32(pulse, 900, 2000); // allow sub-1ms for disarming
    mcpwm_set_duty_in_us(_unit, _timer, _op, pulse);
}

uint32_t ESC::frequency() const { return _freq; }
