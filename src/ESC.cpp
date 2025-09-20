#include "ESC.h"

ESC::ESC(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_generator_t gen,
         int pin, uint32_t freq)
    : _unit(unit), _timer(timer), _gen(gen), _pin(pin), _freq(freq) {}

bool ESC::attach() {
    // Select the appropriate MCPWM output signal for the timer/generator pair
    mcpwm_io_signals_t signal = static_cast<mcpwm_io_signals_t>(
        MCPWM0A + static_cast<int>(_timer) * 2 + (_gen == MCPWM_GEN_B));
    mcpwm_gpio_init(_unit, signal, _pin);

    mcpwm_config_t cfg;
    cfg.frequency = _freq;
    cfg.cmpr_a = 0; // duty cycle of generator A
    cfg.cmpr_b = 0; // duty cycle of generator B
    cfg.counter_mode = MCPWM_UP_COUNTER;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(_unit, _timer, &cfg);

    writeMicroseconds(0); // ensure disarmed on start
    return true;
}

void ESC::writeMicroseconds(int pulse) {
    pulse = constrain(pulse, 1000, 2000);
    mcpwm_set_duty_in_us(_unit, _timer, _gen, pulse);
}

void ESC::writeMicrosecondsUnconstrained(int pulse) {
    mcpwm_set_duty_in_us(_unit, _timer, _gen, pulse);
}