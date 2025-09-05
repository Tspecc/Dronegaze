#include "ESC.h"

ESC::ESC(int pin, uint32_t freq)
    : _pin(pin), _freq(freq), _periodic(nullptr), _offTimer(nullptr), _pulse(1000) {}

bool ESC::attach() {
    pinMode(_pin, OUTPUT);

    esp_timer_create_args_t periodArgs = {
        .callback = &ESC::onPeriod,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "esc_period"};
    if (esp_timer_create(&periodArgs, &_periodic) != ESP_OK) return false;

    esp_timer_create_args_t offArgs = {
        .callback = &ESC::onOff,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "esc_off"};
    if (esp_timer_create(&offArgs, &_offTimer) != ESP_OK) return false;

    return esp_timer_start_periodic(_periodic, 1000000 / _freq) == ESP_OK;
}

void ESC::arm(int pulse) { writeMicroseconds(pulse); }

static inline uint32_t clampu32(uint32_t v, uint32_t lo, uint32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void ESC::writeMicroseconds(int pulse) {
    _pulse = clampu32(pulse, 0, 2000); // allow sub-1ms for disarming
}

uint32_t ESC::frequency() const { return _freq; }

void ESC::onPeriod(void *arg) {
    auto *self = static_cast<ESC *>(arg);
    digitalWrite(self->_pin, HIGH);
    esp_timer_stop(self->_offTimer);
    esp_timer_start_once(self->_offTimer, self->_pulse);
}

void ESC::onOff(void *arg) {
    auto *self = static_cast<ESC *>(arg);
    digitalWrite(self->_pin, LOW);
}

