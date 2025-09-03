#pragma once
#include <Arduino.h>
#include <esp_timer.h>

/*
 * ESC driver using high‑resolution timer interrupts.
 * Generates a 50 Hz PWM signal with 1–2 ms pulse width by
 * toggling the output pin in esp_timer callbacks. Sub‑1 ms
 * pulses are allowed for disarming and calibration routines.
 */

class ESC {
public:
    explicit ESC(int pin, uint32_t freq = 50);
    bool attach();
    void arm(int pulse = 1000);
    void writeMicroseconds(int pulse); // constrained to 900–2000 µs
    uint32_t frequency() const;

private:
    static void onPeriod(void *arg);
    static void onOff(void *arg);

    int _pin;
    uint32_t _freq;
    esp_timer_handle_t _periodic;
    esp_timer_handle_t _offTimer;
    volatile uint32_t _pulse;
};

