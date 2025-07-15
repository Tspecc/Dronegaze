#pragma once
#include <Arduino.h>

class ESC {
public:
    ESC(int pin, int channel, int freq = 50, int resolution = 16);
    void attach();
    void arm(int pulse = 1000);  // Default arm pulse
    void writeMicroseconds(int pulse); // 1000–2000 µs
private:
    int _pin;
    int _channel;
    int _freq;
    int _resolution;
};