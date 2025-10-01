#include "control.h"
#include "pid.h"
#include <EEPROM.h>
#include <array>
#include <math.h>

namespace {

constexpr uint32_t CONFIG_MAGIC = 0x434E544C; // "CNTL"
constexpr size_t kAxisCount = Control::kAxisCount;
constexpr float DEFAULT_FILTER_ALPHA = 0.25f;
constexpr float DEFAULT_QUAD_ALPHA = 0.1f;
constexpr size_t STORAGE_SIZE = 128;

struct RuntimeConfig {
    Control::Gains roll;
    Control::Gains pitch;
    Control::Gains yaw;
    Control::Gains vertical;
    float filterAlpha;
    float quadFilterAlpha;
    bool pidEnabled;
    bool filtersEnabled;
    bool quadFiltersEnabled;
};

struct ConfigBlob {
    uint32_t magic;
    Control::Gains roll;
    Control::Gains pitch;
    Control::Gains yaw;
    Control::Gains vertical;
    float filterAlpha;
    float quadFilterAlpha;
    uint8_t flags;
    uint8_t reserved[7];
};

struct LowPassFilter {
    float state;
    bool initialized;

    LowPassFilter() : state(0.0f), initialized(false) {}

    float process(float value, float alpha) {
        alpha = constrain(alpha, 0.0f, 1.0f);
        if (!initialized) {
            state = value;
            initialized = true;
        } else {
            state += alpha * (value - state);
        }
        return state;
    }

    void reset() {
        state = 0.0f;
        initialized = false;
    }
};

const Control::Gains kDefaultGains[kAxisCount] = {
    {4.0f, 0.0f, 0.1f},   // Roll
    {4.0f, 0.0f, 0.1f},   // Pitch
    {4.0f, 0.0f, 0.1f},   // Yaw
    {20.0f, 0.0f, 5.0f},  // Vertical acceleration
};

RuntimeConfig g_config{
    kDefaultGains[0],
    kDefaultGains[1],
    kDefaultGains[2],
    kDefaultGains[3],
    DEFAULT_FILTER_ALPHA,
    DEFAULT_QUAD_ALPHA,
    false,
    false,
    false,
};

PIDController g_rollPID;
PIDController g_pitchPID;
PIDController g_yawPID;
PIDController g_verticalPID;

std::array<LowPassFilter, kAxisCount> g_stage1Filters{};
std::array<LowPassFilter, kAxisCount> g_stage2Filters{};
std::array<bool, Control::AXIS_COUNT> g_axisEnabled{{true, true, true, true}};

bool g_storageReady = false;
bool g_initialized = false;
uint32_t g_lastMicros = 0;

size_t axisIndex(Control::Axis axis) {
    return static_cast<size_t>(axis);
}

const Control::Gains &defaultGains(Control::Axis axis) {
    return kDefaultGains[axisIndex(axis)];
}

float sanitizeAlpha(float value, float fallback) {
    if (isnan(value) || isinf(value) || value < 0.0f || value > 1.0f) {
        return fallback;
    }
    return value;
}

void applyGainsToControllers() {
    g_rollPID.Kp = g_config.roll.kp;
    g_rollPID.Ki = g_config.roll.ki;
    g_rollPID.Kd = g_config.roll.kd;

    g_pitchPID.Kp = g_config.pitch.kp;
    g_pitchPID.Ki = g_config.pitch.ki;
    g_pitchPID.Kd = g_config.pitch.kd;

    g_yawPID.Kp = g_config.yaw.kp;
    g_yawPID.Ki = g_config.yaw.ki;
    g_yawPID.Kd = g_config.yaw.kd;

    g_verticalPID.Kp = g_config.vertical.kp;
    g_verticalPID.Ki = g_config.vertical.ki;
    g_verticalPID.Kd = g_config.vertical.kd;
}

void resetControllers() {
    g_rollPID.reset();
    g_pitchPID.reset();
    g_yawPID.reset();
    g_verticalPID.reset();
    g_lastMicros = micros();
}

void resetFilters() {
    for (auto &filter : g_stage1Filters) {
        filter.reset();
    }
    for (auto &filter : g_stage2Filters) {
        filter.reset();
    }
}

void resetAxisFilters(Control::Axis axis) {
    const size_t idx = axisIndex(axis);
    g_stage1Filters[idx].reset();
    g_stage2Filters[idx].reset();
}

void resetAxisController(Control::Axis axis) {
    switch (axis) {
        case Control::Axis::Roll:
            g_rollPID.reset();
            break;
        case Control::Axis::Pitch:
            g_pitchPID.reset();
            break;
        case Control::Axis::Yaw:
            g_yawPID.reset();
            break;
        case Control::Axis::Vertical:
            g_verticalPID.reset();
            break;
    }
}

ConfigBlob toBlob() {
    ConfigBlob blob{};
    blob.magic = CONFIG_MAGIC;
    blob.roll = g_config.roll;
    blob.pitch = g_config.pitch;
    blob.yaw = g_config.yaw;
    blob.vertical = g_config.vertical;
    blob.filterAlpha = g_config.filterAlpha;
    blob.quadFilterAlpha = g_config.quadFilterAlpha;
    blob.flags = 0;
    if (g_config.pidEnabled) blob.flags |= 0x01;
    if (g_config.filtersEnabled) blob.flags |= 0x02;
    if (g_config.quadFiltersEnabled) blob.flags |= 0x04;
    return blob;
}

void fromBlob(const ConfigBlob &blob) {
    g_config.roll = blob.roll;
    g_config.pitch = blob.pitch;
    g_config.yaw = blob.yaw;
    g_config.vertical = blob.vertical;
    g_config.filterAlpha = sanitizeAlpha(blob.filterAlpha, DEFAULT_FILTER_ALPHA);
    g_config.quadFilterAlpha = sanitizeAlpha(blob.quadFilterAlpha, DEFAULT_QUAD_ALPHA);
    g_config.pidEnabled = (blob.flags & 0x01) != 0;
    g_config.filtersEnabled = (blob.flags & 0x02) != 0;
    g_config.quadFiltersEnabled = (blob.flags & 0x04) != 0;
}

void persistConfig() {
    if (!g_storageReady) {
        return;
    }
    ConfigBlob blob = toBlob();
    EEPROM.put(0, blob);
    EEPROM.commit();
}

float computeDt() {
    const uint32_t now = micros();
    float dt = 0.0f;
    if (g_lastMicros != 0) {
        const uint32_t delta = now - g_lastMicros;
        dt = static_cast<float>(delta) / 1e6f;
    }
    g_lastMicros = now;
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.01f;
    }
    return dt;
}

float applyFilters(Control::Axis axis, float value) {
    const size_t idx = axisIndex(axis);
    if (g_config.filtersEnabled) {
        value = g_stage1Filters[idx].process(value, sanitizeAlpha(g_config.filterAlpha, DEFAULT_FILTER_ALPHA));
    } else {
        g_stage1Filters[idx].reset();
    }

    if (g_config.quadFiltersEnabled) {
        value = g_stage2Filters[idx].process(value, sanitizeAlpha(g_config.quadFilterAlpha, DEFAULT_QUAD_ALPHA));
    } else {
        g_stage2Filters[idx].reset();
    }

    return value;
}

} // namespace

namespace Control {

void init() {
    if (g_initialized) {
        return;
    }
    g_initialized = true;

    if (EEPROM.begin(STORAGE_SIZE)) {
        g_storageReady = true;
        ConfigBlob blob{};
        EEPROM.get(0, blob);
        if (blob.magic == CONFIG_MAGIC) {
            fromBlob(blob);
        } else {
            persistConfig();
        }
    } else {
        g_storageReady = false;
        Serial.println("WARN: Control configuration storage unavailable");
    }

    applyGainsToControllers();
    resetControllers();
    resetFilters();
}

void computeCorrections(float pitchSetpoint,
                        float rollSetpoint,
                        float yawSetpoint,
                        float pitch,
                        float roll,
                        float yaw,
                        float gyroX,
                        float gyroY,
                        float gyroZ,
                        float verticalAcc,
                        bool throttleStable,
                        bool yawEnabled,
                        ControlOutputs &out) {
    out.roll = out.pitch = out.yaw = out.vertical = 0.0f;

    const bool rollAxisEnabled = g_axisEnabled[axisIndex(Axis::Roll)];
    const bool pitchAxisEnabled = g_axisEnabled[axisIndex(Axis::Pitch)];
    const bool yawAxisEnabled = g_axisEnabled[axisIndex(Axis::Yaw)];
    const bool verticalAxisEnabled = g_axisEnabled[axisIndex(Axis::Vertical)];

    const float rollError = rollSetpoint - roll;
    const float pitchError = pitchSetpoint - pitch;
    float yawError = yawSetpoint - yaw;
    if (yawError > 180.0f) yawError -= 360.0f;
    else if (yawError < -180.0f) yawError += 360.0f;

    float filteredRollError = 0.0f;
    if (rollAxisEnabled) {
        filteredRollError = applyFilters(Axis::Roll, rollError);
    } else {
        resetAxisFilters(Axis::Roll);
        resetAxisController(Axis::Roll);
    }

    float filteredPitchError = 0.0f;
    if (pitchAxisEnabled) {
        filteredPitchError = applyFilters(Axis::Pitch, pitchError);
    } else {
        resetAxisFilters(Axis::Pitch);
        resetAxisController(Axis::Pitch);
    }

    const bool yawActive = yawAxisEnabled && yawEnabled;
    float filteredYawError = 0.0f;
    if (yawActive) {
        filteredYawError = applyFilters(Axis::Yaw, yawError);
    } else {
        resetAxisFilters(Axis::Yaw);
        resetAxisController(Axis::Yaw);
    }

    const bool verticalActive = verticalAxisEnabled && throttleStable;
    float filteredVerticalError = 0.0f;
    if (verticalActive) {
        const float verticalError = -verticalAcc;
        filteredVerticalError = applyFilters(Axis::Vertical, verticalError);
    } else {
        resetAxisFilters(Axis::Vertical);
        resetAxisController(Axis::Vertical);
    }

    if (g_config.pidEnabled) {
        const float dt = computeDt();
        if (rollAxisEnabled) {
            out.roll = g_rollPID.compute(filteredRollError, dt);
        } else {
            g_rollPID.reset();
            out.roll = 0.0f;
        }

        if (pitchAxisEnabled) {
            out.pitch = g_pitchPID.compute(filteredPitchError, dt);
        } else {
            g_pitchPID.reset();
            out.pitch = 0.0f;
        }

        if (yawActive) {
            out.yaw = g_yawPID.compute(filteredYawError, dt);
        } else {
            g_yawPID.reset();
            out.yaw = 0.0f;
        }

        if (verticalActive) {
            out.vertical = g_verticalPID.compute(filteredVerticalError, dt);
        } else {
            g_verticalPID.reset();
            out.vertical = 0.0f;
        }
    } else {
        if (rollAxisEnabled) {
            out.roll = g_config.roll.kp * filteredRollError - g_config.roll.kd * gyroX;
        } else {
            out.roll = 0.0f;
        }

        if (pitchAxisEnabled) {
            out.pitch = g_config.pitch.kp * filteredPitchError - g_config.pitch.kd * gyroY;
        } else {
            out.pitch = 0.0f;
        }

        if (yawActive) {
            out.yaw = g_config.yaw.kp * filteredYawError - g_config.yaw.kd * gyroZ;
        } else {
            out.yaw = 0.0f;
        }

        if (verticalActive) {
            out.vertical = g_config.vertical.kp * filteredVerticalError;
        } else {
            out.vertical = 0.0f;
        }
    }
}

bool pidEnabled() {
    return g_config.pidEnabled;
}

void setPidEnabled(bool enabled) {
    if (g_config.pidEnabled == enabled) {
        return;
    }
    g_config.pidEnabled = enabled;
    resetControllers();
    persistConfig();
}

bool filtersEnabled() {
    return g_config.filtersEnabled;
}

void setFiltersEnabled(bool enabled) {
    if (g_config.filtersEnabled == enabled) {
        return;
    }
    g_config.filtersEnabled = enabled;
    resetFilters();
    persistConfig();
}

bool quadFiltersEnabled() {
    return g_config.quadFiltersEnabled;
}

void setQuadFiltersEnabled(bool enabled) {
    if (g_config.quadFiltersEnabled == enabled) {
        return;
    }
    g_config.quadFiltersEnabled = enabled;
    resetFilters();
    persistConfig();
}

Gains getGains(Axis axis) {
    switch (axis) {
        case Axis::Roll: return g_config.roll;
        case Axis::Pitch: return g_config.pitch;
        case Axis::Yaw: return g_config.yaw;
        case Axis::Vertical: return g_config.vertical;
    }
    return g_config.roll;
}

void setGains(Axis axis, const Gains &gains) {
    switch (axis) {
        case Axis::Roll: g_config.roll = gains; break;
        case Axis::Pitch: g_config.pitch = gains; break;
        case Axis::Yaw: g_config.yaw = gains; break;
        case Axis::Vertical: g_config.vertical = gains; break;
    }
    applyGainsToControllers();
    persistConfig();
}

void resetGains() {
    g_config.roll = defaultGains(Axis::Roll);
    g_config.pitch = defaultGains(Axis::Pitch);
    g_config.yaw = defaultGains(Axis::Yaw);
    g_config.vertical = defaultGains(Axis::Vertical);
    applyGainsToControllers();
    resetControllers();
    persistConfig();
}

float filterAlpha() {
    return g_config.filterAlpha;
}

void setFilterAlpha(float alpha) {
    alpha = sanitizeAlpha(alpha, DEFAULT_FILTER_ALPHA);
    if (fabsf(g_config.filterAlpha - alpha) < 1e-6f) {
        return;
    }
    g_config.filterAlpha = alpha;
    resetFilters();
    persistConfig();
}

uint8_t axisMask() {
    uint8_t mask = 0;
    for (size_t i = 0; i < Control::AXIS_COUNT; ++i) {
        if (g_axisEnabled[i]) {
            mask |= static_cast<uint8_t>(1U << i);
        }
    }
    return mask;
}

void setAxisEnabled(Axis axis, bool enabled) {
    const size_t idx = axisIndex(axis);
    if (g_axisEnabled[idx] == enabled) {
        return;
    }
    g_axisEnabled[idx] = enabled;
    resetAxisFilters(axis);
    resetAxisController(axis);
}

bool axisEnabled(Axis axis) {
    return g_axisEnabled[axisIndex(axis)];
}

void setAxisMask(uint8_t mask) {
    for (size_t i = 0; i < Control::AXIS_COUNT; ++i) {
        const bool enabled = (mask & (1U << i)) != 0;
        setAxisEnabled(static_cast<Axis>(i), enabled);
    }
}

float quadFilterAlpha() {
    return g_config.quadFilterAlpha;
}

void setQuadFilterAlpha(float alpha) {
    alpha = sanitizeAlpha(alpha, DEFAULT_QUAD_ALPHA);
    if (fabsf(g_config.quadFilterAlpha - alpha) < 1e-6f) {
        return;
    }
    g_config.quadFilterAlpha = alpha;
    resetFilters();
    persistConfig();
}

} // namespace Control
