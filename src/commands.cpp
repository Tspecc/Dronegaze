#include "commands.h"
#include <WiFi.h>
#include <cstdlib>
#include <cstdio>
#include "motor.h"
#include "control.h"

extern WiFiClient client;
extern float pitch, roll, yaw;
extern float yawSetpoint;
extern float pitchBias, rollBias, yawBias;
extern bool yawControlEnabled;
extern bool failsafe_enable;
extern bool telemetryEnabled;
extern bool serialActive;
extern bool isArmed;
extern Motor::Outputs currentOutputs;
extern Motor::Outputs targetOutputs;
extern bool stabilizationEnabled;
extern bool requestIMUZero();

static bool parseAxisToken(const String &token, Control::Axis &axis) {
    if (token.equalsIgnoreCase("roll")) {
        axis = Control::Axis::Roll;
        return true;
    }
    if (token.equalsIgnoreCase("pitch")) {
        axis = Control::Axis::Pitch;
        return true;
    }
    if (token.equalsIgnoreCase("yaw")) {
        axis = Control::Axis::Yaw;
        return true;
    }
    if (token.equalsIgnoreCase("vertical") || token.equalsIgnoreCase("vert")) {
        axis = Control::Axis::Vertical;
        return true;
    }
    return false;
}

static const char *axisName(Control::Axis axis) {
    switch (axis) {
        case Control::Axis::Roll: return "roll";
        case Control::Axis::Pitch: return "pitch";
        case Control::Axis::Yaw: return "yaw";
        case Control::Axis::Vertical: return "vertical";
    }
    return "unknown";
}

namespace Commands {

void sendLine(const String &line) {
    Serial.println(line);
    if (client && client.connected()) {
        client.println(line);
        client.flush();
    }
}

static unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 1000;

void sendHeartbeat() {
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        lastHeartbeat = millis();
        sendLine("HEARTBEAT:" + String(millis()));
    }
}

void handleCommand(const String &cmd) {
    String trimmed = cmd;
    trimmed.trim();
    if (trimmed.length() == 0)
        return;

    if (trimmed == "status") {
        sendLine("System: " + String(millis()) + "ms uptime");
        sendLine("Pitch:" + String(pitch, 2) + " Roll:" + String(roll, 2) + " Yaw:" + String(yaw, 2));
        sendLine("Yaw Setpoint: " + String(yawSetpoint, 2) + "\xC2\xB0");
        sendLine("Bias Pitch:" + String(pitchBias, 2) + " Roll:" + String(rollBias, 2) + " Yaw:" + String(yawBias, 2));
        sendLine(String("PID Control: ") + (Control::pidEnabled() ? "ON" : "OFF"));
        sendLine(String("Filters: primary ") + (Control::filtersEnabled() ? "ON" : "OFF") +
                 " (alpha=" + String(Control::filterAlpha(), 3) + "), secondary " +
                 (Control::quadFiltersEnabled() ? "ON" : "OFF") +
                 " (alpha=" + String(Control::quadFilterAlpha(), 3) + ")");
    } else if(trimmed == "failsafe on"){failsafe_enable=1; sendLine("Enabled failsafe mode");}
    else if(trimmed == "failsafe off"){failsafe_enable=0; sendLine("Disabled failsafe mode");}
    else if (trimmed == "filters on") {
        Control::setFiltersEnabled(true);
        sendLine("ACK: Filters enabled");
    }
    else if (trimmed == "filters off") {
        Control::setFiltersEnabled(false);
        sendLine("ACK: Filters disabled");
    }
    else if (trimmed.startsWith("filters alpha ")) {
        String valueStr = trimmed.substring(14);
        valueStr.trim();
        char *endPtr = nullptr;
        float alpha = strtof(valueStr.c_str(), &endPtr);
        if (endPtr == valueStr.c_str()) {
            sendLine("ERROR: Invalid filter alpha");
        } else if (alpha < 0.0f || alpha > 1.0f) {
            sendLine("ERROR: Filter alpha must be between 0 and 1");
        } else {
            Control::setFilterAlpha(alpha);
            sendLine("ACK: Primary filter alpha set to " + String(Control::filterAlpha(), 3));
        }
    }
    else if (trimmed == "filters status") {
        sendLine(String("Primary filter: ") + (Control::filtersEnabled() ? "ON" : "OFF") +
                 " alpha=" + String(Control::filterAlpha(), 3));
        sendLine(String("Secondary filter: ") + (Control::quadFiltersEnabled() ? "ON" : "OFF") +
                 " alpha=" + String(Control::quadFilterAlpha(), 3));
    }
    else if (trimmed == "quadfilters on") {
        Control::setQuadFiltersEnabled(true);
        sendLine("ACK: Quad filters enabled");
    }
    else if (trimmed == "quadfilters off") {
        Control::setQuadFiltersEnabled(false);
        sendLine("ACK: Quad filters disabled");
    }
    else if (trimmed.startsWith("quadfilters alpha ")) {
        String valueStr = trimmed.substring(18);
        valueStr.trim();
        char *endPtr = nullptr;
        float alpha = strtof(valueStr.c_str(), &endPtr);
        if (endPtr == valueStr.c_str()) {
            sendLine("ERROR: Invalid quad filter alpha");
        } else if (alpha < 0.0f || alpha > 1.0f) {
            sendLine("ERROR: Quad filter alpha must be between 0 and 1");
        } else {
            Control::setQuadFilterAlpha(alpha);
            sendLine("ACK: Secondary filter alpha set to " + String(Control::quadFilterAlpha(), 3));
        }
    }
    else if (trimmed == "pid on") {
        Control::setPidEnabled(true);
        sendLine("ACK: PID control enabled");
    }
    else if (trimmed == "pid off") {
        Control::setPidEnabled(false);
        sendLine("ACK: PID control disabled");
    }
    else if (trimmed == "pid reset") {
        Control::resetGains();
        sendLine("ACK: PID gains reset to defaults");
    }
    else if (trimmed == "pid show") {
        Control::Gains rollG = Control::getGains(Control::Axis::Roll);
        Control::Gains pitchG = Control::getGains(Control::Axis::Pitch);
        Control::Gains yawG = Control::getGains(Control::Axis::Yaw);
        Control::Gains vertG = Control::getGains(Control::Axis::Vertical);
        sendLine("PID roll: Kp=" + String(rollG.kp, 3) + " Ki=" + String(rollG.ki, 3) + " Kd=" + String(rollG.kd, 3));
        sendLine("PID pitch: Kp=" + String(pitchG.kp, 3) + " Ki=" + String(pitchG.ki, 3) + " Kd=" + String(pitchG.kd, 3));
        sendLine("PID yaw: Kp=" + String(yawG.kp, 3) + " Ki=" + String(yawG.ki, 3) + " Kd=" + String(yawG.kd, 3));
        sendLine("PID vertical: Kp=" + String(vertG.kp, 3) + " Ki=" + String(vertG.ki, 3) + " Kd=" + String(vertG.kd, 3));
    }
    else if (trimmed.startsWith("pid set ")) {
        String rest = trimmed.substring(8);
        rest.trim();
        int spaceIdx = rest.indexOf(' ');
        if (spaceIdx < 0) {
            sendLine("ERROR: Usage pid set <axis> <Kp> <Ki> <Kd>");
        } else {
            String axisToken = rest.substring(0, spaceIdx);
            Control::Axis axis;
            if (!parseAxisToken(axisToken, axis)) {
                sendLine("ERROR: Unknown axis; use roll, pitch, yaw, or vertical");
            } else {
                String values = rest.substring(spaceIdx + 1);
                values.trim();
                float kp = 0.0f, ki = 0.0f, kd = 0.0f;
                if (sscanf(values.c_str(), "%f %f %f", &kp, &ki, &kd) != 3) {
                    sendLine("ERROR: Usage pid set <axis> <Kp> <Ki> <Kd>");
                } else {
                    Control::Gains gains{kp, ki, kd};
                    Control::setGains(axis, gains);
                    sendLine(String("ACK: PID ") + axisName(axis) + " gains updated");
                }
            }
        }
    }
    else if (trimmed == "yawon") {
        yawControlEnabled = true;
        yawSetpoint = yaw;
        sendLine("ACK: Yaw control enabled");
    } else if (trimmed == "yawoff") {
        yawControlEnabled = false;
        sendLine("ACK: Yaw control disabled");
    } else if (trimmed.startsWith("pitchbias ")) {
        pitchBias = trimmed.substring(10).toFloat();
        sendLine("ACK: Pitch bias set to " + String(pitchBias, 2) + "\xC2\xB0");
    } else if (trimmed.startsWith("rollbias ")) {
        rollBias = trimmed.substring(9).toFloat();
        sendLine("ACK: Roll bias set to " + String(rollBias, 2) + "\xC2\xB0");
    } else if (trimmed.startsWith("yawbias ")) {
        yawBias = trimmed.substring(8).toFloat();
        sendLine("ACK: Yaw bias set to " + String(yawBias, 2) + "\xC2\xB0");
    } else if (trimmed.startsWith("yaw ")) {
        float newYawSetpoint = trimmed.substring(4).toFloat();
        yawSetpoint = newYawSetpoint;
        yawControlEnabled = true;
        sendLine("ACK: Yaw setpoint set to " + String(yawSetpoint, 2) + "\xC2\xB0");
    } else if (trimmed.equalsIgnoreCase("imu zero")) {
        bool queued = requestIMUZero();
        if (queued) {
            sendLine("ACK: IMU zero requested");
        } else {
            sendLine("WARN: IMU zero already pending");
        }
    } else if (trimmed == "telemetry on") {
        telemetryEnabled = true;
        sendLine("ACK: Telemetry enabled");
    } else if (trimmed == "telemetry off") {
        telemetryEnabled = false;
        sendLine("ACK: Telemetry disabled");
    } else if (trimmed == "ping") {
        sendLine("PONG");
    } else if (trimmed == "arm") {
        isArmed = true;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        delay(1000);
        sendLine("ACK: Motors armed");
    } else if (trimmed == "disarm") {
        isArmed = false;
        Motor::update(isArmed, currentOutputs, targetOutputs);
        sendLine("ACK: Motors disarmed");
    } else if (trimmed.equalsIgnoreCase("stabilization on")) {
        stabilizationEnabled = true;
        sendLine("ACK: Stabilization enabled");
    } else if (trimmed.equalsIgnoreCase("stabilization off")) {
        stabilizationEnabled = false;
        sendLine("ACK: Stabilization disabled");
    } else {
        sendLine("ERROR: Unknown command");
    }
}

} // namespace Commands
