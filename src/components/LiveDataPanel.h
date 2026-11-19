
#pragma once

#include "../pch.h"
#include "Component.h"

class LiveDataPanel : public Component {
private:
    std::map<uint16_t, LiveDataPacket_t> liveDataMap;
    std::map<uint16_t, int32_t *> intMap;
    std::map<uint16_t, float *> floatMap;

    void sendModifyPacket(uint8_t packetID, uint16_t valueType);
    void refreshPackets();
    std::map<std::string, std::string> applied;  // label -> value the ESP holds (as received, or as last applied)
    void remember(uint16_t id);

public:
    LiveDataPanel(const char *name, Dispatcher *dispatcher) : Component(name, dispatcher) {};

    ~LiveDataPanel() override;

    void start() override;

    void render() override;

    void stop() override;

    // Values the ESP currently holds, for the telemetry CSV (e.g. the wind speed typed in for each step)
    const std::map<std::string, std::string> &appliedValues() const { return applied; }
};


