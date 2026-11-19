#pragma once

#include <fstream>
#include "../pch.h"

#include "Component.h"
#include "../Util.h"
#include "../Setting.h"

class ImuPanel;
class LiveDataPanel;

class TelemetryPanel : public Component {
private:
    std::map<std::string, std::string> telemetryMap;
    bool paused = false;
    bool initialized = false; // Fixed spelling
    float history = 10.0f;
    bool autoScale = true;
    
    std::map<std::string, Util::ScrollingBuffer *> dataMap;
    std::map<std::string, bool> showMap; // Changed bool* to bool for safety
    
    char csvFileBuffer[256] = "data.csv";
    bool savingFile = false;
    std::vector<std::string> csvHeaders;
    std::vector<std::string> liveHeaders;  // Live Data labels written after the telemetry columns
    std::ofstream file;

    int timer = 0;

public:
    LiveDataPanel *liveData = nullptr;  // its values go into the CSV as "Live <label>" columns
    ImuPanel *imu = nullptr;  // starts and stops its IMU files with this panel's CSV

    TelemetryPanel(const char *name, Dispatcher *dispatcher) 
        : Component(name, dispatcher), initialized(true) { // Set to true here or in start()
    }

    ~TelemetryPanel() override;
    void start() override;
    void render() override;
    void stop() override;
    void graphData();
};