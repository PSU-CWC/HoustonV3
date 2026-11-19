#pragma once

#include <array>
#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include "../pch.h"
#include "Component.h"
#include <simpleble/SimpleBLE.h>

// One WitMotion sensor seen in a scan. Kept for the whole session so its slot, file and reconnects stay put.
struct ImuSensor {
    SimpleBLE::Peripheral peripheral;
    std::string address, label;
    std::string service, notify;
    int slot = 0;                // 1..3 while the user wants it connected, 0 otherwise; names its file (_imu1, _imu2, ...)
    bool connected = false;
    bool dropped = false;        // link lost; the worker reconnects
    std::ofstream file;
    std::string fileName;
    std::string status = "found";
    std::array<float, 9> latest{};
    uint64_t samples = 0, rateCount = 0;
    long long periodUs = 5000;   // time between samples at the rate set on connect
    long long lastUs = 0;        // Unix µs of the last notification
    double rateHz = 0, lastSample = 0;  // lastSample: steady-clock seconds of the last notification
};

// Records up to three WitMotion WT901BLE sensors over Bluetooth into CSV files that blade_imu's load() reads.
// Files are written only while TelemetryPanel saves its CSV, next to it as <csv name>_imu<slot>.csv; on stop all of
// them are joined into <csv name>_merged.csv. Every row
// carries the same system clock the CSV's Time column uses, so the files line up without a sync step.
class ImuPanel : public Component {
private:
    static constexpr int kMaxSensors = 3;
    std::thread worker;
    std::atomic<bool> running{true}, scanRequested{false};
    std::mutex mtx;  // guards everything below; notifications and disconnect callbacks arrive on SimpleBLE's threads
    std::vector<std::unique_ptr<ImuSensor>> sensors;  // only appended to, so callbacks can hold a reference
    std::optional<SimpleBLE::Adapter> adapter;
    std::string status = "Press Scan";
    std::string recordCsv;   // telemetry CSV path of the current or last recording
    std::string recordBase;  // that path without extension while recording, empty otherwise
    char namePrefix[32] = "WT";
    int rateIndex = 4;
    double rateTime = 0;

    void run();
    void addSensor(SimpleBLE::Peripheral &p, const std::string &name, const std::string &status);
    int scan();  // returns how many sensors matched
    void connect(ImuSensor &s);
    void disconnect(ImuSensor &s);
    void onNotify(ImuSensor &s, const SimpleBLE::ByteArray &payload);

public:
    ImuPanel(const char *name, Dispatcher *dispatcher);

    ~ImuPanel() override;

    void render() override;

    void startRecording(const std::string &csvPath);

    void stopRecording();

    // Latest sample of every connected sensor as ("imu1 GyroX", value) pairs, plus "imu1 rpm", for the Digital Plots
    std::vector<std::pair<std::string, float>> latestValues();
};
