#include "ImuPanel.h"
#include "WitMotion.h"
#include "RunMerge.h"
#include <algorithm>
#include <cctype>
#include <ctime>
#include <cmath>
#include <filesystem>
#include <simpleble/Advanced.h>

static const int kRates[] = {10, 20, 50, 100, 200};
static const uint8_t kRateCodes[] = {0x06, 0x07, 0x08, 0x09, 0x0B};  // WitMotion return-rate register (0x03) values
static const double kStallSeconds = 3.0;
// Sensors connected before (one address and name per line), so a restarted Houston can find one macOS still holds
static const char *kKnownFile = "imu_sensors.txt";

static std::vector<std::pair<std::string, std::string>> loadKnown() {
    std::vector<std::pair<std::string, std::string>> known;
    std::ifstream in(kKnownFile);
    std::string addr, name;
    while (in >> addr >> name) known.emplace_back(addr, name);
    return known;
}

static void rememberSensor(const std::string &addr, const std::string &name) {
    auto known = loadKnown();
    for (auto &k: known) if (k.first == addr) return;
    std::ofstream(kKnownFile, std::ios::app) << addr << ' ' << (name.empty() ? "WT" : name) << '\n';
}  // connected but silent this long counts as a dropped link

// Backends report a 16-bit UUID either as "ffe4" or inside the 128-bit base "0000ffe4-0000-1000-8000-00805f9a34fb"
static bool isUuid16(std::string uuid, const char *short16) {
    std::transform(uuid.begin(), uuid.end(), uuid.begin(), [](unsigned char c) { return std::tolower(c); });
    return uuid == short16 || (uuid.size() == 36 && uuid.compare(4, 4, short16) == 0);
}

static std::string localStamp(std::chrono::system_clock::time_point t, const char *fmt) {
    std::time_t tt = std::chrono::system_clock::to_time_t(t);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), fmt, &tm);
    return buf;
}

static double steadyNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

ImuPanel::ImuPanel(const char *name, Dispatcher *dispatcher) : Component(name, dispatcher) {
    worker = std::thread([this] { run(); });
}

ImuPanel::~ImuPanel() {
    running = false;
    if (worker.joinable()) worker.join();
    for (auto &s: sensors) {
        if (s->connected || s->slot > 0) disconnect(*s);  // the second also cancels a pending request
    }
}

// Every scan, connect and disconnect runs on this one thread so BLE calls never overlap; the UI only flips flags.
// A sensor the user wants (slot > 0) is reconnected whenever its link drops or its data stops.
void ImuPanel::run() {
    while (running) {
        if (scanRequested.exchange(false)) {
            // macOS Bluetooth needs a moment after launch (and a sensor just switched on may not have advertised yet)
            for (int i = 0; i < 3 && running && scan() == 0; i++) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        for (size_t i = 0;; i++) {
            ImuSensor *s;
            bool want, conn, drop;
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (i >= sensors.size()) break;
                s = sensors[i].get();
                want = s->slot > 0;
                conn = s->connected;
                drop = s->dropped || (conn && steadyNow() - s->lastSample > kStallSeconds);
            }
            if (conn && (!want || drop)) disconnect(*s);
            if (want && (!conn || drop)) connect(*s);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

// Caller holds mtx.
void ImuPanel::addSensor(SimpleBLE::Peripheral &p, const std::string &name, const std::string &status) {
    auto s = std::make_unique<ImuSensor>();
    std::string addr = p.address(), tail;
    for (char c: addr) if (std::isalnum((unsigned char) c)) tail += c;
    s->peripheral = p;
    s->address = addr;
    s->label = name + "_" + tail.substr(tail.size() > 4 ? tail.size() - 4 : 0);
    s->status = status;
    sensors.push_back(std::move(s));
}

int ImuPanel::scan() {
    int found = 0;
    try {
        if (!adapter) {
            if (!SimpleBLE::Adapter::bluetooth_enabled()) {
                std::lock_guard<std::mutex> lock(mtx);
                status = "Bluetooth is off or not ready yet";
                return 0;
            }
            auto adapters = SimpleBLE::Adapter::get_adapters();
            if (adapters.empty()) {
                std::lock_guard<std::mutex> lock(mtx);
                status = "No Bluetooth adapter found";
                return 0;
            }
            adapter = adapters[0];
        }
        std::string prefix;
        {
            std::lock_guard<std::mutex> lock(mtx);
            prefix = namePrefix;
            status = "Scanning for " + prefix + "...";
        }
        adapter->scan_for(3000);
        for (auto &p: adapter->scan_get_results()) {
            if (p.identifier().rfind(prefix, 0) != 0) continue;
            found++;
            std::string addr = p.address();
            std::lock_guard<std::mutex> lock(mtx);
            auto it = std::find_if(sensors.begin(), sensors.end(), [&](auto &s) { return s->address == addr; });
            if (it == sensors.end()) {
                addSensor(p, p.identifier(), "found");
            } else if (!(*it)->connected) {
                (*it)->peripheral = p;  // fresh handle: macOS can hand back a stale one for a sensor that went away
            }
        }
#ifdef __APPLE__
        // A sensor macOS still holds from an earlier run (Houston quit without releasing it) does not advertise, so
        // the scan misses it; ask CoreBluetooth for remembered ones by address and offer them anyway
        std::vector<std::pair<std::string, std::string>> missing;
        {
            std::lock_guard<std::mutex> lock(mtx);
            for (auto &k: loadKnown()) {
                if (std::none_of(sensors.begin(), sensors.end(), [&](auto &s) { return s->address == k.first; }))
                    missing.push_back(k);
            }
        }
        if (!missing.empty()) {
            std::vector<SimpleBLE::BluetoothAddress> addrs;
            for (auto &m: missing) addrs.push_back(m.first);
            for (auto &p: SimpleBLE::Advanced::MacOS::retrieve_cached_peripherals(*adapter, addrs)) {
                auto m = std::find_if(missing.begin(), missing.end(), [&](auto &k) { return k.first == p.address(); });
                std::lock_guard<std::mutex> lock(mtx);
                addSensor(p, m != missing.end() ? m->second : p.identifier(), "remembered, not advertising: try Connect");
                found++;
            }
        }
#endif
        std::lock_guard<std::mutex> lock(mtx);
        status = found ? "Found " + std::to_string(found) + " sensor(s)"
                       : "No sensor found: is it on, and disconnected from the phone app?";
    } catch (std::exception &e) {
        std::lock_guard<std::mutex> lock(mtx);
        status = std::string("Scan error: ") + e.what();
    }
    return found;
}

void ImuPanel::connect(ImuSensor &s) {
    SimpleBLE::Peripheral p;
    int rate;
    {
        std::lock_guard<std::mutex> lock(mtx);
        p = s.peripheral;
        rate = rateIndex;
        s.status = "Connecting...";
    }
    try {
        // Already connected here means CoreBluetooth finished a request left over from a failed try: SimpleBLE then
        // skips service discovery, so start that link over
        if (p.is_connected()) p.disconnect();
        p.connect();

        std::string service, notify, write, seen;
        for (auto &sv: p.services()) {
            for (auto &c: sv.characteristics()) {
                seen += c.uuid() + " ";
                if (isUuid16(c.uuid(), "ffe4")) service = sv.uuid(), notify = c.uuid();
                if (isUuid16(c.uuid(), "ffe9")) write = c.uuid();
            }
        }
        if (notify.empty() || write.empty()) {
            p.disconnect();
            std::lock_guard<std::mutex> lock(mtx);
            s.slot = 0;  // not a sensor this code can read; stop retrying
            s.status = "FFE4/FFE9 not found. Characteristics: " + seen;
            return;
        }

        // unlock, then set the return rate; not saved to flash, so the sensor keeps its own setting after power-off
        p.write_command(service, write, SimpleBLE::ByteArray(std::vector<uint8_t>{0xFF, 0xAA, 0x69, 0x88, 0xB5}));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        p.write_command(service, write, SimpleBLE::ByteArray(std::vector<uint8_t>{0xFF, 0xAA, 0x03, kRateCodes[rate], 0x00}));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        p.set_callback_on_disconnected([this, &s] {
            std::lock_guard<std::mutex> lock(mtx);
            if (s.slot > 0) {
                s.dropped = true;
                s.status = "Link lost, reconnecting...";
            }
        });
        {
            std::lock_guard<std::mutex> lock(mtx);
            s.service = service;
            s.notify = notify;
            s.connected = true;
            s.dropped = false;
            s.lastSample = steadyNow();
            s.periodUs = 1000000 / kRates[rate];
            s.status = "Connected";
        }
        rememberSensor(s.address, s.label.substr(0, s.label.rfind('_')));
        p.notify(service, notify, [this, &s](SimpleBLE::ByteArray payload) { onNotify(s, payload); });
    } catch (std::exception &e) {
        {
            std::lock_guard<std::mutex> lock(mtx);
            s.status = std::string("Connect failed, retrying: ") + e.what();
        }
        // CoreBluetooth keeps a timed-out connect request alive forever and takes the sensor the moment it advertises
        // again, hidden from scans (the "toggle Bluetooth" case); cancel it so the next try starts clean
        try {
            p.disconnect();
        } catch (std::exception &) {
        }
        scan();  // refresh the handle before the next try
    }
}

void ImuPanel::disconnect(ImuSensor &s) {
    SimpleBLE::Peripheral p;
    std::string service, notify;
    {
        std::lock_guard<std::mutex> lock(mtx);
        p = s.peripheral;
        service = s.service;
        notify = s.notify;
        s.connected = false;
        s.dropped = false;
    }
    try {
        if (p.is_connected() && !service.empty()) p.unsubscribe(service, notify);
        p.disconnect();  // also cancels a request still pending; a no-op when already disconnected
    } catch (std::exception &e) {
        std::cerr << "ImuPanel: disconnect: " << e.what() << std::endl;
    }
    std::lock_guard<std::mutex> lock(mtx);
    s.status = s.slot > 0 ? "Reconnecting..." : "Disconnected";
}

void ImuPanel::onNotify(ImuSensor &s, const SimpleBLE::ByteArray &payload) {
    auto now = std::chrono::system_clock::now();
    auto rows = parseWitMotion(payload.data(), payload.size());
    if (rows.empty()) return;
    long long nowUs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

    std::lock_guard<std::mutex> lock(mtx);
    s.latest = rows.back();
    s.samples += rows.size();
    s.lastSample = steadyNow();
    if (recordBase.empty() || s.slot == 0) return;
    if (!s.file.is_open()) {
        s.fileName = recordBase + "_imu" + std::to_string(s.slot) + ".csv";
        s.file.open(s.fileName);
        if (!s.file.is_open()) {
            s.status = "Could not open " + s.fileName;
            return;
        }
        s.file << "time,DeviceName,AccX(g),AccY(g),AccZ(g),AsX(°/s),AsY(°/s),AsZ(°/s)"
                  ",AngleX(°),AngleY(°),AngleZ(°),UnixMs\n";
    }
    // the sensor packs several samples per notification; the last one arrived now, the others one period apart,
    // squeezed when this notification came sooner than that after the previous one so times never go backwards
    long long n = (long long) rows.size(), step = s.periodUs;
    if (s.lastUs > 0) step = std::max(0LL, std::min(step, (nowUs - s.lastUs) / n));
    s.lastUs = nowUs;
    char num[40];
    for (long long k = 0; k < n; k++) {
        long long us = nowUs - (n - 1 - k) * step;
        auto tp = std::chrono::system_clock::time_point(std::chrono::microseconds(us));
        std::snprintf(num, sizeof(num), ".%06lld", us % 1000000);
        s.file << localStamp(tp, "%Y-%m-%dT%H:%M:%S") << num << ',' << s.label;
        for (float v: rows[k]) {
            std::snprintf(num, sizeof(num), ",%.3f", v);
            s.file << num;
        }
        std::snprintf(num, sizeof(num), ",%.3f\n", us / 1000.0);  // same clock as the telemetry CSV's Time
        s.file << num;
    }
    s.file.flush();  // components are never destroyed on quit, so unflushed rows would be lost
}

void ImuPanel::startRecording(const std::string &csvPath) {
    std::lock_guard<std::mutex> lock(mtx);
    recordCsv = csvPath;
    recordBase = std::filesystem::path(csvPath).replace_extension("").string();
    for (auto &s: sensors) s->file.close();  // each sensor opens its file on its next notification
}

// Call after the telemetry CSV is closed: also writes <csv name>_merged.csv from it and the IMU files.
void ImuPanel::stopRecording() {
    std::vector<std::pair<int, std::string>> imuFiles;  // slot, path
    std::string csv, base;
    {
        std::lock_guard<std::mutex> lock(mtx);
        for (auto &s: sensors) {
            if (s->file.is_open()) imuFiles.emplace_back(s->slot, s->fileName);
            s->file.close();
        }
        csv = recordCsv;
        base = recordBase;
        recordBase.clear();
    }
    if (imuFiles.empty()) return;
    std::sort(imuFiles.begin(), imuFiles.end());

    // shortcut: runs on the UI thread, ~1 s pause per 10 min of 3 sensors; move to a thread if that gets annoying
    std::vector<RunTable> tables;
    for (auto &[slot, path]: imuFiles) {
        RunTable t;
        if (readRunTable(path, "imu" + std::to_string(slot) + "_", t)) tables.push_back(std::move(t));
    }
    RunTable telemetry;
    if (readRunTable(csv, "", telemetry)) tables.push_back(std::move(telemetry));
    std::string out = base + "_merged.csv";
    bool ok = !tables.empty() && writeMerged(tables, out);
    std::lock_guard<std::mutex> lock(mtx);
    status = ok ? "Saved " + out : "Could not write " + out;
}

std::vector<std::pair<std::string, float>> ImuPanel::latestValues() {
    static const char *names[9] = {"AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "AngleX", "AngleY", "AngleZ"};
    std::vector<std::pair<std::string, float>> out;
    std::lock_guard<std::mutex> lock(mtx);
    for (int slot = 1; slot <= kMaxSensors; slot++) {
        auto it = std::find_if(sensors.begin(), sensors.end(), [&](auto &o) { return o->slot == slot && o->connected; });
        if (it == sensors.end()) continue;
        auto &s = *it;
        std::string tag = "imu" + std::to_string(s->slot) + " ";
        for (int k = 0; k < 9; k++) out.emplace_back(tag + names[k], s->latest[k]);
        auto &g = s->latest;
        // spin speed whatever way the sensor is mounted: |gyro| deg/s / 360 * 60 = rpm
        out.emplace_back(tag + "rpm", std::sqrt(g[3] * g[3] + g[4] * g[4] + g[5] * g[5]) / 6.0f);
    }
    return out;
}

void ImuPanel::render() {
    ImGui::Begin("IMU (Bluetooth)");
    std::lock_guard<std::mutex> lock(mtx);

    double now = ImGui::GetTime();
    if (now - rateTime >= 1.0) {
        for (auto &s: sensors) {
            s->rateHz = (s->samples - s->rateCount) / (now - rateTime);
            s->rateCount = s->samples;
        }
        rateTime = now;
    }

    int wanted = 0;
    for (auto &s: sensors) wanted += s->slot > 0;

    ImGui::InputText("Name starts with", namePrefix, IM_ARRAYSIZE(namePrefix));
    ImGui::Combo("Rate", &rateIndex, "10 Hz\0" "20 Hz\0" "50 Hz\0" "100 Hz\0" "200 Hz\0");
    ImGui::SameLine();
    ImGui::TextDisabled("(on connect)");
    if (ImGui::Button("Scan")) scanRequested = true;
    ImGui::SameLine();
    ImGui::TextWrapped("%s", status.c_str());
    if (recordBase.empty()) {
        ImGui::TextColored(ImVec4(1, 0.7f, 0.2f, 1), "Not saving: press Start Saving File in Telemetry");
    } else {
        ImGui::TextColored(ImVec4(0, 1, 0, 1), "REC: %s_imu<slot>.csv", recordBase.c_str());
    }

    for (size_t i = 0; i < sensors.size(); i++) {
        auto &s = *sensors[i];
        ImGui::PushID((int) i);
        ImGui::Separator();
        if (s.slot > 0) ImGui::Text("[%d] %s", s.slot, s.label.c_str());
        else ImGui::Text("[-] %s", s.label.c_str());
        ImGui::SameLine();
        if (s.slot == 0) {
            ImGui::BeginDisabled(wanted >= kMaxSensors);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.6f, 0.1f, 1.0f));
            if (ImGui::Button("Connect")) {
                int slot = 1;
                while (std::any_of(sensors.begin(), sensors.end(), [&](auto &o) { return o->slot == slot; })) slot++;
                s.slot = slot;
                s.status = "Waiting to connect...";
            }
            ImGui::PopStyleColor();
            ImGui::EndDisabled();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.8f, 0.1f, 0.1f, 1.0f));
            if (ImGui::Button("Disconnect")) s.slot = 0;
            ImGui::PopStyleColor();
        }
        ImGui::TextWrapped("%s", s.status.c_str());
        if (s.connected) {
            ImGui::Text("%llu samples, %.1f Hz", (unsigned long long) s.samples, s.rateHz);
            ImGui::Text("acc  %7.3f %7.3f %7.3f g", s.latest[0], s.latest[1], s.latest[2]);
            ImGui::Text("gyro %8.1f %8.1f %8.1f deg/s", s.latest[3], s.latest[4], s.latest[5]);
            if (s.file.is_open()) ImGui::TextDisabled("%s", s.fileName.c_str());
        }
        ImGui::PopID();
    }
    ImGui::End();
}
