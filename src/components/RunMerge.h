#pragma once

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// One CSV file read back for merging: its time column (Unix ms) and the value columns next to it.
struct RunTable {
    std::string prefix;              // put before each column name in the merged file, e.g. "imu1_"
    std::vector<std::string> names;  // value columns only
    std::vector<double> time;
    std::vector<std::vector<std::string>> rows;
};

inline std::vector<std::string> splitCsv(const std::string &line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(cell);
    if (!line.empty() && line.back() == ',') out.emplace_back();
    return out;
}

// Reads a Houston telemetry CSV (time column "Time") or an IMU CSV (time column "UnixMs"); both hold Unix ms.
// Returns false if the file is missing or has no time column.
inline bool readRunTable(const std::string &path, const std::string &prefix, RunTable &t) {
    std::ifstream in(path);
    std::string line;
    if (!in || !std::getline(in, line)) return false;
    auto head = splitCsv(line);
    int timeCol = -1;
    std::vector<int> keep;
    for (int i = 0; i < (int) head.size(); i++) {
        if (head[i] == "Time" || head[i] == "UnixMs") timeCol = i;
        else if (head[i] != "time" && head[i] != "DeviceName") keep.push_back(i);  // the IMU's local clock text and name
    }
    if (timeCol < 0) return false;
    t.prefix = prefix;
    for (int i: keep) t.names.push_back(prefix + head[i]);
    while (std::getline(in, line)) {
        auto cells = splitCsv(line);
        if ((int) cells.size() <= timeCol) continue;  // a row cut short when the app was closed mid-write
        t.time.push_back(std::atof(cells[timeCol].c_str()));
        std::vector<std::string> row;
        for (int i: keep) row.push_back(i < (int) cells.size() ? cells[i] : "");
        t.rows.push_back(std::move(row));
    }
    return true;
}

// Writes one CSV on the first table's timeline (the IMU at 200 Hz, so no IMU sample is lost). Every other table adds
// its latest row at or before each time: telemetry rows hold their last value anyway, a second IMU is at most one
// sample (5 ms) old. Cells stay empty before a table's first row.
inline bool writeMerged(const std::vector<RunTable> &tables, const std::string &outPath) {
    if (tables.empty()) return false;
    std::ofstream out(outPath);
    if (!out) return false;
    out << "Time";
    for (auto &t: tables) for (auto &n: t.names) out << ',' << n;
    out << '\n';
    std::vector<size_t> next(tables.size(), 0);
    char num[32];
    for (size_t r = 0; r < tables[0].time.size(); r++) {
        double now = tables[0].time[r];
        std::snprintf(num, sizeof(num), "%.3f", now);
        out << num;
        for (size_t k = 0; k < tables.size(); k++) {
            auto &t = tables[k];
            size_t &j = next[k];
            if (k == 0) j = r + 1;
            else while (j < t.time.size() && t.time[j] <= now) j++;
            for (size_t c = 0; c < t.names.size(); c++) out << ',' << (j > 0 ? t.rows[j - 1][c] : "");
        }
        out << '\n';
    }
    return true;
}
