#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

// One WitMotion 0x55 0x61 frame (20 bytes): acc ±16 g, gyro ±2000 °/s, angle ±180 °, each int16 little-endian.
// Returns acc xyz, gyro xyz, angle xyz for every frame found in a notification (the sensor packs several per notification).
inline std::vector<std::array<float, 9>> parseWitMotion(const uint8_t *buf, size_t n) {
    static const float scale[9] = {16, 16, 16, 2000, 2000, 2000, 180, 180, 180};
    std::vector<std::array<float, 9>> rows;
    for (size_t i = 0; i + 20 <= n;) {
        if (buf[i] != 0x55 || buf[i + 1] != 0x61) { i++; continue; }
        std::array<float, 9> r{};
        for (int k = 0; k < 9; k++) {
            auto v = (int16_t) (buf[i + 2 + 2 * k] | (buf[i + 3 + 2 * k] << 8));
            r[k] = v / 32768.0f * scale[k];
        }
        rows.push_back(r);
        i += 20;
    }
    return rows;
}
