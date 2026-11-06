/**
 * @file RemoteController.h
 * @brief Wireless Remote Controller Parser for GO2W
 */

#ifndef REMOTE_CONTROLLER_H
#define REMOTE_CONTROLLER_H

#include <cstdint>
#include <cstring>
#include <vector>

namespace KeyMap {
    constexpr int R1 = 0, L1 = 1, start = 2, select = 3;
    constexpr int R2 = 4, L2 = 5, F1 = 6, F2 = 7;
    constexpr int A = 8, B = 9, X = 10, Y = 11;
    constexpr int up = 12, right = 13, down = 14, left = 15;
}

struct RemoteController {
    float lx = 0, ly = 0, rx = 0, ry = 0;
    int button[16] = {0};

    void set(const std::vector<uint8_t>& data) {
        if (data.size() >= 24) {
            uint16_t keys;
            std::memcpy(&keys, &data[2], sizeof(uint16_t));
            for (int i = 0; i < 16; i++)
                button[i] = (keys & (1 << i)) >> i;
            std::memcpy(&lx, &data[4], sizeof(float));
            std::memcpy(&rx, &data[8], sizeof(float));
            std::memcpy(&ry, &data[12], sizeof(float));
            std::memcpy(&ly, &data[20], sizeof(float));
        }
    }
};

#endif
