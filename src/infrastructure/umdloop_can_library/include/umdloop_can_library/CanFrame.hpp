#pragma once

#include <array>
#include <cstdint>

namespace CANLib {

    struct CanFrame {
        uint32_t id;       // CAN ID
        uint8_t dlc;       // Data Length Code
        std::array<uint8_t, 8> data; // Data bytes (max 8 for classical CAN)
        bool is_extended;  // True if extended ID (29-bit), false if standard (11-bit)
        bool is_rtr;       // True if Remote Transmission Request

        CanFrame() : id(0), dlc(0), data({}), is_extended(false), is_rtr(false) {}
    };

} // namespace CANLib
