#pragma once
#include <cstdint>

namespace PiSubmarine::Drv8908
{
    enum class HalfBridge : uint8_t
    {
        HalfBridge1 = 1 << 0,
        HalfBridge2 = 1 << 1,
        HalfBridge3 = 1 << 2,
        HalfBridge4 = 1 << 3,
        HalfBridge5 = 1 << 4,
        HalfBridge6 = 1 << 5,
        HalfBridge7 = 1 << 6,
        HalfBridge8 = 1 << 7
    };
}
