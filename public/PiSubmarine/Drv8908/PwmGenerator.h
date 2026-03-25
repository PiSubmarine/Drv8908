#pragma once
#include <cstdint>

namespace PiSubmarine::Drv8908
{
    enum class PwmGenerator : uint8_t
    {
        PwmGenerator1 = 1 << 0,
        PwmGenerator2 = 1 << 1,
        PwmGenerator3 = 1 << 2,
        PwmGenerator4 = 1 << 3,
        PwmGenerator5 = 1 << 4,
        PwmGenerator6 = 1 << 5,
        PwmGenerator7 = 1 << 6,
        PwmGenerator8 = 1 << 7
    };
}
