#pragma once
#include <cstdint>
#include "PiSubmarine/RegUtils.h"

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

    constexpr PwmGenerator ToPwmGeneratorEnum(uint8_t index)
    {
        if (index >= 8)
        {
            throw std::out_of_range("Index is out of range");
        }
        return static_cast<PwmGenerator>(1 << index);
    }

    constexpr uint8_t ToIndex(PwmGenerator hBridge)
    {
        using namespace RegUtils;
        for (uint8_t index = 0; index < 8; index++)
        {
            auto mask = static_cast<PwmGenerator>(1 << index);
            if ((hBridge & mask) == hBridge)
            {
                return index;
            }
        }
        throw std::invalid_argument("Invalid PwmGenerator value");
    }
}
