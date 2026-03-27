#pragma once
#include <cstdint>
#include "PiSubmarine/RegUtils.h"

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

    constexpr HalfBridge ToHalfBridgeEnum(uint8_t index)
    {
        if (index >= 8)
        {
            throw std::out_of_range("Index is out of range");
        }
        return static_cast<HalfBridge>(1 << index);
    }

    constexpr uint8_t ToIndex(HalfBridge hBridge)
    {
        using namespace RegUtils;
        for (uint8_t index = 0; index < 8; index++)
        {
            auto mask = static_cast<HalfBridge>(1 << index);
            if ((hBridge & mask) == hBridge)
            {
               return index;
            }
        }
        throw std::invalid_argument("Invalid HalfBridge value");
    }
}
