#pragma once
#include <cstdint>
#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Drv8908/HalfBridge.h"

namespace PiSubmarine::Drv8908
{
    enum class HalfBridgeBitMask : uint8_t
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

    constexpr HalfBridgeBitMask ToHalfBridgeBitMask(HalfBridge hb)
    {
        auto index = static_cast<uint8_t>(hb);
        if (index >= 8)
        {
            throw std::out_of_range("Index is out of range");
        }
        return static_cast<HalfBridgeBitMask>(1 << index);
    }

    constexpr HalfBridge ToHalfBridge(HalfBridgeBitMask hBridge)
    {
        using namespace RegUtils;
        for (uint8_t index = 0; index < 8; index++)
        {
            auto mask = static_cast<HalfBridgeBitMask>(1 << index);
            if ((hBridge & mask) == hBridge)
            {
               return static_cast<HalfBridge>(index);
            }
        }
        throw std::invalid_argument("Invalid HalfBridgeBitMask value");
    }
}
