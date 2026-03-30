#pragma once
#include <cstdint>
#include "PiSubmarine/RegUtils.h"

namespace PiSubmarine::Drv8908
{
    enum class HalfBridge : uint8_t
    {
        HalfBridge1 = 0,
        HalfBridge2,
        HalfBridge3,
        HalfBridge4,
        HalfBridge5,
        HalfBridge6,
        HalfBridge7,
        HalfBridge8
    };

}
