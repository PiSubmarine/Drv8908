#pragma once
#include <cstdint>
#include "PiSubmarine/RegUtils.h"

namespace PiSubmarine::Drv8908
{
    enum class PwmGenerator : uint8_t
    {
        PwmGenerator1 =  0,
        PwmGenerator2,
        PwmGenerator3,
        PwmGenerator4,
        PwmGenerator5,
        PwmGenerator6,
        PwmGenerator7,
        PwmGenerator8
    };
}
