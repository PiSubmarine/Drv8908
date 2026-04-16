#pragma once

#include <gmock/gmock.h>
#include <span>
#include "PiSubmarine/SPI/Api/IDriver.h"

namespace PiSubmarine::Drv8908
{
    class MockSpiDriver : public SPI::Api::IDriver
    {
    public:
        MOCK_METHOD(bool, WriteRead,
                    (std::span<const uint8_t> txData, std::span<uint8_t> rxData),
                    (override));
    };
}
