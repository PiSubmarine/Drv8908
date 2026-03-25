#pragma once

#include <gmock/gmock.h>
#include "PiSubmarine/SPI/Api/IDriver.h"

namespace PiSubmarine::Drv8908
{
    class MockSpiDriver : public SPI::Api::IDriver
    {
    public:
        MOCK_METHOD(bool, WriteRead,
                    (uint8_t* txData, uint8_t* rxData, std::size_t len),
                    (override));
    };
}
