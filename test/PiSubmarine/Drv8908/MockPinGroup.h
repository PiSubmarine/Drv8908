#pragma once

#include <gmock/gmock.h>
#include "PiSubmarine/GPIO/Api/IPinGroup.h"

namespace PiSubmarine::Drv8908
{
    class MockPinGroup : public GPIO::Api::IPinGroup
    {
    public:
        MOCK_METHOD(GPIO::Api::Directions, GetDirections, (), (override));
        MOCK_METHOD(void, SetDirections, (GPIO::Api::Directions directions), (override));

        MOCK_METHOD(GPIO::Api::Levels, GetLevels, (), (override));
        MOCK_METHOD(void, SetLevels, (GPIO::Api::Levels levels, GPIO::Api::Mask mask), (override));

        MOCK_METHOD(size_t, Num, (), (const, override));
    };
}
