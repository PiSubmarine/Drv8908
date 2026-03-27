#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/HalfBridge.h"

using namespace ::testing;

namespace PiSubmarine::Drv8908
{
    TEST(HalfBridgeTest, ToIndexConstexpr)
    {
        constexpr auto index = ToIndex(HalfBridge::HalfBridge3);
        EXPECT_EQ(index, 2);
    }

    TEST(HalfBridgeTest, ToHalfBridgeEnumConstexpr)
    {
        constexpr HalfBridge halfBridge = ToHalfBridgeEnum(7);
        EXPECT_EQ(halfBridge, HalfBridge::HalfBridge8);
    }

    TEST(HalfBridgeTest, ToEnumToIndexEquality)
    {
        for (uint8_t index = 0; index < 8; index++)
        {
            HalfBridge hBridge = ToHalfBridgeEnum(index);
            uint8_t expected = ToIndex(hBridge);
            EXPECT_EQ(expected, index);
        }
    }

    TEST(HalfBridgeTest, ToIndexInvalid)
    {
        using namespace RegUtils;
        EXPECT_ANY_THROW(ToIndex(HalfBridge::HalfBridge2 | HalfBridge::HalfBridge3));
    }

    TEST(HalfBridgeTest, ToHalfBridgeEnumInvalid)
    {
        EXPECT_ANY_THROW(ToHalfBridgeEnum(8));
    }
}