#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"

using namespace ::testing;

namespace PiSubmarine::Drv8908
{
    TEST(HalfBridgeTest, ToIndexConstexpr)
    {
        constexpr auto index = ToHalfBridge(HalfBridgeBitMask::HalfBridge3);
        EXPECT_EQ(static_cast<int>(index), 2);
    }

    TEST(HalfBridgeTest, ToHalfBridgeEnumConstexpr)
    {
        constexpr HalfBridgeBitMask halfBridge = ToHalfBridgeBitMask(HalfBridge::HalfBridge8);
        EXPECT_EQ(halfBridge, HalfBridgeBitMask::HalfBridge8);
    }

    TEST(HalfBridgeTest, ToEnumToIndexEquality)
    {
        for (uint8_t index = 0; index < 8; index++)
        {
            HalfBridgeBitMask hBridge = ToHalfBridgeBitMask(static_cast<HalfBridge>(index));
            HalfBridge expected = ToHalfBridge(hBridge);
            EXPECT_EQ(expected, static_cast<HalfBridge>(index));
        }
    }

    TEST(HalfBridgeTest, ToIndexInvalid)
    {
        using namespace RegUtils;
        EXPECT_ANY_THROW(ToHalfBridge(HalfBridgeBitMask::HalfBridge2 | HalfBridgeBitMask::HalfBridge3));
    }

    TEST(HalfBridgeTest, ToHalfBridgeEnumInvalid)
    {
        EXPECT_ANY_THROW(ToHalfBridgeBitMask(HalfBridge{8}));
    }
}