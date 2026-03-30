#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/PwmGeneratorBitMask.h"

using namespace ::testing;

namespace PiSubmarine::Drv8908
{
    TEST(PwmGeneratorTest, ToIndexConstexpr)
    {
        constexpr auto generator = ToPwnGenerator(PwmGeneratorBitMask::PwmGenerator3);
        EXPECT_EQ(generator, PwmGenerator::PwmGenerator3);
    }

    TEST(PwmGeneratorTest, ToPwmGeneratorBitMaskConstexpr)
    {
        constexpr PwmGeneratorBitMask pwmGenerator = ToPwmGeneratorBitMask(PwmGenerator::PwmGenerator8);
        EXPECT_EQ(pwmGenerator, PwmGeneratorBitMask::PwmGenerator8);
    }

    TEST(PwmGeneratorTest, ToEnumToIndexEquality)
    {
        for (uint8_t index = 0; index < 8; index++)
        {
            PwmGeneratorBitMask enumValue = ToPwmGeneratorBitMask(static_cast<PwmGenerator>(index));
            PwmGenerator expected = ToPwnGenerator(enumValue);
            EXPECT_EQ(expected, static_cast<PwmGenerator>(index));
        }
    }

    TEST(PwmGeneratorTest, ToIndexInvalid)
    {
        using namespace RegUtils;
        EXPECT_ANY_THROW(ToPwnGenerator(PwmGeneratorBitMask::PwmGenerator2 | PwmGeneratorBitMask::PwmGenerator3));
    }

    TEST(PwmGeneratorTest, ToPwmGeneratorBitMaskInvalid)
    {
        EXPECT_ANY_THROW(ToPwmGeneratorBitMask(PwmGenerator{8}));
    }
}