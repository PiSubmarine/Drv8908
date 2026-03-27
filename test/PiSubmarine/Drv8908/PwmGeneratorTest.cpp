#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/PwmGenerator.h"

using namespace ::testing;

namespace PiSubmarine::Drv8908
{
    TEST(PwmGeneratorTest, ToIndexConstexpr)
    {
        constexpr auto index = ToIndex(PwmGenerator::PwmGenerator3);
        EXPECT_EQ(index, 2);
    }

    TEST(PwmGeneratorTest, ToPwmGeneratorEnumConstexpr)
    {
        constexpr PwmGenerator pwmGenerator = ToPwmGeneratorEnum(7);
        EXPECT_EQ(pwmGenerator, PwmGenerator::PwmGenerator8);
    }

    TEST(PwmGeneratorTest, ToEnumToIndexEquality)
    {
        for (uint8_t index = 0; index < 8; index++)
        {
            PwmGenerator enumValue = ToPwmGeneratorEnum(index);
            uint8_t expected = ToIndex(enumValue);
            EXPECT_EQ(expected, index);
        }
    }

    TEST(PwmGeneratorTest, ToIndexInvalid)
    {
        using namespace RegUtils;
        EXPECT_ANY_THROW(ToIndex(PwmGenerator::PwmGenerator2 | PwmGenerator::PwmGenerator3));
    }

    TEST(PwmGeneratorTest, ToPwmGeneratorEnumInvalid)
    {
        EXPECT_ANY_THROW(ToPwmGeneratorEnum(8));
    }
}