#include <gtest/gtest.h>
#include "PiSubmarine/Drv8908/Device.h"
#include "PiSubmarine/Drv8908/MockPinGroup.h"
#include "PiSubmarine/Drv8908/MockSpiDriver.h"
#include <chrono>

using namespace std::chrono_literals;
using namespace ::testing;

namespace PiSubmarine::Drv8908
{
	TEST(Drv8908Test, Construct)
	{
		GPIO::Api::Directions directions{0};
		directions.Set(Device::NFaultPinIndex, Device::PinDirections[Device::NFaultPinIndex]);
		directions.Set(Device::NSleepPinIndex, Device::PinDirections[Device::NSleepPinIndex]);

		MockPinGroup pinGroup;
		MockSpiDriver spiDriver;

		EXPECT_CALL(pinGroup, Num()).WillOnce(Return(2));
		EXPECT_CALL(pinGroup, SetDirections(directions));

		Device device(spiDriver, pinGroup);


	}

	TEST(Drv8908Test, IsSleeping)
	{
		GPIO::Api::Directions directions{0};
		directions.Set(Device::NFaultPinIndex, Device::PinDirections[Device::NFaultPinIndex]);
		directions.Set(Device::NSleepPinIndex, Device::PinDirections[Device::NSleepPinIndex]);

		MockPinGroup pinGroup;
		MockSpiDriver spiDriver;

		EXPECT_CALL(pinGroup, Num()).WillOnce(Return(2));
		EXPECT_CALL(pinGroup, SetDirections(directions));

		Device device(spiDriver, pinGroup);

		EXPECT_CALL(pinGroup, GetLevels()).WillOnce(Return(GPIO::Api::Levels(0b00)));
		EXPECT_TRUE(device.IsSleeping());

		EXPECT_CALL(pinGroup, GetLevels()).WillOnce(Return(GPIO::Api::Levels(0b11)));
		EXPECT_FALSE(device.IsSleeping());

	}

}