#pragma once

#include <gmock/gmock.h>
#include "PiSubmarine/Drv8908/IDevice.h"

namespace PiSubmarine::Drv8908
{
    class IDeviceMock : public IDevice
    {
    public:
        MOCK_METHOD(void, SetSleeping, (bool sleepEnabled), (const, override));

        MOCK_METHOD(bool, IsSleeping, (), (const, override));

        MOCK_METHOD(bool, HasFault, (), (const, override));

        MOCK_METHOD(IcStatus, GetStatus, (IcStatus& icStat), (const, override));

        MOCK_METHOD(IcStatus, GetOpenLoadStatus, (OpenLoadStatus& ovp), (const, override));

        MOCK_METHOD(IcStatus, GetOvercurrentStatus, (OverCurrentStatus& ovp), (const, override));

        MOCK_METHOD(IcStatus, GetConfigCtrl, (ConfigCtrl& outConfigCtr), (const, override));

        MOCK_METHOD(IcStatus, SetConfigCtrl, (const ConfigCtrl& inConfigCtr), (const, override));

        MOCK_METHOD(IcStatus, IsHalfBridgeEnabled, (HalfBridge hb, bool& high, bool& low), (const, override));

        MOCK_METHOD(IcStatus, SetHalfBridgeEnabled, (HalfBridge hb, bool high, bool low), (const, override));

        MOCK_METHOD(IcStatus, SetHalfBridgeEnabled, (HalfBridgeBitMask hBridges, bool high, bool low),
                    (const, override));

        MOCK_METHOD(IcStatus, SetPwmFrequency, (PwmGeneratorBitMask generator, PwmFrequency freq), (const, override));

        MOCK_METHOD(IcStatus, SetPwmFrequency, (PwmGenerator generator, PwmFrequency freq), (const, override));

        MOCK_METHOD(IcStatus, GetPwmFrequency, (PwmGenerator generator, PwmFrequency& freq), (const, override));

        MOCK_METHOD(IcStatus, SetPwmMap, (HalfBridge hb, PwmGenerator generator), (const, override));

        MOCK_METHOD(IcStatus, SetPwmMap, (HalfBridgeBitMask hbMask, PwmGenerator generator), (const, override));

        MOCK_METHOD(IcStatus, GetPwmMap, (HalfBridge hb, PwmGenerator& generator), (const, override));

        MOCK_METHOD(IcStatus, GetDutyCycle, (PwmGenerator generator, NormalizedIntFraction<8>& value),
                    (const, override));

        MOCK_METHOD(IcStatus, SetDutyCycle, (PwmGeneratorBitMask generator, NormalizedIntFraction<8> value),
                    (const, override));

        MOCK_METHOD(IcStatus, SetDutyCycle, (PwmGenerator generator, NormalizedIntFraction<8> value),
                    (const, override));

        MOCK_METHOD(IcStatus, SetHalfBridgePwmModes, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetHalfBridgePwmModes, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, SetHalfBridgeActiveFreeWheeling, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetHalfBridgeActiveFreeWheeling, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, SetHalfBridgeFastSlewRate, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetHalfBridgeFastSlewRate, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, SetEnabledPwmGenerators, (PwmGeneratorBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetEnabledPwmGenerators, (PwmGeneratorBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, SetEnabledOpenLoadDetect, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetEnabledOpenLoadDetect, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, SetOpenLoadDetectControl2, (const OpenLoadDetectControl& value), (const, override));

        MOCK_METHOD(IcStatus, GetOpenLoadDetectControl2, (OpenLoadDetectControl& value), (const, override));

        MOCK_METHOD(IcStatus, SetOpenLoadDetectControl3, (OcpDeglitchTime deglitchTime, bool negativeCurrentOldEnabled),
                    (const, override));

        MOCK_METHOD(IcStatus, GetOpenLoadDetectControl3,
                    (OcpDeglitchTime& deglitchTime, bool& negativeCurrentOldEnabled), (const, override));

        MOCK_METHOD(IcStatus, EnableLowCurrentOpenLoadDetect, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetEnabledLowCurrentOpenLoadDetect, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, EnablePassiveOpenLoadDetect, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetEnabledPassiveOpenLoadDetect, (HalfBridgeBitMask& channels), (const, override));

        MOCK_METHOD(IcStatus, EnablePassiveVmOpenLoadDetect, (HalfBridgeBitMask channelMask), (const, override));

        MOCK_METHOD(IcStatus, GetEnabledPassiveVmOpenLoadDetect, (HalfBridgeBitMask& channels), (const, override));
    };
}
