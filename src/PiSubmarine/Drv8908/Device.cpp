#include "PiSubmarine/Drv8908/Device.h"

namespace PiSubmarine::Drv8908
{
    Device::Device(SPI::Api::IDriver& spiDriver, GPIO::Api::IPinGroup& pinGroup) : m_SpiDriver(spiDriver),
        m_PinGroup(pinGroup)
    {
        if (m_PinGroup.Num() != RequiredGpioPinsNum)
        {
            throw std::invalid_argument(
                "Invalid number of GPIO pins in PinGroup. Must be " + std::to_string(RequiredGpioPinsNum) + ", got "
                + std::to_string(m_PinGroup.Num()));
        }
        GPIO::Api::Directions directions{0};
        directions.Set(NSleepPinIndex, PinDirections[NSleepPinIndex]);
        directions.Set(NFaultPinIndex, PinDirections[NFaultPinIndex]);
        pinGroup.SetDirections(directions);
    }

    void Device::SetSleeping(bool sleepEnabled) const
    {
        GPIO::Api::Level nSleepLevel = sleepEnabled ? GPIO::Api::Level::Low : GPIO::Api::Level::High;
        GPIO::Api::Levels levels{0};
        GPIO::Api::Mask mask{0};
        levels.Set(NSleepPinIndex, nSleepLevel);
        mask.Set(NSleepPinIndex, GPIO::Api::MaskBit::Included);
        m_PinGroup.SetLevels(levels, mask);
    }

    bool Device::IsSleeping() const
    {
        return m_PinGroup.GetLevels().Get(NSleepPinIndex) == GPIO::Api::Level::Low;
    }

    bool Device::HasFault() const
    {
        return m_PinGroup.GetLevels().Get(NFaultPinIndex) == GPIO::Api::Level::Low;
    }

    IcStatus Device::GetStatus(IcStatus& icStat) const
    {
        IcStatus status = ReadRegister(Register::IcStat, icStat);
        return status;
    }

    IcStatus Device::GetOpenLoadStatus(OpenLoadStatus& ovp) const
    {
        using namespace RegUtils;

        uint8_t old1 = 0;
        uint8_t old2 = 0;

        IcStatus status = ReadRegister(Register::OldStat1, old1);
        if (!IsValid(status))
        {
            return IcStatus{0};
        }

        status = ReadRegister(Register::OldStat1, old2);
        ovp = static_cast<OpenLoadStatus>((old2 << 8) + old1);
        return status;
    }

    IcStatus Device::GetOvercurrentStatus(OverCurrentStatus& ovp) const
    {
        using namespace RegUtils;

        uint8_t ocp1 = 0;
        uint8_t ocp2 = 0;

        IcStatus status = ReadRegister(Register::OcpStat1, ocp1);
        if (!IsValid(status))
        {
            return IcStatus{0};
        }
        status = ReadRegister(Register::OcpStat2, ocp2);
        ovp = static_cast<OverCurrentStatus>((ocp2 << 8) + ocp1);
        return status;
    }

    IcStatus Device::GetConfigCtrl(ConfigCtrl& outConfigCtr) const
    {
        using namespace RegUtils;

        uint8_t configCtrlByte = 0;
        IcStatus status = ReadRegister(Register::ConfigCtrl, configCtrlByte);
        if (!IsValid(status))
        {
            return IcStatus{0};
        }

        outConfigCtr = {};
        outConfigCtr.PoldEn = (configCtrlByte & ToInt(ConfigCtrlFields::POLD_EN)) != 0;
        outConfigCtr.Id = static_cast<IcId>((configCtrlByte & 0b01110000) >> 4);
        outConfigCtr.OcpRep = (configCtrlByte & ToInt(ConfigCtrlFields::OCP_REP)) != 0;
        outConfigCtr.OtwRep = (configCtrlByte & ToInt(ConfigCtrlFields::OTW_REP)) != 0;
        outConfigCtr.ExtOvp = (configCtrlByte & ToInt(ConfigCtrlFields::EXT_OVP)) != 0;
        outConfigCtr.ClrFlt = (configCtrlByte & ToInt(ConfigCtrlFields::CLR_FLT)) != 0;
        return status;
    }

    IcStatus Device::SetConfigCtrl(const ConfigCtrl& inConfigCtr) const
    {
        using namespace RegUtils;
        uint8_t configCtrlByte = 0;
        if (inConfigCtr.PoldEn)
        {
            configCtrlByte |= ToInt(ConfigCtrlFields::POLD_EN);
        }
        if (inConfigCtr.OcpRep)
        {
            configCtrlByte |= ToInt(ConfigCtrlFields::OCP_REP);
        }
        if (inConfigCtr.OtwRep)
        {
            configCtrlByte |= ToInt(ConfigCtrlFields::OTW_REP);
        }
        if (inConfigCtr.ExtOvp)
        {
            configCtrlByte |= ToInt(ConfigCtrlFields::EXT_OVP);
        }
        if (inConfigCtr.ClrFlt)
        {
            configCtrlByte |= ToInt(ConfigCtrlFields::CLR_FLT);
        }
        uint8_t dataOld;
        return WriteRegister(Register::ConfigCtrl, configCtrlByte, dataOld);
    }

    IcStatus Device::IsHalfBridgeEnabled(uint8_t hbIndex, bool& high, bool& low) const
    {
        using namespace RegUtils;

        // A half-bridge is considered enabled if either its high-side or low-side FET is enabled.
        // This is controlled by the HBx_HS_EN and HBx_LS_EN bits in the OP_CTRL registers.

        // Determine which Operation Control register to read based on the half-bridge index.
        Register regAddr;
        if (hbIndex < 4)
        {
            // Half-bridges 0-3 are in OP_CTRL_1
            regAddr = Register::OpCtrl1;
        }
        else
        {
            // Half-bridges 4-7 are in OP_CTRL_2
            regAddr = Register::OpCtrl2;
        }

        // Read the content of the determined register.
        uint8_t regData = 0;
        const IcStatus status = ReadRegister(regAddr, regData);

        if ((status & IcStatus::TestBit) == 0)
        {
            return static_cast<IcStatus>(0);
        }

        // Calculate the bit offset for the specified half-bridge within the 8-bit register.
        // Each half-bridge uses 2 bits. The modulo operator maps indices 0-3 and 4-7 to 0-3.
        const uint8_t bitOffset = (hbIndex % 4) * 2;

        // Isolate and check the high-side and low-side enable bits.
        // The high-side bit is the most significant bit of the pair.
        high = (regData & (0b10 << bitOffset)) != 0;
        // The low-side bit is the least significant bit of the pair.
        low = (regData & (0b01 << bitOffset)) != 0;

        return status;
    }

    IcStatus Device::SetHalfBridgeEnabled(uint8_t hbIndex, bool high, bool low) const
    {
        using namespace RegUtils;

        Register reg;
        if (hbIndex < 4)
        {
            reg = Register::OpCtrl1;
        }
        else
        {
            reg = Register::OpCtrl2;
        }

        uint8_t regData = 0;
        IcStatus status = ReadRegister(reg, regData);
        if ((status & (IcStatus::TestBit)) == 0)
        {
            return static_cast<IcStatus>(0);
        }

        const uint8_t bitOffset = (hbIndex % 4) * 2;

        regData &= ~(0b11 << bitOffset);

        if (high)
        {
            regData |= (0b10 << bitOffset);
        }

        if (low)
        {
            regData |= (0b01 << bitOffset);
        }

        uint8_t regOld = 0;
        status = WriteRegister(reg, regData, regOld);

        return status;
    }

    IcStatus Device::SetHalfBridgeEnabled(HalfBridge hBridges, bool high, bool low) const
    {
        IcStatus status{0};
        using namespace RegUtils;
        for (uint8_t channel = 0; channel < 8; channel++)
        {
            auto mask = static_cast<HalfBridge>(1 << channel);
            if ((hBridges & mask) != 0)
            {
                status = SetHalfBridgeEnabled(channel, high, low);
                if (!IsValid(status))
                {
                    return IcStatus{0};
                }
            }
        }
        return status;
    }

    IcStatus Device::SetPwmFrequency(PwmGenerator generator, PwmFrequency freq) const
    {
        IcStatus status{0};
        using namespace RegUtils;
        for (uint8_t channel = 0; channel < 8; channel++)
        {
            PwmGenerator generatorMask = static_cast<PwmGenerator>(1 << channel);
            if ((generator & generatorMask) != 0)
            {
                status = SetPwmFrequency(channel, freq);
                if (!IsValid(status))
                {
                    return IcStatus{0};
                }
            }
        }
        return status;
    }

    IcStatus Device::SetPwmFrequency(uint8_t channel, PwmFrequency freq) const
    {
        using namespace RegUtils;

        Register reg;
        if (channel < 4)
        {
            reg = Register::PwmFreqCtrl1;
        }
        else
        {
            reg = Register::PwmFreqCtrl2;
        }

        const uint8_t bitOffset = (channel % 4) * 2;
        uint8_t regData = 0;
        IcStatus status = ReadRegister(reg, regData);
        if ((status & (IcStatus::TestBit)) == 0)
        {
            return static_cast<IcStatus>(0);
        }

        regData &= ~(0b11 << bitOffset);
        regData |= static_cast<uint8_t>(freq) << bitOffset;
        uint8_t regOld = 0;
        status = WriteRegister(reg, regData, regOld);

        return status;
    }

    IcStatus Device::GetPwmFrequency(uint8_t channel, PwmFrequency& freq) const
    {
        using namespace RegUtils;

        Register reg;
        if (channel < 4)
        {
            reg = Register::PwmFreqCtrl1;
        }
        else
        {
            reg = Register::PwmFreqCtrl2;
        }

        const uint8_t bitOffset = (channel % 4) * 2;

        uint8_t regData = 0;
        IcStatus status = ReadRegister(reg, regData);
        if ((status & (IcStatus::TestBit)) == 0)
        {
            return static_cast<IcStatus>(0);
        }

        freq = static_cast<PwmFrequency>((regData >> bitOffset) & 0b11);
        return status;
    }

    IcStatus Device::SetPwmMap(uint8_t halfBridgeIndex, uint8_t pwmChannel) const
    {
        using namespace RegUtils;

        const uint8_t regShift = halfBridgeIndex / 2;
        const auto reg = static_cast<Register>(static_cast<uint8_t>(Register::PwmMapCtrl1) + regShift);
        const uint8_t bitOffset = (halfBridgeIndex % 2) * 3;

        uint8_t regData = 0;
        IcStatus status = ReadRegister(reg, regData);
        if (!IsValid(status))
        {
            return IcStatus{0};
        }

        regData &= ~(0b111 << bitOffset);
        regData |= pwmChannel << bitOffset;
        status = WriteRegister(reg, regData);
        return status;
    }

    IcStatus Device::GetPwmMap(uint8_t halfBridgeIndex, uint8_t& pwmChannel) const
    {
        using namespace RegUtils;

        const uint8_t regShift = halfBridgeIndex / 2;
        const auto reg = static_cast<Register>(static_cast<uint8_t>(Register::PwmMapCtrl1) + regShift);
        const uint8_t bitOffset = (halfBridgeIndex % 2) * 3;

        uint8_t regData = 0;
        const IcStatus status = ReadRegister(reg, regData);
        if ((status & (IcStatus::TestBit)) == 0)
        {
            return static_cast<IcStatus>(0);
        }

        pwmChannel = ((regData >> bitOffset) & 0b111);
        return status;
    }

    IcStatus Device::GetDutyCycle(uint8_t channel, uint8_t& value) const
    {
        const auto reg = static_cast<Register>(static_cast<uint8_t>(Register::PwmDutyCtrl1) + channel);
        const auto status = ReadRegister(reg, value);
        return status;
    }

    IcStatus Device::SetDutyCycle(PwmGenerator generator, uint8_t value) const
    {
        IcStatus status{0};
        using namespace RegUtils;
        for (uint8_t channel = 0; channel < 8; channel++)
        {
            PwmGenerator generatorMask = static_cast<PwmGenerator>(1 << channel);
            if ((generator & generatorMask) != 0)
            {
                status = SetDutyCycle(channel, value);
                if (!IsValid(status))
                {
                    return IcStatus{0};
                }
            }
        }
        return status;
    }

    IcStatus Device::SetDutyCycle(uint8_t channel, uint8_t value) const
    {
        const auto reg = static_cast<Register>(static_cast<uint8_t>(Register::PwmDutyCtrl1) + channel);
        return WriteRegister(reg, value);;
    }

    IcStatus Device::SetHalfBridgePwmModes(HalfBridge channelMask) const
    {
        auto reg = Register::PwmCtrl1;
        uint8_t regData = RegUtils::ToInt(channelMask);
        auto status = WriteRegister(reg, regData);
        return status;
    }

    IcStatus Device::GetHalfBridgePwmModes(HalfBridge& channels) const
    {
        return ReadRegister(Register::PwmCtrl1, channels);
    }

    IcStatus Device::SetHalfBridgeActiveFreeWheeling(HalfBridge channelMask) const
    {
        return WriteRegister(Register::FwCtrl1, channelMask);
    }

    IcStatus Device::GetHalfBridgeActiveFreeWheeling(HalfBridge& channels) const
    {
        return ReadRegister(Register::FwCtrl1, channels);
    }

    IcStatus Device::SetHalfBridgeFastSlewRate(HalfBridge channelMask) const
    {
        return WriteRegister(Register::SrCtrl1, channelMask);
    }

    IcStatus Device::GetHalfBridgeFastSlewRate(HalfBridge& channels) const
    {
        return ReadRegister(Register::SrCtrl1, channels);
    }

    IcStatus Device::SetEnabledPwmGenerators(PwmGenerator channelMask) const
    {
        using namespace RegUtils;
        PwmGenerator oldMask;
        // 0 - PWM enabled, need to inverse
        return WriteRegister(Register::PwmCtrl2, ~channelMask, oldMask);
    }

    IcStatus Device::GetEnabledPwmGenerators(PwmGenerator& channels) const
    {
        using namespace RegUtils;
        PwmGenerator disabledGenerators{0};
        auto status = ReadRegister(Register::PwmCtrl2, disabledGenerators);
        channels = ~disabledGenerators;
        return status;
    }

    IcStatus Device::SetEnabledOpenLoadDetect(HalfBridge channelMask) const
    {
        using namespace RegUtils;
        return WriteRegister(Register::OldCtrl1, ~channelMask);
    }

    IcStatus Device::GetEnabledOpenLoadDetect(HalfBridge& channels) const
    {
        using namespace RegUtils;
        HalfBridge disabledOld{0};
        auto status = ReadRegister(Register::OldCtrl1, disabledOld);
        channels = ~disabledOld;
        return status;
    }

    IcStatus Device::SetOpenLoadDetectControl2(const OpenLoadDetectControl& value) const
    {
        OpenLoadDetectControl old;
        return WriteRegister(Register::OldCtrl2, value, old);
    }

    IcStatus Device::GetOpenLoadDetectControl2(OpenLoadDetectControl& value) const
    {
        return ReadRegister(Register::OldCtrl2, value);
    }

    IcStatus Device::SetOpenLoadDetectControl3(OcpDeglitchTime deglitchTime, bool negativeCurrentOldEnabled) const
    {
        uint8_t oldData;
        uint8_t regData = RegUtils::ToInt(deglitchTime) << 5;
        if (negativeCurrentOldEnabled)
        {
            regData |= (1 << 4);
        }
        return WriteRegister(Register::OldCtrl3, regData, oldData);
    }

    IcStatus Device::GetOpenLoadDetectControl3(OcpDeglitchTime& deglitchTime, bool& negativeCurrentOldEnabled) const
    {
        uint8_t regData;
        auto status = ReadRegister(Register::OldCtrl3, regData);
        if (!IsValid(status))
        {
            return IcStatus{0};
        }

        deglitchTime = static_cast<OcpDeglitchTime>(regData >> 5);
        if (regData & (1 << 4))
        {
            negativeCurrentOldEnabled = true;
        }
        return status;
    }

    IcStatus Device::EnableLowCurrentOpenLoadDetect(HalfBridge channelMask) const
    {
        HalfBridge oldMask;
        return WriteRegister(Register::OldCtrl4, channelMask, oldMask);
    }

    IcStatus Device::GetEnabledLowCurrentOpenLoadDetect(HalfBridge& channels) const
    {
        return ReadRegister(Register::OldCtrl4, channels);
    }

    IcStatus Device::EnablePassiveOpenLoadDetect(HalfBridge channelMask) const
    {
        HalfBridge oldMask;
        return WriteRegister(Register::OldCtrl5, channelMask, oldMask);
    }

    IcStatus Device::GetEnabledPassiveOpenLoadDetect(HalfBridge& channels) const
    {
        return ReadRegister(Register::OldCtrl5, channels);
    }

    IcStatus Device::EnablePassiveVmOpenLoadDetect(HalfBridge channelMask) const
    {
        HalfBridge oldMask;
        return WriteRegister(Register::OldCtrl6, channelMask, oldMask);
    }

    IcStatus Device::GetEnabledPassiveVmOpenLoadDetect(HalfBridge& channels) const
    {
        using namespace RegUtils;
        return ReadRegister(Register::OldCtrl6, channels);
    }
}
