#pragma once

#include <array>
#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/SPI/Api/IDriver.h"
#include "PiSubmarine/GPIO/Api/IDriver.h"
#include "PiSubmarine/Drv8908/IDevice.h"
#include <stdexcept>

namespace PiSubmarine::Drv8908
{


    class Device : public IDevice
    {
    public:
        constexpr static size_t RequiredGpioPinsNum = 2;
        constexpr static size_t NSleepPinIndex = 0;
        constexpr static size_t NFaultPinIndex = 1;
        constexpr static std::array<GPIO::Api::Direction, 2> PinDirections {GPIO::Api::Direction::Output, GPIO::Api::Direction::Input};
        constexpr static size_t HBridgesNum = 8;
        constexpr static size_t PwmGeneratorsNum = 8;

        explicit Device(SPI::Api::IDriver& spiDriver, GPIO::Api::IPinGroup& pinGroup);

        void SetSleeping(bool sleepEnabled) const override;
        [[nodiscard]] bool IsSleeping() const override;
        [[nodiscard]] bool HasFault() const override;

        IcStatus GetStatus(IcStatus& icStat) const override;

        IcStatus GetOpenLoadStatus(OpenLoadStatus& ovp) const override;

        IcStatus GetOvercurrentStatus(OverCurrentStatus& ovp) const override;

        IcStatus GetConfigCtrl(ConfigCtrl& outConfigCtr) const override;

        [[nodiscard]] IcStatus SetConfigCtrl(const ConfigCtrl& inConfigCtr) const override;

        /**
         * @brief Checks if a specific half-bridge is enabled.
         *
         * This function reads the appropriate Operation Control (OP_CTRL) register
         * to determine the status of the high-side and low-side switches for the
         * specified half-bridge.
         *
         * According to the DRV8908 datasheet (Table 50), half-bridges are controlled as follows:
         * - Half-Bridges 1-4 (index 0-3) are in OP_CTRL_1 (0x08).
         * - Half-Bridges 5-8 (index 4-7) are in OP_CTRL_2 (0x09).
         *
         * Each half-bridge 'x' has two enable bits:
         * - HBx_HS_EN (High-Side Enable)
         * - HBx_LS_EN (Low-Side Enable)
         *
         * @param hb The 0-based index of the half-bridge (0-7 for DRV8908).
         * @param[out] high Set to true if the high-side switch is enabled, false otherwise.
         * @param[out] low Set to true if the low-side switch is enabled, false otherwise.
         * @return The IC status read from the SPI transaction. Returns 0 on SPI communication failure.
         */
        IcStatus IsHalfBridgeEnabled(HalfBridge hb, bool& high, bool& low) const override;

        [[nodiscard]] IcStatus SetHalfBridgeEnabled(HalfBridge hb, bool high, bool low) const override;
        [[nodiscard]] IcStatus SetHalfBridgeEnabled(HalfBridgeBitMask hBridges, bool high, bool low) const override;

        [[nodiscard]] IcStatus SetPwmFrequency(PwmGeneratorBitMask generator, PwmFrequency freq) const override;
        [[nodiscard]] IcStatus SetPwmFrequency(PwmGenerator generator, PwmFrequency freq) const override;

        [[nodiscard]] IcStatus GetPwmFrequency(PwmGenerator generator, PwmFrequency& freq) const override;

        [[nodiscard]] IcStatus SetPwmMap(HalfBridge hb, PwmGenerator generator) const override;
        [[nodiscard]] IcStatus SetPwmMap(HalfBridgeBitMask hbMask, PwmGenerator generator) const override;

        [[nodiscard]] IcStatus GetPwmMap(HalfBridge hb, PwmGenerator& generator) const override;

        [[nodiscard]] IcStatus GetDutyCycle(PwmGenerator generator, NormalizedIntFraction<8>& value) const override;

        [[nodiscard]] IcStatus SetDutyCycle(PwmGeneratorBitMask generator, NormalizedIntFraction<8> value) const override;

        [[nodiscard]] IcStatus SetDutyCycle(PwmGenerator generator, NormalizedIntFraction<8> value) const override;

        [[nodiscard]] IcStatus SetHalfBridgePwmModes(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetHalfBridgePwmModes(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus SetHalfBridgeActiveFreeWheeling(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetHalfBridgeActiveFreeWheeling(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus SetHalfBridgeFastSlewRate(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetHalfBridgeFastSlewRate(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus SetEnabledPwmGenerators(PwmGeneratorBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetEnabledPwmGenerators(PwmGeneratorBitMask& channels) const override;

        [[nodiscard]] IcStatus SetEnabledOpenLoadDetect(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetEnabledOpenLoadDetect(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus SetOpenLoadDetectControl2(const OpenLoadDetectControl& value) const override;

        [[nodiscard]] IcStatus GetOpenLoadDetectControl2(OpenLoadDetectControl& value) const override;

        [[nodiscard]] IcStatus SetOpenLoadDetectControl3(OcpDeglitchTime deglitchTime,
                                                         bool negativeCurrentOldEnabled) const override;

        [[nodiscard]] IcStatus GetOpenLoadDetectControl3(OcpDeglitchTime& deglitchTime,
                                                         bool& negativeCurrentOldEnabled) const override;

        [[nodiscard]] IcStatus EnableLowCurrentOpenLoadDetect(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetEnabledLowCurrentOpenLoadDetect(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus EnablePassiveOpenLoadDetect(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetEnabledPassiveOpenLoadDetect(HalfBridgeBitMask& channels) const override;

        [[nodiscard]] IcStatus EnablePassiveVmOpenLoadDetect(HalfBridgeBitMask channelMask) const override;

        [[nodiscard]] IcStatus GetEnabledPassiveVmOpenLoadDetect(HalfBridgeBitMask& channels) const override;

    private:
        SPI::Api::IDriver& m_SpiDriver;
        GPIO::Api::IPinGroup& m_PinGroup;

        template <typename T, typename = std::enable_if_t<sizeof(T) == 1>>
        IcStatus ReadRegister(Register reg, T& regData) const
        {
            using namespace RegUtils;

            std::array<uint8_t, 2> mosiBytes{0};
            mosiBytes[0] = static_cast<uint8_t>(reg);
            mosiBytes[0] |= (1 << 6);

            std::array<uint8_t, 2> misoBytes{0};

            if (!m_SpiDriver.WriteRead(mosiBytes.data(), misoBytes.data(), 2))
            {
                return static_cast<IcStatus>(0);
            }
            const auto status = static_cast<IcStatus>(misoBytes[0]);
            regData = static_cast<T>(misoBytes[1]);
            return status & ~(IcStatus::OverTemperatureShutdown);
        }

        template <typename T, typename = std::enable_if_t<sizeof(T) == 1>>
        IcStatus WriteRegister(Register reg, T dataNew) const
        {
            T dataOld{0};
            return WriteRegister(reg, dataNew, dataOld);
        }

        template <typename T, typename = std::enable_if_t<sizeof(T) == 1>>
        IcStatus WriteRegister(Register reg, T dataNew, T& dataOld) const
        {
            using namespace RegUtils;

            std::array<uint8_t, 2> mosiBytes{0};
            mosiBytes[0] = static_cast<uint8_t>(reg);
            mosiBytes[0] &= ~(1 << 6);
            mosiBytes[1] = static_cast<uint8_t>(dataNew);

            std::array<uint8_t, 2> misoBytes{0};

            if (!m_SpiDriver.WriteRead(mosiBytes.data(), misoBytes.data(), 2))
            {
                return static_cast<IcStatus>(0);
            }
            const auto status = static_cast<IcStatus>(misoBytes[0]);
            dataOld = static_cast<T>(misoBytes[1]);
            return status & ~(IcStatus::OverTemperatureShutdown);
        }
    };
}
