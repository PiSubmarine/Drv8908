#pragma once

#include <stdexcept>
#include <string>
#include <array>
#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/SPI/Api/IDriver.h"
#include "PiSubmarine/Drv8908/Register.h"
#include "PiSubmarine/GPIO/Api/IDriver.h"
#include "PiSubmarine/Drv8908/PwmGenerator.h"
#include "PiSubmarine/Drv8908/HalfBridge.h"

namespace PiSubmarine::Drv8908
{
    enum class IcStatus : uint8_t
    {
        NoPowerOnReset = (1 << 0),
        OverVoltage = (1 << 1),
        UnderVoltage = (1 << 2),
        OverCurrent = (1 << 3),
        OpenLoad = (1 << 4),
        OverTemperatureWarning = (1 << 5),
        OverTemperatureShutdown = (1 << 6),
        TestBit = (1 << 7) // This bit it reserved, so can be used by the code to report read/write errors.
    };

    enum class OverCurrentStatus
    {
        HB1_LS_OCP = (1 << 0),
        HB1_HS_OCP = (1 << 1),
        HB2_LS_OCP = (1 << 2),
        HB2_HS_OCP = (1 << 3),
        HB3_LS_OCP = (1 << 4),
        HB3_HS_OCP = (1 << 5),
        HB4_LS_OCP = (1 << 6),
        HB4_HS_OCP = (1 << 7),
        HB5_LS_OCP = (1 << 8),
        HB5_HS_OCP = (1 << 9),
        HB6_LS_OCP = (1 << 10),
        HB6_HS_OCP = (1 << 11),
        HB7_LS_OCP = (1 << 12),
        HB7_HS_OCP = (1 << 13),
        HB8_LS_OCP = (1 << 14),
        HB8_HS_OCP = (1 << 15)
    };

    enum class OpenLoadStatus
    {
        HB1_LS_OLD = (1 << 0),
        HB1_HS_OLD = (1 << 1),
        HB2_LS_OLD = (1 << 2),
        HB2_HS_OLD = (1 << 3),
        HB3_LS_OLD = (1 << 4),
        HB3_HS_OLD = (1 << 5),
        HB4_LS_OLD = (1 << 6),
        HB4_HS_OLD = (1 << 7),
        HB5_LS_OLD = (1 << 8),
        HB5_HS_OLD = (1 << 9),
        HB6_LS_OLD = (1 << 10),
        HB6_HS_OLD = (1 << 11),
        HB7_LS_OLD = (1 << 12),
        HB7_HS_OLD = (1 << 13),
        HB8_LS_OLD = (1 << 14),
        HB8_HS_OLD = (1 << 15)
    };

    enum class ConfigCtrlFields : uint8_t
    {
        CLR_FLT = 1 << 0,
        EXT_OVP = 1 << 1,
        OTW_REP = 1 << 2,
        OCP_REP = 1 << 3,
        IC_ID = 1 << 4,
        POLD_EN = 1 << 7 // Passive OLD
    };

    enum class IcId : uint8_t
    {
        DRV8912 = 0b000,
        DRV8910 = 0b001,
        DRV8908 = 0b010,
        DRV8906 = 0b011,
        DRV8904 = 0b100
    };

    enum class OpenLoadDetectControl : uint8_t
    {
        // 0 - 3 reserved on DRV8908
        PlModeEn = 1 << 4,
        OldOp = 1 << 6,
        OldRep = 1 << 7
    };

    enum class OcpDeglitchTime : uint8_t
    {
        MicroSeconds10 = 0,
        MicroSeconds5,
        MicroSeconds2_5,
        MicroSeconds1,
        MicroSeconds60,
        MicroSeconds40,
        MicroSeconds30,
        MicroSeconds20
    };

    struct ConfigCtrl
    {
        bool PoldEn; // Passive OLD
        IcId Id;
        bool OcpRep; // 0 - Overcurrent reported in nFAULT pin, 1 - overcurrent NOT reported in nFAULT
        bool OtwRep; // 0 - Overtemperature NOT reported in nFAULT pin, 1 - Overtemperature reported in nFAULT pin
        bool ExtOvp; // 0 - 21 Volts OVP, 1 - 33 Volts OVP
        bool ClrFlt; // Write 1 to clear faults. Always reads as 0.
    };

    enum class PwmFrequency
    {
        Hz80,
        Hz100,
        Hz200,
        Hz2000
    };

    constexpr bool IsValid(IcStatus status)
    {
        using namespace RegUtils;
        return (status & IcStatus::TestBit) == 0;
    }

    class Device
    {
    public:
        constexpr static size_t RequiredGpioPinsNum = 2;
        constexpr static size_t NFaultPinIndex = 0;
        constexpr static size_t NSleepPinIndex = 1;
        constexpr static std::array<GPIO::Api::Direction, 2> PinDirections {GPIO::Api::Direction::Input, GPIO::Api::Direction::Output};

        explicit Device(SPI::Api::IDriver& spiDriver, GPIO::Api::IPinGroup& pinGroup);

        void SetSleeping(bool sleepEnabled) const;
        [[nodiscard]] bool IsSleeping() const;
        [[nodiscard]] bool HasFault() const;

        IcStatus GetOpenLoadStatus(OverCurrentStatus& ovp) const;

        IcStatus GetOvercurrentStatus(OverCurrentStatus& ovp) const;

        IcStatus GetConfigCtrl(ConfigCtrl& outConfigCtr) const;

        [[nodiscard]] IcStatus SetConfigCtrl(const ConfigCtrl& inConfigCtr) const;

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
         * @param hbIndex The 0-based index of the half-bridge (0-7 for DRV8908).
         * @param[out] high Set to true if the high-side switch is enabled, false otherwise.
         * @param[out] low Set to true if the low-side switch is enabled, false otherwise.
         * @return The IC status read from the SPI transaction. Returns 0 on SPI communication failure.
         */
        IcStatus IsHalfBridgeEnabled(uint8_t hbIndex, bool& high, bool& low) const;

        [[nodiscard]] IcStatus SetHalfBridgeEnabled(uint8_t hbIndex, bool high, bool low) const;

        [[nodiscard]] IcStatus SetPwmFrequency(uint8_t channel, PwmFrequency freq) const;

        [[nodiscard]] IcStatus GetPwmFrequency(uint8_t channel, PwmFrequency& freq) const;

        [[nodiscard]] IcStatus SetPwmMap(uint8_t halfBridgeIndex, uint8_t pwmChannel) const;

        [[nodiscard]] IcStatus GetPwmMap(uint8_t halfBridgeIndex, uint8_t& pwmChannel) const;

        [[nodiscard]] IcStatus GetDutyCycle(uint8_t channel, uint8_t& value) const;

        [[nodiscard]] IcStatus SetDutyCycle(uint8_t channel, uint8_t value) const;

        [[nodiscard]] IcStatus SetHalfBridgePwmModes(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetHalfBridgePwmModes(HalfBridge& channels) const;

        [[nodiscard]] IcStatus SetHalfBridgeActiveFreeWheeling(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetHalfBridgeActiveFreeWheeling(HalfBridge& channels) const;

        [[nodiscard]] IcStatus SetHalfBridgeFastSlewRate(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetHalfBridgeFastSlewRate(HalfBridge& channels) const;

        [[nodiscard]] IcStatus EnablePwmGenerators(PwmGenerator channelMask) const;

        [[nodiscard]] IcStatus GetEnabledPwmGenerators(PwmGenerator& channels) const;

        [[nodiscard]] IcStatus DisableOpenLoadDetect(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetDisabledOpenLoadDetect(HalfBridge& channels) const;

        [[nodiscard]] IcStatus SetOpenLoadDetectControl2(const OpenLoadDetectControl& value) const;

        [[nodiscard]] IcStatus GetOpenLoadDetectControl2(OpenLoadDetectControl& value) const;

        [[nodiscard]] IcStatus SetOpenLoadDetectControl3(OcpDeglitchTime deglitchTime,
                                                         bool negativeCurrentOldEnabled) const;

        [[nodiscard]] IcStatus GetOpenLoadDetectControl3(OcpDeglitchTime& deglitchTime,
                                                         bool& negativeCurrentOldEnabled) const;

        [[nodiscard]] IcStatus EnableLowCurrentOpenLoadDetect(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetEnabledLowCurrentOpenLoadDetect(HalfBridge& channels) const;

        [[nodiscard]] IcStatus EnablePassiveOpenLoadDetect(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetEnabledPassiveOpenLoadDetect(HalfBridge& channels) const;

        [[nodiscard]] IcStatus EnablePassiveVmOpenLoadDetect(HalfBridge channelMask) const;

        [[nodiscard]] IcStatus GetEnabledPassiveVmOpenLoadDetect(HalfBridge& channels) const;

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
            return status | IcStatus::TestBit;
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
            return status | IcStatus::TestBit;
        }
    };
}
