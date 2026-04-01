#pragma once

#include "PiSubmarine/Drv8908/Register.h"
#include "PiSubmarine/Drv8908/PwmGeneratorBitMask.h"
#include "PiSubmarine/Drv8908/HalfBridgeBitMask.h"
#include "PiSubmarine/NormalizedIntFraction.h"

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
        return (status & IcStatus::TestBit) != 0;
    }

    class IDevice
    {
    public:
        virtual ~IDevice() = default;

        virtual void SetSleeping(bool sleepEnabled) const = 0;
        
        [[nodiscard]] virtual bool IsSleeping() const = 0;
        
        [[nodiscard]] virtual bool HasFault() const = 0;

        virtual IcStatus GetStatus(IcStatus& icStat) const = 0;

        virtual IcStatus GetOpenLoadStatus(OpenLoadStatus& ovp) const = 0;

        virtual IcStatus GetOvercurrentStatus(OverCurrentStatus& ovp) const = 0;

        virtual IcStatus GetConfigCtrl(ConfigCtrl& outConfigCtr) const = 0;

        [[nodiscard]] virtual IcStatus SetConfigCtrl(const ConfigCtrl& inConfigCtr) const = 0;

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
        virtual IcStatus IsHalfBridgeEnabled(HalfBridge hb, bool& high, bool& low) const = 0;

        [[nodiscard]] virtual IcStatus SetHalfBridgeEnabled(HalfBridge hb, bool high, bool low) const = 0;
        
        [[nodiscard]] virtual IcStatus SetHalfBridgeEnabled(HalfBridgeBitMask hBridges, bool high, bool low) const = 0;

        [[nodiscard]] virtual IcStatus SetPwmFrequency(PwmGeneratorBitMask generator, PwmFrequency freq) const = 0;
        
        [[nodiscard]] virtual IcStatus SetPwmFrequency(PwmGenerator generator, PwmFrequency freq) const = 0;

        [[nodiscard]] virtual IcStatus GetPwmFrequency(PwmGenerator generator, PwmFrequency& freq) const = 0;

        [[nodiscard]] virtual IcStatus SetPwmMap(HalfBridge hb, PwmGenerator generator) const = 0;
        
        [[nodiscard]] virtual IcStatus SetPwmMap(HalfBridgeBitMask hbMask, PwmGenerator generator) const = 0;

        [[nodiscard]] virtual IcStatus GetPwmMap(HalfBridge hb, PwmGenerator& generator) const = 0;

        [[nodiscard]] virtual IcStatus GetDutyCycle(PwmGenerator generator, NormalizedIntFraction<8>& value) const = 0;

        [[nodiscard]] virtual IcStatus SetDutyCycle(PwmGeneratorBitMask generator, NormalizedIntFraction<8> value) const = 0;

        [[nodiscard]] virtual IcStatus SetDutyCycle(PwmGenerator generator, NormalizedIntFraction<8> value) const = 0;

        [[nodiscard]] virtual IcStatus SetHalfBridgePwmModes(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetHalfBridgePwmModes(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus SetHalfBridgeActiveFreeWheeling(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetHalfBridgeActiveFreeWheeling(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus SetHalfBridgeFastSlewRate(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetHalfBridgeFastSlewRate(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus SetEnabledPwmGenerators(PwmGeneratorBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetEnabledPwmGenerators(PwmGeneratorBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus SetEnabledOpenLoadDetect(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetEnabledOpenLoadDetect(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus SetOpenLoadDetectControl2(const OpenLoadDetectControl& value) const = 0;

        [[nodiscard]] virtual IcStatus GetOpenLoadDetectControl2(OpenLoadDetectControl& value) const = 0;

        [[nodiscard]] virtual IcStatus SetOpenLoadDetectControl3(OcpDeglitchTime deglitchTime, bool negativeCurrentOldEnabled) const = 0;

        [[nodiscard]] virtual IcStatus GetOpenLoadDetectControl3(OcpDeglitchTime& deglitchTime, bool& negativeCurrentOldEnabled) const = 0;

        [[nodiscard]] virtual IcStatus EnableLowCurrentOpenLoadDetect(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetEnabledLowCurrentOpenLoadDetect(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus EnablePassiveOpenLoadDetect(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetEnabledPassiveOpenLoadDetect(HalfBridgeBitMask& channels) const = 0;

        [[nodiscard]] virtual IcStatus EnablePassiveVmOpenLoadDetect(HalfBridgeBitMask channelMask) const = 0;

        [[nodiscard]] virtual IcStatus GetEnabledPassiveVmOpenLoadDetect(HalfBridgeBitMask& channels) const = 0;
    };
}