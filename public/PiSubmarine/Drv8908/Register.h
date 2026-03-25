#pragma once
#include <cstdint>

namespace PiSubmarine::Drv8908
{
    enum class Register : uint8_t
    {
        IcStat = 0,
        OcpStat1 = 1,
        OcpStat2 = 2,
        OldStat1 = 4,
        OldStat2 = 5,
        ConfigCtrl = 7,
        OpCtrl1 = 8,
        OpCtrl2 = 9,
        PwmCtrl1 = 0xB,
        PwmCtrl2 = 0xC,
        FwCtrl1 = 0xD,
        PwmMapCtrl1 = 0xF,
        PwmMapCtrl2 = 0x10,
        PwmMapCtrl3 = 0x11,
        PwmMapCtrl4 = 0x12,
        PwmFreqCtrl1 = 0x13,
        PwmFreqCtrl2 = 0x14,
        PwmDutyCtrl1 = 0x15,
        PwmDutyCtrl2 = 0x16,
        PwmDutyCtrl3 = 0x17,
        PwmDutyCtrl4 = 0x18,
        PwmDutyCtrl5 = 0x19,
        PwmDutyCtrl6 = 0x1A,
        PwmDutyCtrl7 = 0x1B,
        PwmDutyCtrl8 = 0x1C,
        SrCtrl1 = 0x1D,
        OldCtrl1 = 0x1F,
        OldCtrl2 = 0x20,
        OldCtrl3 = 0x21,
        OldCtrl4 = 0x22,
        OldCtrl5 = 0x23,
        OldCtrl6 = 0x24
    };
}
