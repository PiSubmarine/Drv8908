#pragma once

#include "PiSubmarine/RegUtils.h"
#include "PiSubmarine/Api/Internal/SPI/DriverConcept.h"

namespace PiSubmarine::Drv8908
{
	enum class RegOffset : uint8_t
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

	enum class IcStatus : uint8_t
	{
		NoPowerOnReset = (1 << 0),
		OverVoltage = (1 << 1),
		UnderVoltage = (1 << 2),
		OverCurrent = (1 << 3),
		OpenLoad = (1 << 4),
		OverTemperatureWarning = (1 << 5),
		OverTemperatureShutdown = (1 << 6),
		TestBit = (1 << 7)
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

	template<PiSubmarine::Api::Internal::SPI::DriverConcept SpiDriver>
	class Drv8908
	{
	public:
		Drv8908(SpiDriver driver) : m_Driver(driver)
		{

		}

		IcStatus GetOpenLoadStatus(OverCurrentStatus& ovp)
		{
			uint8_t old1 = 0;
			uint8_t old2 = 0;

			IcStatus status = ReadRegister(static_cast<uint8_t>(RegOffset::OldStat1, ocp1));
			status = ReadRegister(static_cast<uint8_t>(RegOffset::OldStat1, ocp2));
			ovp = static_cast<OverCurrentStatus>(old2 << 8 + old1);
			return status;
		}

		IcStatus GetOvercurrentStatus(OverCurrentStatus& ovp)
		{
			uint8_t ocp1 = 0;
			uint8_t ocp2 = 0;
			
			IcStatus status = ReadRegister(static_cast<uint8_t>(RegOffset::OcpStat1, ocp1));
			status = ReadRegister(static_cast<uint8_t>(RegOffset::OcpStat2, ocp2));
			ovp = static_cast<OverCurrentStatus>(ocp2 << 8 + ocp1);
			return status;
		}

	private:
		SpiDriver m_Driver;

		IcStatus ReadRegister(uint8_t reg, uint8_t& regData)
		{
			uint16_t dataOut = reg << 8;
			dataOut |= (1 << 14);
			uint16_t dataIn = 0;
			m_Driver.WriteRead(&dataOut, &dataIn, 2);
			IcStatus status = dataIn >> 8;
			regData = dataIn & 0xFF;
			return status;
		}

		IcStatus WriteRegister(uint8_t reg, uint8_t dataNew, uint8_t& dataOld)
		{
			uint16_t dataOut = (reg << 8) + dataNew;
			dataOut &= ~(1 << 14);
			uint16_t dataIn = 0;
			m_Driver.WriteRead(&dataOut, &dataIn, 2);
			IcStatus status = dataIn >> 8;
			dataOld = dataIn & 0xFF;
			return status;
		}
	};
}