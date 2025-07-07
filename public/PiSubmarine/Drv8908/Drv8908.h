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

	enum class PwmFrequency
	{
		Hz80,
		Hz100,
		Hz200,
		Hz2000
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
			
			IcStatus status = ReadRegister(static_cast<uint8_t>(RegOffset::OcpStat1), ocp1);
			status = ReadRegister(static_cast<uint8_t>(RegOffset::OcpStat2), ocp2);
			ovp = static_cast<OverCurrentStatus>(ocp2 << 8 + ocp1);
			return status;
		}

		IcStatus IsHalfBridgeEnabled(uint8_t hbIndex, bool& high, bool& low)
		{
			uint8_t reg;
			if (channel < 4)
			{
				reg = RegOffset::OpCtrl1;
			}
			else
			{
				reg = RegOffset::OpCtrl1;
			}

			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & IcStatus::TestBit == 0)
			{
				return 0;
			}

			uint8_t bitOffset = (hbIndex % 4) * 2;

			high = regData & (0b10 << bitOffset);
			low = regData & (0b01 << bitOffset);
			return status;
		}

		IcStatus SetHalfBridgeEnabled(uint8_t hbIndex, bool high, bool low)
		{
			uint8_t reg;
			if (channel < 4)
			{
				reg = RegOffset::OpCtrl1;
			}
			else
			{
				reg = RegOffset::OpCtrl1;
			}

			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & IcStatus::TestBit == 0)
			{
				return 0;
			}

			uint8_t bitOffset = (hbIndex % 4) * 2;

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

		IcStatus SetPwmFrequency(uint8_t channel, PwmFrequency freq)
		{
			uint8_t reg;
			if (channel < 4)
			{
				reg = RegOffset::PwmCtrl1;
			}
			else
			{
				reg = RegOffset::PwmCtrl2;
			}

			uint8_t bitOffset = (channel % 4) * 2;
			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & (IcStatus::TestBit) == 0)
			{
				return 0;
			}

			regData &= ~(0b11 << bitOffset);
			regData |= static_cast<uint8_t>(freq) << bitOffset;
			uint8_t regOld = 0;
			status = WriteRegister(reg, regData, regOld);

			return status;
		}

		IcStatus GetPwmFrequency(uint8_t channel, PwmFrequency& freq)
		{
			uint8_t reg;
			if (channel < 4)
			{
				reg = RegOffset::PwmCtrl1;
			}
			else
			{
				reg = RegOffset::PwmCtrl2;
			}

			uint8_t bitOffset = (channel % 4) * 2;

			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & (IcStatus::TestBit) == 0)
			{
				return 0;
			}

			freq = static_cast<PwmFrequency>((regData >> bitOffset) & 0b11);
			return status;
		}

		IcStatus SetPwmMap(uint8_t halfBridgeIndex, uint8_t pwmChannel)
		{
			uint8_t regShift = halfBridgeIndex / 2;
			uint8_t reg = static_cast<uint8_t>(RegOffset::PwmMapCtrl1) + regShift;
			uint8_t bitOffset = (halfBridgeIndex % 2) * 3;

			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & (IcStatus::TestBit) == 0)
			{
				return 0;
			}

			regData &= ~(0b111 << bitOffset);
			regData |= pwmChannel << bitOffset;
			uint8_t regOld = 0;
			status = WriteRegister(reg, regData, regOld);
			return status;
		}

		IcStatus GetPwmMap(uint8_t halfBridgeIndex, uint8_t& pwmChannel)
		{
			uint8_t regShift = halfBridgeIndex / 2;
			uint8_t reg = static_cast<uint8_t>(RegOffset::PwmMapCtrl1) + regShift;
			uint8_t bitOffset = (halfBridgeIndex % 2) * 3;

			uint8_t regData = 0;
			IcStatus status = ReadRegister(reg, regData);
			if (status & (IcStatus::TestBit) == 0)
			{
				return 0;
			}

			pwmChannel = ((regData >> bitOffset) & 0b111);
			return status;
		}

		IcStatus GetDutyCycle(uint8_t channel, uint8_t& value)
		{
			uint8_t reg = static_cast<uint8_t>(RegOffset::PwmDutyCtrl1) + channel;
			auto status = ReadRegister(reg, value);
			return status;
		}

		IcStatus SetDutyCycle(uint8_t channel, uint8_t value)
		{
			uint8_t reg = static_cast<uint8_t>(RegOffset::PwmDutyCtrl1) + channel;
			uint8_t regOld = 0;
			auto status = WriteRegister(reg, value, regOld);
			return status;
		}

	private:
		SpiDriver m_Driver;

		IcStatus ReadRegister(uint8_t reg, uint8_t& regData)
		{
			uint16_t dataOut = reg << 8;
			dataOut |= (1 << 14);
			uint16_t dataIn = 0;
			if (!m_Driver.WriteRead(&dataOut, &dataIn, 2))
			{
				return 0;
			}
			IcStatus status = dataIn >> 8;
			regData = dataIn & 0xFF;
			return status;
		}

		IcStatus WriteRegister(uint8_t reg, uint8_t dataNew, uint8_t& dataOld)
		{
			uint16_t dataOut = (reg << 8) + dataNew;
			dataOut &= ~(1 << 14);
			uint16_t dataIn = 0;
			if (!m_Driver.WriteRead(&dataOut, &dataIn, 2))
			{
				return 0;
			}
			IcStatus status = dataIn >> 8;
			dataOld = dataIn & 0xFF;
			return status;
		}
	};
}