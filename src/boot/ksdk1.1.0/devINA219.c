/*
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

uint32_t ina219_currentMultiplier_uA;
int16_t ina219_powerMultiplier_uW;
uint16_t ina219_calValue;

void setCalibration_lzb();
void setCalibration_lzb_double_cal();

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

    setCalibration_lzb();

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case INA219_REG_CONFIG: case INA219_REG_CALIBRATION:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
	payloadByte[1] = payload & 0xFF;        /* LSB */
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		2,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;

	//int numberOfBytes=2

	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case INA219_REG_CONFIG:
        case INA219_REG_SHUNTVOLTAGE:
        case INA219_REG_BUSVOLTAGE:
        case INA219_REG_POWER:
        case INA219_REG_CURRENT:
        case INA219_REG_CALIBRATION:
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceINA219State.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

/*!
 *  @brief Prints the 4 Output Registers of the INA219
 * 
 *  Order is Shunt Voltage, Bus Voltage, Power, Current
 *  @note Prints raw register values - not readings of variables. 
 */
void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

    i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

}

uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	/*
	 *	From the INA219 datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also i
	 mprove things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
    // TODO: CALIBRATE
	// readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
    // TODO: CALIBRATE
	// readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
    // TODO: CALIBRATE
	// readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

    i2cReadStatus                   = readSensorRegisterINA219(INA219_REG_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
    // TODO: CALIBRATE
	// readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
        index += 1;

        buf[index] = (uint8_t)(readSensorRegisterValueCombined);
        index += 1;

	}

	return index;
}


/*!
 *  @brief Set device to calibration best for 4B25 CW3.
 * 
 *  Max expected current from OLED is ~25mA and device functions at
 *  5V -> Therefore set max current to 400mA, max bus voltage to 16V 
 *  and max shunt voltage to 40mV. Then choose current LSB to minimum 
 *  that doesn't overflow 16-bit calibration register: 7uA. 
 *  @note Assumes 0.1 Ohm Shunt Resistor (on Breakout)
 *  @note Code based on Adafruit INA219 Library
 */
void
setCalibration_lzb()
{


    ina219_calValue = 58514;

    // Set multipliers to convert raw current/power values
    ina219_currentMultiplier_uA = 7;    // Current LSB = 5uA per bit 
    ina219_powerMultiplier_uW = 0.14 * 1000; // Power LSB = 0.1mW per bit

    // Set Calibration register to 'Cal' calculated above
    writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    writeSensorRegisterINA219(INA219_REG_CONFIG, config);
}

void
setCalibration_lzb_double_cal()
{


    ina219_calValue = 58514;

    // Set multipliers to convert raw current/power values
    ina219_currentMultiplier_uA = 7;    // Current LSB = 5uA per bit 
    ina219_powerMultiplier_uW = 0.14 * 1000; // Power LSB = 0.1mW per bit

    // Set Calibration register to 'Cal' calculated above
    writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

    // Set Config register to take into account the settings above
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                      INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                      INA219_CONFIG_SADCRES_12BIT_1S_532US |
                      INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    writeSensorRegisterINA219(INA219_REG_CONFIG, config);
}

int16_t getBusVoltage_raw_INA219()
{
    uint16_t value;

    readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t getShuntVoltage_raw_INA219()
{
    uint16_t value;
    readSensorRegisterINA219(INA219_REG_SHUNTVOLTAGE, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];

    return value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t getCurrent_raw_INA219()
{
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

    // Now we can safely read the CURRENT register!
    readSensorRegisterINA219(INA219_REG_CURRENT, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];
    return value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t getPower_raw_INA219()
{
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available ... avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    writeSensorRegisterINA219(INA219_REG_CALIBRATION, ina219_calValue);

    // Now we can safely read the POWER register!
    readSensorRegisterINA219(INA219_REG_POWER, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];
    return value;
}

/*!
 *  @brief  Gets the shunt voltage in uV (so +-327000uV)
 *  @return the shunt voltage converted to microvolts
 */
int32_t getShuntVoltage_uV_INA219()
{
    int16_t value;
    value = getShuntVoltage_raw_INA219();
    return value * 10; // LSB is 10uV
}

/*!
 *  @brief  Gets the bus voltage in millivolts
 *  @return the bus voltage converted to millivolts
 */
int32_t getBusVoltage_mV_INA219()
{
    int16_t value = getBusVoltage_raw_INA219();
    return value;
}

/*!
 *  @brief  Gets the current value in uA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to microamps
 */
int32_t getCurrentINA219()
{
    int16_t valueDec = getCurrent_raw_INA219();
    valueDec *= ina219_currentMultiplier_uA;
    return valueDec;
}


/*!
 *  @brief  Gets the power value in uW, taking into account the
 *          config settings and power LSB
 *  @return power reading converted to microwatts
 */
int32_t getPower_uW_INA219()
{
    int32_t valueDec = getPower_raw_INA219();
    valueDec *= ina219_powerMultiplier_uW;
    return valueDec;
}