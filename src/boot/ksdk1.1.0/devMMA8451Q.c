/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>
#include <math.h>

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


extern volatile WarpI2CDeviceState	deviceMMA8451QState;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initMMA8451Q(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceMMA8451QState.i2cAddress			= i2cAddress;
	deviceMMA8451QState.operatingVoltageMillivolts	= operatingVoltageMillivolts;
	//writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, 0x05)
	write2a();
	return;
}

WarpStatus
writeSensorRegisterMMA8451Q(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x09: case 0x0a: case 0x0e: case 0x0f:
		case 0x11: case 0x12: case 0x13: case 0x14:
		case 0x15: case 0x17: case 0x18: case 0x1d:
		case 0x1f: case 0x20: case 0x21: case 0x23:
		case 0x24: case 0x25: case 0x26: case 0x27:
		case 0x28: case 0x29: case 0x2a: case 0x2b:
		case 0x2c: case 0x2d: case 0x2e: case 0x2f:
		case 0x30: case 0x31:
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
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
		0 /* I2C instance */,
		&slave,
		commandByte,
		1,
		payloadByte,
		1,
		gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorMMA8451Q(uint8_t payloadF_SETUP, uint8_t payloadCTRL_REG1)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QF_SETUP /* register address F_SETUP */,
												  payloadF_SETUP /* payload: Disable FIFO */
	);

	i2cWriteStatus2 = writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1 /* register address CTRL_REG1 */,
												  payloadCTRL_REG1 /* payload */
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterMMA8451Q(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05: case 0x06: case 0x09:
		case 0x0a: case 0x0b: case 0x0c: case 0x0d:
		case 0x0e: case 0x0f: case 0x10: case 0x11:
		case 0x12: case 0x13: case 0x14: case 0x15:
		case 0x16: case 0x17: case 0x18: case 0x1d:
		case 0x1e: case 0x1f: case 0x20: case 0x21:
		case 0x22: case 0x23: case 0x24: case 0x25:
		case 0x26: case 0x27: case 0x28: case 0x29:
		case 0x2a: case 0x2b: case 0x2c: case 0x2d:
		case 0x2e: case 0x2f: case 0x30: case 0x31:
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
		.address = deviceMMA8451QState.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
		0 /* I2C peripheral instance */,
		&slave,
		cmdBuf,
		1,
		(uint8_t *)deviceMMA8451QState.i2cBuffer,
		numberOfBytes,
		gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataMMA8451Q(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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

	i2cReadStatus = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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
appendSensorDataMMA8451Q(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceMMA8451QState.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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

	i2cReadStatus                   = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceMMA8451QState.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceMMA8451QState.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */
	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

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

/*enable acc reading*/
void write2a(){
	uint8_t ctrl1=0x05;

	writeSensorRegisterMMA8451Q(kWarpSensorConfigurationRegisterMMA8451QCTRL_REG1, ctrl1);
}

#define SVM_THRESHOLD (4.1 * 9.81 * 1000)  // Convert 4.1g to m/s^2
#define tangentTHETA_THRESHOLD 274  // Convert 70 degrees to radians

void analyzeFallDetection(int svm, float theta) 
{
    bool fallDetected = false;

    // Calculate THETA (angle)
    //float theta = atan2(sqrt(ax * ax + az * az), ay); // Theta is in radians

    // Check fall condition
    // if (svm > SVM_THRESHOLD && theta > THETA_THRESHOLD) {
    //     fallDetected = true;
    //     }
	    if (svm > SVM_THRESHOLD && theta > tangentTHETA_THRESHOLD) {
        fallDetected = true;
        }

    if (fallDetected) {
        warpPrint("Fall detected\n");
    } else {
        warpPrint("No fall detected\n");
    }
}

int sqrtInt(int base){ // Step 1: calculate the magnitude of the acceleration using Pythagoras' theorem across three cartesian axes.
  if(base == 0 || base == 1){ // If the number equals 0 or 1, the root equals the base.
    return base;
  }
  else{
    int root = base / 8; // Guess the square root at first.
    // warpPrint("sqrtInt(): Square rooting the number %d.\n", base);
    while(1){ // Perform this iterative result until the square root is calculated.
      int oldRoot = root; // Save the old root to compare the new one to.
      root = (root / 2) + (base / (2 * root));
      if(abs(root - oldRoot) <= 1){ // <= 1 to prevent the result hopping between two adjacent numbers and failing to converge.
	// warpPrint("sqrtInt(): Square root of %d = %d mms^-2.\n", base, root + 1);
        return (root + 1); // Add 1 to round up, so the final square root result is accurate.
      }
      else{
        // warpPrint("sqrtInt(): Guessed the number %d.\n", root);
        // warpPrint("sqrtInt(): %d != %d.\n", root, oldRoot);  
      }
    }
  }
}



void readAndConvertAccelerations() {
	// Define constants for scale factor, gravity, and conversion to mm/s^2
	#define SCALE_FACTOR 4096  // For ±2g sensitivity range
	#define GRAVITY 9.81       // Acceleration due to gravity in m/s²
	#define CONVERSION_FACTOR 1000  // Converting to mm/s²

    // Variables to store acceleration data
    int16_t accelerationX, accelerationY, accelerationZ;   // counts 
    int accelerationX_mm_s2, accelerationY_mm_s2, accelerationZ_mm_s2;
	int sumX = 0;
	int sumY = 0;
	int sumZ = 0; 
	int svm = 0;
	int theta_ele =0;
	//int theta =0; 

	//bool fall_detected = false; 
	for (int i = 0; i < 20; i++)
	{
    	// Read X-axis data
		WarpStatus statusX = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 2);
		accelerationX = (deviceMMA8451QState.i2cBuffer[0] << 6) | (deviceMMA8451QState.i2cBuffer[1] >> 2);
		accelerationX = (accelerationX ^ (1 << 13)) - (1 << 13);  // Convert to 16-bit signed value
		accelerationX_mm_s2 = (int)((float)accelerationX / SCALE_FACTOR * GRAVITY * CONVERSION_FACTOR);  // Convert to mm/s²

		// Read Y-axis data
		WarpStatus statusY = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 2);
		accelerationY = (deviceMMA8451QState.i2cBuffer[0] << 6) | (deviceMMA8451QState.i2cBuffer[1] >> 2);
		accelerationY = (accelerationY ^ (1 << 13)) - (1 << 13);  // Convert to 16-bit signed value
		accelerationY_mm_s2 = (int)((float)accelerationY / SCALE_FACTOR * GRAVITY * CONVERSION_FACTOR);  // Convert to mm/s²

		// Read Z-axis data
		WarpStatus statusZ = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 2);
		accelerationZ = (deviceMMA8451QState.i2cBuffer[0] << 6) | (deviceMMA8451QState.i2cBuffer[1] >> 2);
		accelerationZ = (accelerationZ ^ (1 << 13)) - (1 << 13);  // Convert to 16-bit signed value
		accelerationZ_mm_s2 = (int)((float)accelerationZ / SCALE_FACTOR * GRAVITY * CONVERSION_FACTOR);  // Convert to mm/s²

		//addSampleToBuffer(accelerationX_mm_s2, accelerationY_mm_s2, accelerationZ_mm_s2);
		//Print the collected acceleration data
		warpPrint("Acceleration Data(counts): X = %d, Y = %d, Z = %d\n", accelerationX, accelerationY, accelerationZ);
		// Print the collected acceleration data in mm/s²
		//warpPrint("Acceleration Data(mm/s²): X = %d, Y = %d, Z = %d\n", accelerationX_mm_s2, accelerationY_mm_s2, accelerationZ_mm_s2);
		sumX += accelerationX_mm_s2; 
		sumY += accelerationY_mm_s2; 
		sumZ += accelerationZ_mm_s2;
		// sumX += accelerationX; 
		// sumY += accelerationY;
		// sumZ += accelerationZ;
		OSA_TimeDelay(500); // Delay as per your requirement
	}

	int meanX = sumX / 5;
	int meanY = sumY / 5;
	int meanZ = sumZ / 5;
	warpPrint("meanX = %d, meanY = %d, meanZ = %d\n", meanX, meanY, meanZ);
	svm = sqrtInt(meanX * meanX + meanY * meanY + meanZ * meanZ);
  
	//int svm = sqrt(meanX * meanX + meanY * meanY + meanZ * meanZ);
	warpPrint("SVM = %d\n",svm);

	// Calculate THETA (angle)
	theta_ele = sqrtInt(meanX * meanX + meanZ * meanZ);
	warpPrint("theta_ele = %d\n",theta_ele);
	int tangentTHETA = theta_ele *100 / meanY;
	warpPrint("tangentTHETA = %d\n",tangentTHETA);
	analyzeFallDetection(svm,tangentTHETA);

	// float theta = atan2(theta_ele,meanY);
	// warpPrint("theta = %f\n",theta);
    //float theta = atan2(sqrt(ax * ax + az * az), ay); // Theta is in radians
	//theta = calculate_theta(theta_ele, meanY);
	//warpPrint("theta = %d\n",theta);
}



