/*
Library:					STM32F4 Audio Codec - CS43L22
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			29/01/2016
Last modified:			13/02/2020 by JC Toussaint Phelma Grenoble-INP
Description:			This is an STM32 device driver library for the CS43L22 Audio Codec, using STM HAL libraries

References:
			1) Cirrus Logic CS43L22 datasheet
				 https://www.mouser.com/ds/2/76/CS43L22_F2-1142121.pdf
			2) ST opensource CS43L22 Audio Codec dsp drivers.

 * Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
 */

#include "main.h"
#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
//extern I2S_HandleTypeDef hi2s3;
extern SAI_HandleTypeDef hsai_BlockA1;

uint32_t I2cxTimeout = 0xA000; /*<! The value of the maximal timeout for I2C waiting loops */

static void I2Cx_Error(void);

//-------------- Static Functions ---------------//
/**
 * @brief  I2Cx Bus initialization.
 */
static void I2Cx_Init(void)
{
	if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
	{
		  hi2c1.Instance = I2C1;
		  hi2c1.Init.Timing = 0x00404C74;
		  hi2c1.Init.OwnAddress1 = 0;
		  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		  hi2c1.Init.OwnAddress2 = 0;
		  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
		  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		  {
		    Error_Handler();
		  }
	}
}
/**
 * @brief  Writes a value in a register of the device through BUS.
 * @param  Addr: Device address on BUS Bus.
 * @param  Reg: The target register address to write
 * @param  Value: The target register value to be written
 */
static void I2Cx_WriteData(uint16_t Addr, uint8_t Reg, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&hi2c1, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, I2cxTimeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error();
	}
}

/**
 * @brief  Reads a register of the device through BUS.
 * @param  Addr: Device address on BUS Bus.
 * @param  Reg: The target register address to write
 * @retval Data read at register address
 */
static uint8_t I2Cx_ReadData(uint16_t Addr, uint8_t Reg)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;

	status = HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2cxTimeout);

	/* Check the communication status */
	if(status != HAL_OK)
	{
		/* Execute user timeout callback */
		I2Cx_Error();
	}
	return value;
}

/**
 * @brief  I2Cx error treatment function.
 */
static void I2Cx_Error(void)
{
	/* De-initialize the I2C comunication BUS */
	HAL_I2C_DeInit(&hi2c1);

	/* Re- Initiaize the I2C comunication BUS */
	I2Cx_Init();
}

//-------------- Public Functions ----------------//
// Function(1): Initialisation
void CS43_Init(CS43_MODE outputMode)
{
	uint8_t value =  0;
	__HAL_UNLOCK(&hsai_BlockA1);     // THIS IS EXTREMELY IMPORTANT FOR hsai_BlockA1 TO WORK!!
	__HAL_SAI_ENABLE(&hsai_BlockA1); // THIS IS EXTREMELY IMPORTANT FOR hsai_BlockA1 TO WORK!!

	HAL_GPIO_WritePin(AUDIO_RST_GPIO_Port, AUDIO_RST_Pin, GPIO_PIN_SET); // Audio Reset

	//(1): Power down
	value = 0x01;
	I2Cx_WriteData(DAC_I2C_ADDR, POWER_CONTROL1, value);

	//(2): Enable Right and Left headphones
	value  = (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
	value |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
	value |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	value |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	I2Cx_WriteData(DAC_I2C_ADDR, POWER_CONTROL2, value);

	//(3): Automatic clock detection
	value = (1 << 7);
	I2Cx_WriteData(DAC_I2C_ADDR, CLOCKING_CONTROL, value);

	//(4): Interface control 1
	value=I2Cx_ReadData(DAC_I2C_ADDR, INTERFACE_CONTROL1);

	value &=  (1 << 5); // Clear all bits except bit 5 which is reserved
	value &= ~(1 << 7);  // Slave
	value &= ~(1 << 6);  // Clock polarity: Not inverted
	value &= ~(1 << 4);  // No DSP mode
	value &= ~(1 << 2);  // Left justified, up to 24 bit (default)
	value |=  (1 << 2);

	value |=  (3 << 0);  // 16-bit audio word length for I2S interface
	I2Cx_WriteData(DAC_I2C_ADDR, INTERFACE_CONTROL1, value);

	//(5): Passthrough A settings
	value=I2Cx_ReadData(DAC_I2C_ADDR, PASSTHROUGH_A);

	value &= 0xF0;      // Bits [4-7] are reserved
	value |= (1 << 0); // Use AIN1A as source for passthrough
	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_A, value);

	//(6): Passthrough B settings
	value=I2Cx_ReadData(DAC_I2C_ADDR, PASSTHROUGH_B);

	value &= 0xF0;      // Bits [4-7] are reserved
	value |=  (1 << 0); // Use AIN1B as source for passthrough
	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_B, value);

	//(7): Miscellaneous register settings
	value=I2Cx_ReadData(DAC_I2C_ADDR, MISCELLANEOUS_CONTRLS);

	if(outputMode == MODE_ANALOG)
	{
		value |=  (1 << 7);   // Enable passthrough for AIN-A
		value |=  (1 << 6);   // Enable passthrough for AIN-B
		value &= ~(1 << 5);   // Unmute passthrough on AIN-A
		value &= ~(1 << 4);   // Unmute passthrough on AIN-B
		value &= ~(1 << 3);   // Changed settings take affect immediately
	}
	else if(outputMode == MODE_I2S)
	{
		value = 0x02;
	}
	I2Cx_WriteData(DAC_I2C_ADDR, MISCELLANEOUS_CONTRLS, value);

	//(8): Unmute headphone and speaker
	value=I2Cx_ReadData(DAC_I2C_ADDR, PLAYBACK_CONTROL);

	value = 0x00;
	I2Cx_WriteData(DAC_I2C_ADDR, PLAYBACK_CONTROL, value);

	//(9): Set volume to default (0dB)
	value = 0x00;
	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_VOLUME_A, value);
	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_VOLUME_B, value);
	I2Cx_WriteData(DAC_I2C_ADDR, PCM_VOLUME_A, value);
	I2Cx_WriteData(DAC_I2C_ADDR, PCM_VOLUME_B, value);
}

// Function(2): Enable Right and Left headphones
void CS43_Enable_RightLeft(uint8_t side)
{
	uint8_t value =  0;
	switch (side)
	{
	case 0:
		value  = (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
		value |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;
	case 1:
		value  = (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
		value |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;
	case 2:
		value  = (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
		value |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;
	case 3:
		value  = (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
		value |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;
	default:
		break;
	}
	value |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	value |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	I2Cx_WriteData(DAC_I2C_ADDR, POWER_CONTROL2, value);
}

// Function(3): Set Volume Level
#define VOLUME_CONVERT(Volume)    (((Volume) > 100)? 255:((uint8_t)(((Volume) * 255) / 100)))
void CS43_SetVolume(uint8_t volume)
{
	uint8_t value =  0;
	value = VOLUME_CONVERT(volume);
	value = (value + 0x19); // implicit modulo 256;

	/* Set the Master volume */
	I2Cx_WriteData(DAC_I2C_ADDR, CS43L22_REG_MASTER_A_VOL, value);
	I2Cx_WriteData(DAC_I2C_ADDR, CS43L22_REG_MASTER_B_VOL, value);
}

//void CS43_SetVolume(uint8_t volume)
//{
//	uint8_t value =  0;
//	value =  (uint8_t )(volume*127./50.-127);
//	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_VOLUME_A, value);
//	I2Cx_WriteData(DAC_I2C_ADDR, PASSTHROUGH_VOLUME_B, value);
//
//	value = VOLUME_CONVERT_D(value);
//
//	/* Set the Master volume */
//	I2Cx_WriteData(DAC_I2C_ADDR, CS43L22_REG_MASTER_A_VOL, value);
//	I2Cx_WriteData(DAC_I2C_ADDR, CS43L22_REG_MASTER_B_VOL, value);
//}

// Function(4): Start the Audio DAC
void CS43_Start(void)
{
	uint8_t value =  0;
	// Write 0x99 to register 0x00.
	value = 0x99;
	I2Cx_WriteData(DAC_I2C_ADDR, CONFIG_00, value);

	// Write 0x80 to register 0x47.
	value = 0x80;
	I2Cx_WriteData(DAC_I2C_ADDR, CONFIG_47, value);

	// Write '1'b to bit 7 in register 0x32.
	value=I2Cx_ReadData(DAC_I2C_ADDR, CONFIG_32);
	value |= 0x80;
	I2Cx_WriteData(DAC_I2C_ADDR, CONFIG_32, value);

	// Write '0'b to bit 7 in register 0x32.
	value=I2Cx_ReadData(DAC_I2C_ADDR, CONFIG_32);
	value &= ~(0x80);
	I2Cx_WriteData(DAC_I2C_ADDR, CONFIG_32, value);

	// Write 0x00 to register 0x00.
	value = 0x00;
	I2Cx_WriteData(DAC_I2C_ADDR, CONFIG_00, value);

	//Set the "Power Ctl 1" register (0x02) to 0x9E
	value = 0x9E;
	I2Cx_WriteData(DAC_I2C_ADDR, POWER_CONTROL1, value);
}

void CS43_Stop(void)
{
	uint8_t value =  0x01;
	I2Cx_WriteData(DAC_I2C_ADDR, POWER_CONTROL1, value);
}
