/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32l4xx.h"
#include "ff.h"
#include "stm32_adafruit_sd.h"
#include "cs43l22.h"
#include "stm32l476g_discovery.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "fatfs_storage.h"
#include "waveplayer.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define  MAX_BMP_FILES  25
#define  MAX_BMP_FILE_NAME 11
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SAI1_MCK_Pin GPIO_PIN_2
#define SAI1_MCK_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_3
#define AUDIO_RST_GPIO_Port GPIOE
#define SAI1_FS_Pin GPIO_PIN_4
#define SAI1_FS_GPIO_Port GPIOE
#define SAI1_SCK_Pin GPIO_PIN_5
#define SAI1_SCK_GPIO_Port GPIOE
#define SAI1_SD_Pin GPIO_PIN_6
#define SAI1_SD_GPIO_Port GPIOE
#define MFX_IRQ_OUT_Pin GPIO_PIN_13
#define MFX_IRQ_OUT_GPIO_Port GPIOC
#define JOY_CENTER_Pin GPIO_PIN_0
#define JOY_CENTER_GPIO_Port GPIOA
#define JOY_LEFT_Pin GPIO_PIN_1
#define JOY_LEFT_GPIO_Port GPIOA
#define JOY_RIGHT_Pin GPIO_PIN_2
#define JOY_RIGHT_GPIO_Port GPIOA
#define JOY_UP_Pin GPIO_PIN_3
#define JOY_UP_GPIO_Port GPIOA
#define MFX_WAKEUP_Pin GPIO_PIN_4
#define MFX_WAKEUP_GPIO_Port GPIOA
#define JOY_DOWN_Pin GPIO_PIN_5
#define JOY_DOWN_GPIO_Port GPIOA
#define LD_R_Pin GPIO_PIN_2
#define LD_R_GPIO_Port GPIOB
#define AUDIO_DIN_Pin GPIO_PIN_7
#define AUDIO_DIN_GPIO_Port GPIOE
#define LD_G_Pin GPIO_PIN_8
#define LD_G_GPIO_Port GPIOE
#define AUDIO_CLK_Pin GPIO_PIN_9
#define AUDIO_CLK_GPIO_Port GPIOE
#define SD_CS_Pin GPIO_PIN_10
#define SD_CS_GPIO_Port GPIOE
#define SD_SPIx_SCK_Pin GPIO_PIN_13
#define SD_SPIx_SCK_GPIO_Port GPIOE
#define SD_SPIx_MISO_Pin GPIO_PIN_14
#define SD_SPIx_MISO_GPIO_Port GPIOE
#define SD_SPIx_MOSI_Pin GPIO_PIN_15
#define SD_SPIx_MOSI_GPIO_Port GPIOE
#define MFX_I2C_SLC_Pin GPIO_PIN_10
#define MFX_I2C_SLC_GPIO_Port GPIOB
#define MFX_I2C_SDA_Pin GPIO_PIN_11
#define MFX_I2C_SDA_GPIO_Port GPIOB
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_9
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define COM2_Pin GPIO_PIN_10
#define COM2_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SEG10_Pin GPIO_PIN_15
#define SEG10_GPIO_Port GPIOA
#define OTG_FS_OverCurrent_Pin GPIO_PIN_10
#define OTG_FS_OverCurrent_GPIO_Port GPIOC
#define OTG_FS_VBUS_Pin GPIO_PIN_11
#define OTG_FS_VBUS_GPIO_Port GPIOC
#define EXT_RST_Pin GPIO_PIN_0
#define EXT_RST_GPIO_Port GPIOD
#define GYRO_INT1_Pin GPIO_PIN_2
#define GYRO_INT1_GPIO_Port GPIOD
#define USART_TX_Pin GPIO_PIN_5
#define USART_TX_GPIO_Port GPIOD
#define USART_RX_Pin GPIO_PIN_6
#define USART_RX_GPIO_Port GPIOD
#define GYRO_CS_Pin GPIO_PIN_7
#define GYRO_CS_GPIO_Port GPIOD
#define M3V3_REG_ON_Pin GPIO_PIN_3
#define M3V3_REG_ON_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define GYRO_INT2_Pin GPIO_PIN_8
#define GYRO_INT2_GPIO_Port GPIOB
#define XL_CS_Pin GPIO_PIN_0
#define XL_CS_GPIO_Port GPIOE
#define XL_INT_Pin GPIO_PIN_1
#define XL_INT_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define SPIx_TIMEOUT_MAX                        ((uint32_t)0x1000)
#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80
/**
  * @brief  SD Control Lines management
  */
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins
  */

#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOE_CLK_DISABLE()

#define AUDIO_RESET_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOE_CLK_ENABLE()
#define AUDIO_RESET_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOE_CLK_DISABLE()

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
