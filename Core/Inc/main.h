/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l0xx_hal.h"

#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_dma.h"

#include "stm32l0xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// NOTE: this is not thread safe beware of using from FreeRTOS tasks
void debug_printf(const char *fmt, ...);
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_LED_Pin GPIO_PIN_15
#define DEBUG_LED_GPIO_Port GPIOC
#define IRQ_TAMPERn_Pin GPIO_PIN_0
#define IRQ_TAMPERn_GPIO_Port GPIOA
#define BATT_CHRG_STAT_Pin GPIO_PIN_1
#define BATT_CHRG_STAT_GPIO_Port GPIOA
#define MICRO_EPU_TXD_Pin GPIO_PIN_2
#define MICRO_EPU_TXD_GPIO_Port GPIOA
#define MICRO_EPU_RXD_Pin GPIO_PIN_3
#define MICRO_EPU_RXD_GPIO_Port GPIOA
#define BATT_CHRG_LOW_Pin GPIO_PIN_4
#define BATT_CHRG_LOW_GPIO_Port GPIOA
#define ZER_PWR_HOLD_Pin GPIO_PIN_5
#define ZER_PWR_HOLD_GPIO_Port GPIOA
#define PGOOD__5V_ZER_Pin GPIO_PIN_6
#define PGOOD__5V_ZER_GPIO_Port GPIOA
#define PGOOD__5V5_Pin GPIO_PIN_7
#define PGOOD__5V5_GPIO_Port GPIOA
#define BATT_CHRG_ENn_Pin GPIO_PIN_1
#define BATT_CHRG_ENn_GPIO_Port GPIOB
#define EPU_PWR_EN_Pin GPIO_PIN_10
#define EPU_PWR_EN_GPIO_Port GPIOB
#define PWR_BTN_ON_Pin GPIO_PIN_11
#define PWR_BTN_ON_GPIO_Port GPIOB
#define RST_BTN_ON_Pin GPIO_PIN_12
#define RST_BTN_ON_GPIO_Port GPIOB
#define EPU_ON_Pin GPIO_PIN_13
#define EPU_ON_GPIO_Port GPIOB
#define EXT_SHDN_Pin GPIO_PIN_14
#define EXT_SHDN_GPIO_Port GPIOB
#define PWR_FAULTn_Pin GPIO_PIN_8
#define PWR_FAULTn_GPIO_Port GPIOA
#define MICRO_EXT_TXD_Pin GPIO_PIN_9
#define MICRO_EXT_TXD_GPIO_Port GPIOA
#define MICRO_EXT_RXD_Pin GPIO_PIN_10
#define MICRO_EXT_RXD_GPIO_Port GPIOA
#define I2C_ZER_SCL_Pin GPIO_PIN_6
#define I2C_ZER_SCL_GPIO_Port GPIOB
#define I2C_ZER_SDA_Pin GPIO_PIN_7
#define I2C_ZER_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
