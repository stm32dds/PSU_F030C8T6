/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I_DAC7_Pin GPIO_PIN_13
#define I_DAC7_GPIO_Port GPIOC
#define I_DAC8_Pin GPIO_PIN_14
#define I_DAC8_GPIO_Port GPIOC
#define I_DAC9_Pin GPIO_PIN_15
#define I_DAC9_GPIO_Port GPIOC
#define I_DAC3_Pin GPIO_PIN_0
#define I_DAC3_GPIO_Port GPIOF
#define I_DAC5_Pin GPIO_PIN_1
#define I_DAC5_GPIO_Port GPIOF
#define V_ADC_Pin GPIO_PIN_0
#define V_ADC_GPIO_Port GPIOA
#define I_ADC_Pin GPIO_PIN_1
#define I_ADC_GPIO_Port GPIOA
#define I_DAC4_Pin GPIO_PIN_2
#define I_DAC4_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_3
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_4
#define LCD_DC_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_6
#define LCD_CS_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define SW_T_Pin GPIO_PIN_0
#define SW_T_GPIO_Port GPIOB
#define V_DAC0_Pin GPIO_PIN_1
#define V_DAC0_GPIO_Port GPIOB
#define V_DAC6_Pin GPIO_PIN_2
#define V_DAC6_GPIO_Port GPIOB
#define V_DAC7_Pin GPIO_PIN_12
#define V_DAC7_GPIO_Port GPIOB
#define V_DAC8_Pin GPIO_PIN_13
#define V_DAC8_GPIO_Port GPIOB
#define V_DAC9_Pin GPIO_PIN_14
#define V_DAC9_GPIO_Port GPIOB
#define V_DAC5_Pin GPIO_PIN_15
#define V_DAC5_GPIO_Port GPIOB
#define V_DAC4_Pin GPIO_PIN_8
#define V_DAC4_GPIO_Port GPIOA
#define V_DAC3_Pin GPIO_PIN_9
#define V_DAC3_GPIO_Port GPIOA
#define LCD_BL_PWM_Pin GPIO_PIN_10
#define LCD_BL_PWM_GPIO_Port GPIOA
#define V_DAC2_Pin GPIO_PIN_11
#define V_DAC2_GPIO_Port GPIOA
#define V_DAC1_Pin GPIO_PIN_12
#define V_DAC1_GPIO_Port GPIOA
#define I_DAC0_Pin GPIO_PIN_15
#define I_DAC0_GPIO_Port GPIOA
#define I_DAC1_Pin GPIO_PIN_3
#define I_DAC1_GPIO_Port GPIOB
#define SW_A_Pin GPIO_PIN_4
#define SW_A_GPIO_Port GPIOB
#define SW_B_Pin GPIO_PIN_5
#define SW_B_GPIO_Port GPIOB
#define I_DAC2_Pin GPIO_PIN_8
#define I_DAC2_GPIO_Port GPIOB
#define I_DAC6_Pin GPIO_PIN_9
#define I_DAC6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define VREFINT_CAL_ADDR ((uint16_t*)(uint32_t) 0x1FFFF7BA)
#define TEMP30_CAL_ADDR ((uint16_t*)(uint32_t) 0x1FFFF7B8)
#define TEMP110_CAL_ADDR ((uint16_t*)(uint32_t) 0x1FFFF7C2)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
