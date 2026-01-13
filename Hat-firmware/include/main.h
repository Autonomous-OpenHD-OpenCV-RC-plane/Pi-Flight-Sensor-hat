/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define BACKUP_CONT_Pin GPIO_PIN_13
#define BACKUP_CONT_GPIO_Port GPIOC
#define BACKUP_Pin GPIO_PIN_14
#define BACKUP_GPIO_Port GPIOC
#define DROGUE_CONT_Pin GPIO_PIN_15
#define DROGUE_CONT_GPIO_Port GPIOC
#define DROGUE_Pin GPIO_PIN_0
#define DROGUE_GPIO_Port GPIOC
#define RECOVERY_CONT_Pin GPIO_PIN_1
#define RECOVERY_CONT_GPIO_Port GPIOC
#define RECOVERY_Pin GPIO_PIN_2
#define RECOVERY_GPIO_Port GPIOC
#define ELEV_PWM1_Pin GPIO_PIN_0
#define ELEV_PWM1_GPIO_Port GPIOA
#define RUDD_PWM2_Pin GPIO_PIN_1
#define RUDD_PWM2_GPIO_Port GPIOA
#define AIL_L_PWM3_Pin GPIO_PIN_2
#define AIL_L_PWM3_GPIO_Port GPIOA
#define AIL_R_PWM4_Pin GPIO_PIN_3
#define AIL_R_PWM4_GPIO_Port GPIOA
#define IMU_CS2_Pin GPIO_PIN_4
#define IMU_CS2_GPIO_Port GPIOA
#define IMU_CS1_Pin GPIO_PIN_4
#define IMU_CS1_GPIO_Port GPIOC
#define LED_STANDBY_Pin GPIO_PIN_5
#define LED_STANDBY_GPIO_Port GPIOC
#define LED_ARMED_Pin GPIO_PIN_0
#define LED_ARMED_GPIO_Port GPIOB
#define LED_STORAGE_Pin GPIO_PIN_1
#define LED_STORAGE_GPIO_Port GPIOB
#define LED_FLIGHT_Pin GPIO_PIN_2
#define LED_FLIGHT_GPIO_Port GPIOB
#define EN_SERVO_Pin GPIO_PIN_10
#define EN_SERVO_GPIO_Port GPIOB
#define EN_PI_Pin GPIO_PIN_11
#define EN_PI_GPIO_Port GPIOB
#define ALT_CS_Pin GPIO_PIN_12
#define ALT_CS_GPIO_Port GPIOB
#define RADIO_DIO1_Pin GPIO_PIN_6
#define RADIO_DIO1_GPIO_Port GPIOC
#define FLAP_L_PWM5_Pin GPIO_PIN_7
#define FLAP_L_PWM5_GPIO_Port GPIOC
#define FLAP_R_PWM6_Pin GPIO_PIN_8
#define FLAP_R_PWM6_GPIO_Port GPIOC
#define MAG_INT_Pin GPIO_PIN_9
#define MAG_INT_GPIO_Port GPIOC
#define ESC_PWM0_Pin GPIO_PIN_8
#define ESC_PWM0_GPIO_Port GPIOA
#define MEM_WP_Pin GPIO_PIN_10
#define MEM_WP_GPIO_Port GPIOA
#define MEM_HOLD_Pin GPIO_PIN_15
#define MEM_HOLD_GPIO_Port GPIOA
#define MEM_CS_Pin GPIO_PIN_12
#define MEM_CS_GPIO_Port GPIOC
#define RADIO_CS_Pin GPIO_PIN_2
#define RADIO_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
