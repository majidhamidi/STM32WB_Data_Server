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
#include "stm32wbxx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIO1_DRDY_Pin GPIO_PIN_0
#define BIO1_DRDY_GPIO_Port GPIOC
#define BIO1_RST_Pin GPIO_PIN_2
#define BIO1_RST_GPIO_Port GPIOC
#define BIO1_PWDN_Pin GPIO_PIN_3
#define BIO1_PWDN_GPIO_Port GPIOC
#define BIO_START_Pin GPIO_PIN_0
#define BIO_START_GPIO_Port GPIOA
#define BIO1_CS_Pin GPIO_PIN_1
#define BIO1_CS_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_8
#define LED2_GPIO_Port GPIOA
#define BIO2_CS_Pin GPIO_PIN_4
#define BIO2_CS_GPIO_Port GPIOC
#define BIO2_DRDY_Pin GPIO_PIN_5
#define BIO2_DRDY_GPIO_Port GPIOC
#define LDSW_Pin GPIO_PIN_4
#define LDSW_GPIO_Port GPIOE
#define BIO2_RST_Pin GPIO_PIN_6
#define BIO2_RST_GPIO_Port GPIOC
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_0
#define SW2_GPIO_Port GPIOD
#define SW5_Pin GPIO_PIN_1
#define SW5_GPIO_Port GPIOD
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define BIO2_PWDN_Pin GPIO_PIN_4
#define BIO2_PWDN_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_6
#define STLINK_RX_GPIO_Port GPIOB
#define STLINK_TX_Pin GPIO_PIN_7
#define STLINK_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern RTC_HandleTypeDef hrtc; /**< RTC handler declaration */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
