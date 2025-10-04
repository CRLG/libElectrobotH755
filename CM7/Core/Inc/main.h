/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main_app.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern volatile uint8_t tick;

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
#define Mot1_Sens1_Pin GPIO_PIN_2
#define Mot1_Sens1_GPIO_Port GPIOE
#define Mot2_Sens1_Pin GPIO_PIN_3
#define Mot2_Sens1_GPIO_Port GPIOE
#define Mot1_Sens2_Pin GPIO_PIN_4
#define Mot1_Sens2_GPIO_Port GPIOE
#define Mot1_PWM_Pin GPIO_PIN_5
#define Mot1_PWM_GPIO_Port GPIOE
#define Mot2_PWM_Pin GPIO_PIN_6
#define Mot2_PWM_GPIO_Port GPIOE
#define Mot3_PWM_Pin GPIO_PIN_6
#define Mot3_PWM_GPIO_Port GPIOF
#define Mot4_PWM_Pin GPIO_PIN_7
#define Mot4_PWM_GPIO_Port GPIOF
#define Servo6_PWM_Pin GPIO_PIN_8
#define Servo6_PWM_GPIO_Port GPIOF
#define Servo7_PWM_Pin GPIO_PIN_9
#define Servo7_PWM_GPIO_Port GPIOF
#define EanaVbat_Pin GPIO_PIN_0
#define EanaVbat_GPIO_Port GPIOC
#define Etor1_Pin GPIO_PIN_2
#define Etor1_GPIO_Port GPIOC
#define Etor2_Pin GPIO_PIN_3
#define Etor2_GPIO_Port GPIOC
#define Eana1_Pin GPIO_PIN_0
#define Eana1_GPIO_Port GPIOA
#define Eana2_Pin GPIO_PIN_3
#define Eana2_GPIO_Port GPIOA
#define Eana5_Pin GPIO_PIN_4
#define Eana5_GPIO_Port GPIOA
#define Servo3_PWM_Pin GPIO_PIN_5
#define Servo3_PWM_GPIO_Port GPIOA
#define Eana4_Pin GPIO_PIN_6
#define Eana4_GPIO_Port GPIOA
#define Eana3_Pin GPIO_PIN_1
#define Eana3_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_2
#define SPI_MOSI_GPIO_Port GPIOB
#define Mot4_Sens1_Pin GPIO_PIN_14
#define Mot4_Sens1_GPIO_Port GPIOF
#define Mot4_Sens2_Pin GPIO_PIN_15
#define Mot4_Sens2_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_7
#define LED4_GPIO_Port GPIOE
#define LED5_Pin GPIO_PIN_8
#define LED5_GPIO_Port GPIOE
#define Coder1_1_Pin GPIO_PIN_9
#define Coder1_1_GPIO_Port GPIOE
#define Coder1_2_Pin GPIO_PIN_11
#define Coder1_2_GPIO_Port GPIOE
#define LED6_Pin GPIO_PIN_12
#define LED6_GPIO_Port GPIOE
#define Mot3_Sens2_Pin GPIO_PIN_13
#define Mot3_Sens2_GPIO_Port GPIOE
#define Mot3_Sens1_Pin GPIO_PIN_14
#define Mot3_Sens1_GPIO_Port GPIOE
#define LED7_Pin GPIO_PIN_15
#define LED7_GPIO_Port GPIOE
#define Servo5_PWM_Pin GPIO_PIN_10
#define Servo5_PWM_GPIO_Port GPIOB
#define Servo4_PWM_Pin GPIO_PIN_11
#define Servo4_PWM_GPIO_Port GPIOB
#define UART5_RX_UNUSED_Pin GPIO_PIN_12
#define UART5_RX_UNUSED_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define Servo1_PWM_Pin GPIO_PIN_15
#define Servo1_PWM_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define Mot2_Sens2_Pin GPIO_PIN_11
#define Mot2_Sens2_GPIO_Port GPIOD
#define Coder3_1_Pin GPIO_PIN_12
#define Coder3_1_GPIO_Port GPIOD
#define Coder3_2_Pin GPIO_PIN_13
#define Coder3_2_GPIO_Port GPIOD
#define Coder4_1_Pin GPIO_PIN_6
#define Coder4_1_GPIO_Port GPIOC
#define Coder4_2_Pin GPIO_PIN_7
#define Coder4_2_GPIO_Port GPIOC
#define SPI_CS1_Pin GPIO_PIN_8
#define SPI_CS1_GPIO_Port GPIOC
#define SPI_CS2_Pin GPIO_PIN_9
#define SPI_CS2_GPIO_Port GPIOC
#define SPI_SCK_Pin GPIO_PIN_10
#define SPI_SCK_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_11
#define SPI_MISO_GPIO_Port GPIOC
#define LED_RGB_WS2812_Pin GPIO_PIN_12
#define LED_RGB_WS2812_GPIO_Port GPIOC
#define CAN_RX_Pin GPIO_PIN_0
#define CAN_RX_GPIO_Port GPIOD
#define CAN_TX_Pin GPIO_PIN_1
#define CAN_TX_GPIO_Port GPIOD
#define RS232_2_DE_Pin GPIO_PIN_4
#define RS232_2_DE_GPIO_Port GPIOD
#define RS232_2_TX_Pin GPIO_PIN_5
#define RS232_2_TX_GPIO_Port GPIOD
#define RS232_2_RX_Pin GPIO_PIN_6
#define RS232_2_RX_GPIO_Port GPIOD
#define RS232_3_RX_Pin GPIO_PIN_9
#define RS232_3_RX_GPIO_Port GPIOG
#define RS232_3_TX_Pin GPIO_PIN_14
#define RS232_3_TX_GPIO_Port GPIOG
#define Servo2_PWM_Pin GPIO_PIN_3
#define Servo2_PWM_GPIO_Port GPIOB
#define Coder2_1_Pin GPIO_PIN_4
#define Coder2_1_GPIO_Port GPIOB
#define Coder2_2_Pin GPIO_PIN_5
#define Coder2_2_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define Etor3_Pin GPIO_PIN_8
#define Etor3_GPIO_Port GPIOB
#define Etor4_Pin GPIO_PIN_9
#define Etor4_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
