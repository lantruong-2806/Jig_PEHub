/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define IN_CAPTURE_Pin GPIO_PIN_13
#define IN_CAPTURE_GPIO_Port GPIOC
#define IN_TRIG_Pin GPIO_PIN_14
#define IN_TRIG_GPIO_Port GPIOC
#define BTN_Pin GPIO_PIN_15
#define BTN_GPIO_Port GPIOC
#define IN_COM_RX_Pin GPIO_PIN_0
#define IN_COM_RX_GPIO_Port GPIOC
#define IN_COM_TX_Pin GPIO_PIN_1
#define IN_COM_TX_GPIO_Port GPIOC
#define OUT_FFC_ETH_TXN_Pin GPIO_PIN_12
#define OUT_FFC_ETH_TXN_GPIO_Port GPIOB
#define OUT_FFC_ETH_RXP_Pin GPIO_PIN_13
#define OUT_FFC_ETH_RXP_GPIO_Port GPIOB
#define OUT_FFC_ETH_RXN_Pin GPIO_PIN_14
#define OUT_FFC_ETH_RXN_GPIO_Port GPIOB
#define OUT_FFC_USB_DN_Pin GPIO_PIN_15
#define OUT_FFC_USB_DN_GPIO_Port GPIOB
#define IN_PGND3_Pin GPIO_PIN_6
#define IN_PGND3_GPIO_Port GPIOC
#define IN_PGND2_Pin GPIO_PIN_7
#define IN_PGND2_GPIO_Port GPIOC
#define IN_USB_DP_Pin GPIO_PIN_8
#define IN_USB_DP_GPIO_Port GPIOC
#define IN_USB_DN_Pin GPIO_PIN_9
#define IN_USB_DN_GPIO_Port GPIOC
#define IN_ETH_RXN_Pin GPIO_PIN_8
#define IN_ETH_RXN_GPIO_Port GPIOA
#define IN_ETH_RXP_Pin GPIO_PIN_11
#define IN_ETH_RXP_GPIO_Port GPIOA
#define IN_ETH_TXN_Pin GPIO_PIN_12
#define IN_ETH_TXN_GPIO_Port GPIOA
#define IN_ETH_TXP_Pin GPIO_PIN_15
#define IN_ETH_TXP_GPIO_Port GPIOA
#define IN_CANL_Pin GPIO_PIN_10
#define IN_CANL_GPIO_Port GPIOC
#define IN_CANH_Pin GPIO_PIN_11
#define IN_CANH_GPIO_Port GPIOC
#define IN_GPS_PPS_Pin GPIO_PIN_12
#define IN_GPS_PPS_GPIO_Port GPIOC
#define OUT_FFC_USB_DP_Pin GPIO_PIN_2
#define OUT_FFC_USB_DP_GPIO_Port GPIOD
#define OUT_FFC_UART_TX_Pin GPIO_PIN_3
#define OUT_FFC_UART_TX_GPIO_Port GPIOB
#define OUT_FFC_UART_RX_Pin GPIO_PIN_4
#define OUT_FFC_UART_RX_GPIO_Port GPIOB
#define OUT_FFC_CANH_Pin GPIO_PIN_5
#define OUT_FFC_CANH_GPIO_Port GPIOB
#define OUT_FFC_CANL_Pin GPIO_PIN_8
#define OUT_FFC_CANL_GPIO_Port GPIOB
#define OUT_FFC_ETH_TXP_Pin GPIO_PIN_9
#define OUT_FFC_ETH_TXP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//port1
#define EX_GPIO_Pin_1	0x0001
#define EX_GPIO_Pin_2	0x0002
#define EX_GPIO_Pin_3	0x0004
#define EX_GPIO_Pin_4	0x0008
#define EX_GPIO_Pin_5	0x0010
#define EX_GPIO_Pin_6	0x0020
#define EX_GPIO_Pin_7	0x0040
#define EX_GPIO_Pin_8	0x0080

//port 2
#define EX_GPIO_Pin_9	0x0100
#define EX_GPIO_Pin_10	0x0200
#define EX_GPIO_Pin_11	0x0400
#define EX_GPIO_Pin_12	0x0800
#define EX_GPIO_Pin_13	0x1000
#define EX_GPIO_Pin_14	0x2000
#define EX_GPIO_Pin_15	0x4000
#define EX_GPIO_Pin_16	0x8000

// define cho port IO expander
#define IO_EX_UART_TX   EX_GPIO_Pin_1
#define IO_EX_UART_RX   EX_GPIO_Pin_2
#define IO_EX_CANH      EX_GPIO_Pin_3
#define IO_EX_CANL	    EX_GPIO_Pin_4
#define IO_EX_CAPTURE   EX_GPIO_Pin_5
#define IO_EX_GPS_PPS   EX_GPIO_Pin_6
#define IO_EX_TRIG      EX_GPIO_Pin_7
#define IO_EX_USB_DP    EX_GPIO_Pin_8
#define IO_EX_USB_DN    EX_GPIO_Pin_9
#define IO_EX_ETHTXN    EX_GPIO_Pin_10
#define IO_EX_ETHRXN    EX_GPIO_Pin_11
#define IO_EX_ETHRXP    EX_GPIO_Pin_12
#define IO_EX_ETHTXP    EX_GPIO_Pin_13
// define debug test2
//#define GPIO_TEST 	0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
