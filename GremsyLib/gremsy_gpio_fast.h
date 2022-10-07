/** 
  ******************************************************************************
  * @file    gremsy_gpio_fast.h
  * @author  Gremsy Team
  * @version V1.0.0
  * @date    21-Nov-2020
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ******************************************************************************
  * @Copyright
  * COPYRIGHT NOTICE: (c) 2020 Gremsy. All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __GREMSY_GPIO_FAST__
#define __GREMSY_GPIO_FAST__

#ifdef __cplusplus
extern "C" {
#endif

//*** <<< Use Configuration Wizard in Context Menu >>> ***
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/
//<o> STM32             <0x01=> F1    <0x02=> F2
//                      <0x03=> F3    <0x04=> F4
//                      <0x05=> G0    <0x06=> G4
//                      <0x07=> H7
//<i> Chon dong STM32 phu hop
//<i> Tat ca cac io da duoc init trong thu vien HAL
#define STM32_TYPE      0x01
/// COMMON ---------------------------------------------------------------------

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
#if (STM32_TYPE == 0x01) //STM32F1

/** @brief      Ham doc input gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline bool gpio_fast_input_read(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    return (bool)(GPIO_Port->IDR & GPIO_Pin);
}

/** @brief      Ham set input pullup cho gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline void gpio_fast_input_set_pullup(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    //Set pullup
    GPIO_Port->ODR |= GPIO_Pin;
}

/** @brief      Ham set input pulldown cho gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline void gpio_fast_input_set_pulldown(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    //Set pulldown
    GPIO_Port->ODR &= (~GPIO_Pin);
}

///** @brief      Ham set input nopull cho gpio
//    @param[in]  GPIO PORT, GPIO PIN
//    @return     None
//*/
//static inline void gpio_fast_set_pullnopull(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
//{

//}


/** @brief      Ham set input pullup cho gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline void gpio_fast_output_toggle(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    GPIO_Port->ODR^=GPIO_Pin;
}

/** @brief      Ham set output cho gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline void gpio_fast_output_set(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
//    GPIO_Port->BSRR|=GPIO_Pin;
	GPIO_Port->BSRR = GPIO_Pin;
}

/** @brief      Ham reset output cho gpio
    @param[in]  GPIO PORT, GPIO PIN
    @return     None
*/
static inline void gpio_fast_output_reset(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin)
{
    //GPIO_Port->BRR|=GPIO_Pin;
	GPIO_Port->BSRR |= (uint32_t)GPIO_Pin << 16U;
}
//static inline gpio_fast_set( )
//{

//}
//static inline gpio_fast_write( )
//{

//}

#elif (STM32_TYPE == 0x02) //STM32F2
#elif (STM32_TYPE == 0x03) //STM32F3
#elif (STM32_TYPE == 0x04) //STM32F4
#elif (STM32_TYPE == 0x05) //STM32G0
#elif (STM32_TYPE == 0x06) //STM32G4
#elif (STM32_TYPE == 0x07) //STM32H7
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

