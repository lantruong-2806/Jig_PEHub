/** 
  ******************************************************************************
  * @file    gremsy_button.h
  * @author  Gremsy Team
  * @version V1.0.0
  * @date    18-Aug-2020
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

#ifndef __GREMSY_BUTTON_H__
#define __GREMSY_BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "main.h"

/* Exported types ------------------------------------------------------------*/
#define BUTTON_DEBOUNCE_TIME    20
#define BUTTON_MULTICLICK_TIME  250
#define BUTTON_LONGCLICK_TIME   1000
#define BUTTON_ACTIVE_LEVEL     GPIO_PIN_RESET
#define GET_BUTTON HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin)

/// dinh nghia nut nhan
typedef struct
{
    /** @brief tra ve gia tri click cua nut nhan
        @return none
    */
    int8_t (*get_clicks)(void);

    /** @brief update gia tri nut nhan
        @return none
    */
    void (*process)(void);
}
gremsy_button_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
gremsy_button_t* gremsy_button_init(void);
gremsy_button_t* gremsy_button(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/

