/**
******************************************************************************
* @file gremsy_common.h
* @author Gremsy Team
* @version V1.0.0
* @date 09-May-2018
* @brief This file contains all the functions prototypes
for the Gremsy
* common firmware library.
*
************************************************************
******************
* @par
* COPYRIGHT NOTICE: (c) 2016 Gremsy. All rights reserved.
*
* The information contained herein is confidential
* property of Company. The use, copying, transfer or
* disclosure of such information is prohibited except
* by express written agreement with Company.
*
************************************************************
******************
*/
/* Define to prevent recursive inclusion
------------------------------------------------------------------------------*/
#ifndef __GREMSY_GLCD_SSD1306__
#define __GREMSY_GLCD_SSD1306__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes
------------------------------------------------------------------------------*/
//#include "gremsy_glcd_fonts.h"
#include "main.h"
#include "stdbool.h"

/* Exported types
------------------------------------------------------------------------------*/
/* Exported constants
------------------------------------------------------------------------------*/
/* Exported macro
------------------------------------------------------------------------------*/
/* Exported functions
------------------------------------------------------------------------------*/
bool ssd1306_update_screen(bool spi_type);
void ssd1306_init(void);
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color);
#ifdef __cplusplus
}
#endif
#endif /* __GREMSY_COMON_H */
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
