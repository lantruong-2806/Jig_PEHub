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
#ifndef __GREMSY_THREAD__
#define __GREMSY_THREAD__
#ifdef __cplusplus
extern "C" {
#endif
/* Includes
------------------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
/* Exported types
------------------------------------------------------------------------------*/
typedef struct
{
  //funtion for microseconds
  uint32_t (*get_tick_us)(void);
  void     (*sleep_us)(uint32_t time);
  bool (*loop_us)(uint32_t* var, uint32_t time);
  uint32_t (*getloop_us)(uint32_t* var);
  uint32_t (*get_us)(uint32_t* var);
  
  //funtion for milliseconds
  uint32_t (*get_tick_ms)(void);
  void     (*sleep_ms)(uint32_t time);
  bool     (*loop_ms)(uint32_t* var, uint32_t time);
  uint32_t (*getloop_ms)(uint32_t* var);
  uint32_t (*get_ms)(uint32_t* var);
  
  //common
  void (*reset)(uint32_t* var);
}gremsy_thread_t;
/* Exported constants
------------------------------------------------------------------------------*/
/* Exported macro
------------------------------------------------------------------------------*/
/* Exported functions
------------------------------------------------------------------------------*/
gremsy_thread_t* gremsy_thread_init(void);
gremsy_thread_t* gremsy_thread(void);
#ifdef __cplusplus
}
#endif
#endif /* __GREMSY_COMON_H */
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
