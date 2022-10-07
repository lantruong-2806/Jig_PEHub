/**
  ******************************************************************************
  * @file gremsy_common.c
  * @author Gremsy Team
  * @version V1.0.0
  * @date 09-May-2018
  * @brief
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2016 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes
------------------------------------------------------------------------------*/
//#include "gremsy_common.h"
#include "gremsy_thread.h"
#include "main.h"
#include "stdlib.h"
/* Private typedef
------------------------------------------------------------------------------*/
/* Private define
------------------------------------------------------------------------------*/
/* Private macro
------------------------------------------------------------------------------*/
/* Private variables
------------------------------------------------------------------------------*/
static gremsy_thread_t* thread_private;
static uint32_t tick_count;
extern TIM_HandleTypeDef htim2;
/* Private function prototypes
------------------------------------------------------------------------------*/
/* Private functions
------------------------------------------------------------------------------*/
/** @brief      ham dung de doc gia tri tick us cua systick time
    @return     gia tri cua tick ms cua time
 */
static uint32_t gremsy_os_thread_get_tick (void)
{
//    gp_gremsy_os_thread_private->count = (uint32_t)((((uint32_t)gp_gremsy_os_thread_private->it_count<<24)&0xFF000000) | (0x1000000 - (uint32_t)SysTick->VAL));
//    
//    return gp_gremsy_os_thread_private->count;
//    tick_count = uwTick*1000 + TIM16->CNT;
//    tick_count = uwTick*1000 + ((uint32_t)SysTick->LOAD - (uint32_t)SysTick->VAL)/169;
//    tick_count = (uint32_t)((((uint32_t)uwTick<<24)&0xFF000000) | (0x1000000 - (uint32_t)SysTick->VAL));
    tick_count = TIM2->CNT;
  
    return tick_count;
}
/** @brief      ham dung de doc gia tri tick ms cua systick time
    @return     gia tri cua tick ms cua time
 */
static uint32_t gremsy_os_thread_get_tick_ms (void)
{
//    return uwTick;
    return TIM2->CNT/1000;
}
/** @brief      ham reset bien thoi gian theo bo dem
    @param[out] p_time con tro cua bien can reset
    @return     none.
*/
static void gremsy_os_thread_reset (uint32_t* ptime)
{
    uint32_t tick  = gremsy_os_thread_get_tick();
    
//    int32_t  delta = tick - *ptime;
//    
//    /// kiem tra gia tri loi
//    while(delta < 0)
//    {
//        /// cong tim gia tri dung cho tick
//        tick += 0x1000000;
//        delta = tick - *ptime;
//    }
    
    /// gan gia tri vao dia chi
    *ptime = tick;
}

/** @brief      ham doc gia tri thoi gian trong thread theo us
    @param[in]  p_time dia chi con tro luu thoi gian truoc do
    @return     thoi gian khi bat dau ham reset toi ham nay
    @note       ham nay duoc su dung chung voi ham gremsy_thread_time_reset
*/
static uint32_t gremsy_os_thread_get (uint32_t* ptime)
{
//    uint32_t tick  = gremsy_os_thread_get_tick();
//    int32_t  delta = tick - *ptime;
//    
//    /// kiem tra gia tri loi
//    while(delta < 0)
//    {
//        delta += 1000;
//    }
//    
//    return delta;
  
    //Su dung timer 32 bit de dem micros
    uint32_t tick  = gremsy_os_thread_get_tick();
    uint32_t  delta = tick - *ptime;
    
    return delta;
}

/** @brief      ham doc gia tri thoi gian trong thread theo ms
    @param[in]  p_time dia chi con tro luu thoi gian truoc do
    @return     thoi gian khi bat dau ham reset toi ham nay
    @note       ham nay duoc su dung chung voi ham gremsy_thread_time_reset
*/
static uint32_t gremsy_os_thread_get_ms (uint32_t* ptime)
{
    uint32_t time = gremsy_os_thread_get(ptime);
    
    return time/1000;
}

/** @brief      ham doc gia tri 1 vong thoi gian trong thread theo ms
    @param[in]  p_time dia chi con tro luu thoi gian truoc do
    @return     thoi gian khi bat dau va quay lai ham nay
    @note       ham nay khong duoc su dung chung voi ham gremsy_thread_time_reset
*/
static uint32_t gremsy_os_thread_getloop_ms (uint32_t* ptime)
{
    uint32_t time = gremsy_os_thread_get(ptime);
    
    /// reset time
    gremsy_os_thread_reset(ptime);
    
    return time/1000;
}

/** @brief      delay mot khoang thoi gian tinh bang ms.
    @param[in]  time_ms thoi gian delay tinh bang ms.
    @return     none.
*/
static void gremsy_os_thread_sleep_ms (uint32_t time_ms)
{
    uint32_t lastTime = gremsy_os_thread_get_tick();
    
    while (gremsy_os_thread_get_ms(&lastTime) < time_ms)
    {
        /** @NOTE khoang th�?i gian đợi hàm sẽ dừng lại tại đây */
    }
}

/** @brief      ham dung de tao mot vong lap với th�?i gian cài đặt theo ms
    @param[in]  p_time dia chi con tro luu gia tri thoi gian.
    @param[in]  time_ms thoi gian tro lai vong lap tinh bang ms.
    @return     true thoi gian da du de tro lai 1 vong lap,
                false thoi gian chua du de tro lai 1 vong lap.
*/
static bool gremsy_os_thread_loop_ms (uint32_t* ptime, uint32_t time_ms)
{
    bool ret = false;
    
    if (gremsy_os_thread_get_ms(ptime) >= time_ms)
    {
        gremsy_os_thread_reset(ptime);
        
        ret = true;
    }
    
    return ret;
}

/** @brief      ham doc gia tri thoi gian trong thread theo us
    @param[in]  p_time dia chi con tro luu thoi gian truoc do
    @return     thoi gian khi bat dau ham reset toi ham nay
    @note       ham nay duoc su dung chung voi ham gremsy_thread_time_reset
*/
static uint32_t gremsy_os_thread_get_us (uint32_t* ptime)
{
    uint32_t time = gremsy_os_thread_get(ptime);
    
    return time/1;
}

/** @brief      ham doc gia tri 1 vong thoi gian trong thread theo us
    @param[in]  p_time dia chi con tro luu thoi gian truoc do
    @return     thoi gian khi bat dau va quay lai ham nay
    @note       ham nay khong duoc su dung chung voi ham gremsy_thread_time_reset
*/
static uint32_t gremsy_os_thread_getloop_us (uint32_t* ptime)
{
    uint32_t time = gremsy_os_thread_get(ptime);
    
    /// reset time de tao loop
    gremsy_os_thread_reset(ptime);
    
    return time/1;
}

/** @brief      delay mot khoang thoi gian tính bằng us.
    @param[in]  time_ms thoi gian delay tinh bang us.
    @return     none.
*/
static void gremsy_os_thread_sleep_us (uint32_t time_ms)
{
    uint32_t lastTime = gremsy_os_thread_get_tick();
    
    while (gremsy_os_thread_get_us(&lastTime) < time_ms)
    {
        /** @NOTE khoang th�?i gian đợi hàm sẽ dừng lại tại đây */
    }
}

/** @brief      ham dung de tao mot vong lap với th�?i gian cài đặt theo us
    @param[in]  p_time dia chi con tro luu gia tri thoi gian.
    @param[in]  time_ms thoi gian tro lai vong lap tinh bang us.
    @return     true thoi gian da du de tro lai 1 vong lap,
                false thoi gian chua du de tro lai 1 vong lap.
*/
static bool gremsy_os_thread_loop_us (uint32_t* ptime, uint32_t time_ms)
{
    bool ret = false;
    
    if (gremsy_os_thread_get_us(ptime) >= time_ms)
    {
        gremsy_os_thread_reset(ptime);
        
        ret = true;
    }
    
    return ret;
}

/** @brief delay mot khoang thoi gian cho cac ham init.
    @param none
    @ret   none.
*/
//static void gremsy_os_thread_sleep_init (void)
//{
//    gremsy_os_thread_private_t* pthread = gp_gremsy_os_thread_private;
//    uint32_t lastTime = gremsy_os_thread_get_tick();
//    
//    while (gremsy_os_thread_get_ms(&lastTime) < pthread->sleep_time_init)
//    {
//        /** @NOTE khoang th�?i gian đợi hàm sẽ dừng lại tại đây */
//    }
//}

gremsy_thread_t* gremsy_thread_init(void)
{
  //init timer for 1 us
  //=> Init in main    
    /// cai dat systick time full count
//    if (SysTick_Config(0x1000000) != 0)
//    {
//        /** @NOTE khi systick config khong cai dat duoc */
//        while(1);
//    }
    
    //Disable Systick
//    SysTick->CTRL = 0;
  
    //Enable TIM2 for micro second timer
    __HAL_TIM_ENABLE(&htim2);
  
    //cap phat vung nho cho thu vien
    thread_private = calloc(sizeof(gremsy_thread_t), 1);
  
    thread_private->get_tick_us   = gremsy_os_thread_get_tick;
    thread_private->sleep_us      = gremsy_os_thread_sleep_us;
    thread_private->getloop_us    = gremsy_os_thread_getloop_us;
    thread_private->loop_us       = gremsy_os_thread_loop_us;
    thread_private->get_us        = gremsy_os_thread_get_us;
    
    thread_private->get_tick_ms   = gremsy_os_thread_get_tick_ms;
    thread_private->sleep_ms      = gremsy_os_thread_sleep_ms;
    thread_private->getloop_ms    = gremsy_os_thread_getloop_ms;
    thread_private->loop_ms       = gremsy_os_thread_loop_ms;
    thread_private->get_ms        = gremsy_os_thread_get_ms;
    thread_private->reset         = gremsy_os_thread_reset;
  
  //tra ve con tro
  return thread_private;
}
gremsy_thread_t* gremsy_thread(void)
{
  //tra ve con tro
  return thread_private;
}
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
