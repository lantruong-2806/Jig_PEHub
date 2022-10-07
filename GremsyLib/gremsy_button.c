/** 
  ******************************************************************************
  * @file    gremsy_button.c
  * @author  Gremsy Team
  * @version V1.0.0
  * @date    18-Aug-2020
  * @brief   
  *
  ******************************************************************************
  * @par 
  * COPYRIGHT NOTICE: (c) 2020 Gremsy.  
  * All rights reserved.
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or 
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "gremsy_button.h"
#include "gremsy_thread.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static int8_t   clicks;                 // button click counts to return
static bool     depressed;              // the currently debounced button (press) state (presumably it is not sad :)
static uint32_t debounceTime;
static uint32_t multiclickTime;
static uint32_t ClickTime;
//static uint32_t longClickTime;
//static bool     changed;
static bool     _activeHigh;            // Type of button: Active-low = 0 or active-high = 1
static bool     _btnState;              // Current appearant button state
static bool     _lastState;             // previous button reading
static int8_t   _clickCount;            // Number of button clicks within multiclickTime milliseconds
static uint32_t _lastBounceTime;        // the last time the button input pin was toggled, due to noise or a press

static gremsy_button_t* button_ret;
static uint32_t time_process;
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
static int8_t button_get_clicks(void)
{
    return clicks;
}

static void button_process(void)
{
    uint32_t now = gremsy_thread()->get_ms(&time_process);      // get current time
    _btnState = GET_BUTTON; // current appearant button state

    // Make the button logic active-high in code
    if (!_activeHigh)
    {
        _btnState = !_btnState;
    }

    // If the switch changed, due to noise or a button press, reset the debounce timer
    if (_btnState != _lastState)
    {
        _lastBounceTime = now;
    }


    // debounce the button (Check if a stable, changed state has occured)
    if (now - _lastBounceTime > debounceTime && _btnState != depressed)
    {
        depressed = _btnState;
        if (depressed)
        {
            _clickCount++;
        }
    }

    _lastState = _btnState;

    // If the button released state is stable, report nr of clicks and start new cycle
    if (!depressed && (now - _lastBounceTime) > multiclickTime)
    {
        // positive count for released buttons
        clicks = _clickCount;
        _clickCount = 0;
    }

    // Check for "long click"
    if (depressed && (now - _lastBounceTime > ClickTime))
    {
        // negative count for long clicks
        clicks = 0 - _clickCount;
        _clickCount = 0;
    }
}

gremsy_button_t* gremsy_button_init(void)
{
    //cap phat vung nho cho thu vien
    button_ret = calloc(sizeof(gremsy_button_t), 1);
    
    //gan dia chi cho thu vien
    button_ret->get_clicks  = button_get_clicks;
    button_ret->process     = button_process;
  
    //Khoi tao cac gia tri cho nut nhan
    _activeHigh    = BUTTON_ACTIVE_LEVEL;
    _btnState      = !_activeHigh;  // initial button state in active-high logic
    _lastState     = _btnState;
    _clickCount    = 0;
    clicks         = 0;
    depressed      = 0;
    _lastBounceTime= 0;
    debounceTime   = BUTTON_DEBOUNCE_TIME;            // Debounce timer in ms
    multiclickTime = BUTTON_MULTICLICK_TIME;          // Time limit for multi clicks
//    longClickTime  = BUTTON_LONGCLICK_TIME;           // time until "long" click register
    ClickTime      = 1000;                            // time until "uint32_t" click register
//    changed        = false;
    //tra ve ket qua
    return button_ret;
}

gremsy_button_t* gremsy_button(void)
{
  return button_ret;
}
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
