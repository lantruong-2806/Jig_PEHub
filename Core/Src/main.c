/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "gremsy_ssd1306.h"
#include "gremsy_button.h"
#include "gremsy_thread.h"
#include "ugui.h"
#include "math.h"
#include "gremsy_gpio_fast.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PCF8575_ADDR    (0x20 << 1)
#define PIN_CHECK_MAX_SIZE  50
/** @brief      struct quy dinh kieu chan can test
*/
typedef enum
{
    PIN_TYPE_NONE = 0,  //TO CHECK PIN IS INIT OK
    PIN_TYPE_DIGITAL,   //READ 2 CONNECTED PIN
    PIN_TYPE_ANALOG,    //MEASURE ANALOG
    PIN_TYPE_GND,       //ALWAY IN LOW LEVEL
    PIN_TYPE_POWER,     //ALWAY IN HIGH LEVEL
    PIN_TYPE_NOCONNECT, //NO CONNECT TO ANY PIN
	PIN_TYPE_EX,		//CONNECT TO IO EXPANDER PCF8575
}pin_type_t;

typedef enum
{
    PIN_CHECK_STATE_IDLE,
    PIN_CHECK_STATE_INIT,
    PIN_CHECK_STATE_PROCESS,
    PIN_CHECK_STATE_OK,
    PIN_CHECK_STATE_ERROR,
}pin_check_state_t;

/** @brief      struct quy dinh du lieu cua pin check
*/
typedef struct
{
    //Thuoc tinh chung
    pin_type_t  pin_type;
    char     pin_name[20];
    char     error_log[20];

    //thuoc tinh cua analog pin: bo test aevo slipring ko xai
    struct
    {
        uint16_t*   p_analog_val;
        double      analog_ratio;
        double      voltage_normal;
        double      voltage_delta;
        double      voltage_offset;
    }type_analog;

    //Thuoc tinh cua digital pin
    struct
    {
        GPIO_TypeDef*  gpio_out_port;
        uint16_t      gpio_out_pin;
        GPIO_TypeDef*  gpio_in_port;
        uint16_t      gpio_in_pin;
    }type_digital;

    //Thuoc tinh cua gnd pin
    struct
    {
        GPIO_TypeDef*  gpio_gnd_port;
        uint16_t      gpio_gnd_pin;
    }type_gnd;

    //Thuoc tinh cua power pin
    struct
    {
        GPIO_TypeDef*  gpio_power_port;
        uint16_t      gpio_power_pin;
    }type_power;

    //Thuoc tinh cua no connect pin
    struct
    {
        GPIO_TypeDef*  gpio_nc_port;
        uint16_t      gpio_nc_pin;
    }type_nc;

    // thuoc tinh cua pin ex ngoai
    struct
	{
    	uint16_t GPIO_OutExPinNum;
        GPIO_TypeDef*  gpio_in_port;
        uint16_t      gpio_in_pin;
	}type_ex_gpio;
}pin_check_t;

/** @brief      struct quy dinh pin check
*/

typedef struct
{
    uint8_t             count;
    uint8_t             size;
    pin_check_t         array[PIN_CHECK_MAX_SIZE];
    pin_check_state_t   state;
}pin_check_data_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAL_LINE_OLED(line, pixel) (line*pixel)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
UG_GUI gui;
UG_WINDOW window_1;
gremsy_thread_t*    thread;
gremsy_button_t*    button;

//Bien luu bo dem cho oled
char oled_buff[50];
static uint16_t analog_buff[14];

//bien check gia tri cua oled
uint8_t Buffer[25] = {0};
uint8_t Space[] = "-";
uint8_t StartMSG[] = "Starting I2C scanning! \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";

static pin_check_data_t pin_check_data;

static pin_check_t  	com5v,
						comGnd,
						can5v,
						canGnd,
						sbppm5v,
						sbppmGnd,
						spek5V,
						spekGnd,
						usb5v,
						usbGnd,
						vcc1,
						vcc2,
						vcc3,
						pgnd1,
						pgnd2,
						pgnd3;
static pin_check_t      uartTx,
						uartRx,
						ethRxn,
						ethRxp,
						ethTxn,
						ethTxp,
						usbDp,
						usbDn,
						canL,
						canH,
						gpsPps,
						capture,
						trig,
						ffcUartTx,
						ffcUartRx,
						ffcCanH,
						ffcCanL,
						ffcEthTxp,
						ffcEthTxn,
						ffcEthRxp,
						ffcEthRxn,
						ffcusbDp,
						ffcusbDn;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void PCF8575_write_pin(uint16_t GPIO_Pin, GPIO_PinState PinState);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** @brief      Ham them 1 connector vao de test
    @param[in]  Thuoc tinh cua connector
    @return     None
*/
static void pin_check_add(pin_check_t pin_check)
{
    if(pin_check_data.size < PIN_CHECK_MAX_SIZE)
    {
        pin_check_data.array[pin_check_data.size] = pin_check;
        pin_check_data.size ++;
    }
    else
    {
        //Thong bao loi
    }
}
/** @brief      Ham init pin check
    @param[in]  None
    @return     None
*/
static void pin_check_init(void)
{
     pin_check_data.size = 0;
     pin_check_data.count = 0;
     pin_check_data.state = PIN_CHECK_STATE_IDLE;
     //init pin test
     //##########GIMBAL CONTROL connector##########

     strcpy(ffcUartTx.pin_name, "FFC UartTx");
     strcpy(ffcUartTx.error_log, "ERR FFC_UartTx");
     ffcUartTx.pin_type = PIN_TYPE_DIGITAL;
     ffcUartTx.type_digital.gpio_in_port  = IN_COM_TX_GPIO_Port;
     ffcUartTx.type_digital.gpio_in_pin   = IN_COM_TX_Pin;
     ffcUartTx.type_digital.gpio_out_port = OUT_FFC_UART_TX_GPIO_Port;
     ffcUartTx.type_digital.gpio_out_pin  = OUT_FFC_UART_TX_Pin;

     strcpy(ffcUartRx.pin_name, "FFC UartRx");
     strcpy(ffcUartRx.error_log, "ERR FFC_UartRx");
     ffcUartRx.pin_type = PIN_TYPE_DIGITAL;
     ffcUartRx.type_digital.gpio_in_port  = IN_COM_RX_GPIO_Port;
     ffcUartRx.type_digital.gpio_in_pin   = IN_COM_RX_Pin;
     ffcUartRx.type_digital.gpio_out_port = OUT_FFC_UART_RX_GPIO_Port;
     ffcUartRx.type_digital.gpio_out_pin  = OUT_FFC_UART_RX_Pin;
//
     strcpy(ffcCanH.pin_name, "FFC CanH");
     strcpy(ffcCanH.error_log, "ERR FFC_CanH");
     ffcCanH.pin_type = PIN_TYPE_DIGITAL;
     ffcCanH.type_digital.gpio_in_port  = IN_CANH_GPIO_Port;
     ffcCanH.type_digital.gpio_in_pin   = IN_CANH_Pin;
     ffcCanH.type_digital.gpio_out_port = OUT_FFC_CANH_GPIO_Port;
     ffcCanH.type_digital.gpio_out_pin  = OUT_FFC_CANH_Pin;

     strcpy(ffcCanL.pin_name, "FFC ffcCanL");
     strcpy(ffcCanL.error_log, "ERR FFC_CanL");
     ffcCanL.pin_type = PIN_TYPE_DIGITAL;
     ffcCanL.type_digital.gpio_in_port  = IN_CANL_GPIO_Port;
     ffcCanL.type_digital.gpio_in_pin   = IN_CANL_Pin;
     ffcCanL.type_digital.gpio_out_port = OUT_FFC_CANL_GPIO_Port;
     ffcCanL.type_digital.gpio_out_pin  = OUT_FFC_CANL_Pin;

     strcpy(ffcEthTxp.pin_name, "FFC ffcEthTxp");
     strcpy(ffcEthTxp.error_log, "ERR FFC_EthTxp");
     ffcEthTxp.pin_type = PIN_TYPE_DIGITAL;
     ffcEthTxp.type_digital.gpio_in_port  = IN_ETH_TXP_GPIO_Port;
     ffcEthTxp.type_digital.gpio_in_pin   = IN_ETH_TXP_Pin;
     ffcEthTxp.type_digital.gpio_out_port = OUT_FFC_ETH_TXP_GPIO_Port;
     ffcEthTxp.type_digital.gpio_out_pin  = OUT_FFC_ETH_TXP_Pin;

     strcpy(ffcEthTxn.pin_name, "FFC ffcEthTxn");
     strcpy(ffcEthTxn.error_log, "ERR FFC_EthTxn");
     ffcEthTxn.pin_type = PIN_TYPE_DIGITAL;
     ffcEthTxn.type_digital.gpio_in_port  = IN_ETH_TXN_GPIO_Port;
     ffcEthTxn.type_digital.gpio_in_pin   = IN_ETH_TXN_Pin;
     ffcEthTxn.type_digital.gpio_out_port = OUT_FFC_ETH_TXN_GPIO_Port;
     ffcEthTxn.type_digital.gpio_out_pin  = OUT_FFC_ETH_TXN_Pin;

     strcpy(ffcEthRxp.pin_name, "FFC EthRxp");
     strcpy(ffcEthRxp.error_log, "ERR FFC_EthRxp");
     ffcEthRxp.pin_type = PIN_TYPE_DIGITAL;
     ffcEthRxp.type_digital.gpio_in_port  = IN_ETH_RXP_GPIO_Port;
     ffcEthRxp.type_digital.gpio_in_pin   = IN_ETH_RXP_Pin;
     ffcEthRxp.type_digital.gpio_out_port = OUT_FFC_ETH_RXP_GPIO_Port;
     ffcEthRxp.type_digital.gpio_out_pin  = OUT_FFC_ETH_RXP_Pin;

     strcpy(ffcEthRxn.pin_name, "FFC ethRxn");
     strcpy(ffcEthRxn.error_log, "ERR FFC_ethRxn");
     ffcEthRxn.pin_type = PIN_TYPE_DIGITAL;
     ffcEthRxn.type_digital.gpio_in_port  = IN_ETH_RXN_GPIO_Port;
     ffcEthRxn.type_digital.gpio_in_pin   = IN_ETH_RXN_Pin;
     ffcEthRxn.type_digital.gpio_out_port = OUT_FFC_ETH_RXN_GPIO_Port;
     ffcEthRxn.type_digital.gpio_out_pin  = OUT_FFC_ETH_RXN_Pin;

     strcpy(ffcusbDp.pin_name, "FFC usbDp");
     strcpy(ffcusbDp.error_log, "ERR FFC_usbDp");
     ffcusbDp.pin_type = PIN_TYPE_DIGITAL;
     ffcusbDp.type_digital.gpio_in_port  = IN_USB_DP_GPIO_Port;
     ffcusbDp.type_digital.gpio_in_pin   = IN_USB_DP_Pin;
     ffcusbDp.type_digital.gpio_out_port = OUT_FFC_USB_DP_GPIO_Port;
     ffcusbDp.type_digital.gpio_out_pin  = OUT_FFC_USB_DP_Pin;

     strcpy(ffcusbDn.pin_name, "FFC usbDn");
     strcpy(ffcusbDn.error_log, "ERR FFC_usbDn");
     ffcusbDn.pin_type = PIN_TYPE_DIGITAL;
     ffcusbDn.type_digital.gpio_in_port  = IN_USB_DN_GPIO_Port;
     ffcusbDn.type_digital.gpio_in_pin   = IN_USB_DN_Pin;
     ffcusbDn.type_digital.gpio_out_port = OUT_FFC_USB_DN_GPIO_Port;
     ffcusbDn.type_digital.gpio_out_pin  = OUT_FFC_USB_DN_Pin;

    //##########CONNECTOR PIN##########
     strcpy(uartTx.pin_name, "FFC PEHUB uartTx");
     strcpy(uartTx.error_log, "ERR PEHUB uartTx");
     uartTx.pin_type = PIN_TYPE_EX;
     uartTx.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_UART_TX;
     uartTx.type_ex_gpio.gpio_in_port 		= IN_COM_TX_GPIO_Port;
     uartTx.type_ex_gpio.gpio_in_pin		= IN_COM_TX_Pin;

     strcpy(uartRx.pin_name, "FFC PEHUB uartRx");
     strcpy(uartRx.error_log, "ERR PEHUB uartRx");
     uartRx.pin_type = PIN_TYPE_EX;
     uartRx.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_UART_RX;
     uartRx.type_ex_gpio.gpio_in_port 		= IN_COM_RX_GPIO_Port; //IN_COM_RX_GPIO_Port
     uartRx.type_ex_gpio.gpio_in_pin		= IN_COM_RX_Pin;

     strcpy(ethRxn.pin_name, "FFC PEHUB ethRxn");
     strcpy(ethRxn.error_log, "ERR PEHUB ethRxn");
     ethRxn.pin_type = PIN_TYPE_EX;
     ethRxn.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_ETHRXN;
     ethRxn.type_ex_gpio.gpio_in_port 		= IN_ETH_RXN_GPIO_Port;
     ethRxn.type_ex_gpio.gpio_in_pin		= IN_ETH_RXN_Pin;

     strcpy(ethRxp.pin_name, "FFC PEHUB ethRxp");
     strcpy(ethRxp.error_log, "ERR PEHUB ethRxp");
     ethRxp.pin_type = PIN_TYPE_EX;
     ethRxp.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_ETHRXP;
     ethRxp.type_ex_gpio.gpio_in_port 		= IN_ETH_RXP_GPIO_Port;
     ethRxp.type_ex_gpio.gpio_in_pin		= IN_ETH_RXP_Pin;

     strcpy(ethTxn.pin_name, "FFC PEHUB ethTxn");
     strcpy(ethTxn.error_log, "ERR PEHUB ethTxn");
     ethTxn.pin_type = PIN_TYPE_EX;
     ethTxn.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_ETHTXN;
     ethTxn.type_ex_gpio.gpio_in_port 		= IN_ETH_TXN_GPIO_Port;
     ethTxn.type_ex_gpio.gpio_in_pin		= IN_ETH_TXN_Pin;

     strcpy(ethTxp.pin_name, "FFC PEHUB ethTxp");
     strcpy(ethTxp.error_log, "ERR PEHUB ethTxp");
     ethTxp.pin_type = PIN_TYPE_EX;
     ethTxp.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_ETHTXP;
     ethTxp.type_ex_gpio.gpio_in_port 		= IN_ETH_TXP_GPIO_Port;
     ethTxp.type_ex_gpio.gpio_in_pin		= IN_ETH_TXP_Pin;

     strcpy(usbDp.pin_name, "FFC PEHUB usbDp");
     strcpy(usbDp.error_log, "ERR PEHUB usbDp");
     usbDp.pin_type = PIN_TYPE_EX;
     usbDp.type_ex_gpio.GPIO_OutExPinNum    = IO_EX_USB_DP;
     usbDp.type_ex_gpio.gpio_in_port 		= IN_USB_DP_GPIO_Port;
     usbDp.type_ex_gpio.gpio_in_pin			= IN_USB_DP_Pin;

     strcpy(usbDn.pin_name, "FFC PEHUB usbDn");
     strcpy(usbDn.error_log, "ERR PEHUB usbDn");
     usbDn.pin_type = PIN_TYPE_EX;
     usbDn.type_ex_gpio.GPIO_OutExPinNum    = IO_EX_USB_DN;
     usbDn.type_ex_gpio.gpio_in_port 		= IN_USB_DN_GPIO_Port;
     usbDn.type_ex_gpio.gpio_in_pin		    = IN_USB_DN_Pin;

     strcpy(canL.pin_name, "FFC PEHUB canL");
     strcpy(canL.error_log, "ERR PEHUB canL");
     canL.pin_type = PIN_TYPE_EX;
     canL.type_ex_gpio.GPIO_OutExPinNum   	= IO_EX_CANL;
     canL.type_ex_gpio.gpio_in_port 		= IN_CANL_GPIO_Port;
     canL.type_ex_gpio.gpio_in_pin			= IN_CANL_Pin;

     strcpy(canH.pin_name, "FFC PEHUB canH");
     strcpy(canH.error_log, "ERR PEHUB canH");
     canH.pin_type = PIN_TYPE_EX;
     canH.type_ex_gpio.GPIO_OutExPinNum     = IO_EX_CANH;
     canH.type_ex_gpio.gpio_in_port 		= IN_CANH_GPIO_Port;
     canH.type_ex_gpio.gpio_in_pin		    = IN_CANH_Pin;

     strcpy(gpsPps.pin_name, "FFC PEHUB gpsPps");
     strcpy(gpsPps.error_log, "ERR PEHUB gpsPps");
     gpsPps.pin_type = PIN_TYPE_EX;
     gpsPps.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_GPS_PPS;
     gpsPps.type_ex_gpio.gpio_in_port 		= IN_GPS_PPS_GPIO_Port;
     gpsPps.type_ex_gpio.gpio_in_pin		= IN_GPS_PPS_Pin;

     strcpy(capture.pin_name, "FFC PEHUB capture");
     strcpy(capture.error_log, "ERR PEHUB capture");
     capture.pin_type = PIN_TYPE_EX;
     capture.type_ex_gpio.GPIO_OutExPinNum   = IO_EX_CAPTURE;
     capture.type_ex_gpio.gpio_in_port 		 = IN_CAPTURE_GPIO_Port;
     capture.type_ex_gpio.gpio_in_pin		 = IN_CAPTURE_Pin;

     strcpy(trig.pin_name, "FFC PEHUB trig/spek");
     strcpy(trig.error_log, "ERR PEHUB trig/spek");
     trig.pin_type = PIN_TYPE_EX;
     trig.type_ex_gpio.GPIO_OutExPinNum      = IO_EX_TRIG;
     trig.type_ex_gpio.gpio_in_port 		 = IN_TRIG_GPIO_Port;
     trig.type_ex_gpio.gpio_in_pin		     = IN_TRIG_Pin;

    //##########INIT POWER PIN##########
 	strcpy(com5v.pin_name, "com5v");
 	strcpy(com5v.error_log, "ERR com5v");
 	com5v.pin_type = PIN_TYPE_ANALOG;
 	com5v.type_analog.p_analog_val  = &analog_buff[0];
 	com5v.type_analog.analog_ratio   = 17.0/2.0;
 	com5v.type_analog.voltage_normal = 5.0;
 	com5v.type_analog.voltage_delta  = 0.2;
 	com5v.type_analog.voltage_offset = 0.2;

 	strcpy(comGnd.pin_name, "comGnd");
 	strcpy(comGnd.error_log, "ERR comGnd");
 	comGnd.pin_type = PIN_TYPE_ANALOG;
 	comGnd.type_analog.p_analog_val  = &analog_buff[1];
 	comGnd.type_analog.analog_ratio   = 17.0/2.0;
 	comGnd.type_analog.voltage_normal = 0.0;
 	comGnd.type_analog.voltage_delta  = 0.1;
 	comGnd.type_analog.voltage_offset = 0.0;

 	strcpy(can5v.pin_name, "can5v");
 	strcpy(can5v.error_log, "ERR can5v");
 	can5v.pin_type = PIN_TYPE_ANALOG;
 	can5v.type_analog.p_analog_val  = &analog_buff[2];
 	can5v.type_analog.analog_ratio   = 17.0/2.0;
 	can5v.type_analog.voltage_normal = 5.0;
 	can5v.type_analog.voltage_delta  = 0.2;
 	can5v.type_analog.voltage_offset = 0.2;

 	strcpy(canGnd.pin_name, "canGnd");
 	strcpy(canGnd.error_log, "ERR canGnd");
 	canGnd.pin_type = PIN_TYPE_ANALOG;
 	canGnd.type_analog.p_analog_val  = &analog_buff[3];
 	canGnd.type_analog.analog_ratio   = 17.0/2.0;
 	canGnd.type_analog.voltage_normal = 0.0;
 	canGnd.type_analog.voltage_delta  = 0.1;
 	canGnd.type_analog.voltage_offset = 0.0;

 	strcpy(sbppmGnd.pin_name, "sbppmGnd");
 	strcpy(sbppmGnd.error_log, "ERR sbppmGnd");
 	sbppmGnd.pin_type = PIN_TYPE_ANALOG;
 	sbppmGnd.type_analog.p_analog_val  = &analog_buff[5];
 	sbppmGnd.type_analog.analog_ratio   = 17.0/2.0;
 	sbppmGnd.type_analog.voltage_normal = 0.0;
 	sbppmGnd.type_analog.voltage_delta  = 0.1;
 	sbppmGnd.type_analog.voltage_offset = 0.0;

 	strcpy(sbppm5v.pin_name, "sbppm5v");
 	strcpy(sbppm5v.error_log, "ERR sbppm5v");
 	sbppm5v.pin_type = PIN_TYPE_ANALOG;
 	sbppm5v.type_analog.p_analog_val  = &analog_buff[4]; // schematic check
 	sbppm5v.type_analog.analog_ratio   = 17.0/2.0;
 	sbppm5v.type_analog.voltage_normal = 5.0;
 	sbppm5v.type_analog.voltage_delta  = 0.2;
 	sbppm5v.type_analog.voltage_offset = 0.2;

 	strcpy(spek5V.pin_name, "spek3V3");
 	strcpy(spek5V.error_log, "ERR spek3V3");
 	spek5V.pin_type = PIN_TYPE_ANALOG;
 	spek5V.type_analog.p_analog_val  = &analog_buff[6];
 	spek5V.type_analog.analog_ratio   = 17.0/2.0;
 	spek5V.type_analog.voltage_normal = 3.3;
 	spek5V.type_analog.voltage_delta  = 0.2;
 	spek5V.type_analog.voltage_offset = 0.2;

 	strcpy(spekGnd.pin_name, "spekGnd");
 	strcpy(spekGnd.error_log, "ERR spekGnd");
 	spekGnd.pin_type = PIN_TYPE_ANALOG;
 	spekGnd.type_analog.p_analog_val  = &analog_buff[7];
 	spekGnd.type_analog.analog_ratio   = 17.0/2.0;
 	spekGnd.type_analog.voltage_normal = 0.0;
 	spekGnd.type_analog.voltage_delta  = 0.1;
 	spekGnd.type_analog.voltage_offset = 0.0;

 	strcpy(usb5v.pin_name, "usb5v");
 	strcpy(usb5v.error_log, "ERR usb5v");
 	usb5v.pin_type = PIN_TYPE_ANALOG;
 	usb5v.type_analog.p_analog_val  = &analog_buff[8];
 	usb5v.type_analog.analog_ratio   = 17.0/2.0;
 	usb5v.type_analog.voltage_normal = 5.0;
 	usb5v.type_analog.voltage_delta  = 0.2;
 	usb5v.type_analog.voltage_offset = 0.2;

 	strcpy(usbGnd.pin_name, "usbGnd");
 	strcpy(usbGnd.error_log, "ERR usbGnd");
 	usbGnd.pin_type = PIN_TYPE_ANALOG;
 	usbGnd.type_analog.p_analog_val  = &analog_buff[9];
 	usbGnd.type_analog.analog_ratio   = 17.0/2.0;
 	usbGnd.type_analog.voltage_normal = 0.0;
 	usbGnd.type_analog.voltage_delta  = 0.1;
 	usbGnd.type_analog.voltage_offset = 0.0;

 	strcpy(vcc1.pin_name, "linear vcc1");
 	strcpy(vcc1.error_log, "ERR linear vcc1");
 	vcc1.pin_type = PIN_TYPE_ANALOG;
 	vcc1.type_analog.p_analog_val  = &analog_buff[10];
 	vcc1.type_analog.analog_ratio   = 17.0/2.0;
 	vcc1.type_analog.voltage_normal = 5.0;
 	vcc1.type_analog.voltage_delta  = 0.2;
 	vcc1.type_analog.voltage_offset = 0.2;

 	strcpy(vcc2.pin_name, "linear vcc2");
 	strcpy(vcc2.error_log, "ERR linear vcc2");
 	vcc2.pin_type = PIN_TYPE_ANALOG;
 	vcc2.type_analog.p_analog_val  = &analog_buff[11];
 	vcc2.type_analog.analog_ratio   = 17.0/2.0;
 	vcc2.type_analog.voltage_normal = 5.0;
 	vcc2.type_analog.voltage_delta  = 0.2;
 	vcc2.type_analog.voltage_offset = 0.2;

 	strcpy(vcc3.pin_name, "linear vcc3");
 	strcpy(vcc3.error_log, "ERR linear vcc3");
 	vcc3.pin_type = PIN_TYPE_ANALOG;
 	vcc3.type_analog.p_analog_val  = &analog_buff[12];
 	vcc3.type_analog.analog_ratio   = 17.0/2.0;
 	vcc3.type_analog.voltage_normal = 5.0;
 	vcc3.type_analog.voltage_delta  = 0.2;
 	vcc3.type_analog.voltage_offset = 0.2;

 	strcpy(pgnd1.pin_name, "linear gnd1");  ///GND LINEAR CABLE
 	strcpy(pgnd1.error_log, "ERR linear gnd1");
 	pgnd1.pin_type = PIN_TYPE_ANALOG;
 	pgnd1.type_analog.p_analog_val  = &analog_buff[13];
 	pgnd1.type_analog.analog_ratio   = 17.0/2.0;
 	pgnd1.type_analog.voltage_normal = 0.0;
 	pgnd1.type_analog.voltage_delta  = 0.1;
 	pgnd1.type_analog.voltage_offset = 0.0;

    strcpy(pgnd2.pin_name, "FFC pgnd2");
    strcpy(pgnd2.error_log, "ERR pgnd2");
    pgnd2.pin_type = PIN_TYPE_GND;
    pgnd2.type_gnd.gpio_gnd_port  = IN_PGND2_GPIO_Port;
    pgnd2.type_gnd.gpio_gnd_pin  = IN_PGND2_Pin;

    strcpy(pgnd3.pin_name, "FFC pgnd3");
    strcpy(pgnd3.error_log, "ERR pgnd3");
    pgnd3.pin_type = PIN_TYPE_GND;
    pgnd3.type_gnd.gpio_gnd_port  = IN_PGND3_GPIO_Port;
    pgnd3.type_gnd.gpio_gnd_pin  = IN_PGND3_Pin;

}
/** @brief      ham xu ly man hinh
    @param[in]  none
    @return     none
*/
static void screen_process(void)
{
	static uint32_t time_screen_process;
	static uint8_t state_icon;
    char buff[30];
	UG_FontSelect(&FONT_12X16);
	uint16_t len = strlen("GREMSY")* 10 + ((6-1)* 2);//font FONT_12X16 là 10
	uint16_t x_center = (128-len)/2;
	sprintf(oled_buff, "GREMSY");
	UG_PutString(x_center,CAL_LINE_OLED(0, 10), oled_buff);

	UG_FontSelect(&FONT_6X10);
	len = strlen("JIG TEST PE HUB")* 5 + ((15-1)* 2);//font FONT_12X16 là 10
	x_center = (128-len)/2;
	sprintf(oled_buff, "JIG TEST PE HUB");
	UG_PutString(x_center,CAL_LINE_OLED(1, 22), oled_buff);

	// TRANG THAI INIT
	if(pin_check_data.state == PIN_CHECK_STATE_IDLE)
	{
		UG_FontSelect(&FONT_6X10);

		if(thread->loop_ms(&time_screen_process, 200))
		{
		  if(state_icon < 5)
		  {
			state_icon++;
		  }
		  else
		  {
			state_icon = 0;
		  }
		}
		if(state_icon == 0)
		{
		  sprintf(buff,">  Start test  <");
		}
		else if(state_icon == 1)
		{
		  sprintf(buff,">> Start test <<");
		}
		else if(state_icon == 2)
		{
		  sprintf(buff,">>>Start test<<<");
		}
		else if(state_icon == 3)
		{
		  sprintf(buff," >>Start test<< ");
		}
		else if(state_icon == 4)
		{
		  sprintf(buff,"  >Start test< ");
		}
		else if(state_icon == 5)
		{
		  sprintf(buff,"   Start test   ");
		}
		UG_PutString(10,CAL_LINE_OLED(1, 38), buff);
	}
	//Trang thai xu ly test
	else if(pin_check_data.state == PIN_CHECK_STATE_PROCESS)
	{
		UG_FontSelect(&FONT_5X8);
		sprintf(buff, "PROCESSING...");
		UG_PutString(2,CAL_LINE_OLED(1, 38), buff);
	}
	else if(pin_check_data.state == PIN_CHECK_STATE_OK)
	{
//		UG_FontSelect(&FONT_6X10);
//		sprintf(buff, "SIGNAL PASS");
//		UG_PutString(2,CAL_LINE_OLED(1, 38), buff);
//		UG_FontSelect(&FONT_6X10);
		UG_FontSelect(&FONT_6X10);
		len = strlen("SIGNAL PASS")* 5 + ((8-1)* 2);//font FONT_12X16 là 10
		x_center = (128-len)/2;
		sprintf(oled_buff, "SIGNAL PASS");
		UG_PutString(x_center,CAL_LINE_OLED(1, 38), oled_buff);

		if(thread->loop_ms(&time_screen_process, 200))
		{
		  if(state_icon < 5)
		  {
			state_icon++;
		  }
		  else
		  {
			state_icon = 0;
		  }
		}
		if(state_icon == 0)
		{
		  sprintf(buff,">  Back");
		}
		else if(state_icon == 1)
		{
		  sprintf(buff,">> Back");
		}
		else if(state_icon == 2)
		{
		  sprintf(buff,">>>Back");
		}
		else if(state_icon == 3)
		{
		  sprintf(buff," >>Back");
		}
		else if(state_icon == 4)
		{
		  sprintf(buff,"  >Back");
		}
		else if(state_icon == 5)
		{
		  sprintf(buff,"   Back");
		}
		UG_PutString(10,CAL_LINE_OLED(1, 48), buff);
	}
	else if(pin_check_data.state == PIN_CHECK_STATE_ERROR)
	{
		UG_FontSelect(&FONT_6X10);
		sprintf(buff, "SIGNAL FAILED");
		UG_PutString(2,CAL_LINE_OLED(1, 38), buff);

		if(pin_check_data.size == (uint32_t)NULL)
		{
			sprintf(buff, "No pin init");
			UG_PutString(2,CAL_LINE_OLED(1, 38), buff);
		}
		else
		{
			sprintf(buff, "%s", pin_check_data.array[pin_check_data.count].error_log);
			UG_PutString(2,CAL_LINE_OLED(1, 38), buff);
		}

		UG_FontSelect(&FONT_6X10);

		if(thread->loop_ms(&time_screen_process, 200))
		{
		  if(state_icon < 5)
		  {
			state_icon++;
		  }
		  else
		  {
			state_icon = 0;
		  }
		}
		if(state_icon == 0)
		{
		  sprintf(buff,">  Back");
		}
		else if(state_icon == 1)
		{
		  sprintf(buff,">> Back");
		}
		else if(state_icon == 2)
		{
		  sprintf(buff,">>>Back");
		}
		else if(state_icon == 3)
		{
		  sprintf(buff," >>Back");
		}
		else if(state_icon == 4)
		{
		  sprintf(buff,"  >Back");
		}
		else if(state_icon == 5)
		{
		  sprintf(buff,"   Back");
		}
		UG_PutString(10,CAL_LINE_OLED(1, 48), buff);
	}
}
/** @brief      ham xu kiem tra tin hieu board
    @param[in]  none
    @return     none
*/
void test_process(void)
{
//	static uint32_t timewait = 0;
	switch (pin_check_data.state)
	{
		case PIN_CHECK_STATE_IDLE:
		{
			if(button->get_clicks()== 1)//click
			{
				pin_check_data.state = PIN_CHECK_STATE_INIT;
			}
		}
			break;

		case PIN_CHECK_STATE_INIT:
		{
			//Trang thai init
			//Reset cac bien trang thai
			pin_check_data.count = 0;

			//Chuyen tab process
			pin_check_data.state = PIN_CHECK_STATE_PROCESS;
		}
		break;

		case PIN_CHECK_STATE_PROCESS:
		{
			 //Trang thai xu ly test
		   //Chua khoi tao pin checkuint8_t
		   if(pin_check_data.size == (uint32_t)NULL)
		   {
			   pin_check_data.state = PIN_CHECK_STATE_ERROR;

			   //Debug ra loi
			   sprintf(pin_check_data.array[pin_check_data.count].error_log, "Number pin = 0");
		   }
		   else
		   {
			   //Kiem tra neu pin chua duoc khoi tao
			   if(pin_check_data.array[pin_check_data.count].pin_type == PIN_TYPE_NONE)
			   {
				   pin_check_data.state = PIN_CHECK_STATE_ERROR;

				   //Debug ra loi
				   sprintf(pin_check_data.array[pin_check_data.count].error_log, "Pin wrong id = %d", pin_check_data.count);
			   }
			   // kiem tra tin hieu analog
			   if(pin_check_data.array[pin_check_data.count].pin_type == PIN_TYPE_ANALOG)
				   {
					   double voltage = 0;
					   voltage = *pin_check_data.array[pin_check_data.count].type_analog.p_analog_val
					   * 3.3 / 4096.0 * pin_check_data.array[pin_check_data.count].type_analog.analog_ratio
					   - pin_check_data.array[pin_check_data.count].type_analog.voltage_offset;

					   //Kiem tra neu dien ap chenh lenh nhieu so voi muc normal bao loi
					   if(fabs(voltage - pin_check_data.array[pin_check_data.count].type_analog.voltage_normal) >
						 pin_check_data.array[pin_check_data.count].type_analog.voltage_delta)
					   {
						   sprintf(pin_check_data.array[pin_check_data.count].error_log,
								   "%s = %1.2fV",
								   pin_check_data.array[pin_check_data.count].pin_name,
								   voltage);
						   pin_check_data.state = PIN_CHECK_STATE_ERROR;
					   }
				   }

			   //kiem tra cac tin hieu digital
			   if(pin_check_data.array[pin_check_data.count].pin_type == PIN_TYPE_DIGITAL)
			   {
				   //Set muc 0 cho Output
				   gpio_fast_output_reset(pin_check_data.array[pin_check_data.count].type_digital.gpio_out_port,
										pin_check_data.array[pin_check_data.count].type_digital.gpio_out_pin);
		//			                HAL_GPIO_WritePin(pin_check_data.array[pin_check_data.count].type_digital.gpio_out_port,
		//			                				  pin_check_data.array[pin_check_data.count].type_digital.gpio_out_pin,
		//											  RESET);

				   //Khai bao PULLUP cho Input
				   gpio_fast_input_set_pullup(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
											  pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin);

				   //Delay de doi ham thuc hien xong
				   thread->sleep_us(5);

				   //Doc du lieu tu chan
 				   if(gpio_fast_input_read(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
										   pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin) == 1)
				   {
					   pin_check_data.state = PIN_CHECK_STATE_ERROR;
				   }

				   //Set muc 1 cho Output
				   gpio_fast_output_set(pin_check_data.array[pin_check_data.count].type_digital.gpio_out_port,
										pin_check_data.array[pin_check_data.count].type_digital.gpio_out_pin);

				   //Khai bao keo xuong cho Input
				   gpio_fast_input_set_pulldown(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
												pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin);
				   //Delay de doi ham thuc hien xong
				   thread->sleep_us(5);

				   //Doc du lieu tu chan
				   if(gpio_fast_input_read(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
										   pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin) == 0)
				   {
					pin_check_data.state = PIN_CHECK_STATE_ERROR;
				   }
				   //Set muc 0 cho Output
				   gpio_fast_output_reset(pin_check_data.array[pin_check_data.count].type_digital.gpio_out_port,
										pin_check_data.array[pin_check_data.count].type_digital.gpio_out_pin);

			   }
			   if(pin_check_data.array[pin_check_data.count].pin_type == PIN_TYPE_GND)
			   {
				   if(gpio_fast_input_read(pin_check_data.array[pin_check_data.count].type_gnd.gpio_gnd_port, pin_check_data.array[pin_check_data.count].type_gnd.gpio_gnd_pin) == 1)
				   {
					   pin_check_data.state = PIN_CHECK_STATE_ERROR;
				   }
			   }
			   //kiem tra pin ket noi qua ex io
			   if(pin_check_data.array[pin_check_data.count].pin_type == PIN_TYPE_EX)
			   {
				   //Set muc 1 cho Output

				   PCF8575_write_pin(pin_check_data.array[pin_check_data.count].type_ex_gpio.GPIO_OutExPinNum, GPIO_PIN_SET);
				   thread->sleep_us(5);
				   //Khai bao keo xuong cho Input
				   gpio_fast_input_set_pulldown(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
												pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin);
				   //Delay de doi ham thuc hien xong
				   thread->sleep_us(5);

				   //Doc du lieu tu chan
				   if(gpio_fast_input_read(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
										   pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin) == 0)
				   {
					pin_check_data.state = PIN_CHECK_STATE_ERROR;
				   }

				   //Set muc 0 cho Output
				   PCF8575_write_pin(pin_check_data.array[pin_check_data.count].type_ex_gpio.GPIO_OutExPinNum, GPIO_PIN_RESET);
				   thread->sleep_us(5);
				   //Khai bao PULLUP cho Input
				   gpio_fast_input_set_pullup(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
											  pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin);

				   //Delay de doi ham thuc hien xong
				   thread->sleep_us(5);

				   //Doc du lieu tu chan
				   if(gpio_fast_input_read(pin_check_data.array[pin_check_data.count].type_digital.gpio_in_port,
										   pin_check_data.array[pin_check_data.count].type_digital.gpio_in_pin) == 1)
				   {
					   pin_check_data.state = PIN_CHECK_STATE_ERROR;
				   }
			   }

			   //Chuyen tab neu loi
			   if(pin_check_data.state == PIN_CHECK_STATE_ERROR)
			   {

			   }

			   else
			   {
				   if(pin_check_data.count < pin_check_data.size - 1)
				   {
					   pin_check_data.count++;
				   }
				   else
				   {
					   pin_check_data.state = PIN_CHECK_STATE_OK;
				   }
			   }

			}
		}
		break;
		case PIN_CHECK_STATE_OK:
		{
		 	 //Trang thai test ok
			if(button->get_clicks()== 1)//click
			{
				pin_check_data.state = PIN_CHECK_STATE_IDLE;
			}

		}
		break;

		case PIN_CHECK_STATE_ERROR:
		{
			if(button->get_clicks()== 1)//click
			{
				pin_check_data.state = PIN_CHECK_STATE_IDLE;
			}
		}
		break;
		default:
			break;
	}

}
/** @brief      ham out gpio
    @param[in]
    @return
*/
static void PCF8575_init(void)
{

	uint8_t data[2] = {0};
	if (HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)PCF8575_ADDR,3 , 5))
	{
		HAL_I2C_Master_Transmit(&hi2c2, PCF8575_ADDR, data, 2, 100);
	}
	HAL_Delay(1);

}
/** @brief      ham out gpio
    @param[in]
    @return
*/
static void PCF8575_write_pin(uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	uint8_t data[2] = {0};
	if(PinState == GPIO_PIN_SET)
	{
		data[0] = (uint8_t)GPIO_Pin;
		data[1] = (uint8_t)(GPIO_Pin >> 8);
		HAL_I2C_Master_Transmit(&hi2c2, PCF8575_ADDR, data, 2, 100);
	}
	else
	{
		if(GPIO_Pin < 255)
		{
			uint16_t pin_temp = GPIO_Pin;
			data[0] = (uint8_t)(pin_temp & (~GPIO_Pin));
			data[1] = (uint8_t)(GPIO_Pin >>8);
			HAL_I2C_Master_Transmit(&hi2c2, PCF8575_ADDR, data, 2, 100);
		}
		else
		{
			uint16_t pin_temp = GPIO_Pin;
			data[0] = (uint8_t)GPIO_Pin;
			data[1] = (uint8_t)((pin_temp & (~GPIO_Pin) >> 8));
			HAL_I2C_Master_Transmit(&hi2c2, PCF8575_ADDR, data, 2, 100);
			//todo: implenment feature: bat hoac tat gpio tuong ung ma ko lam mat gpio truoc do
		}

	}

}

/** @brief      ham xu check device i2c address
    @param[in]  hi2c: port i2c can check
    @return     i2c device address
*/
static uint8_t iic_device_scan(I2C_HandleTypeDef* hi2c)
{
	uint8_t i2cdevice = 0;
    for(int i = 0; i< 128; i++)
    {
    	i2cdevice = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)i<<1,3 , 5);
		if(i2cdevice!=HAL_OK )
		{
			HAL_UART_Transmit(&huart1, Space, sizeof(Space), 1000);
		}
		else if(i2cdevice == HAL_OK)
		{
			sprintf((char*)Buffer, "0x%X", i);
			HAL_UART_Transmit(&huart1, Buffer, sizeof(Buffer), 1000);
		}
		HAL_UART_Transmit(&huart1, EndMSG, sizeof(EndMSG), 1000);
		thread->sleep_ms(5);
    }
    return i2cdevice;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)analog_buff, 14);
//  PCF8575_init();
  pin_check_init();

  //check pin ffc cable

    pin_check_add(ffcUartTx);
    pin_check_add(ffcUartRx);
  pin_check_add(ffcCanH);
  pin_check_add(ffcCanL);
  pin_check_add(ffcEthTxp);
  pin_check_add(ffcEthTxn);
  pin_check_add(ffcEthRxp);
  pin_check_add(ffcEthRxn);
  pin_check_add(ffcusbDp);
  pin_check_add(ffcusbDn);

  //check pin connector'
	pin_check_add(uartTx);
	pin_check_add(uartRx);
//  pin_check_add(ethRxn);
//  pin_check_add(ethRxp);
//  pin_check_add(ethTxn);
//  pin_check_add(ethTxp);
//  pin_check_add(usbDp);
//  pin_check_add(usbDn);
//  pin_check_add(canL);
//  pin_check_add(canH);
//  pin_check_add(gpsPps);
//  pin_check_add(capture);
//  pin_check_add(trig);

  //check pin power
//
  pin_check_add(com5v);
  pin_check_add(comGnd);
  pin_check_add(can5v);
  pin_check_add(canGnd);
  pin_check_add(sbppm5v);
  pin_check_add(sbppmGnd);
  pin_check_add(spek5V);
  pin_check_add(spekGnd);
  pin_check_add(usb5v);
  pin_check_add(usbGnd);
  pin_check_add(vcc1);
  pin_check_add(vcc2);
  pin_check_add(vcc3);
  pin_check_add(pgnd1);
  pin_check_add(pgnd2);
  pin_check_add(pgnd3);

  //Khoi tao cac thu vien

  thread = gremsy_thread_init();

  //Khoi tao ham button
  button = gremsy_button_init();

  //khoi tao oled
  thread->sleep_ms(100);
  ssd1306_init();

  /* Init uGUI */
  UG_Init(&gui,(void(*)(UG_S16,UG_S16,UG_COLOR))ssd1306_draw_pixel,130,64);
  uint32_t time_process_screen;

  /// Trang thai khoi tao

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //Xu ly nut nhan
    button->process();

    /// trang thai nhan giu 1 lan
    if(button->get_clicks() == -1)
    {

    }

    /// trang thai nhan 1 lan
    else if(button->get_clicks() == 1)
    {

    }
//    iic_device_scan(&hi2c2);
    test_process();

    //Cap nhat man hinh
    if(thread->loop_ms(&time_process_screen, 10))
    {
      if(ssd1306_update_screen(0))
      {
          screen_process();
      }
#ifdef GPIO_TEST
    PCF8575_write_pin(IO_EX_UART_TX, GPIO_PIN_SET);
    uint8_t read = HAL_GPIO_ReadPin(IN_COM_TX_GPIO_Port,IN_COM_TX_Pin);
    thread->sleep_ms(200);

    PCF8575_write_pin(IO_EX_UART_TX, GPIO_PIN_RESET);
    read = HAL_GPIO_ReadPin(IN_COM_TX_GPIO_Port,IN_COM_TX_Pin);
    thread->sleep_ms(200);
#endif
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 14;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (84000000/1000000 - 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_FFC_ETH_TXN_Pin|OUT_FFC_ETH_RXP_Pin|OUT_FFC_ETH_RXN_Pin|OUT_FFC_USB_DN_Pin
                          |OUT_FFC_UART_TX_Pin|OUT_FFC_UART_RX_Pin|OUT_FFC_CANH_Pin|OUT_FFC_CANL_Pin
                          |OUT_FFC_ETH_TXP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_FFC_USB_DP_GPIO_Port, OUT_FFC_USB_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN_CAPTURE_Pin IN_TRIG_Pin IN_COM_RX_Pin IN_COM_TX_Pin
                           IN_USB_DP_Pin IN_USB_DN_Pin IN_CANL_Pin IN_CANH_Pin
                           IN_GPS_PPS_Pin */
  GPIO_InitStruct.Pin = IN_CAPTURE_Pin|IN_TRIG_Pin|IN_COM_RX_Pin|IN_COM_TX_Pin
                          |IN_USB_DP_Pin|IN_USB_DN_Pin|IN_CANL_Pin|IN_CANH_Pin
                          |IN_GPS_PPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_FFC_ETH_TXN_Pin OUT_FFC_ETH_RXP_Pin OUT_FFC_ETH_RXN_Pin OUT_FFC_USB_DN_Pin
                           OUT_FFC_UART_TX_Pin OUT_FFC_UART_RX_Pin OUT_FFC_CANH_Pin OUT_FFC_CANL_Pin
                           OUT_FFC_ETH_TXP_Pin */
  GPIO_InitStruct.Pin = OUT_FFC_ETH_TXN_Pin|OUT_FFC_ETH_RXP_Pin|OUT_FFC_ETH_RXN_Pin|OUT_FFC_USB_DN_Pin
                          |OUT_FFC_UART_TX_Pin|OUT_FFC_UART_RX_Pin|OUT_FFC_CANH_Pin|OUT_FFC_CANL_Pin
                          |OUT_FFC_ETH_TXP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_PGND3_Pin IN_PGND2_Pin */
  GPIO_InitStruct.Pin = IN_PGND3_Pin|IN_PGND2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_ETH_RXN_Pin IN_ETH_RXP_Pin IN_ETH_TXN_Pin IN_ETH_TXP_Pin */
  GPIO_InitStruct.Pin = IN_ETH_RXN_Pin|IN_ETH_RXP_Pin|IN_ETH_TXN_Pin|IN_ETH_TXP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OUT_FFC_USB_DP_Pin */
  GPIO_InitStruct.Pin = OUT_FFC_USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_FFC_USB_DP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

