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
#include "stdlib.h"
//#include "soft_i2c.h"
#include "gremsy_ssd1306.h"
#include "ugui.h"
#include "main.h"
#include "string.h"

/* Private typedef
------------------------------------------------------------------------------*/
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

typedef struct
{
    uint16_t current_x;
    uint16_t current_y;
    uint8_t inverted;
    uint8_t initialized;
}ssd1306_t;
/* Private define
------------------------------------------------------------------------------*/
/** SSD1306 OLED height in pixels */
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT          64
#endif

/** SSD1306 width in pixels */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH           130
#endif

/** SSD1306 buffer size */
#ifndef SSD1306_BUFF_SIZE
#define SSD1306_BUFF_SIZE       (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
#endif

#define SSD1306_I2C_ADDR        ((0x3C << 1))
/* Private macro
------------------------------------------------------------------------------*/
/* Private variables
------------------------------------------------------------------------------*/
static ssd1306_t ssd1306;
/** Screenbuffer */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c1;
//Bo dem cho SPI DMA
uint8_t iic_buff[SSD1306_WIDTH + 1];

//uint8_t* iic_buff2;

//DMA SPI update
uint16_t iic_data_count = 0, iic_data_count_max = 0;

/* Private function prototypes
------------------------------------------------------------------------------*/
/* Private functions
------------------------------------------------------------------------------*/

/** @brief Ham ghi Command cho Oled.
    @param Command
    @ret   None.
*/
static void ssd1306_write_comand(uint8_t byte)
{
    HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x00, 1, &byte, 1, 1000);
  
    HAL_Delay(1);
}

/** @brief ham xoa man hinh oled.
    @param mau sac
    @ret   none.
*/
void ssd1306_fill(uint8_t color)
{
    /* Set memory */
    memset(SSD1306_Buffer, color == 0 ?  0x00 : 0xFF, SSD1306_BUFF_SIZE);
}

/**
  IIC DMA fast
  */
static void I2C_DMAXferCplt_Fast(DMA_HandleTypeDef *hdma)
{
  I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent; /* Derogation MISRAC2012-Rule-11.5 */

  /* Declaration of temporary variable to prevent undefined behavior of volatile usage */
  HAL_I2C_StateTypeDef CurrentState = hi2c->State;
  HAL_I2C_ModeTypeDef CurrentMode   = hi2c->Mode;
  uint32_t CurrentXferOptions       = hi2c->XferOptions;

  /* Disable EVT and ERR interrupt */
//  __HAL_I2C_DISABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);

//  /* Clear Complete callback */
//  if (hi2c->hdmatx != NULL)
//  {
//    hi2c->hdmatx->XferCpltCallback = NULL;
//  }
//  if (hi2c->hdmarx != NULL)
//  {
//    hi2c->hdmarx->XferCpltCallback = NULL;
//  }

//  if ((((uint32_t)CurrentState & (uint32_t)HAL_I2C_STATE_BUSY_TX) == (uint32_t)HAL_I2C_STATE_BUSY_TX) || ((((uint32_t)CurrentState & (uint32_t)HAL_I2C_STATE_BUSY_RX) == (uint32_t)HAL_I2C_STATE_BUSY_RX) && (CurrentMode == HAL_I2C_MODE_SLAVE)))
//  {
//    /* Disable DMA Request */
//    CLEAR_BIT(hi2c->Instance->CR2, I2C_CR2_DMAEN);

//    hi2c->XferCount = 0U;

//    /* Enable EVT and ERR interrupt to treat end of transfer in IRQ handler */
//    __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);
//  }
    /* Disable DMA Request */
    CLEAR_BIT(hi2c->Instance->CR2, I2C_CR2_DMAEN);
}

HAL_StatusTypeDef I2C_DMA_Fast(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{ 
    __IO uint32_t count = 0U;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Check if the I2C is already enabled */
    if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
      /* Enable I2C peripheral */
      __HAL_I2C_ENABLE(hi2c);
    }

    /* Disable Pos */
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr    = pData;
    hi2c->XferCount   = Size;
    hi2c->XferSize    = hi2c->XferCount;
    hi2c->XferOptions = 0xFFFF0000U;
    hi2c->Devaddress  = DevAddress;

    if (hi2c->XferSize > 0U)
    {
        /* Set the I2C DMA transfer complete callback */
        hi2c->hdmatx->XferCpltCallback = I2C_DMAXferCplt_Fast;

        /* Enable the DMA stream */
        HAL_DMA_Start_IT(hi2c->hdmatx, (uint32_t)hi2c->pBuffPtr, (uint32_t)&hi2c->Instance->DR, hi2c->XferSize);

        /* Enable Acknowledge */
        SET_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);
        /* Generate Start */
        SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);
        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        /* Enable EVT and ERR interrupt */
        __HAL_I2C_ENABLE_IT(hi2c, I2C_IT_EVT | I2C_IT_ERR);
        /* Enable DMA Request */
        SET_BIT(hi2c->Instance->CR2, I2C_CR2_DMAEN);
    }
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/** @brief Ham update screen oled.
    @param spi_type = 1 : SPI POLLING
    @param spi_type = 0 : SPI DMA
    @param mask_clean_scr = 0 : Not ready clean screen
    @param mask_clean_scr = 1 : Ready clean screen
    @ret   none.
*/
bool ssd1306_update_screen(bool spi_type)
{
    if(spi_type)
    {
        if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
        {
          //Polling spi update
          uint8_t i;
          for(i = 0; i < 8; i++) {
              ssd1306_write_comand(0xB0 + i);
              ssd1306_write_comand(0x00);
              ssd1306_write_comand(0x10);
              HAL_I2C_Mem_Write(&hi2c1, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH*i],SSD1306_WIDTH, 1000);
          }
          return true;
        }
    }
    else
    {
        if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
        {
            if(iic_data_count < 16)
            {
                //Write command
                if((iic_data_count % 2) == 0)
                {
                    //command 1
                    //iic_buff[1] = (SSD1306_I2C_ADDR << 1) | 1;
                    iic_buff[0] = 0x00;
                    iic_buff[1] = (0xB0 + (iic_data_count/2));
                    
                    //command 2
                    iic_buff[2] = (SSD1306_I2C_ADDR << 1) | 1;
                    iic_buff[3] = 0x00;
                    iic_buff[4] = 0x00;
                    
                    //command 3
                    iic_buff[5] = (SSD1306_I2C_ADDR << 1) | 1;
                    iic_buff[6] = 0x00;
                    iic_buff[7] = 0x10;
                    
//                    memset(iic_buff2, 0xFF, 100);
                    //Write Command
                    HAL_I2C_Master_Transmit_DMA(&hi2c1, SSD1306_I2C_ADDR, iic_buff, 8);
//                    I2C_DMA_Fast(&hi2c3, SSD1306_I2C_ADDR, iic_buff, 8);
                }
                //Write data
                else
                {
                    iic_buff[0] = 0x40;
                    memcpy(&iic_buff[1], &SSD1306_Buffer[SSD1306_WIDTH*(iic_data_count/2)], SSD1306_WIDTH);
                  
//                    memset(iic_buff2, 0xFF, 100);
                    HAL_I2C_Master_Transmit_DMA(&hi2c1, SSD1306_I2C_ADDR, iic_buff, SSD1306_WIDTH);
                  
                    if(iic_data_count_max < SSD1306_WIDTH*(iic_data_count/2)) iic_data_count_max = SSD1306_WIDTH*(iic_data_count/2);
                }
                iic_data_count++;
            }
            else
            {
                iic_data_count = 0;
                //clear screen
                ssd1306_fill(0x00);
//                if(mask_clean_scr)
//                {
//                    ssd1306_fill(0x00);
//                }
                return true;
            }
        }
    }
    
    
    
    return false;
}
void ssd1306_init(void)
{
//    /** Init i2c */
//    soft_i2c_init(B6, B7);
//    iic_buff = calloc(sizeof(uint8_t), SSD1306_WIDTH + 1);
//    iic_buff2 = calloc(sizeof(uint8_t), 100);
  
    /** Init OLED */
    ssd1306_write_comand(0xAE); //display off

    ssd1306_write_comand(0x20); //Set Memory Addressing Mode   
    ssd1306_write_comand(0x10); //00,Horizontal Addressing Mode; 01,Vertical Addressing Mode;
                                //10,Page Addressing Mode (RESET); 11,Invalid

    ssd1306_write_comand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

    #ifdef SSD1306_MIRROR_VERT
    ssd1306_write_comand(0xC0); //Mirror vertically
    #else
    ssd1306_write_comand(0xC8); //Set COM Output Scan Direction
    #endif

    ssd1306_write_comand(0x00); //---set low column address
    ssd1306_write_comand(0x10); //---set high column address

    ssd1306_write_comand(0x40); //--set start line address - CHECK

    ssd1306_write_comand(0x81); //--set contrast control register - CHECK
    ssd1306_write_comand(0xFF);

    #ifdef SSD1306_MIRROR_HORIZ
    ssd1306_write_comand(0xA0); // Mirror horizontally
    #else
    ssd1306_write_comand(0xA1); //--set segment re-map 0 to 127 - CHECK
    #endif

    #ifdef SSD1306_INVERSE_COLOR
    ssd1306_write_comand(0xA7); //--set inverse color
    #else
    ssd1306_write_comand(0xA6); //--set normal color
    #endif

    ssd1306_write_comand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
    ssd1306_write_comand(0x3F); //

    ssd1306_write_comand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    ssd1306_write_comand(0xD3); //-set display offset - CHECK
    ssd1306_write_comand(0x00); //-not offset

    ssd1306_write_comand(0xD5); //--set display clock divide ratio/oscillator frequency
    ssd1306_write_comand(0xF0); //--set divide ratio

    ssd1306_write_comand(0xD9); //--set pre-charge period
    ssd1306_write_comand(0x22); //

    ssd1306_write_comand(0xDA); //--set com pins hardware configuration - CHECK
    ssd1306_write_comand(0x12);

    ssd1306_write_comand(0xDB); //--set vcomh
    ssd1306_write_comand(0x20); //0x20,0.77xVcc

    ssd1306_write_comand(0x8D); //--set DC-DC enable
    ssd1306_write_comand(0x14); //
    ssd1306_write_comand(0xAF); //--turn on SSD1306 panel

    /** Clear screen */
    ssd1306_fill(0x00);
    
    /** Flush buffer to screen */
    ssd1306_update_screen(1);
    
    /** Set default values for screen object */
    ssd1306.current_x = 0;
    ssd1306.current_y = 0;
    
    ssd1306.initialized = 1;
}
void ssd1306_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        /** Don't write outside the buffer */
        return;
    }

    /** Check if pixel should be inverted */
    if(ssd1306.inverted)
    {
        color =!color;
    }

    /** Draw in the right color */
    //BLACK COLOR
    if(color == 0x00)
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
    //WHITE COLOR
    else
    { 
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
}
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/
