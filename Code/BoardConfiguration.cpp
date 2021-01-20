/* ###*B*###
 * Erika Enterprise, version 3
 *
 * Copyright (C) 2017 - 2018 Evidence s.r.l.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License, version 2, for more details.
 *
 * You should have received a copy of the GNU General Public License,
 * version 2, along with this program; if not, see
 * < www.gnu.org/licenses/old-licenses/gpl-2.0.html >.
 *
 * This program is distributed to you subject to the following
 * clarifications and special exceptions to the GNU General Public
 * License, version 2.
 *
 * THIRD PARTIES' MATERIALS
 *
 * Certain materials included in this library are provided by third
 * parties under licenses other than the GNU General Public License. You
 * may only use, copy, link to, modify and redistribute this library
 * following the terms of license indicated below for third parties'
 * materials.
 *
 * In case you make modified versions of this library which still include
 * said third parties' materials, you are obligated to grant this special
 * exception.
 *
 * The complete list of Third party materials allowed with ERIKA
 * Enterprise version 3, together with the terms and conditions of each
 * license, is present in the file THIRDPARTY.TXT in the root of the
 * project.
 * ###*E*### */

/** \file	BoardConfiguration.cpp
 *  \brief	Touch GFX Framework Board Cnfiguration.
 *
 *  This file contains the Touch GFX Framework Board Cnfiguration
 *  for Erika Enterprise.
 *
 *  \author	Giuseppe Serano
 *  \date	2020
 */
#include <touchgfx/hal/BoardConfiguration.hpp>
#include <touchgfx/hal/GPIO.hpp>
#include <touchgfx/hal/OSWrappers.hpp>
#include <CortexMMCUInstrumentation.hpp>
#include <platform/driver/lcd/LCD16bpp.hpp>
#include <platform/driver/lcd/LCD24bpp.hpp>

#include <texts/TypedTextDatabase.hpp>
#include <fonts/ApplicationFontProvider.hpp>
#include <gui/common/FrontendHeap.hpp>
#include <BitmapDatabase.hpp>
#include <STM32DMA.hpp>
#include <TouchGFXHAL.hpp>
#include <STM32TouchController.hpp>
#include <stm32f4xx_hal.h>
#include <TouchGFXGeneratedHAL.hpp>

extern "C" void touchgfx_init();

extern "C" {
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_sdram.h"

#include "../Components/ili9341/ili9341.h"

    bool os_inited = false;
}

/***********************************************************
 ******         24 Bits Per Pixel Support            *******
 ***********************************************************
 *
 * The default bit depth of the framebuffer is 16bpp. If you want 24bpp support,
 * define the symbol "USE_BPP" with a value
 * of "24", e.g. "USE_BPP=24". This symbol affects the following:
 *
 * 1. Type of TouchGFX LCD (16bpp vs 24bpp)
 * 2. Bit depth of the framebuffer(s)
 * 3. TFT controller configuration.
 *
 * WARNING: Remember to modify your image formats accordingly in app/config/.
 *          Please see the following knowledgebase article for further details
 *          on how to choose and configure the appropriate image formats for
 *          your application:
 *
 * https://touchgfx.zendesk.com/hc/en-us/articles/206725849
 */
static uint32_t frameBuf0 = (uint32_t)(0xd0000000);

#define LCD_FRAME_BUFFER                         frameBuf0

#define LCD_FRAME_BUFFER_LAYER0                  (LCD_FRAME_BUFFER+0x130000)
#define LCD_FRAME_BUFFER_LAYER1                  LCD_FRAME_BUFFER

extern "C" {
    LTDC_HandleTypeDef LtdcHandler;
    DMA2D_HandleTypeDef hdma2d;
    static RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    static LCD_DrvTypeDef* LcdDrv;

    /**
      * @brief  Initializes the LTDC MSP.
      */
    static void LCD_MspInit(void)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable the LTDC and DMA2D Clock */
        __HAL_RCC_LTDC_CLK_ENABLE();
        __HAL_RCC_DMA2D_CLK_ENABLE();

        //Enable CRC engine for STM32 Lock check
        __HAL_RCC_CRC_CLK_ENABLE();

        /* Enable GPIOs clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();

        /* GPIOs Configuration */
        /*
        +------------------------+-----------------------+----------------------
        +                       LCD pins assignment                             
        +------------------------+-----------------------+----------------------
        |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06
        |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11
        |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12
        |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03
        |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08
        |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09
        ------------------------------------------------------------------------
                |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
                |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
                 -----------------------------------------------------
        */

        /* GPIOA configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 |
                                 GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
        GPIO_InitStructure.Alternate = GPIO_AF14_LTDC;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

        /* GPIOB configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_8 | \
                                 GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* GPIOC configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* GPIOD configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_6;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

        /* GPIOF configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_10;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

        /* GPIOG configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | \
                                 GPIO_PIN_11;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);

        /* GPIOB configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStructure.Alternate = GPIO_AF9_LTDC;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* GPIOG configuration */
        GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_12;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
    }

    /**
      * @brief  Initializes the LCD.
      * @retval LCD state
      */
    static uint8_t LCD_Init(void)
    {
        /*
         * On STM32F429I-DISCO, it is not possible to read ILI9341 ID because
         * PIN EXTC is not connected to VDD and then LCD_READ_ID4 is not
         * accessible.
         * In this case, ReadID function is bypassed.
         * if(ili9341_drv.ReadID() == ILI9341_ID)
         */

        /* LTDC Configuration ------------------------------------------------*/
        LtdcHandler.Instance = LTDC;

        /* Timing configuration  (Typical configuration from ILI9341 datasheet)
              HSYNC=10 (9+1)
              HBP=20 (29-10+1)
              ActiveW=240 (269-20-10+1)
              HFP=10 (279-240-20-10+1)

              VSYNC=2 (1+1)
              VBP=2 (3-2+1)
              ActiveH=320 (323-2-2+1)
              VFP=4 (327-320-2-2+1)
          */

        /* Configure horizontal synchronization width */
        LtdcHandler.Init.HorizontalSync = ILI9341_HSYNC;
        /* Configure vertical synchronization height */
        LtdcHandler.Init.VerticalSync = ILI9341_VSYNC;
        /* Configure accumulated horizontal back porch */
        LtdcHandler.Init.AccumulatedHBP = ILI9341_HBP;
        /* Configure accumulated vertical back porch */
        LtdcHandler.Init.AccumulatedVBP = ILI9341_VBP;
        /* Configure accumulated active width */
        LtdcHandler.Init.AccumulatedActiveW = 269;
        /* Configure accumulated active height */
        LtdcHandler.Init.AccumulatedActiveH = 323;
        /* Configure total width */
        LtdcHandler.Init.TotalWidth = 279;
        /* Configure total height */
        LtdcHandler.Init.TotalHeigh = 327;

        /* Configure R,G,B component values for LCD background color */
        LtdcHandler.Init.Backcolor.Red = 0;
        LtdcHandler.Init.Backcolor.Blue = 0;
        LtdcHandler.Init.Backcolor.Green = 0;

        /* LCD clock configuration */
        /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
        /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
        /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 Mhz */
        /* 
         * LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_8 = 48/4 = 6Mhz
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
        PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
        PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
        PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

        /* Polarity */
        LtdcHandler.Init.HSPolarity = LTDC_HSPOLARITY_AL;
        LtdcHandler.Init.VSPolarity = LTDC_VSPOLARITY_AL;
        LtdcHandler.Init.DEPolarity = LTDC_DEPOLARITY_AL;
        LtdcHandler.Init.PCPolarity = LTDC_PCPOLARITY_IPC;

        LCD_MspInit();
        HAL_LTDC_Init(&LtdcHandler);

        /* Select the device */
        LcdDrv = &ili9341_drv;

        /* LCD Init */
        LcdDrv->Init();

        /* Initialize the SDRAM */
        BSP_SDRAM_Init();

        return 0;
    }

    uint32_t LCD_GetXSize(void)
    {
        return LcdDrv->GetLcdPixelWidth();
    }

    /**
      * @brief  Gets the LCD Y size.
      * @retval The used LCD Y size
      */
    uint32_t LCD_GetYSize(void)
    {
        return LcdDrv->GetLcdPixelHeight();
    }
}

/**
  * @brief  Initializes the LCD layers.
  * @param  LayerIndex: the layer foreground or background.
  * @param  FB_Address: the layer frame buffer.
  */
static void LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address)
{
    LTDC_LayerCfgTypeDef Layercfg;

    /* Layer Init */
    Layercfg.WindowX0 = 0;
    Layercfg.WindowX1 = LCD_GetXSize();
    Layercfg.WindowY0 = 0;
    Layercfg.WindowY1 = LCD_GetYSize();
#if USE_BPP == 16
    Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
#elif USE_BPP == 24
    Layercfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
#else
#error Unknown USE_BPP
#endif
    Layercfg.FBStartAdress = FB_Address;
    Layercfg.Alpha = 255;
    Layercfg.Alpha0 = 0;
    Layercfg.Backcolor.Blue = 0;
    Layercfg.Backcolor.Green = 0;
    Layercfg.Backcolor.Red = 0;
    Layercfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
    Layercfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
    Layercfg.ImageWidth = LCD_GetXSize();
    Layercfg.ImageHeight = LCD_GetYSize();

    HAL_LTDC_ConfigLayer(&LtdcHandler, &Layercfg, LayerIndex);

    /* Dithering activation */
    HAL_LTDC_EnableDither(&LtdcHandler);
}

/**
  * @brief LCD configuration.
  * @note  This function Configure tha LTDC peripheral :
  *        1) Configure the Pixel Clock for the LCD
  *        2) Configure the LTDC Timing and Polarity
  *        3) Configure the LTDC Layer 1 :
  *           - The frame buffer is located at FLASH memory
  *           - The Layer size configuration : 240x160
  *        4) Configure the LTDC Layer 2.
  *           - The frame buffer is located at FLASH memory
  *           - The Layer size configuration : 240x160
  * @retval
  *  None
  */
static void LCD_Config(void)
{
    LCD_Init();
    LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER1);
}

namespace touchgfx
{
static STM32TouchController tc;
static STM32F4DMA dma;
static LCD16bpp display;
static ApplicationFontProvider fontProvider;
static Texts texts;
static TouchGFXHAL hal(dma, display, tc, 240, 320);
static CortexMMCUInstrumentation mcuInstr;

void touchgfx_init()
{
    Bitmap::registerBitmapDatabase(
        BitmapDatabase::getInstance(), BitmapDatabase::getInstanceSize()
    );
    TypedText::registerTexts(&texts);
    Texts::setLanguage(0);

    FontManager::setFontProvider(&fontProvider);

    FrontendHeap& heap = FrontendHeap::getInstance();
    /* we need to obtain the reference above to initialize the frontend heap. */
    (void)heap;

    hal.initialize();

    mcuInstr.init();

	//Set MCU instrumentation and Load calculation
	hal.setMCUInstrumentation(&mcuInstr);
	hal.enableMCULoadCalculation(true);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /*
     * The voltage scaling allows optimizing the power consumption when the
     * device is clocked below the maximum system frequency, to update the
     * voltage scaling value regarding system frequency refer to product
     * datasheet.
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    /*
     * Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     * clocks dividers
     */
    RCC_ClkInitStruct.ClockType = (
        RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2
    );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void hw_init()
{
    HAL_Init();

    /* Configure the system clock to 168 MHz */
    SystemClock_Config();

    /* Configure LCD */
    LCD_Config();

    GPIO::init();
}

}	/* namespace touchgfx */
