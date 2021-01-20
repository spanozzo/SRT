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

/** \file	TouchGFXGeneratedHAL.cpp
 *  \brief	Touch GFX Framework Generated HAL.
 *
 *  This file contains the Touch GFX Framework Generated HAL adapetd
 *  for Erika Enterprise.
 *
 *  \author	Giuseppe Serano
 *  \date	2020
 */
#include <TouchGFXGeneratedHAL.hpp>
#include <STM32DMA.hpp>
#include <touchgfx/hal/GPIO.hpp>
#include <touchgfx/hal/OSWrappers.hpp>
#include <gui/common/FrontendHeap.hpp>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "ee.h"

volatile unsigned int TIMER;

namespace
{
    /*
     * Use the section "TouchGFX_Framebuffer" in the linker to specify the
     * placement of the buffer
     */
    LOCATION_PRAGMA("TouchGFX_Framebuffer")
    uint32_t frameBuf[(240 * 320 * 2 + 3) / 4 * 2]
    LOCATION_ATTRIBUTE("TouchGFX_Framebuffer");
    static uint16_t lcd_int_active_line;
    static uint16_t lcd_int_porch_line;
}

void TouchGFXGeneratedHAL::initialize()
{
    HAL::initialize();

    registerEventListener(*(touchgfx::Application::getInstance()));

    setFrameBufferStartAddresses(
        (void*)frameBuf,
        (void*)(frameBuf + sizeof(frameBuf) / (sizeof(uint32_t) * 2)),
        (void*)0
    );
    /*
     * Set whether the DMA transfers are locked to the TFT update cycle. If
     * locked, DMA transfer will not begin until the TFT controller has finished
     * updating the display. If not locked, DMA transfers will begin as soon as
     * possible. Default is true (DMA is locked with TFT).
     *
     * Setting to false to increase performance when using double buffering
     */
    lockDMAToFrontPorch(false);
}

void TouchGFXGeneratedHAL::configureInterrupts()
{
}

void TouchGFXGeneratedHAL::enableInterrupts()
{
    ReleaseResource(HalResource);
}

void TouchGFXGeneratedHAL::disableInterrupts()
{
    GetResource(HalResource);
}

void TouchGFXGeneratedHAL::enableLCDControllerInterrupt()
{
    lcd_int_active_line = (LTDC->BPCR & 0x7FF) - 1;
    lcd_int_porch_line = (LTDC->AWCR & 0x7FF) - 1;

    /* Sets the Line Interrupt position */
    LTDC->LIPCR = lcd_int_active_line;
    /* Line Interrupt Enable            */
    LTDC->IER |= LTDC_IER_LIE;
}

uint16_t* TouchGFXGeneratedHAL::getTFTFrameBuffer() const
{
    return (uint16_t*)LTDC_Layer1->CFBAR;
}

void TouchGFXGeneratedHAL::setTFTFrameBuffer(uint16_t* adr)
{
    LTDC_Layer1->CFBAR = (uint32_t)adr;

    /* Reload immediate */
    LTDC->SRCR = (uint32_t)LTDC_SRCR_IMR;
}

void TouchGFXGeneratedHAL::flushFrameBuffer(const touchgfx::Rect& rect)
{
    HAL::flushFrameBuffer(rect);
}

extern "C"
{

    /* User can use this section to tailor TIMx instance used and associated
       resources */
    /* Definition for TIMx clock resources */
    #define TIMx                        TIM3
    #define TIMx_CLK_ENABLE             __HAL_RCC_TIM3_CLK_ENABLE

    /* Definition for TIMx's NVIC */
    #define TIMx_IRQn                   TIM3_IRQn
    #define TIMx_IRQHandler             TIM3_IRQHandler

    static TIM_HandleTypeDef            TimHandle;

    /* Definition for USARTx clock resources */
    #define USARTx                      USART1
    #define USARTx_CLK_ENABLE()         __HAL_RCC_USART1_CLK_ENABLE()
    // DMA_NEW
	#define USARTc_DMA_CLK_ENABLE()     __HAL_RCC_DMA2_CLK_ENABLE()
    #define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
    #define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

    #define USARTx_FORCE_RESET()        __HAL_RCC_USART1_FORCE_RESET()
    #define USARTx_RELEASE_RESET()      __HAL_RCC_USART1_RELEASE_RESET()

    /* Definition for USARTx Pins */
    #define USARTx_TX_PIN               GPIO_PIN_9
    #define USARTx_TX_GPIO_PORT         GPIOA
    #define USARTx_TX_AF                GPIO_AF7_USART1
    #define USARTx_RX_PIN               GPIO_PIN_10
    #define USARTx_RX_GPIO_PORT         GPIOA
    #define USARTx_RX_AF                GPIO_AF7_USART1

    /* Definition for USARTx's NVIC */
    #define USARTx_IRQn                 USART1_IRQn
    #define USARTx_IRQHandler           USART1_IRQHandler

    // DMA_NEW
    /* Definition for USARTx's DMA */
    #define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
    #define USARTx_TX_DMA_STREAM             DMA2_Stream7
    // DMA_NEW
    /* Definition for USARTx's NVIC */
    #define USARTx_DMA_TX_IRQn               DMA2_Stream7_IRQn
    #define USARTx_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler

    UART_HandleTypeDef                  UartHandle;

    // DMA_NEW
    DMA_HandleTypeDef					hdma_tx;

    extern LTDC_HandleTypeDef           LtdcHandler;
    volatile uint32_t                   overrunCnt;

    void HAL_MspInit(void)
    {
        SysTick->CTRL = 0U;
    }

    void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc)
    {
    #ifdef	OS_EE_LIB_STM32_CUBE_F4
        VAR(uint16_t, AUTOMATIC)        uwPrescalerValue = 0;

        (void)hltdc;

        /*##-0- Enable peripherals and GPIO Clocks ###########################*/
        /* TIMx Peripheral clock enable */
        TIMx_CLK_ENABLE();

        /*##-1- Configure the TIM peripheral #################################*/
        /* ------------------------------------------------------------------
           In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock
           (PCLK1), since APB1 prescaler is different from 1.
             TIM3CLK = 2 * PCLK1
             PCLK1 = HCLK / 4
             => TIM3CLK = HCLK / 2 = SystemCoreClock /2
           To get TIM3 counter clock at 10 KHz, the Prescaler is computed as
           following:
           Prescaler = (TIM3CLK / TIM3 counter clock) - 1
           Prescaler = ((SystemCoreClock /2) /10 KHz) - 1

           Note:
             SystemCoreClock variable holds HCLK frequency and is defined in
             system_stm32f4xx.c file.
             Each time the core clock (HCLK) changes, user had to update
             SystemCoreClock variable value. Otherwise, any configuration based
             on this variable will be incorrect.
             This variable is updated in three ways:
              1) by calling CMSIS function SystemCoreClockUpdate()
              2) by calling HAL API function HAL_RCC_GetSysClockFreq()
              3) each time HAL_RCC_ClockConfig() is called to configure the
                 system clock frequency
        --------------------------------------------------------------------- */

        /*
         * Compute the prescaler value to have TIM3 counter clock equal to
         * 10 KHz
         */
        uwPrescalerValue = (uint32_t) ((SystemCoreClock >> 1U) / 1000000U) - 1U;

        /* Set TIMx instance */
        TimHandle.Instance = TIMx;

        /* Initialize TIM3 peripheral as follows:
           + Period = 10000 - 1
           + Prescaler = ((SystemCoreClock/2)/10000) - 1
           + ClockDivision = 0
           + Counter direction = Up
         */
        TimHandle.Init.Period = 1000U - 1U;
        TimHandle.Init.Prescaler = uwPrescalerValue;
        TimHandle.Init.ClockDivision = 0;
        TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
        TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
        if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
        {
            /* Initialization Error */
            for(;;);
         }

        /*##-2- Start the TIM Base generation in interrupt mode ##############*/
        /* Start Channel1 */
        if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
        {
            /* Starting Error */
            for(;;);
        }

        GPIO_InitTypeDef  GPIO_InitStruct;

        /*##-0- Enable peripherals and GPIO Clocks ###########################*/
        /* Enable GPIO TX/RX clock */
        USARTx_TX_GPIO_CLK_ENABLE();
        USARTx_RX_GPIO_CLK_ENABLE();
        /* Enable USART1 clock */
        USARTx_CLK_ENABLE();
    	/* Enable DMA1 clock */
    	USARTc_DMA_CLK_ENABLE();

        /*##-1- Configure peripheral GPIO ####################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = USARTx_TX_PIN;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
        GPIO_InitStruct.Alternate = USARTx_TX_AF;

        HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = USARTx_RX_PIN;
        GPIO_InitStruct.Alternate = USARTx_RX_AF;

        HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

        /*##-2- Configure the UART peripheral ################################*/
        /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
        /* UART1 configured as follow:
           - Word Length = 8 Bits
           - Stop Bit = One Stop bit
           - Parity = None
           - BaudRate = 115200 baud
           - Hardware flow control disabled (RTS and CTS signals) */
        UartHandle.Instance          = USARTx;

        UartHandle.Init.BaudRate     = 115200;
        UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
        UartHandle.Init.StopBits     = UART_STOPBITS_1;
        UartHandle.Init.Parity       = UART_PARITY_NONE;
        UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
        UartHandle.Init.Mode         = UART_MODE_TX_RX;
        UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

        // DMA_NEW
        /*##-3- Configure the DMA streams ##########################################*/
		/* Configure the DMA handler for Transmission process */
		hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
		hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
		hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
		hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
		hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
		hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
		hdma_tx.Init.Mode                = DMA_NORMAL;
		hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
		hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
		hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
		hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
		hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;
		// DMA_NEW
		HAL_DMA_Init(&hdma_tx);
		// DMA_NEW
		/* Associate the initialized DMA handle to the the UART handle */
		__HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

        if(HAL_UART_Init(&UartHandle) != HAL_OK)
        {
            for(;;);
        }
    #endif	/* OS_EE_LIB_STM32_CUBE_F4 */
    }	/* DemoHAL_TimerInit() */

    void HAL_TIM_Base_MspInit(TIM_HandleTypeDef	*htim)
    {
      (void)htim;

      /*##-2- Configure the NVIC for TIMx ####################################*/
      /* Set the TIMx priority */
      HAL_NVIC_SetPriority(TIMx_IRQn, 0, OSEE_CORTEX_M_TIM3_ISR_PRI);

      /* Enable the TIMx global Interrupt */
      HAL_NVIC_EnableIRQ(TIMx_IRQn);
    }

    ISR1(HALTickISR)
    {
    #ifdef	OS_EE_LIB_STM32_CUBE_F4
        HAL_IncTick();
        HAL_TIM_IRQHandler(&TimHandle);
    #endif	/* OS_EE_LIB_STM32_CUBE_F4 */
    }

    ISR2(SerialISR)
    {
        HAL_UART_IRQHandler(&UartHandle);
    }
    // DMA_NEW
    ISR2(SerialDMATxISR)
    {
    	HAL_DMA_IRQHandler(UartHandle.hdmatx);
    }

    ISR1(LtdcErISR)
    {
        unsigned int isr = LTDC->ISR;
        if (LTDC->ISR & 2)
        {
            LTDC->ICR = 2;
            overrunCnt++;
        }
    }

    ISR2(LtdcISR)
    {
        HAL_LTDC_IRQHandler(&LtdcHandler);
    }

    void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef* hltdc)
    {
        if (LTDC->LIPCR == lcd_int_active_line)
        {
            /* entering active area */
            HAL_LTDC_ProgramLineEvent(hltdc, lcd_int_porch_line);
            HAL::getInstance()->vSync();
            OSWrappers::signalVSync();
            /*
             * Swap frame buffers immediately instead of waiting for the task to
             * be scheduled in.
             * Note: task will also swap when it wakes up, but that operation is
             *       guarded and will not have any effect if already swapped.
             */
            HAL::getInstance()->swapFrameBuffers();
            GPIO::set(GPIO::VSYNC_FREQ);
        }
        else
        {
            /* exiting active area */
            HAL_LTDC_ProgramLineEvent(hltdc, lcd_int_active_line);
            GPIO::clear(GPIO::VSYNC_FREQ);
            HAL::getInstance()->frontPorchEntered();
        }
    }

}   /* extern "C" */

