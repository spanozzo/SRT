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

/** \file	STM32F4DMA.cpp
 *  \brief	Touch GFX Framework DMA.
 *
 *  This file contains the Touch GFX Framework DMA for Erika Enterprise.
 *
 *  \author	Giuseppe Serano
 *  \date	2020
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma2d.h"
#include <touchgfx/hal/OSWrappers.hpp>
#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/lcd/LCD.hpp>
#include <cassert>
#include <STM32DMA.hpp>
#include "ee.h"

using namespace touchgfx;

static HAL_StatusTypeDef HAL_DMA2D_SetMode(DMA2D_HandleTypeDef* hdma2d, uint32_t mode, uint32_t color, uint32_t offset)
{
    assert_param(IS_DMA2D_ALL_INSTANCE(hdma2d->Instance));

    MODIFY_REG(hdma2d->Instance->CR, DMA2D_CR_MODE, mode);
    MODIFY_REG(hdma2d->Instance->OPFCCR, DMA2D_OPFCCR_CM, color);
    MODIFY_REG(hdma2d->Instance->OOR, DMA2D_OOR_LO, offset);

    return HAL_OK;
}

extern "C" {
    extern DMA2D_HandleTypeDef hdma2d;

    ISR1(Dma2dISR)
    {
      HAL_DMA2D_IRQHandler(&hdma2d);
    }

    static void DMA2D_XferCpltCallback(DMA2D_HandleTypeDef* handle)
    {
        touchgfx::HAL::getInstance()->signalDMAInterrupt();
    }

    static void DMA2D_XferErrorCallback(DMA2D_HandleTypeDef* handle)
    {
        //assert(0);
    }
}

STM32F4DMA::STM32F4DMA()
    : DMA_Interface(dma_queue), dma_queue(queue_storage, sizeof(queue_storage) / sizeof(queue_storage[0]))
{}

STM32F4DMA::~STM32F4DMA()
{
    HAL_DMA2D_DeInit(&hdma2d);
}

void STM32F4DMA::initialize()
{
    hdma2d.Instance = DMA2D;
    HAL_DMA2D_Init(&hdma2d);

    hdma2d.XferCpltCallback = DMA2D_XferCpltCallback;
    hdma2d.XferErrorCallback = DMA2D_XferErrorCallback;
}

volatile bool disableChromArt = false;

BlitOperations STM32F4DMA::getBlitCaps()
{
    if (disableChromArt)
    {
        return static_cast<BlitOperations>(0);
    }
    else
    {
        return static_cast<BlitOperations>(BLIT_OP_FILL
                                           | BLIT_OP_FILL_WITH_ALPHA
                                           | BLIT_OP_COPY
                                           | BLIT_OP_COPY_WITH_ALPHA
                                           | BLIT_OP_COPY_ARGB8888
                                           | BLIT_OP_COPY_ARGB8888_WITH_ALPHA
                                           | BLIT_OP_COPY_A4
                                           | BLIT_OP_COPY_A8);
    }
}

void STM32F4DMA::setupDataCopy(const BlitOp& blitOp)
{
    uint32_t dma2dTransferMode = DMA2D_M2M_BLEND;
    uint32_t dma2dColorMode = 0;

    bool blendingImage = (blitOp.operation == BLIT_OP_COPY_ARGB8888
                          || blitOp.operation == BLIT_OP_COPY_ARGB8888_WITH_ALPHA
                          || blitOp.operation == BLIT_OP_COPY_WITH_ALPHA);

    bool blendingText = (blitOp.operation == BLIT_OP_COPY_A4
                         || blitOp.operation == BLIT_OP_COPY_A8);

    uint8_t bitDepth = HAL::lcd().bitDepth();

    switch (blitOp.operation)
    {
    case BLIT_OP_COPY_A4:
        dma2dColorMode = CM_A4;
        break;
    case BLIT_OP_COPY_A8:
        dma2dColorMode = CM_A8;
        break;
    case BLIT_OP_COPY_WITH_ALPHA:
        dma2dTransferMode = DMA2D_M2M_BLEND;
        dma2dColorMode = (bitDepth == 16) ? CM_RGB565 : CM_RGB888;
        break;
    case BLIT_OP_COPY_ARGB8888:
    case BLIT_OP_COPY_ARGB8888_WITH_ALPHA:
        dma2dColorMode = CM_ARGB8888;
        break;
    default:
        dma2dTransferMode = DMA2D_M2M;
        dma2dColorMode = (bitDepth == 16) ? CM_RGB565 : CM_RGB888;
        break;
    }

    /* HAL_DMA2D_ConfigLayer() depends on hdma2d.Init! */
    hdma2d.Init.Mode = dma2dTransferMode;
    hdma2d.Init.ColorMode = (bitDepth == 16) ? DMA2D_RGB565 : DMA2D_RGB888;
    hdma2d.Init.OutputOffset = blitOp.dstLoopStride - blitOp.nSteps;

    HAL_DMA2D_SetMode(&hdma2d, dma2dTransferMode,
                      (bitDepth == 16) ? DMA2D_RGB565 : DMA2D_RGB888,
                      blitOp.dstLoopStride - blitOp.nSteps);

    hdma2d.LayerCfg[1].InputColorMode = dma2dColorMode;
    hdma2d.LayerCfg[1].InputOffset = blitOp.srcLoopStride - blitOp.nSteps;

    if (blendingImage || blendingText)
    {
        if (blitOp.alpha < 255)
        {
            hdma2d.LayerCfg[1].AlphaMode = DMA2D_COMBINE_ALPHA;
            hdma2d.LayerCfg[1].InputAlpha = blitOp.alpha;
        }
        else
        {
            hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
        }

        if (blendingText)
        {
            if (bitDepth == 16)
            {
                uint32_t red = (((blitOp.color & 0xF800) >> 11) * 255) / 31;
                uint32_t green = (((blitOp.color & 0x7E0) >> 5) * 255) / 63;
                uint32_t blue = (((blitOp.color & 0x1F)) * 255) / 31;
                uint32_t alpha = blitOp.alpha;
                hdma2d.LayerCfg[1].InputAlpha = (alpha << 24) | (red << 16) | (green << 8) | blue;
            }
            else
            {
                hdma2d.LayerCfg[1].InputAlpha = blitOp.color.getColor32() | (blitOp.alpha << 24);
            }
        }

        hdma2d.LayerCfg[0].InputOffset = blitOp.dstLoopStride - blitOp.nSteps;
        hdma2d.LayerCfg[0].InputColorMode = (bitDepth == 16) ? CM_RGB565 : CM_RGB888;

        HAL_DMA2D_ConfigLayer(&hdma2d, 0);
    }

    HAL_DMA2D_ConfigLayer(&hdma2d, 1);

    if (blendingImage || blendingText)
    {
        HAL_DMA2D_BlendingStart_IT(&hdma2d,
                                   (unsigned int)blitOp.pSrc,
                                   (unsigned int)blitOp.pDst,
                                   (unsigned int)blitOp.pDst,
                                   blitOp.nSteps, blitOp.nLoops);
    }
    else
    {
        HAL_DMA2D_Start_IT(&hdma2d,
                           (unsigned int)blitOp.pSrc,
                           (unsigned int)blitOp.pDst,
                           blitOp.nSteps, blitOp.nLoops);
    }
}

void STM32F4DMA::setupDataFill(const BlitOp& blitOp)
{
    uint8_t bitDepth = HAL::lcd().bitDepth();
    uint32_t dma2dTransferMode;
    uint32_t dma2dColorMode = (bitDepth == 16) ? CM_RGB565 : CM_RGB888;

    uint32_t color = 0;
    if (bitDepth == 16)
    {
        uint32_t red = (((blitOp.color & 0xF800) >> 11) * 255) / 31;
        uint32_t green = (((blitOp.color & 0x7E0) >> 5) * 255) / 63;
        uint32_t blue = (((blitOp.color & 0x1F)) * 255) / 31;
        uint32_t alpha = blitOp.alpha;
        color = (alpha << 24) | (red << 16) | (green << 8) | blue;
    }
    else
    {
        color = (blitOp.alpha << 24) | blitOp.color.getColor32();
    }

    switch (blitOp.operation)
    {
    case BLIT_OP_FILL_WITH_ALPHA:
        dma2dTransferMode = DMA2D_M2M_BLEND;
        break;
    default:
        dma2dTransferMode = DMA2D_R2M;
        break;
    };

    /* HAL_DMA2D_ConfigLayer() depends on hdma2d.Init! */
    hdma2d.Init.Mode = dma2dTransferMode;
    hdma2d.Init.ColorMode = (bitDepth == 16) ? DMA2D_RGB565 : DMA2D_RGB888;
    hdma2d.Init.OutputOffset = blitOp.dstLoopStride - blitOp.nSteps;

    HAL_DMA2D_SetMode(&hdma2d, dma2dTransferMode,
                      (bitDepth == 16) ? DMA2D_RGB565 : DMA2D_RGB888,
                      blitOp.dstLoopStride - blitOp.nSteps);

    if (dma2dTransferMode == DMA2D_M2M_BLEND)
    {
        hdma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
        hdma2d.LayerCfg[1].InputAlpha = color;
        hdma2d.LayerCfg[1].InputColorMode = CM_A8;

        hdma2d.LayerCfg[0].InputOffset = blitOp.dstLoopStride - blitOp.nSteps;
        hdma2d.LayerCfg[0].InputColorMode = (bitDepth == 16) ? CM_RGB565 : CM_RGB888;
        HAL_DMA2D_ConfigLayer(&hdma2d, 0);
    }
    else
    {
        hdma2d.LayerCfg[1].InputColorMode = dma2dColorMode;
        hdma2d.LayerCfg[1].InputOffset = 0;
    }

    HAL_DMA2D_ConfigLayer(&hdma2d, 1);

    if (dma2dTransferMode == DMA2D_M2M_BLEND)
        HAL_DMA2D_BlendingStart_IT(&hdma2d,
                                  (unsigned int)blitOp.pDst,
                                  (unsigned int)blitOp.pDst,
                                  (unsigned int)blitOp.pDst,
                                  blitOp.nSteps, blitOp.nLoops);
    else
        HAL_DMA2D_Start_IT(&hdma2d, color, (unsigned int)blitOp.pDst,
                           blitOp.nSteps, blitOp.nLoops);
}
