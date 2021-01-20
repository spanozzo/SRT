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

/** \file	OSWrappers.cpp
 *  \brief	Touch GFX Framework OS Wrapper.
 *
 *  This file contains the Touch GFX Framework OS Wrapper for Erika Enterprise.
 *
 *  \author	Giuseppe Serano
 *  \date	2020
 */
#include "ee.h"
#include <touchgfx/hal/OSWrappers.hpp>
#include <touchgfx/hal/GPIO.hpp>
#include <touchgfx/hal/HAL.hpp>

using namespace touchgfx;

void OSWrappers::initialize()
{

}

// Called by graphics engine to get exclusive access to the framebuffer. This will block the engine until the DMA2D is done (if running)
void OSWrappers::takeFrameBufferSemaphore()
{
    WaitEvent(FrameBufferEvent);
    ClearEvent(FrameBufferEvent);
}

// Releases the framebuffer lock
void OSWrappers::giveFrameBufferSemaphore()
{
    SetEvent(Task1, FrameBufferEvent);
}

// Ensure that the lock is taken. This method does not block, but ensures that the next call to takeFrameBufferSemaphore will block its caller
void OSWrappers::tryTakeFrameBufferSemaphore()
{
    EventMaskType EventMask;
    GetEvent(Task1, &EventMask);
    if (EventMask & FrameBufferEvent) {
        ClearEvent(FrameBufferEvent);
    }
}

// Releases the framebuffer lock from an interrupt context
void OSWrappers::giveFrameBufferSemaphoreFromISR()
{
    SetEvent(Task1, FrameBufferEvent);
}

// This method should be called from the display driver when the display is ready for the next frame.
void OSWrappers::signalVSync()
{
    SetEvent(Task1, VSyncEvent);
}

// Called by the graphics engine to wait. Should not return until signalVSync is called.
void OSWrappers::waitForVSync()
{
    WaitEvent(VSyncEvent);
    ClearEvent(VSyncEvent);
}

//void OSWrappers::taskDelay(uint16_t ms)
//{
//    SetRelAlarm(AlarmTask1, ms, 0U);
//    WaitEvent(TimerEvent);
//    ClearEvent(TimerEvent);
//}
