#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/hal/BoardConfiguration.hpp>
#include <random>
#include <stdio.h>
#include <string.h>
#include <list>

extern UART_HandleTypeDef UartHandle;

#define UART_BUFFER_SIZE 65000 // 4096

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;

ring_buffer tx_buffer = { { 0 }, 0, 0};

ring_buffer *_tx_buffer = &tx_buffer;

bool greenFlag = true;
bool loggg = false;

void Uart_sendstring (const char *s)
{
	while(*s) {
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;
		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)*s++;
		_tx_buffer->head = i;
	}

	if(_tx_buffer->tail > _tx_buffer->head)
		loggg = true;
	if(HAL_GetTick() > 100000)
		loggg = false;

	if(greenFlag) {
		if(_tx_buffer->head > _tx_buffer->tail) {
			HAL_UART_Transmit_DMA(&UartHandle, _tx_buffer->buffer+_tx_buffer->tail, _tx_buffer->head-_tx_buffer->tail);
			_tx_buffer->tail = _tx_buffer->head;
		}
		else {
			HAL_UART_Transmit_DMA(&UartHandle, _tx_buffer->buffer+_tx_buffer->tail, UART_BUFFER_SIZE-_tx_buffer->tail);
			_tx_buffer->tail = 0;
		}
		greenFlag = false;
	}
}

static uint8_t txCpltBuffer[16];
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	greenFlag = true;
}
