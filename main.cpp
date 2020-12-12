#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/hal/BoardConfiguration.hpp>
#include "MessageQueue.hpp"
#include <random>
#include <stdio.h>
#include <string.h>
#include <list>

using namespace touchgfx;


/* Kernel includes. */
#include "ee.h"

/* TASKs Declaration */
DeclareTask(TaskBackGround);
DeclareTask(Task0);
DeclareTask(Task1);
DeclareTask(Task2);
DeclareTask(Task3);
DeclareTask(Task4);

/* ISRs */
volatile uint32_t buttonCnt = 0;

ISR2(ButtonsISR)
{
    ActivateTask(Task0);
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
    ++buttonCnt;
}

/* TASKs */
volatile int targetX;
volatile int targetY;
volatile uint32_t task0Cnt;
std::random_device rd;     // only used once to initialise (seed) engine
std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
std::uniform_int_distribution<int> uniX(0, 230); // guaranteed unbiased
std::uniform_int_distribution<int> uniY(0, 310);

volatile uint32_t timestampBkg_start_ms;
volatile uint32_t timestampBkg_ms;
volatile uint32_t timestamp0_init_ms;
volatile uint32_t timestamp0_start_ms;
volatile uint32_t timestamp0_ms;
volatile uint32_t timestamp1_start_ms;
volatile uint32_t timestamp1_ms;
volatile uint32_t timestamp2_init_ms;
volatile uint32_t timestamp2_start_ms;
volatile uint32_t timestamp2_ms;
volatile uint32_t timestamp3_init_ms;
volatile uint32_t timestamp3_start_ms;
volatile uint32_t timestamp3_ms;
volatile uint32_t timestamp4_init_ms;
volatile uint32_t timestamp4_start_ms;
volatile uint32_t timestamp4_ms;
volatile uint32_t last_timestamp;
volatile uint32_t uart_timestamp;
volatile uint32_t error_timestamp;
volatile uint32_t nSerial = 0;

MessageQueue* msg = MessageQueue::getInstance();

extern UART_HandleTypeDef UartHandle;

#define UART_BUFFER_SIZE 4096

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;

ring_buffer tx_buffer = { { 0 }, 0, 0};

ring_buffer *_tx_buffer = &tx_buffer;

bool mid = true;
volatile uint32_t timer1 = 0;
volatile uint32_t timer2 = 0;

bool init = false;
volatile uint32_t initTimer = 0;

void Uart_sendstring (const char *s)
{
	// Buffer full!
	if(!init) {
		initTimer = HAL_GetTick();
		init = true;
	}
	if(_tx_buffer->head > UART_BUFFER_SIZE/2 && mid){
		mid = false;
		HAL_UART_Transmit_DMA(&UartHandle, _tx_buffer->buffer, _tx_buffer->head);
		// HAL_UART_Transmit_IT(&UartHandle, _tx_buffer->buffer, _tx_buffer->head);
		_tx_buffer->tail = _tx_buffer->head;
		timer1 = HAL_GetTick();
	}
	else {
		if(_tx_buffer->head > UART_BUFFER_SIZE - 32) {
			HAL_UART_Transmit_DMA(&UartHandle, _tx_buffer->buffer+_tx_buffer->tail, _tx_buffer->head-_tx_buffer->tail);
			// HAL_UART_Transmit_IT(&UartHandle, _tx_buffer->buffer+_tx_buffer->tail, _tx_buffer->head-_tx_buffer->tail);
			_tx_buffer->head = 0;
			_tx_buffer->tail = 0;
			mid = true;
			timer2 = HAL_GetTick();
		}
	}

	while(*s) {
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;
		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)*s++;
		_tx_buffer->head = i;
	}
}

TASK(TaskBackGround)
{
    for(;;);
}

volatile uint32_t timer = 0;
static uint8_t task0Buffer[32];

volatile uint32_t len0 = 0;

TASK(Task0)
{
	// display 240x320 - dimensione palla 10x10
	/*
	 * Scelgo due numeri casuali che rappresentano target.
	 * */
	if(task0Cnt == 0)
		timestamp0_init_ms = HAL_GetTick();
	timestamp0_start_ms = HAL_GetTick();

	targetX = uniX(rng);
	targetY = uniY(rng);
	msg->setTarget(targetX, targetY);
	++task0Cnt;

	timestamp0_ms = HAL_GetTick();

	GetResource(SerialResource);
	if(timestamp0_start_ms != timestamp0_ms)
		sprintf((char *)task0Buffer, "%u-0-%u-%u\n", nSerial, timestamp0_start_ms, timestamp0_ms);
	else
		sprintf((char *)task0Buffer, "%u-0-%u\n", nSerial, timestamp0_start_ms);

	/*
	 * Prova scrittura diretta
	len0 = strlen((char *)task0Buffer);
	sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task0Buffer);
	_tx_buffer->head += strlen((char *)task0Buffer);
	Uart_sendstring ();
	*/

	Uart_sendstring ((const char*)task0Buffer);
	++nSerial;
	ReleaseResource(SerialResource);
	TerminateTask();
}

TASK(Task1)
{
    SetEvent(Task1, FrameBufferEvent);
    GetResource(HalResource);
    touchgfx::HAL::getInstance()->taskEntry();
    TerminateTask();
}

volatile uint32_t task2Cnt;
volatile uint16_t P1PosX = 40;
volatile uint16_t P1PosY = 41;
volatile uint16_t P1Win = 0;
volatile uint16_t len = 0;
static uint8_t task2Buffer[32];

TASK(Task2)
{
	if(task2Cnt == 0)
		timestamp2_init_ms = HAL_GetTick();
	timestamp2_start_ms = HAL_GetTick();

	if ((P1PosX == targetX) && (P1PosY == targetY)) {
		++P1Win;
		ActivateTask(Task0);
	}
	else {
		if (P1PosX < targetX)
			++P1PosX;
		else
			if (P1PosX > targetX)
				--P1PosX;
		if (P1PosY < targetY)
			++P1PosY;
		else
			if (P1PosY > targetY)
				--P1PosY;
		msg -> setP1Position(P1PosX, P1PosY);
	}
	++task2Cnt;
	timestamp2_ms = HAL_GetTick();

	GetResource(SerialResource);
	if(timestamp2_start_ms != timestamp2_ms)
		sprintf((char *)task2Buffer, "%u-2-%u-%u\n", nSerial, timestamp2_start_ms, timestamp2_ms);
	else
		sprintf((char *)task2Buffer, "%u-2-%u\n", nSerial, timestamp2_start_ms);

	/*
	 * Prova scrittura diretta
	len0 = strlen((char *)task2Buffer);
	sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task2Buffer);
	_tx_buffer->head += strlen((char *)task2Buffer);
	Uart_sendstring ();
	*/

	Uart_sendstring ((const char*)task2Buffer);
	++nSerial;
	ReleaseResource(SerialResource);
    TerminateTask();
}

volatile uint32_t task3Cnt;
volatile uint16_t P2PosX = 40;
volatile uint16_t P2PosY = 155;
volatile uint16_t P2Win = 0;
static uint8_t task3Buffer[32];

TASK(Task3)
{
	if(task3Cnt == 0)
		timestamp3_init_ms = HAL_GetTick();
	timestamp3_start_ms = HAL_GetTick();

	if ((P2PosX == targetX) && (P2PosY == targetY)) {
		++P2Win;
		ActivateTask(Task0);
	}
	else {
		if (P2PosX < targetX)
			++P2PosX;
		else
			if (P2PosX > targetX)
				--P2PosX;
		if (P2PosY < targetY)
			++P2PosY;
		else
			if (P2PosY > targetY)
				--P2PosY;
		msg -> setP2Position(P2PosX, P2PosY);
	}

	++task3Cnt;
	timestamp3_ms = HAL_GetTick();

	GetResource(SerialResource);
	if(timestamp3_start_ms != timestamp3_ms)
		sprintf((char *)task3Buffer, "%u-3-%u-%u\n", nSerial, timestamp3_start_ms, timestamp3_ms);
	else
		sprintf((char *)task3Buffer, "%u-3-%u\n", nSerial, timestamp3_start_ms);

	/*
	 * Prova scrittura diretta
	len0 = strlen((char *)task3Buffer);
	sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task3Buffer);
	_tx_buffer->head += strlen((char *)task3Buffer);
	Uart_sendstring ();
	*/

	Uart_sendstring ((const char*)task3Buffer);
	++nSerial;
	ReleaseResource(SerialResource);

    TerminateTask();
}

volatile uint32_t task4Cnt;
volatile uint16_t P3PosX = 40;
volatile uint16_t P3PosY = 250;
volatile uint16_t P3Win = 0;
static uint8_t task4Buffer[32];

TASK(Task4)
{
	if(task4Cnt == 0)
		timestamp4_init_ms = HAL_GetTick();
	timestamp4_start_ms = HAL_GetTick();

	++task4Cnt;
	int sum = 0;
	/*
	for(int i = 0; i < targetX*10; ++i) {
		for(int z = 0; z < targetY*10; ++z) {
			sum += i+z;
		}
	}
	*/
	if ((P3PosX == targetX) && (P3PosY == targetY)) {
		++P3Win;
		ActivateTask(Task0);
	}
	else {
		if (P3PosX < targetX)
			++P3PosX;
		else
			if (P3PosX > targetX)
				--P3PosX;
		if (P3PosY < targetY)
			++P3PosY;
		else
			if (P3PosY > targetY)
				--P3PosY;
		msg -> setP3Position(P3PosX, P3PosY);
	}
	timestamp4_ms = HAL_GetTick();
	uint8_t load = touchgfx::HAL::getInstance()->getMCULoadPct();

	GetResource(SerialResource);
	if(timestamp4_start_ms != timestamp4_ms)
		sprintf((char *)task4Buffer, "%u-4-%u-%u-C%u%%\n", nSerial, timestamp4_start_ms, timestamp4_ms, load);
	else
		sprintf((char *)task4Buffer, "%u-4-%u--C%u%%\n", nSerial, timestamp4_ms, load);

	/*
	 * Prova scrittura diretta
	len0 = strlen((char *)task4Buffer);
	sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task4Buffer);
	_tx_buffer->head += strlen((char *)task4Buffer);
	Uart_sendstring ();
	*/

	Uart_sendstring ((const char*)task4Buffer);
	++nSerial;
	ReleaseResource(SerialResource);

    TerminateTask();
}

TaskType lastTaskID = Task0;
bool bkgFlag = false;

/* Hooks */
void PreTaskHook(void)
{
	TaskType TaskID;
	GetTaskID(&TaskID);

	if (TaskID == TaskBackGround) {
		touchgfx::HAL::getInstance()->setMCUActive(OSEE_FALSE);
		if(!bkgFlag) {
			timestampBkg_start_ms = HAL_GetTick();
			bkgFlag = true;
		}
	}

	if (TaskID == Task1) {
		timestamp1_start_ms = HAL_GetTick();
	}
}


static uint8_t task1Buffer[32];
static uint8_t taskBkgBuffer[32];

void PostTaskHook(void)
{
	TaskType TaskID;
	GetTaskID(&TaskID);

	if (TaskID == TaskBackGround) {
		touchgfx::HAL::getInstance()->setMCUActive(OSEE_TRUE);
	}
	if ((TaskID != Task1) && (lastTaskID == Task1)){
		timestamp1_ms = last_timestamp;
		if(timestamp1_start_ms == timestamp1_ms)
			sprintf((char *)task1Buffer, "%u-1-%u\n", nSerial, timestamp1_start_ms);
		else
			sprintf((char *)task1Buffer, "%u-1-%u-%u\n", nSerial, timestamp1_start_ms, timestamp1_ms);

		/*
		 * Prova scrittura diretta
		len0 = strlen((char *)task1Buffer);
		sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task1Buffer);
		_tx_buffer->head += strlen((char *)task1Buffer);
		Uart_sendstring ();
		*/

		// Uart_sendstring ((const char*)task1Buffer);
		// ++nSerial;
	}

	if ((TaskID != TaskBackGround) && (lastTaskID == TaskBackGround) && bkgFlag){
		bkgFlag = false;
		timestampBkg_ms = last_timestamp;
		if(timestampBkg_start_ms == timestampBkg_ms)
			sprintf((char *)taskBkgBuffer, "%u-B-%u\n", nSerial, timestampBkg_start_ms);
		else
			sprintf((char *)taskBkgBuffer, "%u-B-%u-%u\n", nSerial, timestampBkg_start_ms, timestampBkg_ms);

		/*
		 * Prova scrittura diretta
		len0 = strlen((char *)taskBkgBuffer);
		sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)taskBkgBuffer);
		_tx_buffer->head += strlen((char *)taskBkgBuffer);
		Uart_sendstring ();
		*/

		// Uart_sendstring ((const char*)taskBkgBuffer);
		// ++nSerial;
	}

	last_timestamp = HAL_GetTick();
	lastTaskID = TaskID;
}

void StartupHook(void)
{
    BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    ResumeOSInterrupts();
}

void idle_hook ( void )
{
	;
}

volatile uint32_t errorCnt = 0;
volatile uint32_t errorTBkgCnt = 0;
volatile uint32_t errorT0Cnt = 0;
volatile uint32_t errorT1Cnt = 0;
volatile uint32_t errorT2Cnt = 0;
volatile uint32_t errorT3Cnt = 0;
volatile uint32_t errorT4Cnt = 0;
volatile uint32_t taskN = 100;

static uint8_t errorHookBuffer[32];

void ErrorHook(StatusType Error)
{
	if(Error == E_OS_LIMIT) {
		TaskType TaskID;
		GetTaskID(&TaskID);
		switch (TaskID) {
			case TaskBackGround:
				taskN = 99;
				++errorTBkgCnt;
			break;
			case Task0:
				taskN = 0;
				++errorT0Cnt;
			break;
			case Task1:
				taskN = 1;
				++errorT1Cnt;
			break;
			case Task2:
				taskN = 2;
				errorT2Cnt;
			break;
			case Task3:
				taskN = 3;
				errorT3Cnt;
			break;
			case Task4:
				taskN = 4;
				errorT4Cnt;
			break;
		};
		error_timestamp = HAL_GetTick();
		sprintf((char *)errorHookBuffer, "%u-%u-%u\n", nSerial, taskN, error_timestamp);
		// Uart_sendstring ((const char*)errorHookBuffer);
		// ++nSerial;

		/*
		 * Prova scrittura diretta
		len0 = strlen((char *)errorHookBuffer);
		sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)errorHookBuffer);
		_tx_buffer->head += strlen((char *)errorHookBuffer);
		*/
	}
    ++errorCnt;
    switch (Error) {
    default:
       break;
    };
}

extern void touchgfx::hw_init();
extern void touchgfx::touchgfx_init();
volatile uint32_t timestamp;

int main(void)
{
    SuspendOSInterrupts();

    hw_init();
    touchgfx_init();
    StartOS(OSDEFAULTAPPMODE);
    return 0;
}
