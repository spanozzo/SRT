#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/hal/BoardConfiguration.hpp>
#include "MessageQueue.hpp"
#include <random>
#include <stdio.h>
#include <string.h>
#include <list>
#include "DMA.cpp"

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
    HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
    ++buttonCnt;
    ActivateTask(Task0);
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
volatile uint32_t uart_timestamp;
volatile uint32_t error_timestamp;
volatile uint32_t nSerial = 0;

MessageQueue* msg = MessageQueue::getInstance();



TASK(TaskBackGround)
{
    for(;;);
}

volatile uint32_t timer = 0;
static uint8_t task0Buffer[32];

volatile uint32_t len0 = 0;

volatile bool win = true;

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
	win = false;
	++task0Cnt;

	timestamp0_ms = HAL_GetTick();

	uint16_t load = touchgfx::HAL::getInstance()->getMCULoadPct();

//	GetResource(SerialResource);
	if(timestamp0_start_ms != timestamp0_ms)
		sprintf((char *)task0Buffer, "%u-0-%u-%u-C%u%%\n", nSerial, timestamp0_start_ms, timestamp0_ms, load);
	else
		sprintf((char *)task0Buffer, "%u-0-%u-C%u%%\n", nSerial, timestamp0_start_ms, load);

	/*
	 * Prova scrittura diretta
	len0 = strlen((char *)task0Buffer);
	sprintf((char*)(_tx_buffer->buffer + _tx_buffer->head), (const char*)task0Buffer);
	_tx_buffer->head += strlen((char *)task0Buffer);
	Uart_sendstring ();
	*/

//	Uart_sendstring ((const char*)task0Buffer);
//	++nSerial;
//	ReleaseResource(SerialResource);
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
		if(!win) {
			win = true;
			++P1Win;
			ActivateTask(Task0);	// consentito, mette Task0 in stato ready, non viene eseguito subito essendo a priorità minore
		}
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

	if(timestamp2_start_ms != timestamp2_ms)
		sprintf((char *)task2Buffer, "%u-2-%u-%u\n", nSerial, timestamp2_start_ms, timestamp2_ms);
	else
		sprintf((char *)task2Buffer, "%u-2-%u\n", nSerial, timestamp2_start_ms);
//	if ((P1PosX == targetX) && (P1PosY == targetY))
//		ChainTask(Task0);

//	Uart_sendstring ((const char*)task2Buffer);
//	++nSerial;
    TerminateTask();
}

volatile uint32_t task3Cnt;
volatile uint16_t P2PosX = 40;
volatile uint16_t P2PosY = 155;
volatile uint16_t P2Win = 0;
static uint8_t task3Buffer[32];

TASK(Task3) {
	if(task3Cnt == 0)
		timestamp3_init_ms = HAL_GetTick();
	timestamp3_start_ms = HAL_GetTick();

	if ((P2PosX == targetX) && (P2PosY == targetY)) {
		if(!win) {
			win = true;
			++P2Win;
			ActivateTask(Task0);	// consentito, mette Task0 in stato ready, non viene eseguito subito essendo a priorità minore
		}
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

	if(timestamp3_start_ms != timestamp3_ms)
		sprintf((char *)task3Buffer, "%u-3-%u-%u\n", nSerial, timestamp3_start_ms, timestamp3_ms);
	else
		sprintf((char *)task3Buffer, "%u-3-%u\n", nSerial, timestamp3_start_ms);

//	Uart_sendstring ((const char*)task3Buffer);
//	++nSerial;

//	if ((P2PosX == targetX) && (P2PosY == targetY))
//		ChainTask(Task0);

    TerminateTask();
}

volatile uint32_t task4Cnt;
volatile uint16_t P3PosX = 40;
volatile uint16_t P3PosY = 250;
volatile uint16_t P3Win = 0;
static uint8_t task4Buffer[32];

volatile uint16_t load4 = 0;
volatile int plusLoad = 0;
volatile int sum = 0;
volatile bool stoppi = false;

TASK(Task4)
{
	if(task4Cnt == 0)
		timestamp4_init_ms = HAL_GetTick();
	timestamp4_start_ms = HAL_GetTick();

	++task4Cnt;

	if(timestamp4_start_ms > 10000)
		stoppi = true;

//	plusLoad = (((timestamp4_start_ms / 10000) + 1) * 100 ) + 100;
//
//	if(plusLoad > 1100)
//		stoppi = true;
//
//	for(int i = 0; i < plusLoad; ++i)
//		for(int z = 0; z < plusLoad; ++z)
//			sum += i+z;

	if ((P3PosX == targetX) && (P3PosY == targetY)) {
		if(!win) {
			win = true;
			++P3Win;
			ActivateTask(Task0);	// consentito, mette Task0 in stato ready, non viene eseguito subito essendo a priorità minore
		}
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
	load4 = touchgfx::HAL::getInstance()->getMCULoadPct();

	if(timestamp4_start_ms != timestamp4_ms)
		sprintf((char *)task4Buffer, "%u-4-%u-%u-PLOAD%u-C%u%%\n", nSerial, timestamp4_start_ms, timestamp4_ms, plusLoad, load4);
	else
		sprintf((char *)task4Buffer, "%u-4-%u--C%u%%\n", nSerial, timestamp4_ms, load4);

	Uart_sendstring ((const char*)task4Buffer);
	++nSerial;

//	if ((P3PosX == targetX) && (P3PosY == targetY))
//		ChainTask(Task0);

    TerminateTask();
}


//--------------------------------------------------------------------------------------------
//DeclareTask(Task5);
//DeclareTask(Task6);
//DeclareTask(Task7);
//
//volatile uint32_t task5Cnt;
//volatile uint32_t task6Cnt;
//volatile uint32_t task7Cnt;
//
//volatile uint32_t timestamp5_init_ms;
//volatile uint32_t timestamp5_start_ms;
//volatile uint32_t timestamp5_ms;
//volatile uint32_t timestamp6_init_ms;
//volatile uint32_t timestamp6_start_ms;
//volatile uint32_t timestamp6_ms;
//volatile uint32_t timestamp7_init_ms;
//volatile uint32_t timestamp7_start_ms;
//volatile uint32_t timestamp7_ms;
//static uint8_t task5Buffer[32];
//
//TASK(Task5)
//{
//	if(task5Cnt == 0)
//		timestamp5_init_ms = HAL_GetTick();
//	timestamp5_start_ms = HAL_GetTick();
//
//	if ((P1PosX == targetX) && (P1PosY == targetY)) {
//		P1Win = P1Win;
//	}
//	else {
//		if (P1PosX < targetX)
//			P1PosX = P1PosX;
//		else
//			if (P1PosX > targetX)
//				P1PosX = P1PosX;
//		if (P1PosY < targetY)
//			P1PosX = P1PosX;
//		else
//			if (P1PosY > targetY)
//				P1PosX = P1PosX;
//	}
//
//	++task5Cnt;
//	timestamp5_ms = HAL_GetTick();
//
//	if(timestamp5_start_ms != timestamp5_ms)
//		sprintf((char *)task5Buffer, "%u-5-%u-%u\n", nSerial, timestamp5_start_ms, timestamp5_ms);
//	else
//		sprintf((char *)task5Buffer, "%u-5-%u\n", nSerial, timestamp5_ms);
//
////	Uart_sendstring ((const char*)task5Buffer);
////	++nSerial;
//
//    TerminateTask();
//}
//
//static uint8_t task6Buffer[32];
//
//TASK(Task6)
//{
//	if(task6Cnt == 0)
//		timestamp6_init_ms = HAL_GetTick();
//	timestamp6_start_ms = HAL_GetTick();
//
//	if ((P1PosX == targetX) && (P1PosY == targetY)) {
//		P1Win = P1Win;
//	}
//	else {
//		if (P1PosX < targetX)
//			P1PosX = P1PosX;
//		else
//			if (P1PosX > targetX)
//				P1PosX = P1PosX;
//		if (P1PosY < targetY)
//			P1PosX = P1PosX;
//		else
//			if (P1PosY > targetY)
//				P1PosX = P1PosX;
//	}
//
//	++task6Cnt;
//	timestamp6_ms = HAL_GetTick();
//
//	if(timestamp6_start_ms != timestamp6_ms)
//		sprintf((char *)task6Buffer, "%u-6-%u-%u\n", nSerial, timestamp6_start_ms, timestamp6_ms);
//	else
//		sprintf((char *)task6Buffer, "%u-6-%u\n", nSerial, timestamp6_ms);
//
////	Uart_sendstring ((const char*)task6Buffer);
////	++nSerial;
//
//    TerminateTask();
//}
//
//static uint8_t task7Buffer[32];
//
//TASK(Task7)
//{
//	if(task7Cnt == 0)
//		timestamp7_init_ms = HAL_GetTick();
//	timestamp7_start_ms = HAL_GetTick();
//
//	if ((P1PosX == targetX) && (P1PosY == targetY)) {
//		P1Win = P1Win;
//	}
//	else {
//		if (P1PosX < targetX)
//			P1PosX = P1PosX;
//		else
//			if (P1PosX > targetX)
//				P1PosX = P1PosX;
//		if (P1PosY < targetY)
//			P1PosX = P1PosX;
//		else
//			if (P1PosY > targetY)
//				P1PosX = P1PosX;
//	}
//
//	++task7Cnt;
//	timestamp7_ms = HAL_GetTick();
//
//	if(timestamp7_start_ms != timestamp7_ms)
//		sprintf((char *)task7Buffer, "%u-7-%u-%u\n", nSerial, timestamp7_start_ms, timestamp7_ms);
//	else
//		sprintf((char *)task7Buffer, "%u-7-%u\n", nSerial, timestamp7_ms);
//
////	Uart_sendstring ((const char*)task7Buffer);
////	++nSerial;
//
//    TerminateTask();
//}
//
//volatile uint32_t errorT5Cnt = 0;
//volatile uint32_t errorT6Cnt = 0;
//volatile uint32_t errorT7Cnt = 0;
//--------------------------------------------------------------------------------------------

TaskType lastTaskID = Task0;
volatile uint32_t last_timestamp;
bool bkgFlag = false;

/* Hooks */
void PreTaskHook(void)
{
	TaskType TaskID;
	GetTaskID(&TaskID);

	if (TaskID == TaskBackGround) {
		touchgfx::HAL::getInstance()->setMCUActive(OSEE_FALSE);
		timestampBkg_start_ms = HAL_GetTick();
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

		timestampBkg_ms = HAL_GetTick();
		if(timestampBkg_start_ms == timestampBkg_ms)
			sprintf((char *)taskBkgBuffer, "%u-B-%u\n", nSerial, timestampBkg_start_ms);
		else
			sprintf((char *)taskBkgBuffer, "%u-B-%u-%u\n", nSerial, timestampBkg_start_ms, timestampBkg_ms);
//		Uart_sendstring ((const char*)taskBkgBuffer);
//		++nSerial;
	}

	if(TaskID == Task1) {
		timestamp1_ms = HAL_GetTick();
		if(timestamp1_start_ms == timestamp1_ms)
			sprintf((char *)task1Buffer, "%u-1-%u\n", nSerial, timestamp1_start_ms);
		else
			sprintf((char *)task1Buffer, "%u-1-%u-%u\n", nSerial, timestamp1_start_ms, timestamp1_ms);
//		Uart_sendstring ((const char*)task1Buffer);
//		++nSerial;
	}

//	if ((TaskID != Task1) && (lastTaskID == Task1)){
//		timestamp1_ms = last_timestamp;
//		if(timestamp1_start_ms == timestamp1_ms)
//			sprintf((char *)task1Buffer, "%u-1-%u\n", nSerial, timestamp1_start_ms);
//		else
//			sprintf((char *)task1Buffer, "%u-1-%u-%u\n", nSerial, timestamp1_start_ms, timestamp1_ms);
//
//		Uart_sendstring ((const char*)task1Buffer);
//		++nSerial;
//	}
//
//	if ((TaskID != TaskBackGround) && (lastTaskID == TaskBackGround) && bkgFlag){
//		bkgFlag = false;
//		timestampBkg_ms = last_timestamp;
//		if(timestampBkg_start_ms == timestampBkg_ms)
//			sprintf((char *)taskBkgBuffer, "%u-B-%u\n", nSerial, timestampBkg_start_ms);
//		else
//			sprintf((char *)taskBkgBuffer, "%u-B-%u-%u\n", nSerial, timestampBkg_start_ms, timestampBkg_ms);
//
//		Uart_sendstring ((const char*)taskBkgBuffer);
//		++nSerial;
//	}
//
//	last_timestamp = HAL_GetTick();
//	lastTaskID = TaskID;
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
volatile bool gaga = false;


void ErrorHook(StatusType Error)
{

	if(Error == E_OS_LIMIT) {
		TaskType TaskID;
		GetTaskID(&TaskID);
//		volatile OSServiceIdType serviceID = OSErrorGetServiceId();
//		volatile TaskType TaskError = OSError_ActivateTask_TaskID();
//		volatile TaskType AlarmError = OSError_GetAlarm_AlarmID();
//		volatile TaskType TaskError1 = OSError_OSId_Action_TaskID();
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
				++errorT2Cnt;
			break;
			case Task3:
				taskN = 3;
				++errorT3Cnt;
			break;
			case Task4:
				taskN = 4;
				++errorT4Cnt;
			break;
//			-------------------------------------------------------------
//			case Task5:
//				taskN = 5;
//				++errorT5Cnt;
//			break;
//			case Task6:
//				taskN = 6;
//				++errorT6Cnt;
//			break;
//			case Task7:
//				taskN = 7;
//				++errorT7Cnt;
//			break;
//			-------------------------------------------------------------
		};
		error_timestamp = HAL_GetTick();
		sprintf((char *)errorHookBuffer, "%u-ERR%u-%u-PLOAD%u-C%u%%\n", nSerial, taskN, error_timestamp, plusLoad, load4);
		Uart_sendstring ((const char*)errorHookBuffer);
		++nSerial;
	}
    ++errorCnt;
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
