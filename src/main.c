/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/**************************************************************************************************
 * 		INCLUDES
 *************************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/**************************************************************************************************
 * 		DEFINES
 *************************************************************************************************/
#define TRUE 				1
#define FALSE				0

#ifdef USE_SEMIHOSTING
// Used for semihosting
extern void initialise_monitor_handles();
#endif

typedef struct{
	uint8_t number;
	uint8_t args[10];
} Command_t;

/**************************************************************************************************
 * 		GLOBAL VARIABLES
 *************************************************************************************************/
char usr_msg[250];

void printViaUART(char* msg);

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;
TaskHandle_t xTaskHandle3 = NULL;
TaskHandle_t xTaskHandle4 = NULL;

QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

TimerHandle_t LED_timer = NULL;

uint8_t cmd_buffer[20];
uint8_t cmd_length = 0;

enum {
	EXIT_APP = 0,
	LED_ON,
	LED_OFF,
	LED_TOGGLE,
	LED_TOGGLE_OFF,
	LED_READ_STATUS,
	RTC_PRINT_DATETIME
};

char menu[] = {"\
\r\nLED_ON             -----> 1 \		
\r\nLED_OFF            -----> 2 \
\r\nLED_TOGGLE         -----> 3 \
\r\nLED_TOGGLE_OFF     -----> 4 \
\r\nLED_READ_STATUS    -----> 5 \
\r\nRTC_PRINT_DATETIME -----> 6 \
\r\nEXIT_APP           -----> 0 \
\r\nType your option here:"};

/**************************************************************************************************
 * 		FUNCTION PROTOTYPES
 *************************************************************************************************/
void vTask1_menu_display(void *params);
void vTask2_cmd_handling(void *params);
void vTask3_cmd_processing(void *params);
void vTask4_uart_write(void *params);

static void prvSetupHardware();
static void prvSetupUART();
static void prvSetupGPIO();

static void cmd_toggle_LED(uint8_t cmd);
static void report_LED_status(char* msg);
static void print_err_msg(char* msg);
void toggle_LED_callback(TimerHandle_t xTimer);
static void report_RTC_info(char* msg);
static void exit_sequence(task_msg);

/**************************************************************************************************
 * 		Application MAIN function
 *************************************************************************************************/
int main(void)
{
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("FreeRTOS demo example!\n");
#endif

	// Enable CYCCNT in DWT_CTRL (MX Cycle counter)
	DWT->CTRL |= (1 << 0);

	// Reset the clock configuration to default:
	// HSI ON, HSE and PLL OFF system clock = 16 MHz
	RCC_DeInit();

	// Update SystemCoreClock variable
	SystemCoreClockUpdate();

	//Configure peripherals
	prvSetupHardware();

	sprintf(usr_msg, "\r\nQueue command application starting!\r\n");
	printViaUART(usr_msg);

	// Initialize recording with SEGGER
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();



	// Create the queues required
	command_queue = xQueueCreate(10, sizeof(Command_t*));

	uart_write_queue = xQueueCreate(10, sizeof(char*));

	// Proceed with task creation and scheduler start only if queue creation has been successful
	if ((command_queue != NULL) && (uart_write_queue != NULL))
	{
		// Create tasks
		xTaskCreate( vTask1_menu_display, "menu-display", 500, NULL, 1, &xTaskHandle1 );

		xTaskCreate( vTask2_cmd_handling, "cmd-handling", 500, NULL, 2, &xTaskHandle2 );

		xTaskCreate( vTask3_cmd_processing, "cmd-processing",  500, NULL, 2, &xTaskHandle3 );

		xTaskCreate( vTask4_uart_write, "uart-write", 500, NULL, 2, &xTaskHandle4 );

		// Start the scheduler
		vTaskStartScheduler();
	}


	//
	for(;;);
}

/**************************************************************************************************
 * 		Task 1 implementation
 *************************************************************************************************/
void vTask1_menu_display(void *params)
{
	char * pData = menu;

	while(1)
	{
		xQueueSend(uart_write_queue, &pData, portMAX_DELAY);

		// Wait until notification arrives
		xTaskNotifyWait(0x0, 0x0, NULL, portMAX_DELAY);
	}
}

/**************************************************************************************************
 * 		Task 2 implementation
 *************************************************************************************************/
void vTask2_cmd_handling(void *params)
{
	Command_t *new_cmd;

	while(1)
	{
		xTaskNotifyWait(0,0, NULL, portMAX_DELAY);			// The task will block until it receives something
		new_cmd = (Command_t*) pvPortMalloc(sizeof(Command_t));

		taskENTER_CRITICAL();
		new_cmd->number = cmd_buffer[0] - 48; 				// Extract the last item in the cmd_buffer and transform from ASCII
		xQueueSend(command_queue, &new_cmd, portMAX_DELAY); // Send to the queue, will block if the queue is full
		taskEXIT_CRITICAL();
	}
}

/**************************************************************************************************
 * 		Task 3 implementation
 *************************************************************************************************/
void vTask3_cmd_processing(void *params)
{
	Command_t *new_cmd;
	char task_msg[50];

	while(1)
	{
		xQueueReceive(command_queue, (void*)&new_cmd, portMAX_DELAY);

		switch(new_cmd->number) {

		case LED_ON:
			GPIO_SetBits(GPIOD, GPIO_Pin_12);
			break;

		case LED_OFF:
			GPIO_ResetBits(GPIOD, GPIO_Pin_12);
			break;

		case LED_TOGGLE:
			cmd_toggle_LED(LED_TOGGLE);
			break;

		case LED_TOGGLE_OFF:
			cmd_toggle_LED(LED_TOGGLE_OFF);
			break;

		case LED_READ_STATUS:
			report_LED_status(task_msg);
			break;

		case RTC_PRINT_DATETIME:
			report_RTC_info(task_msg);
			break;

		case EXIT_APP:
			exit_sequence(task_msg);
			break;

		default:
			print_err_msg(task_msg);
			break;
		}

		// Free the allocated memory for the command
		vPortFree(new_cmd);
	}
}


/**************************************************************************************************
 * 		Task 4 implementation
 *************************************************************************************************/
void vTask4_uart_write(void *params)
{
	char * pData = NULL;

	while(1)
	{
		xQueueReceive(uart_write_queue, &pData, portMAX_DELAY);
		printViaUART(pData);
	}
}


/**************************************************************************************************
 * 		General hardware initialization and configuration
 *************************************************************************************************/
static void prvSetupHardware(void)
{

	prvSetupUART();
	prvSetupGPIO();

}

/**************************************************************************************************
 * 		UART initialization and configuration
 *************************************************************************************************/
static void prvSetupUART(void)
{
	GPIO_InitTypeDef gpio_uartPins; 		// GPIO initialization structure
	USART_InitTypeDef usart2_init;			// USART initialization structure

	// Clear the memory used by initialization variables
	memset(&gpio_uartPins, 0, sizeof(gpio_uartPins));
	memset(&usart2_init, 0, sizeof(usart2_init));

	// Enable USART2, GPIOA  and GPIOD peripheral clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//Initialize and configure functionality for:
	//GPIO uart pins 2 and 3
	gpio_uartPins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio_uartPins.GPIO_Mode = GPIO_Mode_AF;
	gpio_uartPins.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpio_uartPins);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // Configure PA2 as TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // Configure PA2 as RX

	// Initialize and configure USART2
	usart2_init.USART_BaudRate = 115200;
	usart2_init.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
	usart2_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart2_init.USART_Parity = USART_Parity_No;
	usart2_init.USART_StopBits = USART_StopBits_1;
	usart2_init.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2, &usart2_init);

	//Enable UART interrupt reception byte
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// Set the priority in NVIC and ENABLE
	NVIC_SetPriority(USART2_IRQn, 5);
	NVIC_EnableIRQ(USART2_IRQn);

	// Enable USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

/**************************************************************************************************
 * 		BUTTON and LED initialization and configuration
 *************************************************************************************************/
static void prvSetupGPIO()
{
	GPIO_InitTypeDef gpio_pushButton;		// GPIOA push button init struct
	GPIO_InitTypeDef gpio_led;				// GPIOD LED init struct

	// Clear the memory used by initialization variables
	memset(&gpio_pushButton, 0, sizeof(gpio_pushButton));
	memset(&gpio_led, 0, sizeof(gpio_led));

	// Enable peripheral clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// GPIOA
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	// GPIOD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);	// SYSCFG

	// Push button
	gpio_pushButton.GPIO_Mode = GPIO_Mode_IN;
	gpio_pushButton.GPIO_Pin = GPIO_Pin_0;
	gpio_pushButton.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &gpio_pushButton);

//	// Interrupt configuration for button
//	// System config for EXTI line
//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
//
//	// Configure EXTI block (ST specific)
//	// EXTI line 0 (PA0), falling edge, interrupt mode
//	EXTI_InitTypeDef exti_init;
//	exti_init.EXTI_Line = EXTI_Line0;
//	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
//	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
//	exti_init.EXTI_LineCmd = ENABLE;
//
//	EXTI_Init(&exti_init);
//
//	// NVIC configuration (ARM specific)
//	// for selected EXTI line
//	NVIC_SetPriority(EXTI0_IRQn, 5);
//	NVIC_EnableIRQ(EXTI0_IRQn);


	// LED
	gpio_led.GPIO_Mode = GPIO_Mode_OUT;
	gpio_led.GPIO_Pin = GPIO_Pin_12;
	gpio_led.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_led.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOD, &gpio_led);
}

/**************************************************************************************************
 * 		Print messages via UART peripheral
 *************************************************************************************************/
void printViaUART(char* msg)
{
	for (uint32_t i = 0; i < strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);
	}
}

/**************************************************************************************************
 * 		External interrupt handler
 *************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint16_t data;
	BaseType_t highPrioAwaken = pdFALSE;

	traceISR_ENTER();
	// Clear interrupt pending bit of the EXTI line
	EXTI_ClearITPendingBit(EXTI_Line0);

	if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE))
	{
		// Data is received from the user
		data = USART_ReceiveData(USART2);
		cmd_buffer[cmd_length++] = data & 0xFF;

		if (data == '\r') // Enter key detection
		{
			// Reset the command length variable
			cmd_length = 0;

			// Notify the cmd_handling task
			xTaskGenericNotifyFromISR(xTaskHandle2, 0, eNoAction,NULL, &highPrioAwaken);
			xTaskGenericNotifyFromISR(xTaskHandle1, 0, eNoAction,NULL, &highPrioAwaken);
		}
	}

	// If any high prio task is woken in the previous API calls, we yield the processor
	if (highPrioAwaken)
	{
		taskYIELD();
	}
	traceISR_EXIT();

}

static void cmd_toggle_LED(uint8_t cmd)
{
	if (cmd == LED_TOGGLE)
	{
		if (LED_timer == NULL)
		{
			// Create and start software timer
			LED_timer = xTimerCreate("LED-toggle-timer", pdMS_TO_TICKS(500), pdTRUE, NULL, toggle_LED_callback);
			xTimerStart(LED_timer, portMAX_DELAY);
		}
		else
		{
			xTimerStart(LED_timer, portMAX_DELAY);
		}
	}
	else if (cmd == LED_TOGGLE_OFF)
	{
		if (LED_timer != NULL)
		{
			xTimerStop(LED_timer, portMAX_DELAY);
		}
	}
}

static void report_LED_status(char* msg)
{
	sprintf(msg, "\r\nLED status is: %d \r\n", GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_12));
	xQueueSend(uart_write_queue, &msg, portMAX_DELAY);
}

static void print_err_msg(char* msg)
{
	sprintf(msg, "\r\nInvalid command: please enter a number between 0 and 6. \r\n");
	xQueueSend(uart_write_queue, &msg, portMAX_DELAY);
}

void toggle_LED_callback(TimerHandle_t xTimer)
{
	GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
}

static void report_RTC_info(char* msg)
{
	RTC_DateTypeDef RTC_date;
	RTC_TimeTypeDef RTC_time;

	RTC_GetDate(RTC_Format_BIN, &RTC_date);
	RTC_GetTime(RTC_Format_BIN, &RTC_time);

	sprintf(msg, "Time: %d:%d:%d\tDate:\r\n%2d-%2d-%02d\r\n", RTC_time.RTC_Hours, RTC_time.RTC_Minutes, RTC_time.RTC_Seconds,
                                                              RTC_date.RTC_Year, RTC_date.RTC_Month, RTC_date.RTC_Date);
	xQueueSend(uart_write_queue, &msg, portMAX_DELAY);


}

static void exit_sequence(char* msg)
{
	sprintf(msg, "\r\nExit application, performing cleanup and entering sleep mode. \r\n");
	xQueueSend(uart_write_queue, &msg, portMAX_DELAY);


	// Delete all tasks
	vTaskDelete(xTaskHandle1);
	vTaskDelete(xTaskHandle2);
	vTaskDelete(xTaskHandle4);
	// Delete the current task last
	vTaskDelete(xTaskHandle3);

	// Disable all interrupts
	NVIC_DisableIRQ(USART2_IRQn);

	// Send the MCU to low power mode
}

void vApplicationIdleHook(void)
{
	// Send the application to sleep mode
	__WFI();
}

