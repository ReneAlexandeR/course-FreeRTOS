/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define TRUE 			1
#define FALSE			0
#define AVAILABLE		TRUE
#define NON_AVAILABLE	FALSE

TaskHandle_t xTask1_Handle = NULL;
TaskHandle_t xTask2_Handle = NULL;

uint8_t UART_ACCESS_KEY = AVAILABLE;
char message[100];

// Task function prototypes
void vTask1_Handler(void *params);
void vTask2_Handler(void *params);

static void prvSetupHardware();
static void prvSetupUART();

void printViaUART(char* msg);

#ifdef USE_SEMIHOSTING
// Used for semihosting
extern void initialise_monitor_handles();
#endif

int main(void)
{
#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("FreeRTOS demo example!\n");
#endif

	// Reset the clock configuration to default:
	// HSI ON, HSE and PLL OFF system clock = 16 MHz
	RCC_DeInit();

	// Update SystemCoreClock variable
	SystemCoreClockUpdate();

	//Configure peripherals
	prvSetupHardware();

	sprintf(message, "Hello world application starting\r\n");
	printViaUART(message);

	// Create tasks
	xTaskCreate( vTask1_Handler,
                 "Task-1", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
				 configMINIMAL_STACK_SIZE,
                 NULL,
                 2,
                 &xTask1_Handle );

	xTaskCreate( vTask2_Handler,
                 "Task-2", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
				 configMINIMAL_STACK_SIZE,
                 NULL,
                 2,
                 &xTask2_Handle );

	// Start the scheduler
	vTaskStartScheduler();

	//
	for(;;);
}


void vTask1_Handler(void *params)
{
	while(1)
	{
		if (UART_ACCESS_KEY == AVAILABLE)
		{
			UART_ACCESS_KEY = NON_AVAILABLE;
			printViaUART("TASK 1 is running!\r\n");
			UART_ACCESS_KEY = AVAILABLE;
			taskYIELD();
		}
	}
}

void vTask2_Handler(void *params)
{
	while(1)
	{
		if (UART_ACCESS_KEY == AVAILABLE)
		{
			UART_ACCESS_KEY = NON_AVAILABLE;
			printViaUART("TASK 2 is running!\r\n");
			UART_ACCESS_KEY = AVAILABLE;
			taskYIELD();
		}
	}
}

static void prvSetupHardware(void)
{

	prvSetupUART();

}

static void prvSetupUART(void)
{
	GPIO_InitTypeDef gpio_uartPins; 		// GPIO initialization structure
	USART_InitTypeDef usart2_init;			// USART initialization structure

	// Clear the memory used by initalization variables
	memset(&gpio_uartPins, 0, sizeof(gpio_uartPins));
	memset(&usart2_init, 0, sizeof(usart2_init));

	// Enable USART2 and GPIOA peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//Initialize and configure functionality for GPIO pins 2
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

	// Enable USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

void printViaUART(char* msg)
{
	for (uint32_t i = 0; i < strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);
	}
}
