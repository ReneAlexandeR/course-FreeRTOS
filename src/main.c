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
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/**************************************************************************************************
 * 		DEFINES
 *************************************************************************************************/
#define TRUE 				1
#define FALSE				0
#define AVAILABLE			TRUE
#define NON_AVAILABLE		FALSE
#define BUTTON_PRESSED		TRUE
#define BUTTON_UNPRESSED	FALSE

#ifdef USE_SEMIHOSTING
// Used for semihosting
extern void initialise_monitor_handles();
#endif

/**************************************************************************************************
 * 		GLOBAL VARIABLES
 *************************************************************************************************/
char message[100];
uint8_t button_input_flag = FALSE;
void printViaUART(char* msg);

/**************************************************************************************************
 * 		FUNCTION PROTOTYPES
 *************************************************************************************************/
void button_handler(void *params);
void led_task_handler(void *params);

static void prvSetupHardware();
static void prvSetupUART();
static void prvSetupButtonAndLED();

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

	sprintf(message, "LED-BUTTON application starting\r\n");
	printViaUART(message);

	// Initialize recording with SEGGER
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	// Create tasks
	xTaskCreate( led_task_handler,
                 "LED-TASK", /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
				 configMINIMAL_STACK_SIZE,
                 NULL,
                 2,
                 NULL );

	// Start the scheduler
	vTaskStartScheduler();

	//
	for(;;);
}

/**************************************************************************************************
 * 		Button toggling handler
 *************************************************************************************************/
void button_handler(void *params)
{
	button_input_flag ^= 1;
}

/**************************************************************************************************
 * 		LED FreeRTOS task
 *************************************************************************************************/
void led_task_handler(void *params)
{
	while(1)
	{
		if (button_input_flag == BUTTON_PRESSED)
		{
			// Set the LED
			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);
		}
		else
		{
			// Reset the LED
			GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
		}
	}
}
/**************************************************************************************************
 * 		General hardware initialization and configuration
 *************************************************************************************************/
static void prvSetupHardware(void)
{

	prvSetupUART();
	prvSetupButtonAndLED();

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

	// Enable USART2 peripheral
	USART_Cmd(USART2, ENABLE);
}

/**************************************************************************************************
 * 		BUTTON and LED initialization and configuration
 *************************************************************************************************/
static void prvSetupButtonAndLED()
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

	// Interrupt configuration for button
	// System config for EXTI line
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	// Configure EXTI block (ST specific)
	// EXTI line 0 (PA0), falling edge, interrupt mode
	EXTI_InitTypeDef exti_init;
	exti_init.EXTI_Line = EXTI_Line0;
	exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
	exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
	exti_init.EXTI_LineCmd = ENABLE;

	EXTI_Init(&exti_init);

	// NVIC configuration (ARM specific)
	// for selected EXTI line
	NVIC_SetPriority(EXTI0_IRQn, 5);
	NVIC_EnableIRQ(EXTI0_IRQn);


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
 * 		External interrupt handler overloading for button detection
 *************************************************************************************************/
void EXTI0_IRQHandler(void)
{
	traceISR_ENTER();
	// Clear interrupt pending bit of the EXTI line
	EXTI_ClearITPendingBit(EXTI_Line0);
	button_handler(NULL);
	traceISR_EXIT();

}
