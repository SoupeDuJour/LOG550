/*****************************************************************************
*
*	Prototype 2
*	Auteurs: Sophie Leduc Major, Paul Leboyer
*	LED_Flash()
*		Effectue le clignotement des LEDs au 200msec.
*		LED1 clignote toujours dès que votre microcontrôleur est alimenté.
*		LED2 clignote lorsque l’acquisition est en service.
*		LED3 s’allume et reste allumé si le « Message Queue » déborde au moins une fois.
*	UART_Cmd_RX()
*		Vérifie, à chaque 200msec, si des commandes sont reçues par le UART.
*		Si une commande est reçue, traiter celle-ci et envoyer l’ordre d’arrêt ou de départ à la tâche ADC_Cmd().
*	UART_SendSample()
*		Vide le « Message Queue » et envoi les échantillons au UART pour une transmission en direction du PC.
*	ADC_Cmd()
*		Cette tâche démarre les conversions, obtient les échantillons numérisés et les place dans le « Message Queue ». Ceci doit être fait à la bonne vitesse.
*		Si le « Message Queue » est plein, envoie l’information à la tâche AlarmMsgQ().
*	AlarmMsgQ()
*		Cette tâche est réveillé seulement si un débordement de la « Message Queue » survient. Elle commande l’allumage du LED3 en informant la tâche LED_Flash().
*	
*	Projet Squelette
*	Auteur : Maxime Turenne
*	Copyright : Maxime Turenne
*	Description: Demo d'utilisation de FreeRTOS dans le cadre d'un thermostate numérique.
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


/* Environment header files. */
#include "adc.h"
#include "board.h"
#include "compiler.h"
#include "conf_clock.h"
#include "delay.h"
#include "dip204.h"
#include "gpio.h"
#include "intc.h"
#include "pm.h"
#include "power_clocks_lib.h"
#include "spi.h"
#include "usart.h"


/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// tasks
static void ADC_Cmd(void *pvParameters);
static void UART_Cmd_RX(void *pvParameters);
static void AlarmMsgQ(void *pvParameters);
static void UART_SendSample(void *pvParameters);
static void LED_Flash(void *pvParameters);

// initialization functions
void initialiseLCD(void);
void init_usart(void);

// global var
volatile uint8_t messageQueue = 0;
volatile U8 AQUIS_START = 0;
volatile uint8_t flagError = 0;
volatile U16 adc_value_pot = 0;
volatile U16 adc_value_light = 0;
volatile U16 sampleCtr = 0;

// semaphore
static xSemaphoreHandle POTENTIOMETER_SEMAPHORE = NULL;
static xSemaphoreHandle LIGHT_SEMAPHORE = NULL;
static xSemaphoreHandle UART_SEMAPHORE = NULL;
static xSemaphoreHandle FLASH_LED1_SEMAPHORE = NULL;
static xSemaphoreHandle MESSAGEQ_SEMAPHORE = NULL;
static xSemaphoreHandle FLAGERROR_SEMAPHORE = NULL;
static xSemaphoreHandle SAMPLECTR_SEMAPHORE = NULL;

int main(void) {
	// Configure Osc0 in crystal mode (i.e. use of an external crystal source, with
	// frequency FOSC0) with an appropriate startup time then switch the main clock
	// source to Osc0.
	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

	/* Setup the LED's all off */
	LED_Display(0);

	initialiseLCD();
	init_usart();

	POTENTIOMETER_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	LIGHT_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	UART_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	FLASH_LED1_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	MESSAGEQ_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	FLAGERROR_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	SAMPLECTR_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	
	/* Start the demo tasks defined within this file. */
	xTaskCreate(
	LED_Flash
	, (const signed portCHAR *)"Led flash"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 1
	, NULL );
	
	xTaskCreate(
	UART_Cmd_RX
	, (const signed portCHAR *)"UART commande"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 2
	, NULL );
	
	xTaskCreate(
	ADC_Cmd
	, (const signed portCHAR *)"ADC commande"
	, configMINIMAL_STACK_SIZE*3
	, NULL
	, tskIDLE_PRIORITY 
	, NULL );
	
	xTaskCreate(
	UART_SendSample
	, (const signed portCHAR *)"SendSample"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 3
	, NULL );
	
	xTaskCreate(
	AlarmMsgQ
	, (const signed portCHAR *)"Alarm"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 4
	, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */

	return 0;
}


void initialiseLCD(void) {
	static const gpio_map_t DIP204_SPI_GPIO_MAP = {
		{ DIP204_SPI_SCK_PIN,	DIP204_SPI_SCK_FUNCTION }, // SPI Clock.
		{ DIP204_SPI_MISO_PIN, DIP204_SPI_MISO_FUNCTION }, // MISO.
		{ DIP204_SPI_MOSI_PIN, DIP204_SPI_MOSI_FUNCTION }, // MOSI.
		{ DIP204_SPI_NPCS_PIN, DIP204_SPI_NPCS_FUNCTION } // Chip Select NPCS.
	};

	// Disable all interrupts.
	Disable_global_interrupt();

	// init the interrupts
	INTC_init_interrupts();

	// Enable all interrupts.
	Enable_global_interrupt();

	// add the spi options driver structure for the LCD DIP204
	spi_options_t spiOptions = {
		.reg = DIP204_SPI_NPCS,
		.baudrate = 1000000,
		.bits = 8,
		.spck_delay = 0,
		.trans_delay = 8, // <---- Very importent with the new compilor in atmel 6.x
		.stay_act = 1,
		.spi_mode = 0,
		.modfdis = 1
	};

	// Assign I/Os to SPI
	gpio_enable_module(DIP204_SPI_GPIO_MAP, sizeof(DIP204_SPI_GPIO_MAP)
	/ sizeof(DIP204_SPI_GPIO_MAP[0]));

	// Initialize as master
	spi_initMaster(DIP204_SPI, &spiOptions);

	// Set selection mode: variable_ps, pcs_decode, delay
	spi_selectionMode(DIP204_SPI, 0, 0, 0);

	// Enable SPI
	spi_enable(DIP204_SPI);

	// setup chip registers
	spi_setupChipReg(DIP204_SPI, &spiOptions, FOSC0);

	// initialize LCD
	dip204_init(backlight_PWM, true);

}

void init_usart(void){
	static const gpio_map_t USART_GPIO_MAP =
	{
		{AVR32_USART1_RXD_0_0_PIN, AVR32_USART1_RXD_0_0_FUNCTION},
		{AVR32_USART1_TXD_0_0_PIN, AVR32_USART1_TXD_0_0_FUNCTION}
	};
	// Assigner les pins du GPIO a etre utiliser par le USART1.
	gpio_enable_module(USART_GPIO_MAP,sizeof(USART_GPIO_MAP) / sizeof(USART_GPIO_MAP[0]));

	static const usart_options_t USART_OPTIONS =
	{
		.baudrate = 57600,
		.charlength = 8,
		.paritytype = USART_NO_PARITY,
		.stopbits = USART_1_STOPBIT,
		.channelmode = USART_NORMAL_CHMODE
	};

	// Initialise le USART1 en mode seriel RS232
	usart_init_rs232(&AVR32_USART1, &USART_OPTIONS, FOSC0);
}

static void LED_Flash(void *pvParameters){
	U8 aquis_start = 0;

	while (1)
	{
		// get command to start sampling
		xSemaphoreTake(FLASH_LED1_SEMAPHORE, portMAX_DELAY);
		aquis_start = AQUIS_START;
		xSemaphoreGive(FLASH_LED1_SEMAPHORE);
		gpio_tgl_gpio_pin(LED0_GPIO);
		
		if(aquis_start == 1){
			// if acquisition is started, flash led 2
			// always flash led 1
			gpio_tgl_gpio_pin(LED1_GPIO);
		}
		else {
			// if acquisition is stopped, stop led 2 flash
			// always flash led 1
			gpio_set_gpio_pin(LED1_GPIO);
			
		}

		vTaskDelay(200);
	}
}

static void UART_Cmd_RX(void *pvParameters) {
	
	char uart_received_command = 0;
	
	while(1){
		if (AVR32_USART1.csr & (AVR32_USART_CSR_RXRDY_MASK)){
			
			// get key command and store it
			xSemaphoreTake(UART_SEMAPHORE, portMAX_DELAY);
			uart_received_command = (AVR32_USART1.rhr & AVR32_USART_RHR_RXCHR_MASK);
			xSemaphoreGive(UART_SEMAPHORE);
				
			// if key command is 's'; start sampling
			if (uart_received_command == 's' || uart_received_command == 'S'){
				xSemaphoreTake(FLASH_LED1_SEMAPHORE, portMAX_DELAY);
				AQUIS_START = 1;
				xSemaphoreGive(FLASH_LED1_SEMAPHORE);
			}
			// if key command is 'x'; stop sampling
			else if (uart_received_command == 'x' || uart_received_command == 'X'){
				xSemaphoreTake(FLASH_LED1_SEMAPHORE, portMAX_DELAY);
				AQUIS_START = 0;
				xSemaphoreGive(FLASH_LED1_SEMAPHORE);
			}
		}
		
		vTaskDelay(200);
	}
	
}

static void ADC_Cmd(void *pvParameters) {
	//int i;
	
	U8 msgQ = 0;

	// GPIO pin/adc-function map.
	static const gpio_map_t ADC_GPIO_MAP = {
		{ ADC_LIGHT_PIN,			ADC_LIGHT_FUNCTION },
		{ ADC_POTENTIOMETER_PIN,	ADC_POTENTIOMETER_FUNCTION }
	};
	
	volatile avr32_adc_t *adc = &AVR32_ADC;

	// Assign the on-board sensors to their ADC channel.
	unsigned short adc_channel_pot = ADC_POTENTIOMETER_CHANNEL;
	unsigned short adc_channel_light = ADC_LIGHT_CHANNEL;

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADC_GPIO_MAP, 1);

	// configure ADC
	// Lower the ADC clock to match the ADC characteristics (because we configured
	// the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	// cf. the ADC Characteristic section in the datasheet).
	//AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(adc);

	// Enable the ADC channels.
	adc_enable(adc, adc_channel_pot);
	adc_enable(adc, adc_channel_light);


	while (1) {
			// Trigger the conversion
			adc_start(adc);

			// get value for the potentiometer adc channel					
			xSemaphoreTake(POTENTIOMETER_SEMAPHORE, portMAX_DELAY);
			if (adc_check_eoc(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL))
			{
				// conversion for potentiometer with relevant bits by applying mask	
				adc_value_pot = adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
				adc_value_pot = ((adc_value_pot & 0b1111111000));
			}
			xSemaphoreGive(POTENTIOMETER_SEMAPHORE);
		
			// get value for the light adc channel with relevant bits by applying mask		
			xSemaphoreTake(LIGHT_SEMAPHORE, portMAX_DELAY);
			if (adc_check_eoc(&AVR32_ADC, ADC_LIGHT_CHANNEL))
			{
				adc_value_light = adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL);
				adc_value_light = (((adc_value_light) & 0b1111111000)) | 1;
			}
			xSemaphoreGive(LIGHT_SEMAPHORE);
			
			// message queue equals 1 when we have data to show
			// but if it's already equal to 1 then we have an overflow
			xSemaphoreTake(MESSAGEQ_SEMAPHORE, portMAX_DELAY);
			msgQ = messageQueue;
			xSemaphoreGive(MESSAGEQ_SEMAPHORE);
			
			if(msgQ != 0){
				xSemaphoreTake(FLAGERROR_SEMAPHORE, portMAX_DELAY);
				flagError = 1;
				xSemaphoreGive(FLAGERROR_SEMAPHORE);
			}
			U8 aqui = 0;
			
			xSemaphoreTake(FLASH_LED1_SEMAPHORE, portMAX_DELAY);
			aqui = AQUIS_START;
			xSemaphoreGive(FLASH_LED1_SEMAPHORE);
			
			if(aqui == 1){
				xSemaphoreTake(MESSAGEQ_SEMAPHORE, portMAX_DELAY);
				messageQueue = 1;
				xSemaphoreGive(MESSAGEQ_SEMAPHORE);
				
			}
			
		vTaskDelay(1);
	}
}


static void AlarmMsgQ(void *pvParameters){
	U8 msgQFlag = 0;
	U16 ctr = 0;
	char str[8];
	
	while (1) {
		
		xSemaphoreTake(FLAGERROR_SEMAPHORE, portMAX_DELAY);
		msgQFlag = flagError;
		xSemaphoreGive(FLAGERROR_SEMAPHORE);

		// when flag error is equal to 1, overflow occurred
		// flash led 3
		if(msgQFlag == 1)
			gpio_clr_gpio_pin(LED2_GPIO);
			
			
		xSemaphoreTake(SAMPLECTR_SEMAPHORE, portMAX_DELAY);
		ctr = sampleCtr;
		sprintf(str, "%d", ctr);
		sampleCtr = 0;
		xSemaphoreGive(SAMPLECTR_SEMAPHORE);
		
		dip204_set_cursor_position(1, 1);
		dip204_write_string(str);
		
		
		vTaskDelay(1);
	}
}


static void UART_SendSample(void *pvParameters){
	U16 adc_pot = 0;
	U16 adc_lig = 0;

	while (1)
	{
		// get adc potentiometer and light values and apply TXCHR mask & offset
		
		xSemaphoreTake(POTENTIOMETER_SEMAPHORE, portMAX_DELAY);
		adc_pot = (adc_value_pot >> 2);
		xSemaphoreGive(POTENTIOMETER_SEMAPHORE);

		AVR32_USART1.thr = (adc_pot << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
		
		xSemaphoreTake(LIGHT_SEMAPHORE, portMAX_DELAY);
		adc_lig = (adc_value_light >> 2);
		xSemaphoreGive(LIGHT_SEMAPHORE);
		
		AVR32_USART1.thr = (adc_lig << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK | 1;
		
		// message queue equals 0 when we have shown the data; we "empty" the message queue 
		xSemaphoreTake(MESSAGEQ_SEMAPHORE, portMAX_DELAY);
		messageQueue = 0;
		xSemaphoreGive(MESSAGEQ_SEMAPHORE);
		
		xSemaphoreTake(SAMPLECTR_SEMAPHORE, portMAX_DELAY);
		sampleCtr++;
		xSemaphoreGive(SAMPLECTR_SEMAPHORE);
		
		//Change this value to 10 to force a overflow 
		vTaskDelay(1);
	}
}



