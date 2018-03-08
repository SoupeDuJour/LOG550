/*****************************************************************************
* Auteur : Maxime Turenne
* Copyright : Maxime Turenne
* Description: Demo d'utilisation de FreeRTOS dans le cadre d'un thermostate numérique.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Environment header files. */
#include "power_clocks_lib.h"

#include "board.h"
#include "compiler.h"
#include "dip204.h"
#include "intc.h"
#include "gpio.h"
#include "pm.h"
#include "delay.h"
#include "spi.h"
#include "conf_clock.h"
#include "adc.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// tasks
static void vOutputLCD(void *pvParameters);
static void vCalculatePower(void *pvParameters);
static void vReadingADC(void *pvParameters);

// initialization functions
void initialiseLCD(void);

// global var
volatile int POWER = 0;
volatile int TEMPERATURE_DESIRED = 0;
volatile int TEMPERATURE_ROOM = 0;

// semaphore
static xSemaphoreHandle POWER_SEMAPHORE = NULL;
static xSemaphoreHandle TEMPERATURE_DESIRED_SEMAPHORE = NULL;
static xSemaphoreHandle TEMPERATURE_ROOM_SEMAPHORE = NULL;

static const unsigned short temperature_code[] = { 0x3B4, 0x3B0, 0x3AB, 0x3A6,
	0x3A0, 0x39A, 0x394, 0x38E, 0x388, 0x381, 0x37A, 0x373, 0x36B, 0x363,
	0x35B, 0x353, 0x34A, 0x341, 0x338, 0x32F, 0x325, 0x31B, 0x311, 0x307,
	0x2FC, 0x2F1, 0x2E6, 0x2DB, 0x2D0, 0x2C4, 0x2B8, 0x2AC, 0x2A0, 0x294,
	0x288, 0x27C, 0x26F, 0x263, 0x256, 0x24A, 0x23D, 0x231, 0x225, 0x218,
	0x20C, 0x200, 0x1F3, 0x1E7, 0x1DB, 0x1CF, 0x1C4, 0x1B8, 0x1AC, 0x1A1,
	0x196, 0x18B, 0x180, 0x176, 0x16B, 0x161, 0x157, 0x14D, 0x144, 0x13A,
	0x131, 0x128, 0x11F, 0x117, 0x10F, 0x106, 0xFE, 0xF7, 0xEF, 0xE8, 0xE1,
	0xDA, 0xD3, 0xCD, 0xC7, 0xC0, 0xBA, 0xB5, 0xAF, 0xAA, 0xA4, 0x9F, 0x9A,
	0x96, 0x91, 0x8C, 0x88, 0x84, 0x80, 0x7C, 0x78, 0x74, 0x71, 0x6D, 0x6A,
	0x67, 0x64, 0x61, 0x5E, 0x5B, 0x58, 0x55, 0x53, 0x50, 0x4E, 0x4C, 0x49,
0x47, 0x45, 0x43, 0x41, 0x3F, 0x3D, 0x3C, 0x3A, 0x38 };

int main(void) {
	// Configure Osc0 in crystal mode (i.e. use of an external crystal source, with
	// frequency FOSC0) with an appropriate startup time then switch the main clock
	// source to Osc0.
	pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

	/* Setup the LED's all off */
	LED_Display(0);

	initialiseLCD();

	POWER_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	TEMPERATURE_DESIRED_SEMAPHORE = xSemaphoreCreateCounting(1,1);
	TEMPERATURE_ROOM_SEMAPHORE = xSemaphoreCreateCounting(1,1);

	/* Start the demo tasks defined within this file. */
	xTaskCreate(
	vOutputLCD
	, (const signed portCHAR *)"Output"
	, configMINIMAL_STACK_SIZE*3
	, NULL
	, tskIDLE_PRIORITY
	, NULL );
	xTaskCreate(
	vCalculatePower
	, (const signed portCHAR *)"Calcul"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 2
	, NULL );
	xTaskCreate(
	vReadingADC
	, (const signed portCHAR *)"Reading"
	, configMINIMAL_STACK_SIZE
	, NULL
	, tskIDLE_PRIORITY + 1
	, NULL );

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */

	return 0;
}

static void vReadingADC(void *pvParameters) {

	int i;

	// GPIO pin/adc-function map.
	static const gpio_map_t ADC_GPIO_MAP = { { ADC_TEMPERATURE_PIN,
		ADC_TEMPERATURE_FUNCTION }, { ADC_POTENTIOMETER_PIN,
	ADC_POTENTIOMETER_FUNCTION } };

	volatile avr32_adc_t *adc = &AVR32_ADC; // ADC IP registers address

	unsigned long adc_value_pot = 0;
	unsigned long adc_value_temp = 0;
	int theTemp = 0;
	
	// Assign the on-board sensors to their ADC channel.
	unsigned short adc_channel_pot = ADC_POTENTIOMETER_CHANNEL;
	unsigned short adc_channel_temp = ADC_TEMPERATURE_CHANNEL;

	// Assign and enable GPIO pins to the ADC function.
	gpio_enable_module(ADC_GPIO_MAP, 1);

	// configure ADC
	// Lower the ADC clock to match the ADC characteristics (because we configured
	// the CPU clock to 12MHz, and the ADC clock characteristics are usually lower;
	// cf. the ADC Characteristic section in the datasheet).
	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	adc_configure(adc);

	// Enable the ADC channels.
	adc_enable(adc, adc_channel_pot);
	adc_enable(adc, adc_channel_temp);

	while (1) {

		// Trigger the conversion
		adc_start(adc);

		// get value for the potentiometer adc channel
		adc_value_pot = adc_get_value(adc, adc_channel_pot);
		adc_value_temp = adc_get_value(adc, adc_channel_temp);

		xSemaphoreTake(TEMPERATURE_DESIRED_SEMAPHORE, portMAX_DELAY);

		TEMPERATURE_DESIRED = (int) (((double)adc_value_pot / 1023) * (25)) + 5;
		//TEMPERATURE_DESIRED = adc_value_pot;

		xSemaphoreGive(TEMPERATURE_DESIRED_SEMAPHORE);

		i = 0;
		if (adc_value_temp > temperature_code[0]) {
			theTemp = -20;
			} else {
			while (temperature_code[i++] > adc_value_temp)
			;
			theTemp = (i - 1 - 20);
		}

		xSemaphoreTake(TEMPERATURE_ROOM_SEMAPHORE, portMAX_DELAY);

		TEMPERATURE_ROOM = theTemp;

		xSemaphoreGive(TEMPERATURE_ROOM_SEMAPHORE);

		vTaskDelay(250);
	}
}

static void vCalculatePower(void *pvParameters) {

	int power = 0;
	int tempDesired = 0;
	int tempRoom = 0;
	int tempValue = 0;

	while (1) {

		// Get the data
		xSemaphoreTake(TEMPERATURE_DESIRED_SEMAPHORE, portMAX_DELAY);
		tempDesired = TEMPERATURE_DESIRED;
		xSemaphoreGive(TEMPERATURE_DESIRED_SEMAPHORE);

		xSemaphoreTake(TEMPERATURE_ROOM_SEMAPHORE, portMAX_DELAY);
		tempRoom = TEMPERATURE_ROOM;
		xSemaphoreGive(TEMPERATURE_ROOM_SEMAPHORE);

		// calculate the power base on the room temperature and the desired temperature.
		if (tempDesired <= tempRoom)
		{
			power = 0;
		}
		else
		{
			//tempDesired > tempRoom
			tempValue= tempDesired - tempRoom;
			if (tempValue > 6)
			{
				power = 100;
			}
			else
			{
				power = (int)(((double)tempValue / 6) * 100);
			}
		}

		xSemaphoreTake(POWER_SEMAPHORE, portMAX_DELAY);

		POWER = power;

		xSemaphoreGive(POWER_SEMAPHORE);

		vTaskDelay(1000);
	}
}

static void vOutputLCD(void *pvParameters) {

	const char *tempDesiredTitle = "Temp. target: 0"; // 15
	const char *puissanceTitle = "Power: 0"; // 8
	const char *tempTitle = "Temp. room: 0"; // 13
	char str[8];
	bool increase = true;
	bool updateLights = true;
	int power = 0;
	unsigned char targetLeds = 0;

	dip204_set_cursor_position(1, 1);

	dip204_write_string(tempDesiredTitle);

	dip204_set_cursor_position(1, 2);

	dip204_write_string(tempTitle);

	dip204_set_cursor_position(1, 3);

	dip204_write_string(puissanceTitle);

	dip204_hide_cursor();

	int leds = 0;
	while (1) {

		// write throttle
		dip204_set_cursor_position(15, 1);
		dip204_write_string("    ");
		dip204_set_cursor_position(15, 1);

		xSemaphoreTake(TEMPERATURE_DESIRED_SEMAPHORE, portMAX_DELAY);
		sprintf(str, "%d C", TEMPERATURE_DESIRED);
		xSemaphoreGive(TEMPERATURE_DESIRED_SEMAPHORE);

		dip204_write_string(str);

		// write the temp
		dip204_set_cursor_position(13, 2);
		dip204_write_string("    ");
		dip204_set_cursor_position(13, 2);

		xSemaphoreTake(TEMPERATURE_ROOM_SEMAPHORE, portMAX_DELAY);
		sprintf(str, "%d C", TEMPERATURE_ROOM);
		xSemaphoreGive(TEMPERATURE_ROOM_SEMAPHORE);

		dip204_write_string(str);

		// write the TANK_LEVEL
		dip204_set_cursor_position(8, 3);
		dip204_write_string("    ");
		dip204_set_cursor_position(8, 3);

		xSemaphoreTake(POWER_SEMAPHORE, portMAX_DELAY);
		power = POWER;
		xSemaphoreGive(POWER_SEMAPHORE);
		sprintf(str, "%d%%", power);

		dip204_write_string(str);

		targetLeds = power / 16;

		if (updateLights) {

			switch (leds) {
				case 0:
				gpio_set_gpio_pin(LED0_GPIO); // close
				gpio_set_gpio_pin(LED1_GPIO); // close
				gpio_set_gpio_pin(LED2_GPIO); // close
				gpio_set_gpio_pin(LED3_GPIO); // close
				gpio_set_gpio_pin(LED4_GPIO); // close
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 1:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_set_gpio_pin(LED1_GPIO); // close
				gpio_set_gpio_pin(LED2_GPIO); // close
				gpio_set_gpio_pin(LED3_GPIO); // close
				gpio_set_gpio_pin(LED4_GPIO); // close
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 2:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_clr_gpio_pin(LED1_GPIO); // open
				gpio_set_gpio_pin(LED2_GPIO); // close
				gpio_set_gpio_pin(LED3_GPIO); // close
				gpio_set_gpio_pin(LED4_GPIO); // close
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 3:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_clr_gpio_pin(LED1_GPIO); // open
				gpio_clr_gpio_pin(LED2_GPIO); // open
				gpio_set_gpio_pin(LED3_GPIO); // close
				gpio_set_gpio_pin(LED4_GPIO); // close
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 4:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_clr_gpio_pin(LED1_GPIO); // open
				gpio_clr_gpio_pin(LED2_GPIO); // open
				gpio_clr_gpio_pin(LED3_GPIO); // open
				gpio_set_gpio_pin(LED4_GPIO); // close
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 5:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_clr_gpio_pin(LED1_GPIO); // open
				gpio_clr_gpio_pin(LED2_GPIO); // open
				gpio_clr_gpio_pin(LED3_GPIO); // open
				gpio_clr_gpio_pin(LED4_GPIO); // open
				gpio_set_gpio_pin(LED6_GPIO); // close
				break;
				case 6:
				gpio_clr_gpio_pin(LED0_GPIO); // open
				gpio_clr_gpio_pin(LED1_GPIO); // open
				gpio_clr_gpio_pin(LED2_GPIO); // open
				gpio_clr_gpio_pin(LED3_GPIO); // open
				gpio_clr_gpio_pin(LED4_GPIO); // open
				gpio_clr_gpio_pin(LED6_GPIO); // open
				break;
			}
		}

		if (leds == targetLeds) {
			increase = false;
			updateLights = false;
			} else {
			updateLights = true;
			if (leds > targetLeds) {
				increase = false;
				} else if (leds < targetLeds) {
				increase = true;
			}
		}

		if (updateLights) {
			if (increase) {
				leds++;
				} else {
				leds--;
			}
		}

		vTaskDelay(100);
	}
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
