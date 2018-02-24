/*
LOG550 - Lab01 

Using Atmel AVR32 EVK110 Board

Using modules:
		LCD Display - DIP204-4ORT01 (component)
		ADC - Analog to Digital Converter (driver)
		GPIO - General-Purpose Input/Output (driver)
		INTC - Interrupt Controller (driver)
		PM - Power Manager (driver)
		TC - Timer/Counter (driver)
		USART - Universal Synchronous/Asynchronous Receiver/Transmitter (driver)
		Generic board support (driver)


Run with Oscilloscope64bit.jar with the following settings:
	Port :			[Which ever usb port the board is connected to]
	Baud Rate :		57600
	Data :			8
	Parity :		None
	Stop:			1
	Flow control :	None
	
	Using UART_0

*/

#include "gpio.h"
#include "pm.h"
#include "evk1100.h"
#include "intc.h"
#include "tc.h"
#include "usart.h"
#include <stdint.h>
#include "adc.h"


void adc_init(void);

void frequency_button_init(void);

void led_timer_init(void);
void led_timer_toggle(void);
void led_timer_handler(void);
void led_gpio_assign_pin(uint32_t pin, uint8_t value);

void sampling_timer_init(void);
void sampling_timer_set_frequency(uint16_t freq);
void sampling_timer_start(void);
void sampling_timer_stop(void);
void sampling_timer_start_sampling(void);

void uart_init(void);
void uart_send_byte(uint8_t byte);
void uart_handler(void);

/*//////////////////////////////////////////////////////////
						VARIABLES 
*///////////////////////////////////////////////////////////

#define ADC_POTENTIOMETER_CHANNEL   1
#define ADC_POTENTIOMETER_PIN       AVR32_ADC_AD_1_PIN
#define ADC_POTENTIOMETER_FUNCTION  AVR32_ADC_AD_1_FUNCTION

#define ADC_LIGHT_CHANNEL           2
#define ADC_LIGHT_PIN               AVR32_ADC_AD_2_PIN
#define ADC_LIGHT_FUNCTION          AVR32_ADC_AD_2_FUNCTION

#define LED_TIMER_CHANNEL 1
#define LED_TIMER_IRQ AVR32_TC_IRQ1
#define LED_TIMER_TICKS_PER_SECOND (FOSC0 / 128)

#define SAMPLING_TIMER_CHANNEL 0
#define SAMPLING_TIMER_IRQ AVR32_TC_IRQ0
#define SAMPLING_TIMER_TICKS_PER_SECOND (FOSC0 / 8)

#define UART_RX_PIN			AVR32_USART0_RXD_0_0_PIN
#define UART_RX_FUNCTION	AVR32_USART0_RXD_0_0_FUNCTION
#define UART_TX_PIN			AVR32_USART0_TXD_0_0_PIN
#define UART_TX_FUNCTION	AVR32_USART0_TXD_0_0_FUNCTION


volatile uint8_t adc_conversion_indices[2] = {};
volatile unsigned long adc_conversion_values[2] = {};
	
uint8_t led_timer_flash_led2 = 0;
volatile uint8_t led_timer_should_toggle_leds = 0;
static const tc_interrupt_t led_timer_interrupts = { .cpcs = 1 };
static uint8_t led_state;	

uint8_t sampling_timer_is_running = 0;
uint16_t sampling_timer_current_frequency = 0;
volatile uint16_t sampling_timer_desired_frequency = 0;
volatile uint8_t sampling_timer_is_sampling_ready = 0;

static const tc_interrupt_t sampling_timer_interrupts = { .cpcs = 1 };
	
volatile char uart_received_command = 0;
volatile uint8_t uart_transmission_completed = 0;

static const gpio_map_t usart_gpio_map = {
	{ UART_RX_PIN, UART_RX_FUNCTION },
	{ UART_TX_PIN, UART_TX_FUNCTION },
};

static const usart_options_t options = {
	.baudrate = 57600,
	.charlength = 8,
	.paritytype = USART_NO_PARITY,
	.stopbits = USART_1_STOPBIT,
	.channelmode = USART_NORMAL_CHMODE
};


static tc_waveform_opt_t led_timer_waveform = {
	.channel  = LED_TIMER_CHANNEL,

	.bswtrg   = TC_EVT_EFFECT_NOOP,
	.beevt    = TC_EVT_EFFECT_NOOP,
	.bcpc     = TC_EVT_EFFECT_NOOP,
	.bcpb     = TC_EVT_EFFECT_NOOP,

	.aswtrg   = TC_EVT_EFFECT_NOOP,
	.aeevt    = TC_EVT_EFFECT_NOOP,
	.acpc     = TC_EVT_EFFECT_NOOP,
	.acpa     = TC_EVT_EFFECT_NOOP,
	.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
	.enetrg   = 0,
	.eevt     = 0,
	.eevtedg  = TC_SEL_NO_EDGE,
	.cpcdis   = 0,
	.cpcstop  = 0,

	.burst    = 0,
	.clki     = 0,
	.tcclks   = TC_CLOCK_SOURCE_TC4
};

static tc_waveform_opt_t sampling_timer_waveform = {
	.channel  = SAMPLING_TIMER_CHANNEL,

	.bswtrg   = TC_EVT_EFFECT_NOOP,
	.beevt    = TC_EVT_EFFECT_NOOP,
	.bcpc     = TC_EVT_EFFECT_NOOP,
	.bcpb     = TC_EVT_EFFECT_NOOP,

	.aswtrg   = TC_EVT_EFFECT_NOOP,
	.aeevt    = TC_EVT_EFFECT_NOOP,
	.acpc     = TC_EVT_EFFECT_NOOP,
	.acpa     = TC_EVT_EFFECT_NOOP,
	.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
	.enetrg   = 0,
	.eevt     = 0,
	.eevtedg  = TC_SEL_NO_EDGE,
	.cpcdis   = 0,
	.cpcstop  = 0,

	.burst    = 0,
	.clki     = 0,
	.tcclks   = TC_CLOCK_SOURCE_TC3
};
/*//////////////////////////////////////////////////////////
						INTURUPTS 
*///////////////////////////////////////////////////////////

__attribute__((__interrupt__))
static void adc_conversion_handler(void)
{
	if (adc_check_eoc(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL))
	{
		// conversion for potentiometer 
		adc_conversion_indices[0]++;
		adc_conversion_values[0] = adc_get_value(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
	}
	
	if (adc_check_eoc(&AVR32_ADC, ADC_LIGHT_CHANNEL))
	{
		// conversion for light sensor
		adc_conversion_indices[1]++;
		adc_conversion_values[1] = adc_get_value(&AVR32_ADC, ADC_LIGHT_CHANNEL);
	}
	
	(void)AVR32_ADC.lcdr;
}

__attribute__((__interrupt__))
void led_timer_handler(void)
{
	static uint8_t ticks = 0;
	
	tc_read_sr(&AVR32_TC, LED_TIMER_CHANNEL);
	
	ticks++;
	if (ticks == 10)
	{
		ticks = 0;
		led_timer_should_toggle_leds = 1;
	}
}

__attribute__((__interrupt__))
static void frequency_button_pushed_irq_handler(void)
{
	switch (sampling_timer_current_frequency)
	{
		// first frequency
		case 1000:
		sampling_timer_desired_frequency = 2000;
		break;
		
		// second frequency
		case 2000:
		sampling_timer_desired_frequency = 1000;
		break;
	}
	
	//clear flag
	gpio_clear_pin_interrupt_flag(GPIO_PUSH_BUTTON_0);
}

__attribute__((__interrupt__))
static void sampling_timer_irq_handler(void)
{
	tc_read_sr(&AVR32_TC, SAMPLING_TIMER_CHANNEL);
	sampling_timer_is_sampling_ready = 1;
}

__attribute__((__interrupt__))
void uart_handler(void)
{
	if (usart_test_hit(&AVR32_USART0))
	{
		//get command
		uart_received_command = (AVR32_USART0.rhr & AVR32_USART_RHR_RXCHR_MASK) >> AVR32_USART_RHR_RXCHR_OFFSET;
	}
	else if (usart_tx_empty(&AVR32_USART0))
	{
		uart_transmission_completed = 1;
		AVR32_USART0.idr = AVR32_USART_IER_TXRDY_MASK;
	}
	else{
		//something went wrong ...? flash led4
		gpio_set_gpio_pin(LED4_GPIO);	
	}
	
}
/*//////////////////////////////////////////////////////////
						FUNCTIONS 
*///////////////////////////////////////////////////////////

void led_gpio_assign_pin(uint32_t pin, uint8_t value)
{
	volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[pin >> 5];
	
	if (value)
	{
		gpio_port->ovrs = 1 << (pin & 0x1F);
	}
	else
	{
		gpio_port->ovrc = 1 << (pin & 0x1F);
	}
	
	gpio_port->oders = 1 << (pin & 0x1F);
	gpio_port->gpers = 1 << (pin & 0x1F);
}

void led_timer_toggle(void)
{
	led_timer_should_toggle_leds = 0;
	led_state ^= 1;
	
	led_gpio_assign_pin(LED0_GPIO, led_state);
	//if the led2 needs to be flashing...
	if (led_timer_flash_led2)
	{
		led_gpio_assign_pin(LED1_GPIO, led_state);
	}
}

void sampling_timer_set_frequency(uint16_t freq)
{
	tc_write_rc(&AVR32_TC, SAMPLING_TIMER_CHANNEL, SAMPLING_TIMER_TICKS_PER_SECOND / freq);
	sampling_timer_desired_frequency = freq;
	sampling_timer_current_frequency = freq;
}

void sampling_timer_start(void)
{
	tc_start(&AVR32_TC, sampling_timer_waveform.channel);
	sampling_timer_is_running = 1;
	led_timer_flash_led2 = 1;
}

void sampling_timer_stop(void)
{
	tc_stop(&AVR32_TC, sampling_timer_waveform.channel);
	sampling_timer_is_running = 0;
	led_timer_flash_led2 = 0;
	gpio_set_gpio_pin(LED1_GPIO);
}

void sampling_timer_start_sampling()
{
	adc_start(&AVR32_ADC);
	sampling_timer_is_sampling_ready = 0;
}

void uart_send_byte(uint8_t byte)
{
	uart_transmission_completed = 0;
	AVR32_USART0.thr = (byte << AVR32_USART_THR_TXCHR_OFFSET) & AVR32_USART_THR_TXCHR_MASK;
	AVR32_USART0.ier = AVR32_USART_IER_TXRDY_MASK;
}

/*//////////////////////////////////////////////////////////
						INITS
*///////////////////////////////////////////////////////////

void adc_init(void)
{
	static const gpio_map_t ADC_GPIO_MAP =
	{
		{ ADC_LIGHT_PIN, ADC_LIGHT_FUNCTION },
		{ ADC_POTENTIOMETER_PIN, ADC_POTENTIOMETER_FUNCTION }
	};

	gpio_enable_module(ADC_GPIO_MAP, sizeof(ADC_GPIO_MAP) / sizeof(ADC_GPIO_MAP[0]));

	AVR32_ADC.mr |= 0x1 << AVR32_ADC_MR_PRESCAL_OFFSET;
	AVR32_ADC.ier = AVR32_ADC_DRDY_MASK;
	adc_configure(&AVR32_ADC);

	adc_enable(&AVR32_ADC, ADC_LIGHT_CHANNEL);
	adc_enable(&AVR32_ADC, ADC_POTENTIOMETER_CHANNEL);
	INTC_register_interrupt(&adc_conversion_handler, AVR32_ADC_IRQ, AVR32_INTC_INT3);
}

void led_timer_init(void)
{
	tc_init_waveform(&AVR32_TC, &led_timer_waveform);
	tc_write_rc(&AVR32_TC, LED_TIMER_CHANNEL, LED_TIMER_TICKS_PER_SECOND / 50);
	tc_configure_interrupts(&AVR32_TC, LED_TIMER_CHANNEL, &led_timer_interrupts);
	INTC_register_interrupt(&led_timer_handler, LED_TIMER_IRQ, AVR32_INTC_INT0);
	tc_start(&AVR32_TC, led_timer_waveform.channel);
}

void frequency_button_init(void)
{
	gpio_enable_pin_interrupt(GPIO_PUSH_BUTTON_0, GPIO_RISING_EDGE);
	gpio_enable_pin_glitch_filter(GPIO_PUSH_BUTTON_0);
	
	unsigned irq = AVR32_GPIO_IRQ_0 + GPIO_PUSH_BUTTON_0 / 8;
	INTC_register_interrupt(&frequency_button_pushed_irq_handler, irq, AVR32_INTC_INT0);
}

void sampling_timer_init()
{
	tc_init_waveform(&AVR32_TC, &sampling_timer_waveform);
	INTC_register_interrupt(&sampling_timer_irq_handler, SAMPLING_TIMER_IRQ, AVR32_INTC_INT2);
	sampling_timer_set_frequency(1000);
	
	tc_configure_interrupts(&AVR32_TC, SAMPLING_TIMER_CHANNEL, &sampling_timer_interrupts);
}

void uart_init(void)
{
	uart_transmission_completed = 1;
	gpio_enable_module(usart_gpio_map, sizeof(usart_gpio_map) / sizeof(usart_gpio_map[0]));
	usart_init_rs232(&AVR32_USART0, &options, FOSC0);
	AVR32_USART0.ier = AVR32_USART_IER_RXRDY_MASK;
	INTC_register_interrupt(&uart_handler, AVR32_USART0_IRQ, AVR32_INTC_INT0);
}


/*//////////////////////////////////////////////////////////
						MAIN 
*///////////////////////////////////////////////////////////

int main(void)
{
	pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);
	
	Disable_global_interrupt();
	
	INTC_init_interrupts();
	
	//init all	
	adc_init();
	sampling_timer_init();
	led_timer_init();
	frequency_button_init();
	uart_init();
	
	Enable_global_interrupt();

	uint8_t last_conversions[2] = {0, 0};
	while (1)
	{
		if (uart_received_command != 0)
		{
			if (uart_received_command == 's')
			{
				sampling_timer_start();
			}
			else if (uart_received_command == 'x')
			{
				sampling_timer_stop();
			}
			uart_received_command = 0;
		}
		
		if (sampling_timer_desired_frequency != sampling_timer_current_frequency)
		{
			sampling_timer_set_frequency(sampling_timer_desired_frequency);
		}
		
		if (led_timer_should_toggle_leds)
		{
			led_timer_toggle();
		}
		
		if (sampling_timer_is_sampling_ready)
		{
			sampling_timer_start_sampling();
		}
		
		//to check if our adc is late
		for (int i = 0; i < 2; i++)
		{
			uint8_t index = adc_conversion_indices[i];
			if (last_conversions[i] != index)
			{
				if ((uint8_t)(last_conversions[i] + 1) != index)
				{
					// something went wrong light up led3
					gpio_set_gpio_pin(LED3_GPIO);
				}
				
				if (uart_transmission_completed)
				{
					last_conversions[i] = index;
					uint8_t byte = ((adc_conversion_values[i] & 0b1111111000) >> 2) | i;
					uart_send_byte(byte);
				}
				
			}
		}
	}
}