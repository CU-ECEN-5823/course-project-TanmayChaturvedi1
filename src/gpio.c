/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "hal-config.h"
#include "gpiointerrupt.h"
#include "em_core.h"
#include "native_gecko.h"
#include "src/log.h"
#include "gecko_main.h"

#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5
#define	LUX_Power_Port	gpioPortD
#define	LUX_Power_Pin	11


void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);
	GPIO_PinModeSet(gpioPortF,6,gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortF,7,gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortD,12,gpioModeInput, 1);							//for MQ2 sensor
	GPIO_PinModeSet(gpioPortC,10,gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortC,11,gpioModePushPull, 0);
	GPIO_PinModeSet(LUX_Power_Port,LUX_Power_Pin,gpioModePushPull, 0);		//for APDS 9301 lux sensor



}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}


/**
 * Enables Lux Power Pin for load power management
 * @param null
 *
 */
void load_power_on(void)
{
	LOG_INFO("Sensor VCC Pin Switched On");
	GPIO_PinOutSet(LUX_Power_Port,LUX_Power_Pin);
}

/**
 * Disables Lux Power Pin for load power management
 * @param null
 *
 */
void load_power_off(void)
{
	GPIO_PinOutClear(LUX_Power_Port,LUX_Power_Pin);
	LOG_INFO("Sensor VCC Pin Switched Off");
}

/**
 * Enables Even Interrupt for PD12 Gpio pin
 * Generates falling/rising edge-based interrupt when fire/no fire is detected.
 * @param null
 *
 */
void gpio_set_interrupt(void)
{
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0, both falling and rising edges */
	GPIO_ExtIntConfig(gpioPortD, (12U), (12U), true, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister((12U), gpioint);
}


/***************************************************************************//**
 * Sourced from SI Labs Mesh Switch Example. This is like GPIO IRQ Handler
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the GPIO pin occurs. Pin number is passed as parameter.
 *
 * @param[in] pin  Pin number where interrupt occurs
 *
 * @note This function is called from ISR context and therefore it is
 *       not possible to call any BGAPI functions directly. The button state
 *       change is signaled to the application using gecko_external_signal()
 *       that will generate an event gecko_evt_system_external_signal_id
 *       which is then handled in the main loop.
 ******************************************************************************/
void gpioint(uint8_t pin)
{
	uint8_t val = GPIO_PinInGet(gpioPortD, pin);
	if (val == 1)
	{
		publish_data(mesh_generic_request_on_off, 0,MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID );
		LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Interrupt Received - No Fire");
	}
	else
	{
		publish_data(mesh_generic_request_on_off, 1,MESH_GENERIC_ON_OFF_CLIENT_MODEL_ID );
		LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Interrupt Received - Fire");
	}
}

void gpioEnableDisplay()
{
	GPIO_PinOutSet(LCD_PORT_DISP_SEL, LCD_PIN_DISP_SEL);
}

void gpioSetDisplayExtcomin(bool high)
{
	if (high == true)
	{
		GPIO_PinOutSet(LCD_PORT_EXTCOMIN, LCD_PIN_EXTCOMIN);
	}
	else
		GPIO_PinOutClear(LCD_PORT_EXTCOMIN, LCD_PIN_EXTCOMIN);
}
