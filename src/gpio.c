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

#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

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


void gpio_set_interrupt(void)
{
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);

	/* configure interrupt for PB0 and PB1, both falling and rising edges */
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);

	/* register the callback function that is invoked when interrupt occurs */
	GPIOINT_CallbackRegister(BSP_BUTTON0_PIN, gpioint);

}


/***************************************************************************//**
 * Sourced from SI Labs Mesh Switch Example. This is like GPIO IRQ Handler
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the pushbutton inputs occurs. Pin number is passed as parameter.
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
	//CORE_DECLARE_IRQ_STATE;
	if (pin == BSP_BUTTON0_PIN)
	{
		ext_sig_event |= PB0_STATE;
		gecko_external_signal(ext_sig_event);
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
