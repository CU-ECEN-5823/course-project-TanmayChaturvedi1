/*
 * letimer0.c
 *
 *  Created on: 25 Apr 2019
 *      Author: TanmayC
 *
 *      This file contains configuration for LETIMER0
 */


#include "em_letimer.h"
#include "em_cmu.h"
#include "gpio.h"
#include "letimer0.h"
#include "event_scheduler.h"
#include "em_core.h"
#include "log.h"


/**
 * Initialize LETIMER0 Clock tree
 * Prescaler set to 2 to achieve 4 seconds max delay
 * @param null
 *
 */
void LETIMER0_En_CLK_Tree(void)
{
	CMU_OscillatorEnable( cmuOsc_ULFRCO, true, true );
	CMU_ClockSelectSet( cmuClock_LFA, cmuSelect_ULFRCO );
	CMU_ClockEnable( cmuClock_LETIMER0, true );
}


/**
* Set configuration parameters for LETIMER0
* After that, Initialize timer and set COMP0
* @param null
* below code structure sourced from: https://www.silabs.com/documents/public/example-code/an0026-efm32-letimer.zip
*
*/
void LETIMER0_Config(void)
{
	/* Set configurations for LETIMER 0 */
	LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
//	{
//			.enable         = true,                   /* Start counting when init completed. */
//			.comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
//			.bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
//			.out0Pol        = 0,                      /* Idle value for output 0. */
//			.out1Pol        = 0,                      /* Idle value for output 1. */
//			.ufoa0          = letimerUFOANone,       /* No output action */
//			.repMode        = letimerRepeatFree       /* Count until stopped */
//	};

	letimerInit.comp0Top = true;

	/* Initialize LETIMER */
	LETIMER_Init(LETIMER0, &letimerInit);

	/*Configure COMP0 based on energy mode*/
	/*Setting the Comp0Top to not exceed 4 seconds(ie 65535)*/
	/*Logic: Comp0Top = 65535 would mean a delay of 4 sec, so Comp0Top = {(3000/4000) * 65535} means delay of 3 second and so on */
	LETIMER_CompareSet( LETIMER0, 0, (uint32_t)(PERIOD_MS)); /*For periodic timer interrupt*/

}


/**
* Set interrupt for LETIMER0
* Enable Underflow Interrupt ONLY
*
*/
void LETIMER0_SetInterrupt(void)
{
	/*Enable Underflow interrupt*/
	LETIMER_IntEnable( LETIMER0, LETIMER_IEN_COMP1 | LETIMER_IEN_UF );
	/*Enable Nested Interrupts*/
	NVIC_EnableIRQ( LETIMER0_IRQn );
	/*Enable LETIMER0 Interrupt*/
	LETIMER_Enable( LETIMER0, true );
}


/**
* LETIMER0 Interrupt Handler
* After Underflow, scheduler checks if UF interrupt has occurred./
* If UF Interrupt, set global scheduler flag to 1.
* Else, clear all events by setting EVENT_STAT = 0./
* Critical Section Set up to protect other interrupts
*/
void LETIMER0_IRQHandler(void)
{
	//LOG_INFO("in Timer Interrupts IRQ\n");
	uint32_t interrupt = LETIMER_IntGet( LETIMER0 );
	LETIMER_IntClear( LETIMER0, interrupt );

	/* Critical Section Start */
	CORE_DECLARE_IRQ_STATE;
	CORE_ENTER_CRITICAL( );
	if( interrupt & LETIMER_IF_UF )
	{
/*
		event_name.EVENT_TIMER_UNDERFLOW_INT = true;
		event_name.EVENT_NONE = false;
		//ext_sig_event |= LETIMER_UF_EVENT;
		gecko_external_signal( event_name.EVENT_TIMER_UNDERFLOW_INT );
*/
		//gecko_external_signal(LETIMER_UF_EVENT);
		LOG_INFO("In UNDER fLOW ISR \n");
		//displayUpdate();
		timer_uf_count ++;
	}

	if( interrupt & LETIMER_IF_COMP1 )
	{
		LOG_INFO("In COMP1 ISR\n");
		event_name.EVENT_SETUP_TIMER_EXPIRED = true;
		event_name.EVENT_NONE = false;
		//ext_sig_event |= LETIMER_UF_EVENT;

		gecko_external_signal( event_name.EVENT_SETUP_TIMER_EXPIRED );
		LETIMER_IntDisable(LETIMER0, LETIMER_IFC_COMP1);
		/*TO ADD*/
	}
	/* Critical Section End */
	CORE_EXIT_CRITICAL( );
}
