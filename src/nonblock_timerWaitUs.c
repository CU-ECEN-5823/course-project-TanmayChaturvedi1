/*
 * nonblock_timerWaitUs.c
 *
 *  Created on: 25 Apr 2019
 *      Author: TanmayC
 *		I fix the initial value of clock tick and keep collecting other ticks value
 *		through LETIMER_CounterGet function. Since 1 tick based on this LETIMER0 Config
 *		equals around 61 micro-second, I check if the difference between initial and subsequent
 *		ticks is more than an integer value of input arg divide by 1 tick (in micro-second)
 */

#ifndef SRC_NONBLOCK_TIMERWAITUS_C_
#define SRC_NONBLOCK_TIMERWAITUS_C_



#include "nonblock_timerWaitUs.h"
#include "letimer0.h"
#include "em_letimer.h"
#include "log.h"

void nonblock_timerWaitUs(uint32_t us_wait)
{
	LETIMER_IntEnable( LETIMER0, LETIMER_IEN_COMP1  );
	if( us_wait >= 61 ) /* Error handling to make sure valid input arg given */
	{
		uint32_t initialtick = LETIMER_CounterGet( LETIMER0 );

		if ( initialtick > (us_wait/1000) )
			LETIMER_CompareSet( LETIMER0, 1, initialtick - (us_wait /1000) );
		else if ( initialtick <= us_wait/1000 )
			LETIMER_CompareSet( LETIMER0, 1, PERIOD_MS + initialtick - (us_wait /1000) );
	}

}



#endif /* SRC_NONBLOCK_TIMERWAITUS_C_ */
