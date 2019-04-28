/*
 * nonblock_timerWaitUs.h
 *
 *  Created on: 25 Apr 2019
 *      Author: TanmayC
 */

#ifndef SRC_NONBLOCK_TIMERWAITUS_H_
#define SRC_NONBLOCK_TIMERWAITUS_H_

#include "stdint.h"


/* @brief	non block (interrupt based) micro-second delay generator
 *
 * Used LETIMER0 values to check current value and generate interrupt based on when the Comparison is met.
 * Timer flow condition is also taken care of in the else loop.
 *
 * @param	us_wait	time to wait in micro-second. Minimum value for us_wait is 61 based on timer & ticks resolution
 * @return	none
 */
void nonblock_timerWaitUs(uint32_t us_wait);

#endif /* SRC_NONBLOCK_TIMERWAITUS_H_ */
