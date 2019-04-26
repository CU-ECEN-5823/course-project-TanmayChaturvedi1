/*
 * nonblock_timerWaitUs.h
 *
 *  Created on: 25 Apr 2019
 *      Author: TanmayC
 */

#ifndef SRC_NONBLOCK_TIMERWAITUS_H_
#define SRC_NONBLOCK_TIMERWAITUS_H_

#include "stdint.h"

/**
 * Create delay based on LETIMER0/
 * Prescaler set to 2 to achieve 4 seconds max delay
 * @param time delay in micro-second
 *	eg 80000Us for 80ms
 */
void nonblock_timerWaitUs(uint32_t us_wait);

#endif /* SRC_NONBLOCK_TIMERWAITUS_H_ */
