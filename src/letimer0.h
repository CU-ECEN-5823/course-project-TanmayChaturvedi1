/*
 * letimer0.h
 *
 *  Created on: 25 Apr 2019
 *      Author: TanmayC
 *
 *      This file contains function prototypes for functions used to setup LETIMER0
 */

#ifndef SRC_LETIMER0_H_
#define SRC_LETIMER0_H_

/*== USER DEFINED PARAMS ==*/
#define	PERIOD_MS	(1000)
#define	EnergyMode	(1)

int timer_uf_count;

/*== PROTOTYPES ==*/
void LETIMER0_En_CLK_Tree(void);
void LETIMER0_Config(void);
void LETIMER0_SetInterrupt(void);


#endif /* SRC_LETIMER0_H_ */
