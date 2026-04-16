/*
 * LED.h
 *
 *  Created on: Apr 16, 2026
 *      Author: emman
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>


typedef enum{
	LED_SLOW,
	LED_FAST,
	LED_OFF
}LedMode_t;


void LED_SetMode(LedMode_t mode);

void LED_Update(void);


#endif /* INC_LED_H_ */
