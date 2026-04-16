/*
 * control.c
 *
 *  Created on: Apr 15, 2026
 *      Author: emman
 */

#include "main.h"
#include "control.h"
#include "math.h"
#include "led.h"

//volatile uint8_t blink_fast = 0;

void Control_Update(SensorData_t *data){
	 if(data->temperature > 33){
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			  }

			  else{
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

			  }

			  if(fabs(data->gx) > 100 || fabs(data->gy) > 100 ){
				  LED_SetMode(LED_FAST);
			  }

			  else {
				  LED_SetMode(LED_SLOW);

			  }
		  }



