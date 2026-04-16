/*
 * control.h
 *
 *  Created on: Apr 15, 2026
 *      Author: emman
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "sensor.h" // This is for the purpose of the Sensor_Data


//This will be needed by the LED task
extern volatile uint8_t blink_fast;


void Control_Update(SensorData_t *data);


#endif /* INC_CONTROL_H_ */
