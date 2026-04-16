/*
 * sensor.h
 *
 *  Created on: Apr 15, 2026
 *      Author: emman
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

typedef struct{
	uint16_t pot;
	float temperature;
	float gx;
	float gy;
	float gz;
} SensorData_t;

void Sensor_ReadAll(SensorData_t *data);

#endif
/* INC_SENSOR_H_ */
