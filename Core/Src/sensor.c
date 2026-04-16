/*
 * sensor.c
 *
 *  Created on: Apr 15, 2026
 *      Author: emman
 */
#include "main.h"
#include "sensor.h"

extern uint16_t ReadPotentiometer(void);
extern float ReadTemperature(void);
extern void Gyroscope_Conversion(float *, float *, float *); // Please during definition, the names do not matter

void Sensor_ReadAll(SensorData_t *data){

	data->pot = ReadPotentiometer();
	data->temperature = ReadTemperature();
	Gyroscope_Conversion(&data->gx, &data->gy, &data->gz);
}

