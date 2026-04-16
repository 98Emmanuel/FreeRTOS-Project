#include <led.h>
#include <main.h>
#include <cmsis_os.h>

static LedMode_t CurentMode = LED_SLOW; // This means that it is private to only this file


void LED_SetMode(LedMode_t mode){ // This setting is coming from the control.c file
	CurentMode = mode;
}


void LED_Update(){ // This setting is coming from the main.c file
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

	switch(CurentMode){

	case LED_SLOW:
		osDelay(500);
		break;

	case LED_FAST:
		osDelay(50);
		break;

	case LED_OFF:
		osDelay(1000);
		break;
	}
}

