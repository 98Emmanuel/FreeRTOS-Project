/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <sensor.h>
#include <control.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <led.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensorQueue */
osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "sensorQueue"
};
/* Definitions for uartSensorQueue */
osMessageQueueId_t uartSensorQueueHandle;
const osMessageQueueAttr_t uartSensorQueue_attributes = {
  .name = "uartSensorQueue"
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};
/* Definitions for dataReadySem */
osSemaphoreId_t dataReadySemHandle;
const osSemaphoreAttr_t dataReadySem_attributes = {
  .name = "dataReadySem"
};
/* Definitions for imuInterruptSerm */
osSemaphoreId_t imuInterruptSermHandle;
const osSemaphoreAttr_t imuInterruptSerm_attributes = {
  .name = "imuInterruptSerm"
};
/* USER CODE BEGIN PV */

uint16_t value;

float addrp;

float read_temp;

#define     ADDR      0x3F
#define     WHOAMI	  0x01
#define 	CTRL	  0x04
#define 	TEMP_L	  0x06
#define 	TEMP_H	  0x07

#define      CS_LOW()   			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#define      CS_HIGH()				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

#define     SPI_READ_BIT		0x80
#define 	SPI_WRITE_BIT		0x7F

#define     IMU_WHOAMI			0x0F
#define     CTRL1				0x10
#define     CTRL2				0x11
#define     CTRL3				0x12
#define 	CTRL6				0x15

#define 	OUTX_L_G 			0x22
#define 	STATUS_REG			0x1E
#define		INT1_CTRL			0x0D


int16_t gx_raw;
int16_t gy_raw;
int16_t gz_raw;

float gx_conv;
float gy_conv;
float gz_conv;

float gx_c;
float gy_c;
float gz_c;


//volatile uint8_t blink_fast = 0;






int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10);
	return ch;
}



//jUST FOR YOUR KNOWLEDGE... PRINTF IS NOT THE BEST FOR RTOS!

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartSENSORTask(void *argument);
void StartCONTROLTask(void *argument);
void StartLEDTask(void *argument);
void StartUARTTask(void *argument);

/* USER CODE BEGIN PFP */

uint16_t ReadPotentiometer(void);
float ReadTemperature(void);
uint8_t I2C_ReadRegister(uint8_t reg);
void I2C_WriteRegister(uint8_t reg, uint8_t data);
uint8_t I2C_Scan(void);

uint8_t SPI_Read_Register(uint8_t reg);
void SPI_Write_Register(uint8_t reg, uint8_t data);
void Gyroscope_Conversion(float *gx_conv, float *gy_conv, float *gz_conv);
void ReadGyroscope(int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

//  ReadTemperature();

  I2C_WriteRegister(CTRL, 0xC8);

  SPI_Write_Register(CTRL3, 0x44); // This is configuring both the BDU and the IF_INC

  SPI_Write_Register(CTRL1, 0x09); //This is powering on the accelerometer and configuring it to high performance and high frequency mode

  SPI_Write_Register(CTRL2, 0x08);  //This is powering the gyroscope and configuring it to high performance high frequency mode

  SPI_Write_Register(CTRL6, 0x20);  // This is configuring the full scale (FS) for the gyroscope

  SPI_Write_Register(INT1_CTRL, 0x02); // This enables the interrupt pin when the gyroscope data is ready



//  Gyroscope_Conversion(&gx_conv, &gy_conv, &gz_conv);
//
//  char msg[64];
//
//  int len = snprintf(msg, sizeof(msg), "gx_c : %.2f, gy_c : %.2f, gz_c : %.2f", gx_conv, gy_conv, gz_conv);
//
//  HAL_UART_Transmit(&huart2, (uint8_t *)msg, len, HAL_MAX_DELAY);

// This is the last place where you can use the HAL_Delay(ms) - before you start the scheduler

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of dataReadySem */
  dataReadySemHandle = osSemaphoreNew(1, 0, &dataReadySem_attributes);

  /* creation of imuInterruptSerm */
  imuInterruptSermHandle = osSemaphoreNew(10, 0, &imuInterruptSerm_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of sensorQueue */
  sensorQueueHandle = osMessageQueueNew (8, sizeof(SensorData_t), &sensorQueue_attributes);

  /* creation of uartSensorQueue */
  uartSensorQueueHandle = osMessageQueueNew (8, sizeof(SensorData_t), &uartSensorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSENSORTask, NULL, &sensorTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartCONTROLTask, NULL, &controlTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLEDTask, NULL, &ledTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(StartUARTTask, NULL, &uartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  Gyroscope_Conversion(&gx_conv, &gy_conv, &gz_conv);
		  osDelay(500);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint16_t ReadPotentiometer(void){

//	In my configurations, I left the continous conversion mode in the disabled state...Because here I manually start and stop the readings!

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);


	HAL_ADC_Start(&hadc1);

	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	value = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	return value;
}

uint8_t I2C_Scan(void){

	for(uint8_t addr = 1; addr < 127; addr++ ){
		if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK){
	//		char msg2[64];
	//
	//		int len1 = snprintf(msg2, sizeof(msg2), "The correct address is : 0x%02X", ADDR);
	//
	//		HAL_UART_Transmit(&huart2, (uint8_t *)msg2, len1, HAL_MAX_DELAY);

			return addr;


		}

	}


	return 0xFF;
}

uint8_t I2C_ReadRegister(uint8_t reg){
	uint8_t whoami = 0;

	HAL_I2C_Mem_Read(&hi2c1, ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 10);

	return whoami;
}

void I2C_WriteRegister(uint8_t reg, uint8_t data){

	HAL_I2C_Mem_Write(&hi2c1, ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);


}

float ReadTemperature(void){
	uint8_t temp[2] = {0};

	HAL_I2C_Mem_Read(&hi2c1, ADDR << 1, TEMP_L, I2C_MEMADD_SIZE_8BIT, temp, 2, 10);

	int16_t temperature = (int16_t)(temp[1] << 8 | temp[0]);

	read_temp = temperature / 100.0f;

	return read_temp;
}


uint8_t SPI_Read_Register(uint8_t reg){
	uint8_t tx[2] = {0};

	uint8_t rx[2] = {0};

	tx[0] = reg | SPI_READ_BIT;
	tx[1] = 0x00;

	CS_LOW();

	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 50);

	CS_HIGH();

	return rx[1];
}

void SPI_Write_Register(uint8_t reg, uint8_t data){

	uint8_t tx[2] = {0};

	tx[0] = reg & SPI_WRITE_BIT;
	tx[1] = data;

	CS_LOW();

	HAL_SPI_Transmit(&hspi2, tx, 2, 10);

	CS_HIGH();


}

void ReadGyroscope(int16_t *gx_raw, int16_t *gy_raw, int16_t *gz_raw){
	uint8_t tx[7] = {0};
	uint8_t rx[7] = {0};

	tx[0] = OUTX_L_G | SPI_READ_BIT;
	tx[1] = 0x00;


	CS_LOW();

	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 7, 10);

	CS_HIGH();

	*gx_raw = (int16_t)(rx[2] << 8 | rx[1]);

	*gy_raw = (int16_t)(rx[4] << 8 | rx[3]);

	*gz_raw = (int16_t)(rx[6] << 8 | rx[5]);

}

void Gyroscope_Conversion(float *gx_conv, float *gy_conv, float *gz_conv){

//	uint8_t status = SPI_Read_Register(STATUS_REG);



		ReadGyroscope(&gx_raw, &gy_raw, &gz_raw);

		*gx_conv = gx_raw * 0.004375f;

		*gy_conv = gy_raw * 0.004375f;

		*gz_conv = gz_raw * 0.004375f;


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */

  if(GPIO_Pin == GPIO_PIN_12){
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	  osSemaphoreRelease(imuInterruptSermHandle);
  }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSENSORTask */
/**
  * @brief  Function implementing the sensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSENSORTask */
void StartSENSORTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	SensorData_t data;

  for(;;){

//	  osSemaphoreAcquire(imuInterruptSermHandle, osWaitForever);



//	  data.pot = ReadPotentiometer();
//
//	  data.temperature = ReadTemperature();
//
////	  data.spi_register = SPI_Read_Register(IMU_WHOAMI);
//
//	  Gyroscope_Conversion(&data.gx, &data.gy, &data.gz); // Interesting part.. because the function expects address of a float whose value its going to update
//
//	  // And the float that we supply in this case is the float contained in our struct

	  Sensor_ReadAll(&data);

	  osMessageQueuePut(sensorQueueHandle, &data, 0, osWaitForever);

	  osMessageQueuePut(uartSensorQueueHandle, &data, 0, osWaitForever);

//	  osSemaphoreRelease(imuInterruptSermHandle); Semaphore released by the interrupt pin

	  osDelay(500);  //Mark you that this delay we put here for the purpose of allowing the sensor to acquire new readings
//	  When we use the imu interrupt, we no longer need this interrupt

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCONTROLTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCONTROLTask */
void StartCONTROLTask(void *argument)
{
  /* USER CODE BEGIN StartCONTROLTask */
  /* Infinite loop */

SensorData_t receivedControl;

  for(;;)
  {

	  if(osMessageQueueGet(sensorQueueHandle, &receivedControl, 0, osWaitForever) == osOK){


		  Control_Update(&receivedControl);

  }

  /* USER CODE END StartCONTROLTask */
}
}

/* USER CODE BEGIN Header_StartLEDTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN StartLEDTask */
  /* Infinite loop */

//	Pleasde I have disabled this task from the top

  for(;;)
  {
	  LED_Update();

  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */

	SensorData_t received;

	char msg1[90];

  for(;;)
  {
	  if(osMessageQueueGet(uartSensorQueueHandle, &received, 0, osWaitForever) == osOK){

		  osMutexAcquire(uartMutexHandle, osWaitForever);

		  int len = snprintf(msg1, sizeof(msg1), "The Potentiometer is : %hu, The Temperature is : %.2f, gx : %.2f, gy : %.2f, gz : %.2f \n\r", received.pot, received.temperature, received.gx, received.gy, received.gz);

		  HAL_UART_Transmit(&huart2, (uint8_t *)msg1, len, HAL_MAX_DELAY);

		  osMutexRelease(uartMutexHandle);
	  }

	  osDelay(500);


  }
  /* USER CODE END StartUARTTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
