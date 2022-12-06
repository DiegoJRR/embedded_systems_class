/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

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
ADC_HandleTypeDef hadc2;

/* Definitions for breakTask */
osThreadId_t breakTaskHandle;
const osThreadAttr_t breakTask_attributes = {
  .name = "breakTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for throttleTask */
osThreadId_t throttleTaskHandle;
const osThreadAttr_t throttleTask_attributes = {
  .name = "throttleTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for dataProcessingT */
osThreadId_t dataProcessingTHandle;
const osThreadAttr_t dataProcessingT_attributes = {
  .name = "dataProcessingT",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for outputTask */
osThreadId_t outputTaskHandle;
const osThreadAttr_t outputTask_attributes = {
  .name = "outputTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gearTask */
osThreadId_t gearTaskHandle;
const osThreadAttr_t gearTask_attributes = {
  .name = "gearTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BrakeQueue */
osMessageQueueId_t BrakeQueueHandle;
const osMessageQueueAttr_t BrakeQueue_attributes = {
  .name = "BrakeQueue"
};
/* Definitions for ThrottleQueue */
osMessageQueueId_t ThrottleQueueHandle;
const osMessageQueueAttr_t ThrottleQueue_attributes = {
  .name = "ThrottleQueue"
};
/* Definitions for GearQueue */
osMessageQueueId_t GearQueueHandle;
const osMessageQueueAttr_t GearQueue_attributes = {
  .name = "GearQueue"
};
/* Definitions for SpeedQueue */
osMessageQueueId_t SpeedQueueHandle;
const osMessageQueueAttr_t SpeedQueue_attributes = {
  .name = "SpeedQueue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void BreakInputTask(void *argument);
void ThrottleInputTask(void *argument);
void DataProcessingTask(void *argument);
void OutputDisplayTask(void *argument);
void GearSelectionTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void writeToDisplay(char hexChar) {
  // Write a char (8 bits) to the 8 data pins (A0 - A7)
  int dataPins[8] = {GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7};

  for(int i = 0; i<8; i++) {
    int result = hexChar & (1 << i);
    GPIO_PinState pinState = result ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(GPIOA, dataPins[i], pinState);
    osDelay(1);
  }

  // Set E = 1 for at least 230ns (latching pulse)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  osDelay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  osDelay(5);
}

void sendData(char hexChar) {
  // E not enabled
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  osDelay(3);

  // Select data register
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  osDelay(3);

  // Select write mode
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  osDelay(3);

  // Call function to write hex character to D7-D0
  writeToDisplay(hexChar);
}

void sendCommand(char hexCommand) {
  // E not enabled
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  osDelay(3);

  // Select command register
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  osDelay(3);

  // Select write mode
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  osDelay(3);

  // Call function to write hex character to D7-D0
  writeToDisplay(hexCommand);
}

void clearDisplay() {
   // E not enabled
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  osDelay(10);

  sendCommand(56);
  osDelay(10);
  sendCommand(15);
  osDelay(10);
  sendCommand(1);
  osDelay(10);
  sendCommand(6);
  osDelay(10);
}

void printDigits(int value, int num_digits) {
    int digits[6] = {0};

    int i = 0;
    while(value) {
        digits[i] = value % 10;
        value /= 10;
        i++;
    }

    for(int i = num_digits - 1; i >= 0; i--){
        sendData(0b00110000 + digits[i]);
    }
}

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of BrakeQueue */
  BrakeQueueHandle = osMessageQueueNew (16, sizeof(float), &BrakeQueue_attributes);

  /* creation of ThrottleQueue */
  ThrottleQueueHandle = osMessageQueueNew (16, sizeof(float), &ThrottleQueue_attributes);

  /* creation of GearQueue */
  GearQueueHandle = osMessageQueueNew (16, sizeof(uint8_t), &GearQueue_attributes);

  /* creation of SpeedQueue */
  SpeedQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &SpeedQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of breakTask */
  breakTaskHandle = osThreadNew(BreakInputTask, NULL, &breakTask_attributes);

  /* creation of throttleTask */
  throttleTaskHandle = osThreadNew(ThrottleInputTask, NULL, &throttleTask_attributes);

  /* creation of dataProcessingT */
  dataProcessingTHandle = osThreadNew(DataProcessingTask, NULL, &dataProcessingT_attributes);

  /* creation of outputTask */
  outputTaskHandle = osThreadNew(OutputDisplayTask, NULL, &outputTask_attributes);

  /* creation of gearTask */
  gearTaskHandle = osThreadNew(GearSelectionTask, NULL, &gearTask_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_BreakInputTask */
/**
  * @brief  Function implementing the breakTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_BreakInputTask */
void BreakInputTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // Read value from potentiometer
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 300);
	int potentiometerValue = HAL_ADC_GetValue(&hadc2);

	float currentValue = potentiometerValue/4095.0f;

    // Place value in BrakeQueue
	xQueueSendToBack(BrakeQueueHandle, &currentValue, 10);
    
    osDelay(10);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ThrottleInputTask */
/**
* @brief Function implementing the throttleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ThrottleInputTask */
void ThrottleInputTask(void *argument)
{
  /* USER CODE BEGIN ThrottleInputTask */
  /* Infinite loop */
  for(;;)
  {
     // Read value from potentiometer
     HAL_ADC_Start(&hadc1);
     HAL_ADC_PollForConversion(&hadc1, 300);
     int potentiometerValue = HAL_ADC_GetValue(&hadc1);

	float currentValue = potentiometerValue/4095.0f;

    // Place value in ThrottleQueue
	xQueueSendToBack(ThrottleQueueHandle, &currentValue, 10);

    osDelay(10);
  }
  /* USER CODE END ThrottleInputTask */
}

/* USER CODE BEGIN Header_DataProcessingTask */
/**
* @brief Function implementing the dataProcessingT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DataProcessingTask */
void DataProcessingTask(void *argument)
{
  /* USER CODE BEGIN DataProcessingTask */
  // Only once. 

  // Iniitialize variables
  int DesiredSpeed = 0;
  float AverageBrakePercentage = 0.0;
  float AverageThrottlePercentage = 0.0;
  int RequestedGear = 0;

  int Acceleration;
  int MaximumSpeed;
  

  /* Infinite loop */
  for(;;)
  {
    // Read values from queues amnd calculate averages
    // Break Queue
    float valueFromBrakeQueue;
    int itemsInBrakeQueue = uxQueueMessagesWaiting(BrakeQueueHandle);
    AverageBrakePercentage = 0.0;
    for(int i = 0; i < itemsInBrakeQueue; i++) {
        if(xQueueReceive(BrakeQueueHandle, &valueFromBrakeQueue, 10) == pdTRUE) {
            AverageBrakePercentage = AverageBrakePercentage + valueFromBrakeQueue/itemsInBrakeQueue;
        }
    }
    
    // Throttle Queue
    float valueFromThrottleQueue;
    int itemsInThrottleQueue = uxQueueMessagesWaiting(ThrottleQueueHandle);
    AverageThrottlePercentage = 0.0;
    for(int i = 0; i < itemsInThrottleQueue; i++) {
        if(xQueueReceive(ThrottleQueueHandle, &valueFromThrottleQueue, 10) == pdTRUE) {
            AverageThrottlePercentage = AverageThrottlePercentage + valueFromThrottleQueue/itemsInThrottleQueue;
        }
    }

    // Read value from Gear queue
    if(xQueueReceive(GearQueueHandle, &RequestedGear, 10) != pdTRUE) {
    	RequestedGear = 0;
    };
    
    if(RequestedGear == 0) {
        Acceleration = -400;
    } else if(RequestedGear == 1) {
        // Gear is Forward Speed 1
        MaximumSpeed = 80000;

        // Invalid state where breaks and throttle are pressed at the same time
        if(AverageBrakePercentage > 0 && AverageThrottlePercentage > 0) {
            Acceleration = 0;
        }

        if(AverageBrakePercentage == 0 && AverageThrottlePercentage > 0) {
             Acceleration = 1600*AverageThrottlePercentage;
        }

        if(AverageThrottlePercentage == 0 && AverageBrakePercentage > 0) {
            Acceleration = -3200*AverageBrakePercentage;
        }
    } else {
        // Gear is Forward Speed 2
        MaximumSpeed = 200000;
    
        // Invalid state where breaks and throttle are pressed at the same time
        if(AverageBrakePercentage > 0 && AverageThrottlePercentage > 0) {
            Acceleration = 0;
        }
        
        if(AverageBrakePercentage == 0 && AverageThrottlePercentage > 0) {
             Acceleration = 2400*AverageThrottlePercentage;
        }

        if(AverageThrottlePercentage == 0 && AverageBrakePercentage > 0) {
            Acceleration = -3200*AverageBrakePercentage;
        }
    }

    DesiredSpeed = DesiredSpeed + Acceleration;
    if(DesiredSpeed < 0) {
        DesiredSpeed = 0;
    } else if(DesiredSpeed > MaximumSpeed) {
        DesiredSpeed = MaximumSpeed;
    }

    // Place desired speed in the queue
    xQueueSendToBack(SpeedQueueHandle, &DesiredSpeed, 10);

    if(RequestedGear == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }

    osDelay(100);
  }
  /* USER CODE END DataProcessingTask */
}

/* USER CODE BEGIN Header_OutputDisplayTask */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutputDisplayTask */
void OutputDisplayTask(void *argument)
{
  /* USER CODE BEGIN OutputDisplayTask */
  // Initialize variables
  int AverageDesiredSpeed = 0;

  /* Infinite loop */
  for(;;)
  {
    // Read data from speed
    int valueFromSpeedQueue;
    int itemsInSpeedQueue = uxQueueMessagesWaiting(SpeedQueueHandle);
    AverageDesiredSpeed = 0;
    for(int i = 0; i < itemsInSpeedQueue; i++) {
        if(xQueueReceive(SpeedQueueHandle, &valueFromSpeedQueue, 10) == pdTRUE) {
            AverageDesiredSpeed = AverageDesiredSpeed + valueFromSpeedQueue/itemsInSpeedQueue;
        }
    }

    float speedPercentage = AverageDesiredSpeed/200000.0f;

    // Convert to km/h
    int SpeedToDisplay = AverageDesiredSpeed/1000;


    // Set percentage as duty cycle in PWM pin
    // Display the SpeedToDisplay to an external display
	clearDisplay();
	printDigits((int)SpeedToDisplay, 3);
	sendData(' ');
	sendData('k');
	sendData('p');
	sendData('h');
	sendCommand(0xC0);
	printDigits((int)(speedPercentage*100), 3);
	sendData('%');

    osDelay(500);
  }
  /* USER CODE END OutputDisplayTask */
}

/* USER CODE BEGIN Header_GearSelectionTask */
/**
* @brief Function implementing the gearTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GearSelectionTask */
void GearSelectionTask(void *argument)
{
  /* USER CODE BEGIN GearSelectionTask */

  /* Infinite loop */
  for(;;)
  {
    // 0 -> Parking/Neutral
    // 1 -> Forward Speed 1
    // 2 -> Forward Speed 2

    // Read 2 bit dip switch and convert to 0, 1, or 2
    // B10, B11
    int selectedGear = 0;

    int firstPin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
    int secondPin = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);

    selectedGear = firstPin + 2*secondPin;

    // Write to GearQueue
    xQueueSendToBack(GearQueueHandle, &selectedGear, 10);

    osDelay(50);
  }
  /* USER CODE END GearSelectionTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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
