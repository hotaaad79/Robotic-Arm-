/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
// ==================== GLOBAL VARIABLES ==================== //
int angle1 = 0;   // Servo1 يبدأ عند 0°
int angle2 = 0;
int angle3 = 180;
int angle4 = 180;

int old_angle1 = 0;
int old_angle2 = 0;
int old_angle3 = 0;
int old_angle4 = 0;

int deadZone = 300;
int center = 2048;

int stepSize = 1;
int delayTime = 20;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ==================== FUNCTIONS ==================== //
void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, int angle)
{
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  uint16_t pulse = 500 + ((angle * 2000) / 180);
  __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}

uint32_t readADC(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  uint32_t sum = 0;
  for (int i = 0; i < 10; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    sum += HAL_ADC_GetValue(&hadc1);
  }
  return sum / 10;
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Start PWM for all servos (PB6–PB9)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Servo1 PB6
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Servo2 PB7
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Servo3 PB8
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Servo4 PB9

  // زوايا البداية
  Set_Servo_Angle(&htim4, TIM_CHANNEL_1, angle1);
  Set_Servo_Angle(&htim4, TIM_CHANNEL_2, angle2);
  Set_Servo_Angle(&htim4, TIM_CHANNEL_3, angle3);
  Set_Servo_Angle(&htim4, TIM_CHANNEL_4, angle4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Joystick1 → Servo1, Servo2
	     uint32_t joyX1 = readADC(ADC_CHANNEL_6); // PA6
	     uint32_t joyY1 = readADC(ADC_CHANNEL_7); // PA7

	     // Joystick2 → Servo3, Servo4
	     uint32_t joyX2 = readADC(ADC_CHANNEL_9); // PB1
	     uint32_t joyY2 = readADC(ADC_CHANNEL_1); // PA1

	     // Servo1 ← X1
	     if ((int)joyX1 > center + deadZone) angle1 += stepSize;
	     else if ((int)joyX1 < center - deadZone) angle1 -= stepSize;

	     // Servo2 ← Y1
	     if ((int)joyY1 > center + deadZone) angle2 += stepSize;
	     else if ((int)joyY1 < center - deadZone) angle2 -= stepSize;

	     // Servo3 ← X2
	     if ((int)joyX2 > center + deadZone) angle3 += stepSize;
	     else if ((int)joyX2 < center - deadZone) angle3 -= stepSize;

	     // Servo4 ← Y2
	     if ((int)joyY2 > center + deadZone) angle4 += stepSize;
	     else if ((int)joyY2 < center - deadZone) angle4 -= stepSize;

	     // حدود الزوايا
	     if (angle1 < 0) angle1 = 0;
	     if (angle1 > 180) angle1 = 180;

	     if (angle2 < 0) angle2 = 0;
	     if (angle2 > 180) angle2 = 180;

	     if (angle3 < 0) angle3 = 0;
	     if (angle3 > 180) angle3 = 180;

	     if (angle4 < 0) angle4 = 0;
	     if (angle4 > 180) angle4 = 180;

	     // تحديث السيرفوهات
	     if (abs(angle1 - old_angle1) > 2) {
	       Set_Servo_Angle(&htim4, TIM_CHANNEL_1, angle1);
	       old_angle1 = angle1;
	     }
	     if (abs(angle2 - old_angle2) > 2) {
	       Set_Servo_Angle(&htim4, TIM_CHANNEL_2, angle2);
	       old_angle2 = angle2;
	     }
	     if (abs(angle3 - old_angle3) > 2) {
	       Set_Servo_Angle(&htim4, TIM_CHANNEL_3, angle3);
	       old_angle3 = angle3;
	     }
	     if (abs(angle4 - old_angle4) > 2) {
	       Set_Servo_Angle(&htim4, TIM_CHANNEL_4, angle4);
	       old_angle4 = angle4;
	     }

	     HAL_Delay(delayTime);

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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
