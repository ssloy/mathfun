
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <math.h>

#include "dwt_stm32_delay.h"
#include "udp_client.h"
#include "gl_svg.h"
#include "dynamixel_protocol_v2.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */

#define TORQUE_CONTROL 0

extern uint32_t dynamixel_comm_err_count1;
extern uint32_t dynamixel_comm_err_count2;
volatile uint8_t system_task = 0;

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM10_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void user_pwm_setvalue(uint16_t value) {
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = value;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  if (DWT_Delay_Init() != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LWIP_Init();
  MX_UART7_Init();
  MX_UART5_Init();
  MX_TIM13_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  uint16_t pwm_value1 = 0, pwm_value2 = 0;
  user_pwm_setvalue(pwm_value1);
  user_pwm_setvalue(pwm_value2);

  udp_client_connect();
  GLVG_init(&huart7);

  const uint8_t dynamixel1_id = 1;
  const uint8_t dynamixel2_id = 2;
  dynamixel_bind_uart(&huart5, 1000000L);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  float current1 = 0, velocity1 = 0, position1 = 0, start_position1 = 0;
  float current2 = 0, velocity2 = 0, position2 = 0, start_position2 = 0;

  float qa_prev = 0, qa_goal = 0;

  float angle_filtered = 0;
  int16_t pwm = 0;

  uint32_t time_start = DWT_us();
  uint32_t time_prev = DWT_us();

#if TORQUE_CONTROL
  float goal_current1 = 0.f;
  float goal_current2 = 0.f;
#else
  float goal_velocity1 = 0.f;
  float goal_velocity2 = 0.f;
#endif
  uint8_t read_send = 0;
  uint8_t need_to_rewind = 1;

/*
  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 255);
  HAL_Delay(10000);
  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 64);
  HAL_Delay(30000);
*/

  while (1) {

	  /*
	  if (time_prev-time_start>1L*1000L*1000L) {
		   system_task = 9;
	  }
	  */

	  MX_LWIP_Process();
	  uint32_t time = DWT_us();
	  if (time-time_prev>5000L) {
		  time_prev = time;
		  read_send = 1-read_send;

		  if (0==system_task) {
			  angle_filtered = -.73;

			  pwm_value1 = pwm_value2 = 0;
			  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_value1);
			  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pwm_value2);
			  need_to_rewind = 1;

			  HAL_Delay(50);
			  dynamixel_torque_on_off(dynamixel1_id, 0);
			  HAL_Delay(50);
			  dynamixel_torque_on_off(dynamixel2_id, 0);
			  HAL_Delay(50);
#if TORQUE_CONTROL
			  dynamixel_set_operating_mode(dynamixel1_id, 0);
			  HAL_Delay(50);
			  dynamixel_set_operating_mode(dynamixel2_id, 0);
#else
			  dynamixel_set_operating_mode(dynamixel1_id, 1);
			  HAL_Delay(50);
			  dynamixel_set_operating_mode(dynamixel2_id, 1);
#endif
			  HAL_Delay(50);
			  dynamixel_torque_on_off(dynamixel1_id, 1);
			  HAL_Delay(50);
			  dynamixel_torque_on_off(dynamixel2_id, 1);
			  HAL_Delay(50);

			  while (!dynamixel_read_current_velocity_position(dynamixel1_id, &pwm, &current1, &velocity1, &start_position1));
			  position1 = start_position1;
			  HAL_Delay(50);
			  while (!dynamixel_read_current_velocity_position(dynamixel2_id, &pwm, &current2, &velocity2, &start_position2));
			  position2 = start_position2;

			  for (uint8_t i=0; i<115; i++) {
				  pwm_value1 = i;
				  if (i<113)
					  pwm_value2 = i;

				  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_value1);
				  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pwm_value2);
				  HAL_Delay(10);
			  }

			  qa_prev = qa_goal = -GLVG_getYaw()*M_PI/180.;

			  time_start = DWT_us();
			  system_task = 1;

			  continue;
		  }

		  if (read_send && 9==system_task) {
			  if (need_to_rewind) {
				  dynamixel_torque_on_off(dynamixel1_id, 0);
				  HAL_Delay(50);
				  dynamixel_torque_on_off(dynamixel2_id, 0);
				  HAL_Delay(50);
				  dynamixel_set_operating_mode(dynamixel1_id, 3);
				  HAL_Delay(50);
				  dynamixel_set_operating_mode(dynamixel2_id, 3);
				  HAL_Delay(50);
				  dynamixel_set_profile_velocity(dynamixel1_id, 32);
				  HAL_Delay(50);
				  dynamixel_set_profile_velocity(dynamixel2_id, 32);
				  HAL_Delay(50);
				  dynamixel_torque_on_off(dynamixel1_id, 1);
				  HAL_Delay(50);
				  dynamixel_torque_on_off(dynamixel2_id, 1);
				  HAL_Delay(50);
				  dynamixel_set_position(dynamixel1_id, start_position1);
				  HAL_Delay(50);
				  dynamixel_set_position(dynamixel2_id, start_position2);
				  HAL_Delay(5000);
				  need_to_rewind = 0;
			  }
			  dynamixel_torque_on_off(dynamixel1_id, 0);
			  HAL_Delay(50);
			  dynamixel_torque_on_off(dynamixel2_id, 0);
		  }

		  if (read_send && 1==system_task) {
#if TORQUE_CONTROL
			  dynamixel_set_current(dynamixel1_id, -goal_current1); // note that the dynamixel basis is inversed w.r.t the paper
			  dynamixel_set_current(dynamixel2_id, -goal_current2); // note that the dynamixel basis is inversed w.r.t the paper
#else
			  dynamixel_set_velocity(dynamixel1_id, -goal_velocity1); // note that the dynamixel basis is inversed w.r.t the paper
			  dynamixel_set_velocity(dynamixel2_id, -goal_velocity2); // note that the dynamixel basis is inversed w.r.t the paper
#endif
		  }

		  if (!read_send) {
			  bool res1 = dynamixel_read_current_velocity_position(dynamixel1_id, &pwm, &current1, &velocity1, &position1);
			  if (!res1) {
				  dynamixel_comm_err_count1++;
			  } else {
				  current1 = -current1; // inversed basis
				  velocity1 = -velocity1;
			  }
			  bool res2 = dynamixel_read_current_velocity_position(dynamixel2_id, &pwm, &current2, &velocity2, &position2);
			  if (!res2) {
				  dynamixel_comm_err_count2++;
			  } else {
				  current2 = -current2; // inversed basis
				  velocity2 = -velocity2;
			  }

			  float qa = -GLVG_getYaw()*M_PI/180.;
			  if (qa>qa_prev+5.) qa -= 2.*M_PI;
			  if (qa<qa_prev-5.) qa += 2.*M_PI;
			  qa_prev = qa;

			  float wa = -GLVG_getGy()*M_PI/180.;

			  float qb = (90 - GLVG_getRoll())*M_PI/180.;
			  float wb = -GLVG_getGx()*M_PI/180.;
			  float qc1 = M_PI/2. - (position1-start_position1);
			  float wc1 = velocity1;
			  float qc2 = M_PI/2. - (position2-start_position2);
			  float wc2 = velocity2;

		      if (fabs(qb+M_PI/4.)<.2) {
		    	  angle_filtered = .996*angle_filtered + .004*qb;
		      }

			  if (1==system_task && ( fabs(qc1-M_PI/2.)>1.5*3.14/4. || fabs(qc2-M_PI/2.)>1.5*3.14/4. )) system_task = 9;

#if TORQUE_CONTROL
			  float K1[] = {-1.43910573e-01,-4.77325439e-02,5.55964166e+00,7.24108243e-01,-1.46748079e-03,2.38036156e-01,7.05639301e-01,-1.16024017e-02};
			  float K2[] = {-1.43910170e-01,-4.67773438e-02,-5.55964166e+00,-7.24108243e-01,7.05639309e-01,-1.15814209e-02,-1.46747157e-03,2.38064575e-01};

		      float tau1 = -(K1[0]*(qa-qa_goal) + K1[1]*wa + K1[2]*(qb + .73 /* M_PI/4. */) + K1[3]*wb + K1[4]*(qc1 - M_PI/2.) + K1[5]*wc1 + K1[6]*(qc2 - M_PI/2.) + K1[7]*wc2);
		      float tau2 = -(K2[0]*(qa-qa_goal) + K2[1]*wa + K2[2]*(qb + .73 /* M_PI/4. */) + K2[3]*wb + K2[4]*(qc1 - M_PI/2.) + K2[5]*wc1 + K2[6]*(qc2 - M_PI/2.) + K2[7]*wc2);

			  float k1 = 1.62;
			  float k2 = 1.46;
		      goal_current1 = tau1 / k1;
		      goal_current2 = tau2 / k2;

		      if (goal_current1 >  4) goal_current1 =  4;
		      if (goal_current1 < -4) goal_current1 = -4;
		      if (goal_current2 >  4) goal_current2 =  4;
		      if (goal_current2 < -4) goal_current2 = -4;

		      if (fabs(qb+M_PI/4.)>.2) {
		    	  goal_current1 = 0;
		    	  goal_current2 = 0;
		      }
			  char msg[255] = {0};
			  sprintf(msg,"%3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f, %3.6f,                %d, %lu", (float)(time-time_start)*1e-6, goal_current1, goal_current2, qa-qa_goal, wa, qb, wb, qc1, wc1, qc2, wc2, system_task, dynamixel_comm_err_count1);
#else
			  float K1[] = { 16.175,  2.418, 0., 1.};
			  float K2[] = {-16.175, -2.418, 1., 0.};
		      goal_velocity1 = -(K1[0]*(qb - angle_filtered) + K1[1]*wb + K1[2]*(qc1 - M_PI/2.)+ K1[3]*(qc2 - M_PI/2.));
		      goal_velocity2 = -(K2[0]*(qb - angle_filtered) + K2[1]*wb + K2[2]*(qc1 - M_PI/2.)+ K2[3]*(qc2 - M_PI/2.));

		      if (goal_velocity1 >  4) goal_velocity1 =  4;
		      if (goal_velocity1 < -4) goal_velocity1 = -4;
		      if (goal_velocity2 >  4) goal_velocity2 =  4;
		      if (goal_velocity2 < -4) goal_velocity2 = -4;

		      if (fabs(qb+M_PI/4.)>.2) {
		    	  goal_velocity2 = goal_velocity1 = 0;
		      }

			  char msg[255] = {0};
			  sprintf(msg,"%3.6f,\t %3.6f,\t %3.6f,\t %3.6f,\t %3.6f,\t %3.6f,\t %3.6f, %lu, %lu", (float)(time-time_start)*1e-6, qb, wb, qc1, wc1, qc2, wc2, dynamixel_comm_err_count1, dynamixel_comm_err_count2);
#endif
			  udp_client_send(msg);
		  }
	  }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2160-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim10);

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1080-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 2000-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim13);

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 1000000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart5.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart5.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_RS485Ex_Init(&huart5, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART7 init function */
static void MX_UART7_Init(void)
{

  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart7.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart7.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_RS485Ex_Init(&huart7, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
