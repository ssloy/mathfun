/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint8_t system_task=0;
volatile int32_t capture4=0, capture4_prev=0, encoder4=0;
volatile int32_t capture3=0, capture3_prev=0, encoder3=0;

#define ADC_BUF_SIZE 256
volatile uint16_t ADCReadings[ADC_BUF_SIZE];
volatile int32_t adc_zero_amps = 2048;
volatile uint32_t useconds=0, useconds_prev=0, time_of_start=0;
volatile int32_t cart_position=0, cart_position_prev=0;
volatile float targetX=0, dtargetX=0.0004;

// UNCOMMENT ONE OF THESE:
// parameter-based estimation
#define PEBO
// simplified GESO for PLvCC
//#define KKL

#if defined PEBO
volatile float hatX=0, hatQ=0, hatLPX=0, hatLPQ=0;
#elif defined KKL
const float gainX = -7.;
const float gainQ = -2.;
volatile float ZX=0, ZQ=0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

#if 1
uint32_t hal_ticks_us(void) {
	uint32_t prim;
	prim = __get_PRIMASK();
	__disable_irq();

	uint32_t counter1 = SysTick->VAL;
	uint32_t millis1 = HAL_GetTick();
	uint32_t counter2 = SysTick->VAL;
	uint32_t millis2 = HAL_GetTick();
	uint32_t counter3 = SysTick->VAL;

	if (!prim) {
		__enable_irq();
	}

	if (counter1>counter2)	{ // no reload while reading millis1
		return millis1*1000L + (23999-counter1)/24;
	}
	if (counter3>counter2) // is not supposed to happen
		while (1);
	return millis2*1000L + (23999-counter3)/24;
}
#else
uint32_t hal_ticks_us(void) {
	uint32_t prim;

	// Read PRIMASK register, check interrupt status before you disable them
	// Returns 0 if they are enabled, or non-zero if disabled
	prim = __get_PRIMASK();

	// Disable interrupts
	__disable_irq();
   (void) SysTick->CTRL;
	uint32_t counter = SysTick->VAL;
	uint32_t milliseconds = HAL_GetTick();
	uint32_t status = SysTick->CTRL;

	// Enable interrupts back only if they were enabled before we disable it here in this function
	if (!prim) {
		__enable_irq();
	}

	// It's still possible for the countflag bit to get set if the counter was
	// reloaded between reading VAL and reading CTRL. With interrupts  disabled
	// it definitely takes less than 50 HCLK cycles between reading VAL and
	// reading CTRL, so the test (counter > 50) is to cover the case where VAL
	// is +ve and very close to zero, and the COUNTFLAG bit is also set.
	if ((status & SysTick_CTRL_COUNTFLAG_Msk) && counter > 50) {
		// This means that the HW reloaded VAL between the time we read VAL and the
		// time we read CTRL, which implies that there is an interrupt pending
		// to increment the tick counter.
		milliseconds++;
	}
	uint32_t load = SysTick->LOAD;
	counter = load - counter; // Convert from decrementing to incrementing

	// ((load + 1) / 1000) is the number of counts per microsecond.
	//
	// counter / ((load + 1) / 1000) scales from the systick clock to microseconds
	// and is the same thing as (counter * 1000) / (load + 1)
	return milliseconds * 1000 + (counter * 1000) / (load + 1);
}
#endif

int32_t get_cart_position() {
    return encoder4*18L; // microns, #ticks*(36 teeth * 0.002m pitch)/(1000 ticks/rev * 4x) gives meters, multiplying it by 10^6 we get microns
}

int32_t get_pendulum_angle() {
    return (encoder3-4000)*45L; // millidegrees, #ticks * 360 / (2000 ticks/rev * 4x) gives degrees
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance==TIM2) {
	}
}

void set_current(float current_ref) {
  int32_t current = current_ref * 4095 / 3.;
  if (current > 0) {
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, current);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
  } else {
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, -current);
  }
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  __HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_1);
  __HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_2);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADCReadings, ADC_BUF_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    { // handle eventual overflows in the encoder reading
      capture4 = TIM4->CNT;
      encoder4 += capture4 - capture4_prev;
      if (labs(capture4 - capture4_prev) > 32767) {
        encoder4 += (capture4 < capture4_prev ? 65535 : -65535);
      }
      capture4_prev = capture4;

      capture3 = TIM3->CNT;
      encoder3 += capture3 - capture3_prev;
      if (labs(capture3 - capture3_prev) > 32767) {
        encoder3 += (capture3 < capture3_prev ? 65535 : -65535);
      }
      capture3_prev = capture3;
    }

    int32_t adc_accum = 0;
    for (uint16_t i = 0; i < ADC_BUF_SIZE; i++) {
      adc_accum += ADCReadings[i];
    }
    adc_accum /= ADC_BUF_SIZE;

    if (0 == system_task) {
      adc_zero_amps = adc_accum;
    }

    float current_measured = (adc_accum - adc_zero_amps) / 4095.0 * 20.0;

    cart_position_prev = cart_position;
    cart_position = get_cart_position();
    int32_t pendulum_angle = get_pendulum_angle();

    while (pendulum_angle > 180000)  pendulum_angle -= 360000; // mod 2pi
    while (pendulum_angle < -180000) pendulum_angle += 360000;

    useconds_prev = useconds;
    useconds = hal_ticks_us();

    float mesQ = pendulum_angle * 3.14159 / 180000.;  // radians
    float mesX = cart_position / 1000000.;          // meters

    if (1 == system_task) {

#if defined PEBO
      hatQ = mesQ;
      hatX = mesX;
#elif defined KKL
      ZQ = mesQ * gainQ;
      ZX = mesX * gainX;
#endif
      if (labs(pendulum_angle) < 5000L) {
        system_task = 2;
        time_of_start = useconds;
        char buff2[255] = "#time, current, target_x, x, q\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t *) buff2, strlen(buff2), 3);
      }
    }

    if (2 == system_task) {
      const float Ki = 9.268;
      const float fricpos = 3.986 * .8;
      const float fricneg = -2.875 * .8;

      float u = current_measured * Ki;
      if (cart_position != cart_position_prev) {
        if (cart_position > cart_position_prev)
          u -= fricpos;
        else
          u -= fricneg;
      } else {
        if (u > 0) {
          if (u > fricpos)
            u -= fricpos;
          else
            u = 0;
        } else {
          if (u < fricneg)
            u -= fricneg;
          else
            u = 0;
        }
      }

      float dt = (useconds - useconds_prev) / 1000000.; // seconds

      float cosQ = cos(mesQ);
      float sinQ = sin(mesQ);

#if defined PEBO
      float A = 1./sqrtf(731.775 - 152.361*cosQ*cosQ);
      float hatDQ = (123.018*hatLPQ) * A;
      float hatDX = 0.933*hatLPX - (11.512*hatLPQ*cosQ) * A;
      float diffX = hatDX + 50*(mesX - hatX);
      float diffQ = 80*(mesQ - hatQ) + hatDQ;
      float diffLPX = 373.066*mesX - 373.066*hatX + .933*u + (4604.916*cosQ*(hatQ - mesQ)) * A;
      float diffLPQ = (18452.7315*(mesQ - hatQ) + 129.832*sinQ - 11.512*u*cosQ) * A;

      hatX += dt*diffX;
      hatQ += dt*diffQ;
      hatLPX += dt*diffLPX;
      hatLPQ += dt*diffLPQ;
#elif defined KKL
      float A = 1. / sqrtf(7.3178 - 1.5236 * cosQ * cosQ);
      float eX = ZX - gainX * mesX;
      float eQ = ZQ - gainQ * mesQ;

      float hatDQ = 12.3018 * eQ * A;
      float hatDX = 0.9327 * eX - 1.1512 * eQ * cosQ * A;

      float diffZQ = (12.9832 * sinQ - 1.1512 * u * cosQ) * A + gainQ * hatDQ;
      float diffZX = 0.9327 * u + gainX * hatDX;

      ZX += dt * diffZX;
      ZQ += dt * diffZQ;
#endif

      const float K[] = { 21.813858, 23.789165, 106.449942, 20.776078 };
      if (targetX > .2 || targetX < -.2)
        dtargetX = -dtargetX;
      targetX += dtargetX;

      int32_t f = K[0] * (mesX - targetX) + K[1] * (hatDX - dtargetX / 0.005) + K[2] * mesQ + K[3] * hatDQ; // Newtons

      float antifriction = 0;
      if (cart_position != cart_position_prev) {
        if (cart_position > cart_position_prev)
          antifriction = fricpos;
        else
          antifriction = fricneg;
      } else {
        if (fabs(f) > 0.01) {
          if (f > 0)
            antifriction = fricpos;
          else
            antifriction = fricneg;
        } else
          f = 0;
      }
      float current_ref = (f + antifriction) / Ki;
      if (current_ref > 3.3)
        current_ref = 3.3;
      if (current_ref < -3.3)
        current_ref = -3.3;

      set_current(current_ref);

      if (labs(cart_position) > 350000L || labs(pendulum_angle) > 30000L) {
        system_task = 9;
        set_current(0);
      }

      char buff[255] = { 0 };
      sprintf(buff, "%3.5f, %1.3f, %3.4f, %3.4f, %3.4f\r\n", (useconds - time_of_start) * 1e-6, u, targetX, mesX, mesQ);
      HAL_UART_Transmit(&huart1, (uint8_t *) buff, strlen(buff), 3);
    }
    while (hal_ticks_us() - useconds < 4900);
  }
  /* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT2 config 
    */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 23;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 12;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 12;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
