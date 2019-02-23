#include "dwt_stm32_delay.h"

volatile uint32_t MHz = 0;
volatile uint32_t overflow_micros = 0;
volatile uint32_t cnt_micros=0, cnt_micros_prev=0, cnt_overflow=0;

/**
 * @brief  Initializes DWT_Clock_Cycle_Count for DWT_Delay_us function
 * @return Error DWT counter
 *         1: clock cycle counter not started
 *         0: clock cycle counter works
 */
HAL_StatusTypeDef DWT_Delay_Init(void) {
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    MHz = HAL_RCC_GetHCLKFreq()/1000000L;
    overflow_micros = 4294967295L/MHz;

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT) {
        return HAL_OK; /*clock cycle counter started*/
    } else {
        return HAL_ERROR; /*clock cycle counter not started*/
    }
}


uint32_t DWT_us() {
  cnt_micros_prev = cnt_micros;
  cnt_micros = DWT->CYCCNT/MHz;
  if (cnt_micros_prev>cnt_micros) cnt_overflow++;
  return cnt_overflow*overflow_micros + cnt_micros;
}
