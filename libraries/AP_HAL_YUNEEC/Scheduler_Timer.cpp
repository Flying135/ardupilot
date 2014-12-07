#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "Scheduler.h"
#include <stdint.h>
#include <core_cm4.h>
#include <core_cmFunc.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <utility/pinmap_typedef.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define TIM_PERIOD		(uint32_t)(1000 - 1)
#define TIM_PRESCALER	(uint16_t)(SystemCoreClock / 2 / 1000000  - 1)

extern "C"
{
	static voidFuncPtr timer6_callback = NULL;
	static voidFuncPtr timer7_failsafe = NULL;
	static volatile uint32_t timer_micros_counter = 0;
	static volatile uint32_t timer_millis_counter = 0;

	void TIM7_IRQHandler(void) {
		TIM7->SR &= (uint16_t)~TIM_FLAG_Update;

		timer_micros_counter += 1000;
		timer_millis_counter += 1;

		if (timer7_failsafe != NULL) {
			timer7_failsafe();
		}

	}

	void TIM6_DAC_IRQHandler(void) {
		TIM6->SR &= (uint16_t)~TIM_FLAG_Update;

	    if(timer6_callback)
	    	timer6_callback();

	}
}

void YUNEECTimer::init() {
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// TIM6 clock enable
    RCC->APB1ENR |= (RCC_APB1Periph_TIM6 | RCC_APB1Periph_TIM7 | RCC_APB1Periph_TIM14);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

    /* Reset the ARR Preload Bit */
    TIM6->CR1 &= ~((uint16_t)TIM_CR1_ARPE);
    TIM7->CR1 &= ~((uint16_t)TIM_CR1_ARPE);
    TIM14->CR1 &= ~((uint16_t)TIM_CR1_ARPE);

	// Configure two bits for preemption priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the TIM6 gloabal Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	// TIM Interrupts enable
    TIM6->DIER |= TIM_IT_Update;
    TIM7->DIER |= TIM_IT_Update;
    TIM14->DIER &= (uint16_t)~TIM_IT_Update;

	// TIM6 enable counter
    TIM6->CR1 |= TIM_CR1_CEN;
    TIM7->CR1 |= TIM_CR1_CEN;
    TIM14->CR1 |= TIM_CR1_CEN;

}

uint32_t YUNEECTimer::micros() {
    return (TIM7->CNT + timer_micros_counter);
}

uint32_t YUNEECTimer::millis() {
    return timer_millis_counter;
}

/* Delay for the given number of microseconds */
void YUNEECTimer::delay_microseconds(uint16_t us)
{
	TIM14->SR &= (uint16_t)~TIM_FLAG_Update;
	TIM14->ARR = us;

	while (!((TIM14->SR) & (uint16_t)0x0001));

	TIM14->ARR = 0;

}

void YUNEECTimer::attachInterrupt(voidFuncPtr callback, voidFuncPtr failsafe) {
	timer6_callback = callback;
	timer7_failsafe = failsafe;
}

#endif
