#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

using namespace YUNEEC;

#define TIM_PERIOD					(uint16_t)(25000 - 1)
// TIM1/9 -> APB2 168MHz
#define TIM1_9_PRESCALER			(uint16_t)(SystemCoreClock / 1000000 - 1)
// TIM4/5 -> APB1 84MHz
#define TIM4_5_PRESCALER			(uint16_t)(SystemCoreClock / 2 / 1000000  - 1)

void YUNEECRCOutputPWM::init(bool escbus_exist) {
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/*
	 * Servo PWM output config
	 */
	/* GPIOs clock enable */
    RCC->AHB1ENR |= (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE);

	/* GPIOs Configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* GPIOA Configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOE Configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect TIM Channels to GPIOs */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

	/* TIMs clock enable */
	RCC->APB1ENR |= RCC_APB1Periph_TIM5;
	RCC->APB2ENR |= RCC_APB2Periph_TIM1;

	// Time base configuration: default 50Hz
	TIM_TimeBaseStructure.TIM_Period 		= TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseStructure.TIM_Prescaler = TIM1_9_PRESCALER;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = TIM4_5_PRESCALER;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse 			= 0;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState 	= TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState 	= TIM_OCIdleState_Reset;
	// PWM Mode configuration: TIM1 Channel1/2/3/4
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	// PWM Mode configuration: TIM5 Channel3/4
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

	// enable timer
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM5->BDTR |= TIM_BDTR_MOE;

	if (!escbus_exist) {
		/*
		 * ESC PWM output config
		 */
		/* GPIOs clock enable */
	    RCC->AHB1ENR |= RCC_AHB1Periph_GPIOD;

		/* GPIOs Configuration */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

		/* GPIOA Configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* GPIOD Configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* GPIOE Configuration */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
		GPIO_Init(GPIOE, &GPIO_InitStructure);

		/* Connect TIM Channels to GPIOs */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	    RCC->APB1ENR |= RCC_APB1Periph_TIM4;
	    RCC->APB2ENR |= RCC_APB2Periph_TIM9;

		TIM_TimeBaseStructure.TIM_Prescaler = TIM4_5_PRESCALER;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = TIM1_9_PRESCALER;
		TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

		// PWM Mode configuration: TIM4 Channel1/2/3/4
		TIM_OC1Init(TIM4, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM4, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
		// PWM Mode configuration: TIM9 Channel1/2
		TIM_OC1Init(TIM9, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM9, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);
		// PWM Mode configuration: TIM5 Channel1/2
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

		// enable timer
	    TIM4->CR1 |= TIM_CR1_CEN;
	    TIM9->CR1 |= TIM_CR1_CEN;
	}

}

void YUNEECRCOutputPWM::set_freq(uint32_t chmask, uint16_t freq_hz) {
	uint32_t period = 1000000 / freq_hz - 1;

    if ((chmask & ( _BV(CH_1) | _BV(CH_2) | _BV(CH_3) | _BV(CH_4))) != 0) {
		TIM4->ARR = period;
    }

    if ((chmask & ( _BV(CH_5) | _BV(CH_6))) != 0) {
    	TIM9->ARR = period;
    }

    if ((chmask & ( _BV(CH_7) | _BV(CH_8) | _BV(CH_9) | _BV(CH_10))) != 0) {
		TIM5->ARR = period;
    }

    if ((chmask & ( _BV(CH_11) | _BV(CH_12) | _BV(CH_13) | _BV(CH_14))) != 0) {
		TIM9->ARR = period;
    }

}

uint16_t YUNEECRCOutputPWM::get_freq(uint8_t ch) {
    uint32_t ARR_Value = 0;

    switch (ch) {
    case CH_1 ... CH_4:
		ARR_Value = TIM4->ARR;
    	break;
    case CH_5 ... CH_6:
		ARR_Value = TIM9->ARR;
    	break;
    case CH_7 ... CH_10:
		ARR_Value = TIM5->ARR;
    	break;
    case CH_11 ... CH_14:
		ARR_Value = TIM1->ARR;
    	break;
    default:
    	return 0;
    }

    return (1000000 / (ARR_Value + 1));
}

void YUNEECRCOutputPWM::enable_ch(uint8_t ch) {
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Enable); break;
        case CH_2: TIM_CCxCmd(TIM4, TIM_Channel_3, TIM_CCx_Enable); break;
        case CH_3: TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Enable); break;
		case CH_4: TIM_CCxCmd(TIM4, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_5: TIM_CCxCmd(TIM9, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_6: TIM_CCxCmd(TIM9, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_7: TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Enable); break;
        case CH_8: TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_9: TIM_CCxCmd(TIM5, TIM_Channel_3, TIM_CCx_Enable); break;
        case CH_10: TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Enable); break;
        case CH_11: TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable); break;
		case CH_12: TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); break;
        case CH_13: TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); break;
        case CH_14: TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); break;
        default:
            break;
    }
}




void YUNEECRCOutputPWM::disable_ch(uint8_t ch) {
    switch (ch) {
        case CH_1: TIM_CCxCmd(TIM4, TIM_Channel_4, TIM_CCx_Disable); break;
        case CH_2: TIM_CCxCmd(TIM4, TIM_Channel_3, TIM_CCx_Disable); break;
        case CH_3: TIM_CCxCmd(TIM4, TIM_Channel_2, TIM_CCx_Disable); break;
		case CH_4: TIM_CCxCmd(TIM4, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_5: TIM_CCxCmd(TIM9, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_6: TIM_CCxCmd(TIM9, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_7: TIM_CCxCmd(TIM5, TIM_Channel_1, TIM_CCx_Disable); break;
        case CH_8: TIM_CCxCmd(TIM5, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_9: TIM_CCxCmd(TIM5, TIM_Channel_3, TIM_CCx_Disable); break;
        case CH_10: TIM_CCxCmd(TIM5, TIM_Channel_4, TIM_CCx_Disable); break;
        case CH_11: TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Disable); break;
		case CH_12: TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable); break;
        case CH_13: TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable); break;
        case CH_14: TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable); break;
        default:
            break;
    }
}

/* constrain pwm to be between min and max pulsewidth. */
static inline uint16_t constrain_period(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void YUNEECRCOutputPWM::write(uint8_t ch, uint16_t period_us) {
    uint16_t pwm = constrain_period(period_us);

    switch (ch) {
        case CH_1: TIM4->CCR4 = pwm; break;
        case CH_2: TIM4->CCR3 = pwm; break;
        case CH_3: TIM4->CCR2 = pwm; break;
		case CH_4: TIM4->CCR1 = pwm; break;
        case CH_5: TIM9->CCR1 = pwm; break;
        case CH_6: TIM9->CCR2 = pwm; break;
        case CH_7: TIM5->CCR1 = (uint32_t)pwm; break;
        case CH_8: TIM5->CCR2 = (uint32_t)pwm; break;
        case CH_9: TIM5->CCR3 = (uint32_t)pwm; break;
        case CH_10: TIM5->CCR4 = (uint32_t)pwm; break;
        case CH_11: TIM1->CCR4 = pwm; break;
		case CH_12: TIM1->CCR3 = pwm; break;
        case CH_13: TIM1->CCR2 = pwm; break;
        case CH_14: TIM1->CCR1 = pwm; break;
        default:
            break;
    }
}

void YUNEECRCOutputPWM::write(uint8_t ch, uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t YUNEECRCOutputPWM::read(uint8_t ch) {
	uint16_t pwm = 0;

    switch (ch) {
        case CH_1: pwm = TIM4->CCR4; break;
        case CH_2: pwm = TIM4->CCR3; break;
        case CH_3: pwm = TIM4->CCR2; break;
		case CH_4: pwm = TIM4->CCR1; break;
        case CH_5: pwm = TIM9->CCR1; break;
        case CH_6: pwm = TIM9->CCR2; break;
        case CH_7: pwm = (uint16_t)TIM5->CCR1; break;
        case CH_8: pwm = (uint16_t)TIM5->CCR2; break;
        case CH_9: pwm = (uint16_t)TIM5->CCR3; break;
        case CH_10: pwm = (uint16_t)TIM5->CCR4; break;
        case CH_11: pwm = TIM1->CCR4; break;
		case CH_12: pwm = TIM1->CCR3; break;
        case CH_13: pwm = TIM1->CCR2; break;
        case CH_14: pwm = TIM1->CCR1; break;
        default:
            break;
    }

    return pwm;
}

void YUNEECRCOutputPWM::read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

#endif
