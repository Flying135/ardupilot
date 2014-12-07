#ifndef _PINMAP_TYPEDEF_H_
#define _PINMAP_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stm32f4xx.h>

enum GPIOS  {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
			 PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
			 PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
			 PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
			 PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
			 PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15};

static inline GPIO_TypeDef * get_port(uint8_t pin) {
	switch(pin){
		case PA0 ... PA15: return GPIOA;
			break;
		case PB0 ... PB15: return GPIOB;
			break;
		case PC0 ... PC15: return GPIOC;
			break;
		case PD0 ... PD15: return GPIOD;
			break;
		case PE0 ... PE15: return GPIOE;
			break;
		case PF0 ... PF15: return GPIOF;
			break;
		default: return NULL;
			break;
	}
}

static inline uint16_t get_bit(uint8_t pin) {
	return (uint16_t)(1 << (pin % 16));
}

static inline int get_adc_channel(uint8_t pin) {
	switch(pin){
	case PA0: return ADC_Channel_0;
	case PA1: return ADC_Channel_1;
	case PA2: return ADC_Channel_2;
	case PA3: return ADC_Channel_3;
	case PA4: return ADC_Channel_4;
	case PA5: return ADC_Channel_5;
	case PA6: return ADC_Channel_6;
	case PA7: return ADC_Channel_7;
	case PB0: return ADC_Channel_8;
	case PB1: return ADC_Channel_9;
	case PC0: return ADC_Channel_10;
	case PC1: return ADC_Channel_11;
	case PC2: return ADC_Channel_12;
	case PC3: return ADC_Channel_13;
	case PC4: return ADC_Channel_14;
	case PC5: return ADC_Channel_15;
	default: return -1;
	}
}
#ifdef __cplusplus
} // extern "C"
#endif

#endif
