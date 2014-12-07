#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "AnalogIn.h"
#include "GPIO.h"
#include <stm32f4xx.h>
#include <stm32f4xx_adc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define ADC1_DR_Address				((uint32_t)0x4001204C)
#define TIM8_PERIOD					(uint16_t)(25000 - 1)
#define TIM8_PRESCALER				(uint16_t)(SystemCoreClock / 1000000 - 1)
#define YUNEEC_VOLTAGE_SCALING		(float)3.3f/4095


#define DMA_Stream0_IT_MASK     (uint32_t)(DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | \
                                           DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | \
                                           DMA_LISR_TCIF0)
#define SQR1_L_RESET              ((uint32_t)0xFF0FFFFF)

extern "C"
{
	static voidFuncPtr dma1_callback = NULL;
	void DMA2_Stream0_IRQHandler(void) {
	    	DMA2->LIFCR = DMA_Stream0_IT_MASK;

		    if(dma1_callback != NULL)
		    	dma1_callback();
	}
}

/*
  scaling table between ADC count and actual input voltage, to account
  for voltage dividers on the board.
 */
//static const struct {
//    uint8_t pin;
//    float scaling;
//} pin_scaling[] = {
//    { PC5,   9 * YUNEEC_VOLTAGE_SCALING }, 	// Battary voltage, 9:1 scaling
//    { PB0,  40 * YUNEEC_VOLTAGE_SCALING }		// Battary current, 20:1 scaling
//};

//const uint8_t YUNEECAnalogSource::_num_pin_scaling = sizeof(pin_scaling) / sizeof(pin_scaling[0]);
volatile uint16_t YUNEECAnalogSource::_ADCConvData_Tab[YUNEEC_INPUT_MAX_CHANNELS] = {0};
int YUNEECAnalogSource::_ADCChannels_Tab[YUNEEC_INPUT_MAX_CHANNELS] = {-1, -1, -1};
uint8_t YUNEECAnalogSource::_num_adc_channels = 0;

YUNEECAnalogSource::YUNEECAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _latest(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _stop_pin_high(false),
    _settle_time_ms(0),
    _read_start_time_ms(0),
//    _pin_scaling_id(-1),
    _channel_rank(-1)
{
    set_pin(pin);
}

float YUNEECAnalogSource::read_average() {
    float sum_count = 0;
    float sum = 0;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    __disable_irq();
    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;
    __enable_irq();

    float avg = sum / sum_count;
    _last_average = avg;

    return avg;
}

float YUNEECAnalogSource::read_latest() {
    return _latest;
}

/*
  return scaling from ADC count to Volts
 */
//float YUNEECAnalogSource::_pin_scaler(void) {
//	if (_pin_scaling_id != -1)
//		return pin_scaling[_pin_scaling_id].scaling;
//	else {
//        hal.console->println(PSTR("YUNEEC::YUNEECAnalogIn can't find pin scaling message, you need to check the pin_scaling[] definiton\r\n"));
//		return 0;
//	}
//
//}

/*
  return voltage in Volts
 */
float YUNEECAnalogSource::voltage_average(void) {
    return YUNEEC_VOLTAGE_SCALING * read_average();
}

float YUNEECAnalogSource::voltage_latest(void) {
    return YUNEEC_VOLTAGE_SCALING * read_latest();
}

/*
  return voltage from 0.0 to 3.3V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 3.3V.
 */
float YUNEECAnalogSource::voltage_average_ratiometric(void) {
    return YUNEEC_VOLTAGE_SCALING * read_average();
}

void YUNEECAnalogSource::set_pin(uint8_t pin) {
	// This pin has been set before
    if (pin == _pin)
    	return;

	// Ensure the pin is marked as an INPUT pin
	if (pin == ANALOG_INPUT_NONE && _channel_rank != -1)
		_unregister_adc_channel();
	else if(pin != ANALOG_INPUT_BOARD_VCC) {
		_register_adc_channel(pin);
		// set pin mode to analog input
		hal.gpio->pinMode(pin, HAL_GPIO_ANALOG);
	}

	// Clear data
	__disable_irq();
	_pin = pin;
	_sum = 0;
	_sum_count = 0;
	_last_average = 0;
	_latest = 0;
	__enable_irq();

	// Update ADC1 configuration
	_update_adc1_config();
}

uint16_t YUNEECAnalogSource::_get_conv_data(void) {
	if(_channel_rank != -1)
		return _ADCConvData_Tab[_channel_rank];
	else
		return 0;
}

// Register a adc channel for the given pin
void YUNEECAnalogSource::_register_adc_channel(uint8_t pin) {
	// Find the index in _pin_sacling[]
//    for(uint8_t i = 0; i < _num_pin_scaling; i++) {
//    	if(pin_scaling[i].pin == pin)
//    	    _pin_scaling_id = i;
//    }
	int adc_channel = get_adc_channel(pin);
	if (adc_channel == -1) {
		hal.scheduler->panic("Invalid analog pin specified!\n");
		return; //never reach
	}

    // Add this pin into _ADCChannels_Tab[] for ADC1 configuration
    _ADCChannels_Tab[_num_adc_channels] = adc_channel;
    // The rank for conversion and also for find data in _ADCConvData_Tab[]
    _channel_rank = _num_adc_channels;
    // We have one more adc channel set
    _num_adc_channels++;
}

// Unregister a adc channel for the given pin
void YUNEECAnalogSource::_unregister_adc_channel() {
//	_pin_scaling_id = -1;

	for(int i = _channel_rank; i < (_num_adc_channels - 1); i++)
	    _ADCChannels_Tab[i] = _ADCChannels_Tab[i + 1];

    _num_adc_channels--;
    _channel_rank = -1;
}

void YUNEECAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void YUNEECAnalogSource::set_settle_time(uint16_t settle_time_ms) {
    _settle_time_ms = settle_time_ms;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * __disable_irq()/__enable_irq() to read these variables from outside an interrupt.
 */
void YUNEECAnalogSource::new_sample(uint16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 15) { // YUNEEC has a 12 bit ADC, so can only sum 16 in a uint16_t
        _sum >>= 1;
        _sum_count = 8;
    } else {
        _sum_count++;
    }
}

void YUNEECAnalogSource::_init_adc1(void) {
	ADC_InitTypeDef       	ADC_InitStructure;
	ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
	DMA_InitTypeDef 		DMA_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  		TIM_OCInitStructure;

	// TIM8 clock enable
    RCC->APB2ENR |= RCC_APB2Periph_TIM8;
	// DMA2 clock enable
    RCC->AHB1ENR |= RCC_AHB1Periph_DMA2;
	// ADC1 Periph clock enable
    RCC->APB2ENR |= RCC_APB2Periph_ADC1;

	// Time base configuration: 25ms per interrupt
	TIM_TimeBaseStructure.TIM_Period 		= TIM8_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler 	= TIM8_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode 			= TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState 	= TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse 			= TIM8_PERIOD / 2;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState 	= TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState 	= TIM_OCIdleState_Reset;
	// PWM Mode configuration: TIM8 Channel1
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	// DMA2 Channel1 Config
	// Reset DMA2
	/* Disable the selected DMAy Streamx */
	DMA2_Stream0->CR &= ~((uint32_t)DMA_SxCR_EN);
	/* Reset DMAy Streamx control register */
	DMA2_Stream0->CR  = 0;
	/* Reset DMAy Streamx Number of Data to Transfer register */
	DMA2_Stream0->NDTR = 0;
	/* Reset DMAy Streamx peripheral address register */
	DMA2_Stream0->PAR  = 0;
	/* Reset DMAy Streamx memory 0 address register */
	DMA2_Stream0->M0AR = 0;
	/* Reset DMAy Streamx memory 1 address register */
	DMA2_Stream0->M1AR = 0;
	/* Reset DMAy Streamx FIFO control register */
	DMA2_Stream0->FCR = (uint32_t)0x00000021;


	DMA_InitStructure.DMA_Channel 				= DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr 	= (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_Memory0BaseAddr 		= (uint32_t)_ADCConvData_Tab;
	DMA_InitStructure.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize 			= 0;
	DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode 					= DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority 				= DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	// Clear all INT flag
	DMA2->LIFCR = DMA_Stream0_IT_MASK;
	// DMA2 enable
	DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;
	// DMA2 interrupt enable
	DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_TCIE;

	// Configure two bits for preemption priority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	// Configure and enable ADC1 interrupt
	NVIC_InitStructure.NVIC_IRQChannel 						= DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode 				= ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler 			= ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode 		= ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay 	= ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution 			= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode 			= ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge 	= ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign 			= ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion 		= 0;
	ADC_Init(ADC1, &ADC_InitStructure);

    /* Enable the selected ADC DMA request after last transfer */
    ADC1->CR2 |= (uint32_t)ADC_CR2_DDS;
    /* Enable the selected ADC DMA request */
    ADC1->CR2 |= (uint32_t)ADC_CR2_DMA;

    /* Enable the TIM Main Output */
    TIM8->BDTR |= TIM_BDTR_MOE;
}

void YUNEECAnalogSource::_update_adc1_config() {
    TIM8->CR1 &= (uint16_t)(~TIM_CR1_CEN);
	DMA2_Stream0->CR &= (uint32_t)(~DMA_SxCR_EN);
    ADC1->CR2 &= (uint32_t)(~ADC_CR2_ADON);

    __disable_irq();
	// Set DMA buffer size
	DMA2_Stream0->NDTR = (uint32_t)(_num_adc_channels);
	// Set ADC1 number of regular channel
	uint32_t tmpreg1 = ADC1->SQR1;
	uint8_t tmpreg2 = _num_adc_channels - (uint8_t)1;
	tmpreg1 &= SQR1_L_RESET;
	tmpreg1 |= (uint32_t)tmpreg2 << 20;
	ADC1->SQR1 = tmpreg1;
	__enable_irq();

	for(uint8_t i = 0; i < _num_adc_channels; i++) {
		ADC_RegularChannelConfig(ADC1, _ADCChannels_Tab[i], (i + 1), ADC_SampleTime_3Cycles);
	}

	// Clear all INT flag
	DMA2->LIFCR = DMA_Stream0_IT_MASK;
    ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;
	DMA2_Stream0->CR |= (uint32_t)DMA_SxCR_EN;
	TIM8->CR1 |= TIM_CR1_CEN;
}

YUNEECAnalogSource* YUNEECAnalogIn::_channels[YUNEEC_INPUT_MAX_CHANNELS] = {NULL};
uint8_t YUNEECAnalogIn::_num_channels = 0;
uint8_t YUNEECAnalogIn::_current_stop_pin_i = 0;

void YUNEECAnalogIn::init(void* machtnichts) {
	YUNEECAnalogSource::_init_adc1();
	_attach_interrupt(_dma_event);
}

AP_HAL::AnalogSource* YUNEECAnalogIn::channel(int16_t pin)
{
	if (pin == ANALOG_INPUT_NONE) {
	    hal.console->println_P(PSTR("YUNEECAnalogIn: Register a analog channel without a actual input pin\r\n"));
	}

    if (_num_channels >= YUNEEC_INPUT_MAX_CHANNELS) {
        hal.console->println(PSTR("Error: YUNEEC::YUNEECAnalogIn out of channels\r\n"));
        return NULL;
    }

    YUNEECAnalogSource *ch = new YUNEECAnalogSource(pin);
    _channels[_num_channels++] = ch;
    return ch;
}

/*
  move to the next stop pin
 */
void YUNEECAnalogIn::_next_stop_pin(void)
{
	// Set current stop pin low to stop device
	if(_channels[_current_stop_pin_i]->_stop_pin != ANALOG_INPUT_NONE) {
		hal.gpio->pinMode(_channels[_current_stop_pin_i]->_stop_pin, HAL_GPIO_OUTPUT);
		hal.gpio->write(_channels[_current_stop_pin_i]->_stop_pin, 0);
		_channels[_current_stop_pin_i]->_stop_pin_high = false;
	}

	for(uint8_t i = 1; i <= _num_channels; i++) {
		uint8_t idx = (_current_stop_pin_i + i) % _num_channels;
		if(_channels[idx]->_stop_pin != ANALOG_INPUT_NONE) {
			_current_stop_pin_i = idx;
			hal.gpio->pinMode(_channels[idx]->_stop_pin, HAL_GPIO_OUTPUT);
			hal.gpio->write(_channels[idx]->_stop_pin, 1);
			_channels[_current_stop_pin_i]->_stop_pin_high = true;
			_channels[_current_stop_pin_i]->_read_start_time_ms = hal.scheduler->millis();
			return;
		}
	}
}

void YUNEECAnalogIn::_dma_event() {
    for (uint8_t i = 0; i < _num_channels; i++) {
    	if(_channels[i]->_pin == ANALOG_INPUT_NONE)
    		continue;
    	else {
			if (_channels[i]->_stop_pin == ANALOG_INPUT_NONE) {
				_channels[i]->new_sample(_channels[i]->_get_conv_data());
			}
			else if ( (_channels[i]->_stop_pin_high == true) &&
					( (hal.scheduler->millis() - _channels[i]->_read_start_time_ms) > _channels[i]->_settle_time_ms)) {
				_channels[i]->new_sample(_channels[i]->_get_conv_data());
			}

    	}
    }
	_next_stop_pin();
}

void YUNEECAnalogIn::_attach_interrupt(voidFuncPtr callback) {
	if(callback != NULL)
		dma1_callback = callback;
}

#endif
