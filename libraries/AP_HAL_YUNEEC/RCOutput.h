
#ifndef __AP_HAL_YUNEEC_RCOUTPUT_H__
#define __AP_HAL_YUNEEC_RCOUTPUT_H__

#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

#define CH_1 	0
#define CH_2 	1
#define CH_3 	2
#define CH_4 	3
#define CH_5 	4
#define CH_6 	5
#define CH_7 	6
#define CH_8 	7

#define	_BV(_N)		(1<<_N)

class YUNEEC::YUNEECRCOutputPWM : public AP_HAL::RCOutput {
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);
};

#define ESC_CHANNEL_NUM		6
#define MAX_CHANNEL_PORT	8
#define ESC_RXCH_S1_PIN		PC15
#define ESC_RXCH_S2_PIN		PD8
#define ESC_RXCH_S3_PIN		PB2

class YUNEEC::YUNEECRCOutputESCBUS : public AP_HAL::RCOutput {
public:
    void     init(void* machtnichts);
    void     set_freq(uint32_t chmask, uint16_t freq_hz);
    uint16_t get_freq(uint8_t ch);
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);
    void     write(uint8_t ch, uint16_t period_us);
    void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    uint16_t read(uint8_t ch);
    void     read(uint16_t* period_us, uint8_t len);

//    void 	rx_enable(uint8_t ch);

private:
    static AP_HAL::UARTDriver* _escbus_uart;
    static int8_t _rx_enable_table[MAX_CHANNEL_PORT];
    static uint8_t _update_period;
    static uint8_t _channel_enable_mask;
    static int16_t _channel_value[ESC_CHANNEL_NUM];
    static uint32_t _last_update_time;

    uint16_t _constrain_period(uint8_t ch, uint16_t p);
    void _uart_init(AP_HAL::UARTDriver* uartX);
    void _escbus_init(void);
    void _escbus_update(void);
};

#endif // __AP_HAL_YUNEEC_RCOUTPUT_H__
