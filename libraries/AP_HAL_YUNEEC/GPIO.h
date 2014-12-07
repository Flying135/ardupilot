/*
  YUNEEC port by Maelok
 */

#ifndef __AP_HAL_YUNEEC_GPIO_H__
#define __AP_HAL_YUNEEC_GPIO_H__

#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

#define HAL_GPIO_USB_MUX_PIN	PA9

#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ANALOG 2
#define HAL_GPIO_AF		3

class YUNEEC::YUNEECGPIO : public AP_HAL::GPIO {
public:
    YUNEECGPIO();
    void    init();
    void    pinMode(uint8_t pin, uint8_t output);
    int8_t  analogPinToDigitalPin(uint8_t pin);
    uint8_t read(uint8_t pin);
    void    write(uint8_t pin, uint8_t value);
    void    toggle(uint8_t pin);

    /* Alternative interface: */
    AP_HAL::DigitalSource* channel(uint16_t n);

    /* Interrupt interface: */
    bool    attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
            uint8_t mode);

    /* return true if USB cable is connected */
    bool    usb_connected(void);

};

class YUNEEC::YUNEECDigitalSource : public AP_HAL::DigitalSource {
public:
    YUNEECDigitalSource(GPIO_TypeDef* port, uint16_t bit) : _port(port), _bit(bit) {}
    void    mode(uint8_t output);
    uint8_t read();
    void    write(uint8_t value); 
    void    toggle();

private:
    GPIO_TypeDef* _port;
    uint16_t _bit;
};

#endif // __AP_HAL_YUNEEC_GPIO_H__
