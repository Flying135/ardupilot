
#ifndef __AP_HAL_YUNEEC_SPIDRIVER_H__
#define __AP_HAL_YUNEEC_SPIDRIVER_H__

#include <AP_HAL_YUNEEC.h>
#include "Semaphores.h"

class YUNEEC::YUNEECSPIDeviceDriver : public AP_HAL::SPIDeviceDriver {
public:
    void init();
    AP_HAL::Semaphore* get_semaphore();
    void transaction(const uint8_t *tx, uint8_t *rx, uint16_t len);

    void cs_assert();
    void cs_release();
    uint8_t transfer (uint8_t data);
    void transfer (const uint8_t *data, uint16_t len);
private:
    YUNEECSemaphore _semaphore;
};

class YUNEEC::YUNEECSPIDeviceManager : public AP_HAL::SPIDeviceManager {
public:
    void init(void *);
    AP_HAL::SPIDeviceDriver* device(enum AP_HAL::SPIDevice d);
private:
    YUNEECSPIDeviceDriver* _dataflash;
};

#endif // __AP_HAL_YUNEEC_SPIDRIVER_H__
