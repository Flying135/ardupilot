
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "HAL_YUNEEC_Class.h"
#include "AP_HAL_YUNEEC_Private.h"

#include <stm32f37x.h>

using namespace YUNEEC;

YUNEECUARTDriverHandler(USART1, 0);
YUNEECUARTDriverHandler(USART2, 1);
YUNEECUARTDriverHandler(USART3, 2);

YUNEECUARTDriverInstance(USART1, 0);
YUNEECUARTDriverInstance(USART2, 1);
YUNEECUARTDriverInstance(USART3, 2);

static YUNEECSemaphore  		i2c1Semaphore;
static YUNEECSemaphore  		i2c2Semaphore;
static YUNEECI2CDriverInstance(I2C1, &i2c1Semaphore);
static YUNEECI2CDriverInstance(I2C2, &i2c2Semaphore);

static YUNEECSPIDeviceManager 	spiDeviceManager;
static YUNEECAnalogIn 			analogIn;
static YUNEECStorage 			storageDriver;
static YUNEECGPIO 				gpioDriver;
static YUNEECRCInputST24 		rcinDriver;
static YUNEECRCOutputESCBUS		rcoutDriver;
static YUNEECScheduler 			schedulerInstance;
static YUNEECUtil 				utilInstance;

HAL_YUNEEC::HAL_YUNEEC() :
    AP_HAL::HAL(
        &USART1Driver,	/* console/GCS */
        &USART2Driver,	/* 1st GPS */
        &USART3Driver,	/* Tx-ESCbus/Rx-ST24 */
        NULL,           /* telem2: no uartD */
        NULL,           /* 2nd GPS: no uartE */
        &I2C1Driver,
        &I2C2Driver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &USART1Driver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_YUNEEC::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);
    console->begin(115200);
    rcin->init(NULL);
    rcout->init(NULL);
    analogin->init(NULL);
    i2c->begin();
    i2c->setTimeout(100);
    i2c2->begin();
    i2c2->setTimeout(100);
    storage->init(NULL);
}

const HAL_YUNEEC AP_HAL_YUNEEC;

#endif
