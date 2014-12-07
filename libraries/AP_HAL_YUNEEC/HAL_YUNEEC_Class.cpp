
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "HAL_YUNEEC_Class.h"
#include "AP_HAL_YUNEEC_Private.h"

#include <stm32f4xx.h>

using namespace YUNEEC;

YUNEECUARTDriverHandler(USART1, 0);
YUNEECUARTDriverHandler(USART2, 1);
YUNEECUARTDriverHandler(USART3, 2);
YUNEECUARTDriverHandler(UART4,  3);
YUNEECUARTDriverHandler(UART5,  4);
YUNEECUARTDriverHandler(USART6, 5);

static YUNEECUARTDriverInstance(USART1, 0);
static YUNEECUARTDriverInstance(USART2, 1);
static YUNEECUARTDriverInstance(USART3, 2);
static YUNEECUARTDriverInstance(UART4,  3);
static YUNEECUARTDriverInstance(UART5,  4);
static YUNEECUARTDriverInstance(USART6, 5);

static YUNEECSemaphore  		i2c1Semaphore;
static YUNEECSemaphore  		i2c2Semaphore;
static YUNEECI2CDriverInstance(I2C1, &i2c1Semaphore);
static YUNEECI2CDriverInstance(I2C2, &i2c2Semaphore);

static YUNEECSPIDeviceManager 	spiDeviceManager;
static YUNEECAnalogIn 			analogIn;
static YUNEECStorage 			storageDriver;
static YUNEECGPIO 				gpioDriver;
static YUNEECRCInputST24 		rcinDriver;
static YUNEECRCOutput 			rcoutDriver;
static YUNEECScheduler 			schedulerInstance;
static YUNEECUtil 				utilInstance;

HAL_YUNEEC::HAL_YUNEEC() :
    AP_HAL::HAL(
        &UART4Driver,    /* UARTA: Console/Telemetry1 */
        &USART1Driver,	 /* UARTB: 1st GPS */
        &USART3Driver,   /* UARTC: ESCBUS */
        &USART6Driver,   /* UARTD: Telemetry2/ST24/DSM */
        &USART2Driver,   /* UARTE: 2nd GPS */
        &I2C1Driver,
        &I2C2Driver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &UART4Driver,	/* Console */
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
//    rcin->init(NULL);
    rcout->init(NULL);
    analogin->init(NULL);
    i2c->begin();
    i2c->setTimeout(2);
    i2c2->begin();
    i2c2->setTimeout(2);
	spi->init(NULL);
    storage->init(NULL);
}

const HAL_YUNEEC AP_HAL_YUNEEC;

#endif
