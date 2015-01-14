#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "SPIDriver.h"

using namespace YUNEEC;

#define DataFlash_SPI                           SPI2
#define DataFlash_SPI_CLK                       RCC_APB1Periph_SPI2

#define DataFlash_SPI_SCK_PIN                   GPIO_Pin_13
#define DataFlash_SPI_SCK_GPIO_PORT             GPIOB
#define DataFlash_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define DataFlash_SPI_SCK_SOURCE                GPIO_PinSource13
#define DataFlash_SPI_SCK_AF                    GPIO_AF_SPI2

#define DataFlash_SPI_MISO_PIN                  GPIO_Pin_14
#define DataFlash_SPI_MISO_GPIO_PORT            GPIOB
#define DataFlash_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define DataFlash_SPI_MISO_SOURCE               GPIO_PinSource14
#define DataFlash_SPI_MISO_AF                   GPIO_AF_SPI2

#define DataFlash_SPI_MOSI_PIN                  GPIO_Pin_15
#define DataFlash_SPI_MOSI_GPIO_PORT            GPIOB
#define DataFlash_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define DataFlash_SPI_MOSI_SOURCE               GPIO_PinSource15
#define DataFlash_SPI_MOSI_AF                   GPIO_AF_SPI2

#define DataFlash_CS_PIN                        GPIO_Pin_12
#define DataFlash_CS_GPIO_PORT                  GPIOB
#define DataFlash_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOB

void YUNEECSPIDeviceDriver::init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* enable clock */
	RCC->APB1ENR |= DataFlash_SPI_CLK;
    RCC->AHB1ENR |= (DataFlash_SPI_SCK_GPIO_CLK | DataFlash_SPI_MISO_GPIO_CLK | DataFlash_SPI_MOSI_GPIO_CLK | DataFlash_CS_GPIO_CLK);

    /* Connect SPI pins to AF5 */
    GPIO_PinAFConfig(DataFlash_SPI_SCK_GPIO_PORT, DataFlash_SPI_SCK_SOURCE, DataFlash_SPI_SCK_AF);
    GPIO_PinAFConfig(DataFlash_SPI_MISO_GPIO_PORT, DataFlash_SPI_MISO_SOURCE, DataFlash_SPI_MISO_AF);
    GPIO_PinAFConfig(DataFlash_SPI_MOSI_GPIO_PORT, DataFlash_SPI_MOSI_SOURCE, DataFlash_SPI_MOSI_AF);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    /* SPI SCK pin configuration */
    GPIO_InitStructure.GPIO_Pin = DataFlash_SPI_SCK_PIN;
    GPIO_Init(DataFlash_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /* SPI MOSI pin configuration */
    GPIO_InitStructure.GPIO_Pin =  DataFlash_SPI_MOSI_PIN;
    GPIO_Init(DataFlash_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /* SPI MISO pin configuration */
    GPIO_InitStructure.GPIO_Pin =  DataFlash_SPI_MISO_PIN;
    GPIO_Init(DataFlash_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /* Configure sFLASH Card CS pin in output pushpull mode ********************/
    GPIO_InitStructure.GPIO_Pin = DataFlash_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DataFlash_CS_GPIO_PORT, &GPIO_InitStructure);

    /* Chip Select High */
    DataFlash_CS_GPIO_PORT->BSRRL = DataFlash_CS_PIN;

    /* SPI configuration */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(DataFlash_SPI, &SPI_InitStructure);

    /* Enable the DataFlash_SPI  */
    DataFlash_SPI->CR1 |= SPI_CR1_SPE;
    _last_speed = SPI_SPEED_LOW;
}

void YUNEECSPIDeviceDriver::set_bus_speed(enum bus_speed speed) {
	if (_last_speed == speed)
		return;
	uint16_t tmpreg = DataFlash_SPI->CR1 & (uint16_t)0xFFC7;
	if (speed == SPI_SPEED_LOW) {
		tmpreg |= SPI_BaudRatePrescaler_128;
	} else {
		tmpreg |= SPI_BaudRatePrescaler_2;
	}
	DataFlash_SPI->CR1 = tmpreg;
	_last_speed = speed;
}

AP_HAL::Semaphore* YUNEECSPIDeviceDriver::get_semaphore() {
    return &_semaphore;
}

void YUNEECSPIDeviceDriver::transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) {
    cs_assert();
    if (rx == NULL) {
        transfer(tx, len);
    } else {
        for (uint16_t i = 0; i < len; i++) {
            rx[i] = transfer(tx[i]);
        }
    }
    cs_release();
}


void YUNEECSPIDeviceDriver::cs_assert() {
    /* Chip Select Low */
    DataFlash_CS_GPIO_PORT->BSRRH = DataFlash_CS_PIN;
}

void YUNEECSPIDeviceDriver::cs_release() {
    /* Chip Select High */
    DataFlash_CS_GPIO_PORT->BSRRL = DataFlash_CS_PIN;
}

uint8_t YUNEECSPIDeviceDriver::transfer (uint8_t data) {
    /* Wait for empty transmit buffer */
    while (!(DataFlash_SPI->SR & SPI_I2S_FLAG_TXE)) ;

    /* Put data into buffer, sends the data */
    DataFlash_SPI->DR = (uint16_t)data;

    /* Wait for data to be received */
    while (!(DataFlash_SPI->SR & SPI_I2S_FLAG_RXNE)) ;

    /* Get and return received data from buffer */
    return DataFlash_SPI->DR;
}

void YUNEECSPIDeviceDriver::transfer (const uint8_t *data, uint16_t len) {
	while (len--) {
		/* Wait for empty transmit buffer */
		while (!(DataFlash_SPI->SR & SPI_I2S_FLAG_TXE)) ;

		/* Put data into buffer, sends the data */
		DataFlash_SPI->DR = (uint16_t)*data;
		data++;

		/* Wait for data to be received */
		while (!(DataFlash_SPI->SR & SPI_I2S_FLAG_RXNE)) ;

		/* Get and return received data from buffer */
		(void)DataFlash_SPI->DR;
	}

}

void YUNEECSPIDeviceManager::init(void *) {
	_dataflash = new YUNEECSPIDeviceDriver();
	_dataflash->init();
}

AP_HAL::SPIDeviceDriver* YUNEECSPIDeviceManager::device(enum AP_HAL::SPIDevice d) {
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return _dataflash;
        default:
            return NULL;
    };
}

#endif
