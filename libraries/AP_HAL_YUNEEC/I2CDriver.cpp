#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "I2CDriver.h"
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_i2c.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define GPIO_AF_I2C          	((uint8_t)0x04)

#define I2C_SR1_FLAG_BTF	 	((uint16_t)0x0004)
#define I2C_SR1_FLAG_ADDR	 	((uint16_t)0x0002)
#define I2C_SR1_FLAG_RXNE		((uint16_t)0x0040)
#define I2C_CR1_FLAG_STOP	 	((uint16_t)0x0200)

#define __I2C_GET_SR1_FLAG(i2c, flag)						(((i2c)->SR1) & flag)
#define __I2C_GET_CR1_FLAG(i2c, flag)						(((i2c)->CR1) & flag)

#define __I2C_TIMEOUT(cmd, start, timeout, isTimeout)		do {\
																if (timeout != 0) {\
																	isTimeout = ((hal.scheduler->micros() - start) >= timeout)?true:false;\
																	if (isTimeout == true)\
																		goto error;\
																}\
															} while ( (cmd) == 0 )

#define I2C_FAST_SPEED 400000
#define I2C_STANDARD_SPEED 100000

uint32_t YUNEECI2CDriver::_start_time = 0;

YUNEECI2CDriver::YUNEECI2CDriver(I2C_TypeDef* i2c, const uint32_t i2c_clk,
		GPIO_TypeDef* scl_port, GPIO_TypeDef* sda_port,
		const uint32_t scl_clk, const uint32_t sda_clk,
		const uint16_t scl_bit, const uint16_t sda_bit,
		const uint8_t scl_pinSource, const uint8_t sda_pinSource,
		AP_HAL::Semaphore* semaphore) :
	_i2c_info{i2c, i2c_clk, scl_port, sda_port, scl_clk, sda_clk, scl_bit, sda_bit, scl_pinSource, sda_pinSource},
	_last_i2c_speed(I2C_FAST_SPEED), _semaphore(semaphore), _lockup_count(0), _ignore_errors(false), _timeout(0)
{}

void YUNEECI2CDriver::begin() {
	_i2c_config(_i2c_info, I2C_FAST_SPEED);
}

void YUNEECI2CDriver::end() {
	/* Disable I2C */
	_i2c_info.i2c->CR1 &= ~I2C_CR1_PE;
	/* Reset I2C */
    RCC->APB1RSTR &= ~_i2c_info.i2c_clk;
    RCC->APB1RSTR |= _i2c_info.i2c_clk;
    /* Disable I2C clock */
    RCC->APB1ENR &= ~_i2c_info.i2c_clk;
    /* Disable I2Cx SCL and SDA Pin Clock */
    RCC->AHB1ENR &= ~(_i2c_info.scl_clk | _i2c_info.sda_clk);
}

void YUNEECI2CDriver::setTimeout(uint16_t ms) {
	_timeout  = (uint32_t) 1000 * ms;
}

void YUNEECI2CDriver::setHighSpeed(bool active) {
	uint32_t clock_speed = 0;
	if (active)
		clock_speed = I2C_FAST_SPEED;
	else
		clock_speed = I2C_STANDARD_SPEED;

	if (_last_i2c_speed != clock_speed) {
		_last_i2c_speed = clock_speed;
		_i2c_config(_i2c_info, clock_speed);
	}
}

uint8_t YUNEECI2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data) {
	bool isTimeout = false;
	_start_time = hal.scheduler->micros();

    /* Generate a START condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_START;
	/* Test on I2Cx EV5 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_MODE_SELECT), _start_time, _timeout, isTimeout);

	/* Send I2Cx slave Address for write */
	_i2c_info.i2c->DR = (addr << 1) & 0xFE;
	/* Test on I2Cx EV6 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), _start_time, _timeout, isTimeout);

	/* Transmit data */
	while (len > 1) {
		_i2c_info.i2c->DR = *data;
		__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTING), _start_time, _timeout, isTimeout);
		len--;
		data++;
	}

	/* Transmit last one data */
	_i2c_info.i2c->DR = *data;
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED), _start_time, _timeout, isTimeout);

	/* Generate a STOP condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_STOP;

    /* Wait to make sure that STOP control bit has been cleared */
	__I2C_TIMEOUT(!__I2C_GET_CR1_FLAG(_i2c_info.i2c, I2C_CR1_FLAG_STOP), _start_time, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
	return 	writeRegisters(addr, reg, 1, &val);
}

uint8_t YUNEECI2CDriver::writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	bool isTimeout = false;
	_start_time = hal.scheduler->micros();

    /* Generate a START condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_START;
	/* Test on I2Cx EV5 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_MODE_SELECT), _start_time, _timeout, isTimeout);

	/* Send I2Cx slave Address for write */
	_i2c_info.i2c->DR = (addr << 1) & 0xFE;
	/* Test on I2Cx EV6 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), _start_time, _timeout, isTimeout);

	/* Send I2Cx device register Address */
	_i2c_info.i2c->DR = reg;
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTING), _start_time, _timeout, isTimeout);

	/* Transmit data */
	while (len > 1) {
		_i2c_info.i2c->DR = *data;
		__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTING), _start_time, _timeout, isTimeout);
		len--;
		data++;
	}

	/* Transmit last one data */
	_i2c_info.i2c->DR = *data;
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_TRANSMITTED), _start_time, _timeout, isTimeout);

	/* Generate a STOP condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_STOP;

    /* Wait to make sure that STOP control bit has been cleared */
	__I2C_TIMEOUT(!__I2C_GET_CR1_FLAG(_i2c_info.i2c, I2C_CR1_FLAG_STOP), _start_time, _timeout, isTimeout);

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data) {
	if (len == 0)
		return 0;

	bool isTimeout = false;

	memset(data, 0, len);

	_start_time = hal.scheduler->micros();

    /* Generate a START condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_START;
	/* Test on I2Cx EV5 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_MODE_SELECT), _start_time, _timeout, isTimeout);

	/* Send I2Cx slave Address for read */
	_i2c_info.i2c->DR = (addr << 1) | 0x01;

	if (len < 2) {
		/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_ADDR), _start_time, _timeout, isTimeout);

		/* Receive last one data */
		/*!< Disable Acknowledgement */
		_i2c_info.i2c->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);

		/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
		(void)_i2c_info.i2c->SR2;

		/* Send STOP condition */
		_i2c_info.i2c->CR1 |= I2C_CR1_STOP;

		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_RXNE), _start_time, _timeout, isTimeout);
		*data = _i2c_info.i2c->DR;

	} else {
		/* Test on I2Cx EV6 and clear it or time out */
		__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), _start_time, _timeout, isTimeout);

		/* Transmit data */
		while (len > 1) {
			__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_RECEIVED), _start_time, _timeout, isTimeout);
			*data = _i2c_info.i2c->DR;
			len--;
			data++;
		}

		/* Receive last one data */
		/*!< Disable Acknowledgement */
		uint16_t reg = _i2c_info.i2c->CR1;
		reg &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
		/* Send STOP condition */
		reg |= I2C_CR1_STOP;
		_i2c_info.i2c->CR1 = reg;

		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_RXNE), _start_time, _timeout, isTimeout);
		*data = _i2c_info.i2c->DR;
	}

    /* Wait to make sure that STOP control bit has been cleared */
	__I2C_TIMEOUT(!__I2C_GET_CR1_FLAG(_i2c_info.i2c, I2C_CR1_FLAG_STOP), _start_time, _timeout, isTimeout);
    /* Re-Enable ACK to be ready for another reception */
	_i2c_info.i2c->CR1 |= I2C_CR1_ACK;

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data) {
    return readRegisters(addr, reg, 1, data);
}

uint8_t YUNEECI2CDriver::readRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data) {
	if (len == 0)
		return 0;

	bool isTimeout = false;

	memset(data, 0, len);

	_start_time = hal.scheduler->micros();

    /* Enable the acknowledgement and Generate a START condition */
	_i2c_info.i2c->CR1 |= I2C_CR1_START;
	/* Test on I2Cx EV5 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_MODE_SELECT), _start_time, _timeout, isTimeout);

	/* Send I2Cx slave Address for write */
	_i2c_info.i2c->DR = (addr << 1) & 0xFE;
	/* Test on I2Cx EV6 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), _start_time, _timeout, isTimeout);

	/* Send I2Cx device register Address */
	_i2c_info.i2c->DR = reg;
	__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_BTF), _start_time, _timeout, isTimeout);

    /* Generate a START condition a second time */
	_i2c_info.i2c->CR1 |= I2C_CR1_START;
	/* Test on I2Cx EV5 and clear it or time out */
	__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_MODE_SELECT), _start_time, _timeout, isTimeout);

	/* Send I2Cx slave Address for read */
	_i2c_info.i2c->DR = (addr << 1) | 0x01;

	if (len < 2) {
		/* Wait on ADDR flag to be set (ADDR is still not cleared at this level */
		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_ADDR), _start_time, _timeout, isTimeout);

		/* Receive last one data */
		/*!< Disable Acknowledgement */
		_i2c_info.i2c->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);

		/* Clear ADDR register by reading SR1 then SR2 register (SR1 has already been read) */
		(void)_i2c_info.i2c->SR2;

		/* Send STOP condition */
		_i2c_info.i2c->CR1 |= I2C_CR1_STOP;

		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_RXNE), _start_time, _timeout, isTimeout);
		*data = _i2c_info.i2c->DR;

	} else {
		/* Test on I2Cx EV6 and clear it or time out */
		__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), _start_time, _timeout, isTimeout);

		/* Transmit data */
		while (len > 1) {
			__I2C_TIMEOUT(I2C_CheckEvent(_i2c_info.i2c, I2C_EVENT_MASTER_BYTE_RECEIVED), _start_time, _timeout, isTimeout);
			*data = _i2c_info.i2c->DR;
			len--;
			data++;
		}

		/* Receive last one data */
		/*!< Disable Acknowledgement */
		_i2c_info.i2c->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);
		/* Send STOP condition */
		_i2c_info.i2c->CR1 |= I2C_CR1_STOP;

		__I2C_TIMEOUT(__I2C_GET_SR1_FLAG(_i2c_info.i2c, I2C_SR1_FLAG_RXNE), _start_time, _timeout, isTimeout);
		*data = _i2c_info.i2c->DR;

	}

    /* Wait to make sure that STOP control bit has been cleared */
	__I2C_TIMEOUT(!__I2C_GET_CR1_FLAG(_i2c_info.i2c, I2C_CR1_FLAG_STOP), _start_time, _timeout, isTimeout);
    /* Re-Enable ACK to be ready for another reception */
	_i2c_info.i2c->CR1 |= I2C_CR1_ACK;

    return 0;

error:
	// transmission failed
	if (!_ignore_errors)
		_lockup_count++;

	_i2c_bus_reset(_i2c_info);
	return 1;
}

uint8_t YUNEECI2CDriver::lockup_count() {
	return _lockup_count;
}


void YUNEECI2CDriver::_i2c_bus_reset(struct I2C_Info &i2c_info) {
	// xxx: This clockout function is dangerous for MS56XX, it will lock itself
	// if just START and STOP condition detected
#if USE_I2C_CLOCKOUT
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Disable I2C device */
	i2c_info.i2c->CR1 &= ~I2C_CR1_PE;

	/*
	 * Release both lines
	 */
    /* Enable I2Cx SCL and SDA Pin Clock */
    RCC->AHB1ENR |= (i2c_info.scl_clk | i2c_info.sda_clk);

    i2c_info.scl_port->BSRRL |= i2c_info.scl_bit;
    i2c_info.sda_port->BSRRL |= i2c_info.sda_bit;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = i2c_info.scl_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.scl_port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = i2c_info.sda_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.sda_port, &GPIO_InitStructure);

    /*
     * Make sure the bus is free by clocking it until any slaves release the
     * bus.
     */
    for (uint8_t clockout_times = 0; clockout_times < 10; clockout_times++) {
		if (!((i2c_info.sda_port->IDR) & (i2c_info.sda_bit))) {
			/* pull low */
			i2c_info.scl_port->BSRRH = i2c_info.scl_bit;
			hal.scheduler->delay_microseconds(10);

			/* release high again */
			i2c_info.scl_port->BSRRL = i2c_info.scl_bit;
			hal.scheduler->delay_microseconds(10);
		}
		else
			break;
    }
    /* generate start then stop condition */
	i2c_info.sda_port->BSRRH = i2c_info.sda_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.scl_port->BSRRH = i2c_info.scl_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.scl_port->BSRRL = i2c_info.scl_bit;
	hal.scheduler->delay_microseconds(10);
	i2c_info.sda_port->BSRRL = i2c_info.sda_bit;
#endif
	/* Restart I2C device */
	_i2c_config(i2c_info, _last_i2c_speed);
}

void YUNEECI2CDriver::_i2c_config(struct I2C_Info &i2c_info, uint32_t i2c_clock_speed) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

	/* Reset I2C */
    RCC->APB1RSTR |= i2c_info.i2c_clk;
    RCC->APB1RSTR &= ~i2c_info.i2c_clk;

    /* Enable I2Cx SCL and SDA Pin Clock */
    RCC->AHB1ENR |= (i2c_info.scl_clk | i2c_info.sda_clk);

    /* Connect PXx to I2C_SCL */
    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.scl_port, i2c_info.scl_pinSource, GPIO_AF_I2C);
    /* Connect PXx to I2C_SDA */
    GPIO_PinAFConfig((GPIO_TypeDef*)i2c_info.sda_port, i2c_info.sda_pinSource, GPIO_AF_I2C);

    /* make sure both line be high level */
    i2c_info.scl_port->BSRRH |= i2c_info.scl_bit;
    i2c_info.sda_port->BSRRH |= i2c_info.sda_bit;

    /* Set GPIO frequency to 50MHz */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /* Select Alternate function mode */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    /* Select output Open Drain type */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    /* Disable internal Pull-up */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    /* Initialize I2Cx SCL Pin */
    GPIO_InitStructure.GPIO_Pin = i2c_info.scl_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.scl_port, &GPIO_InitStructure);
    /* Initialize I2Cx SDA Pin */
    GPIO_InitStructure.GPIO_Pin = i2c_info.sda_bit;
    GPIO_Init((GPIO_TypeDef*)i2c_info.sda_port, &GPIO_InitStructure);

    /* Enable I2C clock */
    RCC->APB1ENR |= i2c_info.i2c_clk;

	/* Disable I2C and clear all SR register */
    i2c_info.i2c->CR1 &= ~I2C_CR1_PE;
    /* Get the old register value */
    uint16_t tmpreg = i2c_info.i2c->FLTR;
    /* Enable the analog filter */
    tmpreg &= (uint16_t)~((uint16_t)0x001f);
    /* Set I2Cx DNF coefficient */
    tmpreg |= (uint16_t)((uint16_t)0x0F & I2C_FLTR_DNF);
    /* Store the new register value */
    i2c_info.i2c->FLTR = tmpreg;

    /* I2C Struct Initialize */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_ClockSpeed = i2c_clock_speed;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    /* Initiate I2C and enable it */
    I2C_Init((I2C_TypeDef*)i2c_info.i2c, &I2C_InitStructure);

    /* Disable all interrupt */
    i2c_info.i2c->CR2 &= ~(I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
}

#endif
