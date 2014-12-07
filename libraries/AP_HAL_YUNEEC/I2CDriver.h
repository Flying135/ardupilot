
#ifndef __AP_HAL_YUNEEC_I2CDRIVER_H__
#define __AP_HAL_YUNEEC_I2CDRIVER_H__

#include <AP_HAL_YUNEEC.h>

#define USE_I2C_CLOCKOUT 0

class YUNEEC::YUNEECI2CDriver : public AP_HAL::I2CDriver {
public:
    YUNEECI2CDriver(I2C_TypeDef* i2c, const uint32_t i2c_clk,
    				GPIO_TypeDef* scl_port, GPIO_TypeDef* sda_port,
    				const uint32_t scl_clk, const uint32_t sda_clk,
					const uint16_t scl_bit, const uint16_t sda_bit,
					const uint8_t scl_pinSource, const uint8_t sda_pinSource,
					AP_HAL::Semaphore* semaphore);

    void begin();
    void end();
    void setTimeout(uint16_t ms);
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);
    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
	// Specify info of I2C port
	struct I2C_Info {
		I2C_TypeDef* 	i2c;
		const uint32_t	i2c_clk;
		GPIO_TypeDef*	scl_port;
		GPIO_TypeDef*	sda_port;
		const uint32_t	scl_clk;
		const uint32_t	sda_clk;
		const uint16_t 	scl_bit;
		const uint16_t 	sda_bit;
		const uint8_t 	scl_pinSource;
		const uint8_t 	sda_pinSource;
	} _i2c_info;

	uint32_t _last_i2c_speed;

    AP_HAL::Semaphore* _semaphore;
    uint8_t _lockup_count;
    bool _ignore_errors;
    uint32_t _timeout;
    static uint32_t _start_time;

    void _i2c_bus_reset(struct I2C_Info &i2c_info);
    static void _i2c_config(struct I2C_Info &i2c_info, uint32_t i2c_clock_speed);
};

//----------------------------------------------------------------------------
// I2C Instance
//----------------------------------------------------------------------------
#define I2C1_CLK				RCC_APB1Periph_I2C1
#define I2C1_SCL_PORT			GPIOB
#define I2C1_SDA_PORT			GPIOB
#define I2C1_GPIO_SCL_CLK		RCC_AHB1Periph_GPIOB
#define I2C1_GPIO_SDA_CLK		RCC_AHB1Periph_GPIOB
#define I2C1_SCL_BIT			GPIO_Pin_8
#define I2C1_SDA_BIT			GPIO_Pin_9
#define I2C1_SCL_PINSOURCE 		GPIO_PinSource8
#define I2C1_SDA_PINSOURCE 		GPIO_PinSource9

#define I2C2_CLK				RCC_APB1Periph_I2C2
#define I2C2_SCL_PORT			GPIOB
#define I2C2_SDA_PORT			GPIOB
#define I2C2_GPIO_SCL_CLK		RCC_AHB1Periph_GPIOB
#define I2C2_GPIO_SDA_CLK		RCC_AHB1Periph_GPIOB
#define I2C2_SCL_BIT			GPIO_Pin_10
#define I2C2_SDA_BIT			GPIO_Pin_11
#define I2C2_SCL_PINSOURCE 		GPIO_PinSource10
#define I2C2_SDA_PINSOURCE 		GPIO_PinSource11

#define I2C3_CLK				RCC_APB1Periph_I2C3
#define I2C3_SCL_PORT			GPIOA
#define I2C3_SDA_PORT			GPIOC
#define I2C3_GPIO_SCL_CLK		RCC_AHB1Periph_GPIOA
#define I2C3_GPIO_SDA_CLK		RCC_AHB1Periph_GPIOC
#define I2C3_SCL_BIT			GPIO_Pin_8
#define I2C3_SDA_BIT			GPIO_Pin_9
#define I2C3_SCL_PINSOURCE 		GPIO_PinSource8
#define I2C3_SDA_PINSOURCE 		GPIO_PinSource9

#define YUNEECI2CDriverInstance(I2Cx, semaphore)                            					\
YUNEECI2CDriver I2Cx##Driver((I2C_TypeDef*) I2Cx, I2Cx##_CLK,									\
							 (GPIO_TypeDef*) I2Cx##_SCL_PORT, (GPIO_TypeDef*) I2Cx##_SDA_PORT,	\
							 I2Cx##_GPIO_SCL_CLK, I2Cx##_GPIO_SDA_CLK,							\
							 I2Cx##_SCL_BIT, I2Cx##_SDA_BIT,									\
							 I2Cx##_SCL_PINSOURCE, I2Cx##_SDA_PINSOURCE,						\
							 semaphore)

#endif // __AP_HAL_YUNEEC_I2CDRIVER_H__
