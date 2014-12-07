/*
  YUNEEC port by Maelok
 */

#ifndef __AP_HAL_YUNEEC_UARTDRIVER_H__
#define __AP_HAL_YUNEEC_UARTDRIVER_H__

#include <AP_HAL_YUNEEC.h>
#include <AP_Common.h>

class YUNEEC::YUNEECUARTDriver : public AP_HAL::UARTDriver {
public:
    YUNEECUARTDriver(
    		const uint8_t portNumber,
    		USART_TypeDef* usart, IRQn_Type usart_irqn,
    		GPIO_TypeDef* tx_port, GPIO_TypeDef* rx_port,
    		const uint16_t tx_bit, const uint16_t rx_bit,
    		const uint8_t tx_pinSource, const uint8_t rx_pinSource,
    		const uint8_t gpio_af, const uint32_t usart_clk,
    		const uint32_t tx_clk, const uint32_t rx_clk);
    /* YUNEEC implementations of UARTDriver virtual methods */
    void begin(uint32_t baud) { begin(baud, 0, 0); };
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* YUNEEC implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    int16_t read();

    /* YUNEEC implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);

	/// Transmit/Receive buffer descriptor.
	///
	/// Public so the interrupt handlers can see it
	typedef struct {
		volatile uint16_t head, tail;	///< head and tail pointers
		uint16_t mask;					///< buffer size mask for pointer wrap
		uint8_t *bytes;					///< pointer to allocated buffer
	} Buffer;

private:
	// Specify info of USART port
	struct USART_Info{
		USART_TypeDef*	usart;
		IRQn_Type		usart_irqn;
		GPIO_TypeDef*	tx_port;
		GPIO_TypeDef*	rx_port;
		const uint16_t 	tx_bit;
		const uint16_t 	rx_bit;
		const uint8_t 	tx_pinSource;
		const uint8_t 	rx_pinSource;
		const uint8_t 	gpio_af;
		const uint32_t	usart_clk;
		const uint32_t	tx_clk;
		const uint32_t 	rx_clk;
		uint32_t		baudrate;
	} _usart_info;

    // Instance Variables
    bool 				_initialized;
	bool 				_open;
	// ring buffers
	volatile Buffer	*const 	_rxBuffer;
	volatile Buffer	*const 	_txBuffer;

	// whether writes to the port should block waiting
	// for enough space to appear
	bool				_nonblocking_writes;

	// Configure port as USART with interrupt enabled
	static void _configPort(const struct USART_Info &usart_info);

	// Allocates a buffer of the given size
	//
	// @param	buffer		The buffer descriptor for which the buffer will
	//						will be allocated.
	// @param	size		The desired buffer size.
	// @returns			True if the buffer was allocated successfully.
	//
	static bool _allocBuffer(volatile Buffer *buffer, uint16_t size);

	// Frees the allocated buffer in a descriptor
	//
	// @param	buffer		The descriptor whose buffer should be freed.
	//
	static void _freeBuffer(volatile Buffer *buffer);

	// default receive buffer size
	static const uint16_t _default_rx_buffer_size = 256;

	// default transmit buffer size
	static const uint16_t _default_tx_buffer_size = 256;

	// maxium tx/rx buffer size
	static const uint16_t _max_buffer_size = 512;
};

extern volatile YUNEEC::YUNEECUARTDriver::Buffer __YUNEECUARTDriver__rxBuffer[];
extern volatile YUNEEC::YUNEECUARTDriver::Buffer __YUNEECUARTDriver__txBuffer[];

//----------------------------------------------------------------------------
// USART Interrupt Handlers
//----------------------------------------------------------------------------
inline void UARTBufferUpdater(USART_TypeDef* usart, uint8_t portNum) {
	static uint8_t c;
	static uint16_t i;

	if((usart->CR1 & USART_CR1_RXNEIE) && (usart->SR & USART_FLAG_RXNE)) {
		/* read the byte as quickly as possible */
		c = (uint8_t)(usart->DR & 0x0ff);
		/* work out where the head will go next */
		i = (__YUNEECUARTDriver__rxBuffer[portNum].head + 1) & __YUNEECUARTDriver__rxBuffer[portNum].mask;
		/* decide whether we have space for another byte */
		if (i != __YUNEECUARTDriver__rxBuffer[portNum].tail) {
			/* we do, move the head */
			__YUNEECUARTDriver__rxBuffer[portNum].bytes[__YUNEECUARTDriver__rxBuffer[portNum].head] = c;
			__YUNEECUARTDriver__rxBuffer[portNum].head = i;
		}
	}

	if((usart->CR1 & USART_CR1_TXEIE) && (usart->SR & USART_FLAG_TXE)) {
		/* if there is another character to send */
		if (__YUNEECUARTDriver__txBuffer[portNum].tail != __YUNEECUARTDriver__txBuffer[portNum].head) {
			usart->DR = (uint16_t)__YUNEECUARTDriver__txBuffer[portNum].bytes[__YUNEECUARTDriver__txBuffer[portNum].tail] & (uint16_t)0x00FF;
			/* increment the tail */
			__YUNEECUARTDriver__txBuffer[portNum].tail = (__YUNEECUARTDriver__txBuffer[portNum].tail + 1) & __YUNEECUARTDriver__txBuffer[portNum].mask;
		} else {
			/* there are no more bytes to send, disable the interrupt */
			usart->CR1 &= ~USART_CR1_TXEIE;
		}
	}

}

#define YUNEECUARTDriverHandler(USARTx, portNum) 			\
extern "C" {									 			\
	void USARTx##_IRQHandler(void)               			\
	{														\
		__disable_irq();									\
		UARTBufferUpdater((USART_TypeDef* )USARTx, portNum);\
		__enable_irq();										\
	}											 			\
}												 			\
struct hack

//----------------------------------------------------------------------------
// USART Instance
//----------------------------------------------------------------------------
#define USART1_TX_PORT				GPIOB
#define USART1_RX_PORT				GPIOB
#define USART1_TX_BIT				GPIO_Pin_6
#define USART1_RX_BIT				GPIO_Pin_7
#define USART1_TX_PINSOURCE 		GPIO_PinSource6
#define USART1_RX_PINSOURCE 		GPIO_PinSource7
#define USART1_GPIO_AF				GPIO_AF_USART1
#define USART1_RCC_CLK				RCC_APB2Periph_USART1
#define USART1_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOB
#define USART1_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOB

#define USART2_TX_PORT				GPIOD
#define USART2_RX_PORT				GPIOD
#define USART2_TX_BIT				GPIO_Pin_5
#define USART2_RX_BIT				GPIO_Pin_6
#define USART2_TX_PINSOURCE 		GPIO_PinSource5
#define USART2_RX_PINSOURCE 		GPIO_PinSource6
#define USART2_GPIO_AF				GPIO_AF_USART2
#define USART2_RCC_CLK				RCC_APB1Periph_USART2
#define USART2_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOD
#define USART2_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOD

#define USART3_TX_PORT				GPIOD
#define USART3_RX_PORT				GPIOD
#define USART3_TX_BIT				GPIO_Pin_8
#define USART3_RX_BIT				GPIO_Pin_9
#define USART3_TX_PINSOURCE 		GPIO_PinSource8
#define USART3_RX_PINSOURCE 		GPIO_PinSource9
#define USART3_GPIO_AF				GPIO_AF_USART3
#define USART3_RCC_CLK				RCC_APB1Periph_USART3
#define USART3_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOD
#define USART3_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOD

#define UART4_TX_PORT				GPIOC
#define UART4_RX_PORT				GPIOC
#define UART4_TX_BIT				GPIO_Pin_10
#define UART4_RX_BIT				GPIO_Pin_11
#define UART4_TX_PINSOURCE 			GPIO_PinSource10
#define UART4_RX_PINSOURCE 			GPIO_PinSource11
#define UART4_GPIO_AF				GPIO_AF_UART4
#define UART4_RCC_CLK				RCC_APB1Periph_UART4
#define UART4_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOC
#define UART4_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOC

#define UART5_TX_PORT				GPIOC
#define UART5_RX_PORT				GPIOD
#define UART5_TX_BIT				GPIO_Pin_12
#define UART5_RX_BIT				GPIO_Pin_2
#define UART5_TX_PINSOURCE 			GPIO_PinSource12
#define UART5_RX_PINSOURCE 			GPIO_PinSource2
#define UART5_GPIO_AF				GPIO_AF_UART5
#define UART5_RCC_CLK				RCC_APB1Periph_UART5
#define UART5_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOC
#define UART5_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOD

#define USART6_TX_PORT				GPIOC
#define USART6_RX_PORT				GPIOC
#define USART6_TX_BIT				GPIO_Pin_6
#define USART6_RX_BIT				GPIO_Pin_7
#define USART6_TX_PINSOURCE 		GPIO_PinSource6
#define USART6_RX_PINSOURCE 		GPIO_PinSource7
#define USART6_GPIO_AF				GPIO_AF_USART6
#define USART6_RCC_CLK				RCC_APB2Periph_USART6
#define USART6_RCC_GPIO_TX_CLK		RCC_AHB1Periph_GPIOC
#define USART6_RCC_GPIO_RX_CLK		RCC_AHB1Periph_GPIOC

#define YUNEECUARTDriverInstance(USARTx, portNum)                       			\
YUNEECUARTDriver USARTx##Driver(portNum, 											\
								USARTx, USARTx##_IRQn,								\
								USARTx##_TX_PORT, USARTx##_RX_PORT,  				\
								USARTx##_TX_BIT, USARTx##_RX_BIT,					\
								USARTx##_TX_PINSOURCE, USARTx##_RX_PINSOURCE,		\
								USARTx##_GPIO_AF, USARTx##_RCC_CLK,					\
								USARTx##_RCC_GPIO_TX_CLK, USARTx##_RCC_GPIO_RX_CLK  \
								)

#endif // __AP_HAL_YUNEEC_UARTDRIVER_H__
