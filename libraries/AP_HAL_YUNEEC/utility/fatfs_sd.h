/*-----------------------------------------------------------------------/
/  Low level disk interface modlue include file   (C)ChaN, 2013          /
/-----------------------------------------------------------------------*/

#ifndef _DISKIO_DEFINED_SD
#define _DISKIO_DEFINED_SD

#ifdef __cplusplus
extern "C" {
#endif

#define _USE_WRITE	1	/* 1: Enable disk_write function */
#define _USE_IOCTL	1	/* 1: Enable disk_ioctl fucntion */

#include <diskio.h>
#include <integer.h>

#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <misc.h>
#include <defines.h>

#define FATFS_CS_RCC		RCC_AHB1Periph_GPIOB
#define FATFS_CS_PORT		GPIOB
#define FATFS_CS_PIN		GPIO_Pin_12

#define FATFS_USE_DETECT_PIN				1
#define FATFS_USE_DETECT_PIN_RCC			RCC_AHB1Periph_GPIOD
#define FATFS_USE_DETECT_PIN_PORT			GPIOD
#define FATFS_USE_DETECT_PIN_PIN			GPIO_Pin_10

#define FATFS_CS_LOW						FATFS_CS_PORT->BSRRH = FATFS_CS_PIN
#define FATFS_CS_HIGH						FATFS_CS_PORT->BSRRL = FATFS_CS_PIN

/*---------------------------------------*/
/* Prototypes for disk control functions */
DSTATUS TM_FATFS_SD_disk_initialize(void);
DSTATUS TM_FATFS_SD_disk_status(void);
DRESULT TM_FATFS_SD_disk_read(BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_write(const BYTE* buff, DWORD sector, UINT count);
DRESULT TM_FATFS_SD_disk_ioctl(BYTE cmd, void* buff);


#ifdef __cplusplus
}
#endif

#endif

