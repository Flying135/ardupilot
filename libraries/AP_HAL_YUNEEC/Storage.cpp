#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include <string.h>
#include "Storage.h"
#include "utility/EEPROM.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

// The EEPROM class uses 2x16k FLASH ROM pages to emulate 8k of EEPROM.
#define EEPROM_SIZE		(uint16_t)(8 * 1024)

// This is the size of each FLASH ROM page
const uint32_t pageSize  = 0x4000;

// This defines the base addresses of the 2 FLASH ROM pages that will be used to emulate EEPROM
const uint32_t pageBase0 = 0x08004000;
const uint32_t pageBase1 = 0x08008000;

static EEPROMClass eeprom;

YUNEECStorage::YUNEECStorage()
{}

void YUNEECStorage::init(void*)
{
	uint16_t result = eeprom.init(pageBase0, pageBase1, pageSize);

	if (result != EEPROM_OK)
		hal.console->printf("YUNEECStorage::init eeprom failed: %x\n", result);

}

uint8_t YUNEECStorage::read_byte(uint16_t loc){
//hal.console->printf("read_byte %d\n", loc);

    // 'bytes' are packed 2 per word
    // Read existing dataword and change upper or lower byte
    uint16_t data = eeprom.read(loc >> 1);
    if (loc & (uint16_t)0x01)
    	return data >> 8; // Odd, upper byte
    else
    	return data & 0xff; // Even lower byte
}

uint16_t YUNEECStorage::read_word(uint16_t loc){
//hal.console->printf("read_word %d\n", loc);
    uint16_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

uint32_t YUNEECStorage::read_dword(uint16_t loc){
//hal.console->printf("read_dword %d\n", loc);
    uint32_t value;
    read_block(&value, loc, sizeof(value));
    return value;
}

void YUNEECStorage::read_block(void* dst, uint16_t src, size_t n) {
//    hal.console->printf("read_block %d %d\n", src, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
    	((uint8_t*)dst)[i] = read_byte(src+i);
}

void YUNEECStorage::write_byte(uint16_t loc, uint8_t value)
{
//    hal.console->printf("write_byte %d, %d\n", loc, value);

    // 'bytes' are packed 2 per word
    // Read existing data word and change upper or lower byte
    uint16_t data = eeprom.read(loc >> 1);
    if (loc & (uint16_t)0x01)
    	data = (data & 0x00ff) | (value << 8); // Odd, upper byte
    else
    	data = (data & 0xff00) | value; // Even, lower byte
    eeprom.write(loc >> 1, data);
}

void YUNEECStorage::write_word(uint16_t loc, uint16_t value)
{
//    hal.console->printf("write_word %d, %d\n", loc, value);
    write_block(loc, &value, sizeof(value));
}

void YUNEECStorage::write_dword(uint16_t loc, uint32_t value)
{
//    hal.console->printf("write_dword %d, %d\n", loc, value);
    write_block(loc, &value, sizeof(value));
}

void YUNEECStorage::write_block(uint16_t loc, const void* src, size_t n)
{
//    hal.console->printf("write_block %d, %d\n", loc, n);
    // Treat as a block of bytes
    for (size_t i = 0; i < n; i++)
    	write_byte(loc+i, ((uint8_t*)src)[i]);
}

#endif
