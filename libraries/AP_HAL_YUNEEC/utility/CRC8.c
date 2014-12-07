
#include "CRC8.h"

/*
 * Bitwise CRC8 algorithm
 */
uint8_t crc_bitwise (uint8_t* data, size_t length, uint8_t polynomial)
{
	uint8_t d, rval = 0;
	int i;

	while (length--) {
		d = (*data ^ rval);
		for (i = 0; i < 8; i++) {
			if (d & 0x80)
				d = (d << 1) ^ polynomial;
			else
				d <<= 1;
		}

		rval = d;
		data++;
	}

	return rval;
}

/*
 * Lookup table CRC8 algorithm
 */
uint8_t crc_table (uint8_t* data, size_t length, const uint8_t table[256])
{
	uint8_t rval = 0;
	while (length--) {
		rval = PGM_READ_BYTE(table[rval ^ *data]);
		data++;
	}

	return rval;
}

/*
 * Generate CRC8 table
 */
void generate_table (uint8_t polynomial, uint8_t table[256])
{
	int i, j;
	for (i = 0; i < 256; i++) {
		uint8_t entry = i;

		for (j = 0; j < 8; j++) {
			if (entry & 0x80)
				entry = (entry << 1) ^ polynomial;
			else
				entry <<= 1;
		}
		table[i] = entry;
	}
}

/*
 * Print crc8 table according to your polynomial in case user want to directly
 * define a table. You need to define CRC8_PRINT to one instance such as UART,
 * USB and etc.
 */
#if PRINT_CRC8_TABLE == 1

#ifndef CRC8_PRINT
#error	"CRC8: You have to instantiate CRC8_PRINT to show CRC8 table!"
#endif

void print_generate_table (uint8_t polynomial)
{
	CRC8_PRINT("const uint8_t crc8_lut[256] = {\n");

	for (int i = 0; i < 256; i++) {
		uint8_t entry = i;

		for (int j = 0; j < 8; j++) {
			if (entry & 0x80)
				entry = (entry << 1) ^ polynomial;
			else
				entry <<= 1;
		}
		CRC8_PRINT("%02x", entry);
		if (i < 255)
			CRC8_PRINT(", ");
		if ( (i + 1) % 12 == 0)
			CRC8_PRINT("\n");
	}
	CRC8_PRINT("};\n");
}
#endif
