/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file st24.h
 *
 * RC protocol implementation for Yuneec ST24 transmitter.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * 		   Maelok Dong
 */

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "st24.h"
#include "CRC8.h"

typedef enum {
	ST24_DECODE_STATE_UNSYNCED = 0,
	ST24_DECODE_STATE_GOT_STX1,
	ST24_DECODE_STATE_GOT_STX2,
	ST24_DECODE_STATE_GOT_LEN,
	ST24_DECODE_STATE_GOT_TYPE,
	ST24_DECODE_STATE_GOT_DATA
} ST24_DECODE_STATE;

const char *decode_states[] = {"UNSYNCED",
			       "GOT_STX1",
			       "GOT_STX2",
			       "GOT_LEN",
			       "GOT_TYPE",
			       "GOT_DATA"
			      };

/* define range mapping here, -+100% -> 1000..2000 */
#define ST24_RANGE_MIN 0.0f
#define ST24_RANGE_MAX 4096.0f

#define ST24_TARGET_MIN 1000.0f
#define ST24_TARGET_MAX 2000.0f

/* pre-calculate the floating point stuff as far as possible at compile time */
#define ST24_SCALE_FACTOR ((ST24_TARGET_MAX - ST24_TARGET_MIN) / (ST24_RANGE_MAX - ST24_RANGE_MIN))
#define ST24_SCALE_OFFSET (int)(ST24_TARGET_MIN - (ST24_SCALE_FACTOR * ST24_RANGE_MIN + 0.5f))

static ST24_DECODE_STATE _decode_state = ST24_DECODE_STATE_UNSYNCED;
static unsigned _rxlen;

static ReceiverFcPacket _rxpacket;
static ReceiverFcPacket _txpacket;

#ifdef ST24_USE_CRC8_TABLE
uint8_t st24_crc8(uint8_t *ptr, uint8_t len)
{
	static const uint8_t st24_crcTable[256] = {
		0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31,
		0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
		0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,
		0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
		0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1,
		0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
		0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE,
		0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
		0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16,
		0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
		0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80,
		0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
		0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8,
		0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
		0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
		0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
		0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F,
		0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
		0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7,
		0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
		0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
		0xFA, 0xFD, 0xF4, 0xF3};

	return crc_table (ptr, len, st24_crcTable);

}
#else
uint8_t st24_crc8(uint8_t *ptr, uint8_t len)
{
	uint8_t i, crc ;
	crc = 0;

	while (len--) {
		for (i = 0x80; i != 0; i >>= 1) {
			if ((crc & 0x80) != 0) {
				crc <<= 1;
				crc ^= 0x07;

			} else {
				crc <<= 1;
			}

			if ((*ptr & i) != 0) {
				crc ^= 0x07;
			}
		}

		ptr++;
	}

	return (crc);
}
#endif

int st24_decode(uint8_t byte, volatile uint8_t *rssi, volatile uint8_t *rx_count, volatile uint16_t *channel_count,
			     volatile uint16_t *channels, volatile uint16_t max_chan_count)
{

	int ret = 1;

	switch (_decode_state) {
	case ST24_DECODE_STATE_UNSYNCED:
		if (byte == ST24_STX1) {
			_decode_state = ST24_DECODE_STATE_GOT_STX1;
		} else {
			ret = 3;
		}

		break;

	case ST24_DECODE_STATE_GOT_STX1:
		if (byte == ST24_STX2) {
			_decode_state = ST24_DECODE_STATE_GOT_STX2;

		} else {
			_decode_state = ST24_DECODE_STATE_UNSYNCED;
		}

		break;

	case ST24_DECODE_STATE_GOT_STX2:

		/* ensure no data overflow failure or hack is possible */
		if ((unsigned)byte <= ST24_LENGTH_VALUE) {
			_rxpacket.length = byte;
			_rxlen = 0;
			_decode_state = ST24_DECODE_STATE_GOT_LEN;

		} else {
			_decode_state = ST24_DECODE_STATE_UNSYNCED;
		}

		break;

	case ST24_DECODE_STATE_GOT_LEN:
		_rxpacket.type = byte;
		_rxlen++;
		_decode_state = ST24_DECODE_STATE_GOT_TYPE;
		break;

	case ST24_DECODE_STATE_GOT_TYPE:
		_rxpacket.st24_data[_rxlen - 1] = byte;
		_rxlen++;

		if (_rxlen == (_rxpacket.length - 1)) {
			_decode_state = ST24_DECODE_STATE_GOT_DATA;
		}

		break;

	case ST24_DECODE_STATE_GOT_DATA:
		_rxpacket.crc8 = byte;
		_rxlen++;

		if (st24_crc8((uint8_t *) & (_rxpacket.length), _rxlen) == _rxpacket.crc8) {

			ret = 0;

			/* decode the actual packet */

			switch (_rxpacket.type) {

			case ST24_PACKET_TYPE_CHANNELDATA12: {
					ChannelData12 *d = (ChannelData12 *)_rxpacket.st24_data;

					*rssi = d->rssi;
					*rx_count = d->packet_count;

					/* this can lead to rounding of the strides */
					*channel_count = (max_chan_count < 12) ? max_chan_count : 12;

					unsigned stride_count = (*channel_count * 3) / 2;
					unsigned chan_index = 0;

					for (unsigned i = 0; i < stride_count; i += 3) {
						channels[chan_index] = ((uint16_t)d->channel[i] << 4);
						channels[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;

						channels[chan_index] = ((uint16_t)d->channel[i + 2]);
						channels[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;
					}
				}
				break;

			case ST24_PACKET_TYPE_CHANNELDATA24: {
					ChannelData24 *d = (ChannelData24 *)&_rxpacket.st24_data;

					*rssi = d->rssi;
					*rx_count = d->packet_count;

					/* this can lead to rounding of the strides */
					*channel_count = (max_chan_count < 24) ? max_chan_count : 24;

					unsigned stride_count = (*channel_count * 3) / 2;
					unsigned chan_index = 0;

					for (unsigned i = 0; i < stride_count; i += 3) {
						channels[chan_index] = ((uint16_t)d->channel[i] << 4);
						channels[chan_index] |= ((uint16_t)(0xF0 & d->channel[i + 1]) >> 4);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;

						channels[chan_index] = ((uint16_t)d->channel[i + 2]);
						channels[chan_index] |= (((uint16_t)(0x0F & d->channel[i + 1])) << 8);
						/* convert values to 1000-2000 ppm encoding in a not too sloppy fashion */
						channels[chan_index] = (uint16_t)(channels[chan_index] * ST24_SCALE_FACTOR + .5f) + ST24_SCALE_OFFSET;
						chan_index++;
					}
				}
				break;

			case ST24_PACKET_TYPE_CHANNELDATA12_GPS: {

					// ReceiverFcPacket* d = (ReceiverFcPacket*)&_rxpacket.st24_data;
					/* we silently ignore this data for now, as it is unused */
					ret = 2;
				}
				break;

			default:
				ret = 2;
				break;
			}

		} else {
			/* decoding failed */
			ret = 4;
		}

		_decode_state = ST24_DECODE_STATE_UNSYNCED;
		break;
	}

	return ret;
}

ReceiverFcPacket* st24_encode_bind(StBindCmd *bindCmd)
{
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = PACKET_LEGNTH_STBINDCMD + 2;
	_txpacket.type = ST24_PACKET_TYPE_BINDCMD;
	memcpy(_txpacket.st24_data, (const uint8_t *)bindCmd, PACKET_LEGNTH_STBINDCMD);
	_txpacket.st24_data[PACKET_LEGNTH_STBINDCMD] = st24_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);
	return &_txpacket;
}
