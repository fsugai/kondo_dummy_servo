#ifndef __ICS_H
#define __ICS_H

#include "stm32l0xx_hal.h"

// CMD
#define CMD_POS 0x80
#define CMD_RD 0xa0
#define CMD_WR 0xc0
#define CMD_ID 0xe0

// SC
#define SC_EEPROM 0x00
#define SC_STRC 0x01
#define SC_SPD 0x02
#define SC_CUR 0x03
#define SC_TMP 0x04
#define SC_TCH 0x05


struct servo_params {
	// read write data
	uint8_t id;
	uint8_t strc;
	uint8_t spd;

	// write data
	uint16_t pos;
	uint8_t cur_lim;
	uint8_t tmp_lim;

	// read data
	uint16_t tch;
	uint8_t cur;
	uint8_t tmp;
};

extern struct servo_params sp;
extern uint8_t recv_buf;


#endif
