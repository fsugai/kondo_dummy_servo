/*
 * ics.c
 *
 *  Created on: 2019/06/27
 *      Author: sugai
 */

#include "ics.h"

struct servo_params sp;
uint8_t recv_buf;

typedef enum {CMD, DATA, ACK} RX_STATE;
volatile uint8_t data_length;
volatile uint8_t data_cnt = 0;
volatile RX_STATE rx_stat = CMD;
volatile uint8_t recv_data[10];
volatile uint8_t send_data[10];
volatile uint8_t send_length = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    if(huart->Instance==LPUART1){
		if(((recv_buf & 0x80) == 0x80) && ((recv_buf & 0xe0) == CMD_ID || (recv_buf & 0x1f) == sp.id)){
			switch(recv_buf & 0xe0){
			case CMD_POS:
				rx_stat = DATA;
				data_length = 3;
				break;
			case CMD_RD:
				rx_stat = DATA;
				data_length = 2;
				break;
			case CMD_WR:
				rx_stat = DATA;
				data_length = 3;
				break;
			case CMD_ID:
				if((recv_buf & 0x1f) == 0x1f){ // id read
					send_data[0] = 0xe0 | sp.id;
					send_length = 1;
					rx_stat = DATA;
					data_length = 4;
				}else{ // id write
					sp.id = recv_buf & 0x1f;
					send_data[0] = recv_buf;
					send_length = 1;
					rx_stat = DATA;
					data_length = 4;
				}
				break;
			default:
				break;
			}
			data_cnt = 0;

			recv_data[0] = 0xff;
			recv_data[1] = 0xff;
			recv_data[2] = 0xff;
			recv_data[3] = 0xff;
			recv_data[4] = 0xff;
		}

		switch(rx_stat){
		case DATA:
			recv_data[data_cnt++] = recv_buf;

			if(data_cnt == data_length){
				switch(recv_data[0] & 0xe0){
				case CMD_POS:
					sp.pos = recv_data[1] << 7 | recv_data[2];
					send_data[0] = recv_data[0];
					send_data[1] = sp.tch & 0x3f80 >> 7;
					send_data[2] = sp.tch & 0x7f;
					send_length = 3;
					break;
				case CMD_RD:
					send_data[0] = recv_data[0] & 0x7f;
					send_data[1] = recv_data[1];
					switch(recv_data[1]){
					case SC_EEPROM:
						break;
					case SC_STRC:
						send_data[2] = sp.strc;
						send_length = 3;
						break;
					case SC_SPD:
						send_data[2] = sp.spd;
						send_length = 3;
						break;
					case SC_CUR:
						send_data[2] = sp.cur++;
						send_length = 3;
						break;
					case SC_TMP:
						send_data[2] = sp.tmp++;
						send_length = 3;
						break;
					case SC_TCH:
						send_data[2] = sp.tch & 0x3f80 >> 7;
						send_data[3] = sp.tch & 0x7f;
						send_length = 4;
						break;
					default:
						break;
					}
					break;
				case CMD_WR:
					send_data[0] = recv_data[0] & 0x7f;
					send_data[1] = recv_data[1];
					send_data[2] = recv_data[2];
					send_length = 3;
					switch(recv_data[1]){
					case SC_EEPROM:
						break;
					case SC_STRC:
						sp.strc = recv_data[2];
						break;
					case SC_SPD:
						sp.spd = recv_data[2];
						break;
					case SC_CUR:
						sp.cur_lim = recv_data[2];
						break;
					case SC_TMP:
						sp.tmp_lim = recv_data[2];
						break;
					default:
						break;
					}
					break;
				default:
					break;
				}
				rx_stat = ACK;
			}
			break;
		default:
			break;

		}

		if(rx_stat == ACK){
			HAL_HalfDuplex_EnableTransmitter(huart);
			HAL_UART_Transmit(huart, send_data, send_length,-1);
			rx_stat = CMD;
		}
		HAL_HalfDuplex_EnableReceiver(huart);
		HAL_UART_Receive_IT(huart, &recv_buf, 1);
    }
}
