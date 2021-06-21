/*
TITLE	: UART
DATE	: 2019/11/12
AUTHOR	: e-Yantra Team
*/

#define F_CPU 16000000UL
#define USART0_ENABLED

#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"
#include "uart.c"


char uart0_readByte(void){

	uint16_t rx;
	uint8_t rx_status, rx_data;

	rx = uart0_getc();
	rx_status = (uint8_t)(rx >> 8);
	rx = rx << 8;
	rx_data = (uint8_t)(rx >> 8);

	if(rx_status == 0 && rx_data != 0){
		return rx_data;
		} else {
		return -1;
	}

}

