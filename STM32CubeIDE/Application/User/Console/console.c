/*
 * console.c
 *
 *  Created on: Sep 16, 2024
 *      Author: mose
 */
#include "console.h"
#include <stdint.h>

typedef enum {
	CONSOLE_BUSY = 0,
	CONSOLE_PENDING_RX,
	CONSOLE_PENDING_TX,
} ConsoleFlags_t;

static uint32_t console_flags_g = 0;

void OnRxBufferFull() {

}

void OnTxDone() {

}

void Console_Init() {

}

void Console_Process() {
	if (console_flags_g & (1 << CONSOLE_PENDING_RX)) {

	}
}
