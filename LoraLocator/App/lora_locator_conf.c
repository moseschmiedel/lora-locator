/*
 * lora_locator_conf.c
 *
 *  Created on: Sep 23, 2024
 *      Author: mose
 */

#include "lora_locator_conf.h"

void LoraLocator_Config() {
	/* Set verbose-level for debug printing
	 * VLEVEL_H = All error and debug messages.
	 * VLEVEL_M = Data output (RSSI readings) and critical messages.
	 * VLEVEL_L = Only critical messages.
	 */
	UTIL_ADV_TRACE_SetVerboseLevel(VLEVEL_L);
}
