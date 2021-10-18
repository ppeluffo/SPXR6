/*
 * ul_utils.h
 *
 *  Created on: 13 jul. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_UTILS_H_
#define SPX_ULIBS_INCLUDE_UL_UTILS_H_

#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <avr/pgmspace.h>

#include "FreeRTOS.h"
#include "semphr.h"

// Mensajes entre tareas
#define SGN_FRAME_READY			0x01
#define SGN_MON_SQE				0x02
#define SGN_REDIAL				0x03
#define SGN_RESET_COMMS_DEV		0x04
#define SGN_SMS					0x05
#define SGN_WAKEUP				0x06
#define SGN_POLL_NOW			0x07

#define PARAMNAME_LENGTH	7

bool SPX_SIGNAL( uint8_t signal );
bool SPX_SEND_SIGNAL( uint8_t signal );
bool SPX_CLEAR_SIGNAL( uint8_t signal );

void initMCU(void);
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);

uint8_t u_control_string( char *s_name );
void u_load_defaults( char *opt );
void u_save_params_in_NVMEE(void);
bool u_load_params_from_NVMEE(void);

void u_config_timerpoll ( char *s_timerpoll );

uint8_t u_base_hash(void);
uint8_t u_checksum( uint8_t *s, uint16_t size );
uint8_t u_hash(uint8_t checksum, char ch );
void u_hash_test(void);

void u_convert_int_to_time_t ( int int16time, uint8_t *hour, uint8_t *min );

void u_wdg_kick( uint8_t wdg_id, uint16_t timeout_in_secs );

typedef struct {
	uint8_t hour;
	uint8_t min;
} st_time_t;

char hash_buffer[64];

#endif /* SPX_ULIBS_INCLUDE_UL_UTILS_H_ */
