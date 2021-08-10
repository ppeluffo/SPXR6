/*
 * ul_modbus.h
 *
 *  Created on: 10 ago. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_MODBUS_H_
#define SPX_ULIBS_INCLUDE_UL_MODBUS_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "ctype.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "l_printf.h"
#include "ul_utils.h"



#define MODBUS_CHANNELS 20

/*
 * En modbus leemos de a 1 canal, no bloques !!
 * Los canales pueden ser holding_registers ( 0x03 ) o input_registers (0x04).
 * Cada registro son 2 bytes por lo que siempre leemos 2 x N.
 * N: nro_recds.
 * En tipo ( 'F','I' ) indicamos como los interpretamos.
 * En divisor_10 indicamos el factor por el que multiplicamos para tener la magnitud.
 *
 */
typedef struct {
	char name[PARAMNAME_LENGTH];
	char type;			// ( 'F','I' )
	uint16_t address;
	uint8_t nro_recds; 	// Cada registro son 2 bytes por lo que siempre leemos 2 x N.
	float divisor_p10;	// factor por el que multiplicamos para tener la magnitud
	uint8_t rcode;		// Codigo con el que se lee
} modbus_channel_t;

typedef struct {
	uint8_t slave_address;
	modbus_channel_t channel[MODBUS_CHANNELS];
} modbus_conf_t;

modbus_conf_t modbus_conf;

void modbus_config_defaults(void);
void modbus_config_status(void);
bool modbus_config_slave( char *s_slave_address );
bool modbus_config_channel( char *s_ch, char *s_name, char *s_addr, char *s_nro_recds, char *s_rcode, char *s_type, char *s_divisor_p10 );
uint8_t modbus_hash(void);


#endif /* SPX_ULIBS_INCLUDE_UL_MODBUS_H_ */
