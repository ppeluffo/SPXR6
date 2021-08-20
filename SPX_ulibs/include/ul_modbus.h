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
	uint8_t nro_regs; 	// Cada registro son 2 bytes por lo que siempre leemos 2 x N.
	float divisor_p10;	// factor por el que multiplicamos para tener la magnitud
	uint8_t rcode;		// Codigo con el que se lee
} modbus_channel_t;

typedef struct {
	uint8_t slave_address;
	uint16_t waiting_poll_time;
	modbus_channel_t channel[MODBUS_CHANNELS];
} modbus_conf_t;

modbus_conf_t modbus_conf;

#define MBUS_TXMSG_LENGTH	16
#define MBUS_RXMSG_LENGTH	16

/*
 * Los registros de modbus pueden ser enteros, long o float.
 * Recibo un string en modo BIGENDIAN que lo guardo en raw_value.
 * De acuerdo a lo que lea es como queda interpretado
 */
typedef union {
	uint32_t u32_value;
	float float_value;
	uint8_t raw_value[4];
	char str_value[4];		// Almaceno NaN cuando hay un error.
} modbus_hold_t; // (4)

typedef struct {
	uint8_t sla_address;
	uint8_t function_code;
	uint16_t address;
	uint8_t nro_recds;
	char type;
	uint8_t divisor_p10;
	uint8_t tx_buffer[MBUS_TXMSG_LENGTH];
	uint8_t rx_buffer[MBUS_RXMSG_LENGTH];
	uint8_t tx_size;
	uint8_t rx_size;
	uint8_t payload[4];
} mbus_CONTROL_BLOCK_t;


bool modbus_read( float mbus_data[] );
void modbus_config_defaults(void);
void modbus_config_status(void);
bool modbus_config_slave( char *s_slave_address );
bool modbus_config_channel( char *s_ch, char *s_name, char *s_addr, char *s_nro_recds, char *s_rcode, char *s_type, char *s_divisor_p10 );
bool modbus_config_waiting_poll_time( char *s_waiting_poll_time);
uint8_t modbus_hash(void);
bool modbus_io( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb, modbus_hold_t *hreg );
void modbus_read_channel ( uint8_t ch, float *result );

void pv_modbus_make_ADU( mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_txmit_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
bool pv_modbus_rcvd_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
bool pv_modbus_decode_ADU ( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb, modbus_hold_t *hreg );


#endif /* SPX_ULIBS_INCLUDE_UL_MODBUS_H_ */
