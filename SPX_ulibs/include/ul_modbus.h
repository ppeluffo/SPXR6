/*
 * ul_modbus.h
 *
 *  Created on: 10 ago. 2021
 *      Author: pablo
 *
 *  Testing: Registros a programar en pyModSlave
 *  cd /home/pablo/Spymovil/python/pennv/ml/lib/python3.8/site-packages/pyModSlave
 *  python3 pyModSlave.py
 *
 *
 *  FLOATS:
 *  150.31 0x4316 51EC
 *  934.77 0x4469 B148
 *  5.22   0x40A7 0A3D
 *  -18.3  0xC192 6666
 *  7641   0x45EE C800
 *
 *  SHMETERS:
 *
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

typedef enum { u16=0,i16,u32,i32,FLOAT } t_modbus_types;
typedef enum { CODEC0123, CODEC1032, CODEC3210, CODEC2301 } t_modbus_codec;

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
	t_modbus_types type;		// Indica si es entero con o sin signo, float, etc
	t_modbus_codec codec;		// Indica el orden de los bytes en numeros de mas de 1 byte
	uint8_t slave_address;		// Direccion del dispositivo en el bus.
	uint16_t reg_address;		// Direccion de la posicion de memoria donde leer.
	uint8_t nro_regs; 			// Cada registro son 2 bytes por lo que siempre leemos 2 x N.
	uint8_t divisor_p10;		// factor de potencia de 10 por el que dividimos para tener la magnitud
	uint8_t fcode;				// Codigo de la funcion con que se accede al registro.
} modbus_channel_t;

/*
 * Definimos un dispositivo al cual le reportamos el status del datalogger.
 * En particular las comunicaciones.
 */
typedef struct {
	uint8_t slave_address;		// Direccion del dispositivo en el bus.
	uint16_t reg_address;		// Direccion de la posicion de memoria donde leer.
} modbus_control_channel_t;

typedef struct {
	uint8_t modbus_delay_inter_bytes;
	uint16_t waiting_poll_time;
	modbus_channel_t channel[MODBUS_CHANNELS];
	modbus_control_channel_t control_channel;
} modbus_conf_t;

modbus_conf_t modbus_conf;

#define MBUS_TXMSG_LENGTH	16
#define MBUS_RXMSG_LENGTH	90

/*
 * Los registros de modbus pueden ser enteros, long o float.
 * Recibo un string en modo BIGENDIAN que lo guardo en raw_value.
 * De acuerdo a lo que lea es como queda interpretado
 */
typedef union {
	uint8_t raw_value[4];
	// Interpretaciones
	uint16_t u16_value;
	int16_t i16_value;
	uint32_t u32_value;
	int32_t i32_value;
	float float_value;
	char str_value[4];		// Almaceno NaN cuando hay un error.
} modbus_hold_t; // (4)


typedef struct {

	modbus_channel_t channel;
	uint8_t tx_buffer[MBUS_TXMSG_LENGTH];
	uint8_t rx_buffer[MBUS_RXMSG_LENGTH];
	uint8_t tx_size;		// Cantidad de bytes en el txbuffer para transmitir
	uint8_t rx_size;		// Cantidad de bytes leidos en el rxbufer.
	uint8_t payload_ptr;	// Indice al primer byte del payload.
	modbus_hold_t udata;	// estructura para interpretar los distintos formatos de los datos.
	bool io_status;			// Flag que indica si la operacion de lectura fue correcta o no
} mbus_CONTROL_BLOCK_t;

typedef struct {
	modbus_channel_t channel;
	modbus_hold_t udata;
} mbus_queue_t;

#define MBUSFIFO_STORAGE_SIZE  8
mbus_queue_t mbusFifo_storage[MBUSFIFO_STORAGE_SIZE];
void_ringBuffer_s mbusFIFO;

void modbus_config_defaults(void);
void modbus_config_status(void);
bool modbus_config_channel( uint8_t channel,char *s_name,char *s_sla,char *s_addr,char *s_nro_recds,char *s_fcode,char *s_type,char *s_codec,char *s_divisor_p10 );
bool modbus_config_waiting_poll_time( char *s_waiting_poll_time);
bool modbus_config_dib( char *s_delay_inter_bytes);
bool modbus_config_chcontrol ( char *sla_addr, char *reg_addr);

uint8_t modbus_hash(void);

void modbus_io( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_read( float mbus_data[] );
float modbus_read_channel ( uint8_t ch );
void modbus_report_status(uint8_t bit_pos, uint8_t bit_value );
void modbus_read_block_channel( float mbus_data[] );

void pv_modbus_make_ADU( mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_txmit_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_rcvd_ADU( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void pv_modbus_decode_ADU ( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );

void modbus_print( bool f_debug, mbus_CONTROL_BLOCK_t *mbus_cb );
void modbus_data_print( file_descriptor_t fd, float mbus_data[] );
char * modbus_sprintf( char *sbuffer, float src[] );

void modbus_test_genpoll(char *arg_ptr[16] );
void modbus_test_frame(char *arg_ptr[16] );
void modbus_test_chpoll(char *s_channel);
void modbus_test_int( char *s_nbr );
void modbus_test_long( char *s_nbr );
void modbus_test_float( char *s_nbr );

void modbus_init_output_cmds_queue(void);
bool modbus_enqueue_output_cmd( char *s_slaaddr,char *s_regaddr,char *s_nro_regs,char *s_fcode, char *s_type,char *s_codec, char *s_value );
void modbus_dequeue_output_cmd(void);
void modbus_write_output_register( char *s_slaaddr,char *s_regaddr,char *s_nro_regs,char *s_fcode, char *s_type,char *s_codec, char *s_value );
float modbus_write_output_channel( uint8_t ch, float fvalue );

//------------------------------------------------------------------------------------
// Flag que indica si el la orden enviada por el dataframe pudo ser ejecutada o no
typedef enum { MB_NONE=0, MB_ACK, MB_NACK } t_modbus_responses;

t_modbus_responses modbus_cmd_status_flag;

#define SET_MBUS_STATUS_FLAG_NONE()	( modbus_cmd_status_flag = MB_NONE )
#define SET_MBUS_STATUS_FLAG_NACK()	( modbus_cmd_status_flag = MB_NACK )
#define SET_MBUS_STATUS_FLAG_ACK()	( modbus_cmd_status_flag = MB_ACK )

#define IS_MBUS_STATUS_FLAG_NONE()	( modbus_cmd_status_flag == MB_NONE )
#define IS_MBUS_STATUS_FLAG_ACK()	( modbus_cmd_status_flag == MB_ACK )
#define IS_MBUS_STATUS_FLAG_NACK()	( modbus_cmd_status_flag == MB_NACK )

int16_t mbus_tag;

#define INIT_MBUS_TAG()	(mbus_tag = -1 )
#define MBUS_TAG_VALID() ( mbus_tag != -1 )

void modbus_set_mbtag( char *s_mbtag);
int16_t modbus_get_mbtag( void );


#endif /* SPX_ULIBS_INCLUDE_UL_MODBUS_H_ */
