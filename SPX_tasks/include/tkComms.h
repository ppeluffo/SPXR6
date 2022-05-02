/*
 * tkComms.h
 *
 *  Created on: 5 mar. 2020
 *      Author: pablo
 */

#ifndef SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_
#define SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_

#include "spx.h"

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "inttypes.h"

#include "FreeRTOS.h"

#define DF_COMMS ( (systemVars.debug == DEBUG_COMMS ) || (systemVars.debug == DEBUG_ALL ))

#define TDIAL_MIN_DISCRETO 300

#define MODO_DISCRETO ( ( comms_conf.timerDial >= TDIAL_MIN_DISCRETO ) ? true : false )

#define CCIDBUFFSIZE	24
#define IMEIBUFFSIZE	24

typedef struct {
	uint8_t csq;
	char ip_assigned[IP_LENGTH];
	bool gprs_prendido;
	bool gprs_inicializado;
	uint8_t errores_comms;
	uint8_t douts;
	bool set_douts;
	uint8_t gprs_mode;
	uint8_t gprs_pref;
	char gprs_bands[17];
	char gprs_ccid[CCIDBUFFSIZE];
	char gprs_imei[IMEIBUFFSIZE];
	int8_t modem_starts;

} t_xCOMMS_stateVars;

t_xCOMMS_stateVars xCOMMS_stateVars;

#define MAX_MODEM_STARTS	5

#define MAX_ERRORES_COMMS 	5

#define RESET_DEEP_SLEEP() (  xCOMMS_stateVars.modem_starts = -1 )

comms_conf_t comms_conf;

#define GPRS_TXBUFFER_LEN	384
struct {
	char buffer[GPRS_TXBUFFER_LEN];
	uint16_t ptr;
} gprs_txbuffer;

#define GPRS_RXBUFFER_LEN	576
struct {
	char buffer[GPRS_RXBUFFER_LEN];
	uint16_t ptr;
} gprs_rxbuffer;

#define MAX_DATA_WINDOW_SIZE	5


#define MAX_TRYES_SOCKET_CLOSE	1

#define  SOCKET_OPEN_TIMEOUT_secs 15

//------------------------------------------------------------------------------------

#define AUX_TXBUFFER_LEN	16
struct {
	char buffer[AUX_TXBUFFER_LEN];
	uint8_t ptr;
} aux_txbuffer;

#define AUX_RXBUFFER_LEN	90
struct {
	char buffer[AUX_RXBUFFER_LEN];
	uint8_t ptr;
} aux_rxbuffer;

//------------------------------------------------------------------------------------

typedef enum { APAGADO, PRENDIDO_OFFLINE, PRENDIDO_ONLINE } t_xcomms_states;
typedef enum { ATRSP_NONE, ATRSP_OK, ATRSP_ERROR, ATRSP_TIMEOUT, ATRSP_OUT_INMEDIATE, ATRSP_UNKNOWN } t_at_commands_responses;

#define TIMETOCHECKSMS 			60
#define MAXTIMEOFFLINEAWAITON	600
#define MAXHWTRYESPRENDER		3
#define MAXSWTRYESPRENDER		3
#define MAXCPASTRYES			3
#define MAX_TRYES_SUBSTATE_ONLINE	4

//------------------------------------------------------------------------------------

int8_t tkXComms_APAGADO(void);
int8_t tkXComms_PRENDIDO_OFFLINE(void);
int8_t tkXComms_PRENDIDO_ONLINE(void);

int32_t pubcomms_awaittime_for_dial(void);
int8_t FSM_sendATcmd( const uint8_t timeout, char *cmd );

#define SEC_CHECK_RSP	10

//------------------------------------------------------------------------------------

void gprs_rxbuffer_reset(void);
void gprs_txbuffer_reset(void);

bool gprs_rxbuffer_full(void);
bool gprs_rxbuffer_empty(void);
uint16_t gprs_rxbuffer_usedspace(void);

void gprs_rxbuffer_put( char data);
bool gprs_rxbuffer_put2( char data );
void gprs_flush_RX_buffer(void);
void gprs_flush_TX_buffer(void);
void gprs_print_RX_buffer(void);
bool gprs_check_response( const uint16_t timeout, const char *rsp );

void gprs_init(void);
bool gprs_prender( void );
void gprs_hw_pwr_on(uint8_t delay_factor);
void gprs_sw_pwr(void);
void gprs_apagar(void);
int gprs_findstr_lineal( uint16_t start, const char *rsp );

void comms_config_defaults(char *opt);
void comms_config_status(void);

void gprs_set_MODO( uint8_t modo);
void gprs_set_PREF(uint8_t modo);
void gprs_set_BANDS( char *s_bands);
void gprs_set_SAT(uint8_t modo);

void gprs_read_MODO(void);
void gprs_read_PREF(void);
void gprs_read_BANDS(void);

//--------------------------------------------------------------------------------------------

void aux_init(void);
void aux_prender(void);
bool is_aux_prendido(void);
void aux_apagar(void);
void aux_rts_on(void);
void aux_rts_off(void);

void aux_rxbuffer_reset(void);
void aux_txbuffer_reset(void);
bool aux_rxbuffer_full(void);
bool aux_rxbuffer_empty(void);
uint16_t aux_rxbuffer_usedspace(void);
void aux_rxbuffer_put( char data);
bool aux_rxbuffer_put2( char data );
void aux_flush_RX_buffer(void);
void aux_flush_TX_buffer(void);
bool aux_rxbuffer_copyto ( uint8_t *dst_buffer, uint8_t *size, int8_t max_size );
void aux_print_RX_buffer( bool ascii_mode );
uint8_t aux_get_RX_buffer_ptr(void);

bool test_xmit_window_data( void );


void comms_idle_task(void);

#endif /* SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_ */
