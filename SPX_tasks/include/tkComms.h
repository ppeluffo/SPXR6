/*
 * tkComms.h
 *
 *  Created on: 5 mar. 2020
 *      Author: pablo
 */

#ifndef SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_
#define SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_


#include "spx.h"


#define DF_COMMS ( systemVars.debug == DEBUG_COMMS )

#define TDIAL_MIN_DISCRETO 300

#define MODO_DISCRETO ( (sVarsComms.timerDial >= TDIAL_MIN_DISCRETO ) ? true : false )

int32_t time_to_next_dial;

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
} t_xCOMMS_stateVars;

t_xCOMMS_stateVars xCOMMS_stateVars;

#define MAX_ERRORES_COMMS 5

typedef struct {
	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char simpwd[SIM_PASSWD_LENGTH];
	uint32_t timerDial;
} xComms_conf_t;

xComms_conf_t sVarsComms;

//------------------------------------------------------------------------------------

typedef enum { APAGADO, PRENDIDO_OFFLINE, PRENDIDO_ONLINE } t_xcomms_states;
typedef enum { ATRSP_NONE, ATRSP_EXPECTED, ATRSP_NOTEXPECTED, ATRSP_ERR, ATRSP_TIMEOUT, ATRSP_UNKNOWN } t_at_commands_responses;

#define TIMETOCHECKSMS 			60
#define MAXTIMEOFFLINEAWAITON	600
#define MAXHWTRYESPRENDER		3
#define MAXSWTRYESPRENDER		3
#define MAXCPASTRYES			3

//------------------------------------------------------------------------------------

int8_t tkXComms_APAGADO(void);
int8_t tkXComms_PRENDIDO_OFFLINE(void);
int8_t tkXComms_PRENDIDO_ONLINE(void);

int32_t pubcomms_awaittime_for_dial(void);
int8_t FSM_sendATcmd(int8_t timeout, char *cmd, char *rsp );

//------------------------------------------------------------------------------------

void gprs_rxbuffer_reset(void);
bool gprs_rxbuffer_full(void);
bool gprs_rxbuffer_empty(void);
uint16_t gprs_rxbuffer_usedspace(void);


void gprs_rxbuffer_put( char data);
bool gprs_rxbuffer_put2( char data );
bool gprs_rxbuffer_get( char * data );
void gprs_flush_RX_buffer(void);
void gprs_flush_TX_buffer(void);
void gprs_print_RX_buffer(void);
int gprs_check_response( uint16_t start, const char *rsp );
int gprs_check_response_with_to( uint16_t start, const char *rsp, uint8_t timeout );
void gprs_rxbuffer_copy_to( char *dst, uint16_t start, uint16_t size );

void gprs_atcmd_preamble(void);
void gprs_init(void);
bool gprs_prender( void );
void gprs_hw_pwr_on(uint8_t delay_factor);
void gprs_sw_pwr(void);
void gprs_apagar(void);
bool gprs_SAT_set(uint8_t modo);
int gprs_findstr_lineal( uint16_t start, const char *rsp );
void gprs_modem_status(void);

//--------------------------------------------------------------------------------------------


#endif /* SRC_SPX_TASKS_SPX_TKCOMMS_TKCOMMS_H_ */
