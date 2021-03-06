/*
 * tkXComms_ONLINE.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 */


#include <tkComms.h>
#include <tkComms_sms.h>
#include "tkApp.h"
#include "ul_pilotos.h"

typedef enum { ONLINE_ENTRY, ONLINE_AUTH, ONLINE_GLOBAL, ONLINE_BASE, ONLINE_ANALOG, ONLINE_DIGITAL, ONLINE_COUNTER, ONLINE_MODBUS, ONLINE_APP, ONLINE_DATA, ONLINE_ESPERA, ONLINE_EXIT, ONLINE_SMS } t_states_prendido_online;
typedef enum { SF_ENTRY, SF_SOCK_STATUS, SF_SOCK_OPEN, SF_NET_STATUS, SF_SEND, SF_RSP, SF_EXIT } t_sendFrames_states;
typedef enum { FRM_AUTH, FRM_GLOBAL, FRM_BASE, FRM_ANALOG, FRM_DIGITAL, FRM_COUNTER, FRM_MODBUS,  FRM_APP, FRM_SMS, FRM_DATA } t_frames;

typedef enum { sock_OPEN = 0, sock_CLOSE, sock_UNKNOWN, sock_TOUT } t_socket_status;
typedef enum { net_OPEN = 0, net_CLOSE, net_UNKNOWN, net_TOUT } t_network_status;

static bool state_online_entry(void);
static bool state_online_auth(void);
static bool state_online_global(void);
static bool state_online_base(void);
static bool state_online_analog(void);
static bool state_online_digital(void);
static bool state_online_counter(void);
static bool state_online_modbus(void);
static bool state_online_sms(void);
static bool state_online_app(void);
static bool state_online_data(void);
static bool state_online_espera(void);
static bool state_online_exit(void);

bool sendFrame( int8_t frame_type );
bool xmit_frame( int8_t frame_type );
bool send_txbuffer(void);
bool rcvd_response(void);
bool xmit_window_data( void );

int8_t socket_status(void);
int8_t socket_open(void);
int8_t socket_close(void);

int8_t netservice_status(void);
int8_t netservice_open(void);
int8_t netservice_close(void);

static void read_IPADDRES(void);
int16_t prepare_header( int8_t frame_type );
int16_t prepare_tail(int16_t pos );

static bool process_rsp_auth(void);
static bool process_rsp_global(void);
static bool process_rsp_base(void);
static bool process_rsp_analog(void);
static bool process_rsp_digital(void);
static bool process_rsp_counters(void);
static bool process_rsp_app(void);
static bool process_rsp_modbus(void);
static bool process_rsp_data(void);
static bool process_rsp_sms(void);

uint16_t datos_pendientes_transmitir(void);
void data_resync_clock(bool force_adjust);
void data_process_response_MBUS(void);
void data_process_response_PILOTO(void);
void data_process_response_STEPPER(void);

bool f_send_init_frame_base;
bool f_send_init_frame_analog;
bool f_send_init_frame_digital;
bool f_send_init_frame_counters;
bool f_send_init_frame_modbus;
bool f_send_init_frame_app;
bool f_send_init_frame_sms;

bool reset_datalogger;

void DEBUG_del_bd(uint16_t max_rcsd_in_bd, uint16_t min_rcsd_in_bd );

bool check_frame_integrity(void);

#define MAX_MBUS_CH_LIST 4

bool modbus_conf_channels[MODBUS_CHANNELS];
int8_t modbus_ch_list[MAX_MBUS_CH_LIST];


//------------------------------------------------------------------------------------
int8_t tkXComms_PRENDIDO_ONLINE(void)
{

int8_t state;
int8_t tryes_substate;

	xprintf_PD( DF_COMMS, PSTR("COMMS: state prendidoONLINE.\r\n"));

	state = ONLINE_ENTRY;
	tryes_substate = 0;
	SET_MBUS_STATUS_FLAG_NONE();
	INIT_MBUS_TAG();

	// loop
	for( ;; )
	{
		u_wdg_kick(WDG_COMMS, 300);
		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );

		// Verifico no entrar mas de N veces al mismo subestado
		if ( tryes_substate++ == MAX_TRYES_SUBSTATE_ONLINE ) {
			state = ONLINE_EXIT;
		}

		switch ( state ) {
		case ONLINE_ENTRY:
			if ( state_online_entry() ) {
				state = ONLINE_AUTH;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_AUTH:
			if ( state_online_auth() ) {
				state = ONLINE_GLOBAL;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_GLOBAL:
			if ( state_online_global() ) {
				state = ONLINE_BASE;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_BASE:
			if ( ! f_send_init_frame_base ) {
				state = ONLINE_ANALOG;
				tryes_substate = 0;
			} else	if ( state_online_base() ) {
				state = ONLINE_ANALOG;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_ANALOG:
			if ( ! f_send_init_frame_analog ) {
				state = ONLINE_DIGITAL;
				tryes_substate = 0;
			} else if ( state_online_analog() ) {
				state = ONLINE_DIGITAL;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_DIGITAL:
			if ( ! f_send_init_frame_digital ) {
				state = ONLINE_COUNTER;
				tryes_substate = 0;
			} else if ( state_online_digital() ) {
				state = ONLINE_COUNTER;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_COUNTER:
			if ( ! f_send_init_frame_counters ) {
				state = ONLINE_MODBUS;
				tryes_substate = 0;
			} else if ( state_online_counter() ) {
				state = ONLINE_MODBUS;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_MODBUS:
			if ( ! f_send_init_frame_modbus ) {
				state = ONLINE_SMS;
				tryes_substate = 0;
			} else if ( state_online_modbus() ) {
				state = ONLINE_SMS;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_SMS:
			if ( ! f_send_init_frame_sms ) {
				state = ONLINE_APP;
				tryes_substate = 0;
			} else if ( state_online_sms() ) {
				state = ONLINE_APP;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_APP:
			if ( ! f_send_init_frame_app ) {
				state = ONLINE_DATA;
				tryes_substate = 0;
				// Cuando entro en el modo DATA indico el estado
				modbus_report_status(BIT_POS_MODEMSTATUS, BIT_VAL_MODEM_ONLINE );
			} else if ( state_online_app() ) {
				state = ONLINE_DATA;
				tryes_substate = 0;
				modbus_report_status(BIT_POS_MODEMSTATUS, BIT_VAL_MODEM_ONLINE );
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_DATA:
			if ( state_online_data() ) {
				state = ONLINE_ESPERA;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			};
			break;

		case ONLINE_ESPERA:
			u_wdg_kick(WDG_COMMS, 300);
			if ( state_online_espera() ) {
				state = ONLINE_DATA;
				tryes_substate = 0;
			} else {
				state = ONLINE_EXIT;
			};
			break;

		case ONLINE_EXIT:
			modbus_report_status(BIT_POS_MODEMSTATUS, BIT_VAL_MODEM_OFFLINE );
			state = state_online_exit();
			// Cambio de estado.
			// Del modo ONLINE siempre salgo a APAGADO.
			netservice_close();
			return(APAGADO);
			break;

		default:
			xprintf_P( PSTR("COMMS: prendidoONLINE ERROR !!.\r\n\0"));
			return(APAGADO);
			break;
		}
	}

	return(-1);

}
//------------------------------------------------------------------------------------
static bool state_online_data(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DATA in\r\n"));

	if ( reset_datalogger) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reset...\r\n"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */
	}

	while ( datos_pendientes_transmitir() != 0 ) {

		if ( sendFrame( FRM_DATA )) {
			process_rsp_data();
			socket_close();
			exit_code = true;
		} else {
			exit_code = false;
			break;
		}
	}

	// No hay mas datos pendientes de trasmitir
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DATA out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_espera(void)
{
	// Cuando estoy en modo continuo, espero prendido por nuevos datos para transmitir

bool exit_code = false;
int8_t timer = 60;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ESPERA in\r\n"));

	RESET_DEEP_SLEEP();

	// No debo esperar
	if (  MODO_DISCRETO ) {
		goto quit;
	}

	// Loop:
	while (true) {

		vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );
		timer -= 10;

		// Cada 60s salgo.
		if ( timer <= 0) {
			exit_code = true;
			goto quit;
		}

		// Si recibo una senal de datos ready: Salgo
		if ( SPX_SIGNAL( SGN_FRAME_READY )) {
			SPX_CLEAR_SIGNAL( SGN_FRAME_READY );
			xprintf_PD( DF_COMMS, PSTR("COMMS: SGN_FRAME_READY rcvd.\r\n\0"));
			exit_code = true;
			goto quit;
		}

		if ( SPX_SIGNAL( SGN_REDIAL )) {
			SPX_CLEAR_SIGNAL( SGN_REDIAL );
			xprintf_PD( DF_COMMS, PSTR("COMMS: SGN_REDIAL rcvd.\r\n\0"));
			exit_code = true;
			goto quit;
		}

	 }

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ESPERA out\r\n"));

quit:

	SMS_rxcheckpoint();

	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_sms(void)
{
bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:SMS in\r\n"));

	if (sendFrame( FRM_SMS )) {
		process_rsp_sms();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:SMS out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_app(void)
{
bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:APP in\r\n"));

	if (sendFrame( FRM_APP )) {
		process_rsp_app();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:APP out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_modbus(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;
uint8_t i;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS in\r\n"));

	for ( i=0; i < MODBUS_CHANNELS; i = i+4 ) {
		modbus_ch_list[0] = i;
		modbus_ch_list[1] = i+1;
		modbus_ch_list[2] = i+2;
		modbus_ch_list[3] = i+3;

		// El sendFrame()->xmit_frame usa la modbus_ch_list
		if (sendFrame( FRM_MODBUS )) {
			process_rsp_modbus();
			socket_close();
		}
	}

	// Si hay canales que no se pudieron configurar, lo reintento ahora

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_counter(void)
{
bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:COUNTER in\r\n"));

	if (sendFrame( FRM_COUNTER )) {
		process_rsp_counters();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:COUNTER out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_digital(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DIGITAL in\r\n"));

	if (sendFrame( FRM_DIGITAL )) {
		process_rsp_digital();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DIGITAL out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_analog(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ANALOG in\r\n"));

	if (sendFrame( FRM_ANALOG )) {
		process_rsp_analog();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ANALOG out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_base(void)
{
bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE in\r\n"));

	if (sendFrame( FRM_BASE )) {
		process_rsp_base();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_global(void)
{
bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:GLOBAL in\r\n"));

	if (sendFrame( FRM_GLOBAL )) {
		process_rsp_global();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:GLOBAL out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_auth(void)
{

	// Intento enviar y procesar un frame de AUTH.
	// Si lo logro paso a procesar el siguiente tipo de frame.

bool exit_code = false;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:AUTH in\r\n"));

	if (sendFrame( FRM_AUTH )) {

		if ( ! process_rsp_auth() ) {
			// ERROR de autorizacion. Paso a modo discreto y
			// espero 1h.
			systemVars.comms_conf.timerDial = 3600;
			xprintf_P( PSTR("\r\n****************************************************************\r\n"));
			xprintf_P( PSTR("COMMS: FATAL ERROR: El dlgid / uid no esta definido en la BD.!!!\r\n"));
			xprintf_P( PSTR("****************************************************************\r\n"));
			exit_code = false;
		} else {
			exit_code = true;
		}
		socket_close();
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:AUTH out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_entry(void)
{

uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ENTRY in\r\n"));

	// Inicializo todo lo necesario al modo prendido ONLINE
	f_send_init_frame_base = false;
	f_send_init_frame_analog = false;
	f_send_init_frame_digital = false;
	f_send_init_frame_counters = false;
	f_send_init_frame_modbus = false;
	f_send_init_frame_app = false;
	f_send_init_frame_sms = false;

	reset_datalogger = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ENTRY out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(true);
}
//------------------------------------------------------------------------------------
static bool state_online_exit(void)
{
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:EXIT\r\n"));
	return(true);
}
/* ------------------------------------------------------------------------------------
 FSM SendFrames
 ------------------------------------------------------------------------------------

 * Para enviar un frame implementamos una FSM que se encarga de los sockets, activar el
 * PDP ( netopen ), enviar el frame, esperar y procesar la respuesta.
 * La flag complete_frame nos indica si vienen mas partes o es la unica (ultima )
 * Si vienen mas partes no espero la respuesta sino que salgo a enviar el resto.
 *
 */
bool sendFrame( int8_t frame_type )
{

int8_t state;
int8_t sock_status;
int8_t net_status;
int8_t tryes;
bool ret_code;

	state = SF_ENTRY;
	tryes = 0;
	ret_code = false;

	for (;;) {

		vTaskDelay( ( TickType_t)( 50 / portTICK_RATE_MS ) );

		switch (state) {
		case SF_ENTRY:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stENTRY\r\n"));
			// Controlo no quedar en un loop infinito
			tryes++;
			if ( tryes == 8 ) {
				ret_code = false;
				state = SF_EXIT;
			} else {
				state = SF_SOCK_STATUS;
			}
			break;

		case SF_SOCK_STATUS:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stSOCK_STATUS\r\n"));
			// Chequeo si tengo un socket abierto
			sock_status = socket_status();
			if ( sock_status == sock_OPEN ) {
				state = SF_SEND;
			} else {
				state = SF_NET_STATUS;
			}
			break;

		case SF_NET_STATUS:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stNET_STATUS\r\n"));
			// Chequeo si el servicio de sockets esta activo
			net_status = netservice_status();
			if ( net_status == net_OPEN ) {
				read_IPADDRES();
				state = SF_SOCK_OPEN;
			} else {
				memset( xCOMMS_stateVars.ip_assigned, '\0', sizeof(xCOMMS_stateVars.ip_assigned) );
				netservice_open();
				state = SF_ENTRY;
			}
			break;

		case SF_SOCK_OPEN:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stSOCK_OPEN\r\n"));
			sock_status = socket_open();
			if ( sock_status == sock_OPEN ) {
				state = SF_ENTRY;
			} else {
				netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_SEND:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stSEND\r\n"));
			// Envio el frame
			if ( xmit_frame( frame_type ) ) {
				state = SF_RSP;
			} else {
				socket_close();
				//netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_RSP:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stRSP\r\n"));
			// Espero la respuesta
			if ( rcvd_response() ) {
				ret_code = true;
				state = SF_EXIT;
			} else {
				socket_close();
				//netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_EXIT:
			xprintf_PD( DF_COMMS, PSTR("COMMS: FSMsendFrame stEXIT\r\n"));
			return(ret_code);
			break;
		}

	}

	return(false);
}
//------------------------------------------------------------------------------------
int8_t socket_status(void)
{

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CIPOPEN?\r");

		if (cmd_rsp	== ATRSP_OK ) {

			if ( gprs_check_response( 0, "CIPOPEN: 0,\"TCP\"")  ) {
				//xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (open) OK (t=%d)\r\n\0"), tryes );
				return ( sock_OPEN );
			}

			if ( gprs_check_response( 0, "CIPOPEN: 0") ) {
				//xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (close) OK (t=%d)\r\n\0"), tryes );
				return ( sock_CLOSE );
			}

		} else if ( cmd_rsp == ATRSP_ERROR ) {
			// +IP ERROR: Operation not supported
			//
			// ERROR

			if ( gprs_check_response( 0, "+IP ERROR:") ) {
				//xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (unknown) OK (t=%d) (rsp=%d) \r\n\0"), tryes, cmd_rsp );
				return ( sock_UNKNOWN );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketSTATUS retry (%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );

	}

	// No puedo en 3 veces responder.
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (uknown) ERROR\r\n"));

	return( sock_UNKNOWN );


}
	//------------------------------------------------------------------------------------
int8_t socket_open(void)
{
	// Abre un socket en el servidor remoto.
	// Reintenta hasta 3 veces

int8_t tryes;
char strapn[48];
int8_t cmd_rsp;
int8_t timeout;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN in\r\n"));

	memset(strapn,'\0', sizeof(strapn));
	snprintf_P( strapn, sizeof(strapn), PSTR("AT+CIPOPEN=0,\"TCP\",\"%s\",%s\r"), comms_conf.server_ip_address, comms_conf.server_tcp_port);

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {
		XPRINT_ELAPSED_wTAG( DF_COMMS, 'A', init_ticks);
		cmd_rsp = FSM_sendATcmd( 5, strapn );
		XPRINT_ELAPSED_wTAG( DF_COMMS, 'B', init_ticks);

		if (cmd_rsp	== ATRSP_OK ) {
			// Respondio al comando: Espero el resultado del sockopen.
			// Espero la respuesta del socket abierto
			for ( timeout = 0; timeout < SOCKET_OPEN_TIMEOUT_secs; timeout++) {
				vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
				if ( gprs_check_response( 0, "+CIPOPEN: 0,0" ) ) {
					gprs_print_RX_buffer();
					xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
					XPRINT_ELAPSED_wTAG( DF_COMMS, 'C', init_ticks);
					return ( sock_OPEN );
				}
			}
			XPRINT_ELAPSED_wTAG( DF_COMMS, 'D', init_ticks);

		} else 	if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque ya esta abierto
			if ( gprs_check_response( 0, "+CIPOPEN:") ) {
				gprs_print_RX_buffer();
				xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
				XPRINT_ELAPSED_wTAG( DF_COMMS, 'E', init_ticks);
				return ( sock_OPEN );
			}
		}

		// TIMEOUT del comando o la espera de socket open.
		// Reintento dando antes un AT.
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN out (unknown) ERROR (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	XPRINT_ELAPSED_wTAG( DF_COMMS, 'F', init_ticks);
	return( sock_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t socket_close(void)
{
	// Cierra el socket
	// Reintenta hasta MAX_TRYES_SOCKET_CLOSE veces controlando que ya no este cerrado
	//

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < MAX_TRYES_SOCKET_CLOSE; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+CIPCLOSE=0\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			//xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
			xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out (close) OK (%d)\r\n\0"), tryes );
			return ( sock_CLOSE );

		} else 	if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque el socket ya esta cerrado
			if ( gprs_check_response( 0, "+CIPCLOSE:") ) {
				//xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out (close) OK (%d)\r\n\0"), tryes );
				return ( sock_CLOSE );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketCLOSE retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );

	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	//xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out (unknown) ERROR\r\n"));

	return( sock_UNKNOWN );


}
//------------------------------------------------------------------------------------
int8_t netservice_status(void)
{

int8_t tryes;
int8_t cmd_rsp;

	xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETOPEN?\r" );

		if (cmd_rsp	== ATRSP_OK ) {

			if ( gprs_check_response( 0, "+NETOPEN: 1") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out (open) OK (%d)\r\n\0"), tryes );
				return ( net_OPEN );
			}

			if ( gprs_check_response( 0, "+NETOPEN: 0") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out (close) OK (%d)\r\n\0"), tryes );
				return ( net_CLOSE );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netSTATUS retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );

	}
	// No puedo en 3 veces responder.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out (unknown) ERROR\r\n"));
	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t netservice_open(void)
{
	// Inicia el servicio de sockets abriendo un socket. ( no es una conexion sino un
	// socket local por el cual se va a comunicar. )
	// Activa el contexto y crea el socket local
	// La red asigna una IP.
	// Puede demorar unos segundos por lo que espero para chequear el resultado
	// y reintento varias veces.
	// OK si el comando responde ( no importa la respuesta )
	//
	//AT+NETOPEN
	//OK
	//
	//+NETOPEN: 0
	// Si ya esta abierta, responde con un
	// +IP ERROR: Network is already opened
	//
	// ERROR


int8_t tryes;
int8_t cmd_rsp;
int8_t timeout;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN in\r\n"));

	for ( tryes = 0; tryes < 3; tryes++ ) {

		// Envio el comando y espero hasta 10s por una respuesta OK o ERROR.
		cmd_rsp = FSM_sendATcmd( 10, "AT+NETOPEN\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			// Respondio al comando: Espero el resultado del netopen.
			// Espero que se abra la conexion
			for ( timeout = 0; timeout < 45; timeout++) {
				vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
				if ( gprs_check_response( 0, "+NETOPEN: 0" ) ) {
					gprs_print_RX_buffer();
					xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN out (open) OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
					return ( net_OPEN );
				}
			}

		} else if (cmd_rsp== ATRSP_ERROR ) {
			// Respondio al comando con error: Tal vez ya esta abierto.
			if ( gprs_check_response( 0, "Network is already opened") ) {
				gprs_print_RX_buffer();
				XPRINT_ELAPSED(init_ticks);
				xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN out (open) OK\r\n\0") );
				return ( net_OPEN );
			}
		}

		// Timeout o rsp. ERROR.
		// Reintento dando antes un AT.
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responder: Salgo a apagar y prender.
	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN out (unknown) ERROR. (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t netservice_close(void)
{
	/*
	 *  Cierra el servicio local de sockets
	 *  Reintenta hasta 3 veces controlando que ya no este cerrado.
	 *  sent->AT+NETCLOSE
	 *  cmd>read gprs rsp
	 *
	 *  GPRS: rxbuff>
	 *  AT+NETCLOSE
	 *  OK
	 *
	 *  +NETCLOSE: 0
	 *
	 *  [34]
	 */


int8_t tryes;
int8_t cmd_rsp;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETCLOSE\r" );
		if (cmd_rsp	== ATRSP_OK ) {

			vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
			if ( gprs_check_response( 0, "+NETCLOSE: 0" ) ) {
				gprs_print_RX_buffer();
				xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (close) OK (%d) (%.3f)\r\n"), tryes, ELAPSED_TIME_SECS(init_ticks));
				return ( net_CLOSE );
			}

		} else if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque ya esta cerrado
			if ( gprs_check_response( 0, "Network is already closed") ) {
				gprs_print_RX_buffer();
				XPRINT_ELAPSED(init_ticks);
				xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (close) OK (%d)\r\n\0"), tryes );
				return ( net_CLOSE );
			}

		}

		// Timeout o rsp. ERROR.
		// Reintento dando antes un AT.
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netCLOSE retry (%d) (%.3f)\r\n"), tryes, ELAPSED_TIME_SECS(init_ticks));
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responde.
	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (unknown) ERROR. (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
bool xmit_frame( int8_t frame_type )
{

	// Prepara el frame ( header, data, tail ) y llama a send_txbuffer para enviarlo
	// por el modem

bool ret_code = false;
int16_t i;
uint32_t init_ticks = sysTicks;

	switch ( frame_type ) {
	case FRM_AUTH:
		// Preparo el frame
		i = prepare_header(FRM_AUTH);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:AUTH;UID:%s;" ),NVMEE_readID() );
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_GLOBAL:
		i = prepare_header(FRM_GLOBAL);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:GLOBAL;NACH:%d;NDCH:%d;NCNT:%d;" ),ANALOG_CHANNELS, DINPUTS_CHANNELS, COUNTER_CHANNELS );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("IMEI:%s;" ), xCOMMS_stateVars.gprs_imei );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("SIMID:%s;CSQ:%d;WRST:%02X;" ), xCOMMS_stateVars.gprs_ccid, xCOMMS_stateVars.csq, wdg_resetCause );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("BASE:0x%02X;" ), u_base_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("AN:0x%02X;" ), ainputs_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("DG:0x%02X;" ), dinputs_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CNT:0x%02X;" ), counters_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("MB:0x%02X;" ), modbus_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("APP:0x%02X;" ), aplicacion_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("SMS:0x%02X;" ), sms_hash() );
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_BASE:
		i = prepare_header(FRM_BASE);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_BASE;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_ANALOG:
		i = prepare_header(FRM_ANALOG);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_ANALOG;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_DIGITAL:
		i = prepare_header(FRM_DIGITAL);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_DIGITAL;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_COUNTER:
		i = prepare_header(FRM_COUNTER);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_COUNTER;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_MODBUS:
		i = prepare_header(FRM_MODBUS);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_MBUS;DATA:[%d,%d,%d,%d];"), modbus_ch_list[0],modbus_ch_list[1],modbus_ch_list[2],modbus_ch_list[3]);
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_APP:
		i = prepare_header(FRM_COUNTER);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_APP;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_SMS:
		i = prepare_header(FRM_SMS);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_SMS;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_DATA:
		// Como es un frame multiparte lo hacemos en una funcion aparte
		ret_code = xmit_window_data();
		break;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:xmit_frame out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool send_txbuffer(void)
{
	// Envia el frame que esta en txbuffer por el modem.
	// Calcula el tama??o y envia un CIPSEND.
	// Cuando recibe el prompt envia el resto y sale

uint16_t size = 0;
char str_sendCmd[24];
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer\r\n"));

	// Solicito el prompt para transmitir
	memset(str_sendCmd,'\0', sizeof(str_sendCmd));
	size = strlen(gprs_txbuffer.buffer);
	snprintf_P( str_sendCmd, sizeof(str_sendCmd), PSTR("AT+CIPSEND=0,%d\r"),size);
	FSM_sendATcmd( 0, str_sendCmd );

	// Espero el prompt 1000 ms.
	if ( ! gprs_check_response( 20, ">") ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer out. (SEND ERROR No prompt) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
		gprs_print_RX_buffer();
		return(false);
	}

	gprs_print_RX_buffer();

	// Envio el frame. El buffer es mayor que lo que maneja xprintf por lo que lo envio directo !!!
	xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer send\r\n"));
	gprs_flush_RX_buffer();
	sxprintf_D( fdGPRS, DF_COMMS , gprs_txbuffer.buffer, size );
	//sxprintf_D( fdGPRS, true , gprs_txbuffer.buffer, size );

	// Espero la confirmacion del modem hasta 2000 msecs. No borro el RX buffer !!!.
	// Si el socket se cierra recibo +IPCLOSE: 0,1.
	if ( gprs_check_response( 20, "+CIPSEND: 0,") ) {
		gprs_print_RX_buffer();

		// Borro el buffer para que quede listo a recibir la respuesta
		gprs_flush_RX_buffer();

		xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer out. (OK) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
		return(true);
	}

	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer out (SEND ERROR No response) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(false);

}
//------------------------------------------------------------------------------------
bool rcvd_response(void)
{
	// Espero una respuesta del server.
	// Si es correcto el formato salgo y dentro de c/state la analizo.

int8_t timeout;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response in.\r\n"));

	// Espero hasta 10s.
	for ( timeout = 0; timeout < 100; timeout++) {
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		if ( gprs_check_response( 0, "RECV FROM:" ) ) {
			if ( gprs_check_response( 0, "</h1>" ) ) {

				if (check_frame_integrity()) {
					xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response out (OK) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
					return(true);
				} else {
					xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response out (ERROR CKS) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
					return(false);
				}
			}

		} else if ( gprs_check_response( 0, "+IPCLOSE:" ) ) {
			gprs_print_RX_buffer();
			xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response out (ERROR) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
			return(false);
		}
		// Espero
	}

	// TIMEOUT:
	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response out (TIMEOUT) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));;
	return(false);

}
//------------------------------------------------------------------------------------
bool xmit_window_data_ori( void )
{
	/*
	 * Transmite un frame de datos multiparte.
	 * Envia el header, luego c/linea y finalmente el tail
	 * DLGID=PAYCTRL01&TYPE=DATA&VER=3.0.7.a&PLOAD=
	 * CTL:1;DATE:210315;TIME:151857;MPA:12.34;MPB:-19.80;MDIN:2135;
	 * CTL:2;DATE:210315;TIME:151911;MPA:12.34;MPB:-19.80;MDIN:2135;
	 *
	 */

size_t bRead;
FAT_t fat;
st_dataRecord_t dataRecord;
bool ret_code = false;
uint8_t registros_trasmitidos = 0;

	// Para que no se resetee x wdg si hay muchos datos pendientes.

	u_wdg_kick(WDG_COMMS, 120);

	// Header
	prepare_header(FRM_DATA);
	ret_code = send_txbuffer();
	if ( !ret_code)
		return (ret_code);

	// Window data
	FF_rewind();
	while ( (  datos_pendientes_transmitir() > 0 ) && ( registros_trasmitidos < MAX_DATA_WINDOW_SIZE ) ) {

		memset ( &fat, '\0', sizeof(FAT_t));
		memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
		bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
		if ( bRead == 0 )
			return(false);
		FAT_read(&fat);

		// Serializo el dataRecors en txbuffer
		gprs_txbuffer_reset();
		data_sprintf_inputs( gprs_txbuffer.buffer , &dataRecord, fat.rdPTR );
		registros_trasmitidos++;
		ret_code = send_txbuffer();
		if ( !ret_code)
			return (ret_code);

		// Espero 50ms entre records
		vTaskDelay( (portTickType)( 10 / portTICK_RATE_MS ) );
	}

	// Tail
	prepare_tail(0);
	ret_code = send_txbuffer();
	if ( !ret_code)
		return (ret_code);

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool xmit_window_data_1frame( void )
{
	/*
	 * VER01: Cargo en el buffer de transmision 1 solo frame de datos
	 */

size_t bRead;
FAT_t fat;
st_dataRecord_t dataRecord;
bool ret_code = false;
uint16_t i;

	// Para que no se resetee x wdg si hay muchos datos pendientes.

	u_wdg_kick(WDG_COMMS, 120);

	FF_rewind();
	if ( datos_pendientes_transmitir() > 0 ) {
		// Header
		i = prepare_header(FRM_DATA);
		// Datos
		memset ( &fat, '\0', sizeof(FAT_t));
		memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
		bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
		if ( bRead == 0 )
			return(false);
		FAT_read(&fat);
		data_sprintf_inputs( &gprs_txbuffer.buffer[i] , &dataRecord, fat.rdPTR );
		i = strnlen(gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN );
		// Tail
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
	}

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool xmit_window_data( void )
{
	/*
	 * VER01: Cargo en el buffer de transmision hasta 2 frames si hay lugar
	 *
	 */

size_t bRead;
FAT_t fat;
st_dataRecord_t dataRecord;
bool ret_code = false;
uint16_t i;
uint16_t datos_pendientes;
uint16_t freespace = 0;
uint16_t payload_length;
uint8_t header_length;

	// Para que no se resetee x wdg si hay muchos datos pendientes.

	u_wdg_kick(WDG_COMMS, 120);

	FF_rewind();
	datos_pendientes = datos_pendientes_transmitir();

	if ( datos_pendientes > 0 ) {
		// Header
		i = prepare_header(FRM_DATA);
		header_length = i;
		freespace = GPRS_TXBUFFER_LEN - header_length;
		//xprintf_P(PSTR("FRAME_A: [i=%d][fsp=%d]\r\n"),i, freespace);
		//
		// Primer Frame
		// Datos
		memset ( &fat, '\0', sizeof(FAT_t));
		memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
		bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
		if ( bRead == 0 )
			return(false);
		FAT_read(&fat);
		data_sprintf_inputs( &gprs_txbuffer.buffer[i] , &dataRecord, fat.rdPTR );
		i = strnlen(gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN );
		// Este i tiene el primer frame + header
		payload_length = i - header_length;
		freespace = GPRS_TXBUFFER_LEN - i;
		//xprintf_P(PSTR("FRAME_B=[i=%d][fsp=%d]\r\n"), i, freespace );
		//xprintf_P(PSTR("FRAME_B=[%s]\r\n"), gprs_txbuffer.buffer );
		//
		// Siguientes frames
		while  (  ( datos_pendientes > 1 ) && ( freespace > (payload_length + 45) ) ) {
			// Agrego otro frame
			memset ( &fat, '\0', sizeof(FAT_t));
			memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
			bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
			if ( bRead == 0 )	{// Si no tengo mas datos, salgo: Corrige el BUG001
				xprintf_PD( DF_COMMS, PSTR("DEBUG: No more frames in windown\r\n"));
				break;
			}
			FAT_read(&fat);
			data_sprintf_inputs( &gprs_txbuffer.buffer[i] , &dataRecord, fat.rdPTR );
			i = strnlen(gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN );
			freespace = GPRS_TXBUFFER_LEN - i;
			//xprintf_P(PSTR("FRAME_C=[i=%d][fsp=%d]\r\n"), i, freespace );
			//xprintf_P(PSTR("FRAME_C=[%s]\r\n"),gprs_txbuffer.buffer );
		}
		// Tail
		i +=  prepare_tail(i);
		//xprintf_P(PSTR("FRAME_D=[i=%d][%s]\r\n"), i, gprs_txbuffer.buffer );
		ret_code = send_txbuffer();

	}

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool tesmodbus_config_channelst_xmit_window_data( void )
{
	/*
	 * VER02: Pruebo transitir al menos 2 frames de datos
	 * VER01: Cargo en el buffer de transmision 1 solo frame de datos
	 * 	// Preparo el frame
		i = prepare_header(FRM_AUTH);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:AUTH;UID:%s;" ),NVMEE_readID() );
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
	 */

size_t bRead;
FAT_t fat;
st_dataRecord_t dataRecord;
bool ret_code = false;
uint16_t i;
uint16_t datos_pendientes;
uint16_t freespace = 0;
uint16_t payload_length;
uint8_t header_length;

	// Para que no se resetee x wdg si hay muchos datos pendientes.

	u_wdg_kick(WDG_COMMS, 120);

	FF_rewind();
	datos_pendientes = datos_pendientes_transmitir();
	xprintf_P(PSTR("DATOS PENDIENTES=%d\r\n"), datos_pendientes);

	if ( datos_pendientes > 0 ) {
		// Header
		i = prepare_header(FRM_DATA);
		header_length = i;
		freespace = GPRS_TXBUFFER_LEN - header_length;
		xprintf_P(PSTR("FRAME_A: [i=%d][fsp=%d]\r\n"),i, freespace);
		//
		// Primer Frame
		// Datos
		memset ( &fat, '\0', sizeof(FAT_t));
		memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
		bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
		if ( bRead == 0 )
			return(false);
		FAT_read(&fat);
		data_sprintf_inputs( &gprs_txbuffer.buffer[i] , &dataRecord, fat.rdPTR );
		i = strnlen(gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN );
		// Este i tiene el primer frame + header
		payload_length = i - header_length;
		freespace = GPRS_TXBUFFER_LEN - i;
		xprintf_P(PSTR("FRAME_B=[i=%d][fsp=%d]\r\n"), i, freespace );
		xprintf_P(PSTR("FRAME_B=[%s]\r\n"), gprs_txbuffer.buffer );
		//
		// Siguientes frames
		while  (  ( datos_pendientes > 1 ) && ( freespace > (payload_length + 45) ) ) {
			// Agrego otro frame
			memset ( &fat, '\0', sizeof(FAT_t));
			memset ( &dataRecord, '\0', sizeof( st_dataRecord_t));
			bRead = FF_readRcd( &dataRecord, sizeof(st_dataRecord_t));
			FAT_read(&fat);
			data_sprintf_inputs( &gprs_txbuffer.buffer[i] , &dataRecord, fat.rdPTR );
			i = strnlen(gprs_txbuffer.buffer, GPRS_TXBUFFER_LEN );
			freespace = GPRS_TXBUFFER_LEN - i;
			xprintf_P(PSTR("FRAME_C=[i=%d][fsp=%d]\r\n"), i, freespace );
			xprintf_P(PSTR("FRAME_C=[%s]\r\n"),gprs_txbuffer.buffer );
		}
		// Tail
		i +=  prepare_tail(i);
		xprintf_P(PSTR("FRAME_D=[i=%d][%s]\r\n"), i, gprs_txbuffer.buffer );

	}
	xprintf_P(PSTR("TEST end\r\n"));

}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES
//------------------------------------------------------------------------------------
static void read_IPADDRES(void)
{

int8_t cmd_rsp;
char *ts = NULL;
char c = '\0';
char *ptr = NULL;


	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE: IPADDRESS\r\n"));

	memset( xCOMMS_stateVars.ip_assigned, '\0', sizeof(xCOMMS_stateVars.ip_assigned) );
	cmd_rsp = FSM_sendATcmd( 5, "AT+IPADDR\r" );
	if (cmd_rsp	== ATRSP_OK ) {
		// Extraigo la IP
		if ( gprs_check_response( 0, "+IPADDR:") ) {
			ptr = xCOMMS_stateVars.ip_assigned;
			ts = strstr( gprs_rxbuffer.buffer, "+IPADDR:");
			ts += 9;
			while ( (c = *ts) != '\r') {
				*ptr++ = c;
				ts++;
			}
			*ptr = '\0';
			xprintf_PD( DF_COMMS, PSTR("COMMS: IPADDR [%s]\r\n\0"), xCOMMS_stateVars.ip_assigned );
		}
	}
}
//------------------------------------------------------------------------------------
int16_t prepare_header( int8_t frame_type )
{
int16_t i;

	gprs_txbuffer_reset();
	// Frames de DATA
	if ( frame_type == FRM_DATA ) {
		if (IS_MBUS_STATUS_FLAG_ACK() ) {
			i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=DATA&VER=%s&PLOAD=ACK:%d;" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV, modbus_get_mbtag() );
		} else if ( IS_MBUS_STATUS_FLAG_NACK() ) {
			// La orden anterior fue procesada correctamente
			i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=DATA&VER=%s&PLOAD=NACK:%d;" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV, modbus_get_mbtag() );
		} else {
			// IS_MBUS_STATUS_FLAG_NONE()
			i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=DATA&VER=%s&PLOAD=" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		}

	} else {
		// Frames de INIT / CONTROL
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s&PLOAD=" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
	}
	return(i);
}
//------------------------------------------------------------------------------------
int16_t prepare_tail(int16_t pos )
{
int16_t i;

	if ( pos == 0 )
		gprs_txbuffer_reset();

	i = sprintf_P( &gprs_txbuffer.buffer[pos], PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n") );
	return(i);
}
//------------------------------------------------------------------------------------
bool check_frame_integrity(void)
{
	/*
	 * Toma el frame recibido entre <h1> y ;CKS:xx<\h1> y calcula el checksum
	 * Lo compara con el que trae el frame e indica si esta bien o no
	 * En las versiones anteriores no trae el CKS. En este caso siempre retorno TRUE
	 */

uint8_t calculated_cks;
uint8_t frame_cks;
char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
char *ts = NULL;
char *sof;
char *eof;
char *p;

	// Paso1: Extraigo el checksum del frame:
	if ( gprs_check_response( 0, "CKS") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "CKS");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);	    // CKS
		token = strsep(&stringp,delim);	 	// cks_val
		frame_cks = atoi(token);
		//xprintf_P( PSTR("COMMS: check_frame_integrity frame=[%d]\r\n"), frame_cks );
	} else {
		//xprintf_P( PSTR("COMMS: check_frame_integrity. No cks.\r\n"));
		return(true);
	}

	// Paso2: Calculo el checksum del frame
	sof = strstr(gprs_rxbuffer.buffer, "<h1>");
	if (sof != NULL) {
		sof += 4;
	} else {
		xprintf_P( PSTR("COMMS: check_frame_integrity ERROR: No sof.\r\n"));
		return(false);
	}

	eof = strstr(gprs_rxbuffer.buffer, "CKS");
	if ( eof != NULL ) {
		//eof--;
	} else {
		xprintf_P( PSTR("COMMS: check_frame_integrity ERROR: No eof.\r\n"));
		return(false);
	}

	p = sof;
	calculated_cks = 0;
	while ( p != eof) {
		calculated_cks = (calculated_cks + *p) % 256;
		p++;
	}

	//xprintf_P( PSTR("COMMS: check_frame_integrity calc=[%d]\r\n"), calculated_cks );
	if ( calculated_cks == frame_cks ) {
		return(true);
	}
	xprintf_P( PSTR("COMMS: check_frame_integrity FAILED !!\r\n"));
	xprintf_P( PSTR("COMMS: check_frame_integrity frame=[%d]\r\n"), frame_cks );
	xprintf_P( PSTR("COMMS: check_frame_integrity calc=[%d]\r\n"), calculated_cks );
	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_auth(void)
{
	// Recibimos un frame de autorizacion que indica si los parametros UID/DLGID son
	// correctos o debe reconfigurar el DLGID.
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:OK
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:RECONF;DLGID:TEST01
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:ERROR_DS

	/*
	 * Version 4.0.4b: Agregamos el checksum al final
	 * <html><body><h1>TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:OK;CKS:91</h1></body></html>
	 */

char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
char *ts = NULL;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_auth in\r\n"));
	gprs_print_RX_buffer();

	if ( gprs_check_response( 0, "AUTH") ) {

		if ( gprs_check_response( 0, "ERROR_DS") ) {
			// No autorizado
			return(false);
		}

		if ( gprs_check_response( 0, "STATUS:OK") ) {
			return(true);
		}

		if ( gprs_check_response( 0, "RECONF") ) {
			// Autorizado. Debo reconfigurar el DLGID
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, "RECONF;");
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			token = strsep(&stringp,delim);	    // RECONF
			token = strsep(&stringp,delim);	 	// DLGID
			token = strsep(&stringp,delim);	 	// TEST01
			// Copio el dlgid recibido al systemVars.
			memset(comms_conf.dlgId,'\0', sizeof(comms_conf.dlgId) );
			strncpy(comms_conf.dlgId, token, DLGID_LENGTH);
			u_save_params_in_NVMEE();
			xprintf_P( PSTR("COMMS_INIT_AUTH: reconfig DLGID to %s\r\n\0"), comms_conf.dlgId );
			return(true);
		}
	}

	return(false);
}
//------------------------------------------------------------------------------------
static bool process_rsp_global(void)
{

	// Recibimos un frame que trae la fecha y hora y parametros que indican
	// que otras flags de configuraciones debemos prender.
	// GLOBAL;CLOCK:1910120345;BASE;ANALOG;DIGITAL;COUNTERS;RANGE;PSENSOR;APP_A;APP_B;APP_C;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_global in\r\n\0"));
	gprs_print_RX_buffer();

	// CLOCK
	if ( gprs_check_response( 0, "CLOCK") ) {
		data_resync_clock(true);	// Fuerzo ajustar el clock.
	}

	// Flags de configuraciones particulares: BASE;ANALOG;DIGITAL;COUNTERS;
	f_send_init_frame_base = false;
	f_send_init_frame_analog = false;
	f_send_init_frame_digital = false;
	f_send_init_frame_counters = false;
	f_send_init_frame_modbus = false;
	f_send_init_frame_app = false;
	f_send_init_frame_sms = false;

	if ( gprs_check_response( 0, "BASE") ) {
		f_send_init_frame_base = true;
	}

	if ( gprs_check_response( 0, "ANALOG") ) {
		f_send_init_frame_analog = true;
	}

	if ( gprs_check_response( 0, "DIGITAL") ) {
		f_send_init_frame_digital = true;
	}

	if ( gprs_check_response( 0, "COUNTERS") ) {
		f_send_init_frame_counters = true;
	}

	if ( gprs_check_response( 0, "MBUS") ) {
		f_send_init_frame_modbus = true;
	}

	if ( gprs_check_response( 0, "APLICACION") ) {
		f_send_init_frame_app = true;
	}

	if ( gprs_check_response( 0, "SMS") ) {
		f_send_init_frame_sms = true;
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_base(void)
{
	// V401a: TYPE=INIT&PLOAD=CLASS:BASE;TDIAL:0;TPOLL:60;PWST:5;HW_CNT:OPTO;CTRL_SLA:1;CTRL_ADDR:0;
	//
	//	TYPE=INIT&PLOAD=CLASS:BASE;TPOLL:60;TDIAL:60;PWST:5;HW_CNT:OPTO

char *ts = NULL;
char localStr[64] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *tk_ctl_sla = NULL;
char *tk_clt_addr = NULL;
char *delim = ",;:=><";
bool save_flag = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_base in\r\n"));
	gprs_print_RX_buffer();

	// TDIAL
	if ( gprs_check_response( 0, "TDIAL") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "TDIAL");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// TDIAL
		token = strsep(&stringp,delim);		// timerDial
		u_config_timerdial(token);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig TDIAL\r\n"));
	}


	// TPOLL
	if ( gprs_check_response( 0, "TPOLL") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "TPOLL");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// TPOLL
		token = strsep(&stringp,delim);		// timerPoll
		u_config_timerpoll(token);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig TPOLL\r\n"));
	}

	// PWST
	if ( gprs_check_response( 0, "PWST") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "PWST");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PWST
		token = strsep(&stringp,delim);		// timePwrSensor
		ainputs_config_timepwrsensor(token);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig PWRS\r\n"));
	}

	// CNT_HW
	if ( gprs_check_response( 0, "HW_CNT") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "HW_CNT");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// CNT_HW
		token = strsep(&stringp,delim);		// opto/simple

		counters_config_hw(token);
		save_flag = true;
		reset_datalogger = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig COUNTERS_HW\r\n"));
	}

	// CTL_SLAVE
	if ( gprs_check_response( 0, "CTRL_SLA") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "CTRL_SLA");
		strncpy(localStr, ts, sizeof(localStr));
		//xprintf_P(PSTR("DEBUG_CTL_SLA: [%s]\r\n"), localStr);
		stringp = localStr;
		tk_ctl_sla = strsep(&stringp,delim);		// CTL_SLA
		tk_ctl_sla = strsep(&stringp,delim);		// Value
		tk_clt_addr = strsep(&stringp,delim);		// CTRL_ADDR
		tk_clt_addr = strsep(&stringp,delim);		// Value
		//
		modbus_config_chcontrol(tk_ctl_sla,tk_clt_addr );
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Modbus CTL[%s][%s]\r\n"), tk_ctl_sla,tk_clt_addr );
	}


	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_analog(void)
{
	//	CLASS:ANALOG;A0:PA,4,20,0,100;A1:X,0,0,0,0;A2:OPAC,0,20,0,100,A3:NOX,4,20,0,2000;A4:SO2,4,20,0,1000;A5:CO,4,20,0,25;A6:NO2,4,20,0,000;A7:CAU,4,20,0,1000
	// 	PLOAD=CLASS:ANALOG;A0:PA,4,20,0.0,10.0;A1:X,4,20,0.0,10.0;A3:X,4,20,0.0,10.0;
	//  TYPE=INIT&PLOAD=CLASS:ANALOG;A0:PA,4,20,0.0,10.0;A1:X,4,20,0.0,10.0;A2:X,4,20,0.0,10.0;A3:X,4,20,0.0,10.0;A4:X,4,20,0.0,10.0;


char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_name= NULL;
char *tk_iMin= NULL;
char *tk_iMax = NULL;
char *tk_mMin = NULL;
char *tk_mMax = NULL;
char *tk_offset = NULL;
char *delim = ",;:=><";
bool save_flag = false;
uint8_t ch;
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_analog in\r\n\0"));
	gprs_print_RX_buffer();

	// A?
	for (ch=0; ch < ANALOG_CHANNELS; ch++ ) {
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("A%d\0"), ch );

		if ( gprs_check_response( 0, str_base ) ) {
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//A0
			tk_name = strsep(&stringp,delim);		//name
			tk_iMin = strsep(&stringp,delim);		//iMin
			tk_iMax = strsep(&stringp,delim);		//iMax
			tk_mMin = strsep(&stringp,delim);		//mMin
			tk_mMax = strsep(&stringp,delim);		//mMax
			tk_offset = strsep(&stringp,delim);		//offset

			ainputs_config_channel( ch, tk_name ,tk_iMin, tk_iMax, tk_mMin, tk_mMax, tk_offset );
			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig A%d\r\n"), ch);
			save_flag = true;
		}
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_digital(void)
{
	//	PLOAD=CLASS:DIGITAL;D0:DIN0;D1:DIN1;

char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_name= NULL;
char *delim = ",;:=><";
bool save_flag = false;
uint8_t ch;
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_digital in\r\n\0"));
	gprs_print_RX_buffer();

	// D?
	for (ch=0; ch < DINPUTS_CHANNELS; ch++ ) {
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("D%d\0"), ch );

		if ( gprs_check_response( 0, str_base ) ) {
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//D0
			tk_name = strsep(&stringp,delim);		//DIN0

			dinputs_config_channel( ch, tk_name );
			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig D%d\r\n"), ch);
			save_flag = true;
		}
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_counters(void)
{
	//	PLOAD=CLASS:COUNTER;C0:CNT0,1.0,15,1000,0;C1:X,1.0,10,100,1;
	//  PLOAD=CLASS:COUNTER;C0:CNT0,1.0,15,1000,0;C1:X,1.0,10,100,1;

char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_name = NULL;
char *tk_magpp = NULL;
char *tk_pwidth = NULL;
char *tk_period = NULL;
char *tk_sensing = NULL;
char *delim = ",;:=><";
bool save_flag = false;
uint8_t ch;
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_counters in\r\n\0"));
	gprs_print_RX_buffer();

	// C?
	for (ch=0; ch < COUNTER_CHANNELS; ch++ ) {
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("C%d\0"), ch );

		if ( gprs_check_response( 0, str_base ) ) {
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//C0
			tk_name = strsep(&stringp,delim);		//name
			tk_magpp = strsep(&stringp,delim);		//magpp
			tk_pwidth = strsep(&stringp,delim);
			tk_period = strsep(&stringp,delim);
			tk_sensing = strsep(&stringp,delim);

			counters_config_channel( ch ,tk_name , tk_magpp, tk_pwidth, tk_period, tk_sensing );
			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig C%d\r\n\0"), ch);
			save_flag = true;
		}
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);

}
//------------------------------------------------------------------------------------
static bool process_rsp_modbus(void)
{

	/*
	 * TYPE=INIT&PLOAD=CLASS:MBUS;MBWT:10;MB00:T0,2069,2,3,FLOAT,0;MB01:T1,2069,2,3,FLOAT,0;MB02:T2,2062,2,3,FLOAT,0;
	 * MB03:T3,2063,2,3,FLOAT,0;MB04:T4,2064,2,3,FLOAT,0;MB05:X,0,1,3,U16,0;MB06:X,0,1,3,U16,0;MB07:X,0,1,3,U16,0;
	 * MB08:X,0,1,3,U16,0;MB09:X,0,1,3,U16,0;MB10:X,0,1,3,U16,0;MB11:X,0,1,3,U16,0;MB12:X,0,1,3,U16,0;MB13:X,0,1,3,U16,0;
	 * MB14:X,0,1,3,U16,0;MB15:X,0,1,3,U16,0;MB16:X,0,1,3,U16,0;MB17:X,0,1,3,U16,0;MB18:X,0,1,3,U16,0;MB19:X,0,1,3,U16,0;
	 *
	 */


char *ts = NULL;
char localStr[64] = { 0 };
char *stringp = NULL;
char *tk_sla= NULL;
char *tk_mbwt= NULL;
char *tk_name= NULL;
char *tk_address= NULL;
char *tk_size = NULL;
char *tk_fcode = NULL;
char *tk_type = NULL;
char *tk_codec = NULL;
char *tk_pow10 = NULL;
char *delim = ",;:=><";
bool save_flag = false;
uint8_t ch;
char str_base[8];
bool retS = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_modbus in\r\n\0"));
	gprs_print_RX_buffer();

	// MBWT
	if ( gprs_check_response( 0, "MBWT") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "MBWT");
		strncpy(localStr, ts, sizeof(localStr));
		//xprintf_P(PSTR("DEBUG_MBWT: [%s]\r\n"), localStr);
		stringp = localStr;
		tk_mbwt = strsep(&stringp,delim);		// MBWT
		tk_mbwt = strsep(&stringp,delim);		// Value
		modbus_config_waiting_poll_time(tk_mbwt);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Modbus MBWT[%s]\r\n"), tk_mbwt);
	}

	// MBxx?
	for (ch=0; ch < MODBUS_CHANNELS; ch++ ) {
		memset( &str_base, '\0', sizeof(str_base) );
		snprintf_P( str_base, sizeof(str_base), PSTR("MB%02d\0"), ch );

		if ( gprs_check_response( 0, str_base ) ) {
			retS = false;
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			//xprintf_P(PSTR("DEBUG_MB%02d: [%s]\r\n"), ch, localStr);
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//MB00
			tk_name = strsep(&stringp,delim);		//name
			tk_sla = strsep(&stringp,delim);		//sla
			tk_address = strsep(&stringp,delim);	//Address
			tk_size = strsep(&stringp,delim);		//size
			tk_fcode = strsep(&stringp,delim);		//fcode
			tk_type = strsep(&stringp,delim);		//type
			tk_codec = strsep(&stringp,delim);		//codec
			tk_pow10 = strsep(&stringp,delim);		//pow10

			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig MB%02d\r\n"), ch );
			//xprintf_PD( DF_COMMS, PSTR("COMMS: %s,%s,%s,%s,%s,%s,%s,%s\r\n"), tk_name, tk_sla, tk_address, tk_size, tk_fcode, tk_type, tk_codec, tk_pow10 );
			retS = modbus_config_channel( ch, tk_name, tk_sla, tk_address, tk_size, tk_fcode, tk_type, tk_codec, tk_pow10 );
			if ( retS ) {
				save_flag = true;
				/*
				 * Lo marco como que ya lo configure para no pedir mas su configuracion.
				 */
				modbus_conf_channels[ch] = true;

			}
		}
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_app(void)
{
	//	PLOAD=CLASS:APP;OFF;
	//  PLOAD=CLASS:APP;CONSIGNA;HHMM1:530;HHMM2:2330;
	//  PLOAD=CLASS:APP;PILOTO;PPR:3000;PWIDTH:20;SLOT0:0530,1.20;SLOT1:0630,2.30;SLOT2:0730,3.10;SLOT3:1230,2.50;SLOT4:2330,2.70
	//  PLOAD=CLASS:APP;GENPULSOS;PULSOSXMT3:100;PULSOWIDTH:10;

char *ts = NULL;
char localStr[32] = { 0 };
uint8_t slot;
char *stringp = NULL;
char *token = NULL;
char *tk_hhmm = NULL;
char *tk_pres = NULL;
char *delim = ",;:=><";
bool save_flag = false;
char id[4];
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_app in\r\n\0"));
	gprs_print_RX_buffer();

	// APP: OFF
	if ( gprs_check_response( 0, "APP;OFF") ) {
		systemVars.aplicacion_conf.aplicacion = APP_OFF;
		save_flag = true;

	} else 	if ( gprs_check_response( 0, "OCEANUS") ) {
		systemVars.aplicacion_conf.aplicacion = APP_OCEANUS;
		save_flag = true;

	} else if ( gprs_check_response( 0, "GENPULSOS") ) {
		systemVars.aplicacion_conf.aplicacion = APP_GENPULSOS;
		save_flag = true;
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "PULSOSXMT3");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PULSOSXMT3
		token = strsep(&stringp,delim);		// 100
		genpulsos_config_pulsosXmt3( token );
		ts = strstr( gprs_rxbuffer.buffer, "PULSOWIDTH");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PULSOWIDTH
		token = strsep(&stringp,delim);		// 10
		genpulsos_config_pulsoWidth( token );
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig GENPULSOS\r\n"));

	} else if ( gprs_check_response( 0, "CONSIGNA") ) {
		systemVars.aplicacion_conf.aplicacion = APP_CONSIGNA;
		save_flag = true;
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "HHMM1");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// HHMM1
		token = strsep(&stringp,delim);		// 530
		consigna_config("DIURNA", token);
		ts = strstr( gprs_rxbuffer.buffer, "HHMM2");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// HHMM2
		token = strsep(&stringp,delim);		// 2330
		consigna_config("NOCTURNA", token);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig CONSIGNAS\r\n"));

	} else if ( gprs_check_response( 0, "PILOTO") ) {

		systemVars.aplicacion_conf.aplicacion = APP_PILOTO;
		save_flag = true;

		// PULSEXREV
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "PPR");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PPR
		token = strsep(&stringp,delim);		// 3000
		piloto_config_ppr(token);

		// PWIDTH
		ts = strstr( gprs_rxbuffer.buffer, "PWIDTH");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PWIDTH
		token = strsep(&stringp,delim);		// 20
		piloto_config_pwidth(token);

		// SLOTS?
		for (slot=0; slot < MAX_PILOTO_PSLOTS; slot++ ) {
			memset( &str_base, '\0', sizeof(str_base) );
			snprintf_P( str_base, sizeof(str_base), PSTR("SLOT%d\0"), slot );
			if ( gprs_check_response( 0, str_base ) ) {
				memset(localStr,'\0',sizeof(localStr));
				ts = strstr( gprs_rxbuffer.buffer, str_base);
				strncpy(localStr, ts, sizeof(localStr));
				stringp = localStr;
				tk_hhmm = strsep(&stringp,delim);		//SLOTx
				tk_hhmm = strsep(&stringp,delim);		//1230
				tk_pres = strsep(&stringp,delim);		//1.34
				memset(&id, '\0', sizeof(id));
				itoa(slot, id, 10);
				//id[0] = '0' + slot;
				//id[1] = '\0';

				//xprintf_P(PSTR("DEBUG: slot=%s, hhmm=%s, pres=%s\r\n"), id, tk_hhmm, tk_pres);
				piloto_config_slot( id, tk_hhmm, tk_pres );
			}
		}
		reset_datalogger = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig PILOTOS\r\n"));
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	return(true);

}
//------------------------------------------------------------------------------------
static bool process_rsp_sms(void)
{
	//  PLOAD=CLASS:SMS;NRO0:0991234;NRO1:9900001;NRO2:99000002;DICT1:2,BOMBA;DICT2:-1,X;...;DICT9:-1,X;
	//  PLOAD=CLASS:SMS;NRO0:99000000;NRO1:99000001;NRO2:990000022;DICT1:2,BOMBA;DICT2:-1,X;DICT3:-1,X;DICT4:-1,X;DICT5:-1,X;DICT6:-1,X;DICT7:-1,X;DICT8:-1,X;DICT9:-1,X;

char localStr[32] = { 0 };
uint8_t pos;
char *ts = NULL;
char str_base[8];
char *stringp = NULL;
char *s_auth_nro = NULL;
char *s_mbus_channel = NULL;
char *s_order = NULL;
char *delim = ",;:=><";
bool save_flag = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_sms in\r\n\0"));
	gprs_print_RX_buffer();

	if ( gprs_check_response( 0, "SMS") ) {

		save_flag = true;

		// Nros Autorizados ?
		for (pos=0; pos < SMS_AUTH_NUMBER_MAX; pos++ ) {
			memset( &str_base, '\0', sizeof(str_base) );
			snprintf_P( str_base, sizeof(str_base), PSTR("NRO%d\0"), pos );
			if ( gprs_check_response( 0, str_base ) ) {
				memset(localStr,'\0',sizeof(localStr));
				ts = strstr( gprs_rxbuffer.buffer, str_base);
				strncpy(localStr, ts, sizeof(localStr));
				stringp = localStr;
				s_auth_nro = strsep(&stringp,delim);		//NROx
				s_auth_nro = strsep(&stringp,delim);		//99000001
				//xprintf_P(PSTR("DEBUG SMS NRO=%d, nbr=%s\r\n"), pos, s_auth_nro );
				sms_config_auth_number(pos, s_auth_nro );
			}
		}

		// Diccionario de ordenes
		for (pos=1; pos < SMS_ORDERS_MAX; pos++ ) {
			memset( &str_base, '\0', sizeof(str_base) );
			snprintf_P( str_base, sizeof(str_base), PSTR("DICT%d\0"), pos );
			if ( gprs_check_response( 0, str_base ) ) {
				memset(localStr,'\0',sizeof(localStr));
				ts = strstr( gprs_rxbuffer.buffer, str_base);
				strncpy(localStr, ts, sizeof(localStr));
				stringp = localStr;
				s_mbus_channel = strsep(&stringp,delim);		//DICTx
				s_mbus_channel = strsep(&stringp,delim);		//2
				s_order = strsep(&stringp,delim);				//BOMBA
				//xprintf_P(PSTR("DEBUG SMS DICT=%d, chmb=%s, order=%s\r\n"), pos, s_mbus_channel, s_order );
				sms_config_order_dict(pos, s_mbus_channel, s_order );
			}
		}

		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig SMS\r\n"));
		if ( save_flag ) {
			u_save_params_in_NVMEE();
		}
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_data(void)
{

	// Procesamos la respuesta.

uint8_t recds_borrados = 0;
FAT_t fat;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_data in\r\n"));
	gprs_print_RX_buffer();

	if ( gprs_check_response( 0, "CLOCK") ) {
		// Ajusto la hora local.
		data_resync_clock(false); // Es en respuesta a un frame. Solo ajusto si hay mas de 90s de diferencia.
	}

	if ( gprs_check_response (0, "MBUS") ) {
		data_process_response_MBUS();
		// rsp = rsp_OK;
		// return(rsp);
	}

	if ( gprs_check_response (0, "PILOTO") ) {
		data_process_response_PILOTO();
		// rsp = rsp_OK;
		// return(rsp);
	}

	if ( gprs_check_response (0, "STEPPER") ) {
		data_process_response_STEPPER();
		// rsp = rsp_OK;
		// return(rsp);
	}

	/*
	 * Lo ultimo que debo procesar es el OK !!!
	 * Borro los registros transmitidos
	 */
	if ( gprs_check_response( 0, "OK") ) {

		memset ( &fat, '\0', sizeof( FAT_t));
		// Borro los registros.
		while (  u_check_more_Rcds4Del(&fat) ) {
			FF_deleteRcd();
			recds_borrados++;
			FAT_read(&fat);
			xprintf_PD( DF_COMMS, PSTR("COMMS: mem wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), fat.wrPTR, fat.rdPTR, fat.delPTR, fat.rcds4wr, fat.rcds4rd, fat.rcds4del );
		}

		//DEBUG_del_bd( systemVars.max_rcsd_in_bd, systemVars.min_rcsd_in_bd );
	}

	if ( gprs_check_response( 0, "RESET") ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// RESET
		CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */
	}

	return(true);

}
//------------------------------------------------------------------------------------
uint16_t datos_pendientes_transmitir(void)
{
/* Veo si hay datos en memoria para trasmitir
 * Memoria vacia: rcds4wr = MAX, rcds4del = 0;
 * Memoria llena: rcds4wr = 0, rcds4del = MAX;
 * Memoria toda leida: rcds4rd = 0;
 * gprs_fat.wrPTR, gprs_fat.rdPTR, gprs_fat.delPTR,gprs_fat.rcds4wr,gprs_fat.rcds4rd,gprs_fat.rcds4del
 */

uint16_t nro_recs_pendientes;
FAT_t fat;

	memset( &fat, '\0', sizeof ( FAT_t));
	FAT_read(&fat);

	nro_recs_pendientes = fat.rcds4rd;
	// Si hay registros para leer
	if ( nro_recs_pendientes == 0) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: bd EMPTY\r\n\0"));
	}

	return(nro_recs_pendientes);
}
//------------------------------------------------------------------------------------
void data_resync_clock(bool force_adjust)
{
	/*
	 * Estrae el CLOCK del frame recibido y ajusta el clock interno
	 * Bug 01: 2021-12-14:
	 * El ajuste no considera los segundos entonces si el timerpoll es c/15s, cada 15s
	 * se reajusta y cambia la hora del datalogger.
	 * Modifico para que el reajuste se haga si hay una diferencia de mas de 90s entre
	 * el reloj local y el del server
	 */

char localStr[32] = { 0 };
char rtcStr[12];
RtcTimeType_t rtc_s,rtc_l;
int8_t xBytes = 0;
char *ts = NULL;
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
float diff_seconds;

	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( gprs_rxbuffer.buffer, "CLOCK:");
	strncpy(localStr, ts, sizeof(localStr));
	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK
	token = strsep(&stringp,delim);			// 1910120345
	memset(rtcStr, '\0', sizeof(rtcStr));
	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora

	// Error en el string recibido
	if ( strlen(rtcStr) < 10 ) {
		// Hay un error en el string que tiene la fecha.
		// No lo reconfiguro
		xprintf_P(PSTR("COMMS: data_resync_clock ERROR:[%s]\r\n\0"), rtcStr );
		return;
	}


	// Ajusto la hora

	// Convierto el string YYMMDDHHMM a RTC.
	memset( &rtc_s, '\0', sizeof(rtc_s) );
	RTC_str2rtc(rtcStr, &rtc_s);

	// Leo la hora actual del datalogger
	RTC_read_dtime( &rtc_l);


	if ( force_adjust ) {
		// Fuerzo el ajuste.( al comienzo )
		xBytes = RTC_write_dtime(&rtc_s);		// Grabo el RTC
		if ( xBytes == -1 ) {
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_process_server_clock\r\n\0"));
		} else {
			xprintf_PD( DF_COMMS, PSTR("COMMS: Update rtc to: %s\r\n\0"), rtcStr );
		}
		return;
	}

	// Solo ajusto si la diferencia es mayor de 90s
	// Veo la diferencia de segundos entre ambos.
	// Asumo yy,mm,dd iguales
	diff_seconds = abs( rtc_l.hour * 3600 + rtc_l.min * 60 + rtc_l.sec - ( rtc_s.hour * 3600 + rtc_s.min * 60 + rtc_s.sec));
	//xprintf_P( PSTR("COMMS: rtc diff=%.01f\r\n"), diff_seconds );

	if ( diff_seconds > 90 ) {
		// Ajusto
		xBytes = RTC_write_dtime(&rtc_s);		// Grabo el RTC
		if ( xBytes == -1 ) {
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_process_server_clock\r\n\0"));
		} else {
			xprintf_P( PSTR("COMMS: Update rtc to: %s\r\n\0"), rtcStr );
		}
		return;
	}
}
//------------------------------------------------------------------------------------
void data_process_response_MBUS(void)
{
	/*
	 * Recibo una respuesta que me dice que valores enviar por modbus
	 * para escribir un holding register
	 * <html><body><h1>TYPE=DATA&PLOAD=RX_OK:22;CLOCK:2103151132;MBUS=[1,2091,2,16,FLOAT,c3210,435.92]</h1></body></html>
	 *
	 * Testing desde el server:
	 * Se abre una consola redis: >redis-cli
	 * Comandos:
	 *    hset PABLO BROADCAST [1,2091,2,16,FLOAT,c3210,435.92][2,2091,2,16,U16,c3210,435][3,2091,2,16,U16,c3210,1234]
	 *    hset PTEST01 MODBUS "[2091,I,435][2093,F,12.45]"
	 *    hgetall PTEST01
	 *    hset PTEST01 MODBUS "NUL"
	 *
	 * Version 4.0.4:
	 * Junto al frame MBUS tambien viene MBTAG:xxx.
	 * Se debe extraer el tag xxx y devolverlo en los ACK o NACKs
	 * <html><body><h1>TYPE=DATA&PLOAD=RX_OK:22;CLOCK:2103151132;MBTAG:132;MBUS:[1,2091,2,16,FLOAT,c3210,435.92]</h1></body></html>
	 *
	 */

char *ts = NULL;
char *start, *end;
char localStr[64] = { 0 };
char *stringp = NULL;
char *tk_mbtag = NULL;
char *tk_slaaddr = NULL;
char *tk_regaddr = NULL;
char *tk_nro_regs = NULL;
char *tk_fcode = NULL;
char *tk_type = NULL;
char *tk_codec = NULL;
char *tk_value = NULL;
char *delim = ",;:=><[]";
int len;
bool enqueue_status;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_modbus in\r\n\0"));
	//gprs_print_RX_buffer();

	// MBTAG
	INIT_MBUS_TAG();

	if ( gprs_check_response( 0, "MBTAG") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "MBTAG");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		tk_mbtag = strsep(&stringp,delim);		// MBTAG
		tk_mbtag = strsep(&stringp,delim);		// tag

		modbus_set_mbtag(tk_mbtag);
		xprintf_PD( DF_COMMS, PSTR("COMMS: get MBTAG=%d\r\n"), atoi(tk_mbtag));
	}


	// MBUS
	if ( gprs_check_response( 0, "MBUS") ) {

		ts = strstr( gprs_rxbuffer.buffer, "MBUS");
		//xprintf_P(PSTR("DEBUG_MBUS: [%s]\r\n"), localStr);

		start = strchr(ts,'[');
		end = strchr(start,']');

		while ( ( start != NULL) && ( end != NULL ) ) {
			memset(localStr,'\0',sizeof(localStr));
			len = ( end - start );
			memcpy(&localStr, start, len );
			//xprintf_P(PSTR("RES_A =[%s]\r\n"), localStr );

			stringp = localStr;
			tk_slaaddr = strsep(&stringp,delim);
			tk_slaaddr = strsep(&stringp,delim);	// sla_address
			tk_regaddr = strsep(&stringp,delim);	// reg_address
			tk_nro_regs = strsep(&stringp,delim);	// nro_regs
			tk_fcode = strsep(&stringp,delim);		// f_code
			tk_type = strsep(&stringp,delim);		// type
			tk_codec = strsep(&stringp,delim);		// codec
			tk_value = strsep(&stringp,delim);

			enqueue_status = modbus_enqueue_output_cmd( tk_slaaddr, tk_regaddr, tk_nro_regs, tk_fcode, tk_type,tk_codec,tk_value );

			/*
			if ( enqueue_status ) {
				SET_MBUS_STATUS_FLAG_ACK();
				xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_modbus out ACK\r\n\0"));
			} else {
				SET_MBUS_STATUS_FLAG_NACK();
				xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_modbus out NACK\r\n\0"));
			}
			*/

			//xprintf_P(PSTR("MBUS_DEBUG=[%s][%s][%s][%s][%s][%s]\r\n"), tk_slaaddr, tk_regaddr, tk_nro_regs, tk_fcode, tk_type, tk_value );

			start++;
			ts = start;
			start = strchr(ts,'[');
			if ( start != NULL ) {
				end = strchr(start,']');
			} else {
				end = NULL;
			}

		}
	}

}
//------------------------------------------------------------------------------------
void data_process_response_PILOTO(void)
{
	/*
	 * Recibo una respuesta que me dice que presion debe setear el piloto ahora.
	 * <html><body><h1>TYPE=DATA&PLOAD=RX_OK:22;CLOCK:2103151132;PILOTO=3.45</h1></body></html>
	 *
	 * Testing desde el server:
	 * Se abre una consola redis: >redis-cli
	 * Comandos:
	 *    hset PTEST01 PILOTO "3.45"
	 *    hgetall PTEST01
	 *    hset PTEST01 MODBUS "NUL"
	 *
	 *
	 */

char *ts = NULL;
char localStr[48] = { 0 };
char *stringp = NULL;
char *tk_presion = NULL;
float presion = 0.0;
char *delim = ",;:=><";

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_piloto in\r\n\0"));
	//gprs_print_RX_buffer();

	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( gprs_rxbuffer.buffer, "PILOTO");
	strncpy(localStr, ts, sizeof(localStr));
	stringp = localStr;
	tk_presion = strsep(&stringp,delim);			// PILOTO
	tk_presion = strsep(&stringp,delim);			// 3.45
	presion = atof(tk_presion);
	piloto_productor_handler_onlineOrders(presion);

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_piloto out\r\n\0"));
}
//------------------------------------------------------------------------------------
void data_process_response_STEPPER(void)
{
	/*
	 * Recibo una respuesta que me dice que cuanto mover el stepper
	 * <html><body><h1>TYPE=DATA&PLOAD=RX_OK:22;CLOCK:2103151132;STEPPER=0.43,FW</h1></body></html>
	 *
	 * 0.43 es la fraccion de revolucion
	 * FW es el sentido
	 * Testing desde el server:
	 * Se abre una consola redis: >redis-cli
	 * Comandos:
	 *    hset PTEST01 STEPPER "0.43,FW"
	 *    hgetall PTEST01
	 *    hset PTEST01 MODBUS "NUL"
	 *
	 *
	 */

char *ts = NULL;
char localStr[48] = { 0 };
char *stringp = NULL;
char *tk_rev = NULL;
char *tk_direccion = NULL;
char *delim = ",;:=><";

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_stepper in\r\n\0"));
	//gprs_print_RX_buffer();

	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( gprs_rxbuffer.buffer, "STEPPER");
	strncpy(localStr, ts, sizeof(localStr));
	stringp = localStr;
	tk_rev = strsep(&stringp,delim);			// STEPPER
	tk_rev = strsep(&stringp,delim);			// 0.43
	tk_direccion = strsep(&stringp,delim);		// FW
//	plt_producer_stepper_online_handler(tk_rev, tk_direccion);

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_stepper out\r\n\0"));
}
//------------------------------------------------------------------------------------
void DEBUG_del_bd(uint16_t max_rcsd_in_bd, uint16_t min_rcsd_in_bd )
{
	/*
	 * En modo testing borramos los registros de modo de mantener entre min
	 * y max de 'rcsd_in_bd' siempre.
	 *
	 */

FAT_t fat;

	xprintf_PD( DF_COMMS, PSTR("COMMS: bd erase START.\r\n") );

	memset ( &fat, '\0', sizeof( FAT_t));
	FAT_read( &fat);

/*
	// Borro los registros.
	if ( fat.rcds4del >  max_rcsd_in_bd ) {
		// Alacance el maximo de registros a tener en la base.
		// Comienzo a borrar.
		while ( fat.rcds4del >  min_rcsd_in_bd ) {
			FF_deleteRcd();
			FAT_read( &fat);
			xprintf_PD( DF_COMMS, PSTR("COMMS: mem wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), fat.wrPTR, fat.rdPTR, fat.delPTR, fat.rcds4wr, fat.rcds4rd, fat.rcds4del );
		}
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: bd erase END.\r\n") );
*/

}
//------------------------------------------------------------------------------------



