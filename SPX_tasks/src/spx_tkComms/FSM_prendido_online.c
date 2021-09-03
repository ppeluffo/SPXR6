/*
 * tkXComms_ONLINE.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 */


#include <tkComms.h>
#include "tkApp.h"

typedef enum { ONLINE_ENTRY, ONLINE_AUTH, ONLINE_GLOBAL, ONLINE_BASE, ONLINE_ANALOG, ONLINE_DIGITAL, ONLINE_COUNTER, ONLINE_MODBUS_LOW, ONLINE_MODBUS_HIGH, ONLINE_APP, ONLINE_DATA, ONLINE_ESPERA, ONLINE_EXIT } t_states_prendido_online;
typedef enum { SF_ENTRY, SF_SOCK_STATUS, SF_SOCK_OPEN, SF_NET_STATUS, SF_SEND, SF_RSP, SF_EXIT } t_sendFrames_states;
typedef enum { FRM_AUTH, FRM_GLOBAL, FRM_BASE, FRM_ANALOG, FRM_DIGITAL, FRM_COUNTER, FRM_MODBUS_LOW, FRM_MODBUS_HIGH, FRM_APP, FRM_DATA } t_frames;

typedef enum { sock_OPEN = 0, sock_CLOSE, sock_UNKNOWN, sock_TOUT } t_socket_status;
typedef enum { net_OPEN = 0, net_CLOSE, net_UNKNOWN, net_TOUT } t_network_status;

static bool state_online_entry(void);
static bool state_online_auth(void);
static bool state_online_global(void);
static bool state_online_base(void);
static bool state_online_analog(void);
static bool state_online_digital(void);
static bool state_online_counter(void);
static bool state_online_modbus_low(void);
static bool state_online_modbus_high(void);
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

uint16_t datos_pendientes_transmitir(void);
void data_resync_clock(void);

bool f_send_init_frame_base;
bool f_send_init_frame_analog;
bool f_send_init_frame_digital;
bool f_send_init_frame_counters;
bool f_send_init_frame_modbus_low;
bool f_send_init_frame_modbus_high;
bool f_send_init_frame_app;

bool reset_datalogger;

//------------------------------------------------------------------------------------
int8_t tkXComms_PRENDIDO_ONLINE(void)
{

int8_t state;

	xprintf_PD( DF_COMMS, PSTR("COMMS: state prendidoONLINE.\r\n"));

	state = ONLINE_ENTRY;

	// loop
	for( ;; )
	{
		u_wdg_kick(WDG_COMMS, 300);
		vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );

		switch ( state ) {
		case ONLINE_ENTRY:
			if ( state_online_entry() ) {
				state = ONLINE_AUTH;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_AUTH:
			if ( state_online_auth() ) {
				state = ONLINE_GLOBAL;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_GLOBAL:
			if ( state_online_global() ) {
				state = ONLINE_BASE;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_BASE:
			if ( ! f_send_init_frame_base ) {
				state = ONLINE_ANALOG;
			} else	if ( state_online_base() ) {
				state = ONLINE_ANALOG;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_ANALOG:
			if ( ! f_send_init_frame_analog ) {
				state = ONLINE_DIGITAL;
			} else if ( state_online_analog() ) {
				state = ONLINE_DIGITAL;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_DIGITAL:
			if ( ! f_send_init_frame_digital ) {
				state = ONLINE_COUNTER;
			} else if ( state_online_digital() ) {
				state = ONLINE_COUNTER;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_COUNTER:
			if ( ! f_send_init_frame_counters ) {
				state = ONLINE_MODBUS_LOW;
			} else if ( state_online_counter() ) {
				state = ONLINE_MODBUS_LOW;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_MODBUS_LOW:
			if ( ! f_send_init_frame_modbus_low ) {
				state = ONLINE_MODBUS_HIGH;
			} else if ( state_online_modbus_low() ) {
				state = ONLINE_MODBUS_HIGH;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_MODBUS_HIGH:
			if ( ! f_send_init_frame_modbus_high ) {
				state = ONLINE_APP;
			} else if ( state_online_modbus_high() ) {
				state = ONLINE_APP;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_APP:
			if ( ! f_send_init_frame_app ) {
				state = ONLINE_DATA;
			} else if ( state_online_app() ) {
				state = ONLINE_DATA;
			} else {
				state = ONLINE_EXIT;
			}
			break;

		case ONLINE_DATA:
			if ( state_online_data() ) {
				state = ONLINE_ESPERA;
			} else {
				state = ONLINE_EXIT;
			};
			break;

		case ONLINE_ESPERA:
			u_wdg_kick(WDG_COMMS, 300);
			if ( state_online_espera() ) {
				state = ONLINE_DATA;
			} else {
				state = ONLINE_EXIT;
			};
			break;

		case ONLINE_EXIT:
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
static bool state_online_modbus_low(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS_LOW in\r\n"));

	if (sendFrame( FRM_MODBUS_LOW )) {
		process_rsp_modbus();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS_LOW out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return(exit_code);
}
//------------------------------------------------------------------------------------
static bool state_online_modbus_high(void)
{
bool exit_code = true;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS_HIGH in\r\n"));

	if (sendFrame( FRM_MODBUS_HIGH )) {
		process_rsp_modbus();
		socket_close();
		exit_code = true;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:MODBUS_HIGH out (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
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
		process_rsp_auth();
		socket_close();
		exit_code = true;
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
	f_send_init_frame_modbus_low = false;
	f_send_init_frame_modbus_high = false;
	f_send_init_frame_app = false;

	reset_datalogger = false;

	// No quiero ECHO en los comandos
	// Sacamos el ECHO porque sino c/frame que mandamos lo repite y llena el rxbuffer al pedo.
	if ( FSM_sendATcmd( 3, "ATE0\r" ) != ATRSP_OK ) {
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:ATE0 ERROR\r\n"));
	}


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

		cmd_rsp = FSM_sendATcmd( 10, strapn );

		if (cmd_rsp	== ATRSP_OK ) {
			// Respondio al comando: Espero el resultado del sockopen.
			// Espero la respuesta del socket abierto
			for ( timeout = 0; timeout < 45; timeout++) {
				vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
				if ( gprs_check_response( 0, "+CIPOPEN: 0,0" ) ) {
					gprs_print_RX_buffer();
					xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
					return ( sock_OPEN );
				}
			}

		} else 	if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque ya esta abierto
			if ( gprs_check_response( 0, "+CIPOPEN:") ) {
				gprs_print_RX_buffer();
				xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
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
	return( sock_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t socket_close(void)
{
	// Cierra el socket
	// Reintenta hasta 3 veces controlando que ya no este cerrado

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

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
	case FRM_MODBUS_LOW:
		i = prepare_header(FRM_MODBUS_LOW);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_MBUS_LOW;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_MODBUS_HIGH:
		i = prepare_header(FRM_MODBUS_HIGH);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_MBUS_HIGH;"));
		i +=  prepare_tail(i);
		ret_code = send_txbuffer();
		break;
	case FRM_APP:
		i = prepare_header(FRM_COUNTER);
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CLASS:CONF_APP;"));
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
	// Calcula el tamaño y envia un CIPSEND.
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
	if ( ! gprs_check_response( 10, ">") ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: send_txbuffer out. (SEND ERROR No prompt) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
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
				xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response out (OK) (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
				return(true);
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
bool xmit_window_data( void )
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
	if ( frame_type == FRM_DATA ) {
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=DATA&VER=%s&PLOAD=" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
	} else {
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
static bool process_rsp_auth(void)
{
	// Recibimos un frame de autorizacion que indica si los parametros UID/DLGID son
	// correctos o debe reconfigurar el DLGID.
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:OK
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:RECONF;DLGID:TEST01
	// TYPE=INIT&PLOAD=CLASS:AUTH;STATUS:ERROR_DS

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
		data_resync_clock();
	}

	// Flags de configuraciones particulares: BASE;ANALOG;DIGITAL;COUNTERS;
	f_send_init_frame_base = false;
	f_send_init_frame_analog = false;
	f_send_init_frame_digital = false;
	f_send_init_frame_counters = false;
	f_send_init_frame_modbus_low = false;
	f_send_init_frame_modbus_high = false;
	f_send_init_frame_app = false;

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

	if ( gprs_check_response( 0, "MBUS_LOW") ) {
		f_send_init_frame_modbus_low = true;
	}

	if ( gprs_check_response( 0, "MBUS_HIGH") ) {
		f_send_init_frame_modbus_high = true;
	}

	if ( gprs_check_response( 0, "APLICACION") ) {
		f_send_init_frame_app = true;
	}

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_base(void)
{
	//	TYPE=INIT&PLOAD=CLASS:BASE;TPOLL:60;TDIAL:60;PWST:5;HW_CNT:OPTO

char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
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
	 * TYPE=INIT&PLOAD=CLASS:MODBUS;SLA:0;MBWT:10;MB00:T0,2069,2,3,FLOAT,0;MB01:T1,2069,2,3,FLOAT,0;MB02:T2,2062,2,3,FLOAT,0;
	 * MB03:T3,2063,2,3,FLOAT,0;MB04:T4,2064,2,3,FLOAT,0;MB05:X,0,1,3,U16,0;MB06:X,0,1,3,U16,0;MB07:X,0,1,3,U16,0;
	 * MB08:X,0,1,3,U16,0;MB09:X,0,1,3,U16,0;MB10:X,0,1,3,U16,0;MB11:X,0,1,3,U16,0;MB12:X,0,1,3,U16,0;MB13:X,0,1,3,U16,0;
	 * MB14:X,0,1,3,U16,0;MB15:X,0,1,3,U16,0;MB16:X,0,1,3,U16,0;MB17:X,0,1,3,U16,0;MB18:X,0,1,3,U16,0;MB19:X,0,1,3,U16,0;
	 *
	 */


char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *tk_sla= NULL;
char *tk_mbwt= NULL;
char *tk_name= NULL;
char *tk_address= NULL;
char *tk_size = NULL;
char *tk_fcode = NULL;
char *tk_type = NULL;
char *tk_pow10 = NULL;
char *delim = ",;:=><";
bool save_flag = false;
uint8_t ch;
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_modbus in\r\n\0"));
	gprs_print_RX_buffer();

	// SLA
	if ( gprs_check_response( 0, "SLA") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "SLA");
		strncpy(localStr, ts, sizeof(localStr));
		//xprintf_P(PSTR("DEBUG_SLA: [%s]\r\n"), localStr);
		stringp = localStr;
		tk_sla = strsep(&stringp,delim);		// SLA
		tk_sla = strsep(&stringp,delim);		// Value
		modbus_config_slave(tk_sla);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Modbus SLA[%s]\r\n"), tk_sla);
	}

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
			memset(localStr,'\0',sizeof(localStr));
			ts = strstr( gprs_rxbuffer.buffer, str_base);
			strncpy(localStr, ts, sizeof(localStr));
			xprintf_P(PSTR("DEBUG_MB%02d: [%s]\r\n"), ch, localStr);
			stringp = localStr;
			tk_name = strsep(&stringp,delim);		//MB00
			tk_name = strsep(&stringp,delim);		//name
			tk_address = strsep(&stringp,delim);	//Address
			tk_size = strsep(&stringp,delim);		//size
			tk_fcode = strsep(&stringp,delim);		//fcode
			tk_type = strsep(&stringp,delim);		//type
			tk_pow10 = strsep(&stringp,delim);		//pow10

			modbus_config_channel( ch, tk_name, tk_address, tk_size, tk_fcode, tk_type, tk_pow10 );

			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig MB%02d\r\n"), ch );
			save_flag = true;
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

char *ts = NULL;
char localStr[32] = { 0 };
uint8_t slot;
char *stringp = NULL;
char *token = NULL;
char *tk_hhmm = NULL;
char *tk_pres = NULL;
char *delim = ",;:=><";
bool save_flag = false;
char id[2];
char str_base[8];

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_app in\r\n\0"));
	gprs_print_RX_buffer();

	// APP: OFF
	if ( gprs_check_response( 0, "APP;OFF") ) {
		systemVars.aplicacion_conf.aplicacion = APP_OFF;
		save_flag = true;

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
				id[0] = '0' + slot;
				id[1] = '\0';
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
static bool process_rsp_data(void)
{

	// Procesamos la respuesta.

uint8_t recds_borrados = 0;
FAT_t fat;

	xprintf_PD( DF_COMMS, PSTR("COMMS: process_rsp_data in\r\n\0"));
	gprs_print_RX_buffer();


	if ( gprs_check_response( 0, "CLOCK") ) {
		// Ajusto la hora local.
		data_resync_clock();
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
void data_resync_clock(void)
{
	// Estrae el CLOCK del frame recibido y ajusta el clock interno

char localStr[32] = { 0 };
char rtcStr[12];
RtcTimeType_t rtc;
int8_t xBytes = 0;
char *ts = NULL;
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";

	memset(localStr,'\0',sizeof(localStr));
	ts = strstr( gprs_rxbuffer.buffer, "CLOCK:");
	strncpy(localStr, ts, sizeof(localStr));
	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK
	token = strsep(&stringp,delim);			// 1910120345
	memset(rtcStr, '\0', sizeof(rtcStr));
	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora

	if ( strlen(rtcStr) < 10 ) {
		// Hay un error en el string que tiene la fecha.
		// No lo reconfiguro
		xprintf_P(PSTR("COMMS: data_resync_clock ERROR:[%s]\r\n\0"), rtcStr );
	} else {
		memset( &rtc, '\0', sizeof(rtc) );
		RTC_str2rtc(rtcStr, &rtc);			// Convierto el string YYMMDDHHMM a RTC.
		xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
		if ( xBytes == -1 )
			xprintf_P(PSTR("ERROR: I2C:RTC:pv_process_server_clock\r\n\0"));

		xprintf_PD( DF_COMMS, PSTR("COMMS: Update rtc to: %s\r\n\0"), rtcStr );
	}
}

