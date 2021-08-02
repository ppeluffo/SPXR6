/*
 * tkXComms_ONLINE.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 */


#include <tkComms.h>

typedef enum { ONLINE_ENTRY, ONLINE_AUTH, ONLINE_GLOBAL, ONLINE_BASE, ONLINE_ANALOG, ONLINE_DIGITAL, ONLINE_COUNTER, ONLINE_DATA, ONLINE_EXIT } t_states_prendido_online;
typedef enum { SF_ENTRY, SF_SOCKET, SF_NET, SF_SEND, SF_RSP, SF_EXIT } t_sendFrames_states;
typedef enum { FRM_AUTH, FRM_GLOBAL, FRM_BASE, FRM_ANALOG, FRM_DIGITAL, FRM_COUNTER, FRM_DATA } t_frames;

typedef enum { sock_OPEN = 0, sock_CLOSE, sock_UNKNOWN, sock_TOUT } t_socket_status;
typedef enum { net_OPEN = 0, net_CLOSE, net_UNKNOWN, net_TOUT } t_network_status;


static int8_t state_online_entry(void);
static int8_t state_online_auth(void);
static int8_t state_online_global(void);
static int8_t state_online_base(void);
static int8_t state_online_analog(void);
static int8_t state_online_digital(void);
static int8_t state_online_counter(void);
static int8_t state_online_data(void);
static int8_t state_online_exit(void);

static void read_IPADDRES(void);

bool sendFrame(int8_t tipo_frame);
bool xmit_data_frame(int8_t tipo_frame);
bool rcvd_data_response(int8_t tipo_frame);

int8_t socket_status(void);
int8_t socket_open(void);
int8_t socket_close(void);

int8_t netservice_status(void);
int8_t netservice_open(void);
int8_t netservice_close(void);

static bool process_rsp_auth(void);
static bool process_rsp_global(void);
static bool process_rsp_base(void);
static bool process_rsp_analog(void);
static bool process_rsp_digital(void);
static bool process_rsp_counters(void);

bool f_send_init_frame_base;
bool f_send_init_frame_analog;
bool f_send_init_frame_digital;
bool f_send_init_frame_counters;

//------------------------------------------------------------------------------------
int8_t tkXComms_PRENDIDO_ONLINE(void)
{

int8_t state;

	xprintf_PD( DF_COMMS, PSTR("COMMS: state prendidoONLINE.\r\n"));

	state = ONLINE_ENTRY;

	// loop
	for( ;; )
	{
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		switch ( state ) {
		case ONLINE_ENTRY:
			state = state_online_entry();
			break;

		case ONLINE_AUTH:
			state = state_online_auth();
			break;

		case ONLINE_GLOBAL:
			state = state_online_global();
			break;

		case ONLINE_BASE:
			state = state_online_base();
			break;

		case ONLINE_ANALOG:
			state = state_online_analog();
			break;

		case ONLINE_DIGITAL:
			state = state_online_digital();
			break;

		case ONLINE_COUNTER:
			state = state_online_counter();
			break;

		case ONLINE_DATA:
			state = state_online_data();
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
static int8_t state_online_data(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DATA in\r\n"));


	exit_code = ONLINE_EXIT;
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DATA out\n"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_counter(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:COUNTER in\r\n"));

	if (sendFrame( FRM_COUNTER )) {
		exit_code = ONLINE_DATA;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:COUNTER out\n"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_digital(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:DIGITAL in\r\n"));

	if (sendFrame( FRM_DIGITAL )) {
		exit_code = ONLINE_COUNTER;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:COUNTER out\n"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_analog(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ANALOG in\r\n"));

	if (sendFrame( FRM_ANALOG )) {
		exit_code = ONLINE_DIGITAL;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ANALOG out\n"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_base(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE in\r\n"));

	if (sendFrame( FRM_BASE )) {
		exit_code = ONLINE_ANALOG;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE out\r\n"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_global(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:GLOBAL in\r\n\0"));

	if (sendFrame( FRM_GLOBAL )) {
		exit_code = ONLINE_BASE;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:GLOBAL out\r\n\0"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_auth(void)
{

	// Intento enviar y procesar un frame de AUTH.
	// Si lo logro paso a procesar el siguiente tipo de frame.

int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:AUTH in\r\n\0"));

	if (sendFrame( FRM_AUTH )) {
		exit_code = ONLINE_GLOBAL;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:AUTH out\r\n\0"));

	return(exit_code);
}
//------------------------------------------------------------------------------------
static int8_t state_online_entry(void)
{
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ENTRY in\r\n\0"));

	// Inicializo todo lo necesario al modo prendido ONLINE
	f_send_init_frame_base = false;
	f_send_init_frame_analog = false;
	f_send_init_frame_digital = false;
	f_send_init_frame_counters = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:ENTRY out\r\n\0"));

	return(ONLINE_AUTH);
}
//------------------------------------------------------------------------------------
static int8_t state_online_exit(void)
{
	return(-1);
}
//------------------------------------------------------------------------------------
// FSM SendFrames
//------------------------------------------------------------------------------------
/*
 * Para enviar un frame implementamos una FSM que se encarga de los sockets, activar el
 * PDP ( netopen ), enviar el frame, esperar y procesar la respuesta.
 *
 */
bool sendFrame(int8_t tipo_frame)
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

		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		switch (state) {
		case SF_ENTRY:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame ENTRY\r\n"));
			// Controlo no quedar en un loop infinito
			tryes++;
			if ( tryes == 8 ) {
				ret_code = false;
				state = SF_EXIT;
			} else {
				state = SF_SOCKET;
			}
			break;

		case SF_SOCKET:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame SOCKET\r\n"));
			// Chequeo si tengo un socket abierto
			sock_status = socket_status();
			if ( sock_status == sock_OPEN ) {
				state = SF_SEND;
			} else {
				state = SF_NET;
			}
			break;

		case SF_NET:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame NET\r\n"));
			// Chequeo si el servicio de sockets esta activo
			net_status = netservice_status();
			if ( net_status == net_OPEN ) {
				read_IPADDRES();
				socket_open();
				state = SF_ENTRY;
			} else {
				memset( xCOMMS_stateVars.ip_assigned, '\0', sizeof(xCOMMS_stateVars.ip_assigned) );
				netservice_open();
				state = SF_ENTRY;
			}
			break;

		case SF_SEND:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame SEND\r\n"));
			// Envio el frame
			if ( xmit_data_frame(tipo_frame) ) {
				state = SF_RSP;
			} else {
				socket_close();
				netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_RSP:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame RSP\r\n"));
			// Espero la respuesta
			if ( rcvd_data_response(tipo_frame) ) {
				ret_code = true;
				state = SF_EXIT;
			} else {
				socket_close();
				netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_EXIT:
			xprintf_PD( DF_COMMS, PSTR("COMMS: sendFrame EXIT\r\n"));
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

		cmd_rsp = FSM_sendATcmd( 10, "AT+CIPOPEN?\r");

		if (cmd_rsp	== ATRSP_OK ) {

			if ( gprs_check_response( 0, "CIPOPEN: 0,\"TCP\"")  ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (open) OK (%d)\r\n\0"), tryes );
				return ( sock_OPEN );
			}

			if ( gprs_check_response( 0, "CIPOPEN: 0") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (close) OK (%d)\r\n\0"), tryes );
				return ( sock_CLOSE );
			}

		} else if ( cmd_rsp == ATRSP_ERROR ) {
			// +IP ERROR: Operation not supported
			//
			// ERROR

			if ( gprs_check_response( 0, "+IP ERROR:") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out (unknown) OK (%d)\r\n\0"), tryes );
				return ( sock_UNKNOWN );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketSTATUS retry (%d)\r\n\0"), tryes );
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

	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN in\r\n"));

	memset(strapn,'\0', sizeof(strapn));
	snprintf_P( strapn, sizeof(strapn), PSTR("AT+CIPOPEN=0,\"TCP\",\"%s\",%s\r"), comms_conf.server_ip_address, comms_conf.server_tcp_port);

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, strapn );

		if (cmd_rsp	== ATRSP_OK ) {
			// Respondio al comando: Espero el resultado del netopen.
			goto await_sock_status;
		}

		if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque ya esta abierto
			if ( gprs_check_response( 0, "+CIPOPEN:") ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK\r\n\0") );
				gprs_print_RX_buffer();
				return ( net_OPEN );
			}
		}

		// Reintento dando antes un AT.
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN out (unknown) ERROR\r\n"));

	gprs_print_RX_buffer();
	return( sock_UNKNOWN );

await_sock_status:

	// Espero la respuesta del socket abierto
	for ( timeout = 0; timeout < 45; timeout++) {
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( gprs_check_response( 0, "+CIPOPEN: 0" ) ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out (open) OK\r\n\0") );
			gprs_print_RX_buffer();
			return ( net_OPEN );
		}
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN out (unkown) ERROR\r\n"));
	gprs_print_RX_buffer();
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
			xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
			xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out (close) OK (%d)\r\n\0"), tryes );
			return ( sock_CLOSE );
		}

		if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque el socket ya esta cerrado
			if ( gprs_check_response( 0, "+CIPCLOSE:") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
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
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
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

int8_t tryes;
int8_t cmd_rsp;
int8_t timeout;

	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN in\r\n"));

	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETOPEN\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			// Respondio al comando: Espero el resultado del netopen.
			goto await_net_status;
		}

		if (cmd_rsp	== ATRSP_ERROR ) {
			// Respondio al comando con error: Tal vez ya esta abierto.
			if ( gprs_check_response( 0, "Network is already opened") ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN out (open) OK\r\n\0") );
				gprs_print_RX_buffer();
				return ( net_OPEN );
			}
		}

		// Reintento dando antes un AT.
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN out (unknown) ERROR.\r\n"));
	gprs_print_RX_buffer();
	return( net_UNKNOWN );

await_net_status:

	// Espero que se abra la conexion
	for ( timeout = 0; timeout < 45; timeout++) {
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		if ( gprs_check_response( 0, "+NETOPEN: 0" ) ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN out (open) OK\r\n\0") );
			gprs_print_RX_buffer();
			return ( net_OPEN );
		}
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN out (unkown) ERROR\r\n"));
	gprs_print_RX_buffer();
	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t netservice_close(void)
{
	// Cierra el servicio local de sockets
	// Reintenta hasta 3 veces controlando que ya no este cerrado.

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETCLOSE\r" );
		if (cmd_rsp	== ATRSP_OK ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (close) OK (%d)\r\n\0"), tryes );
			return ( net_CLOSE );
		}

		if (cmd_rsp	== ATRSP_ERROR ) {
			// Puede haber dado error porque ya esta cerrado
			if ( gprs_check_response( 0, "+NETCLOSE:") ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (close) OK (%d)\r\n\0"), tryes );
				return ( net_CLOSE );
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netCLOSE retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out (unknown) ERROR\r\n"));

	return( net_UNKNOWN );

}
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
			xprintf_PD( DF_COMMS,  PSTR("COMMS: IPADDR [%s]\r\n\0"), xCOMMS_stateVars.ip_assigned );
		}
	}
}
//------------------------------------------------------------------------------------
bool xmit_data_frame(int8_t tipo_frame)
{
uint16_t i = 0;
uint16_t size = 0;
char strapn[24];

	xprintf_PD( DF_COMMS, PSTR("COMMS: xmit_data_frame\r\n"));

	// Preparo el frame
	gprs_txbuffer_reset();

	// HEADER
	// BODY
	switch(tipo_frame) {
	case FRM_AUTH:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:AUTH;UID:%s;" ),NVMEE_readID() );
		break;
	case FRM_GLOBAL:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:GLOBAL;NACH:%d;NDCH:%d;NCNT:%d;" ),ANALOG_CHANNELS, DINPUTS_CHANNELS, COUNTER_CHANNELS );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("IMEI:%s;" ), xCOMMS_stateVars.gprs_imei );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("SIMID:%s;CSQ:%d;WRST:%02X;" ), xCOMMS_stateVars.gprs_ccid, xCOMMS_stateVars.csq, wdg_resetCause );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("BASE:0x%02X;" ), u_base_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("AN:0x%02X;" ), ainputs_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("DG:0x%02X;" ), dinputs_hash() );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("CNT:0x%02X;" ), counters_hash() );
		break;
	case FRM_BASE:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:CONF_BASE;"));
		break;
	case FRM_ANALOG:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:CONF_ANALOG;"));
		break;
	case FRM_DIGITAL:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:CONF_DIGITAL;"));
		break;
	case FRM_COUNTER:
		i = sprintf_P( gprs_txbuffer.buffer, PSTR("GET %s?DLGID=%s&TYPE=INIT&VER=%s" ), comms_conf.serverScript, comms_conf.dlgId, SPX_FW_REV );
		i += sprintf_P( &gprs_txbuffer.buffer[i], PSTR("&PLOAD=CLASS:CONF_COUNTER;"));
		break;
	case FRM_DATA:
		break;
	default:
		break;
	}

	// TAIL
	i +=  sprintf_P( &gprs_txbuffer.buffer[i], PSTR(" HTTP/1.1\r\nHost: www.spymovil.com\r\n\r\n\r\n") );

	// Solicito el prompt para transmitir
	memset(strapn,'\0', sizeof(strapn));
	size = strlen(gprs_txbuffer.buffer);
	snprintf_P( strapn, sizeof(strapn), PSTR("AT+CIPSEND=0,%d\r"),size);
	FSM_sendATcmd( 1, strapn );
	// Espero el prompt 1000 ms.
	if ( ! gprs_check_response( 10, ">") ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: xmit_data_frame SEND ERROR No prompt.\r\n"));
		return(false);
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: xmit_data_frame SENDING\r\n"));
	// Envio el frame. El buffer es mayor que lo que maneja xprintf por lo que lo envio directo !!!
	gprs_flush_RX_buffer();
	sxprintf_D( fdGPRS, DF_COMMS , gprs_txbuffer.buffer, size );

	// Espero la confirmacion del modem hasta 2000 msecs. No borro el RX buffer !!!.
	if ( gprs_check_response( 20, "+CIPSEND: 0,") ) {
		gprs_print_RX_buffer();
		return(true);
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: xmit_data_frame SEND ERROR No response.\r\n"));
	gprs_print_RX_buffer();
	return(false);

}
//------------------------------------------------------------------------------------
bool rcvd_data_response(int8_t tipo_frame)
{
	// Espero una respuesta del server.

int8_t timeout;

	xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response in.\r\n"));

	for ( timeout = 0; timeout < 100; timeout++) {
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		if ( gprs_check_response( 0, "RECV FROM:" ) ) {
			if ( gprs_check_response( 0, "</h1>" ) ) {
				goto analize_response;
			}

		} else if ( gprs_check_response( 0, "+IPCLOSE:" ) ) {
			gprs_print_RX_buffer();
			xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response ERROR.\r\n"));
			return(false);
		}
	}

	// TIMEOUT:
	gprs_print_RX_buffer();
	xprintf_PD( DF_COMMS, PSTR("COMMS: rcvd_data_response TIMEOUT.\r\n"));
	return(false);

analize_response:

	switch(tipo_frame) {
	case FRM_AUTH:
		xprintf_P(PSTR("AUTH rsp. OK\r\n"));
		process_rsp_auth();
		break;
	case FRM_GLOBAL:
		xprintf_P(PSTR("GLOBAL rsp. OK\r\n"));
		process_rsp_global();
		break;
	case FRM_BASE:
		xprintf_P(PSTR("BASE rsp. OK\r\n"));
		process_rsp_base();
		break;
	case FRM_ANALOG:
		xprintf_P(PSTR("ANALOG rsp. OK\r\n"));
		process_rsp_analog();
		break;
	case FRM_DIGITAL:
		xprintf_P(PSTR("DIGITAL rsp. OK\r\n"));
		process_rsp_digital();
		break;
	case FRM_COUNTER:
		xprintf_P(PSTR("COUNTER rsp. OK\r\n"));
		process_rsp_counters();
		break;
	case FRM_DATA:
		xprintf_P(PSTR("DATA rsp. OK\r\n"));
		break;
	default:
		xprintf_P(PSTR("ERROR rsp. OK\r\n"));
		break;
	}

	gprs_print_RX_buffer();
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

char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
char *ts = NULL;

	xprintf_PD( DF_COMMS, PSTR("process_rsp_auth.\r\n\0"));

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

char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
char rtcStr[12];
uint8_t i = 0;
char c = '\0';
RtcTimeType_t rtc;
int8_t xBytes = 0;
char *ts = NULL;

	xprintf_PD( DF_COMMS, PSTR("COMMS_INIT_GLOBAL\r\n\0"));

	// CLOCK
	if ( gprs_check_response( 0, "CLOCK") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "RECONF;");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);			// CLOCK
		token = strsep(&stringp,delim);			// 1910120345
		memset(rtcStr, '\0', sizeof(rtcStr));
		memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
		for ( i = 0; i<12; i++) {
			c = *token;
			rtcStr[i] = c;
			c = *(++token);
			if ( c == '\0' )
				break;
		}
		memset( &rtc, '\0', sizeof(rtc) );
		if ( strlen(rtcStr) < 10 ) {
			// Hay un error en el string que tiene la fecha.
			// No lo reconfiguro
			xprintf_P(PSTR("COMMS: RTCstring ERROR:[%s]\r\n\0"), rtcStr );
		} else {
			RTC_str2rtc(rtcStr, &rtc);			// Convierto el string YYMMDDHHMM a RTC.
			xBytes = RTC_write_dtime(&rtc);		// Grabo el RTC
			if ( xBytes == -1 )
				xprintf_P(PSTR("ERROR: I2C:RTC:pv_process_server_clock\r\n\0"));

			xprintf_PD( DF_COMMS, PSTR("COMMS: Update rtc to: %s\r\n\0"), rtcStr );
		}

	}

	// Flags de configuraciones particulares: BASE;ANALOG;DIGITAL;COUNTERS;RANGE;PSENSOR;OUTS
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

	return(true);
}
//------------------------------------------------------------------------------------
static bool process_rsp_base(void)
{
	//	TYPE=INIT&PLOAD=CLASS:BASE;TPOLL:60;TDIAL:60;PWST:5;CNT_HW:OPTO

char *ts = NULL;
char localStr[32] = { 0 };
char *stringp = NULL;
char *token = NULL;
char *delim = ",;:=><";
bool save_flag = false;
bool reset_datalogger = false;

	xprintf_PD( DF_COMMS, PSTR("COMMS_INIT_BASE\r\n\0"));

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
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig TDIAL\r\n\0"));
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
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig TPOLL\r\n\0"));
	}

	// PWST
	if ( gprs_check_response( 0, "PWSR") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "PWST");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// PWST
		token = strsep(&stringp,delim);		// timePwrSensor
		ainputs_config_timepwrsensor(token);
		save_flag = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig PWRSTIME\r\n\0"));
	}

	// CNT_HW
	if ( gprs_check_response( 0, "HW_CNT") ) {
		memset(localStr,'\0',sizeof(localStr));
		ts = strstr( gprs_rxbuffer.buffer, "PWST");
		strncpy(localStr, ts, sizeof(localStr));
		stringp = localStr;
		token = strsep(&stringp,delim);		// CNT_HW
		token = strsep(&stringp,delim);		// opto/simple

		counters_config_hw(token);
		save_flag = true;
		reset_datalogger = true;
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig COUNTERS_HW\r\n\0"));
	}

	if ( save_flag ) {
		u_save_params_in_NVMEE();
	}

	if ( reset_datalogger) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: Reset...\r\n\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */
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

	xprintf_PD( DF_COMMS, PSTR("COMMS_INIT_ANALOG\r\n\0"));

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
			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig A%d\r\n\0"), ch);
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

	xprintf_PD( DF_COMMS, PSTR("COMMS_INIT_DIGITAL\r\n\0"));

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
			xprintf_PD( DF_COMMS, PSTR("COMMS: Reconfig D%d\r\n\0"), ch);
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

	xprintf_PD( DF_COMMS, PSTR("COMMS_INIT_COUNTERS\r\n\0"));

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
