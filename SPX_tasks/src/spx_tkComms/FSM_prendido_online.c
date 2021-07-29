/*
 * tkXComms_ONLINE.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 */


#include <tkComms.h>

typedef enum { ONLINE_ENTRY, ONLINE_AUTH, ONLINE_GLOBAL, ONLINE_BASE, ONLINE_EXIT } t_states_prendido_online;
typedef enum { SF_ENTRY, SF_SOCKET, SF_NET, SF_SEND, SF_RSP, SF_EXIT } t_sendFrames_states;
typedef enum { FRM_AUTH, FRM_GLOBAL, FRM_BASE, FRM_ANALOG } t_frames;

bool sendFrame(int8_t tipo_frame);

static int8_t state_online_entry(void);
static int8_t state_online_auth(void);
static int8_t state_online_global(void);
static int8_t state_online_base(void);
static int8_t state_online_exit(void);
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

		case ONLINE_EXIT:
			state = state_online_exit();
			// Cambio de estado.
			// Del modo ONLINE siempre salgo a APAGADO.
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
static int8_t state_online_base(void)
{
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE in\r\n\0"));

	if (sendFrame( FRM_BASE )) {
		//exit_code = ONLINE_ANALOG;
		exit_code = ONLINE_EXIT;
	} else {
		exit_code = ONLINE_EXIT;
	}
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoONLINE:BASE out\r\n\0"));

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

bool xmit_data_frame(int8_t tipo_frame);
bool rcvd_right_response(int8_t tipo_frame);

int8_t socket_status(void);
int8_t socket_open(void);
int8_t socket_close(void);

int8_t netservice_status(void);
int8_t netservice_open(void);
int8_t netservice_close(void);

typedef enum { sock_OPEN = 0, sock_CLOSE, sock_UNKNOWN, sock_TOUT } t_socket_status;
typedef enum { net_OPEN = 0, net_CLOSE, net_UNKNOWN, net_TOUT } t_network_status;

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
			// Chequeo si tengo un socket abierto
			sock_status = socket_status();
			if ( sock_status == sock_OPEN ) {
				state = SF_SEND;
			} else {
				state = SF_NET;
			}
			break;

		case SF_NET:
			// Chequeo si el servicio de sockets esta activo
			net_status = netservice_status();
			if ( net_status == net_OPEN ) {
				socket_open();
				state = SF_ENTRY;
			} else {
				netservice_open();
				state = SF_ENTRY;
			}
			break;

		case SF_SEND:
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
			// Espero la respuesta
			// Await
			vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
			if ( rcvd_right_response(tipo_frame) ) {
				ret_code = true;
				state = SF_EXIT;
			} else {
				socket_close();
				netservice_close();
				state = SF_ENTRY;
			}
			break;

		case SF_EXIT:
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

		cmd_rsp = FSM_sendATcmd( 10, "AT+CIPOPEN?\r", "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {

			if ( gprs_check_response(0, "CIPOPEN: 0,\"TCP\"") >= 0 ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out: OK (%d)\r\n\0"), tryes );
				return ( sock_OPEN );
			}

			if ( gprs_check_response(0, "CIPOPEN: 0") >= 0 ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out: OK (%d)\r\n\0"), tryes );
				return ( sock_CLOSE );
			}

		} else if ( cmd_rsp == ATRSP_ERR ) {
			// +IP ERROR: Operation not supported
			//
			// ERROR

			if ( gprs_check_response(0, "+IP ERROR:") >= 0 ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS dcd=%d\r\n"), IO_read_DCD() );
				xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out: OK (%d)\r\n\0"), tryes );
				return ( sock_UNKNOWN );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketSTATUS retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}

	// No puedo en 3 veces responder.
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketSTATUS out: ERROR.\r\n"));

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


	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN in\r\n"));

	memset(strapn,'\0', sizeof(strapn));
	snprintf_P( strapn, sizeof(strapn), PSTR("AT+CIPOPEN=0,\"TCP\",\"%s\",%s\r"), sVarsComms.server_ip_address, sVarsComms.server_tcp_port);

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, strapn, "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN out: OK (%d)\r\n\0"), tryes );
			return ( sock_OPEN );
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketOPEN out: ERROR.\r\n"));

	return( sock_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t socket_close(void)
{
	// Cierra el socket
	// Reintenta hasta 3 veces

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+CIPCLOSE=0\r", "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
			xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out: OK (%d)\r\n\0"), tryes );
			return ( sock_CLOSE );
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: socketCLOSE retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE dcd=%d\r\n"), IO_read_DCD() );
	xprintf_PD( DF_COMMS, PSTR("COMMS: socketCLOSE out: ERROR.\r\n"));

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

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETOPEN?\r", "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {

			if ( gprs_check_response(0, "+NETOPEN: 1") >= 0 ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out(open): OK (%d)\r\n\0"), tryes );
				return ( net_OPEN );
			}

			if ( gprs_check_response(0, "+NETOPEN: 0") >= 0 ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out(close): OK (%d)\r\n\0"), tryes );
				return ( net_CLOSE );
			}
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netSTATUS retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}
	// No puedo en 3 veces responder.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netSTATUS out: ERROR.\r\n"));
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

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETOPEN\r", "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN out: OK (%d)\r\n\0"), tryes );
			return ( net_OPEN );
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netOPEN retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netOPEN out: ERROR.\r\n"));

	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
int8_t netservice_close(void)
{
	// Cierra el servicio local de sockets
	// Reintenta hasta 3 veces

int8_t tryes;
int8_t cmd_rsp;


	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE in\r\n"));

	// Envio el comando hasta 3 veces.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+NETCLOSE=0\r", "OK" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out: OK (%d)\r\n\0"), tryes );
			return ( net_CLOSE );
		}

		// Reintento
		vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
		xprintf_PD( DF_COMMS,  PSTR("COMMS: netCLOSE retry (%d)\r\n\0"), tryes );
		// Probamos con un AT
		cmd_rsp = FSM_sendATcmd( 2, "AT\r", "OK" );
		if ( cmd_rsp != ATRSP_EXPECTED ) {
			// Si no responde no puedo seguir.
			break;
		}
	}

	// No puedo en 3 veces responder la secuencia CPAS,AT: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: netCLOSE out: ERROR.\r\n"));

	return( net_UNKNOWN );

}
//------------------------------------------------------------------------------------
bool xmit_data_frame(int8_t tipo_frame)
{
	return(false);
}
//------------------------------------------------------------------------------------
bool rcvd_right_response(int8_t tipo_frame)
{
	return(false);
}
//------------------------------------------------------------------------------------
