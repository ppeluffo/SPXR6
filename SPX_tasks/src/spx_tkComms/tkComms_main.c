/*
 * tkXComms_01_main.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 */

#include <tkComms.h>

//------------------------------------------------------------------------------------
void tkComms(void * pvParameters)
{

( void ) pvParameters;
int8_t state = APAGADO;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );


	gprs_init();
	xCOMMS_stateVars.gprs_prendido = false;
	xCOMMS_stateVars.gprs_inicializado = false;
	xCOMMS_stateVars.errores_comms = 0;

	xprintf_P( PSTR("starting tkComms..\r\n\0"));

	// loop
	for( ;; )
	{
		switch ( state ) {
		case APAGADO:
			state = tkXComms_APAGADO();
			break;
		case PRENDIDO_OFFLINE:
			state = tkXComms_PRENDIDO_OFFLINE();
			break;
		case PRENDIDO_ONLINE:
			state = tkXComms_PRENDIDO_ONLINE();
			break;
		default:
			state = APAGADO;
			xprintf_P( PSTR("COMMS: state ERROR !!.\r\n\0"));
		}
	}


}
//------------------------------------------------------------------------------------
void tkCommsRX(void * pvParameters)
{
	// Esta tarea lee y procesa las respuestas del GPRS. Lee c/caracter recibido y lo va
	// metiendo en un buffer circular propio del GPRS que permite luego su impresion,
	// analisis, etc.

( void ) pvParameters;
char c;
uint32_t ulNotifiedValue;


	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("starting tkCommsRX..\r\n\0"));


	for( ;; )	{

		if ( xCOMMS_stateVars.gprs_prendido == true ) {
			// Leo el UART de GPRS
			if ( frtos_read( fdGPRS, &c, 1 ) == 1 ) {
				gprs_rxbuffer_put2(c);
			}

		} else {

			// Espero hasta 25s o que me llegue una señal
			xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 10000 / portTICK_RATE_MS ) );

		}
	}


}
//------------------------------------------------------------------------------------
void comms_config_defaults(void)
{

	strncpy_P( comms_conf.apn, PSTR("SPYMOVIL.VPNANTEL\0"), APN_LENGTH );
	strncpy_P( comms_conf.server_ip_address, PSTR("192.168.0.20\0"),16);
	strncpy_P( comms_conf.dlgId, PSTR("DEFAULT\0"),  DLGID_LENGTH );
	strncpy_P( comms_conf.serverScript, PSTR("/cgi-bin/SPY/spy.py\0"), SCRIPT_LENGTH);
	strncpy_P( comms_conf.server_tcp_port, PSTR("80\0"), PORT_LENGTH	);

	comms_conf.timerDial = 900;

}
//------------------------------------------------------------------------------------
void comms_config_status(void)
{

uint8_t dbm;

	xprintf_P( PSTR(">Device Gprs:\r\n"));
	xprintf_P( PSTR("  apn: %s\r\n\0"), comms_conf.apn );
	xprintf_P( PSTR("  server ip:port: %s:%s\r\n"), comms_conf.server_ip_address, comms_conf.server_tcp_port );
	xprintf_P( PSTR("  server script: %s\r\n"), comms_conf.serverScript );
	//xprintf_P( PSTR("  simpwd: %s\r\n\0"), comms_conf.simpwd );
	xprintf_P( PSTR("  imei: %s\r\n"), xCOMMS_stateVars.gprs_imei ) ;
	xprintf_P( PSTR("  ccid: %s\r\n"), xCOMMS_stateVars.gprs_ccid ) ;

	dbm = 113 - 2 * xCOMMS_stateVars.csq;
	xprintf_P( PSTR("  signalQ: csq=%d, dBm=%d\r\n"), xCOMMS_stateVars.csq, dbm );
	xprintf_P( PSTR("  ip address: %s\r\n"), xCOMMS_stateVars.ip_assigned) ;

	// MODO:
	switch(xCOMMS_stateVars.gprs_mode) {
	case 2:
		xprintf_P( PSTR("  modo: AUTO\r\n"));
		break;
	case 13:
		xprintf_P( PSTR("  modo: 2G(GSM) only\r\n"));
		break;
	case 14:
		xprintf_P(  PSTR("  modo: 3G(WCDMA) only\r\n"));
		break;
	default:
		xprintf_P( PSTR("  modo: ??\r\n") );
	}

	// PREFERENCE
	switch(xCOMMS_stateVars.gprs_pref) {
	case 0:
		xprintf_P( PSTR("  pref: AUTO\r\n"));
		break;
	case 1:
		xprintf_P(  PSTR("  pref: 2G,3G\r\n"));
		break;
	case 2:
		xprintf_P( PSTR("  pref: 3G,2G\r\n"));
		break;
	default:
		xprintf_P( PSTR("  pref: ??\r\n") );
		return;
	}

	// BANDS:
	xprintf_P( PSTR("  bands:[%s]\r\n\0"), xCOMMS_stateVars.gprs_bands );

}
//------------------------------------------------------------------------------------
