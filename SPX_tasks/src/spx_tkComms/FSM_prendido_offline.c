/*
 * FSM_offline_configurar.c
 *
 *  Created on: 23 jul. 2021
 *      Author: pablo
 */

#include <tkComms.h>

static bool cmd_set_ATE(int8_t modo);
static bool cmd_read_ATI(void);
static bool cmd_set_CIPMODE(void);
static bool cmd_set_CSUART(void);
static bool cmd_read_CPIN(void);
static bool cmd_read_CCID(void);
static bool cmd_read_CREG(void);
static bool cmd_read_CPSI(void);
static bool cmd_set_CIPHEAD(void);
static bool cmd_set_CGDSOCKCONT(void);
static bool cmd_set_CMGF(void);
static void state_MONSQE(void);
//static bool state_configurar_C1(void);
static void pv_read_SQE(void) ;


//------------------------------------------------------------------------------------
int8_t tkXComms_PRENDIDO_OFFLINE(void)
{

	u_wdg_kick(WDG_COMMS, 300);

	xprintf_PD( DF_COMMS, PSTR("COMMS: state prendidoOFFLINE.\r\n\0"));

	// Quito el Echo para no desperdiciar buffers
	cmd_set_ATE(0);

	// Leo el IMEI. Si da error no importa.
	cmd_read_ATI();

	// Pongo el modo TCP normal. Es el que tiene por defecto por lo que
	// si no contesta, no importa tanto.
	cmd_set_CIPMODE();

	// Pongo la uart en control 7 hilos. Si no puedo no es grave.
	cmd_set_CSUART();

	//state_configurar_C1();

	if ( !cmd_read_CPIN() )		// Veo si tengo instalado un SIM
		return (APAGADO);

	// Leo el SIMID. Si da error no importa.
	cmd_read_CCID();

	if ( !cmd_read_CREG() )		// Confirmo esta registrado en la red
		return (APAGADO);

	if ( !cmd_read_CPSI() )		// Confirmo estar online en GSM p WCDMA.
		return (APAGADO);

	cmd_set_CIPHEAD();			// No muestro los IP head ( consumen mucho buffer al pedo )

	if ( !cmd_set_CGDSOCKCONT() )	// Configuro el APN
		return (APAGADO	);

	cmd_set_CMGF();				// Envio los SMS como texto.

	state_MONSQE();		// Monitoreo la SQE

	return(PRENDIDO_ONLINE);

}
//------------------------------------------------------------------------------------
static bool cmd_set_ATE(int8_t modo)
{
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE: ATE%d: OK\r\n"), modo );
	if (modo == 0) {
		// Apago el echo
		FSM_sendATcmd( 1, "ATE0\r" );
	} else {
		// Prendo el echo
		FSM_sendATcmd( 1, "ATE1\r" );
	}
	return(true);

}
//------------------------------------------------------------------------------------
static bool cmd_read_ATI(void)
{
	/*
	 * Leo el imei del modem para poder trasmitirlo al server
	 * ATI
	 * Manufacturer: SIMCOM INCORPORATED
	 * Revision: SIM5320J_V1.5
	 * IMEI: 860585007136848
	 * +GCAP: +CGSM,+DS,+ES
	 *
	 * OK
	 */

int8_t cmd_rsp;
int8_t tryes;
char *ts = NULL;
char c = '\0';
char *ptr = NULL;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:ATI in\r\n"));

	memset( xCOMMS_stateVars.gprs_imei, '\0', sizeof(xCOMMS_stateVars.gprs_imei) );

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "ATI\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "IMEI:" ) ) {
				// Extraigo el IMEI
				ptr = xCOMMS_stateVars.gprs_imei;
				ts = strstr( gprs_rxbuffer.buffer, "IMEI: ");
				ts += 6;
				while ( (c = *ts) != '\r') {
					*ptr++ = c;
					ts++;
				}
				*ptr = '\0';
				xprintf_PD( DF_COMMS, PSTR("COMMS: gprs imei=[%s]\r\n\0"), xCOMMS_stateVars.gprs_imei );
			}
			xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:ATI out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:ATI retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:ATI out: ERROR\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_set_CIPMODE(void)
{
	// Pone al modem en modo normal ( no transparente )

int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CIPMODE in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CIPMODE?\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "+CIPMODE: 0" ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CIPMODE out: OK\r\n") );
				return ( true );
			} else {
				// Respondio pero no es lo que esperaba.
				goto set_cipmode;
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CIPMODE retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r");
		// No importa si no responde.

	}

	// Errores luego de 3 reintentos:
	// Intento setearlo.

set_cipmode:

	// Aqui llego porque respondio pero no esta configurado como queremos.
	for ( tryes = 0; tryes <= 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CIPMODE=0\r" );

		if ( cmd_rsp == ATRSP_OK ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CIPMODE set out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CIPMODE set retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );
		// No importa si no responde.
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CIPMODE set out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_set_CSUART(void)
{
	// Pone la uart en modo 7 lines.
	// Vemos si ya esta configurado.

int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CSUART in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CSUART?\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "+CSUART: 1" ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CSUART out: OK\r\n") );
				return ( true );
			} else {
				// Respondio pero no es lo que esperaba.
				goto set_csuart;
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CSUART retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );

	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CSUART out: ERROR.\r\n"));
	return(false);

set_csuart:

	// Aqui llego porque respondio pero no esta configurado como queremos.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CSUART=1\r" );

		if ( cmd_rsp == ATRSP_OK ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CSUART set out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CSUART set retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );
		// No importa si no responde.
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CSUART set out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_read_CPIN(void)
{
	// Consulta al modem si tiene un SIM operativo.

int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CPIN in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CPIN?\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "+CPIN: READY" ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CPIN out: OK\r\n") );
				return ( true );
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CPIN retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );

	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CPIN out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_read_CCID(void)
{
	/*
	 * Leo el ccid del sim para poder trasmitirlo al server y asi llevar un control de donde esta c/sim
	 * AT+CCID
	 * +CCID: "8959801615239182186F"
	 *
	 * OK
	 */

int8_t cmd_rsp;
int8_t tryes;
char *ts = NULL;
char c = '\0';
char *ptr = NULL;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CCID in\r\n"));

	memset( xCOMMS_stateVars.gprs_ccid, '\0', sizeof(xCOMMS_stateVars.gprs_ccid) );

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CCID\r" );

		if (cmd_rsp	== ATRSP_OK ) {

			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "+CCID: " ) ) {
				// Extraigo el CCID
				ptr = xCOMMS_stateVars.gprs_ccid;
				ts = strstr( gprs_rxbuffer.buffer, "+CCID: ");
				ts += 8;
				while ( (c = *ts) != '"') {
					*ptr++ = c;
					ts++;
				}
				*ptr = '\0';
				xprintf_PD( DF_COMMS,  PSTR("COMMS: gprs ccid=[%s]\r\n\0"), xCOMMS_stateVars.gprs_ccid );
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CCID out: OK\r\n") );
				return ( true );
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CCID retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CCID out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_read_CREG(void)
{

	/* Chequeo que el TE este registrado en la red.
	 Esto deberia ser automatico.
	 Normalmente el SIM esta para que el dispositivo se registre automaticamente
	 Esto se puede ver con el comando AT+COPS? el cual tiene la red preferida y el modo 0
	 indica que esta para registrarse automaticamente.
	 Este comando se usa para de-registrar y registrar en la red.
	 Conviene dejarlo automatico de modo que si el TE se puede registrar lo va a hacer.
	 Solo chequeamos que este registrado con CGREG.
	 AT+CGREG?
	 +CGREG: 0,1
	 HAy casos como con los sims CLARO que puede llegar hasta 1 minuto a demorar conectarse
	 a la red, por eso esperamos mas.
	 En realidad el comando retorna en no mas de 5s, pero el registro puede demorar hasta 1 minuto
	 o mas, dependiendo de la calidad de señal ( antena ) y la red.
	 Para esto, el mon_sqe lo ponemos antes.

	 CREG testea estar registrado en la red celular
	 CGREG testea estar registrado en la red GPRS.
	 https://www.multitech.com/documents/publications/manuals/S000700.pdf

	*/

uint8_t timeout = 0;
int8_t tryes;
int8_t cmd_rsp;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CREG in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 4; tryes++ ) {

		timeout = 15 * tryes + 1; // Puede demorar mucho !!

		// La respuesta al comando suele ser inmediata
		cmd_rsp = FSM_sendATcmd( 5, "AT+CREG?\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "CREG: 0,1" ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CREG out: OK\r\n") );
				return ( true );
			}
		}

		// Espero que se registre.
		vTaskDelay( (portTickType)( timeout* 1000 / portTICK_RATE_MS ) );

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CREG retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CREG out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_read_CPSI(void)
{
	// Chequeo que la red este operativa

uint8_t timeout = 0;
int8_t tryes;
int8_t cmd_rsp;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CPSI in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		timeout = 5 * ( 1 + tryes ); // Puede demorar mucho !!
		cmd_rsp = FSM_sendATcmd( timeout, "AT+CPSI?\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "Online" ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CPSI out: OK\r\n") );
				return ( true );
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CPSI retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );

	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CPSI out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_set_CIPHEAD(void)
{
	// El comando CIPHEAD es para que no mande el header IP en la respuesta
	// Ocupa rxbuffer al pedo.

int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CIPHEAD\r\n"));

	//FSM_sendATcmd( 5, "AT+CIPSRIP=0\r" );

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CIPHEAD=0\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CIPHEAD out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CIPHEAD retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );

	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CIPHEAD out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
static bool cmd_set_CGDSOCKCONT(void)
{
	// Setea el APN.

char strapn[48];
int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 10, "AT+CGSOCKCONT?\r" );

		if (cmd_rsp	== ATRSP_OK ) {

			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, comms_conf.apn ) ) {
				xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT out: OK\r\n") );
				return ( true );
			} else {
				// Respondio pero no es lo que esperaba.
				goto set_cgdsockcont;
			}
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		// Probamos con un AT
		FSM_sendATcmd( 2, "AT\r" );
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT out: ERROR.\r\n"));
	return(false);

set_cgdsockcont:

	// Aqui llego porque respondio pero no esta configurado como queremos.
	memset(strapn,'\0', sizeof(strapn));
	snprintf_P( strapn, sizeof(strapn), PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), comms_conf.apn );

	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, strapn );

		if ( cmd_rsp == ATRSP_OK ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT set out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT set retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );
		// No importa si no responde.
	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CGDSOCKCONT set out: ERROR.\r\n"));
	return(false);


}
//------------------------------------------------------------------------------------
static void state_MONSQE(void)
{
	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:CSQ in\r\n"));

	if ( SPX_SIGNAL( SGN_MON_SQE ) ) {
		// Me quedo monitoreando la CSQ
		for(;;) {
			pv_read_SQE();
			vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
		}
	} else {
		// Solo mido 1 vez
		pv_read_SQE();
	}
}
//------------------------------------------------------------------------------------
static void pv_read_SQE(void)
{
char csqBuffer[32] = { 0 };
char *ts = NULL;
uint8_t csq = 0;
uint8_t dbm = 0;
int8_t cmd_rsp = -1;

	// Veo si ya esta configurado
	cmd_rsp = FSM_sendATcmd( 5, "AT+CSQ\r" );

	if (cmd_rsp	== ATRSP_OK ) {

		strncpy( csqBuffer, gprs_rxbuffer.buffer, sizeof(csqBuffer) );
		if ( (ts = strchr(csqBuffer, ':')) ) {
			ts++;
			csq = atoi(ts);
			dbm = 113 - 2 * csq;
		}
		xprintf_PD( DF_COMMS,  PSTR("COMMS: monitorCSQ: csq=%d, DBM=%d\r\n\0"), csq, dbm );
		xCOMMS_stateVars.csq = csq;

	}
}
//------------------------------------------------------------------------------------
static bool cmd_set_CMGF(void)
{
	// CMGF: Configura los SMS para enviarse en modo texto.

int8_t cmd_rsp;
int8_t tryes;

	xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE: CMGF\r\n"));

	//FSM_sendATcmd( 5, "AT+CIPSRIP=0\r" );

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	for ( tryes = 0; tryes < 3; tryes++ ) {

		cmd_rsp = FSM_sendATcmd( 5, "AT+CMGF=1\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CMGF out: OK\r\n") );
			return ( true );
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: prendidoOFFLINE:CMGF retry (t=%d) (rsp=%d)\r\n\0"), tryes, cmd_rsp );
		FSM_sendATcmd( 2, "AT\r" );

	}

	// Errores luego de 3 reintentos: salgo.
	xprintf_P( PSTR("COMMS: prendidoOFFLINE:CMGF out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------

/*
 * static bool state_configurar_C1(void)
{
	// Pone el registro C1 de modo que el DCD responda al Carrier del modem.

int8_t cmd_rsp;

	xprintf_P( PSTR("COMMS: prendidoOFFLINE:C1 in\r\n"));

	// Veo si ya esta configurado. Pregunto hasta 3 veces.
	cmd_rsp = FSM_sendATcmd( 2, "AT&C1\r" );

	if (cmd_rsp	== ATRSP_OK ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: prendidoOFFLINE:C1 out: OK\r\n") );
		return ( true );
	}

	xprintf_P( PSTR("COMMS: prendidoOFFLINE:C1 out: ERROR.\r\n"));
	return(false);

}
//------------------------------------------------------------------------------------
 *
 */



