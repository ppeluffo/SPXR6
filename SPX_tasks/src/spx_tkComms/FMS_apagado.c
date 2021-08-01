/*
 * FSM_apagado.c
 *
 *  Created on: 22 jul. 2021
 *      Author: pablo
 *
 *  En este estado esperamos apagados o prendido.
 *  Si estoy prendido, salgo por señal que llego un frame y durante la espera, c/60s chequeo si hay SMS entrantes.
 *  Si estoy apagado, al finalizar prendo, configuro y salgo.
 *  Puedo tener errores al prender o al configurar.
 *  En estos casos puedo llegar a apagar el modem pero ya no espero el tiempo timerDial sino que espero por ej. 30s, 60s, 120s
 *  y recien si no pude, vuelvo a esperar el timerDial.
 *
 *  PROBLEMAS:
 *  En 13 hs de analisis, 51 bloques no hubieron errores de PBDONE ni de CPAS.
 *  El tema es que luego del PBDONE, en 18 veces el AT no respondio por lo que se reintento con on/off.
 *  Luego que el AT responde, el CPAS siempre respondio.
 *  Luego que el PBDONE responde, esperamos 10s mas. Lo aumento a 15s.
 *  Los resultados globales fueron que en los 51 bloques, siempre paso el CPAS.
 *
 */

#include "tkComms.h"

typedef enum { APAGADO_ENTRY, PRENDERHW, PRENDERSW, PBDONE, CPAS, APAGADO_EXIT } t_states_apagado;

static int8_t state;
static int32_t awaittime_for_dial;
static int8_t prender_hw_tryes;
static int8_t prender_sw_tryes;

static int8_t state_apagado(void);
static int8_t state_prenderHW(void);
static int8_t state_prenderSW(void);
static int8_t state_pbdone(void);
static int8_t state_cpas(void);
static int8_t state_exit(void);

//------------------------------------------------------------------------------------
int8_t tkXComms_APAGADO(void)
{

	xprintf_PD( DF_COMMS, PSTR("COMMS: state apagado\r\n"));

	state = APAGADO_ENTRY;

	// loop
	for( ;; )
	{
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		switch ( state ) {
		case APAGADO_ENTRY:
			state = state_apagado();
			break;

		case PRENDERHW:
			state = state_prenderHW();
			break;

		case PRENDERSW:
			state = state_prenderSW();
			break;

		case PBDONE:
			// Espero prendido
			state = state_pbdone();
			break;

		case CPAS:
			state = state_cpas();
			break;

		case APAGADO_EXIT:
			state = state_exit();
			// Cambio de estado.
			// Del modo OFFLINE siempre salgo a STARTING.
			return(PRENDIDO_OFFLINE);
			break;

		default:
			xprintf_P( PSTR("COMMS: state APAGADO ERROR !!.\r\n\0"));
			state = APAGADO_ENTRY;
			break;
		}
	}

	return(-1);

}
//------------------------------------------------------------------------------------
static int8_t state_exit(void)
{
	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:EXIT\r\n\0"));
	return(-1);
}
//------------------------------------------------------------------------------------
static int8_t state_cpas(void)
{
	// Aqui el modem ya se inicializo ( PBDONE ) y respondio un AT.
	// Ademas yo esperamos 10s para asegurarnos que esta inicializado.
	// Este comando nos indica si el modem esta listo a recibir comandos ( READY )
	// La respuesta puede llegar a demorar hasta 10s.

uint8_t timeout = 0;
int8_t tryes;
int8_t cmd_rsp;

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS in:\r\n\0"));

	// Primero probamos con un AT
	cmd_rsp = FSM_sendATcmd( 1, "AT\r", "OK" );
	if ( cmd_rsp != ATRSP_EXPECTED ) {
		// Si no responde no puedo seguir.
		gprs_sw_pwr();
		return(PRENDERSW);
	}

	// Envio el comando hasta 3 veces esperando 10,20,30 s c/vez.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes <= 3; tryes++ ) {

		// Espera progresiva de a 15s.
		timeout = 15 * ( 1 + tryes );
		cmd_rsp = FSM_sendATcmd( timeout , "AT+CPAS\r", "CPAS: 0" );

		if (cmd_rsp	== ATRSP_EXPECTED ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:CPAS out: OK (%d)\r\n"), tryes );
			return ( APAGADO_EXIT );

		} else if ( cmd_rsp == ATRSP_TIMEOUT ) {
			// Dio timeout: reintento.
			xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS: TIMEOUT.\r\n") );

		} else if ( cmd_rsp == ATRSP_ERR ) {
			// ERROR: apago sw y reintento.
			xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS out: ERR.\r\n"));
			// Hago un switch para apagarlo ??
			gprs_sw_pwr();
			return(PRENDERSW);

		} else if ( cmd_rsp == ATRSP_UNKNOWN ) {
			// UNKNOWN: apago sw y reintento.
			xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS out: ERROR UNKN.\r\n"));
			// Hago un switch para apagarlo ??
			gprs_sw_pwr();
			return(PRENDERSW);
		}

		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:CPAS retry (%d)\r\n\0"), tryes );
	}

	// No puedo en 3 veces responder la secuencia CPAS: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS ERROR.\r\n"));
	// Hago un switch para apagarlo ??
	gprs_sw_pwr();
	return( PRENDERSW);

}
//------------------------------------------------------------------------------------
static int8_t state_pbdone(void)
{
	// Inicializa las comunicaciones.
	// Espera hasta 15 s una respuesta de inicializacion del modem.
	// Si responde PBDONE, espero y doy un AT.

uint16_t timeout = 0;
int8_t exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PBDONE in\r\n"));
	// PBDONE.
	timeout = 150 * ( 1 + prender_sw_tryes ) ;	// En intervalos de 100 ms.
	if ( gprs_check_response ( timeout, "PB DONE" ) ) {
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:PBDONE out: OK\r\n"));
		gprs_print_RX_buffer();
		// Espero 10s que termine de inicializarze
		vTaskDelay( ( TickType_t)( 15000 / portTICK_RATE_MS ) );

		// No quiero ECHO en los comandos
		FSM_sendATcmd( 3, "ATE0\r", "OK" );

		// Leemos el voltaje de alimentacion
		FSM_sendATcmd( 3, "AT+CBC\r", "OK" );
		exit_code = CPAS;

	} else {
		// Dio timeout: apago sw y reintento.
		xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PBDONE out: TIMEOUT.\r\n"));
		// Hago un switch para apagarlo ??
		gprs_sw_pwr();
		exit_code = PRENDERSW;
	}

	gprs_print_RX_buffer();
	return(exit_code);

}
//------------------------------------------------------------------------------------
static int8_t state_prenderSW(void)
{
	//  Genero un pulso en el pin on/off del modem para prenderlo.

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PRENDERSW in.\r\n\0"));

	prender_sw_tryes++;
	if ( prender_sw_tryes > MAXSWTRYESPRENDER ) {
		// Apago el modem
		gprs_apagar();
		xCOMMS_stateVars.gprs_prendido = false;
		xCOMMS_stateVars.gprs_inicializado = false;
		return( PRENDERHW );

	} else {
		//
		gprs_sw_pwr();
		return ( PBDONE );
	}

	return(-1);
}
//------------------------------------------------------------------------------------
static int8_t state_prenderHW(void)
{

	//  Activa la alimentacion del modem y genera una espera basada en las veces de intento.

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PRENDERHW in.\r\n\0"));

	prender_hw_tryes++;
	if ( prender_hw_tryes > MAXHWTRYESPRENDER ) {
		// Maximo nro. de reintentos de prender sin resultado
		// Apago el modem
		gprs_apagar();
		xCOMMS_stateVars.gprs_prendido = false;
		xCOMMS_stateVars.gprs_inicializado = false;
		return( APAGADO_ENTRY );

	} else {

		// Para salir del tickless
		xCOMMS_stateVars.gprs_prendido = true;
		// Prendo la fuente del modem (HW)
		gprs_hw_pwr_on(prender_hw_tryes);
		prender_sw_tryes = 0;
		return ( PRENDERSW );
	}

	return(-1);
}
//------------------------------------------------------------------------------------
static int8_t state_apagado(void)
{
	// Apago el modem y quedo esperando que pase el tiempo.

static bool starting_flag = true;

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado: in.\r\n\0"));

	// Si llegue al maximo de errores de comunicaciones reseteo al micro
	if ( xCOMMS_stateVars.errores_comms >= MAX_ERRORES_COMMS ) {
		xprintf_PD( DF_COMMS, PSTR("COMMS: RESET x MAX_ERRORES_COMMS\r\n\0"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */
	}

	// Apago el modem
	gprs_apagar();
	xCOMMS_stateVars.gprs_prendido = false;
	xCOMMS_stateVars.gprs_inicializado = false;

	// Calculo cuanto tiempo voy a esperar apagado (Al menos siempre espero 10s )
	if ( comms_conf.timerDial < 10 ) {
		awaittime_for_dial = 10;
	} else {
		awaittime_for_dial = comms_conf.timerDial;
	}

	if ( starting_flag ) {
		// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
		awaittime_for_dial = 10;
		starting_flag = false;	// Ya no vuelvo a esperar 10s. por arranque.
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: state apagado: await %lu s\r\n\0"), awaittime_for_dial );

	// Espero durmiendo de a 10s para poder entrar en tickless
	while ( awaittime_for_dial > 0 )  {

		// Espero de a 10s para poder entrar en tickless.
		vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );
		awaittime_for_dial -= 10;

		// Expiro el tiempo ?.
		if (  awaittime_for_dial <= 0 ) {
			awaittime_for_dial = 0;
			break;
		}

		// Proceso las señales:
//		if ( xCOMMS_SGN_REDIAL()) {
//			break;
//		}
	}

	// Siempre salgo a prender el modem
	prender_hw_tryes = 0;

	// Aviso a la tarea de RX que se despierte ( para leer las respuestas del AT ) !!!
	while ( xTaskNotify( xHandle_tkCommsRX, SGN_WAKEUP , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

	xprintf_P(PSTR("FECHA: "));
	RTC_read_time();

	return(PRENDERHW);

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
int32_t pubcomms_awaittime_for_dial(void)
{
	return(awaittime_for_dial);
}
//------------------------------------------------------------------------------------

