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

typedef enum { APAGADO_ENTRY, PBDONE, CPAS, APAGADO_EXIT } t_states_apagado;

static int8_t state;
static int32_t awaittime_for_dial;

static void state_apagado(void);
static bool state_pbdone(void);
static bool state_cpas(void);
static void state_exit(void);

void pv_prender_modem(void);
void pv_apagar_modem(void);

#define DEEP_SLEEP ( ( xCOMMS_stateVars.modem_starts == -1) || ( xCOMMS_stateVars.modem_starts == MAX_MODEM_STARTS ) )
//------------------------------------------------------------------------------------
int8_t tkXComms_APAGADO(void)
{

	xprintf_PD( DF_COMMS, PSTR("COMMS: state apagado\r\n"));

	state = APAGADO_ENTRY;

	// loop
	for( ;; )
	{
		u_wdg_kick(WDG_COMMS, 300);
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

		switch ( state ) {
		case APAGADO_ENTRY:
			state_apagado();
			state = PBDONE;
			break;

		case PBDONE:
			// Espero prendido
			if ( state_pbdone() ) {
				state = CPAS;
			} else {
				state = APAGADO_ENTRY;
			}
			break;

		case CPAS:
			if ( state_cpas() ) {
				state = APAGADO_EXIT;
			} else {
				state = APAGADO_ENTRY;
			}
			break;

		case APAGADO_EXIT:
			state_exit();
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
static void state_exit(void)
{
	xprintf_P( PSTR("COMMS: apagado:EXIT\r\n\0"));
}
//------------------------------------------------------------------------------------
static bool state_cpas(void)
{
	// Aqui el modem ya se inicializo ( PBDONE ) y respondio un AT.
	// Ademas yo esperamos 10s para asegurarnos que esta inicializado.
	// Este comando nos indica si el modem esta listo a recibir comandos ( READY )
	// La respuesta puede llegar a demorar hasta 10s.

uint8_t timeout = 0;
int8_t tryes;
int8_t cmd_rsp;
uint32_t init_ticks = sysTicks;

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS in:\r\n\0"));

	// Envio el comando hasta 3 veces esperando 10,20,30 s c/vez.
	// Si no responde mando un AT.
	for ( tryes = 0; tryes <= 3; tryes++ ) {

		// Espera progresiva de a 15s.

		init_ticks = sysTicks;
		vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
		cmd_rsp = FSM_sendATcmd( 10 , "AT+CPAS\r" );

		if (cmd_rsp	== ATRSP_OK ) {
			// RSP OK: Veo si es lo que espero
			if ( gprs_check_response ( 1 * SEC_CHECK_RSP, "+CPAS: 0" ) ) {
				xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:CPAS out: OK (%d) (%.3f)\r\n"), tryes, ELAPSED_TIME_SECS(init_ticks));
				return (true );
			} else {
				// Respondio al comando pero no es la respuesta esperada.
				// Espero para reintentar el comando
				xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:CPAS out: FAIL (t=%d) (rsp=%d) (et=%.3f)\r\n"), tryes, cmd_rsp, ELAPSED_TIME_SECS(init_ticks));
			}

		} else if ( cmd_rsp == ATRSP_ERROR ) {
			xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS ERROR. (rsp=%d) (et=%.3f)\r\n"), cmd_rsp, ELAPSED_TIME_SECS(init_ticks));
			return( false );
		}

		// OK o TIMEOUT: Espero y repito el comando
		vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
		FSM_sendATcmd( 2, "AT\r" );
		// Reintento
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:CPAS retry (%d) (%.3f)\r\n"), tryes, ELAPSED_TIME_SECS(init_ticks));
		timeout = 10 * ( 1 + tryes );	// secs
		vTaskDelay( ( TickType_t)( (timeout * 1000) / portTICK_RATE_MS ) );

	}

	// No puedo en 3 veces responder la secuencia CPAS: Salgo a apagar y prender.
	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:CPAS ERROR. (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
	return( false );

}
//------------------------------------------------------------------------------------
static bool state_pbdone(void)
{
	// Inicializa las comunicaciones.
	// Espera hasta 20 s una respuesta de inicializacion del modem.
	// Si responde PBDONE, espero y doy un AT.

uint16_t timeout = 0;
int8_t retS = false;
uint32_t init_ticks = sysTicks;
int8_t at_exit_code = -1;

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PBDONE in (%d s.)\r\n"), timeout );

	// PBDONE.
	if ( gprs_check_response ( 20 * SEC_CHECK_RSP, "PB DONE" ) ) {
		// Respondio bien lo esperado
		gprs_print_RX_buffer();
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:PBDONE out: OK (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));

		// Espero 10s que termine de inicializarze
		timeout = 5 * ( 1 + xCOMMS_stateVars.modem_starts ) ;	// secs
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:awaiting %d s...\r\n"), timeout );
		vTaskDelay( ( TickType_t)( (timeout * 1000) / portTICK_RATE_MS ) );

		at_exit_code = FSM_sendATcmd( 2, "AT\r" );

		// Pongo ECHO en los comandos de configuracion para
		// seguir que va haciendo. Luego en online lo saco
		//vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
		//if ( ( at_exit_code = FSM_sendATcmd( 3, "ATE1\r" )) != ATRSP_OK ) {
		//	xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:ATE1 ERROR (%d)\r\n"), at_exit_code );
		//}

		// Leemos el voltaje de alimentacion
		vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
		if ( ( at_exit_code = FSM_sendATcmd( 5, "AT+CBC\r" )) != ATRSP_OK ) {
			xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:AT+CBC ERROR (%d)\r\n"), at_exit_code);
		}

		retS = true;
		xprintf_PD( DF_COMMS,  PSTR("COMMS: apagado:PBDONE out: ATs (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));

	} else {
		// Dio timeout: No envio el PBDONE
		gprs_print_RX_buffer();
		xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PBDONE out: TIMEOUT. (%.3f)\r\n"), ELAPSED_TIME_SECS(init_ticks));
		// Hago un switch para apagarlo ??
		gprs_sw_pwr();
		retS = false;
	}

	return(retS);

}
//------------------------------------------------------------------------------------
static void state_apagado(void)
{
	// Apago el modem y quedo esperando que pase el tiempo.

	xprintf_P( PSTR("COMMS: apagado: in.\r\n\0"));

// ENTRY
	pv_apagar_modem();
	xCOMMS_stateVars.csq = 0;

	// Calculo cuanto tiempo voy a esperar apagado (Al menos siempre espero 10s )
	// DEEP SLEEP es cuando termine de transmitir bien o llegue al maximo de reintentos
	// Si no pude, duermo 10s y vuelvo a reintentar.
	if ( DEEP_SLEEP ) {
		awaittime_for_dial = comms_conf.timerDial;
	} else {
		awaittime_for_dial = 10;
	}

	xprintf_PD( DF_COMMS, PSTR("COMMS: state apagado: await %lu s\r\n\0"), awaittime_for_dial );

// AWAIT

	// Espero durmiendo de a 10s para poder entrar en tickless
	while ( awaittime_for_dial > 0 )  {

		u_wdg_kick(WDG_COMMS, 300);

		// Espero de a 10s para poder entrar en tickless.
		vTaskDelay( (portTickType)( 10000 / portTICK_RATE_MS ) );
		awaittime_for_dial -= 10;

		// Expiro el tiempo ?.
		if (  awaittime_for_dial <= 0 ) {
			awaittime_for_dial = 0;
			break;
		}

		if ( SPX_SIGNAL( SGN_REDIAL )) {
			SPX_CLEAR_SIGNAL( SGN_REDIAL );
			xprintf_PD( DF_COMMS, PSTR("COMMS: SGN_REDIAL rcvd.\r\n\0"));
			break;
		}

	}

// PRENDER

	// Incremento las veces que intento prender el modem.
	if ( ++xCOMMS_stateVars.modem_starts > MAX_MODEM_STARTS ) {
		// Estaba en DEEP_SLEEP + RESET
		xprintf_PD( DF_COMMS, PSTR("COMMS: RESET x MAX_ERRORES_PRENDER_MODEM...\r\n"));
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */
	}

	pv_prender_modem();

	xprintf_P(PSTR("FECHA: "));
	RTC_read_time();

}
//------------------------------------------------------------------------------------
void pv_prender_modem(void)
{
	// Prendo hardware y softare.
	// Hardware significa activar la fuente.
	// Software es hacer el toggle en el pin PWR del modem.

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:PRENDER_MODEM in.\r\n"));
	xprintf_PD( DF_COMMS, PSTR("COMMS: modem_starts = %d\r\n"), xCOMMS_stateVars.modem_starts );

	// Arranco la task de RX.
	xCOMMS_stateVars.gprs_prendido = true;
	// Aviso a la tarea de RX que se despierte ( para leer las respuestas del AT ) !!!
	while ( xTaskNotify( xHandle_tkGprsRX, SGN_WAKEUP , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}
	// Prendo la fuente del modem (HW)
	gprs_hw_pwr_on(1);
	//
	// Settle time.
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	//
	// SW switch para arrancar.
	gprs_sw_pwr();
	gprs_flush_RX_buffer();

}
//------------------------------------------------------------------------------------
void pv_apagar_modem(void)
{
	// Apago logica ( ordenado ) y fisicamente ( quito la energia ).

	xprintf_PD( DF_COMMS, PSTR("COMMS: apagado:APAGAR_MODEM in.\r\n\0"));

	gprs_apagar();
	xCOMMS_stateVars.gprs_prendido = false;
	xCOMMS_stateVars.gprs_inicializado = false;

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
int32_t pubcomms_awaittime_for_dial(void)
{
	return(awaittime_for_dial);
}
//------------------------------------------------------------------------------------

