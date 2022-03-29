/*
 * tkApp_main.c
 *
 *  Created on: 7 ago. 2021
 *      Author: pablo
 */

#include "tkApp.h"

void tkApp_off( uint8_t app_wdt );
void tkApp_genpulsos( uint8_t app_wdt );

//------------------------------------------------------------------------------------
void tkApp(void * pvParameters)
{

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( ((start_byte >> WDG_APP) & 1 ) != 1 )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("starting tkAplicacion..") );

	if ( systemVars.aplicacion_conf.aplicacion == APP_OFF ) {
		 tkApp_off( WDG_APP );

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_CONSIGNA ) {
		consigna_app_service( WDG_APP );

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_PILOTO ) {
		FSM_piloto_app_service( WDG_APP );

	}  else if ( systemVars.aplicacion_conf.aplicacion == APP_GENPULSOS ) {
		tkApp_genpulsos( WDG_APP );
	}

	// Default
	xprintf_P( PSTR("\r\nAPP: ERROR. Set to off.\r\n"));
	tkApp_off( WDG_APP );
}
//------------------------------------------------------------------------------------
void tkApp_off( uint8_t app_wdt )
{
	// Cuando no hay una funcion especifica habilidada en el datalogger
	// ( solo monitoreo ), debemos dormir para que pueda entrar en
	// tickless

	xprintf_P( PSTR("OFF\r\n"));

	for( ;; )
	{
		u_wdg_kick( app_wdt, 120);
		vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

	}
}
//------------------------------------------------------------------------------------
void tkApp_genpulsos( uint8_t app_wdt )
{
	/* El trabajo lo hace la funcion del callback de un timer.
	 * Esta funcion solo configura y arranca el timer.
	 * El tiempo de espera es de 5s ya que esta función no es para lowpower.
	 * Procesa las señales que le envia la tkDatos al finalizar un poleo para
	 * que pueda recalcular en base al caudal el periodo de los pulsos.
	 */

	xprintf_P( PSTR("GENPULSOS\r\n"));

	genpulsos_init();

	for( ;; )
	{
		u_wdg_kick( app_wdt, 120);
		vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );

		// Si recibo una senal de datos ready: Salgo
		if ( SPX_SIGNAL( SGN_DOSIFICADORA )) {
			SPX_CLEAR_SIGNAL( SGN_DOSIFICADORA );
			genpulsos_ajustar_litrosXtimertick( DF_APP );
		}
	}
}
//------------------------------------------------------------------------------------
