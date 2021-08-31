/*
 * tkApp_main.c
 *
 *  Created on: 7 ago. 2021
 *      Author: pablo
 */

#include "tkApp.h"

void tkApp_off( uint8_t app_wdt );
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
		piloto_app_service( WDG_APP );
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
