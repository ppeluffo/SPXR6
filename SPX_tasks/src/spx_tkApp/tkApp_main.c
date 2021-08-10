/*
 * tkApp_main.c
 *
 *  Created on: 7 ago. 2021
 *      Author: pablo
 */

#include "tkApp.h"

void tkApp_off(void);
//------------------------------------------------------------------------------------
void tkApp(void * pvParameters)
{

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	xprintf_P( PSTR("starting tkAplicacion..\r\n\0"));

	if ( systemVars.aplicacion_conf.aplicacion == APP_OFF ) {
		 tkApp_off();

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_CONSIGNA ) {
		consigna_app_service();

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_PILOTO ) {
		piloto_app_service();
	}

	// Default
	tkApp_off();
}
//------------------------------------------------------------------------------------
void tkApp_off(void)
{
	// Cuando no hay una funcion especifica habilidada en el datalogger
	// ( solo monitoreo ), debemos dormir para que pueda entrar en
	// tickless

	xprintf_P( PSTR("APP: OFF\r\n\0"));

	for( ;; )
	{
		vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );
	}
}
//------------------------------------------------------------------------------------
