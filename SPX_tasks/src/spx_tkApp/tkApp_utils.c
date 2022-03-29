/*
 * tkApp_utils.c
 *
 *  Created on: 7 ago. 2021
 *      Author: pablo
 */

#include "tkApp.h"

//------------------------------------------------------------------------------------
bool aplicacion_config( char *modo )
{

	if ( strcmp_P( strupr(modo), PSTR("OFF\0")) == 0) {
		systemVars.aplicacion_conf.aplicacion = APP_OFF;
		return(true);
	}

	if ( strcmp_P( strupr(modo), PSTR("PILOTO\0"))  == 0)  {
		systemVars.aplicacion_conf.aplicacion = APP_PILOTO;
		return(true);
	}

	if ( strcmp_P( strupr(modo), PSTR("CONSIGNA\0"))  == 0 )  {
		systemVars.aplicacion_conf.aplicacion = APP_CONSIGNA;
		return(true);
	}

	if ( strcmp_P( strupr(modo), PSTR("GENPULSOS"))  == 0 )  {
		systemVars.aplicacion_conf.aplicacion = APP_GENPULSOS;
		return(true);
	}


	return(false);
}
//------------------------------------------------------------------------------------
void aplicacion_config_status(void)
{

	xprintf_P( PSTR(">Aplicacion:\r\n"));

	if ( systemVars.aplicacion_conf.aplicacion == APP_OFF ) {
		xprintf_P( PSTR("  modo: OFF\r\n"));

	} else 	if ( systemVars.aplicacion_conf.aplicacion == APP_GENPULSOS ) {
		genpulsos_config_status();

	} else 	if ( systemVars.aplicacion_conf.aplicacion == APP_CONSIGNA ) {
		consigna_config_status();

	} else 	if ( systemVars.aplicacion_conf.aplicacion == APP_PILOTO ) {
		piloto_config_status();
	};

}
//------------------------------------------------------------------------------------
void aplicacion_config_defaults(void)
{
	systemVars.aplicacion_conf.aplicacion = APP_OFF;
	consigna_config_defaults();
	piloto_config_defaults();
	genpulsos_config_defaults();

}
//------------------------------------------------------------------------------------
uint8_t aplicacion_hash(void)
{

uint8_t hash = 0;
char *p;
uint8_t i = 0;
int16_t free_size = sizeof(hash_buffer);

	// Vacio el buffer temporal
	memset(hash_buffer,'\0', sizeof(hash_buffer));

	if ( systemVars.aplicacion_conf.aplicacion == APP_OFF ) {
		i += snprintf_P( &hash_buffer[i], free_size, PSTR("OFF"));
		free_size = (  sizeof(hash_buffer) - i );
		if ( free_size < 0 )
			goto exit_error;
		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}
		return(hash);

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_GENPULSOS ) {
		return( genpulsos_hash() );

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_CONSIGNA ) {
		return( consigna_hash() );

	} else if ( systemVars.aplicacion_conf.aplicacion == APP_PILOTO ) {
		return( piloto_hash() );
	}

exit_error:

	xprintf_P( PSTR("APP: Hash ERROR !!!\r\n\0"));
	return(0x00);
}
//------------------------------------------------------------------------------------
