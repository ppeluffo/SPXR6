/*
 * ul_oceanus.c
 *
 *  Created on: 11 may. 2022
 *      Author: pablo
 *
 *  Al leer del AUX y descartar los caracteres no imprimibles, obtenemos algo del
 *  estilo
 *  5,@\FO000TEMPER:22rZ3*@\FO000HUM:54%RHsZ,#$@\FO000Z
 *  6-@\FO000PM2.5:7ug/m3Z6-@\FO000PM10:40ug/m3~DZ5,$@\FO000TSP:72ug/m3zZ
 *
 *  TEMPER:22
 *  HUM:54
 *  PM2.5:7
 *  PM10:40
 *  TSP:72
 *
 */

#include "ul_oceanus.h"
#include "tkApp.h"
#include "tkComms.h"

float oceanus_proces_TEMPER( char *str);
float oceanus_proces_HUM( char *str);
float oceanus_proces_PM25( char *str, uint8_t size );
float oceanus_proces_PM10( char *str, uint8_t size );
float oceanus_proces_TSP( char *str, uint8_t size );

//------------------------------------------------------------------------------------
void tkApp_oceanus( uint8_t app_wdt )
{

	/* La estacion manda datos cada 60 secs así que espero y leo c/30secs
	 * La tarea tkAux se encarga de ir leyendo los datos y dejarlos en el aux_buffer.
	 * Parseo el aux_buffer buscando patrones de las magnitudes.
	 */

uint8_t i;
char *p = NULL;
char localStr[8] = { 0 };

	xprintf_P( PSTR("OCEANUS\r\n"));

	oceanus_init();

	for( ;; )
	{
		u_wdg_kick( app_wdt, 120);
		vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );

		// Paso1: Leo el AUX port y almaceno en el buffer.
		// Lo hace la tarea tkAux.

		xprintf_PD( DF_APP,PSTR("APP_OCEANUS: %s\r\n"), aux_rxbuffer.buffer );
		// Paso2: Recorro los canales buscando el nombre en el string de datos recibidos.
		//        Si lo encuentro extraigo el valor de la magnitud.

		for ( i = 0; i < MODBUS_CHANNELS; i++ ) {
			if ( strcmp ( modbus_conf.channel[i].name, "X" ) == 0 ) {
				break;
			}

			//xprintf_P( PSTR("OCEANUS: %s\r\n"), modbus_conf.channel[i].name );
			// Busco en aux_buffer el string con el nombre de la magnitud

			if ( (p = strstr(aux_rxbuffer.buffer, modbus_conf.channel[i].name )) ) {
				p = strchr( p, ':');
				p++;
				// p apunta a la primera cifra del valor
				memset(localStr,'\0',sizeof(localStr));
				strncpy(localStr, p, sizeof(localStr));

				if ( strcmp_P( modbus_conf.channel[i].name, PSTR("TEMPER")) == 0 ) {
					oceanusValues[i] = oceanus_proces_TEMPER(localStr);

				} else if ( strcmp_P( modbus_conf.channel[i].name, PSTR("HUM")) == 0 ) {
					oceanusValues[i] = oceanus_proces_HUM(localStr);

				} else if ( strcmp_P( modbus_conf.channel[i].name, PSTR("PM2.5")) == 0 ) {
					oceanusValues[i] = oceanus_proces_PM25(localStr, 8 );

				} else if ( strcmp_P( modbus_conf.channel[i].name, PSTR("PM10")) == 0 ) {
					oceanusValues[i] = oceanus_proces_PM10(localStr, 8);

				} else if ( strcmp_P( modbus_conf.channel[i].name, PSTR("TSP")) == 0 ) {
					oceanusValues[i] = oceanus_proces_TSP(localStr, 8);
				}
			}
		}

		// Paso3: Vacio el AUX buffer
		if ( aux_rxbuffer_full() )
			aux_rxbuffer_reset();

	}
}
//------------------------------------------------------------------------------------
float oceanus_proces_TEMPER( char *str)
{
	/* El str contiene algo como 263Z3Z.
	 * Solo uso 3 bytes que representan la temperatura como 26.3
	 */

float temp;

	//temp = (( str[0] - '0') * 100 + (str[1] - '0') * 10 + (str[2] - '0')) / 10.0;
	temp = (( str[0] - '0') * 10 + (str[1] - '0'));

	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: TEMPER S0=%c, S1=%c, S2=%c\r\n"), str[0], str[1], str[2] );
	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: TEMPER=%0.2f\r\n"), temp);

	return(temp);
}
//------------------------------------------------------------------------------------
float oceanus_proces_HUM( char *str)
{
	/*
	 * La humedad relativa son 2 digitos del 0 al 99
	 */

float hum;

	hum = (( str[0] - '0') * 10 + (str[1] - '0'));

	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: HUM S0=%c, S1=%c, S2=%c\r\n"), str[0], str[1], str[2] );
	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: HUM=%0.2f\r\n"), hum);
	return(hum);

}
//------------------------------------------------------------------------------------
float oceanus_proces_PM25( char *str, uint8_t size )
{
	/*
	 * str contiene algo del tipo 5ug/m
	 * Borro todos los caracteres no numericos y convierto a entero
	 *
	 */
float pm25;
uint8_t i;

	for (i=0; i<size;i++) {
		if ( ! isdigit(str[i])) {
			str[i] = '\0';
		}
	}

	pm25 = atof(str);

	//xprintf_P(PSTR("OCEANUS: str=%s\r\n"), str );
	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: PM2.5=%0.2f\r\n"), pm25 );
	return(pm25);

}
//------------------------------------------------------------------------------------
float oceanus_proces_PM10( char *str, uint8_t size )
{
float pm10;
uint8_t i;

	for (i=0; i<size;i++) {
		if ( ! isdigit(str[i])) {
			str[i] = '\0';
		}
	}

	pm10 = atof(str);

	//xprintf_P(PSTR("OCEANUS: str=%s\r\n"), str );
	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: PM10=%0.2f\r\n"), pm10 );
	return(pm10);
}
//------------------------------------------------------------------------------------
float oceanus_proces_TSP( char *str, uint8_t size )
{

float tsp;
uint8_t i;

	for (i=0; i<size;i++) {
		if ( ! isdigit(str[i])) {
			str[i] = '\0';
		}
	}

	tsp = atof(str);

	//xprintf_P(PSTR("OCEANUS: str=%s\r\n"), str );
	xprintf_PD( DF_APP,PSTR("APP_OCEANUS: TSP=%0.2f\r\n"), tsp );
	return(tsp);
}
//------------------------------------------------------------------------------------
void oceanus_init(void)
{

uint8_t i;

	// Le indico al driver de la AUX1 que descarte los caracteres no imprimibles.
	aux1_discard_noprintables = true;

	for (i=0; i<OCEANUS_CHANNELS; i++) {
		oceanusValues[i] = 0.0;
	}
}
//------------------------------------------------------------------------------------
bool oceanus_read( float oceanus_data[] )
{
	/*
	 * Devuelve los ultimos datos leidos
	 */

uint8_t i;

	if ( systemVars.aplicacion_conf.aplicacion == APP_OCEANUS ) {
		// Copio los datos a la salida
		for (i=0; i<OCEANUS_CHANNELS; i++) {
			oceanus_data[i] = oceanusValues[i];
		}
		return (true);

	} else {
		return (false);
	}
}
//------------------------------------------------------------------------------------
