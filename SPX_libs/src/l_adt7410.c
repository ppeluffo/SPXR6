/*
 * l_adt7410.c
 *
 *  Created on: 1 nov. 2019
 *      Author: pablo
 */
#include "l_i2c.h"
#include "l_adt7410.h"


//------------------------------------------------------------------------------------
int16_t adt7410_raw_read( char *data )
{

int16_t rcode = 0;

	// Leo 2 bytes del sensor de presion.
	// El resultado es de 14 bits.
	I2C_get_semaphore();
	rcode =  I2C_read( BUSADDR_ADT7410, 0x00, 0x00, data, 0x02 );
	I2C_release_semaphore();

	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf_P(PSTR("ERROR: ADT7410_raw_read recovering i2c bus\r\n\0") );
		I2C_reinit_devices();
	}

	return( rcode );
}
//------------------------------------------------------------------------------------


