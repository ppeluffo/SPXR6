/*
 * l_psensor.c
 *
 *  Created on: 23 ago. 2019
 *      Author: pablo
 */



#include "l_i2c.h"
#include "l_bps120.h"


//------------------------------------------------------------------------------------
int16_t bps120_raw_read( char *data )
{
	// EL sensor BOURNS bps120 mide presion diferencial.
	// Por lo tanto retorna PRESION de acuerdo a la funcion de transferencia
	// con la pMax la de la hoja de datos.
	// De acuerdo a la hoja de datos, la funcion de transferencia es:
	// p(psi) = (pmax - pmin ) * ( counts - 0.1Max) / ( 0.8Max) + pmin
	// pmin = 0
	// Max = 16384 ( 14 bits )
	// 0.1xMax = 1638
	// 0.8xMax = 13107
	// counts es el valor leido del sensor.
	// PMAX = 1.0 psi
	// PMIN = 0 psi

int16_t rcode = 0;

	// Leo 2 bytes del sensor de presion.
	I2C_get_semaphore();
	rcode =  I2C_read( BUSADDR_BPS120, 0x00, 0x00, data, 0x02 );
	I2C_release_semaphore();

	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf_P(PSTR("ERROR: BPS120_raw_read recovering i2c bus\r\n\0") );
		I2C_reinit_devices();
	}

	return( rcode );
}
//------------------------------------------------------------------------------------
