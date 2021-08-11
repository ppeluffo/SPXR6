/*
 * ul_consigna.c
 *
 *  Created on: 9 ago. 2021
 *      Author: pablo
 */

#include "ul_consignas.h"

//------------------------------------------------------------------------------------
void consigna_init(void)
{
	DRV8814_init();
}
//------------------------------------------------------------------------------------
bool consigna_config( char *tipo, char *hhmm )
{
	if ( ( tipo == NULL ) || ( hhmm == NULL ) ) {
		return(false);
	}

	if (!strcmp_P( strupr(tipo), PSTR("DIURNA")) ) {
		u_convert_int_to_time_t( atoi( hhmm ), &consigna_conf.consigna_diurna.hour, &consigna_conf.consigna_diurna.min  );
		return(true);
	}

	if (!strcmp_P( strupr(tipo), PSTR("NOCTURNA")) ) {
		u_convert_int_to_time_t( atoi( hhmm ), &consigna_conf.consigna_nocturna.hour, &consigna_conf.consigna_nocturna.min  );
		return(true);
	}

	return(false);

}
//------------------------------------------------------------------------------------
void consigna_set_nocturna(void)
{
	// En consigna nocturna la valvula A (JP28) queda abierta y la valvula B (JP2) cerrada.
	//
	// ( VA close / VB open ) -> ( VA open / VB close )
	// Abro VA ya que esta con pAtm de un lado.
	// Cierro VB.


	// Proporciono corriente.
	DRV8814_power_on();
	// Espero 15s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

	// EL orden importa para que las valvulas no queden a contrapresion
	DRV8814_vclose( 'B', 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );
	DRV8814_vopen( 'A', 100 );

	DRV8814_power_off();

	consigna_aplicada = CONSIGNA_NOCTURNA;
}
//------------------------------------------------------------------------------------
void consigna_set_diurna(void)
{

	// ( VA open / VB close ) -> ( VA close / VB open )
	// Open VB con lo que el punto común de las válvulas queda a pAtm y la VA puede operar correctamente.
	// Close VA.

	// Proporciono corriente.
	DRV8814_power_on();
	// Espero 15s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

	// EL orden importa para que las valvulas no queden a contrapresion
	DRV8814_vopen( 'B', 100 );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	DRV8814_vclose( 'A', 100 );

	DRV8814_power_off();

	consigna_aplicada = CONSIGNA_DIURNA;

}
//------------------------------------------------------------------------------------
void consigna_config_defaults(void)
{
	consigna_conf.consigna_diurna.hour = 5;
	consigna_conf.consigna_diurna.min = 0;
	consigna_conf.consigna_nocturna.hour = 23;
	consigna_conf.consigna_nocturna.min = 0;
}
//------------------------------------------------------------------------------------
void consigna_config_status(void)
{
	xprintf_P( PSTR("  modo: CONSIGNA\r\n"));

	if ( consigna_aplicada == CONSIGNA_DIURNA ) {
		xprintf_P( PSTR("        (*)Diurna: %02d:%02d, Nocturna: %02d:%02d\r\n"),
				consigna_conf.consigna_diurna.hour,
				consigna_conf.consigna_diurna.min,
				consigna_conf.consigna_nocturna.hour,
				consigna_conf.consigna_nocturna.min
				);
	} else if ( consigna_aplicada == CONSIGNA_NOCTURNA ) {
		xprintf_P( PSTR("        Diurna: %02d:%02d, (*)Nocturna: %02d:%02d\r\n"),
				consigna_conf.consigna_diurna.hour,
				consigna_conf.consigna_diurna.min,
				consigna_conf.consigna_nocturna.hour,
				consigna_conf.consigna_nocturna.min
				);
	}

}
//------------------------------------------------------------------------------------
bool consigna_init_service(void)
{
	// Configuracion inicial cuando las salidas estan en CONSIGNA.

RtcTimeType_t rtcDateTime;
uint16_t now = 0;
uint16_t horaConsNoc = 0;
uint16_t horaConsDia = 0;
uint8_t consigna_a_aplicar = 99;

	memset( &rtcDateTime, '\0', sizeof(RtcTimeType_t));

	DRV8814_init();

	// Hora actual en minutos.
	if ( ! RTC_read_dtime(&rtcDateTime) )
		xprintf_P(PSTR("APP: CONSIGNA ERROR: I2C:RTC init_consignas\r\n\0"));

	// Caso 1: C.Diurna < C.Nocturna
	//           C.diurna                      C.nocturna
	// |----------|-------------------------------|---------------|
	// 0         hhmm1                          hhmm2            24
	//   nocturna             diurna                 nocturna

	now = rtcDateTime.hour * 60 + rtcDateTime.min;
	horaConsDia = consigna_conf.consigna_diurna.hour * 60 + consigna_conf.consigna_diurna.min;
	horaConsNoc = consigna_conf.consigna_nocturna.hour * 60 + consigna_conf.consigna_nocturna.min;

	if ( horaConsDia < horaConsNoc ) {
		// Caso A:
		if ( now <= horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso B:
		if ( ( horaConsDia <= now ) && ( now <= horaConsNoc )) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}

		// Caso C:
		if ( now > horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
	}

	// Caso 2: C.Nocturna < Diurna
	//           C.Nocturna                      C.diurna
	// |----------|-------------------------------|---------------|
	// 0         hhmm2                          hhmm1            24
	//   diurna             nocturna                 diurna

	if (  horaConsNoc < horaConsDia ) {
		// Caso A:
		if ( now <= horaConsNoc ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
		// Caso B:
		if ( ( horaConsNoc <= now ) && ( now <= horaConsDia )) {
			consigna_a_aplicar = CONSIGNA_NOCTURNA;
		}
		// Caso C:
		if ( now > horaConsDia ) {
			consigna_a_aplicar = CONSIGNA_DIURNA;
		}
	}

	// Aplico la consigna
	switch (consigna_a_aplicar) {
	case 99:
		// Incompatibilidad: seteo por default.
		xprintf_P( PSTR("APP: CONSIGNA ERROR al setear consignas: horas incompatibles\r\n\0"));
		consigna_conf.consigna_diurna.hour = 05;
		consigna_conf.consigna_diurna.min = 30;
		consigna_conf.consigna_diurna.hour = 23;
		consigna_conf.consigna_diurna.min = 30;
		break;
	case CONSIGNA_DIURNA:
		consigna_set_diurna();
		xprintf_P(PSTR("APP: Set init CONSIGNA diurna: %02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min);
		break;
	case CONSIGNA_NOCTURNA:
		consigna_set_nocturna();
		xprintf_P(PSTR("APP: Set init CONSIGNA nocturna: %02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min);
		break;
	}

	return(true);

}
//------------------------------------------------------------------------------------
void consigna_app_service( uint8_t app_wdt )
{
	// Las salidas estan configuradas para modo consigna.
	// c/25s reviso si debo aplicar una o la otra y aplico
	// Espero con lo que puedo entrar en tickless

RtcTimeType_t rtcDateTime;

	xprintf_P("APP: CONSIGNA\r\n\0");

	if ( !consigna_init_service() )
		return;

	for (;;) {

		u_wdg_kick( app_wdt, 120);
		vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

		// Chequeo y aplico.
		// Las consignas se chequean y/o setean en cualquier modo de trabajo, continuo o discreto
		memset( &rtcDateTime, '\0', sizeof(RtcTimeType_t));
		if ( ! RTC_read_dtime(&rtcDateTime) ) {
			xprintf_P(PSTR("APP: CONSIGNA ERROR: I2C:RTC chequear_consignas\r\n\0"));
			continue;
		}

		// Consigna diurna ?
		if ( ( rtcDateTime.hour == consigna_conf.consigna_diurna.hour ) &&
				( rtcDateTime.min == consigna_conf.consigna_diurna.min )  ) {

			consigna_set_diurna();
			xprintf_P(PSTR("APP: Set CONSIGNA diurna %02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min);
			continue;
		}

		// Consigna nocturna ?
		if ( ( rtcDateTime.hour == consigna_conf.consigna_nocturna.hour ) &&
				( rtcDateTime.min == consigna_conf.consigna_nocturna.min )  ) {

			consigna_set_nocturna();
			xprintf_P(PSTR("APP: Set CONSIGNA nocturna %02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min);
			continue;
		}

	}
}
//------------------------------------------------------------------------------------
uint8_t consigna_hash(void)
{

uint8_t hash = 0;
char *p;
uint8_t i = 0;
int16_t free_size = sizeof(hash_buffer);

	// Vacio el buffer temporal
	memset(hash_buffer,'\0', sizeof(hash_buffer));

	i += snprintf_P( &hash_buffer[i], free_size, PSTR("CONSIGNA,"));
	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;

	i += snprintf_P(&hash_buffer[i], free_size, PSTR("%02d%02d,"), consigna_conf.consigna_diurna.hour, consigna_conf.consigna_diurna.min );
	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;

	i += snprintf_P(&hash_buffer[i], free_size, PSTR("%02d%02d"), consigna_conf.consigna_nocturna.hour, consigna_conf.consigna_nocturna.min );
	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;

	p = hash_buffer;
	while (*p != '\0') {
		hash = u_hash(hash, *p++);
	}
	return(hash);

exit_error:

	xprintf_P( PSTR("CONSIGNAS: Hash ERROR !!!\r\n\0"));
	return(0x00);

}
//------------------------------------------------------------------------------------
