/*
 * ul_genpulsos.c
 *
 *  Created on: 17 mar. 2022
 *      Author: pablo
 *
 *  Como trabajamos con una frecuencia de ticks de 1000, entonces c/TICK dura 1MS
 *  En las funciones de timers, los tiempos hay que ponerlos en TICKS pero en nuestro
 *  caso es lo mismo ponerlos en MS.
 *  El tamaño de TickType_t es uint16_t por lo tanto el mayor tiempo del timer
 *  va a ser 65536 ms o sea 65s.
 *  Esto hace que para combinaciones de pocos pulsoXmt3 con caudales bajos, el periodo
 *  de los pulsos sea muy alto como para manejarlo solo con el contador del timer.
 *  En este caso debemos implementar un prescaler.
 *
 *  Fundamentos:
 *  LA idea es generar pulsos cada tantos mt3 que se miden.
 *  El tema es que el caudal es variable.
 *  Lo que debemos hacer es medir entonces el volumen que pasa y cuando se llega al valor
 *  esperado genero un pulso.
 *  El caudal lo leo cada timerPoll en la tarea de tkData.
 *  Asumo que durante el siguiente intervalo el caudal es constante.
 *  Para minimizar errores de calculo, el caudal lo calculo integrando en pequeños intervalos,
 *  en particular cada 100ms.
 *  Para esto genero un timer FIJO con intervalo de 100ms.
 *  - Tengo un parámetro de configuración que es pulsosXmt3 por lo tanto, este parámetro me
 *  determina cada cuantos mt3 o lts genero un pulso.
 *  - Por otro lado, cuando llega un timerPoll leo el caudal. En base a este caudal medido
 *  calculo cuantos mts3 o lts corresponden a cada tick.
 *  - En la funcion de callback del timer incremento en c/u de acuerdo al caudal correspondiente
 *  al tick. Cuando llego al valor deseado, genero el pulso y comienzo de nuevo a contar volumen.
 *
 */


#include "ul_genpulsos.h"


StaticTimer_t genpulsos_xTimerBuffer;
TimerHandle_t genpulsos_xTimer;

//------------------------------------------------------------------------------------
void genpulsos_config_defaults(void)
{
	genpulsos_conf.pulsosXmt3 = 100;
	genpulsos_conf.pulsoWidth = 10;
}
//------------------------------------------------------------------------------------
void genpulsos_config_status(void)
{
	if ( genpulsos_svars.modo_testing ) {
		xprintf_P( PSTR("  modo: GENPULSO (*Modo Testing)\r\n"));
	} else {
		xprintf_P( PSTR("  modo: GENPULSO\r\n"));
	}
	xprintf_P( PSTR("    PulsosXmt3=%d\r\n"), genpulsos_conf.pulsosXmt3  );
	xprintf_P( PSTR("    AnchoPulso=%d (ms)\r\n"), genpulsos_conf.pulsoWidth  );
}
//------------------------------------------------------------------------------------
bool genpulsos_config_pulsosXmt3( char *s_pulsosXmt3 )
{
	/* Debe ser 1, 10, 100, 1000
	 */

uint16_t pulsosXmt3 =  atoi(s_pulsosXmt3);
bool retS = false;

	switch( pulsosXmt3 ) {
	case 1:
	case 10:
	case 100:
	case 1000:
		genpulsos_conf.pulsosXmt3 = pulsosXmt3;
		retS = true;
		break;
	default:
		genpulsos_conf.pulsosXmt3 = 100;
		retS = false;
	}

	// Si cambio la configuracion, recargo los nuevos datos. (actualizo)
	CONFIG_LITROS_X_PULSO();

	return(retS);
}
//------------------------------------------------------------------------------------
bool genpulsos_config_pulsoWidth( char *s_pulsoWidth)
{

uint16_t pulsoWidth = atoi(s_pulsoWidth);

	if ( (pulsoWidth > 0) && (pulsoWidth <= 100) ) {
		genpulsos_conf.pulsoWidth = pulsoWidth;
		return (true);
	}
	return(false);
}
//------------------------------------------------------------------------------------
void genpulsos_config_caudal( char *s_caudal)
{
	// Funcion que se usa para testing
	genpulsos_svars.caudal = atof(s_caudal);

}
//------------------------------------------------------------------------------------
bool genpulsos_config_testing( char *s_modo_testing)
{
	// Funcion que se usa para testing
	if (!strcmp_P( strupr(s_modo_testing), PSTR("ON")) ) {
		genpulsos_svars.modo_testing = true;
		return (true);
	}

	if (!strcmp_P( strupr(s_modo_testing), PSTR("OFF")) ) {
		genpulsos_svars.modo_testing = false;
		return (true);
	}

	return(false);

}
//------------------------------------------------------------------------------------
void genpulsos_ajustar_litrosXtimertick( bool debug_flag )
{
	/*
	 * Esta funcion se invoca desde la tarea de la aplicacion genpulsos
	 * cuando llega una señal de tkData indicando que se actualizo el valor del caudal
	 * En base al nuevo caudal recalculo el valor de litrosXtimertick
	 * litrosXtimertick es float para minimizar errores.
	 * ( Ej. 10mts3/h son 0.277 ltsXtick )
	 *
	 */

float caudal;

	// Paso el valor que me manda la aplicacion(actualizo)
	f_GENPULSOS = debug_flag;

	// Actualizo el caudal
	if ( genpulsos_svars.modo_testing ) {
		// Modo testing
		caudal = genpulsos_svars.caudal;
	} else {
		// Leo caudal del sistema
		caudal = get_Q();
	}

	// Calculo en el siguente timerPoll intervalo, cuantos litros son c/tick del timer.(100ms)
	genpulsos_svars.litrosXtimertick = 1000 * caudal * GENPULSOS_TIMER_TICK_MS / 3600000;
	if ( genpulsos_svars.litrosXtimertick > 0 ) {
		genpulsos_svars.periodoPulsos = ( genpulsos_svars.litrosXpulso / genpulsos_svars.litrosXtimertick ) * 100;
	}

	genpulsos_print_vars();
}
//------------------------------------------------------------------------------------
void genpulsos_print_vars(void)
{
	if ( f_GENPULSOS ) {
		xprintf_P( PSTR("PulsosXmt3=%d\r\n"), genpulsos_conf.pulsosXmt3  );
		xprintf_P( PSTR("ltsXpulso=%d(lts)\r\n"), genpulsos_svars.litrosXpulso  );
		xprintf_P( PSTR("PulseWidth=%d(ms)\r\n"), genpulsos_conf.pulsoWidth  );
		if ( genpulsos_svars.modo_testing ) {
			xprintf_P( PSTR("Caudal(testing)=%0.03f(mt3/h)\r\n"), genpulsos_svars.caudal  );
		} else {
			xprintf_P( PSTR("Caudal=%0.03f(mt3/h)\r\n"), genpulsos_svars.caudal  );
		}
		xprintf_P( PSTR("ltsXtick=%0.05f\r\n"), genpulsos_svars.litrosXtimertick  );
		xprintf_P( PSTR("Periodo pulsos=%d(ms)\r\n"), genpulsos_svars.periodoPulsos  );

	}
}
//------------------------------------------------------------------------------------
void genpulsos_Callback( TimerHandle_t xTimer )
{
	/*
	 * Esta funcion se invoca cada vez que expira el timer o sea cada 100ms
	 * Acumulo litrosXtimertick ( lts que pasaron en esos 100ms ) en el contador litros_acumulados.
	 * Si litros_acumulados >= litrosXpulso genero un pulso y reinicio
	 */

	// Si no hay modulo configurado, salgo
	if ( ! MIDO_CAUDAL() )
		return;

	//  En cada timer tick, incremento los litros acumulados
	genpulsos_svars.litros_acumulados += genpulsos_svars.litrosXtimertick;

	// Si llegue al valor de generar un pulso lo genero e inicializo.
	if ( genpulsos_svars.litros_acumulados >= genpulsos_svars.litrosXpulso ) {
		// Genero pulso
		pulse_Amas_Amenos( genpulsos_conf.pulsoWidth );
		genpulsos_svars.litros_acumulados = 0.0;
	}

}
//------------------------------------------------------------------------------------
void genpulsos_init( void )
{

	//
	CONFIG_LITROS_X_PULSO();
	genpulsos_svars.caudal = 0.0;
	genpulsos_svars.litros_acumulados = 0.0;
	genpulsos_svars.litrosXtimertick = 0.0;
	genpulsos_svars.modo_testing = false;

	// Genera en forma indefinida una 'interrupcion' cada 100 ms.
	genpulsos_xTimer = xTimerCreateStatic ("PULS",
			pdMS_TO_TICKS( GENPULSOS_TIMER_TICK_MS ),
			pdTRUE,
			( void * ) 0,
			genpulsos_Callback,
			&genpulsos_xTimerBuffer
			);

	// Arranco el timer.
	xTimerStart(genpulsos_xTimer, 10);

	// Prendo la fuente del driver
	DRV8814_power_on();
	stepper_awake();

	xprintf_P(PSTR("Dosificadora Init.\r\n"));
}
//------------------------------------------------------------------------------------
uint8_t genpulsos_hash(void)
{
	// PLT;PXR:5000;PWIDTH:20;SLOT0:0630,1.20;SLOT1:0745,2.40;SLOT2:1230,3.60;SLOT3:2245,4.80;SLOT4:2345,5.00;

uint8_t hash = 0;
char *p;
uint8_t i = 0;
int16_t free_size = sizeof(hash_buffer);

	// Vacio el buffer temporal
	memset(hash_buffer,'\0', sizeof(hash_buffer));

	i += snprintf_P( &hash_buffer[i], free_size, PSTR("GENPULSOS;PXM3:%04d;PW:%04d"), genpulsos_conf.pulsosXmt3, genpulsos_conf.pulsoWidth );
	//xprintf_P(PSTR("HASH: [%s]\r\n"), hash_buffer);

	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;
	p = hash_buffer;
	while (*p != '\0') {
		hash = u_hash(hash, *p++);
	}

	return(hash);

exit_error:

	xprintf_P( PSTR("GENPULSOS: Hash ERROR !!!\r\n\0"));
	return(0x00);

}
//------------------------------------------------------------------------------------

