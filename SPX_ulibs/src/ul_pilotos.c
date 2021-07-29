/*
 * ul_pilotos.c
 *
 *  Created on: 13 may. 2021
 *      Author: pablo
 */

#include "ul_pilotos.h"

TimerHandle_t plt_pulse_xTimer;
StaticTimer_t plt_pulse_xTimerBuffer;
void pv_piloto_pulse_TimerCallback( TimerHandle_t xTimer );
void pv_piloto_calcular_parametros_ajuste( void );
void pv_piloto_print_parametros(void);
bool pv_piloto_determinar_canales_presion(void);
void pv_piloto_leer_presiones( int8_t samples, uint16_t intervalo_secs );

//------------------------------------------------------------------------------------
void piloto_run_presion_test(char *s_pRef )
{
	// Inicializa la estructura de control con una flag para iniciar el test
	// y la presion de referencia a la cual llevar el piloto.

	PLTCB.start_test = true;
	PLTCB.pRef = atof(s_pRef);

}
//------------------------------------------------------------------------------------
void piloto_run_stepper_test(char *s_dir, char *s_npulses, char *s_pwidth )
{
	// Funcion invocada desde cmdline.
	// Inicializa la estructura de control para hacer el test.

	if ( strcmp_P( strupr(s_dir), PSTR("FW")) == 0 ) {
		PLTCB.dir = STEPPER_FWD;
	} else if ( strcmp_P( strupr(s_dir), PSTR("REV")) == 0) {
		PLTCB.dir = STEPPER_REV;
	} else {
		xprintf_P(PSTR("Error en direccion\r\n"));
		return;
	}

	PLTCB.pulsos_a_aplicar = atoi(s_npulses);
	PLTCB.pwidth = atoi(s_pwidth);
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("STEPPER driver pulses start..\r\n"));
	stepper_awake();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

}
//------------------------------------------------------------------------------------
void piloto_setup(void )
{
	// Configuro el timer que va a generar los pulsos del stepper
	// Se debe correr antes que empieze el RTOS

	plt_pulse_xTimer = xTimerCreateStatic ("PLTPULSE",
			pdMS_TO_TICKS( 20 ),
			pdTRUE,
			( void * ) 0,
			pv_piloto_pulse_TimerCallback,
			&plt_pulse_xTimerBuffer
			);

}
//------------------------------------------------------------------------------------
void piloto_config_defaults(void)
{

	 // Configura el default del piloto con todos los valores en 0

uint8_t slot;

	piloto_conf.piloto_enabled = false;

	for (slot=0; slot < MAX_PILOTO_PSLOTS; slot++) {
		piloto_conf.pltSlots[slot].hour = 0;
		piloto_conf.pltSlots[slot].min = 0;
		piloto_conf.pltSlots[slot].presion = 0.0;
	}

}
//------------------------------------------------------------------------------------
void piloto_print_status( void )
{
	// Muestra en cmdline la configuracion.

uint8_t slot;

	xprintf_P( PSTR(">Piloto Slots:\r\n"));
	xprintf_P( PSTR("  "));
	for (slot=0; slot<MAX_PILOTO_PSLOTS ;slot++) {
		xprintf_P( PSTR("[%02d]%02d:%02d->%0.2f "), slot, piloto_conf.pltSlots[slot].hour, piloto_conf.pltSlots[slot].min, piloto_conf.pltSlots[slot].presion  );
	}
	xprintf_P( PSTR("\r\n"));
}
//------------------------------------------------------------------------------------
bool piloto_config( char *param1, char *param2, char *param3, char *param4 )
{
	// Configura un slot

uint8_t slot;

	if (!strcmp_P( strupr(param1), PSTR("ON\0"))) {
		piloto_conf.piloto_enabled = true;
		return(true);
	}

	if (!strcmp_P( strupr(param1), PSTR("OFF\0"))) {
		piloto_conf.piloto_enabled = false;
		return(true);
	}

	if (!strcmp_P( strupr(param1), PSTR("SLOT\0"))) {

		// Intervalos tiempo:presion:

		slot = atoi(param2);
		if ( slot < MAX_PILOTO_PSLOTS ) {
			if ( param3 != NULL ) {
				u_convert_int_to_time_t( atoi( param3), &piloto_conf.pltSlots[slot].hour, &piloto_conf.pltSlots[slot].min  );
			}
			if ( param4 != NULL ) {
				piloto_conf.pltSlots[slot].presion = atof(param4);
			}
			return(true);
		}
	}

	return(false);
}
//------------------------------------------------------------------------------------
bool piloto_leer_slot_actual( uint8_t *slot_id )
{
	// Determina el id del slot en que estoy.
	// Los slots deben tener presion > 0.1


RtcTimeType_t rtcDateTime;
uint16_t now;

int8_t last_slot;
int8_t slot;
uint16_t slot_hhmm;

	// Vemos si hay slots configuradas.
	// Solo chequeo el primero. DEBEN ESTAR ORDENADOS !!
	if ( piloto_conf.pltSlots[0].presion < 0.1 ) {
		// No tengo slots configurados. !!
		return(false);
	}

	//xprintf_P(PSTR("DEBUG: Hay slots configurados\r\n"));

	// Determino la hhmm actuales
	memset( &rtcDateTime, '\0', sizeof(RtcTimeType_t));
	if ( ! RTC_read_dtime(&rtcDateTime) ) {
		xprintf_P(PSTR("PILOTO ERROR: I2C:RTC:pv_get_hhhmm_now\r\n\0"));
		return(false);
	}
	now = rtcDateTime.hour * 100 + rtcDateTime.min;

	// Hay slots configurados:
	// El ultimo slot configurado es anterior al primero en tener p=0
	last_slot = MAX_PILOTO_PSLOTS - 1;
	for ( slot = 1; slot < MAX_PILOTO_PSLOTS; slot++ ) {
		if ( piloto_conf.pltSlots[slot].presion < 0.1 ) {
			last_slot = slot - 1;
			break;
		}
	}

	//xprintf_P(PSTR("DEBUG: Last slot=%d\r\n"), last_slot);
	//xprintf_P(PSTR("DEBUG: Now=%d\r\n"), now );

	// Buscamos dentro de que slot se encuentra now.
	*slot_id = last_slot;
	for ( slot = 0; slot <= last_slot; slot++ ) {
		slot_hhmm = piloto_conf.pltSlots[slot].hour * 100 + piloto_conf.pltSlots[slot].min;
		//xprintf_P(PSTR("DEBUG: slot=%d, hhmm_slot=%d\r\n"), slot, slot_hhmm );

		// Chequeo inside
		if ( now < slot_hhmm ) {
			// Si estoy al ppio.
			if ( slot == 0 ) {
				//xprintf_P(PSTR("DEBUG: 1\r\n") );
				*slot_id = last_slot;
			// Estoy en un slot comun
			} else {
				//xprintf_P(PSTR("DEBUG: 2\r\n") );
				*slot_id = slot - 1;
			}
			break;
		}
	}

	//xprintf_P(PSTR("DEBUG: inside_slot=%d\r\n"), *slot_id );

	return(true);

}
//------------------------------------------------------------------------------------
void piloto_ajustar_presion( void )
{

uint8_t loops;

	xprintf_P(PSTR("PILOTO: Ajuste de presion a (%.03f)\r\n"), PLTCB.pRef );

	if ( ! pv_piloto_determinar_canales_presion() ) {
		xprintf_P(PSTR("PILOTO: ERROR: No se puede determinar el canal de pA/pB.!!\r\n"));
		return;
	}

	// Realizo hasta MAX_INTENTOS de ajustar la presion.
	for ( loops = 0; loops < MAX_INTENTOS; loops++ ) {

		xprintf_P(PSTR("PILOTO: Ajuste #%d\r\n"), loops );

		// Lee la presion actual
		pv_piloto_leer_presiones( 5, INTERVALO_PB_SECS );

		// Controles previos a intentar ajustar la presion
		// Las presiones deben ser positivas.
		if ( PLTCB.pA < MIN_PA_PB ) {
			xprintf_P(PSTR("PILOTO: Ajuste ERROR: pA < %.02f.!!\r\n"), MIN_PA_PB );
			return;
		}

		if ( PLTCB.pB < MIN_PA_PB ) {
			xprintf_P(PSTR("PILOTO: Ajuste ERROR: pB < %.02f.!!\r\n"), MIN_PA_PB);
			return;
		}

		// Debe haber una diferencia de DELTA_PA_PB (300 gr) minima para ajustar
		if ( ( PLTCB.pA - PLTCB.pB ) < DELTA_PA_PB ) {
			xprintf_P(PSTR("PILOTO: Ajuste ERROR: (pA-pB) < %.02f gr.!!\r\n"), DELTA_PA_PB );
			return;
		}

		// La presion de referencia debe ser menor a pA
		if ( ( PLTCB.pA - PLTCB.pRef ) < DELTA_PA_PREF ) {
			xprintf_P(PSTR("PILOTO: Ajuste ERROR: (pA-pRef) < %.02f gr.!!\r\n"), DELTA_PA_PREF );
			return;
		}

		// Calculo la direccion y los pulsos a aplicar
		pv_piloto_calcular_parametros_ajuste();

		// Muestro el resumen de datos
		pv_piloto_print_parametros();

		// Si la diferencia de presiones es superior al error tolerable muevo el piloto
		if ( fabs(PLTCB.pB - PLTCB.pRef) > PLTCB.pError ) {
			// Muevo el piloto.
			// Arranca el timer que por callbacks va a generar los pulsos
			xprintf_P(PSTR("PILOTO: start\r\n"));
			stepper_awake();
			PLTCB.motor_running = true;
			xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
			xTimerStart( plt_pulse_xTimer, 10 );

			// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
			while ( PLTCB.motor_running ) {
				vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
				u_wdg_kick(&piloto_wdg, 120);
			}

		} else {
			xprintf_P(PSTR("PILOTO: Presion alcanzada\r\n"));
			xprintf_P(PSTR("PILOTO: Fin de ajuste\r\n"));
			return;
		}

		//Espero que se estabilize la presión 30 segundos antes de repetir.
		vTaskDelay( ( TickType_t)( INTERVALO_TRYES_SECS * 1000 / portTICK_RATE_MS ) );

	}

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
bool pv_piloto_determinar_canales_presion(void)
{
	// Busca en la configuracion de los canales aquel que se llama pB

uint8_t i;
bool sRet = false;
char l_data[10] = { '\0','\0','\0','\0','\0','\0','\0','\0','\0','\0' };

	PLTCB.pA_channel = -1;
	PLTCB.pB_channel = -1;

	for ( i = 0; i < ANALOG_CHANNELS; i++) {
		memcpy(l_data, ainputs_conf.name[i], sizeof(l_data));
		strupr(l_data);
		if ( ! strcmp_P( l_data, PSTR("PA") ) ) {
			PLTCB.pA_channel = i;
			xprintf_P(PSTR("PILOTO: pAchannel=%d\r\n"), PLTCB.pA_channel);
		}
		if ( ! strcmp_P( l_data, PSTR("PB") ) ) {
			PLTCB.pB_channel = i;
			xprintf_P(PSTR("PILOTO: pBchannel=%d\r\n"), PLTCB.pB_channel);
		}
	};

	if ( (PLTCB.pA_channel != -1) && (PLTCB.pB_channel != -1) )
		sRet = true;

	return(sRet);

}
//------------------------------------------------------------------------------------
void pv_piloto_leer_presiones( int8_t samples, uint16_t intervalo_secs )
{
	// Medir la presión de baja N veces a intervalos de 10 s y promediar
	// Deja el valor EN GRAMOS en spiloto.pB

uint8_t i;
float presion[ANALOG_CHANNELS];

	// Mido pA/pB
	PLTCB.pA = 0;
	PLTCB.pB = 0;
	for ( i = 0; i < samples; i++) {
		u_wdg_kick(&piloto_wdg, 120);
		ainputs_read( &presion[0], NULL );
		xprintf_P(PSTR("PILOTO pA:[%d]->%0.3f, pB:[%d]->%0.3f\r\n"), i, presion[PLTCB.pA_channel], i, presion[PLTCB.pB_channel] );
		// La presion la expreso en gramos !!!
		PLTCB.pA += presion[PLTCB.pA_channel];
		PLTCB.pB += presion[PLTCB.pB_channel];
		vTaskDelay( ( TickType_t)( intervalo_secs * 1000 / portTICK_RATE_MS ) );
	}

	PLTCB.pA /= samples;
	PLTCB.pB /= samples;
	xprintf_P(PSTR("PILOTO pA=%.02f, pB=%.02f\r\n"), PLTCB.pA, PLTCB.pB );
}
//------------------------------------------------------------------------------------
void pv_piloto_pulse_TimerCallback( TimerHandle_t xTimer )
{
	// Genera un pulso.
	// Cuando la cuenta de pulsos llega a 0, se desactiva.

	stepper_pulse( PLTCB.dir, PLTCB.pwidth );

	if ( (PLTCB.pulse_counts % 100) == 0 ) {
		xprintf_P(PSTR("."));
	}

	if ( PLTCB.pulse_counts-- == 0 ) {
		xprintf_P(PSTR("\r\n"));
		xprintf_P(PSTR("PILOTO: stop\r\n"));
		PLTCB.motor_running = false;
		// Detengo el timer.
		xTimerStop( plt_pulse_xTimer, 10 );
		stepper_sleep();
	}

}
//------------------------------------------------------------------------------------
void pv_piloto_calcular_parametros_ajuste( void )
{

float delta_pres = 0.0;

	// Paso 1: La presion a la que debo poner el piloto es la que me da el slot actual
	//PLTCB.pRef = pRef;

	// Paso 2: pERROR
	PLTCB.pError = PERROR;

	// Paso 3: Calculo el sentido del giro
	if ( PLTCB.pB < PLTCB.pRef ) {
		// Debo aumentar pB o sxprintf_P(PSTR("PILOTO: npulses=%d\r\n"), spiloto.npulses);ea apretar el tornillo (FWD)
		PLTCB.dir = STEPPER_FWD; // Giro forward, aprieto el tornillo, aumento la presion de salida
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	// Paso 4: Intervalo de tiempo entre pulsos en ms.
	PLTCB.pwidth = 10;

	/* Paso 5: Calculo los pulsos a aplicar.
	 * El motor es de 200 pasos /rev
	 * El servo reduce 15:1
	 * Esto hace que para girar el vastago 1 rev necesite 3000 pulsos
	 * El piloto es de 4->1500gr.
	 */
	delta_pres = fabs(PLTCB.pB - PLTCB.pRef);
	PLTCB.pulsos_calculados = (uint16_t) ( delta_pres * PULSOS_X_REV  / DPRES_X_REV );

	//xprintf_P(PSTR("DEBUG: pulsos_calculados: %d\r\n"), PLTCB.pulsos_calculados );

	if ( PLTCB.pulsos_calculados < 0) {
		xprintf_P(PSTR("PILOTO: ERROR pulsos_calculados < 0\r\n"));
		PLTCB.pulsos_calculados = 0;
	}

	// Paso 6: Ajusto los pasos al 70%
	/*
	 * METODO 1: Cuando estoy en reverse( bajando la presion ) aplico los pulsos calculados
	 *           Si estoy en forward (subiendo la presion), aplico solo el 70% de los pulsos
	 *           calculados si estos son altos.
	 *           Si son menos de 500 no lo corrijo.
	 */
	if ( ( PLTCB.dir == STEPPER_FWD) && ( PLTCB.pulsos_calculados > 500 ) ) {
		PLTCB.pulsos_a_aplicar = (uint16_t) (0.7 * PLTCB.pulsos_calculados);
	} else {
		PLTCB.pulsos_a_aplicar = PLTCB.pulsos_calculados;
	}

	//xprintf_P(PSTR("DEBUG: pulsos_a_aplicar_1: %d\r\n"), PLTCB.pulsos_a_aplicar );
	//AJUSTE_BASICO:
	// METODO 2: Los pulsos son los que me da el calculo.

	// Paso 7: Controlo no avanzar mas de 500gr aprox x loop !!!
	if ( PLTCB.pulsos_a_aplicar > 3000 ) {
		PLTCB.pulsos_a_aplicar = 3000;
	}

	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;
	//xprintf_P(PSTR("DEBUG: pulsoe_counts: %d\r\n"), PLTCB.pulse_counts );
}
//------------------------------------------------------------------------------------
void pv_piloto_print_parametros(void)
{
	// Muestro en pantalla los parametros calculados de ajuste de presion.

	xprintf_P(PSTR("-----------------------------\r\n"));
	xprintf_P(PSTR("PILOTO: pA=%.03f\r\n"), PLTCB.pA );
	xprintf_P(PSTR("PILOTO: pB=%.03f\r\n"), PLTCB.pB );
	xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"),   PLTCB.pRef );
	xprintf_P(PSTR("PILOTO: deltaP=%.03f\r\n"), ( PLTCB.pB - PLTCB.pRef));
	xprintf_P(PSTR("PILOTO: pulses calc=%d\r\n"), PLTCB.pulsos_calculados );
	xprintf_P(PSTR("PILOTO: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
	}
	xprintf_P(PSTR("-----------------------------\r\n"));
}
//------------------------------------------------------------------------------------

