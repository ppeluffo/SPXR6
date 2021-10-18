/*
 * ul_pilotos.c
 *
 *  Created on: 13 may. 2021
 *      Author: pablo
 *
 *  Armo una FSM que maneja el movimiento del servo.
 *  Agrego en el PLTCB un control de los pulsos que llevo aplicados por si tengo que hacer
 *  un rollback.
 *  En un estado chequeo las condiciones para moverlo.
 *  - Si detecte un error puedo llegar a hacer un rollback para poner al piloto en su poscion original.
 *  Control dinámico:
 *  - Agrego las variables dync_pB y dync_pulses.
 *  - Si luego de haber girado al menos 1 vuelta completa, no movio pB en al menos 200gr, hago un rollback
 *
 *  TESTING:
 *  1- Caso normal OK.
 *  2- Subir pB mas alto de pA. Debe dejarlo en pA-pband. OK
 *  3- Bajar.OK. ( Cambio para que en este caso tambien aplicar el 70% de los pulsos )
 *
 *  Considerar leer el slot c/ajuste por si cambio en medio de un cambio de slot
 *  o al final para reajustar
 */

#include "ul_pilotos.h"
#include "tkApp.h"

TimerHandle_t plt_pulse_xTimer;
StaticTimer_t plt_pulse_xTimerBuffer;

void pv_piloto_pulse_TimerCallback( TimerHandle_t xTimer );
bool pv_piloto_leer_slot_actual( uint8_t *slot_id );
void FSM_piloto_ajustar_presion( uint8_t app_wdt );
bool pv_piloto_check_inputs_conf(void);
void pv_piloto_read_inputs( int8_t samples, uint16_t intervalo_secs );
bool pv_piloto_conditions4adjust( void );
void pv_piloto_ajustar(bool rollback, uint8_t app_wdt );
void pv_piloto_calcular_parametros_ajuste( void );
void pv_piloto_print_parametros(bool rollback);
void pv_piloto_process_output(void);

void pv_dyn_control_init(void);
void pv_dyn_control_update(void);
bool pv_dyn_control_check(void);

bool pv_ctl_max_reintentos(void);
bool pv_ctl_pA_positiva(void);
bool pv_ctl_pB_positiva(void);
bool pv_ctl_pA_mayor_pB(void);
bool pv_ctl_pB_alcanzada(void);
bool pv_ctl_band_gap_limit(void);

//------------------------------------------------------------------------------------
// CONFIGURACION
//------------------------------------------------------------------------------------
void piloto_config_defaults(void)
{

int8_t slot;

	for (slot=0; slot < MAX_PILOTO_PSLOTS; slot++) {
		piloto_conf.pltSlots[slot].pTime.hour = 0;
		piloto_conf.pltSlots[slot].pTime.min = 0;
		piloto_conf.pltSlots[slot].presion = 0.0;
	}
	piloto_conf.pulsesXrev = 3000;
	piloto_conf.pWidth = 20;
}
//------------------------------------------------------------------------------------
void piloto_config_status(void)
{

int8_t slot;

	xprintf_P( PSTR("  modo: PILOTO\r\n"));
	xprintf_P( PSTR("    PulsosXrev=%d, pWidth=%d(ms)\r\n"), piloto_conf.pulsesXrev, piloto_conf.pWidth  );
	xprintf_P( PSTR("    Slots:\r\n"));
	xprintf_P( PSTR("    "));
	for (slot=0; slot < (MAX_PILOTO_PSLOTS / 2);slot++) {
		xprintf_P( PSTR("[%02d]%02d:%02d->%0.2f "), slot, piloto_conf.pltSlots[slot].pTime.hour, piloto_conf.pltSlots[slot].pTime.min,piloto_conf.pltSlots[slot].presion  );
	}
	xprintf_P( PSTR("\r\n"));

	xprintf_P( PSTR("    "));
	for (slot=(MAX_PILOTO_PSLOTS / 2); slot < MAX_PILOTO_PSLOTS;slot++) {
		xprintf_P( PSTR("[%02d]%02d:%02d->%0.2f "), slot, piloto_conf.pltSlots[slot].pTime.hour, piloto_conf.pltSlots[slot].pTime.min,piloto_conf.pltSlots[slot].presion  );
	}
	xprintf_P( PSTR("\r\n"));

}
//------------------------------------------------------------------------------------
void piloto_setup_outofrtos(void )
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
bool piloto_config_slot( char *s_slot, char *s_hhmm, char *s_presion )
{
	// Configura un slot

uint8_t slot;

	// Intervalos tiempo:presion:

	slot = atoi(s_slot);

	//xprintf_P(PSTR("DEBUG: slot=%d\r\n"), slot);

	if ( slot < MAX_PILOTO_PSLOTS ) {
		if ( s_hhmm != NULL ) {
			u_convert_int_to_time_t( atoi(s_hhmm), &piloto_conf.pltSlots[slot].pTime.hour, &piloto_conf.pltSlots[slot].pTime.min  );
		} else {
			return(false);
		}

		//xprintf_P(PSTR("DEBUG: %d:%d\r\n"), piloto_conf.pltSlots[slot].pTime.hour, piloto_conf.pltSlots[slot].pTime.min );

		if ( s_presion != NULL ) {
			piloto_conf.pltSlots[slot].presion = atof(s_presion);
		} else {
			return(false);
		}

		//xprintf_P(PSTR("DEBUG: %.02f\r\n"), piloto_conf.pltSlots[slot].presion );
		return(true);
	}

	return(false);
}
//------------------------------------------------------------------------------------
void piloto_config_ppr( char *s_pulseXrev )
{
	piloto_conf.pulsesXrev = atoi(s_pulseXrev);
}
//------------------------------------------------------------------------------------
void piloto_config_pwidth( char *s_pwidth )
{
	piloto_conf.pWidth = atoi(s_pwidth);
}
//------------------------------------------------------------------------------------
// SERVICE
//------------------------------------------------------------------------------------
bool piloto_init_service(void)
{

	PLTCB.start_test = false;
	return(true);

}
//------------------------------------------------------------------------------------
void piloto_app_service( uint8_t app_wdt )
{

uint8_t slot_actual = -1;
uint8_t slot;

	xprintf_P(PSTR("PILOTO\r\n"));

	if ( !piloto_init_service() )
		return;

	for( ;; )
	{
		u_wdg_kick( app_wdt,  240 );
		vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );

		// Callback para arrancar el test
		if ( PLTCB.start_test) {
			PLTCB.start_test = false;
			FSM_piloto_ajustar_presion( app_wdt );
			continue;
		}

		if ( pv_piloto_leer_slot_actual( &slot ) ) {
			if ( slot_actual != slot ) {
				slot_actual = slot;
				xprintf_P(PSTR("PILOTO: Inicio de ciclo.\r\n"));
				xprintf_P(PSTR("PILOTO: slot=%d, pRef=%.03f\r\n"), slot_actual, piloto_conf.pltSlots[slot_actual].presion);
				PLTCB.pRef = piloto_conf.pltSlots[slot_actual].presion;
				FSM_piloto_ajustar_presion( app_wdt );
			}
		}
	}
}
//------------------------------------------------------------------------------------
uint8_t piloto_hash(void)
{
	// PLT;PXR:5000;PWIDTH:20;SLOT0:0630,1.20;SLOT1:0745,2.40;SLOT2:1230,3.60;SLOT3:2245,4.80;SLOT4:2345,5.00;

uint8_t hash = 0;
char *p;
uint8_t slot = 0;
uint8_t i = 0;
uint8_t j = 0;
int16_t free_size = sizeof(hash_buffer);

	// Vacio el buffer temporal
	memset(hash_buffer,'\0', sizeof(hash_buffer));

	i += snprintf_P( &hash_buffer[i], free_size, PSTR("PLT;PXR:%d;PWIDTH:%d;"), piloto_conf.pulsesXrev, piloto_conf.pWidth);
	//xprintf_P(PSTR("HASH: [%s]\r\n"), hash_buffer);

	free_size = (  sizeof(hash_buffer) - i );
	if ( free_size < 0 ) goto exit_error;
	p = hash_buffer;
	while (*p != '\0') {
		hash = u_hash(hash, *p++);
	}

	// SLOTS
	for ( slot = 0; slot < MAX_PILOTO_PSLOTS; slot++ ) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		free_size = sizeof(hash_buffer);
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		j = snprintf_P( hash_buffer, free_size, PSTR("SLOT%d:%02d%02d,%0.2f;"),
				slot,
				piloto_conf.pltSlots[slot].pTime.hour,
				piloto_conf.pltSlots[slot].pTime.min,
				piloto_conf.pltSlots[slot].presion );
		//xprintf_P(PSTR("HASH: [%s]\r\n"), hash_buffer);
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}
	}

	return(hash);

exit_error:

	xprintf_P( PSTR("PILOTOS: Hash ERROR !!!\r\n\0"));
	return(0x00);

}
//------------------------------------------------------------------------------------
void piloto_set_presion_momentanea( float presion)
{
	// Funcion invocada online para fijar una nueva presion hasta que finalize
	// el slot

	xprintf_P(PSTR("PILOTO: Set presion from Server.\r\n"));
	xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"), presion);
	PLTCB.start_test = true;
	PLTCB.pRef = presion;

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
		stepper_pwr_off();
	}

}
//------------------------------------------------------------------------------------
// TESTS
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

	// Activo el driver
	xprintf_P(PSTR("STEPPER driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("STEPPER driver pulses start..\r\n"));
	stepper_awake();
	stepper_start();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

}
//-------------------------------------u_wdg_kick-----------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
bool pv_piloto_leer_slot_actual( uint8_t *slot_id )
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
		slot_hhmm = piloto_conf.pltSlots[slot].pTime.hour * 100 + piloto_conf.pltSlots[slot].pTime.min;
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
void FSM_piloto_ajustar_presion( uint8_t app_wdt )
{

uint8_t state = PLT_ENTRY;

	for (;;) {

		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );

		switch( state ) {

		case PLT_ENTRY:
			// Inicializo
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state ENTRY\r\n"));
			xprintf_P(PSTR("PILOTO: Ajuste de presion a (%.03f)\r\n"), PLTCB.pRef );
			PLTCB.loops = 0;
			PLTCB.exit_code = UNKNOWN;
			PLTCB.pulsos_rollback = 0.0;
			state = PLT_CHECK_INPUTS;
			break;

		case PLT_CHECK_INPUTS:
			// Veo si tengo entradas configuradas
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state CHECK_INPUTS\r\n"));
			if ( pv_piloto_check_inputs_conf()) {
				state = PLT_READ_INPUTS;
			} else {
				state = PLT_EXIT;
			}
			break;

		case PLT_READ_INPUTS:
			// Leo las entradas
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state READ_INPUTS\r\n"));
			pv_piloto_read_inputs(5, INTERVALO_PB_SECS );
			state = PLT_CHECK_CONDITIONS;
			break;

		case PLT_CHECK_CONDITIONS:
			// Veo si tengo condiciones para ajustar
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state CHECK_CODITIONS\r\n"));
			if ( pv_piloto_conditions4adjust() ) {
				state = PLT_AJUSTE;
			} else {
				state = PLT_OUTPUT_STATUS;
			}
			break;

		case PLT_AJUSTE:
			// Ajusto
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state AJUSTE\r\n"));
			pv_piloto_ajustar(false, app_wdt );
			state = PLT_READ_INPUTS;
			break;

		case PLT_OUTPUT_STATUS:
			// Evaluo condiciones de salida
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state OUTPUT\r\n"));
			pv_piloto_process_output();
			state = PLT_EXIT;
			break;

		case PLT_EXIT:
			// Termino y salgo.
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: state EXIT\r\n"));
			xprintf_P(PSTR("PILOTO: Fin de Ajuste\r\n"));
			return;
			break;
		}
	}
}
//------------------------------------------------------------------------------------
bool pv_piloto_check_inputs_conf(void)
{
	/*
	 *  Se fija si en la configuracion del eqipo hay algun canal con el
	 *  nombre pA y pB.
	 *  Si no los hay entonces retorna FALSE
	 */


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
void pv_piloto_read_inputs( int8_t samples, uint16_t intervalo_secs )
{
	/*
	 * Lee las entradas: pA,pB.
	 * Medimos N veces y promediamos
	 * Dejamos el valor en gramos
	 *
	 */

uint8_t i;
float presion[ANALOG_CHANNELS];

	// Mido pA/pB
	PLTCB.pA = 0;
	PLTCB.pB = 0;
	for ( i = 0; i < samples; i++) {
		ainputs_read( &presion[0], NULL, false );
		xprintf_P(PSTR("PILOTO pA:[%d]->%0.3f, pB:[%d]->%0.3f\r\n"), i, presion[PLTCB.pA_channel], i, presion[PLTCB.pB_channel] );
		// La presion la expreso en gramos !!!
		PLTCB.pA += presion[PLTCB.pA_channel];
		PLTCB.pB += presion[PLTCB.pB_channel];
		vTaskDelay( ( TickType_t)( intervalo_secs * 1000 / portTICK_RATE_MS ) );
	}

	PLTCB.pA /= samples;
	PLTCB.pB /= samples;
	xprintf_P(PSTR("PILOTO pA=%.02f, pB=%.02f\r\n"), PLTCB.pA, PLTCB.pB );

	// DYNAMIC CONTROL: Si estoy al inicio del ciclo, inicializo el control dinamico
	if ( PLTCB.loops == 0 ) {
		pv_dyn_control_init();
	}
}
//------------------------------------------------------------------------------------
bool pv_piloto_conditions4adjust( void )
{
	/*
	 * Chequeo todas las condiciones necesarias para activar el servo o salir
	 */

bool retS = false;

	if ( pv_ctl_max_reintentos()) {
		PLTCB.exit_code = MAX_TRYES;
		goto quit;
	}

	if ( ! pv_ctl_pA_positiva() ) {
		PLTCB.exit_code = PA_ERR;
		goto quit;
	}

	if ( ! pv_ctl_pB_positiva() ) {
		PLTCB.exit_code = PB_ERR;
		goto quit;
	}

	if ( ! pv_ctl_pA_mayor_pB() ) {
		PLTCB.exit_code = PA_LESS_PB;
		goto quit;
	}

	if ( pv_ctl_band_gap_limit() ) {
		PLTCB.exit_code = BAND_ERR;
		goto quit;
	}

	if ( ! pv_dyn_control_check() ) {
		PLTCB.exit_code = ADJUST_ERR;
		goto quit;
	}

	// Ultima condicion a evaluar.

	if ( pv_ctl_pB_alcanzada() ) {
		PLTCB.exit_code = POUT_REACHED;
		goto quit;
	}

	// Condiciones para ajustar
	retS = true;

quit:

	return(retS);
}
//------------------------------------------------------------------------------------
void pv_piloto_process_output(void)
{
	/*
	 * Procesa la salida de la FSM.
	 * Puede salir directamente o hacer un rollback de los pulsos generados
	 * para no tener al piloto con riesgo de salirse del tornillo
	 */

	if ( PLTCB.exit_code == MAX_TRYES ) {
		// En varios intentos no pude llegar pero ajuste lo mas posible.
		return;

	} else if ( PLTCB.exit_code == POUT_REACHED ) {
		// Alcance la presion target.
		return;

	} else {
		// En todos los otros casos debo hacer un rollback
		//pv_piloto_ajustar(true);
		xprintf_P(PSTR("PILOTOS: ROLLBACK\r\n"));
	}

}
//------------------------------------------------------------------------------------
void pv_piloto_ajustar( bool rollback, uint8_t app_wdt )
{
	/*
	 * Calculo los parametros para el ajuste, los muestro en pantalla
	 * y arranco el motor
	 */

	if ( rollback ) {
		/*
		 *  Aplico los pulsos que aplique en el ciclo pero en sentido contrario
		 *  para volver al servo a la posicion inicial
		 */
		xprintf_P(PSTR("PILOTO: ROLLBACK: pulses_rollback=%.01f\r\n"), PLTCB.pulsos_rollback );

		PLTCB.pulse_counts = (uint16_t) fabs(PLTCB.pulsos_rollback);
		// Controlo errores ( horrores ) de calculo
		if ( PLTCB.pulse_counts > 15000 ) {
			xprintf_P(PSTR("PILOTO: ROLLBACK ERROR\r\n"));
			return;
		}
		// Direccion
		if ( PLTCB.pulsos_rollback < 0 ) {
			PLTCB.dir = STEPPER_FWD;
		} else {
			PLTCB.dir = STEPPER_REV;
		}

	} else {
		// Ajuste Normal
		// Calculo la direccion y los pulsos a aplicar
		pv_piloto_calcular_parametros_ajuste();
	}

	// Muestro el resumen de datos
	pv_piloto_print_parametros( rollback );

	u_wdg_kick( app_wdt,  240 );

	// Muevo el piloto.
	// Arranca el timer que por callbacks va a generar los pulsos
	xprintf_P(PSTR("PILOTO: start\r\n"));
	// Activo el driver
	xprintf_P(PSTR("STEPPER driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	stepper_awake();
	stepper_start();
	PLTCB.motor_running = true;
	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

	// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
	while ( PLTCB.motor_running ) {
		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
	}

	xprintf_P(PSTR("PILOTO: end\r\n"));
}
//------------------------------------------------------------------------------------
void pv_piloto_calcular_parametros_ajuste( void )
{

	/*
	 * Calcula en base a los parametros de entrada y la salida esperada, los
	 * parametros necesarios para mover el motor
	 *
	 */

float delta_pres = 0.0;

	// A) pERROR
	PLTCB.pError = PERROR;

	// B) Calculo el sentido del giro
	if ( PLTCB.pB < PLTCB.pRef ) {
		// Debo aumentar pB o sxprintf_P(PSTR("PILOTO: npulses=%d\r\n"), spiloto.npulses);ea apretar el tornillo (FWD)
		PLTCB.dir = STEPPER_FWD; // Giro forward, aprieto el tornillo, aumento la presion de salida
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	// C) Intervalo de tiempo entre pulsos en ms.
	//PLTCB.pwidth = 20;
	PLTCB.pwidth = piloto_conf.pWidth;

	/*
	 * D) Calculo los pulsos a aplicar.
	 * El motor es de 200 pasos /rev
	 * El servo reduce 15:1
	 * Esto hace que para girar el vastago 1 rev necesite 3000 pulsos
	 * El piloto es de 4->1500gr.
	 */
	delta_pres = fabs(PLTCB.pB - PLTCB.pRef);
	PLTCB.pulsos_calculados = ( delta_pres * piloto_conf.pulsesXrev  / DPRES_X_REV );

	//xprintf_P(PSTR("DEBUG: pulsos_calculados: %d\r\n"), PLTCB.pulsos_calculados );

	if ( PLTCB.pulsos_calculados < 0) {
		xprintf_P(PSTR("PILOTO: ERROR pulsos_calculados < 0\r\n"));
		PLTCB.pulsos_calculados = 0;
	}

	// E) Ajusto los pasos al 70%
	/*
	 * METODO 1: Cuando estoy en reverse( bajando la presion ) aplico los pulsos calculados
	 *           Si estoy en forward (subiendo la presion), aplico solo el 70% de los pulsos
	 *           calculados si estos son altos.
	 *           Si son menos de 500 no lo corrijo.
	 */
	//if ( ( PLTCB.dir == STEPPER_FWD) && ( PLTCB.pulsos_calculados > 500 ) ) {
	if ( PLTCB.pulsos_calculados > 500 ) {
		PLTCB.pulsos_a_aplicar = (0.7 * PLTCB.pulsos_calculados);
	} else {
		PLTCB.pulsos_a_aplicar = PLTCB.pulsos_calculados;
	}

	//xprintf_P(PSTR("DEBUG: pulsos_a_aplicar_1: %d\r\n"), PLTCB.pulsos_a_aplicar );
	//AJUSTE_BASICO:
	// METODO 2: Los pulsos son los que me da el calculo.

	// E) Controlo no avanzar mas de 500gr aprox x loop !!! ( 1 revolucion )
	if ( PLTCB.pulsos_a_aplicar > piloto_conf.pulsesXrev ) {
		PLTCB.pulsos_a_aplicar = piloto_conf.pulsesXrev;
	}

	PLTCB.pulse_counts = (uint16_t) PLTCB.pulsos_a_aplicar;

	// Los pulsos de rollback tienen sentido contrario.
	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.pulsos_rollback -= PLTCB.pulse_counts;
	} else {
		PLTCB.pulsos_rollback += PLTCB.pulse_counts;
	}

	//xprintf_P(PSTR("DEBUG: pulsoe_counts: %d\r\n"), PLTCB.pulse_counts );
}
//------------------------------------------------------------------------------------
void pv_piloto_print_parametros( bool rollback )
{
	// Muestro en pantalla los parametros calculados de ajuste de presion.

	xprintf_P(PSTR("-----------------------------\r\n"));

	if ( !rollback ) {
		xprintf_P(PSTR("PILOTO: loops=%d\r\n"), PLTCB.loops );
		xprintf_P(PSTR("PILOTO: pA=%.03f\r\n"), PLTCB.pA );
		xprintf_P(PSTR("PILOTO: pB=%.03f\r\n"), PLTCB.pB );
		xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"),   PLTCB.pRef );
		xprintf_P(PSTR("PILOTO: deltaP=%.03f\r\n"), ( PLTCB.pB - PLTCB.pRef));
		xprintf_P(PSTR("PILOTO: pulses calc=%.01f\r\n"), PLTCB.pulsos_calculados );
		xprintf_P(PSTR("PILOTO: pulses apply=%.01f\r\n"), PLTCB.pulsos_a_aplicar );
	}

	xprintf_P(PSTR("PILOTO: pulses rollback=%.01f\r\n"), PLTCB.pulsos_rollback );
	xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
	}
	xprintf_P(PSTR("-----------------------------\r\n"));
}
//------------------------------------------------------------------------------------
// Controles generales
//------------------------------------------------------------------------------------
bool pv_ctl_max_reintentos(void)
{
	// Controla si alcanzamos la maxima cantidad de reintentos de ajustes de presion.

bool retS = false;

	if ( ++PLTCB.loops >=  MAX_INTENTOS ) {
		xprintf_P(PSTR("PILOTO: Maxima cantidad de intentos alcanzada. Exit.\r\n"));
		retS = true;

	} else {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check intentos OK.\r\n"));
		retS = false;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
bool pv_ctl_pA_positiva(void)
{

	// Controlo que la pA sea positiva ( mayor que un minimo de 0.5k)

bool retS = false;

	if ( PLTCB.pA > MIN_PA_PB ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check pA>0 OK\r\n") );
		retS = true;
	} else {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: pA < %.02f.!!\r\n"), MIN_PA_PB );
		retS = false;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_pB_positiva(void)
{

	// Controlo que la pB sea positiva ( mayor que un minimo de 0.5k)

bool retS = false;

	if ( PLTCB.pB > MIN_PA_PB ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check pB>0 OK\r\n") );
		retS = true;
	} else {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: pB < %.02f.!!\r\n"), MIN_PA_PB );
		retS = false;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_pA_mayor_pB(void)
{
	// pA debe ser mayor que pB
	// Puede ocurrir que baje la alta debajo de la regulacion con lo que pB se pega
	// a la alta y ambas quedan iguales.

bool retS = false;

	if ( PLTCB.pA < PLTCB.pB  ) {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: ( pA < pB) .!!\r\n") );
		retS = false;
	} else {
		retS = true;
		xprintf_P(PSTR("PILOTO: Check (pA>pB) OK\r\n") );
	}
	return(retS);
}
//------------------------------------------------------------------------------------
bool pv_ctl_pB_alcanzada(void)
{

	// Presion alcanzada: Ultima condicion a evaluar

bool retS = false;

	if ( fabs(PLTCB.pB - PLTCB.pRef) < PLTCB.pError ) {
		xprintf_P(PSTR("PILOTO: Presion alcanzada\r\n"));
		retS = true;
	} else {
		retS = false;
	}
	return(retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_band_gap_limit(void)
{
	/*
	 * Cuando ajusto subiendo pB, debe haber una banda entre el pB y el pA para
	 * que la reguladora trabaje.
	 * Tambien puede que la pRef sea mayor a lo posible pero no por esto no subirla
	 * sino subirla todo lo posible.
	 * Cuando debo bajar pB, no importa la banda sino solo controlar no llegar
	 * al minimo y cerrar la valvula.
	 * La banda entre pA y pB debe ser mayor a 300gr para que  trabaje la reguladora
	 */

bool retS = false;

	// Ajuste a la baja. La nueva presion es menor que la pB actual.
	if ( PLTCB.pRef < PLTCB.pB ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check bandgap OK\r\n") );
		retS = false;
		goto quit;
	}

	// Ajuste a la suba. Debo subir pB.
	// 1. Veo si hay margen de regulacion
	if ( ( PLTCB.pA - PLTCB.pB ) < DELTA_PA_PB ) {
		// No hay margen
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: (pA-pB) < %.02f gr.!!\r\n"), DELTA_PA_PB );
		retS = true;
		goto quit;
	}

	// 2. Hay margen de regulacion pero la nueva presion a alcanzar esta el limite.
	if ( ( PLTCB.pA - PLTCB.pRef ) < DELTA_PA_PREF ) {
		// La nueva pA ( PLTCB.pRef ) debe dejar una banda de ajuste con pA.
		PLTCB.pRef = PLTCB.pA - DELTA_PA_PREF;
		xprintf_P(PSTR("PILOTO: Recalculo (pA-pRef) < %.02f gr.!!\r\n"), DELTA_PA_PREF );
		xprintf_P(PSTR("        Nueva pRef=%.02f\r\n"), PLTCB.pRef );
		retS = false;
		goto quit;
	}

quit:

	return(retS);

}
//------------------------------------------------------------------------------------
// Control dinamico
//------------------------------------------------------------------------------------
void pv_dyn_control_init(void)
{
	xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control_init\r\n"));
	PLTCB.dync_pB0 = 0;
	PLTCB.dync_pulsos = 0;
}

//------------------------------------------------------------------------------------
void pv_dyn_control_update(void)
{
	// Luego de mover el piloto, actulizamos los parametros para hacer este control.
	// Llevo el conteo de los pulsos aplicados al servo.

	xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control_update\r\n"));

	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.dync_pulsos += PLTCB.pulse_counts;
	} else {
		PLTCB.dync_pulsos -= PLTCB.pulse_counts;
	}
}
//------------------------------------------------------------------------------------
bool pv_dyn_control_check(void)
{
	/*
	 *  Movi el piloto algo mas de 1 vuelta y la pB no se modifico en al menos 200 gr. ( Rollback )
	 *  El piloto se esta moviendo pero la pB no esta ajustando.
	 *  Si di mas de una revolucion, la presion debe haber cambiado al menos 200grs.
	 *  Si cambio, renicio el control dinamico al nuevo punto
	 */

	xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control_check\r\n"));

	if ( fabs(PLTCB.dync_pulsos) > piloto_conf.pulsesXrev ) {

		xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control 1xrev.\r\n"));

		if ( fabs( PLTCB.pB - PLTCB.dync_pB0 ) < 200 ) {
			// Giro al menos 1 vuelta y no cambio ni 200 grs. ERROR.
			xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control PresERROR.\r\n"));
			return(false);

		} else {
			// Actualizo los parametros del dyn_control.
			xprintf_PD( DF_APP, PSTR("APP:PILOTO dyn_control Update\r\n"));
			PLTCB.dync_pulsos = 0;
			PLTCB.dync_pB0 = PLTCB.pB;
		}
	}

	return(true);
}
//------------------------------------------------------------------------------------

