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
 *  En dyn_PB pongo la presion al incio del ciclo.
 *  En dyn_pulses cuento los pulsos que voy aplicando.
 *  Cuando supere 1 revolucion, veo la presion actual con la del inicio del ciclo y veo si algo cambio o no.
 *
 *  Ajuste Atrasado:
 *  Puede ocurrir que llegue un slot pero no se pueda ajustar la presion porque no estan las condiciones.
 *  Si el siguiente slot ocurre a las horas, el sistema va a quedar mucho tiempo desconfigurado.
 *  El Ajuste Atrasado implica controlar cuando hayan condiciones y reintentarlo lo antes posible.
 *
 *  Modelo:
 *  Usamos un modelo proveedor -> consumidor asincronico.
 *  Los proveedores ingresan sus solicitudes ( presiones ) en un stack.
 *  Los proveedores son:
 *  - Subsistema de control de cambio de slot.
 *  - Funcion de testing
 *  - Online.
 *
 *  El consumidor es un proceso unico.
 *  Consiste en:
 *  - Saca un valor. Chequea si estan las condiciones para ajustar.
 *  - Si estan intenta ajustar, si no pone de nuevo el valor en el stack.
 *  - Si trato de ajustar y no pudo, pone de nuevo el valor en el stack.
 *
 *  De este modo, cada ciclo atiende las condiciones pendientes y nuevas.
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
bool pv_piloto_leer_slot_actual( int8_t *slot_id );
bool FSM_piloto_ajustar_presion( void );

void plt_producer_slot_handler(void);
void plt_consumer_handler(void);

bool pv_piloto_check_inputs_conf(void);
void pv_piloto_read_inputs( int8_t samples, uint16_t intervalo_secs );
bool pv_piloto_adjust_conditions( void );
bool pv_piloto_pre_conditions( void );
bool pv_piloto_pre_conditions_4increase( void );
bool pv_piloto_pre_conditions_4reduce( void );

void pv_piloto_ajustar( void );
void pv_piloto_calcular_parametros_ajuste( void );
void pv_piloto_print_parametros( void );
bool pv_piloto_process_output( void );

bool pv_ctl_max_reintentos_reached(void);
bool pv_ctl_pA_es_positiva( float pA);
bool pv_ctl_pB_es_positiva( float pB);
bool pv_ctl_pA_mayor_pB( float pA, float pB);
bool pv_ctl_pB_alcanzada(void);
bool pv_ctl_band_gap_suficiente(float pA, float pB, float pRef );
bool pv_ctl_caudal( float qMin, float pA, float pB, float pRef );

void pv_rollback_init(void);
void pv_rollback_update(void);
void pv_rollback_ajustar_piloto( void );

void pv_dyncontrol_init(void);
void pv_dyncontrol_update(void);
bool pv_dyncontrol_check(void);

/*
 * -----------------------------------------------------------------------------------
 * FSM de ajustar la presion.
 * -----------------------------------------------------------------------------------
 */
bool FSM_piloto_ajustar_presion( void )
{

uint8_t state = PLT_READ_INPUTS;
bool retS = false;

	// Inicializo
	xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste ENTRY\r\n"));
	xprintf_P(PSTR("PILOTO: Ajuste de presion a (%.03f)\r\n"), PLTCB.pRef );
	PLTCB.loops = 0;
	PLTCB.exit_code = UNKNOWN;
	pv_rollback_init();
	state = PLT_READ_INPUTS;

	for (;;) {

		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );

		switch( state ) {

		case PLT_READ_INPUTS:
			// Leo las entradas
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste: state READ_INPUTS\r\n"));
			// Espero siempre 30s antes para que se estabilizen. Sobre todo en valvulas grandes
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: await 30s\r\n"));
			u_wdg_kick( plt_app_wdg,  240 );
			vTaskDelay( ( TickType_t) (30000 / portTICK_RATE_MS ) );
			//
			pv_piloto_read_inputs(5, INTERVALO_PB_SECS );
			state = PLT_CHECK_CONDITIONS;
			break;

		case PLT_CHECK_CONDITIONS:
			// Veo si tengo condiciones para ajustar
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste: state CHECK_CONDITIONS\r\n"));
			if ( pv_piloto_adjust_conditions() ) {
				state = PLT_AJUSTE;
			} else {
				state = PLT_OUTPUT_STATUS;
			}
			break;

		case PLT_AJUSTE:
			// Ajusto
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste: state AJUSTE\r\n"));
			// Calculo la direccion y los pulsos a aplicar
			pv_piloto_calcular_parametros_ajuste();
			//
			pv_rollback_update();
			//
			pv_dyncontrol_update();
			//
			// Muestro el resumen de datos
			pv_piloto_print_parametros();
			//
			pv_piloto_ajustar();
			state = PLT_READ_INPUTS;
			break;

		case PLT_OUTPUT_STATUS:
			// Evaluo condiciones de salida
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste: state OUTPUT\r\n"));
			retS = pv_piloto_process_output();
			state = PLT_EXIT;
			break;

		case PLT_EXIT:
			// Termino y salgo.
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMajuste: state EXIT\r\n"));
			xprintf_P(PSTR("PILOTO: Fin de Ajuste\r\n"));
			return(retS);
			break;
		}
	}
	return(retS);
}
/*
 * -----------------------------------------------------------------------------------
 * Funciones auxiliares de la FSM de ajustar
 * -----------------------------------------------------------------------------------
 */
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
		xprintf_P(PSTR("PILOTO: pA:[%d]->%0.3f, pB:[%d]->%0.3f\r\n"), i, presion[PLTCB.pA_channel], i, presion[PLTCB.pB_channel] );
		// La presion la expreso en gramos !!!
		PLTCB.pA += presion[PLTCB.pA_channel];
		PLTCB.pB += presion[PLTCB.pB_channel];
		vTaskDelay( ( TickType_t)( intervalo_secs * 1000 / portTICK_RATE_MS ) );
	}

	PLTCB.pA /= samples;
	PLTCB.pB /= samples;
	xprintf_P(PSTR("PILOTO: pA=%.02f, pB=%.02f\r\n"), PLTCB.pA, PLTCB.pB );

	// DYNAMIC CONTROL: Si estoy al inicio del ciclo, inicializo el control dinamico
	// Lo hago aca porque es donde tengo las presiones.
	if ( PLTCB.loops == 0 ) {
		pv_dyncontrol_init();
	}
}
//------------------------------------------------------------------------------------
bool pv_piloto_adjust_conditions( void )
{
	/*
	 * Chequeo todas las condiciones necesarias para activar el servo o salir
	 */

bool retS = false;

	// Debo tener algún cartucho
	if ( pv_ctl_max_reintentos_reached()) {
		PLTCB.exit_code = MAX_TRYES;
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:MAX_TRYES\r\n"));
		retS = false;
		goto quit;
	}

	// La pA siempre debe ser positiva
	if ( ! pv_ctl_pA_es_positiva( PLTCB.pA) ) {
		PLTCB.exit_code = PA_ERR;
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:PA_ERR\r\n"));
		retS = false;
		goto quit;
	}

	// La pB siempre debe ser positiva
	if ( ! pv_ctl_pB_es_positiva( PLTCB.pB) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:PB_ERR\r\n"));
		PLTCB.exit_code = PB_ERR;
		retS = false;
		goto quit;
	}

	// Si todo anda bien, pA debe ser mayor que pB
	if ( ! pv_ctl_pA_mayor_pB( PLTCB.pA,  PLTCB.pB) ) {
		PLTCB.exit_code = PA_LESS_PB;
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:PA_LESS_PB\r\n"));
		retS = false;
		goto quit;
	}

	// Si ya movi el motor, debe haber cambiado la presion con 1 revolucion completa.
	if ( ! pv_dyncontrol_check() ) {
		PLTCB.exit_code = ADJUST_ERR;
		retS = false;
		goto quit;
	}

	// En los siguentes controles hay que ver si se ajusta a la baja o al alza.
	if ( ! pv_ctl_band_gap_suficiente( PLTCB.pA, PLTCB.pB, PLTCB.pRef) ) {
		PLTCB.exit_code = BAND_ERR;
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:BAND_ERR\r\n"));
		retS = false;
		goto quit;
	}

	if ( ! pv_ctl_caudal(1.0, PLTCB.pA,  PLTCB.pB, PLTCB.pRef ) )  {
		PLTCB.exit_code = CAUDAL_ERR;
		retS = false;
		goto quit;
	}

	// Ultima condicion a evaluar.
	if ( pv_ctl_pB_alcanzada() ) {
		PLTCB.exit_code = POUT_REACHED;
		xprintf_PD( DF_APP, PSTR("PILOTO: FSMajuste: outcode:PA_REACHED\r\n"));
		retS = false;
		goto quit;
	}

	retS = true;
quit:

	return(retS);
}
//------------------------------------------------------------------------------------
bool pv_piloto_process_output( void )
{
	/*
	 * Procesa la salida de la FSM.
	 * Puede salir directamente o hacer un rollback de los pulsos generados
	 * para no tener al piloto con riesgo de salirse del tornillo.
	 * Si no pudimos ajustar la presion, indico con false para que la FSM reponga
	 * la presion de ajuste en la cola para un proximo reintento.
	 * Condiciones:
	 * MAX_TRYES:  No deberia reintentar mas. (true)
	 * POUT_REACHED: True
	 * OTROS_CASOS: Deberia reintentar cuando esten las condiciones ( false )
	 */

	xprintf_PD( DF_APP, PSTR("PILOTO: process_output (%d)\r\n"), PLTCB.exit_code );

	if ( PLTCB.exit_code == MAX_TRYES ) {
		// En varios intentos no pude llegar pero ajuste lo mas posible.
		return(true);

	} else if ( PLTCB.exit_code == POUT_REACHED ) {
		// Alcance la presion target.
		return(true);

	} else {
		// En todos los otros casos debo hacer un rollback
		pv_rollback_ajustar_piloto();
		xprintf_P(PSTR("PILOTOS: ROLLBACK\r\n"));
		return(false);
	}

	return(false);
}
//------------------------------------------------------------------------------------
void pv_piloto_ajustar(void)
{
	/*
	 * Arranco el motor
	 */

	u_wdg_kick( plt_app_wdg,  240 );

	/*
	 * Dynamic Control:
	 * Antes de mover el piloto, actulizamos los parametros para hacer este control.
	 * Llevo el conteo de los pulsos aplicados al servo.
	 * pulse_count decrece a 0 al final del ajuste !!
	 *
	 */

	// Muevo el piloto.
	// Arranca el timer que por callbacks va a generar los pulsos
	xprintf_P(PSTR("PILOTO: start\r\n"));
	// Activo el driver
	xprintf_P(PSTR("STEPPER: driver pwr on\r\n"));
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
	PLTCB.pulsos_calculados = (int16_t)( delta_pres * piloto_conf.pulsesXrev  / DPRES_X_REV );

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
		PLTCB.pulsos_a_aplicar = (int16_t)(0.7 * PLTCB.pulsos_calculados);
	} else {
		PLTCB.pulsos_a_aplicar = (int16_t)(0.9 * PLTCB.pulsos_calculados);
	}
	// METODO 2: Los pulsos son los que me da el calculo.

	// F) Controlo no avanzar mas de 500gr aprox x loop !!! ( 1 revolucion )
	if ( PLTCB.pulsos_a_aplicar > piloto_conf.pulsesXrev ) {
		PLTCB.pulsos_a_aplicar = piloto_conf.pulsesXrev;
	}
	PLTCB.pulse_counts = (int16_t) PLTCB.pulsos_a_aplicar;
}
//------------------------------------------------------------------------------------
void pv_piloto_print_parametros( void )
{
	// Muestro en pantalla los parametros calculados de ajuste de presion.

	xprintf_P(PSTR("-----------------------------\r\n"));

	xprintf_P(PSTR("PILOTO: loops=%d\r\n"), PLTCB.loops );
	xprintf_P(PSTR("PILOTO: pA=%.03f\r\n"), PLTCB.pA );
	xprintf_P(PSTR("PILOTO: pB=%.03f\r\n"), PLTCB.pB );
	xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"),   PLTCB.pRef );
	xprintf_P(PSTR("PILOTO: deltaP=%.03f\r\n"), ( PLTCB.pB - PLTCB.pRef));
	xprintf_P(PSTR("PILOTO: pulses calc=%d\r\n"), PLTCB.pulsos_calculados );
	xprintf_P(PSTR("PILOTO: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );

	xprintf_P(PSTR("PILOTO: dync_pB0=%.03f\r\n"), PLTCB.dync_pB0 );
	xprintf_P(PSTR("PILOTO: dync_pulsos=%d\r\n"), PLTCB.dync_pulsos );

	xprintf_P(PSTR("PILOTO: pulses rollback=%d\r\n"), PLTCB.pulsos_rollback );

	xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
	}
	xprintf_P(PSTR("-----------------------------\r\n"));
}
/*
 *------------------------------------------------------------------------------------
 * CONTROLES GENERALES
 *------------------------------------------------------------------------------------
 */
bool pv_ctl_max_reintentos_reached(void)
{
	// Controla si alcanzamos la maxima cantidad de reintentos de ajustes de presion.

bool retS = false;

	if ( ++PLTCB.loops >=  MAX_INTENTOS ) {
		xprintf_P(PSTR("PILOTO: Maxima cantidad de intentos alcanzada. Exit.\r\n"));
		retS = true;

	} else {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check intentos (%d) OK.\r\n"), PLTCB.loops );
		retS = false;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
bool pv_ctl_pA_es_positiva( float pA )
{

	// Controlo que la pA sea positiva ( mayor que un minimo de 0.5k)

bool retS = false;

	if ( pA > MIN_PA ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check pA(%.03f) > %.03f OK\r\n"),pA, MIN_PA );
		retS = true;
	} else {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: pA < %.03f.!!\r\n"), MIN_PA );
		retS = false;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_pB_es_positiva( float pB)
{

	// Controlo que la pB sea positiva ( mayor que un minimo de 0.5k)

bool retS = false;

	if ( pB >= MIN_PB ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check pB(%.03f) > %.03f OK\r\n"), pB, MIN_PB );
		retS = true;
	} else {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: pB < %.03f.!!\r\n"), MIN_PB );
		retS = false;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_pA_mayor_pB( float pA, float pB )
{
	// pA debe ser mayor que pB
	// Puede ocurrir que baje la alta debajo de la regulacion con lo que pB se pega
	// a la alta y ambas quedan iguales.

bool retS = false;

	if ( pA > pB  ) {
		retS = true;
		xprintf_P(PSTR("PILOTO: Check (pA > pB) OK\r\n") );
	} else {
		xprintf_P(PSTR("PILOTO: Ajuste ERROR: ( pA < pB) .!!\r\n") );
		retS = false;
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
bool pv_ctl_band_gap_suficiente( float pA, float pB, float pRef )
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
bool ajuste_a_baja = false;

	ajuste_a_baja = ( pRef < pB );

	// Si ajusto a la baja no tengo que controlar el bandgap.
	if ( ajuste_a_baja ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check bandgap OK (Ajuste a baja)\r\n") );
		retS = true;
		goto quit;
	}

	// Ajuste a la suba. Debo subir pB.

	// 1. No hay margen de regulacion
	if ( ( pA - pB ) < DELTA_PA_PB ) {
		xprintf_P(PSTR("PILOTO: Check bandgap ERROR: (pA-pB) < %.02f gr.!!\r\n"), DELTA_PA_PB );
		retS = false;
		goto quit;
	}

	// 2. Hay margen de regulacion pero la nueva presion a alcanzar supera el limite.
	//    La ajusto al maximo permitido
	if ( ( pA - pRef ) < DELTA_PA_PREF ) {
		// La nueva pA ( PLTCB.pRef ) debe dejar una banda de ajuste con pA.
		PLTCB.pRef = pA - DELTA_PA_PREF;
		xprintf_P(PSTR("PILOTO: Recalculo (pA-pRef) < %.03f gr.!!\r\n"), DELTA_PA_PREF );
		xprintf_P(PSTR("        Nueva pRef=%.03f\r\n"), PLTCB.pRef );
		retS = true;
		goto quit;
	}

	// Entonces hay banda suficiente para subir.
	xprintf_PD(DF_APP, PSTR("PILOTO: Check bandgap OK (Ajuste al alza %.03f)\r\n"), ( pA - pB ) );
	retS = true;

quit:

	return(retS);

}
//------------------------------------------------------------------------------------
bool pv_ctl_caudal( float qMin, float pA, float pB, float pRef)
{
	/*
	 * Si medimos caudal, en un ajuste a la baja debemos partir de un caudal positivo.
	 * Si el ajuste es al alza, podemos partir de un caudal 0.
	 */

bool mido_caudal = false;
bool ajuste_a_baja = false;

	mido_caudal = ( plt_ctl_vars.Q_module != NONE );
	ajuste_a_baja = ( pRef < pB );

	// No mido caudal: No puedo hacer nada.
	if ( ! mido_caudal ) {
		xprintf_PD(DF_APP, PSTR("PILOTO: Check Caudal OK (No mido)r\n") );
		return(true);
	}

	// Si mido caudal.
	if ( ajuste_a_baja ) {
		// Debo tener caudal para poder bajar la presion
		if ( plt_ctl_vars.caudal > qMin ) {
			xprintf_PD(DF_APP, PSTR("PILOTO: Check Caudal OK (%.03f)\r\n"), plt_ctl_vars.caudal );
			return(true);
		} else {
			xprintf_PD(DF_APP, PSTR("PILOTO: Check Caudal ERROR !! (%.03f)\r\n"), plt_ctl_vars.caudal );
			return(false);
		}
	} else {
		// Ajuste al alza. Puedo partir de Q=0
		xprintf_PD(DF_APP, PSTR("PILOTO: Check Caudal OK (Ajuste al alza)\r\n") );
		return(true);
	}

	return(true);

}
/*
 *------------------------------------------------------------------------------------
 * ROLLBACK
 *------------------------------------------------------------------------------------
 */
void pv_rollback_init(void)
{
	// Cuando arranca un ciclo de mover el piloto, inicializo
	PLTCB.pulsos_rollback = 0;
}
/*------------------------------------------------------------------------------------*/
void pv_rollback_update(void)
{
	/*
	 *  Cada vez que voy a mover el piloto, actualizo el rollback
	 *  Los pulsos de rollback tienen sentido contrario.
	 */

	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.pulsos_rollback -= PLTCB.pulse_counts;
	} else {
		PLTCB.pulsos_rollback += PLTCB.pulse_counts;
	}
}
/*------------------------------------------------------------------------------------*/
void pv_rollback_ajustar_piloto( void )
{
	/*
	 * Aplico los pulsos de rollback para llevar el piloto a la posicion inicial
	 * del ciclo.
	 * No controlo condiciones ni presiones.
	 *
	 */

	xprintf_P( PSTR("PILOTO: Rollback Start\r\n"));
	u_wdg_kick( plt_app_wdg,  240 );

	if ( PLTCB.pulsos_rollback > 0 ) {
		PLTCB.dir = STEPPER_FWD;
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	PLTCB.pulsos_a_aplicar = PLTCB.pulsos_rollback;
	PLTCB.pwidth = piloto_conf.pWidth;
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;

	// Print datos.
	xprintf_P(PSTR("-----------------------------\r\n"));
	xprintf_P(PSTR("PILOTO: ROLLBACK.\r\n"));
	xprintf_P(PSTR("PILOTO: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
	}
	xprintf_P(PSTR("-----------------------------\r\n"));


	// Activo el driver
	xprintf_P(PSTR("STEPPER: driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("STEPPER: driver pulses start..\r\n"));
	stepper_awake();
	stepper_start();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

	// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
	while ( PLTCB.motor_running ) {
		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
		// El rollback puede demorar mucho por lo que debo controlar el wdg.
		u_wdg_kick( plt_app_wdg,  240 );
	}

}
/*
 *------------------------------------------------------------------------------------
 * DYNCONTROL
 *------------------------------------------------------------------------------------
 */
void pv_dyncontrol_init(void)
{
	// Inicializa las variables del control dinamico

	PLTCB.dync_pB0 = PLTCB.pB;
	PLTCB.dync_pulsos = 0;
	xprintf_PD( DF_APP, PSTR("PILOTO: dyn_control_init\r\n"));
}
/*------------------------------------------------------------------------------------*/
void pv_dyncontrol_update(void)
{

	xprintf_PD( DF_APP, PSTR("PILOTO: dyn_control_update\r\n"));
	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.dync_pulsos += PLTCB.pulse_counts;
	} else {
		PLTCB.dync_pulsos -= PLTCB.pulse_counts;
	}

}
/*------------------------------------------------------------------------------------*/
bool pv_dyncontrol_check(void)
{
	/*
	 *  Movi el piloto algo mas de 1 vuelta y la pB no se modifico en al menos 200 gr. ( Rollback )
	 *  El piloto se esta moviendo pero la pB no esta ajustando.
	 *  Si di mas de una revolucion, la presion debe haber cambiado al menos 200grs.
	 *  Si cambio, renicio el control dinamico al nuevo punto
	 */

	xprintf_PD( DF_APP, PSTR("PILOTO: Check dyn_control:\r\n"));
	xprintf_PD( DF_APP, PSTR("PILOTO:   pxrev=%d\r\n"), piloto_conf.pulsesXrev );
	xprintf_PD( DF_APP, PSTR("PILOTO:   dync_pulsos=%d\r\n"), PLTCB.dync_pulsos );
	xprintf_PD( DF_APP, PSTR("PILOTO:   pB0=%.03f\r\n"), PLTCB.dync_pB0 );
	xprintf_PD( DF_APP, PSTR("PILOTO:   pB=%.03f\r\n"), PLTCB.pB );

	// Si di al menos una vuelta completa....
	if ( fabs(PLTCB.dync_pulsos) >= piloto_conf.pulsesXrev ) {

		xprintf_PD( DF_APP, PSTR("PILOTO: dyn_control 1xrev.\r\n"));

		if ( fabs( PLTCB.pB - PLTCB.dync_pB0 ) < 0.2 ) {
			// Giro al menos 1 vuelta y no cambio ni 200 grs. ERROR.
			xprintf_P( PSTR("PILOTO: dyn_control PresERROR !!.\r\n"));
			return(false);

		} else {
			// Giro y cambio la presion. Está funcionando.
			// Actualizo los parametros del dyn_control.
			pv_dyncontrol_init();
		}
	}

	xprintf_PD( DF_APP, PSTR("PILOTO: Check dyn_control OK.\r\n"));

	return(true);
}
/*
 * -----------------------------------------------------------------------------------
 * Funciones de servicio
 * -----------------------------------------------------------------------------------
 */
void FSM_piloto_app_service( uint8_t app_wdt )
{
	/*
	 * Implemento una maquina de estados que lee una FIFO donde guardamos las
	 * presiones a fijar. Si hay datos los saca y los ejecuta.
	 *
	 */

int8_t state;

	/*
	 * Inicializo el watchdog de la aplicacion como variable global porque
	 * voy a usarla en varias parte.
	 */
	plt_app_wdg = app_wdt;
	u_wdg_kick( plt_app_wdg,  240 );
	vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );
	xprintf_P(PSTR("PILOTO\r\n"));

	// Vemos si tengo una configuracion que permita trabajar ( definidos canales de presiones )
	if ( ! pv_piloto_check_inputs_conf() ) {
		xprintf_P(PSTR("PILOTOS: No tengo canales pA/pB configurados. EXIT !!\r\n"));
		return;
	}

	// Creo la cola fifo (ringbuffer) de presiones
	rbf_CreateStatic( &pFIFO, &pFifo_storage[0], PFIFO_STORAGE_SIZE );

	state = ST_PRODUCER;

	for(;;) {

		u_wdg_kick( plt_app_wdg,  240 );

		switch(state) {

		case ST_PRODUCER:
			/*
			 * PRODUCTOR:
			 * Chequea si cambio el slot o no. Si cambio, pone la nueva presion en
			 * la cola FIFO.
			 * Los otros productores son testing, online y el propio proceso.
			 */
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMservice: state ST_PRODUCER\r\n"));
			plt_producer_slot_handler();
			state = ST_CONSUMER;
			break;

		case ST_CONSUMER:
			/*
			 * CONSUMER:
			 * Si hay algun dato en la fifo, vemos si es posible ejecutar la accion.
			 */
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMservice: state ST_CONSUMER\r\n"));
			plt_consumer_handler();
			state = ST_AWAIT;
			break;

		case ST_AWAIT:
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO: FSMservice: state ST_AWAIT\r\n"));
			vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );
			state = ST_PRODUCER;
			break;

		default:
			// Error: No deberiamos estar aqui
			rbf_Flush(&pFIFO);
			xprintf_P(PSTR("PILOTO: ERROR. FSMservice: State Default !! (Reset)\r\n"));
			state = ST_PRODUCER;
			break;

		}

	}
}
//------------------------------------------------------------------------------------
void plt_producer_online_handler( float presion)
{
	/*
	 * PRODUCER ( online )
	 * Funcion invocada online para fijar una nueva presion hasta que finalize
	 * el slot.
	 * Solo debo ponerla en la FIFO.
	 *
	 */

	xprintf_P(PSTR("PILOTO: Set presion from Server.\r\n"));
	xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"), presion);
	// Guardo la presion en la cola FIFO.
	rbf_Poke(&pFIFO, &presion );

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
/*
 * -----------------------------------------------------------------------------------
 * Funciones generales
 * -----------------------------------------------------------------------------------
 */
void plt_producer_slot_handler(void)
{
	/*
	 * Es la funcion que controla que si cambio de slot, pone en la cola de datos
	 * la nueva presion
	 */

static int8_t slot_actual = -1;
int8_t slot = -1;

	xprintf_PD( DF_APP, PSTR("\r\nPILOTO: SLOTS_A: slot_actual=%d, slot=%d\r\n"), slot_actual, slot );
	if ( pv_piloto_leer_slot_actual( &slot ) ) {
		xprintf_PD( DF_APP, PSTR("\r\nPILOTO: SLOTS_B: slot_actual=%d, slot=%d\r\n"), slot_actual, slot );
		if ( slot_actual != slot ) {
			// Cambio el slot.
			slot_actual = slot;
			xprintf_P(PSTR("PILOTO: Inicio de ciclo.\r\n"));
			xprintf_P(PSTR("PILOTO: slot=%d, pRef=%.03f\r\n"), slot_actual, piloto_conf.pltSlots[slot_actual].presion);
			// Guardo la presion en la cola FIFO.
			rbf_Poke(&pFIFO, &piloto_conf.pltSlots[slot_actual].presion );
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
void plt_consumer_handler(void)
{
	/*
	 * Leo la presion de la FIFO y veo si puedo ajustar.
	 * Si ajusto y todo sale bien, la saco de la FIFO.
	 * Si no pude ajustar, la dejo en la FIFO para que el proximo ciclo vuelva a reintentar
	 * con esta accion pendiente.
	 */

	if ( rbf_GetCount(&pFIFO) == 0 ) {
		// No hay datos en la FIFO
		return;
	}

	// Leo sin sacar.
	rbf_PopRead(&pFIFO, &PLTCB.pRef );
	/*
	 * Veo si estan las pre-condiciones para intentar ajustar.
	 * Si no estan salgo y queda el ajuste pendiente.
	 */
	if ( ! pv_piloto_pre_conditions() ) {
		return;
	}

	// Estan las condiciones para intentar ajustar.
	if ( FSM_piloto_ajustar_presion() ) {
		/*
		 * Si pude ajustar la presion, la saco de la FIFO.
		 * Si no lo dejo y entonces queda pendiente.
		 */
		rbf_Pop(&pFIFO, &PLTCB.pRef );
	}

}
//------------------------------------------------------------------------------------
bool pv_piloto_pre_conditions( void )
{
	/*
	 * Evalua si existen las condiciones previas necesarias para iniciar el ajuste.
	 * Las precondiciones no son las mismas para ajsutar a la baja que a la alta.
	 */

bool retS = false;

	xprintf_P(PSTR("PILOTO: PREconditions\r\n"));

	xprintf_P(PSTR("PILOTO: pA=%.03f\r\n"), plt_ctl_vars.pA );
	xprintf_P(PSTR("PILOTO: pB=%.03f\r\n"), plt_ctl_vars.pB );
	xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"), PLTCB.pRef );

	if ( PLTCB.pRef > plt_ctl_vars.pB ) {
		// Ajuste al alza
		retS = pv_piloto_pre_conditions_4increase();
	} else {
		// Ajuste a la baja
		retS = pv_piloto_pre_conditions_4reduce();
	}

	return( retS );
}
//------------------------------------------------------------------------------------
bool pv_piloto_pre_conditions_4increase( void )
{
	/*
	 * Evalua si existen las condiciones previas necesarias para iniciar el ajuste a la alta
	 * 1- La presion pA debe ser positiva
	 * 2- Si voy a subirla, la presion pB puede ser que sea 0. pero si debe ser positiva
	 * 3- pA debe ser mayor que pB
	 * 4- No importa que haya o no margen para subir pB. Esto se calcula al ajustar.
	 * 5- El caudal no importa ya que puede estar la valvula cerrada con caudal 0.
	 */

	// Vemos si estan las condiciones para intentar el ajuste.
	if ( ! pv_ctl_pA_es_positiva(plt_ctl_vars.pA) ) {
		return(false);
	}

	if ( ! pv_ctl_pB_es_positiva(plt_ctl_vars.pB ) ) {
		return(false);
	}

	if ( ! pv_ctl_pA_mayor_pB( plt_ctl_vars.pA, plt_ctl_vars.pB ) ) {
		return(false);
	}

	// Estan las condiciones.
	// Guardo la presion en la cola FIFO.
	xprintf_PD( DF_APP, PSTR("PILOTO: Condiciones para aumentar pB OK.\r\n"));
	return(true);

}
//------------------------------------------------------------------------------------
bool pv_piloto_pre_conditions_4reduce( void )
{
	/*
	 * Evalua si existen las condiciones previas necesarias para iniciar el ajuste a la baja
	 * 1- La presion pA debe ser positiva
	 * 2- Si voy a bajarla, la presion pB puede ser positiva
	 * 3- pA debe ser mayor que pB
	 * 5- El caudal no puede ser 0.
	 */

	// Vemos si estan las condiciones para intentar el ajuste.
	if ( ! pv_ctl_pA_es_positiva(plt_ctl_vars.pA) ) {
		return(false);
	}

	if ( ! pv_ctl_pB_es_positiva(plt_ctl_vars.pB ) ) {
		return(false);
	}

	if ( ! pv_ctl_pA_mayor_pB( plt_ctl_vars.pA, plt_ctl_vars.pB ) ) {
		return(false);
	}

	if ( ! pv_ctl_caudal( 1.0, plt_ctl_vars.pA, plt_ctl_vars.pB, PLTCB.pRef )) {
		return(false);
	}

	// Estan las condiciones.
	// Guardo la presion en la cola FIFO.
	xprintf_PD( DF_APP, PSTR("PILOTO: Condiciones para reducir pB OK.\r\n"));
	return(true);

}
//------------------------------------------------------------------------------------
bool pv_piloto_check_inputs_conf(void)
{
	/*
	 *  Se fija si en la configuracion del equipo hay algun canal con el
	 *  nombre pA y pB.
	 *  Si no los hay entonces retorna FALSE
	 *  Tambien determino si mide o no caudal.
	 */

uint8_t i;
bool sRet = false;
char lname[PARAMNAME_LENGTH];


	PLTCB.pA_channel = -1;
	PLTCB.pB_channel = -1;

	for ( i = 0; i < ANALOG_CHANNELS; i++) {
		strncpy(lname, ainputs_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ! strcmp_P( lname, PSTR("PA") ) ) {
			PLTCB.pA_channel = i;
			xprintf_P(PSTR("PILOTO: pAchannel=%d\r\n"), PLTCB.pA_channel);
		}
		if ( ! strcmp_P( lname, PSTR("PB") ) ) {
			PLTCB.pB_channel = i;
			xprintf_P(PSTR("PILOTO: pBchannel=%d\r\n"), PLTCB.pB_channel);
		}
	};

	if ( (PLTCB.pA_channel != -1) && (PLTCB.pB_channel != -1) )
		sRet = true;

	// En plt_ctl_vars indicamos que canales miden c/variable.
	plt_ctl_vars.pA_channel = PLTCB.pA_channel;
	plt_ctl_vars.pB_channel = PLTCB.pB_channel;

	// ------------------------------------------------------------------------------------------
	// CAUDAL
	plt_ctl_vars.Q_module = NONE;
	plt_ctl_vars.Q_channel = -1;

	// Canales analogicos
	for (i=0; i<ANALOG_CHANNELS; i++) {

		strncpy(lname, ainputs_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			plt_ctl_vars.Q_module = ANALOG;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			plt_ctl_vars.Q_module = ANALOG;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}
	}

	// Canales contadores
	for (i=0; i<COUNTER_CHANNELS; i++) {

		strncpy(lname, counters_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			plt_ctl_vars.Q_module = COUNTER;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			plt_ctl_vars.Q_module = COUNTER;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}
	}

	// Canales modbus
	for (i=0; i<MODBUS_CHANNELS; i++) {

		strncpy(lname, modbus_conf.channel[i].name, PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			plt_ctl_vars.Q_module = MODBUS;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			plt_ctl_vars.Q_module = MODBUS;
			plt_ctl_vars.Q_channel = i;
			goto quit;
		}
	}

quit:

	if ( plt_ctl_vars.Q_module == MODBUS ) {
		xprintf_P(PSTR("CAUDAL: Canal %d MODBUS\r\n"), plt_ctl_vars.Q_channel );
	} else if ( plt_ctl_vars.Q_module == ANALOG ) {
		xprintf_P(PSTR("CAUDAL: Canal %d ANALOG\r\n"), plt_ctl_vars.Q_channel );
	} else 	if ( plt_ctl_vars.Q_module == COUNTER ) {
		xprintf_P(PSTR("CAUDAL: Canal %d COUNTER\r\n"), plt_ctl_vars.Q_channel );
	} else {
		xprintf_P(PSTR("CAUDAL: No hay canales configurados\r\n"));
	}
	return(sRet);

}
//------------------------------------------------------------------------------------
bool pv_piloto_leer_slot_actual( int8_t *slot_id )
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

	// Buscamos dentro de que slot se encuentra now.
	*slot_id = last_slot;
	for ( slot = 0; slot <= last_slot; slot++ ) {
		slot_hhmm = piloto_conf.pltSlots[slot].pTime.hour * 100 + piloto_conf.pltSlots[slot].pTime.min;

		// Chequeo inside
		if ( now < slot_hhmm ) {
			// Si estoy al ppio.
			if ( slot == 0 ) {
					*slot_id = last_slot;
			// Estoy en un slot comun
			} else {
				*slot_id = slot - 1;
			}
			break;
		}
	}
	return(true);

}
/*
 * -----------------------------------------------------------------------------------
 * Funciones de test
 * -----------------------------------------------------------------------------------
 */
void plt_producer_testing_handler(char *s_pRef )
{
	/*
	 * PRODUCER ( test )
	 * Guardo la presion en la FIFO.
	 */

float pRef = atof(s_pRef);

	rbf_Poke(&pFIFO, &pRef );

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

	if ( *s_npulses == '\0' ) {
		xprintf_P(PSTR("steppertest ERROR: debe indicar npulses\r\n"));
		return;
	}

	PLTCB.pwidth = atoi(s_pwidth);
	if ( *s_pwidth == '\0' ) {
		xprintf_P(PSTR("steppertest ERROR: debe indicar pwidth\r\n"));
		return;
	}


	PLTCB.pulsos_a_aplicar = atoi(s_npulses);
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;

	// Print datos.
	xprintf_P(PSTR("-----------------------------\r\n"));
	xprintf_P(PSTR("PILOTO: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
	}
	xprintf_P(PSTR("-----------------------------\r\n"));

	// Activo el driver
	xprintf_P(PSTR("STEPPER: driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("STEPPER: driver pulses start..\r\n"));
	stepper_awake();
	stepper_start();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

	// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
	//while ( PLTCB.motor_running ) {
	//	vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
	//}

	xprintf_P(PSTR("STEPPER: test running.\r\n"));

}
/*
 * -----------------------------------------------------------------------------------
 * Funciones de configuracion
 * -----------------------------------------------------------------------------------
 */
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

