/*
 * ul_pilotos.c
 *
 *  Created on: 13 may. 2021
 *      Author: pablo
 *
 *  TESTING:
 *  2021-11-12:
 *  1- Paso PB_MIN a 0.0 ya que puede ser que la valvula este cerrada y quiera subirla. OK
 *  2- Agrego una condicion que vea que pREF sea mayor a pB - pGAP. OK
 *  3- El control dinamico no se hace el primer loop porque esta en 0 y da error. OK
 *  4- Si en condiciones4start/adjust ya estoy en el bandgap, borro el request OK
 *  5- Revisaer inicilizar variables antes de usarlas en funciones. OK.
 *  6- El ajuste dinamico se va actualizando y tambien el rollback.
 *     Ver si volver al ppio o al punto anterior.
 *     Volvemos al ultimo punto correcto dado por el dyncontrol de modo que quedar
 *     lo mas cerca de la consigna.
 *      Cuantas veces reintento un rollback antes de borrar el pedido ?
 *
 *
 *
 *
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
 *  Funcionamiento:
 *  - Antes de comenzar a ajustar, controlo si tengo las condiciones.
 *    Si no las tengo, pospongo el ajuste.
 *  - Comienzo a ajustar:
 *    Cada ajuste re-evaluo las condiciones.
 *    Si termino OK, salgo y borro la accion
 *    So no termino bien a lo sumo hago un rollback pero no dejo pendiente el ajuste.
 *
 *
 *  TESTING:
 *  1- Caso normal OK.
 *  2- Subir pB mas alto de pA. Debe dejarlo en pA-pband. OK
 *  3- Bajar.OK. ( Cambio para que en este caso tambien aplicar el 70% de los pulsos )
 *
 *  Considerar leer el slot c/ajuste por si cambio en medio de un cambio de slot
 *  o al final para reajustar
 *
 *  Casos particulares:
 *
 *  1- Reduccion de presion, mide caudal
 *     pB nunca llega a 0 porque queda presurizado
 *     La salida es por q=0
 *
 *  2- Reduccion de presion, NO mide caudal
 *     pB nunca llega a 0 porque queda presurizado
 *     La salida es por dyncontrol pero se hace un rollback y entonces se vuelve a reintentar
 *     En este caso doy por terminado pero no dejo pendiente.
 *
 *  3- Ajuste al alza.
 *
 */

#include "ul_pilotos.h"
#include "tkApp.h"

TimerHandle_t plt_pulse_xTimer;
StaticTimer_t plt_pulse_xTimerBuffer;


void plt_productor_handler(void);
void plt_productor_handler_testStepper( int16_t npulses, int8_t pwidth );

void plt_consumer_handler(void);
bool plt_consumer_handler_presion(float presion );
bool plt_consumer_handler_stepper(float pulsos );

void plt_pulse_TimerCallback( TimerHandle_t xTimer );
bool plt_leer_slot_actual( int8_t *slot_id );
bool FSM_piloto_ajustar_presion( void );

void plt_read_inputs( int8_t samples, uint16_t intervalo_secs );


void plt_ajustar( void );
void plt_calcular_parametros_ajuste( void );
void plt_print_parametros( void );
void plt_process_output( void );

bool plt_ctl_max_reintentos_reached(void);
bool plt_ctl_limites_pA( float pA, float pA_min, float pA_max );
bool plt_ctl_limites_pB( float pB, float pB_min, float pB_max );
bool plt_ctl_pA_mayor_pB( float pA, float pB);
bool plt_ctl_pB_alcanzada( float pB, float pRef );
bool plt_ctl_band_gap_suficiente(float pA, float pB, float pRef );
bool plt_ctl_conditions4adjust( void );
bool plt_ctl_conditions4start( void );

bool plt_ctl_Caudal_minimo( float Qmin );

void plt_rollback_init(void);
void plt_rollback_update(void);
void plt_rollback_ajustar_piloto( void );

void plt_dyncontrol_init(void);
void plt_dyncontrol_update(void);
bool plt_dyncontrol_pass(void);

/*
 * -----------------------------------------------------------------------------------
 * Funcion de servicio
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

	PLTCB.pA_channel = ctlapp_vars.pA_channel;
	PLTCB.pB_channel = ctlapp_vars.pB_channel;

	xprintf_P(PSTR("PILOTO pA_channel: %d\r\n"), PLTCB.pA_channel);
	xprintf_P(PSTR("PILOTO pB_channel: %d\r\n"), PLTCB.pB_channel);

	// Vemos si tengo una configuracion que permita trabajar ( definidos canales de presiones )
	if ( PRESIONES_NO_CONFIGURADAS() ) {
		xprintf_P(PSTR("PILOTO: No tengo canales pA/pB configurados. EXIT !!\r\n"));
		return;
	}

	// Creo la cola fifo (ringbuffer) de elementos
	ringBuffer_CreateStatic ( &pFIFO, &pFifo_storage, PFIFO_STORAGE_SIZE, sizeof(s_jobOrder)  );
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
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO FSMservice: state ST_PRODUCER\r\n"));
			plt_productor_handler();
			state = ST_CONSUMER;
			break;

		case ST_CONSUMER:
			/*
			 * CONSUMER:
			 * Si hay algun dato en la fifo, vemos si es posible ejecutar la accion.
			 */
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO FSMservice: state ST_CONSUMER\r\n"));
			plt_consumer_handler();
			state = ST_AWAIT;
			break;

		case ST_AWAIT:
			xprintf_PD( DF_APP, PSTR("\r\nPILOTO FSMservice: state ST_AWAIT\r\n"));
			//vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );
			vTaskDelay( ( TickType_t)( (systemVars.timerPoll) * 1000 / portTICK_RATE_MS ) );

			state = ST_PRODUCER;
			break;

		default:
			// Error: No deberiamos estar aqui
			ringBuffer_Flush(&pFIFO);
			xprintf_P(PSTR("PILOTO FSMservice: ERROR. State Default !! (Reset)\r\n"));
			state = ST_PRODUCER;
			break;

		}

	}
}
/*
 * -----------------------------------------------------------------------------------
 * Productors
 * -----------------------------------------------------------------------------------
 */
void plt_productor_handler(void)
{
	/*
	 * PRDUCTOR HANDLE
	 * Es la funcion que controla que si cambio de slot, pone en la cola de datos
	 * la nueva orden de trabajo de presion.
	 */

static int8_t slot_actual = -1;
int8_t slot = -1;
s_jobOrder jobOrder;

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: start.\r\n"));

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: SLOTS_A: slot_actual=%d, slot=%d\r\n"), slot_actual, slot );
	if ( plt_leer_slot_actual( &slot ) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: SLOTS_B: slot_actual=%d, slot=%d\r\n"), slot_actual, slot );
		// Cambio el slot. ?
		if ( slot_actual != slot ) {
			slot_actual = slot;
			xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR:: Inicio de ciclo.\r\n"));
			xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: slot=%d, pRef=%.03f\r\n"), slot_actual, piloto_conf.pltSlots[slot_actual].presion);

			// Un nuevo slot borra todo lo anterior.
			ringBuffer_Flush(&pFIFO);

			// Guardo la presion en la cola FIFO.
			jobOrder.tipo = ORDER_PRESION;
			jobOrder.valor = piloto_conf.pltSlots[slot_actual].presion;

			ringBuffer_Poke(&pFIFO, &jobOrder );
		}
	}
	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: end.\r\n"));
}
//------------------------------------------------------------------------------------
void piloto_productor_handler_cmdOrders(char *s_pRef )
{
	/*
	 * PRODUCTOR ( test ) - asincronico
	 * Se activa desde cmdMode
	 * Guardo una orden de trabajo de PRESION en la cola
	 * Borro las ordenes pendientes para ejecutar esta en forma inmediata.
	 */

s_jobOrder jobOrder;

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: cmdOrders start.\r\n"));

	// Borro todas las peticiones pendientes.
	//ringBuffer_Flush(&pFIFO);

	jobOrder.tipo = ORDER_PRESION;
	jobOrder.valor = atof(s_pRef);
	ringBuffer_Poke(&pFIFO, &jobOrder );

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: cmdOrders end.\r\n"));
}
//------------------------------------------------------------------------------------
void piloto_productor_handler_onlineOrders( float presion )
{
	/*
	 * PRODUCTOR ( online ) - asincronico
	 * Cuando remotamente se quiere alterar la presion, via comando online se envia
	 * un frame que invoca a esta funcion.
	 * Funcion invocada online para fijar una nueva presion hasta que finalize
	 * el slot.
	 * Solo debo ponerla en la FIFO.
	 *
	 * Guardo una orden de trabajo de PRESION.
	 */

s_jobOrder jobOrder;

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: onlineOrders start.\r\n"));

	jobOrder.tipo = ORDER_PRESION;
	jobOrder.valor = presion;
	ringBuffer_Poke(&pFIFO, &jobOrder );

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: onlineOrders end.\r\n"));

}
//------------------------------------------------------------------------------------
void plt_productor_handler_testStepper( int16_t npulses, int8_t pwidth )
{
	/*
	 * Estoy en modo testing de modo que borro todas las otras ordenes
	 * de la cola.
	 * Inserta la orden de mover el stepper en la cola.
	 */

s_jobOrder jobOrder;

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: testStepper start.\r\n"));

	// Borro todas las peticiones pendientes.
	//ringBuffer_Flush(&pFIFO);

	// Completo PLT_CB
	PLTCB.pwidth = pwidth;

	// Creo la orden de trabajo
	jobOrder.tipo = ORDER_STEPPER;
	jobOrder.valor = npulses;
	ringBuffer_Poke(&pFIFO, &jobOrder );

	xprintf_PD( DF_APP, PSTR("PILOTO PRODUCTOR: testStepper end.\r\n"));
}
/*
 * -----------------------------------------------------------------------------------
 * Consumers:
 * -----------------------------------------------------------------------------------
 */
void plt_consumer_handler(void)
{
	/*
	 * CONSUMIDOR:
	 * Leo la presion de la FIFO y veo si puedo ajustar.
	 * Si ajusto y todo sale bien, la saco de la FIFO.
	 * Si no pude ajustar, la dejo en la FIFO para que el proximo ciclo vuelva a reintentar
	 * con esta accion pendiente.
	 */

s_jobOrder jobOrder;
bool borrar_request = false;

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: start.\r\n"));

	// No hay datos en la FIFO
	if ( ringBuffer_GetCount(&pFIFO) == 0 ) {
		goto quit;
	}

	// Leo sin sacar.
	ringBuffer_PopRead(&pFIFO, &jobOrder );
	//xprintf_PD( DF_APP, PSTR(">>PILOTO: consumer tipo=%d, valor=%f\r\n"), jobOrder.tipo, jobOrder.valor );

	if ( jobOrder.tipo == ORDER_PRESION ) {
		// Ajuste de presion
		borrar_request = plt_consumer_handler_presion( jobOrder.valor);
	} else if ( jobOrder.tipo == ORDER_STEPPER ) {
		// Movimiento del stepper
		borrar_request = plt_consumer_handler_stepper( jobOrder.valor);
	} else {
		xprintf_P( PSTR("PILOTO CONSUMER: ERROR (jobOrder)\r\n"));
		goto quit;
	}

	// Si pude ejecutar el handler, borro el request.
	if ( borrar_request ) {
		ringBuffer_Pop(&pFIFO, NULL );
	}

quit:

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: end.\r\n"));
}
//------------------------------------------------------------------------------------
bool plt_consumer_handler_presion(float presion )
{
	/*
	 * Debo procesar una order de ajustar la presion.
	 * Chequeo las precondiciones.
	 * Si las paso, mando ajustar.
	 */

bool borrar_request = false;

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: handler_presion start.\r\n"));

	PLTCB.pRef = presion;
	PLTCB.pError = PERROR;

	// Si la presion esta en la banda, salgo y borro el request
	if ( plt_ctl_pB_alcanzada( get_pB(), PLTCB.pRef )) {
		xprintf_PD(DF_APP, PSTR("PILOTO CONSUMER: Presion en banda.\r\n") );
		borrar_request = true;	// Para borrar el request
		goto quit;
	}

	if ( ! plt_ctl_conditions4start() ) {
		// No hay condiciones para empezar a ajustar.
		// Salgo y espero el proximo turno pero no borro el request
		borrar_request = false;
		goto quit;
	}

	// Estan las condiciones para intentar ajustar.
	// Dependiendo si ajuste o no es que borro el request o queda pendiente
	borrar_request = FSM_piloto_ajustar_presion();

quit:

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: handler_presion stop.\r\n"));
	return(borrar_request);
}
//------------------------------------------------------------------------------------
bool plt_consumer_handler_stepper(float pulsos )
{
	/*
	 * Debo procesar una orden de mover el piloto.
	 * El sentido lo indica el signo: + es FW, - es REV
	 * Nunca lo giro mas de 1 revolucion.
	 */

int16_t lpulsos;

	u_wdg_kick( plt_app_wdg,  240 );

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: handler_stepper start.\r\n"));

	lpulsos = (int16_t)pulsos;

	// Calculo los limites para no girar mas de 1 revolucion
	if ( ( lpulsos > 0) && ( lpulsos > piloto_conf.pulsesXrev ) ) {
		lpulsos = piloto_conf.pulsesXrev;
	}

	if ( ( lpulsos < 0) && ( abs(lpulsos) > piloto_conf.pulsesXrev ) ) {
		lpulsos = -1 * piloto_conf.pulsesXrev;
	}

	// Muevo el stepper
	// Completo PLT_CB
	PLTCB.pulsos_a_aplicar = abs(lpulsos);
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;
	if ( lpulsos > 0 ) {
		PLTCB.dir = STEPPER_FWD;
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	// Print datos.
	xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: dir=Reverse\r\n"));
	}

	// Activo el driver
	xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("PILOTO CONSUMER: handler_stepper: driver pulses start..\r\n"));
	stepper_awake();
	stepper_start();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

	// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
	while ( PLTCB.motor_running ) {
		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
	}

	xprintf_PD( DF_APP, PSTR("PILOTO CONSUMER: handler_stepper stop.\r\n"));

	return(true);

}
//------------------------------------------------------------------------------------
/*
 * -----------------------------------------------------------------------------------
 * FSM de ajustar la presion.
 * -----------------------------------------------------------------------------------
 */
bool FSM_piloto_ajustar_presion( void )
{

uint8_t state = PLT_READ_INPUTS;
bool borrar_request = false;

	// Inicializo
	xprintf_PD( DF_APP, PSTR("\r\nPILOTO FSMajuste: ENTRY\r\n"));
	xprintf_P(PSTR("PILOTO FSMajuste: Ajuste de presion a (%.03f)\r\n"), PLTCB.pRef );

	PLTCB.loops = 0;
	PLTCB.exit_code = UNKNOWN;
	PLTCB.accion_pendiente = false;
	PLTCB.f_emergencia = false;
	PLTCB.recalculo_de_pB = false;

	plt_rollback_init();

	state = PLT_READ_INPUTS;

	for (;;) {

		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );

		switch( state ) {

		case PLT_READ_INPUTS:
			// Leo las entradas
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: state READ_INPUTS\r\n"));
			// Espero siempre 30s antes para que se estabilizen. Sobre todo en valvulas grandes
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: await 30s\r\n"));
			u_wdg_kick( plt_app_wdg,  240 );
			vTaskDelay( ( TickType_t) (30000 / portTICK_RATE_MS ) );
			//
			plt_read_inputs(5, INTERVALO_PB_SECS );
			state = PLT_CHECK_CONDITIONS4ADJUST;
			break;

		case PLT_CHECK_CONDITIONS4ADJUST:
			// Veo si tengo las condiciones para continuar el ajuste.
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: state CHECK_CONDITIONS\r\n"));
			if ( plt_ctl_conditions4adjust() ) {
				state = PLT_AJUSTE;
			} else {
				state = PLT_PROCESS_OUTPUT;
			}
			break;

		case PLT_AJUSTE:
			// Ajusto
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: state AJUSTE\r\n"));
			// Calculo la direccion y los pulsos a aplicar
			plt_calcular_parametros_ajuste();
			//
			plt_rollback_update();
			//
			plt_dyncontrol_update();
			//
			// Muestro el resumen de datos
			plt_print_parametros();
			//
			plt_ajustar();
			state = PLT_READ_INPUTS;
			break;

		case PLT_PROCESS_OUTPUT:
			// Evaluo condiciones de salida
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: state OUTPUT\r\n"));
			plt_process_output();
			state = PLT_EXIT;
			break;

		case PLT_EXIT:
			// Termino y salgo.

			/*
			 * Si pude ajustar la presion, la saco de la FIFO.
			 * Si no lo dejo y entonces queda pendiente.
			 */
			xprintf_PD( DF_APP, PSTR("PILOTO FSMajuste: state EXIT\r\n"));

			if (PLTCB.accion_pendiente ) {
				xprintf_P(PSTR("PILOTO FSMajuste: Fin de Ajuste (pendiente)\r\n"));
				borrar_request = false;
			} else {
				xprintf_P(PSTR("PILOTO FSMajuste: Fin de Ajuste\r\n"));
				borrar_request = true;
			}
			return(borrar_request);
			break;
		}
	}

	return(borrar_request);
}
/*
 * -----------------------------------------------------------------------------------
 * Funciones auxiliares de la FSM de ajustar
 * -----------------------------------------------------------------------------------
 */
void plt_read_inputs( int8_t samples, uint16_t intervalo_secs )
{
	/*
	 * Lee las entradas: pA,pB.
	 * Medimos N veces y promediamos
	 * Dejamos el valor en gramos
	 *
	 */

uint8_t i;
float presion[ANALOG_CHANNELS];


	//xprintf_P(PSTR("DEBUG PILOTO: pA_channel:[%d], pB_channel:[%d]\r\n"), PLTCB.pA_channel,PLTCB.pB_channel);

	// Mido pA/pB
	PLTCB.pA = 0;
	PLTCB.pB = 0;
	for ( i = 0; i < samples; i++) {
		ainputs_read( presion, NULL, false );
		xprintf_P(PSTR("PILOTO READINPUTS: pA:[%d]->%0.3f, pB:[%d]->%0.3f\r\n"), i, presion[PLTCB.pA_channel], i, presion[PLTCB.pB_channel] );
		// La presion la expreso en gramos !!!
		PLTCB.pA += presion[PLTCB.pA_channel];
		PLTCB.pB += presion[PLTCB.pB_channel];
		vTaskDelay( ( TickType_t)( intervalo_secs * 1000 / portTICK_RATE_MS ) );
	}

	PLTCB.pA /= samples;
	PLTCB.pB /= samples;
	xprintf_P(PSTR("PILOTO READINPUTS: pA=%.02f, pB=%.02f\r\n"), PLTCB.pA, PLTCB.pB );

	// DYNAMIC CONTROL: Si estoy al inicio del ciclo, inicializo el control dinamico
	// Lo hago aca porque es donde tengo las presiones.
	if ( PLTCB.loops == 0 ) {
		plt_dyncontrol_init();
	}
}
//------------------------------------------------------------------------------------
bool plt_ctl_conditions4adjust( void )
{
	/*
	 * Chequeo todas las condiciones necesarias para activar el servo o salir
	 * Ahora se chequean con los valores leidos para esto de las entradas.
	 */

bool ajustar = true;
bool ajuste_al_alza = false;

	xprintf_P(PSTR("PILOTO CONDXADJUST: start\r\n"));

	xprintf_P(PSTR("PILOTO CONDXADJUST: pA=%.03f\r\n"), PLTCB.pA );
	xprintf_P(PSTR("PILOTO CONDXADJUST: pB=%.03f\r\n"), PLTCB.pB );
	xprintf_P(PSTR("PILOTO CONDXADJUST: pRef=%.03f\r\n"), PLTCB.pRef );
	if ( MIDO_CAUDAL() ) {
		xprintf_P(PSTR("PILOTO CONDXADJUST: Q=%.03f\r\n"), get_Q() );
	}
	xprintf_P(PSTR("PILOTO CONDXADJUST: pError=%.03f\r\n"), PLTCB.pError );

	// Primero debemos saber si va al alza o a la baja.
	if ( PLTCB.pRef > PLTCB.pB ) {
		ajuste_al_alza = true;
		xprintf_PD(DF_APP, PSTR("PILOTO CONDXADJUST: Ajuste al alza.\r\n") );
	} else {
		ajuste_al_alza = false;
		xprintf_PD(DF_APP, PSTR("PILOTO CONDXADJUST: Ajuste a la baja.\r\n") );
	}

	// CONDICION 1: Alcance la presion buscada. Salgo.
	if ( plt_ctl_pB_alcanzada( PLTCB.pB, PLTCB.pRef ) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: PB_REACHED\r\n"));
		PLTCB.exit_code = POUT_REACHED;
		PLTCB.run_rollback = false;
		if ( PLTCB.recalculo_de_pB ) {
			// Ajuste lo mas que pude pero no el valor real. Queda pendiente
			PLTCB.accion_pendiente = true;
		} else {
			// Ajuste al valor del slot. OK.
			PLTCB.accion_pendiente = false;
		}
		ajustar = false;
		goto quit;
	}


	// CONDICION 2: Debo tener algún cartucho en el bolsillo.
	if ( plt_ctl_max_reintentos_reached()) {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: MAX_TRYES\r\n"));
		PLTCB.exit_code = MAX_TRYES;
		PLTCB.run_rollback = false;			// Hice el maximo esfuerzo
		PLTCB.accion_pendiente = false;		// Por ahora no queda pendiente.
		ajustar = false;
		goto quit;
	}


	// CONDICION 3: La pA siempre debe ser positiva
	if ( ! plt_ctl_limites_pA( PLTCB.pA, PA_MIN, PA_MAX ) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: PA_ERR\r\n"));
		PLTCB.exit_code = PA_ERR;
		PLTCB.run_rollback = true;			// Vuelvo al punto de partida
		PLTCB.accion_pendiente = true;
		PLTCB.f_emergencia = true;			// Por las dudas ya que perdi el sensor
		ajustar = false;
		goto quit;
	}

	// CONDICION 4: La pB siempre debe ser positiva
	if ( ! plt_ctl_limites_pB( PLTCB.pB, PB_MIN, PB_MAX ) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: PB_ERR\r\n"));
		PLTCB.exit_code = PB_ERR;
		PLTCB.run_rollback = true;			// Vuelvo al punto de partida
		PLTCB.accion_pendiente = true;
		PLTCB.f_emergencia = true;			// Por las dudas ya que perdi el sensor
		ajustar = false;
		goto quit;
	}

	// CONDICION 5: Si todo anda bien, pA debe ser mayor que pB
	if ( ! plt_ctl_pA_mayor_pB( PLTCB.pA,  PLTCB.pB) ) {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: PA_LESS_PB\r\n"));
		PLTCB.exit_code = PA_LESS_PB;
		PLTCB.run_rollback = true;			// Vuelvo al punto de partida
		PLTCB.accion_pendiente = true;
		PLTCB.f_emergencia = true;			// Por las dudas ya que perdi el sensor
		ajustar = false;
		goto quit;
	}

	// CONDICION 6: Si ya movi el motor, debe haber cambiado la presion.

	if ( ! plt_dyncontrol_pass() ) {
		// Movil el motor pero no cambio la presion.
		// Estoy en los limites del piloto.
		PLTCB.exit_code = DYNC_ERR;
		PLTCB.run_rollback = true;		// Vuelvo al punto anterior
		PLTCB.total_pulsos_rollback = PLTCB.dync_pulsos_rollback;
		PLTCB.accion_pendiente = true;
		ajustar = false;
		goto quit;
	}

	//-------------------------------------------------------------------------------
	// CONDICION 7A: AJUSTE AL ALZA
	if ( ajuste_al_alza ) {
		// Band gap:
		if ( ! plt_ctl_band_gap_suficiente( PLTCB.pA, PLTCB.pB, PLTCB.pRef) ) {
			xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: BAND_ERR\r\n"));
			PLTCB.exit_code = BAND_ERR;
			PLTCB.run_rollback = false;
			PLTCB.accion_pendiente = true;
			ajustar = false;
		}
		goto quit;
	}

	//-------------------------------------------------------------------------------
	// CONDICION 7B: AJUSTE A LA BAJA:
	if ( ! ajuste_al_alza ) {
		// Si el caudal se fue a 0, termino.
		if ( ! plt_ctl_Caudal_minimo(1.0) ) {
			PLTCB.run_rollback = false;
			PLTCB.accion_pendiente = true;
			PLTCB.exit_code = CAUDAL_CERO;
			ajustar = false;
		}
		goto quit;
	}

quit:

	if ( ajustar ) {
		if ( ajuste_al_alza ) {
			xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: Condiciones para aumentar pB OK.\r\n"));
		} else {
			xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: Condiciones para reducir pB OK.\r\n"));
		}
	} else {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXADJUST: No hay condiciones para modificar pB.!!\r\n"));
	}

	xprintf_P(PSTR("PILOTO CONDXADJUST: stop\r\n"));
	return(ajustar);

}
//------------------------------------------------------------------------------------
void plt_ajustar(void)
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
	xprintf_P(PSTR("PILOTO AJUSTAR: start\r\n"));
	// Activo el driver
	xprintf_P(PSTR("PILOTO: AJUSTAR driver pwr on\r\n"));
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

	xprintf_P(PSTR("PILOTO: AJUSTAR end\r\n"));
}
//------------------------------------------------------------------------------------
void plt_calcular_parametros_ajuste( void )
{

	/*
	 * Calcula en base a los parametros de entrada y la salida esperada, los
	 * parametros necesarios para mover el motor
	 *
	 */

float delta_pres = 0.0;

	// B) Calculo el sentido del giro
	if ( PLTCB.pB < PLTCB.pRef ) {
		// Debo aumentar pB o sxprintf_P(PSTR("PILOTO: npulses=%d\r\n"), spiloto.npulses);ea apretar el tornillo (FWD)
		PLTCB.dir = STEPPER_FWD; // Giro forward, aprieto el tornillo, aumento la presion de salida
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	// C) Intervalo de tiempo entre pulsos en ms.
	PLTCB.pwidth = piloto_conf.pWidth;

	// Ajuste de pRef por baja pA ??
	// 2. Hay margen de regulacion pero la nueva presion a alcanzar supera el limite.
	//    La ajusto al maximo permitido
	if ( (  PLTCB.pA - PGAP) <  PLTCB.pRef ) {
		// CASO 2 ( pA > pRef ) y CASO 3 ( pA < pRef )
		// La pA no es suficiente para subir todo lo requerido.
		// La nueva pA ( PLTCB.pRef ) debe dejar una banda de ajuste con pA.
		PLTCB.pRef = PLTCB.pA - PGAP;
		PLTCB.recalculo_de_pB = true;	// Indico que no tengo la presion real.
		xprintf_P(PSTR("PILOTO ADJPARAM: Recalculo (pA-pRef) < %.03f gr.!!\r\n"), PGAP );
		xprintf_P(PSTR("                 Nueva pRef=%.03f\r\n"), PLTCB.pRef );
	}
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
		xprintf_P(PSTR("PILOTO ADJPARAM: ERROR pulsos_calculados < 0\r\n"));
		PLTCB.pulsos_calculados = 0;
	}

	// E) Ajusto los pasos al 70%
	PLTCB.pulsos_a_aplicar = (int16_t)(0.7 * PLTCB.pulsos_calculados);

	// F) Controlo no avanzar mas de 500gr aprox x loop !!! ( 1 revolucion )
	if ( PLTCB.pulsos_a_aplicar > piloto_conf.pulsesXrev ) {
		PLTCB.pulsos_a_aplicar = piloto_conf.pulsesXrev;
	}

	// pulse_counts lo usa el timer callback para llevar la cuenta.
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;
}
//------------------------------------------------------------------------------------
void plt_print_parametros( void )
{
	// Muestro en pantalla los parametros calculados de ajuste de presion.

	if ( DF_APP ) {

		xprintf_P(PSTR("PILOTO: loops=%d\r\n"), PLTCB.loops );
		xprintf_P(PSTR("PILOTO: pA=%.03f\r\n"), PLTCB.pA );
		xprintf_P(PSTR("PILOTO: pB=%.03f\r\n"), PLTCB.pB );
		xprintf_P(PSTR("PILOTO: pRef=%.03f\r\n"),   PLTCB.pRef );
		xprintf_P(PSTR("PILOTO: deltaP=%.03f\r\n"), ( PLTCB.pB - PLTCB.pRef));
		xprintf_P(PSTR("PILOTO: pulses calc=%d\r\n"), PLTCB.pulsos_calculados );
		xprintf_P(PSTR("PILOTO: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );

		xprintf_P(PSTR("PILOTO: dync_pB0=%.03f\r\n"), PLTCB.dync_pB0 );
		xprintf_P(PSTR("PILOTO: dync_pulsos=%d\r\n"), PLTCB.dync_pulsos );

		xprintf_P(PSTR("PILOTO: pulses rollback=%d\r\n"), PLTCB.total_pulsos_rollback );

		xprintf_P(PSTR("PILOTO: pwidth=%d\r\n"), PLTCB.pwidth );
		if ( PLTCB.dir == STEPPER_FWD ) {
			xprintf_P(PSTR("PILOTO: dir=Forward\r\n"));
		} else {
			xprintf_P(PSTR("PILOTO: dir=Reverse\r\n"));
		}

	}
}
//------------------------------------------------------------------------------------
void plt_process_output( void )
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

	if ( PLTCB.run_rollback ) {

		plt_rollback_ajustar_piloto();
		xprintf_P(PSTR("PILOTOS: ROLLBACK\r\n"));

	}

}
/*
 *------------------------------------------------------------------------------------
 * CONTROLES GENERALES
 *------------------------------------------------------------------------------------
 */
bool plt_ctl_max_reintentos_reached(void)
{
	// Controla si alcanzamos la maxima cantidad de reintentos de ajustes de presion.

bool retS = false;

	if ( ++PLTCB.loops >=  MAX_INTENTOS ) {
		xprintf_P(PSTR("PILOTO CTL: Maxima cantidad de intentos alcanzada. Exit.\r\n"));
		retS = true;

	} else {
		xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check intentos (%d) OK.\r\n"), PLTCB.loops );
		retS = false;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
bool plt_ctl_limites_pA( float pA, float pA_min, float pA_max )
{

	// Controlo que la pA este entre los limites.

bool retS = false;

	if ( ( pA < pA_min ) || ( pA > pA_max) ) {
		xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check pA(%.03f) OUT limits (%.03f, %.03f).\r\n"),pA, pA_min, pA_max );
		retS = false;
	} else {
		xprintf_P(PSTR("PILOTO CTL: Check pA(%.03f) IN limits (%.03f, %.03f).\r\n"),pA, pA_min, pA_max );
		retS = true;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool plt_ctl_limites_pB( float pB, float pB_min, float pB_max )
{

	// Controlo que la pB este entre los limites.

bool retS = false;

	if ( ( pB < pB_min ) || ( pB > pB_max) ) {
		xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check pB(%.03f) OUT limits (%.03f, %.03f).\r\n"),pB, pB_min, pB_max );
		retS = false;
	} else {
		xprintf_P(PSTR("PILOTO CTL: Check pB(%.03f) IN limits (%.03f, %.03f).\r\n"),pB, pB_min, pB_max );
		retS = true;
	}
	return (retS);

}
//------------------------------------------------------------------------------------
bool plt_ctl_pA_mayor_pB( float pA, float pB )
{
	// pA debe ser mayor que pB
	// Puede ocurrir que baje la alta debajo de la regulacion con lo que pB se pega
	// a la alta y ambas quedan iguales.

bool retS = false;

	if ( pA > pB  ) {
		retS = true;
		xprintf_P(PSTR("PILOTO CTL: Check (pA > pB) OK\r\n") );
	} else {
		xprintf_P(PSTR("PILOTO CTL: Ajuste ERROR: ( pA < pB) .!!\r\n") );
		retS = false;
	}
	return(retS);
}
//------------------------------------------------------------------------------------
bool plt_ctl_pB_alcanzada( float pB, float pRef )
{

	// Presion alcanzada: Ultima condicion a evaluar

bool retS = false;

	if ( fabs( pB - pRef) < PERROR ) {
		xprintf_P(PSTR("PILOTO CTL: Presion alcanzada\r\n"));
		retS = true;
	} else {
		retS = false;
	}
	return(retS);

}
//------------------------------------------------------------------------------------
bool plt_ctl_Caudal_minimo( float Qmin )
{

	// Si mido caudal, este debe ser mayor a un minimo.

bool retS = false;


	if ( ! MIDO_CAUDAL() ) {
		// No mido caudal.
		xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check Caudal OK (No mido)\r\n") );
		retS = true;

	} else {
		// Mido caudal.
		if ( get_Q() < Qmin ) {
			xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check Caudal ERROR (%.03f) < 1.0\r\n"), get_Q() );
			retS = false;
		} else {
			xprintf_PD(DF_APP, PSTR("PILOTO CTL: Check Caudal OK (%.03f)\r\n"),get_Q() );
			retS = true;
		}
	}
	return(retS);

}
//------------------------------------------------------------------------------------
bool plt_ctl_band_gap_suficiente( float pA, float pB, float pRef )
{
	/*
	 * Cuando ajusto subiendo pB, debe haber una banda entre el pB y el pA para
	 * que la reguladora trabaje.
	 * Tambien puede que la pRef sea mayor a lo posible pero no por esto no subirla
	 * sino subirla todo lo posible.
	 * La banda entre pA y pB debe ser mayor a 300gr para que  trabaje la reguladora
	 *
	 * De hecho esta funcion solo la invoco cuando controlo un aumento de presion.
	 */

bool retS = true;
float margen_subida = 0.0;

	// Ajuste a la suba. Debo subir pB.

	// No hay margen de regulacion
	if ( ( pA - pB ) < PGAP ) {
		xprintf_P(PSTR("PILOTO CTL: Check bandgap ERROR: (pA-pB) < %.03f gr.!!\r\n"), PGAP );
		retS = false;
		goto quit;
	}

	// Hay margen de regulacion
	// Caso Normal.
	if ( (pA - PGAP) >= pRef ) {
		retS = true;
		xprintf_P(PSTR("PILOTO CTL: Check bandgap OK\r\n") );
		goto quit;

	} else {

		// Hay margen pero la nueva presion a alcanzar supera el limite.
		// CASO 2 ( pA > pRef ) y CASO 3 ( pA < pRef )
		// La pA no es suficiente para subir todo lo requerido.
		// Si tengo al menos un margen de 65gr, ajusto modificando el limite.

		margen_subida = ( pA - PGAP - pB );
		if ( margen_subida > PERROR ) {
			retS = true;
			xprintf_P(PSTR("PILOTO CTL: Check bandgap OK: margen=%.03f gr.!!\r\n"), margen_subida );
		} else {
			retS = false;
			xprintf_P(PSTR("PILOTO CTL: Check bandgap FAIL: margen=%.03f gr.!!\r\n"), margen_subida );
		}
		goto quit;
	}

	retS = true;

quit:

	return(retS);

}
//------------------------------------------------------------------------------------
bool plt_ctl_conditions4start( void )
{
	/*
	 * Evalua si existen las condiciones previas necesarias para iniciar el ajuste.
	 * Las precondiciones no son las mismas para ajustar a la baja que a la alta.
	 *
	 * Condiciones comunes:
	 * 1- La presion pA debe ser positiva
	 * 2- Si voy a subirla, la presion pB puede ser que sea 0. pero si debe ser positiva
	 * 3- pA debe ser mayor que pB
	 *
	 * Al Alza:
	 * 4- No importa que haya o no margen para subir pB. Esto se calcula al ajustar.
	 * 5- El caudal no importa ya que puede estar la valvula cerrada con caudal 0.
	 *
	 * A la Baja:
	 *
	 */

bool ajustar = true;
bool ajuste_al_alza = false;

	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: start\r\n"));

	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: pA=%.03f\r\n"), get_pA() );
	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: pB=%.03f\r\n"), get_pB() );
	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: pRef=%.03f\r\n"), PLTCB.pRef );
	if ( MIDO_CAUDAL() ) {
		xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: Q=%.03f\r\n"), get_Q() );
	}
	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: pError=%.03f\r\n"), PLTCB.pError );

	// Determino si ajusto al alza o a la baja.
	if ( PLTCB.pRef > get_pB() ) {
		xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: Ajuste al alza.\r\n") );
		ajuste_al_alza = true;
	} else {
		xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: Ajuste a la baja.\r\n") );
		ajuste_al_alza = false;
	}

	// Condiciones comunes (alza, baja)
	if ( ! plt_ctl_limites_pA( get_pA(), PA_MIN, PA_MAX )) {
		ajustar = false;
		goto quit;
	}

	if ( ! plt_ctl_limites_pB( get_pB(), PB_MIN, PB_MAX ) ) {
		ajustar = false;
		goto quit;
	}

	if ( ! plt_ctl_pA_mayor_pB( get_pA(), get_pB() ) ) {
		ajustar = false;
		goto quit;
	}

	//-------------------------------------------------------------------------------
	// AJUSTE AL ALZA
	if ( ajuste_al_alza ) {
		// Band gap:
		// 1. No hay margen de regulacion
		if ( ! plt_ctl_band_gap_suficiente(get_pA(), get_pB(), PLTCB.pRef )) {
			//xprintf_P(PSTR("PILOTO: Check bandgap ERROR: (pA-pB) < %.02f gr.!!\r\n"), DELTA_PA_PB );
			ajustar = false;
		}
		goto quit;
	}

	//-------------------------------------------------------------------------------
	// AJUSTE A LA BAJA:
	if ( ! ajuste_al_alza ) {

		if ( ! plt_ctl_Caudal_minimo(1.0) ) {
			ajustar = false;
		}
		goto quit;
	}

quit:

	if ( ajustar ) {
		if ( ajuste_al_alza ) {
			xprintf_PD( DF_APP, PSTR("PILOTO CONDXSTART: PRE-Condiciones para aumentar pB OK.\r\n"));
		} else {
			xprintf_PD( DF_APP, PSTR("PILOTO CONDXSTART: PRE-Condiciones para reducir pB OK.\r\n"));
		}
	} else {
		xprintf_PD( DF_APP, PSTR("PILOTO CONDXSTART: PRE-Condiciones. No se modifica pB.!!\r\n"));
	}

	xprintf_PD(DF_APP, PSTR("PILOTO CONDXSTART: end.\r\n"));
	return(ajustar);
}
//------------------------------------------------------------------------------------
/*
 * -----------------------------------------------------------------------------------
 * Funciones generales
 * -----------------------------------------------------------------------------------
 */

void plt_pulse_TimerCallback( TimerHandle_t xTimer )
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
bool plt_leer_slot_actual( int8_t *slot_id )
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
//-------------------------------------------------------------------------------------
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
/*
 *------------------------------------------------------------------------------------
 * ROLLBACK
 * Se lleva un control de los pulsos aplicados en el ciclo para que si es necesario
 * volver atrás, al punto inicial del arranque
 *------------------------------------------------------------------------------------
 */
void plt_rollback_init(void)
{
	// Cuando arranca un ciclo de mover el piloto, inicializo
	PLTCB.run_rollback = false;
	PLTCB.total_pulsos_rollback = 0;
}
/*------------------------------------------------------------------------------------*/
void plt_rollback_update(void)
{
	/*
	 *  Cada vez que voy a mover el piloto, actualizo el rollback
	 *  Los pulsos de rollback tienen sentido contrario.
	 */

	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.total_pulsos_rollback -= PLTCB.pulse_counts;
	} else {
		PLTCB.total_pulsos_rollback += PLTCB.pulse_counts;
	}
}
/*------------------------------------------------------------------------------------*/
void plt_rollback_ajustar_piloto( void )
{
	/*
	 * Aplico los pulsos de rollback para llevar el piloto a la posicion inicial
	 * del ciclo o al punto anterior ( dyncontrol ).
	 * No controlo condiciones ni presiones.
	 * Los pulsos de rollback pueden tener signo.
	 *
	 *
	 */

	xprintf_P( PSTR("PILOTO ROLLBACK: Start\r\n"));
	u_wdg_kick( plt_app_wdg,  240 );

	if ( PLTCB.total_pulsos_rollback > 0 ) {
	//if ( PLTCB.dync_pulsos_rollback > 0 ) {
		PLTCB.dir = STEPPER_FWD;
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	PLTCB.pulsos_a_aplicar = abs(PLTCB.total_pulsos_rollback);
	//PLTCB.pulsos_a_aplicar = abs(PLTCB.dync_pulsos_rollback);
	PLTCB.pwidth = piloto_conf.pWidth;
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;

	// Print datos.
	xprintf_P(PSTR("PILOTO ROLLBACK: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO ROLLBACK: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO ROLLBACK: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO ROLLBACK: dir=Reverse\r\n"));
	}

	if ( PLTCB.pulsos_a_aplicar == 0 ) {
		return;
	}

	// Activo el driver
	xprintf_P(PSTR("PILOTO ROLLBACK: stepper driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("PPILOTO ROLLBACK: stepper driver pulses start..\r\n"));
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
	xprintf_P( PSTR("PILOTO ROLLBACK: Stop\r\n"));
}
/*
 *------------------------------------------------------------------------------------
 * DYNCONTROL
 * Se controlan los pulsos ( con signo ) que se aplican.
 * Si se dió una revolución y la presion no vario, se considera que algo esta mal
 * y se abora la operacion.
 *------------------------------------------------------------------------------------
 */
void plt_dyncontrol_init(void)
{
	// Inicializa las variables del control dinamico

	PLTCB.dync_pB0 = PLTCB.pB;
	PLTCB.dync_pulsos = 0;
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: init\r\n"));
}
/*------------------------------------------------------------------------------------*/
void plt_dyncontrol_update(void)
{

	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: update\r\n"));
	if ( PLTCB.dir == STEPPER_FWD) {
		PLTCB.dync_pulsos += PLTCB.pulse_counts;
	} else {
		PLTCB.dync_pulsos -= PLTCB.pulse_counts;
	}

	// Pulsos de rollback para volver al punto anterior ( no al inicial )
	PLTCB.dync_pulsos_rollback = PLTCB.dync_pulsos;
}
/*------------------------------------------------------------------------------------*/
bool plt_dyncontrol_pass(void)
{
	/*
	 *  Movi el piloto algo mas de 1 vuelta y la pB no se modifico en al menos 200 gr. ( Rollback )
	 *  El piloto se esta moviendo pero la pB no esta ajustando.
	 *  Si di mas de una revolucion, la presion debe haber cambiado al menos 200grs.
	 *  Si cambio, renicio el control dinamico al nuevo punto
	 *
	 *  Agregamos controlar el cociente DpresB / Dpulsos.
	 *  Dado que el resorte es lineal, debemos movernos en cierto rango. Si no se cumple, es error.
	 *
	 */

float sigma = 0.0;
float limite_sup, limite_inf;

	// Cuando arranco no hago el control dinamico. Da error porque esta inicializado en 0
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: loops=%d\r\n"), PLTCB.loops );

	if ( PLTCB.loops == 1 ) {
		return(true);
	}

	// dync_pulsos tiene signo !!
	if ( abs(PLTCB.dync_pulsos) > 1 ) {
		sigma = fabs( ( PLTCB.dync_pB0 - PLTCB.pB) * 1000 / PLTCB.dync_pulsos ) ;
	} else {
		xprintf_P( PSTR("PILOTO DYNCONTROL: Check ERROR:\r\n"));
		return(false);
	}

	// Limite inferior
	limite_inf = fabs( 0.2 * DPRES_X_REV * 1000 / piloto_conf.pulsesXrev );
	// Limite superior
	limite_sup = fabs( 1.5 * DPRES_X_REV * 1000 / piloto_conf.pulsesXrev );

	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: pxrev=%d\r\n"), piloto_conf.pulsesXrev );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: dync_pulsos=%d\r\n"), PLTCB.dync_pulsos );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: dync_pulsos_rollback=%d\r\n"), PLTCB.dync_pulsos_rollback );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: pB0=%.03f\r\n"), PLTCB.dync_pB0 );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: pB=%.03f\r\n"), PLTCB.pB );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: sigma=%.03f\r\n"), sigma );
	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: Linf=%.03f,Lsup=%.03f\r\n"), limite_inf, limite_sup );

	if ( sigma < limite_inf ) {
		xprintf_P( PSTR("PILOTO DYNCONTROL: ERROR baja pendiente (%f < [%f]!!.\r\n"), sigma, limite_inf);
		return(false);
	}

	if ( sigma > limite_sup ) {
		xprintf_P( PSTR("PILOTO DYNCONTROL: ERROR alta pendiente (%f > [%f]!!.\r\n"), sigma, limite_sup);
		return(false);
	}

	// Giro y cambio la presion. Está funcionando.
	// Actualizo los parametros del dyn_control.
	plt_dyncontrol_init();

	/*
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
			plt_dyncontrol_init();
		}
	}
	*/


	xprintf_PD( DF_APP, PSTR("PILOTO DYNCONTROL: OK\r\n"));

	return(true);
}
/*
 * -----------------------------------------------------------------------------------
 * Funciones de test
 * -----------------------------------------------------------------------------------
 */
void piloto_run_stepper_test(char *s_dir, char *s_npulses, char *s_pwidth )
{
	// Funcion invocada desde cmdline.
	// No hacemos un Wrapper de 'piloto_run_stepper' porque solo quiero
	// mover el stepper sin necesidad de ver presiones o condiciones.

int16_t npulses;
int8_t pwidth;

	if ( *s_npulses == '\0' ) {
		xprintf_P(PSTR("steppertest ERROR: debe indicar npulses\r\n"));
		return;
	}
	npulses = atoi(s_npulses);

	if ( strcmp_P( strupr(s_dir), PSTR("FW")) == 0) {
		npulses *= 1;
	} else if ( strcmp_P( strupr(s_dir), PSTR("REV")) == 0) {
		npulses *= -1;
	} else {
		xprintf_P(PSTR("Error en direccion\r\n"));
		return;
	}

	pwidth = atoi(s_pwidth);
	if ( *s_pwidth == '\0' ) {
		xprintf_P(PSTR("steppertest ERROR: debe indicar pwidth\r\n"));
		return;
	}

	/*
	 * Mueve el stepper.
	 */

	// Completo PLT_CB
	PLTCB.pwidth = pwidth;
	PLTCB.pulsos_a_aplicar = abs(npulses);
	PLTCB.pulse_counts = PLTCB.pulsos_a_aplicar;
	if ( npulses > 0 ) {
		PLTCB.dir = STEPPER_FWD;
	} else {
		PLTCB.dir = STEPPER_REV;
	}

	// Print datos.
	xprintf_P(PSTR("PILOTO STEPPERTEST:\r\n"));
	xprintf_P(PSTR("PILOTO STEPPERTEST: pulses apply=%d\r\n"), PLTCB.pulsos_a_aplicar );
	xprintf_P(PSTR("PILOTO STEPPERTEST: pwidth=%d\r\n"), PLTCB.pwidth );
	if ( PLTCB.dir == STEPPER_FWD ) {
		xprintf_P(PSTR("PILOTO STEPPERTEST: dir=Forward\r\n"));
	} else {
		xprintf_P(PSTR("PILOTO STEPPERTEST: dir=Reverse\r\n"));
	}

	// Activo el driver
	xprintf_P(PSTR("PILOTO STEPPERTEST: driver pwr on\r\n"));
	stepper_pwr_on();
	vTaskDelay( ( TickType_t)( 20000 / portTICK_RATE_MS ) );

	// Arranca el timer que por callbacks va a generar los pulsos
	PLTCB.motor_running = true;
	xprintf_P(PSTR("PILOTO STEPPERTEST: driver pulses start..\r\n"));
	stepper_awake();
	stepper_start();

	xTimerChangePeriod(plt_pulse_xTimer, ( PLTCB.pwidth * 2) / portTICK_PERIOD_MS , 10 );
	xTimerStart( plt_pulse_xTimer, 10 );

	// Espero que termine de mover el motor. EL callback pone motor_running en false. !!
	while ( PLTCB.motor_running ) {
		vTaskDelay( ( TickType_t) (1000 / portTICK_RATE_MS ) );
	}

	xprintf_P(PSTR("PILOTO STEPPERTEST: running.\r\n"));
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
			plt_pulse_TimerCallback,
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
void piloto_read_ring_buffer(void)
{
int8_t i;

	xprintf_P( PSTR("RB: Count=%d \r\n"),ringBuffer_GetCount(&pFIFO) );
	xprintf_P( PSTR("RB: FreeCount=%d \r\n"),ringBuffer_GetFreeCount(&pFIFO) );
	for (i=0; i < PFIFO_STORAGE_SIZE; i++) {
		xprintf_P( PSTR("RB: [%d] tipo_tarea=%d, valor=%0.3f\r\n"), i, pFifo_storage[i].tipo, pFifo_storage[i].valor);
	}
}
//------------------------------------------------------------------------------------

