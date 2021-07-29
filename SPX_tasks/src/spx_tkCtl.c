/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 *
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_ctl_init_system(void);
static void pv_ctl_wink_led(void);
static void pv_ctl_check_wdg(void);
static void pv_ctl_ticks(void);
static void pv_ctl_daily_reset(void);
static void pv_ctl_check_terminal_present(void);
static void pv_ctl_fire_counters(void);

#define MAX_TIMERS	2
#define TIME_TO_NEXT_POLL	0
#define TIME_TO_NEXT_DIAL	1

static uint32_t pv_timers[MAX_TIMERS];

static bool f_terminal_connected = false;

// Timpo que espera la tkControl entre round-a-robin
#define TKCTL_DELAY_S	5

// La tarea pasa por el mismo lugar c/5s.
uint16_t ctl_wdg;

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	pv_ctl_init_system();
	u_wdg_register( WDG_CTL, &ctl_wdg );

	xprintf_P( PSTR("\r\nstarting tkControl..\r\n\0"));

	for( ;; )
	{

		// Paso c/5s plt 30s es suficiente.
		u_wdg_kick(&ctl_wdg, 30);

		// Cada 5s controlo el watchdog y los timers.
		pv_ctl_check_wdg();
		pv_ctl_ticks();
		pv_ctl_check_terminal_present();
		pv_ctl_wink_led();
		pv_ctl_daily_reset();
		pv_ctl_fire_counters();

		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( TKCTL_DELAY_S * 1000 / portTICK_RATE_MS ) );

	}
}
//------------------------------------------------------------------------------------
static void pv_ctl_init_system(void)
{

	// Esta funcion corre cuando el RTOS esta inicializado y corriendo lo que nos
	// permite usar las funciones del rtos-io y rtos en las inicializaciones.
	//

uint8_t wdg = 0;
FAT_t l_fat;
uint16_t recSize = 0;
uint8_t i;

	memset( &l_fat, '\0', sizeof(FAT_t));

	// Deteccion inicial de la termial conectada o no.
	f_terminal_connected = false;
	if (  IO_read_TERMCTL_PIN() == 1 ) {
		f_terminal_connected = true;
	}

	// Al comienzo leo este handle para asi usarlo para leer el estado de los stacks.
	// En la medida que no estoy usando la taskIdle podria deshabilitarla. !!!
	xHandle_idle = xTaskGetIdleTaskHandle();

	// Inicializo todos los watchdogs a 30s ( 3 * 5s de loop )
	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		watchdog_timers[wdg] = (uint16_t)( TKCTL_DELAY_S * 6 );
	}

	// Inicializo los timers
	for ( i = 0; i < MAX_TIMERS; i++ ) {
		pv_timers[i] = 0;
	}

	// Luego del posible error del bus I2C espero para que se reponga !!!
	vTaskDelay( ( TickType_t)( 100 ) );

	// Inicializo las señales
	SPX_CLEAR_SIGNAL( SGN_MON_SQE );
	SPX_CLEAR_SIGNAL( SGN_REDIAL );
	SPX_CLEAR_SIGNAL( SGN_FRAME_READY );


	// Leo los parametros del la EE y si tengo error, cargo por defecto
	if ( ! u_load_params_from_NVMEE() ) {
		u_load_defaults( NULL );
		u_save_params_in_NVMEE();
		xprintf_P( PSTR("\r\nLoading defaults !!\r\n\0"));
	}

	// Inicializo el timer de poleo
	pv_timers[TIME_TO_NEXT_POLL] = systemVars.timerPoll;

	// Inicializo la memoria EE ( fileSysyem)
	if ( FF_open() ) {
		xprintf_P( PSTR("FSInit OK\r\n\0"));
	} else {
		FF_format(false );	// Reformateo soft.( solo la FAT )
		xprintf_P( PSTR("FSInit FAIL !!.Reformatted...\r\n\0"));
	}

	FAT_read(&l_fat);
	xprintf_P( PSTR("MEMsize=%d,wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"),FF_MAX_RCDS, l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

	// Imprimo el tamanio de registro de memoria
	recSize = sizeof(st_dataRecord_t);
	xprintf_P( PSTR("RCD size %d bytes.\r\n\0"),recSize);

	// Arranco el RTC. Si hay un problema lo inicializo.
	RTC_init();

	xprintf_P( PSTR("SPX: DINPUTS=%d, COUNTERS=%d, ANINPUTS=%d\r\n\0"), DINPUTS_CHANNELS, COUNTER_CHANNELS, ANALOG_CHANNELS );

	xprintf_P( PSTR("------------------------------------------------\r\n\0"));

	// Habilito a arrancar al resto de las tareas
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_ctl_check_terminal_present(void)
{
	// Lee el pin de la terminal para ver si hay o no una conectada.
	// Si bien en la IO8 no es necesario desconectar la terminal ya que opera
	// con corriente, por simplicidad uso un solo codigo para ambas arquitecturas.

	if ( IO_read_TERMCTL_PIN() == 1) {
		f_terminal_connected = true;
	} else {
		f_terminal_connected = false;
	}
	return;
}
//------------------------------------------------------------------------------------
static void pv_ctl_wink_led(void)
{

	//xprintf_P( PSTR("DEBUG Led..\r\n\0"));
	// SI la terminal esta desconectada salgo.
	if ( ! ctl_terminal_connected() )
		return;

	// Prendo los leds
	IO_set_LED_KA();
	vTaskDelay( ( TickType_t)( 10 ) );

	// Apago
	IO_clr_LED_KA();
	IO_clr_LED_COMMS();

}
//------------------------------------------------------------------------------------
static void pv_ctl_check_wdg(void)
{
	// Cada tarea periodicamente reinicia su wdg timer.
	// Esta tarea los decrementa cada 5 segundos.
	// Si alguno llego a 0 es que la tarea se colgo y entonces se reinicia el sistema.

uint8_t wdg = 0;

		// Cada ciclo reseteo el wdg para que no expire.
		WDT_Reset();
		return;

		// Si algun WDG no se borro, me reseteo
		while ( xSemaphoreTake( sem_WDGS, ( TickType_t ) 5 ) != pdTRUE )
			taskYIELD();

		for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {

			if ( watchdog_timers[wdg] > TKCTL_DELAY_S ) {
				watchdog_timers[wdg] -= TKCTL_DELAY_S;
			} else {
				watchdog_timers[wdg] = 0;
			}

			if ( watchdog_timers[wdg] == 0 ) {
				// Me reseteo por watchdog
				while(1)
				  ;
				//CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */

			}
		}

		xSemaphoreGive( sem_WDGS );
}
//------------------------------------------------------------------------------------
static void pv_ctl_ticks(void)
{
uint8_t i = 0;

	//xprintf_P( PSTR("DEBUG ctlTicks..\r\n\0"));

	// Ajusto los timers hasta llegar a 0.

	for ( i = 0; i < MAX_TIMERS; i++ ) {

		if ( pv_timers[i] > TKCTL_DELAY_S ) {
			pv_timers[i] -= TKCTL_DELAY_S;
		} else {
			pv_timers[i] = 0;
		}
	}
}
//------------------------------------------------------------------------------------
static void pv_ctl_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.
	// Se invoca 1 vez por minuto ( 60s ).

static uint32_t ticks_to_reset = 86400 / TKCTL_DELAY_S ; // ticks en 1 dia.

	//xprintf_P( PSTR("DEBUG dailyReset..\r\n\0"));
	while ( --ticks_to_reset > 0 ) {
		return;
	}

	xprintf_P( PSTR("Daily Reset !!\r\n\0") );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void pv_ctl_fire_counters(void)
{
static int8_t time_to_fire_counters = 6;

	if ( time_to_fire_counters == 0 )
		return;

	if ( --time_to_fire_counters ==  0 ) {
		counters_run();
		xprintf_P( PSTR("COUNTERS Enabled.\r\n\0") );
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs )
{
	// Reinicia el watchdog de la tarea taskwdg con el valor timeout.
	// timeout es uint16_t por lo tanto su maximo valor en segundos es de 65536 ( 18hs )

	while ( xSemaphoreTake( sem_WDGS, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	watchdog_timers[taskWdg] = timeout_in_secs;

	xSemaphoreGive( sem_WDGS );
}
//------------------------------------------------------------------------------------
bool ctl_terminal_connected(void)
{
	return(f_terminal_connected);
}
//------------------------------------------------------------------------------------
uint16_t ctl_readTimeToNextPoll(void)
{
	return( (uint16_t) pv_timers[TIME_TO_NEXT_POLL] );
}
//------------------------------------------------------------------------------------
void ctl_reload_timerPoll( uint16_t new_time )
{
	// Como trabajo en modo tickless, no puedo estar cada 1s despertandome solo para
	// ajustar el timerpoll.
	// La alternativa es en tkData dormir todo lo necesario y al recargar el timerpoll,
	// en tkCtl setear una variable con el mismo valor e irla decrementandola c/5 s solo
	// a efectos de visualizarla.
	pv_timers[TIME_TO_NEXT_POLL] = new_time;
}
//------------------------------------------------------------------------------------
uint32_t ctl_read_timeToNextDial(void)
{
	return( pv_timers[TIME_TO_NEXT_DIAL] );
}
//------------------------------------------------------------------------------------
void ctl_set_timeToNextDial( uint32_t new_time )
{
	pv_timers[TIME_TO_NEXT_DIAL] = new_time;
}
//------------------------------------------------------------------------------------
