/*
 * spx_tkData.c
 *
 *  Created on: 11 dic. 2018
 *      Author: pablo
 *
 *  Esta tarea se encarga de leer los canales analogicos y luego armar un frame de datos
 *  con los datos de los contadores y entradas digitales.
 *  Luego los muestra en pantalla y los guarda en memoria.
 *  Todo esta pautado por el timerPoll.
 *  Usamos una estructura unica global para leer los datos.
 *  Cuando usamos el comando de 'read frame' desde el cmdMode, escribimos sobre la misma
 *  estructura de datos por lo cual si coincide con el momento de poleo se pueden
 *  llegar a sobreescribir.
 *  Esto en realidad no seria importante ya que en modo cmd esta el operador.
 *
 */

#include "spx.h"
#include "tkComms.h"

st_dataRecord_t dataRecd;
float battery;
uint16_t data_wdg;

//------------------------------------------------------------------------------------
// PROTOTIPOS
static void pv_data_guardar_en_BD(void);
static void pv_data_read_inputs_normal(st_dataRecord_t *dst, bool f_copy );
static void pv_data_print_inputs_normal(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl);
//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime = 0;
uint32_t waiting_ticks = 0;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	ainputs_init();
	dinputs_init();
	counters_init();

	u_wdg_register( WDG_DATA, &data_wdg );

	xprintf_P( PSTR("starting tkData..\r\n\0"));

	// Initialise the xLastWakeTime variable with the current time.
 	 xLastWakeTime = xTaskGetTickCount();

 	 // Al arrancar poleo a los 10s
 	 waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

  	// loop
  	for( ;; ) {

  		u_wdg_kick(&data_wdg, ( systemVars.timerPoll + 60 ));

  		// Espero. Da el tiempo necesario para entrar en tickless.
  		vTaskDelayUntil( &xLastWakeTime, waiting_ticks );

  		// Poleo
  		memset( &dataRecd,'\0',sizeof(dataRecd));
 		data_read_inputs(&dataRecd, false);
  		data_print_inputs(fdTERM, &dataRecd, 0);
  		pv_data_guardar_en_BD();

  		// Aviso a tkGprs que hay un frame listo. En modo continuo lo va a trasmitir enseguida.
  		if ( ! MODO_DISCRETO ) {
 			SPX_SEND_SIGNAL( SGN_FRAME_READY );
  		}

    		// Calculo el tiempo para una nueva espera
  		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
  			taskYIELD();

 		// timpo real que voy a dormir esta tarea
 		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
 		// tiempo similar que voy a decrementar y mostrar en consola.
 		ctl_reload_timerPoll(systemVars.timerPoll);

 		xSemaphoreGive( sem_SYSVars );

  	}

}
//------------------------------------------------------------------------------------
void data_read_inputs(st_dataRecord_t *dst, bool f_copy )
{
	pv_data_read_inputs_normal( dst, f_copy );
}
//------------------------------------------------------------------------------------
static void pv_data_read_inputs_normal(st_dataRecord_t *dst, bool f_copy )
{
	// Leo las entradas digitales sobre la estructura local din.
	// En los IO5 el sensor de temperatura y presion actuan juntos !!!!.

int8_t xBytes = 0;

	// Solo copio el buffer. No poleo.
	if ( f_copy ) {
		memcpy(dst, &dataRecd, sizeof(dataRecd));
		return;
	}

	// Poleo.
	ainputs_read( dst->df.io.ainputs, &dst->df.io.battery );
	dinputs_read( dst->df.io.dinputs );
	counters_read( dst->df.io.counters );
	counters_clear();

	// Agrego el timestamp
	xBytes = RTC_read_dtime( &dst->rtc );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:RTC:data_read_inputs\r\n\0"));


}
//------------------------------------------------------------------------------------
void data_print_inputs(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl )
{
	pv_data_print_inputs_normal( fd, dr, ctl );

}
//------------------------------------------------------------------------------------
static void pv_data_print_inputs_normal(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl )
{

	// timeStamp.
	xfprintf_P(fd, PSTR("CTL:%d;DATE:%02d"),ctl, dr->rtc.year );
	xfprintf_P(fd, PSTR("%02d%02d;"),dr->rtc.month, dr->rtc.day );

	xfprintf_P(fd, PSTR("TIME:%02d"), dr->rtc.hour );
	xfprintf_P(fd, PSTR("%02d%02d;"), dr->rtc.min, dr->rtc.sec );

	ainputs_print( fd, dr->df.io.ainputs );
	dinputs_print( fd, dr->df.io.dinputs );
	counters_print( fd, dr->df.io.counters );

	ainputs_battery_print( fd, dr->df.io.battery );

	// TAIL
	// Esto es porque en gprs si mando un cr corto el socket !!!
	if ( fd == fdTERM ) {
		xfprintf_P(fd, PSTR("\r\n\0") );
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_data_guardar_en_BD(void)
{

int8_t bytes_written = 0;
static bool primer_frame = true;
FAT_t fat;

	memset( &fat, '\0', sizeof(FAT_t));

	// Para no incorporar el error de los contadores en el primer frame no lo guardo.
	if ( primer_frame ) {
		primer_frame = false;
		return;
	}

	// Guardo en BD
	bytes_written = FF_writeRcd( &dataRecd, sizeof(st_dataRecord_t) );

	if ( bytes_written == -1 ) {
		// Error de escritura o memoria llena ??
		xprintf_P(PSTR("DATA: WR ERROR (%d)\r\n\0"),FF_errno() );
		// Stats de memoria
		FAT_read(&fat);
		xprintf_P( PSTR("DATA: MEM [wr=%d,rd=%d,del=%d]\0"), fat.wrPTR,fat.rdPTR,fat.delPTR );
	}

}
//------------------------------------------------------------------------------------
