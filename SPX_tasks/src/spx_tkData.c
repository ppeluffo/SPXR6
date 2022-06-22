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
#include "ul_oceanus.h"

st_dataRecord_t dataRecd;
float battery;
uint32_t ticks;

static void pv_check_inputs_conf(void);


//------------------------------------------------------------------------------------
// PROTOTIPOS
static void pv_data_guardar_en_BD(void);
//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime = 0;
uint32_t waiting_ticks = 0;

	while (!run_tasks)
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P( PSTR("starting tkData..\r\n\0"));

	ainputs_init();
	dinputs_init();
	counters_init();

	// Initialise the xLastWakeTime variable with the current time.
 	xLastWakeTime = xTaskGetTickCount();

 	// Al arrancar poleo a los 10s
 	waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

 	// Inicializo la cola de modbus
 	modbus_init_output_cmds_queue();

	// Inicializo las variables de intercambio con las aplicaciones.
	pv_check_inputs_conf();

  	// loop
  	for( ;; ) {

  		//print_running_ticks("SYS: MAIN");
  		u_wdg_kick( WDG_DATA, ( systemVars.timerPoll + 120 ));

  		// Espero. Da el tiempo necesario para entrar en tickless.
  		vTaskDelayUntil( &xLastWakeTime, waiting_ticks );

  		// Proceso modbus outputs
  		modbus_dequeue_output_cmd();

  		// Poleo
  		memset( &dataRecd,'\0',sizeof(dataRecd));
 		data_read_inputs(&dataRecd, false);
  		data_print_inputs(fdTERM, &dataRecd, 0);
  		pv_data_guardar_en_BD();

  		// Aviso a tkGprs que hay un frame listo. En modo continuo lo va a trasmitir enseguida.
  		if ( ! MODO_DISCRETO ) {
 			SPX_SEND_SIGNAL( SGN_FRAME_READY );
  		}

  		SPX_SEND_SIGNAL( SGN_DOSIFICADORA );

    		// Calculo el tiempo para una nueva espera
  		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
  			//taskYIELD();
  			vTaskDelay( ( TickType_t)( 1 ) );

 		// timpo real que voy a dormir esta tarea
 		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
 		// tiempo similar que voy a decrementar y mostrar en consola.
 		ctl_reload_timerPoll(systemVars.timerPoll);

 		xSemaphoreGive( sem_SYSVars );

 		//xprintf_PD( DF_COMMS, PSTR("DATA: await loop\r\n\0"));
  	}

}
//------------------------------------------------------------------------------------
void data_read_inputs(st_dataRecord_t *dst, bool f_copy )
{

bool status_flag;


	//xprintf_PD( DF_COMMS, PSTR("DATA: data_read_inputs in\r\n\0"));

	// Solo copio el buffer. No poleo.
	if ( f_copy ) {
		memcpy(dst, &dataRecd, sizeof(dataRecd));
		return;
	}

	// Poleo.
	ainputs_read( dst->ainputs, &dst->battery, DF_DATA );
	dinputs_read( dst->dinputs );
	counters_read( dst->counters );
	counters_clear();

	// Los canales de oceanus y modbus son los mismos. Leo uno o la otra.
	if ( ! oceanus_read (dst->modbus ) ) {
		modbus_read (dst->modbus );
	}

	// Agrego el timestamp
	status_flag = RTC_read_dtime( &dst->rtc );
	if ( ! status_flag )
		xprintf_P(PSTR("ERROR: I2C:RTC:data_read_inputs\r\n\0"));

	// Guardo pA,pB,Q forma persistente porque lo puedo requierir por la alguna aplicacion.
	if ( ctlapp_vars.pA_channel != -1 ) {
		ctlapp_vars.pA = dst->ainputs[ctlapp_vars.pA_channel];
	} else {
		ctlapp_vars.pA = 0.0;
	}

	if ( ctlapp_vars.pB_channel != -1 ) {
		ctlapp_vars.pB = dst->ainputs[ctlapp_vars.pB_channel];
	} else {
		ctlapp_vars.pB = 0.0;
	}

	switch ( ctlapp_vars.Q_module ) {
	case NONE:
		ctlapp_vars.caudal = 0.0;
		break;
	case MODBUS:
		ctlapp_vars.caudal = dst->modbus[ctlapp_vars.Q_channel];
		break;
	case ANALOG:
		ctlapp_vars.caudal = dst->ainputs[ctlapp_vars.Q_channel];
		break;
	case COUNTER:
		ctlapp_vars.caudal = dst->counters[ctlapp_vars.Q_channel];
		break;
	default:
		ctlapp_vars.caudal = 0.0;
	}

	//xprintf_PD( DF_COMMS, PSTR("DATA: data_read_inputs out\r\n\0"));
}
//------------------------------------------------------------------------------------
void data_print_inputs(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl)
{

	//xprintf_PD( DF_COMMS, PSTR("DATA: data_print_inputs in\r\n\0"));

	// timeStamp.
	xfprintf_P(fd, PSTR("CTL:%d;DATE:%02d"),ctl, dr->rtc.year );
	xfprintf_P(fd, PSTR("%02d%02d;"),dr->rtc.month, dr->rtc.day );

	xfprintf_P(fd, PSTR("TIME:%02d"), dr->rtc.hour );
	xfprintf_P(fd, PSTR("%02d%02d;"), dr->rtc.min, dr->rtc.sec );

	ainputs_print( fd, dr->ainputs );
	dinputs_print( fd, dr->dinputs );
	counters_print( fd, dr->counters );

	modbus_data_print( fd, dr->modbus );

	ainputs_battery_print( fd, dr->battery );

	// TAIL
	// Esto es porque en gprs si mando un cr corto el socket !!!
	if ( fd == fdTERM ) {
		xfprintf_P(fd, PSTR("\r\n\0") );
	}

	//xprintf_PD( DF_COMMS, PSTR("DATA: data_print_inputs out\r\n\0"));
}
//------------------------------------------------------------------------------------
int16_t data_sprintf_inputs( char *sbuffer ,st_dataRecord_t *dr, uint16_t ctl )
{

int16_t i = 0;
char *p;

	// timeStamp.
	p = sbuffer;
	i = sprintf_P( p, PSTR("CTL:%d;DATE:%02d"),ctl, dr->rtc.year );
	p += i;
	i = sprintf_P( p, PSTR("%02d%02d;"),dr->rtc.month, dr->rtc.day );
	p += i;

	i = sprintf_P( p, PSTR("TIME:%02d"), dr->rtc.hour );
	p += i;
	i = sprintf_P( p, PSTR("%02d%02d;"), dr->rtc.min, dr->rtc.sec );
	p += i;

	p = ainputs_sprintf( p, dr->ainputs );
	p = dinputs_sprintf( p, dr->dinputs );
	p = counters_sprintf( p, dr->counters );

	p = modbus_sprintf( p, dr->modbus );

	p = ainputs_battery_sprintf(p, dr->battery );

	return(i);
}
//------------------------------------------------------------------------------------
int16_t data_sprintf_actual_inputs( char *sbuffer)
{

int16_t i = 0;
char *p;

	// timeStamp.
	p = sbuffer;
	i = sprintf_P( p, PSTR("DATE:%02d"),dataRecd.rtc.year );
	p += i;
	i = sprintf_P( p, PSTR("%02d%02d;"),dataRecd.rtc.month, dataRecd.rtc.day );
	p += i;

	i = sprintf_P( p, PSTR("TIME:%02d"), dataRecd.rtc.hour );
	p += i;
	i = sprintf_P( p, PSTR("%02d%02d;"), dataRecd.rtc.min, dataRecd.rtc.sec );
	p += i;

	p = ainputs_sprintf( p, dataRecd.ainputs );
	p = dinputs_sprintf( p, dataRecd.dinputs );
	p = counters_sprintf( p, dataRecd.counters );

	p = modbus_sprintf( p, dataRecd.modbus );

	p = ainputs_battery_sprintf(p, dataRecd.battery );
	p++;
	*p = '\0';

	return(i);
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
static void pv_check_inputs_conf(void)
{
	/*
	 *  Se fija si en la configuracion del equipo hay algun canal con el
	 *  nombre pA y pB.
	 *  Tambien determino si mide o no caudal y en que canal
	 */

uint8_t i;
char lname[PARAMNAME_LENGTH];


	// Pa, Pb
	ctlapp_vars.pA_channel = -1;
	ctlapp_vars.pB_channel = -1;

	for ( i = 0; i < ANALOG_CHANNELS; i++) {
		strncpy(lname, ainputs_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ! strcmp_P( lname, PSTR("PA") ) ) {
			ctlapp_vars.pA_channel = i;

		}
		if ( ! strcmp_P( lname, PSTR("PB") ) ) {
			ctlapp_vars.pB_channel = i;

		}
	};
	xprintf_P(PSTR("pA channel=%d\r\n"), ctlapp_vars.pA_channel);
	xprintf_P(PSTR("pB channel=%d\r\n"), ctlapp_vars.pB_channel);

	// CAUDAL
	ctlapp_vars.Q_module = NONE;
	ctlapp_vars.Q_channel = -1;

	// Canales analogicos
	for (i=0; i<ANALOG_CHANNELS; i++) {

		strncpy(lname, ainputs_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			ctlapp_vars.Q_module = ANALOG;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			ctlapp_vars.Q_module = ANALOG;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}
	}

	// Canales contadores
	for (i=0; i<COUNTER_CHANNELS; i++) {

		strncpy(lname, counters_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			ctlapp_vars.Q_module = COUNTER;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			ctlapp_vars.Q_module = COUNTER;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}
	}

	// Canales modbus
	for (i=0; i<MODBUS_CHANNELS; i++) {

		strncpy(lname, modbus_conf.channel[i].name, PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			ctlapp_vars.Q_module = MODBUS;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}

		if ( strstr ( lname, "CAU" ) ) {
			ctlapp_vars.Q_module = MODBUS;
			ctlapp_vars.Q_channel = i;
			goto quit;
		}
	}

quit:

	if ( ctlapp_vars.Q_module == MODBUS ) {
		xprintf_P(PSTR("CAUDAL: Canal %d MODBUS\r\n"), ctlapp_vars.Q_channel );
	} else if ( ctlapp_vars.Q_module == ANALOG ) {
		xprintf_P(PSTR("CAUDAL: Canal %d ANALOG\r\n"), ctlapp_vars.Q_channel );
	} else 	if ( ctlapp_vars.Q_module == COUNTER ) {
		xprintf_P(PSTR("CAUDAL: Canal %d COUNTER\r\n"), ctlapp_vars.Q_channel );
	} else {
		xprintf_P(PSTR("CAUDAL: No hay canales configurados\r\n"));
	}

}
//------------------------------------------------------------------------------------
