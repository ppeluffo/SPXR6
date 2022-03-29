/*
 * spx_counters.c
 *
 *  Created on: 5 jun. 2019
 *      Author: pablo
 *
 *  Entradas contadores tipo B:
 *  Son las entradas que tienen optoacoplador.
 *  Normalmente con la entrada flotando ( 12V ), el transistor del opto esta cortado y la entrada al micro esta en 0V.
 *  Cuando se activa la entrada contra tierra ( 0V), el diodo conduce, el transistor satura y la entrada al micro pasa
 *  a ser 3.6V.
 *  Por esto, las entradas del micro ( interrupcion ) las configuramos para sensar RISING_EDGE.
 *  Cuando el timer expira vuelve a leer el pin y si esta en 1 incrementa el contador.
 *  Arranca otro timer de periodo que es el que rearma la interrupcion.
 *
 *  R4.0.4a @ 2022-03-25:
 *  Usamos una sola base de tiempos ( timer ) que genera un tick c/10ms.
 *  La interrupcion me indica que llego un flanco con lo que inicializo el contador y comienzo a contar ticks
 *  En el tick 2 (20ms) leo la entrada de nuevo. Si cambio entonces fue un falso disparo. Rearmo todo y salgo.
 *  Si se mantiene el nivel, entonces es un pulso real. Calculo el caudal.
 *  Sigo contando ticks.
 *  En el tick 5 (50ms) rearmo la interrupcion. ( tiempo minimo de ancho de pulso a contar ).
 *
 *  R4.0.2b @ 2021-10-18:
 *  Cuando arrancan los contadores, si el nombre es q{x),Q{x},CAU{x},CAUDAL{x} entonces mido caudal y debo hacer
 *  un manejo de tiempos diferente.
 *  Utilizo 1 timestamp para c/contador.
 *  Cuando hago un read lo pongo en 0
 *  Cuando llega el primer pulso, actualizo el primer timeStamp restando al valor que tiene.
 *  Al llegar un read, tengo en dicho timestamp los ms del intervalo en que llegaron los pulsos.
 *  Con esto calculo el caudal.
 */

#include "ul_counters.h"

StaticTimer_t counter_xTimerBuffer;
TimerHandle_t counter_xTimer;

// Estructura de control de los contadores.
typedef struct {
	bool f_mide_caudal;
	float caudal;
	uint32_t ticks_count;
	uint16_t ctlTicks_count;
	uint16_t pulse_count;
	bool pulse;

} t_counter_s;

t_counter_s CNTCB[2];

static void pv_counters_TimerCallback( TimerHandle_t xTimer );
void pv_counters_restore_int(uint8_t cnt);
void pv_process_counter(uint8_t i);

#define UPULSE_RUN_BITPOS		4
#define UPULSE_RUN_PORT		PORTB
#define RMETER_config_IO_UPULSE_RUN()	PORT_SetPinAsOutput( &UPULSE_RUN_PORT, UPULSE_RUN_BITPOS)
#define RMETER_set_UPULSE_RUN()			PORT_SetOutputBit( &UPULSE_RUN_PORT, UPULSE_RUN_BITPOS)
#define RMETER_clr_UPULSE_RUN()			PORT_ClearOutputBit( &UPULSE_RUN_PORT, UPULSE_RUN_BITPOS)

//------------------------------------------------------------------------------------
void counters_init(void)
{

uint8_t i;
char lname[PARAMNAME_LENGTH];

	// Configuro los timers con el timeout dado por el tiempo de minimo pulse width.
	// Esto lo debo hacer aqui porque ya lei el systemVars y tengo los valores.
	// Esto arranca el timer por lo que hay que apagarlos

	if ( counters_conf.period[0] <= counters_conf.pwidth[0] ) {
		counters_conf.period[0] = counters_conf.pwidth[0] + 10;
		xprintf_P(PSTR("COUNTERS ERROR!! C0: periodo debe ser mayor que el ancho\r\n\0"));
	}

	if ( counters_conf.period[1] <= counters_conf.pwidth[1] ) {
		counters_conf.period[1] = counters_conf.pwidth[1] + 10;
		xprintf_P(PSTR("COUNTERS ERROR!! C1: periodo debe ser mayor que el ancho\r\n\0"));
	}

	// Configuro el HW
	COUNTERS_init(0, counters_conf.hw_type, counters_conf.sensing_edge[0] );
	COUNTERS_init(1, counters_conf.hw_type, counters_conf.sensing_edge[1] );

	for ( uint8_t i=0;i<2;i++) {
		CNTCB[i].caudal = 0.0;
		CNTCB[i].ticks_count = 0;
		CNTCB[i].ctlTicks_count = 0;
		CNTCB[i].pulse_count = 0;
		CNTCB[i].pulse = false;
	}

	// Determino si mido caudal
	CNTCB[0].f_mide_caudal = false;
	CNTCB[1].f_mide_caudal = false;
	for (i=0; i<COUNTER_CHANNELS; i++) {

		strncpy(lname, counters_conf.name[i], PARAMNAME_LENGTH );
		strupr(lname);

		if ( ( lname[0] == 'q') && ( isdigit(lname[1]) ) ) {
			CNTCB[i].f_mide_caudal = true;
			continue;
		}

		if ( ( lname[0] == 'Q') && ( isdigit(lname[1]) ) ) {
			CNTCB[i].f_mide_caudal = true;
			continue;
		}

		if ( strstr ( lname, "CAU" ) ) {
			CNTCB[i].f_mide_caudal = true;
			continue;
		}
	}

	// Habilito las interrupciones externas.
	pv_counters_restore_int(0);
	pv_counters_restore_int(1);

	// Arranco los ticks
	counter_xTimer = xTimerCreateStatic ("CNTA",
			pdMS_TO_TICKS( 10 ),
			pdTRUE,
			( void * ) 0,
			pv_counters_TimerCallback,
			&counter_xTimerBuffer
			);

	debug_counters = false;
	//RMETER_config_IO_UPULSE_RUN();

	xTimerStart(counter_xTimer, 10);

	if ( CNTCB[0].f_mide_caudal ) {
		xprintf_P(PSTR("Counter 0 mide caudal.\r\n"));
	}
	if ( CNTCB[1].f_mide_caudal ) {
		xprintf_P(PSTR("Counter 1 mide caudal.\r\n"));
	}
}
//------------------------------------------------------------------------------------
static void pv_counters_TimerCallback( TimerHandle_t xTimer )

{
	// Funcion de callback de la entrada de contador A.
	// Controla el pulse_width de la entrada A
	// Leo la entrada y si esta aun en X, incremento el contador y
	// prendo el timer xTimer1X que termine el debounce.

	//RMETER_set_UPULSE_RUN();
	//vTaskDelay( ( TickType_t)( 1 ) );
	//RMETER_clr_UPULSE_RUN();

	pv_process_counter(0);
	pv_process_counter(1);

}
//------------------------------------------------------------------------------------
void pv_process_counter(uint8_t i)
{

uint8_t input_val;

	CNTCB[i].ticks_count++;

	// Si estoy dentro de un pulso externo
	if ( CNTCB[i].pulse == true ) {

		CNTCB[i].ctlTicks_count++;		// Controlo los ticks de debounce y de restore ints.

		// Controlo el periodo de debounce. (2ticks = 20ms)
		if ( CNTCB[i].ctlTicks_count == counters_conf.pwidth[i]/10 ) {
			// Leo la entrada para ver si luego de un tdebounce esta en el mismo nivel.(pulso valido)
			input_val = CNT_read(i);
			if ( (counters_conf.sensing_edge[i] == RISING_EDGE ) && ( input_val == 1) ) {
				CNTCB[i].pulse_count++;
			} else 	if ( (counters_conf.sensing_edge[i] == FALLING_EDGE ) && ( input_val == 0) ) {
				CNTCB[i].pulse_count++;
			} else {
				// Falso disparo: rearmo y salgo
				CNTCB[i].pulse = false;
				pv_counters_restore_int(i);
				return;
			}

			// Estoy dentro de un pulso bien formado
			// 1 pulso -------> ticks_counts * 10 mS
			// magpp (mt3) ---> ticks_counts * 10 mS
			if ( CNTCB[i].f_mide_caudal ) {
				// Calculo el caudal
				// Tengo 1 pulso en N ticks.
				if ( CNTCB[i].ticks_count > 0 ) {
					CNTCB[i].caudal =  (( counters_conf.magpp[i] * 3600000) /  ( CNTCB[i].ticks_count * 10)  ); // En mt3/h
				} else {
					CNTCB[i].ticks_count = 1;
					CNTCB[i].caudal = 0;
				}
				//xprintf_P( PSTR("COUNTERS: DEBUG Q%d=%0.3f,TICKS=%d\r\n"), i, CNTCB[i].caudal, CNTCB[i].ticks_count );
				xprintf_PD( debug_counters, PSTR("COUNTERS: Q%d=%0.3f, pulses=%d\r\n"), i, CNTCB[i].caudal, CNTCB[i].pulse_count );
			} else {
				//xprintf_P( PSTR("COUNTERS: C%d=%d (PULSES)\r\n"), i, CNTCB[i].pulse_count );
				xprintf_PD( debug_counters, PSTR("COUNTERS: C%d=%d (PULSES)\r\n"), i, CNTCB[i].pulse_count );
			}
			// Reinicio los ticks
			CNTCB[i].ticks_count = 0;

		}

		// Cuando se cumple el periodo minimo del pulso, rearmo para el proximo
		if ( CNTCB[i].ctlTicks_count == counters_conf.period[i]/10 ) {
			//xprintf_P( PSTR("COUNTERS: DEBUG C0=%d,CTL=%d, TICKS=%d\r\n"), CNTCB[i].pulse_count, CNTCB[i].ctlTicks_count, CNTCB[i].ticks_count );
			CNTCB[i].pulse = false;
			pv_counters_restore_int(i);
		}
	}
}
//------------------------------------------------------------------------------------
void pv_counters_restore_int(uint8_t cnt)
{
	if ( cnt == 0 ) {
		PORTA.INT0MASK = PIN2_bm;
		PORTA.INTCTRL = PORT_INT0LVL0_bm;
		PORTA.INTFLAGS = PORT_INT0IF_bm;
		return;
	}

	if ( cnt == 1 ) {
		PORTB.INT0MASK = PIN2_bm;
		PORTB.INTCTRL = PORT_INT0LVL0_bm;
		PORTB.INTFLAGS = PORT_INT0IF_bm;
		return;
	}

}
//------------------------------------------------------------------------------------
void counters_clear(void)
{
	CNTCB[0].pulse_count = 0;
	CNTCB[1].pulse_count = 0;
	CNTCB[0].caudal = 0.0;
	CNTCB[1].caudal = 0.0;
}
//------------------------------------------------------------------------------------
void counters_read(float cnt[])
{
	/*
	 * En este momento, si estoy midiendo caudal hago la conversion
	 */

uint8_t i;

	for (i=0; i<2; i++ ) {

		if ( CNTCB[0].f_mide_caudal ) {
			cnt[i] = CNTCB[i].caudal;
		} else {
			cnt[i] = CNTCB[i].pulse_count * counters_conf.magpp[i];
		}
	}

}
//------------------------------------------------------------------------------------
void counters_print(file_descriptor_t fd, float cnt[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

	if ( strcmp ( counters_conf.name[0], "X" ) ) {
		xfprintf_P(fd, PSTR("%s:%.03f;"), counters_conf.name[0], cnt[0] );
	}

	if ( strcmp ( counters_conf.name[1], "X" ) ) {
		xfprintf_P(fd, PSTR("%s:%.03f;"), counters_conf.name[1], cnt[1] );
	}

}
//------------------------------------------------------------------------------------
char *counters_sprintf( char *sbuffer, float cnt[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

int16_t pos = 0;
char *p;

	p = sbuffer;
	if ( strcmp ( counters_conf.name[0], "X" ) ) {
		pos = sprintf_P( p, PSTR("%s:%.03f;"), counters_conf.name[0], cnt[0] );
		p += pos;
	}

	if ( strcmp ( counters_conf.name[1], "X" ) ) {
		pos = sprintf_P( p, PSTR("%s:%.03f;"), counters_conf.name[1], cnt[1] );
		p += pos;
	}

	return(p);
}
//------------------------------------------------------------------------------------
void counters_config_defaults(void)
{

	// Realiza la configuracion por defecto de los canales contadores.
	// Los valores son en ms.

uint8_t i = 0;

	for ( i = 0; i < COUNTER_CHANNELS; i++ ) {
		snprintf_P( counters_conf.name[i], PARAMNAME_LENGTH, PSTR("C%d\0"), i );
		counters_conf.magpp[i] = 1;
		counters_conf.period[i] = 100;
		counters_conf.pwidth[i] = 10;
		counters_conf.sensing_edge[i] = RISING_EDGE;
	}

	// Por defecto quedan en modo con optoacoplador.
	counters_conf.hw_type = COUNTERS_HW_OPTO;

}
//------------------------------------------------------------------------------------
bool counters_config_channel( uint8_t channel,char *s_name, char *s_magpp, char *s_pw, char *s_period, char *s_sensing )
{
	// Configuro un canal contador.
	// channel: id del canal
	// s_param0: string del nombre del canal
	// s_param1: string con el valor del factor magpp.
	//
	// {0..1} dname magPP

bool retS = false;

	if ( s_name == NULL ) {
		return(retS);
	}

	if ( ( channel >=  0) && ( channel < COUNTER_CHANNELS ) ) {

		// NOMBRE
		if ( u_control_string(s_name) == 0 ) {
			return( false );
		}

		snprintf_P( counters_conf.name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_name );

		// MAGPP
		if ( s_magpp != NULL ) { counters_conf.magpp[channel] = atof(s_magpp); }

		// PW
		if ( s_pw != NULL ) { counters_conf.pwidth[channel] = atoi(s_pw); }

		// PERIOD
		if ( s_period != NULL ) { counters_conf.period[channel] = atoi(s_period); }

		// SENSING ( RISE/FALL )
		if ( s_sensing != NULL ) {
			if ( strcmp_P( strupr(s_sensing), PSTR("RISE\0")) == 0 ) {
				counters_conf.sensing_edge[channel] = RISING_EDGE;

			} else if ( strcmp_P( strupr(s_sensing) , PSTR("FALL\0")) == 0 ) {
				counters_conf.sensing_edge[channel] = FALLING_EDGE;

			} else {
				xprintf_P(PSTR("ERROR: counters RISE o FALL only!!\r\n\0"));
			}
		}

		retS = true;
	}

	return(retS);

}
//------------------------------------------------------------------------------------
bool counters_config_hw( char *s_type )
{
	/*
	 * Las entradas de contadores son el un hardware sin optoacoplador, por lo tanto detectan
	 * flancos de bajada y en otros casos son con opto y detectan flanco de subida.
	 */

bool retS = false;

	//xprintf_P(PSTR("DEBUG counters_conf: %s\r\n\0"), s_type);

	if ( strcmp_P( strupr(s_type), PSTR("SIMPLE")) == 0 ) {
		counters_conf.hw_type = COUNTERS_HW_SIMPLE;
		retS = true;
	} else if ( strcmp_P( strupr(s_type), PSTR("OPTO")) == 0 ) {
		counters_conf.hw_type = COUNTERS_HW_OPTO;
		retS = true;
	} else {
		retS = false;
	}

	return(retS);

}
//------------------------------------------------------------------------------------
uint8_t counters_hash(void)
{

uint16_t i;
uint8_t hash = 0;
//char dst[40];
char *p;
uint8_t j = 0;
int16_t free_size = sizeof(hash_buffer);

	// C0:C0,1.000,100,10,0;C1:C1,1.000,100,10,0;

	// calculate own checksum
	for(i=0;i< COUNTER_CHANNELS;i++) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		free_size = sizeof(hash_buffer);
		j = 0;
		j += snprintf_P(&hash_buffer[j], free_size, PSTR("C%d:%s,"), i, counters_conf.name[i]);
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		j += snprintf_P(&hash_buffer[j], free_size, PSTR("%.03f,%d,"),counters_conf.magpp[i], counters_conf.period[i]);
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		j += snprintf_P(&hash_buffer[j], free_size, PSTR("%d,"), counters_conf.pwidth[i] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		if ( counters_conf.sensing_edge[i] == RISING_EDGE ) {
			j += snprintf_P(&hash_buffer[j], free_size, PSTR("RISE;"));
		} else {
			j += snprintf_P(&hash_buffer[j], free_size, PSTR("FALL;"));
		}
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		// Mientras no sea NULL calculo el checksum deol buffer
		while (*p != '\0') {
			// checksum += *p++;
			hash = u_hash(hash, *p++);
		}

	}

	return(hash);

exit_error:
	xprintf_P( PSTR("COMMS: counters_hash ERROR !!!\r\n\0"));
	return(0x00);
}
//------------------------------------------------------------------------------------
void counters_set_debug(void)
{
	debug_counters = true;
}
//------------------------------------------------------------------------------------
void counters_clr_debug(void)
{
	debug_counters = false;
}
//------------------------------------------------------------------------------------
void counters_print_status(void)
{

uint8_t channel;

	for ( channel = 0; channel <  COUNTER_CHANNELS; channel++) {
		xprintf_P( PSTR("  c%d: [%s,magpp=%.03f,pw=%d,T=%d "), channel, counters_conf.name[channel], counters_conf.magpp[channel],  counters_conf.pwidth[channel], counters_conf.period[channel] );
		if ( counters_conf.sensing_edge[channel] == RISING_EDGE ) {
			xprintf_P(PSTR("(RISE)\r\n\0"));
		} else {
			xprintf_P(PSTR("(FALL)\r\n\0"));
		}
	}
}
//------------------------------------------------------------------------------------
t_counters_hw_type counters_get_hwType(void)
{
	return (counters_conf.hw_type);

}
//------------------------------------------------------------------------------------
ISR ( PORTA_INT0_vect )
{
	// Esta ISR se activa cuando el contador D2 (PA2) genera un flaco se subida.
	// Si el contador es de HS solo cuenta

	// Indico que estoy dentro de un pulso
	// e inicializo el contador de control
	CNTCB[0].pulse = true;
	CNTCB[0].ctlTicks_count = 0;

	// Deshabilita la interrupcion por ahora ( enmascara )
	PORTA.INT0MASK = 0x00;
	PORTA.INTCTRL = 0x00;

}
//------------------------------------------------------------------------------------
ISR( PORTB_INT0_vect )
{
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.

	CNTCB[1].pulse = true;
	CNTCB[1].ctlTicks_count = 0;

	PORTB.INT0MASK = 0x00;
	PORTB.INTCTRL = 0x00;
	//PORTF.OUTTGL = 0x80;	// Toggle A2
	//IO_set_LED_KA();

}
//------------------------------------------------------------------------------------
