/*
 * spx_dinputs.c
 *
 */

#include "ul_dinputs.h"

//------------------------------------------------------------------------------------
void dinputs_init(void)
{
	// En el caso del SPX_8CH se deberia inicializar el port de salidas del MCP
	// pero esto se hace en la funcion MCP_init(). Esta tambien inicializa el port
	// de entradas digitales.

	IO_config_PA0();	// D0
	IO_config_PB7();	// D1

}
//------------------------------------------------------------------------------------
bool dinputs_config_channel( uint8_t channel, char *s_aname )
{

	// Configura los canales digitales. Es usada tanto desde el modo comando como desde el modo online por gprs.
	// config digital {0..N} dname {timer}

bool retS = false;

	if ( u_control_string(s_aname) == 0 ) {
		//xprintf_P( PSTR("DEBUG DIGITAL ERROR: D%d\r\n\0"), channel );
		return( false );
	}

	if ( s_aname == NULL ) {
		return(retS);
	}

	if ( ( channel >=  0) && ( channel < DINPUTS_CHANNELS ) ) {
		snprintf_P( dinputs_conf.name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_aname );
		retS = true;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
void dinputs_config_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.

uint8_t channel = 0;

	for ( channel = 0; channel < DINPUTS_CHANNELS; channel++ ) {
		snprintf_P( dinputs_conf.name[channel], PARAMNAME_LENGTH, PSTR("DIN%d\0"), channel );
	}
}
//------------------------------------------------------------------------------------
bool dinputs_read( uint8_t dst[] )
{
	// Leo las entradas que estan configuradas en modo NORMAL y copio de pv_din
	// los valores de aquellas que estan en modo TIMER
	// DIN0
	dst[0] = IO_read_PA0();
	// DIN1
	dst[1] = IO_read_PB7();

	return(true);
}
//------------------------------------------------------------------------------------
void dinputs_print(file_descriptor_t fd, uint8_t src[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

uint8_t channel = 0;

	for ( channel = 0; channel < DINPUTS_CHANNELS; channel++) {
		if ( ! strcmp ( dinputs_conf.name[channel], "X" ) )
			continue;

		xfprintf_P(fd, PSTR("%s:%d;"), dinputs_conf.name[channel], src[channel] );
	}

}
//------------------------------------------------------------------------------------
char *dinputs_sprintf( char *sbuffer, uint8_t src[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

uint8_t channel = 0;
int16_t i = 0;
int16_t pos = 0;
char *p;

	p = sbuffer;
	for ( channel = 0; channel < DINPUTS_CHANNELS; channel++) {
		if ( ! strcmp ( dinputs_conf.name[channel], "X" ) )
			continue;

		pos = sprintf_P( p, PSTR("%s:%d;"), dinputs_conf.name[channel], src[channel] );
		p += pos;
	}

	return(p);
}
//------------------------------------------------------------------------------------
uint8_t dinputs_hash(void)
{

uint8_t channel;
uint8_t hash = 0;
char *p;
uint8_t j = 0;
uint16_t free_size = sizeof(hash_buffer);

	// D0:D0;D1:D1

	// calculate own checksum
	for(channel=0;channel<DINPUTS_CHANNELS;channel++) {
		// Vacio el buffer temoral
		memset(hash_buffer,'\0', sizeof(hash_buffer));
		j = 0;
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		j += snprintf_P(&hash_buffer[j], free_size, PSTR("D%d:%s;"), channel , dinputs_conf.name[channel] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		// Mientras no sea NULL calculo el checksum deol buffer
		while (*p != '\0') {
			//checksum += *p++;
			hash = u_hash(hash, *p++);
		}

	}

	return(hash);

exit_error:
	xprintf_P( PSTR("COMMS: dinputs_hash ERROR !!!\r\n\0"));
	return(0x00);
}
//------------------------------------------------------------------------------------
void dinputs_test_read(void)
{
uint8_t dinputs[2];

	dinputs_read( &dinputs[0] );
	xprintf_P(PSTR("DIN0=%d, DIN1=%d\r\n"), dinputs[0], dinputs[1]);

}
//------------------------------------------------------------------------------------
void dinputs_print_status(void)
{

uint8_t channel;

	for ( channel = 0; channel <  DINPUTS_CHANNELS; channel++) {
		xprintf_P( PSTR("  d%d: %s \r\n\0"),channel, dinputs_conf.name[channel]);
	}
}
//------------------------------------------------------------------------------------

