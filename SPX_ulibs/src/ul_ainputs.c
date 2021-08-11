/*
 * spx_ainputs.c
 *
 *  Created on: 8 mar. 2019
 *      Author: pablo
 *
 *  Funcionamiento:
 *  Al leer los INA con la funcion pv_ainputs_read_channel_raw() nos devuelve el valor
 *  del conversor A/D interno del INA.
 *  De acuerdo a las hojas de datos ( sección 8.6.2.2 tenemos que 1LSB corresponde a 40uV ) de mod
 *  que si el valor row lo multiplicamos por 40/1000 tenemos los miliVolts que el ina esta midiendo.
 *
 */

#include "ul_ainputs.h"

// Factor por el que hay que multitplicar el valor raw de los INA para tener
// una corriente con una resistencia de 7.32 ohms.
// Surge que 1LSB corresponde a 40uV y que la resistencia que ponemos es de 7.32 ohms
// 1000 / 7.32 / 40 = 183 ;
#define INA_FACTOR  183

static bool sensores_prendidos = false;

static uint16_t pv_ainputs_read_battery_raw(void);
static uint16_t pv_ainputs_read_channel_raw(uint8_t channel_id );
static void pv_ainputs_apagar_12Vsensors(void);
static void pv_ainputs_prender_12V_sensors(void);
static void pv_ainputs_read_channel ( uint8_t io_channel, float *mag, uint16_t *raw, bool debug );
static void pv_ainputs_read_battery(float *battery);
static void pv_ainputs_prender_sensores(void);
static void pv_ainputs_apagar_sensores(void);

//------------------------------------------------------------------------------------
void ainputs_init(void)
{
	// Inicializo los INA con los que mido las entradas analogicas.
	ainputs_awake();
//	ainputs_sleep();

}
//------------------------------------------------------------------------------------
void ainputs_awake(void)
{

	INA_config_avg128(INA_A );
	INA_config_avg128(INA_B );

}
//------------------------------------------------------------------------------------
void ainputs_sleep(void)
{

	INA_config_sleep(INA_A );
	INA_config_sleep(INA_B );

}
//------------------------------------------------------------------------------------
bool ainputs_config_channel( uint8_t channel,char *s_aname,char *s_imin,char *s_imax,char *s_mmin,char *s_mmax,char *s_offset )
{

	// Configura los canales analogicos. Es usada tanto desde el modo comando como desde el modo online por gprs.
	// Detecto si hubo un cambio en el rango de valores de corriente para entonces restaurar los valores de
	// las corrientes equivalente.


bool retS = false;

	//xprintf_P( PSTR("DEBUG ANALOG CONFIG: A%d,name=%s,imin=%s,imax=%s,mmin=%s, mmax=%s\r\n\0"), channel, s_aname, s_imin, s_imax, s_mmin, s_mmax );

	if ( u_control_string(s_aname) == 0 ) {
		xprintf_P( PSTR("DEBUG ANALOG ERROR: A%d[%s]\r\n\0"), channel, s_aname );
		return( false );
	}

	if ( s_aname == NULL ) {
		return(retS);
	}

	if ( ( channel >=  0) && ( channel < ANALOG_CHANNELS ) ) {

		snprintf_P( ainputs_conf.name[channel], PARAMNAME_LENGTH, PSTR("%s\0"), s_aname );

		if ( s_imin != NULL ) {
			if ( ainputs_conf.imin[channel] != atoi(s_imin) ) {
				ainputs_conf.imin[channel] = atoi(s_imin);
			}
		}

		if ( s_imax != NULL ) {
			if ( ainputs_conf.imax[channel] != atoi(s_imax) ) {
				ainputs_conf.imax[channel] = atoi(s_imax);
			}
		}

		if ( s_offset != NULL ) {
			if ( ainputs_conf.offset[channel] != atof(s_offset) ) {
				ainputs_conf.offset[channel] = atof(s_offset);
			}
		}

		if ( s_mmin != NULL ) {
			ainputs_conf.mmin[channel] = atof(s_mmin);
		}

		if ( s_mmax != NULL ) {
			ainputs_conf.mmax[channel] = atof(s_mmax);
		}

		retS = true;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
void ainputs_config_timepwrsensor ( char *s_timepwrsensor )
{
	// Configura el tiempo de espera entre que prendo  la fuente de los sensores y comienzo el poleo.
	// Se utiliza solo desde el modo comando.
	// El tiempo de espera debe estar entre 1s y 15s

	ainputs_conf.pwr_settle_time = atoi(s_timepwrsensor);

	if ( ainputs_conf.pwr_settle_time < 1 )
		ainputs_conf.pwr_settle_time = 1;

	if ( ainputs_conf.pwr_settle_time > 15 )
		ainputs_conf.pwr_settle_time = 15;

	return;
}
//------------------------------------------------------------------------------------
void ainputs_config_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.

uint8_t channel = 0;

	ainputs_conf.pwr_settle_time = 5;

	for ( channel = 0; channel < ANALOG_CHANNELS; channel++) {
		ainputs_conf.imin[channel] = 0;
		ainputs_conf.imax[channel] = 20;
		ainputs_conf.mmin[channel] = 0.0;
		ainputs_conf.mmax[channel] = 6.0;
		ainputs_conf.offset[channel] = 0.0;

		snprintf_P( ainputs_conf.name[channel], PARAMNAME_LENGTH, PSTR("A%d\0"),channel );
	}

}
//------------------------------------------------------------------------------------
bool ainputs_read( float ain[], float *battery, bool debug )
{

bool retS = false;

	while ( xSemaphoreTake( sem_AINPUTS, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	pv_ainputs_prender_sensores();
	// Leo.
	// Los canales de IO no son los mismos que los canales del INA !! ya que la bateria
	// esta en el canal 1 del ina2
	// Lectura general.
	pv_ainputs_read_channel(0, &ain[0], NULL, debug );
	pv_ainputs_read_channel(1, &ain[1], NULL, debug );
	pv_ainputs_read_channel(2, &ain[2], NULL, debug );
	pv_ainputs_read_channel(3, &ain[3], NULL, debug );
	pv_ainputs_read_channel(4, &ain[4], NULL, debug );
	pv_ainputs_read_battery(battery);

	pv_ainputs_apagar_sensores();

	xSemaphoreGive( sem_AINPUTS );

	return(retS);

}
//------------------------------------------------------------------------------------
void ainputs_print(file_descriptor_t fd, float src[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

uint8_t i = 0;

	for ( i = 0; i < ANALOG_CHANNELS; i++) {
		if ( strcmp ( ainputs_conf.name[i], "X" ) != 0 )
			xfprintf_P(fd, PSTR("%s:%.02f;"), ainputs_conf.name[i], src[i] );
	}

}
//------------------------------------------------------------------------------------
char * ainputs_sprintf( char *sbuffer, float src[] )
{
	// Imprime los canales configurados ( no X ) en un fd ( tty_gprs,tty_xbee,tty_term) en
	// forma formateada.
	// Los lee de una estructura array pasada como src

uint8_t i = 0;
int16_t pos = 0;
char *p;

	p = sbuffer;
	for ( i = 0; i < ANALOG_CHANNELS; i++) {
		if ( strcmp ( ainputs_conf.name[i], "X" ) != 0 ) {
			pos = sprintf_P( p, PSTR("%s:%.02f;"), ainputs_conf.name[i], src[i] );
			p += pos;
		}
	}

	return(p);

}
//------------------------------------------------------------------------------------
void ainputs_battery_print( file_descriptor_t fd, float battery )
{
	// bateria
	xfprintf_P(fd, PSTR("bt:%.02f;"), battery );
}
//------------------------------------------------------------------------------------
char *ainputs_battery_sprintf( char *sbuffer, float battery )
{

int16_t pos = 0;
char *p;

	// bateria
	p = sbuffer;
	pos = sprintf_P( p, PSTR("bt:%.02f;"), battery );
	p += pos;
	return(p);

}
//------------------------------------------------------------------------------------
uint8_t ainputs_hash(void)
{
 // https://portal.u-blox.com/s/question/0D52p00008HKDMyCAP/python-code-to-generate-checksums-so-that-i-may-validate-data-coming-off-the-serial-interface

uint16_t i;
uint8_t hash = 0;
//char dst[40];
char *p;
uint8_t j = 0;
int16_t free_size = sizeof(hash_buffer);

	//	char name[MAX_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	//	uint8_t imin[MAX_ANALOG_CHANNELS];	// Coeficientes de conversion de I->magnitud (presion)
	//	uint8_t imax[MAX_ANALOG_CHANNELS];
	//	float mmin[MAX_ANALOG_CHANNELS];
	//	float mmax[MAX_ANALOG_CHANNELS];

	// A0:A0,0,20,0.000,6.000;A1:A1,0,20,0.000,6.000;A2:A2,0,20,0.000,6.000;A3:A3,0,20,0.000,6.000;A4:A4,0,20,0.000,6.000;

	for(i=0;i<ANALOG_CHANNELS;i++) {
		// Vacio el buffer temoral

		memset(hash_buffer,'\0', sizeof(hash_buffer));
		// Copio sobe el buffer una vista ascii ( imprimible ) de c/registro.
		j = 0;
		//xprintf_P( PSTR("DEBUG1: analog_free_size[%d]\r\n\0"), free_size );
		j += snprintf_P( &hash_buffer[j], free_size, PSTR("A%d:%s,"), i, ainputs_conf.name[i] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		//xprintf_P( PSTR("DEBUG1: analog_free_size[%d]\r\n\0"), free_size );
		j += snprintf_P(&hash_buffer[j], free_size, PSTR("%d,%d,"), ainputs_conf.imin[i], ainputs_conf.imax[i] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		//xprintf_P( PSTR("DEBUG1: analog_free_size[%d]\r\n\0"), free_size );
		j += snprintf_P(&hash_buffer[j], free_size, PSTR("%.02f,%.02f,"), ainputs_conf.mmin[i], ainputs_conf.mmax[i] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;

		//xprintf_P( PSTR("DEBUG1: analog_free_size[%d]\r\n\0"), free_size );
		j += snprintf_P(&hash_buffer[j], free_size , PSTR("%.02f;"), ainputs_conf.offset[i] );
		free_size = (  sizeof(hash_buffer) - j );
		if ( free_size < 0 ) goto exit_error;


		// Apunto al comienzo para recorrer el buffer
		p = hash_buffer;
		while (*p != '\0') {
			hash = u_hash(hash, *p++);
		}
		//xprintf_P( PSTR("COMMS: analog_hash(%d) OK[%d]\r\n\0"),i,free_size);
	}


	return(hash);

exit_error:
	xprintf_P( PSTR("COMMS: ainputs_hash ERROR !!!\r\n\0"));
	return(0x00);

}
//------------------------------------------------------------------------------------
void ainputs_test_channel( uint8_t io_channel, bool debug )
{

float mag;
uint16_t raw;

	while ( xSemaphoreTake( sem_AINPUTS, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	pv_ainputs_prender_sensores();
	// Fake read
	pv_ainputs_read_channel ( io_channel, &mag, &raw, debug  );
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	pv_ainputs_read_channel ( io_channel, &mag, &raw, debug );
	pv_ainputs_apagar_sensores();

	xSemaphoreGive( sem_AINPUTS );

	if ( io_channel != 99) {
		xprintf_P( PSTR("Analog Channel Test: CH[%02d] raw=%d,mag=%.02f\r\n\0"),io_channel,raw, mag);
	} else {
		// Convierto el raw_value a la magnitud ( 8mV por count del A/D)
		mag =  0.008 * raw;
		xprintf_P( PSTR("Analog Channel Test: Battery raw=%d,mag=%.02f\r\n\0"),raw, mag);

	}

}
//------------------------------------------------------------------------------------
void ainputs_print_channel_status(void)
{

uint8_t channel = 0;

	// Sensor Pwr Time
	//xprintf_P( PSTR("  timerPwrSensor: [%d s]\r\n\0"), ainputs_conf.pwr_settle_time );

	for ( channel = 0; channel < ANALOG_CHANNELS; channel++) {
		xprintf_P( PSTR("  a%d: [%d-%d mA/ %.02f,%.02f | %.03f | %s]\r\n"),
			channel,
			ainputs_conf.imin[channel],
			ainputs_conf.imax[channel],
			ainputs_conf.mmin[channel],
			ainputs_conf.mmax[channel],
			ainputs_conf.offset[channel] ,
			ainputs_conf.name[channel] );
	}

}
//------------------------------------------------------------------------------------
uint8_t ainputs_get_timePwrSensor(void)
{
	// Sensor Pwr Time
	return(ainputs_conf.pwr_settle_time );
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pv_ainputs_prender_sensores(void)
{

	ainputs_awake();
	//
	if ( ! sensores_prendidos ) {
		pv_ainputs_prender_12V_sensors();
		sensores_prendidos = true;
		// Normalmente espero 1s de settle time que esta bien para los sensores
		// pero cuando hay un caudalimetro de corriente, necesita casi 5s
		// vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		vTaskDelay( ( TickType_t)( ( 1000 * ainputs_conf.pwr_settle_time ) / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
static void pv_ainputs_apagar_sensores(void)
{

	pv_ainputs_apagar_12Vsensors();
	sensores_prendidos = false;
	ainputs_sleep();

}
//------------------------------------------------------------------------------------
static void pv_ainputs_prender_12V_sensors(void)
{
	IO_set_SENS_12V_CTL();
}
//------------------------------------------------------------------------------------
static void pv_ainputs_apagar_12Vsensors(void)
{
	IO_clr_SENS_12V_CTL();
}
//------------------------------------------------------------------------------------
static uint16_t pv_ainputs_read_channel_raw(uint8_t channel_id )
{
	// Como tenemos 2 arquitecturas de dataloggers, SPX_5CH y SPX_8CH,
	// los canales estan mapeados en INA con diferentes id.

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// Aqui convierto de io_channel a (ina_id, ina_channel )
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// ina_id es el parametro que se pasa a la funcion INA_id2busaddr para
	// que me devuelva la direccion en el bus I2C del dispositivo.


uint8_t ina_reg = 0;
uint8_t ina_id = 0;
uint16_t an_raw_val = 0;
uint8_t MSB = 0;
uint8_t LSB = 0;
char res[3] = { '\0','\0', '\0' };
int8_t xBytes = 0;
//float vshunt;

	//xprintf_P( PSTR("in->ACH: ch=%d, ina=%d, reg=%d, MSB=0x%x, LSB=0x%x, ANV=(0x%x)%d, VSHUNT = %.02f(mV)\r\n\0") ,channel_id, ina_id, ina_reg, MSB, LSB, an_raw_val, an_raw_val, vshunt );

	switch ( channel_id ) {
	case 0:
		ina_id = INA_A; ina_reg = INA3221_CH1_SHV;
		break;
	case 1:
		ina_id = INA_A; ina_reg = INA3221_CH2_SHV;
		break;
	case 2:
		ina_id = INA_A; ina_reg = INA3221_CH3_SHV;
		break;
	case 3:
		ina_id = INA_B; ina_reg = INA3221_CH2_SHV;
		break;
	case 4:
		ina_id = INA_B; ina_reg = INA3221_CH3_SHV;
		break;
	case 99:
		ina_id = INA_B; ina_reg = INA3221_CH1_BUSV;
		break;	// Battery
	default:
		return(-1);
		break;
	}

	// Leo el valor del INA.
//	xprintf_P(PSTR("DEBUG: INAID = %d\r\n\0"), ina_id );
	xBytes = INA_read( ina_id, ina_reg, res ,2 );

	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR I2C: pv_ainputs_read_channel_raw.\r\n\0"));

	an_raw_val = 0;
	MSB = res[0];
	LSB = res[1];
	an_raw_val = ( MSB << 8 ) + LSB;
	an_raw_val = an_raw_val >> 3;

//	vshunt = (float) an_raw_val * 40 / 1000;
//	xprintf_P( PSTR("out->ACH: ch=%d, ina=%d, reg=%d, MSB=0x%x, LSB=0x%x, ANV=(0x%x)%d, VSHUNT = %.02f(mV)\r\n\0") ,channel_id, ina_id, ina_reg, MSB, LSB, an_raw_val, an_raw_val, vshunt );

	return( an_raw_val );
}
//------------------------------------------------------------------------------------
static void pv_ainputs_read_channel ( uint8_t io_channel, float *mag, uint16_t *raw, bool debug )
{
	/*
	Lee un canal analogico y devuelve el valor convertido a la magnitud configurada.
	Es publico porque se utiliza tanto desde el modo comando como desde el modulo de poleo de las entradas.
	Hay que corregir la correspondencia entre el canal leido del INA y el canal fisico del datalogger
	io_channel. Esto lo hacemos en AINPUTS_read_ina.

	la funcion read_channel_raw me devuelve el valor raw del conversor A/D.
	Este corresponde a 40uV por bit por lo tanto multiplico el valor raw por 40/1000 y obtengo el valor en mV.
	Como la resistencia es de 7.32, al dividirla en 7.32 tengo la corriente medida.
	Para pasar del valor raw a la corriente debo hacer:
	- Pasar de raw a voltaje: V = raw * 40 / 1000 ( en mV)
	- Pasar a corriente: I = V / 7.32 ( en mA)
	- En un solo paso haria: I = raw / 3660
	  3660 = 40 / 1000 / 7.32.
	  Este valor 3660 lo llamamos INASPAN y es el valor por el que debo multiplicar el valor raw para que con una
	  resistencia shunt de 7.32 tenga el valor de la corriente medida. !!!!
	*/


uint16_t an_raw_val = 0;
float an_mag_val = 0.0;
float I = 0.0;
float M = 0.0;
float P = 0.0;
uint16_t D = 0;

	// Leo el valor del INA.(raw)
	an_raw_val = pv_ainputs_read_channel_raw( io_channel );

	// Convierto el raw_value a corriente
	I = (float) an_raw_val / INA_FACTOR;

	xprintf_PD( debug, PSTR("DEBUG ANALOG READ CHANNEL: A%d (RAW=%d), I=%.03f\r\n\0"), io_channel, an_raw_val, I );
	xprintf_PD( debug, PSTR("                         : Imin=%d, Imax=%d\r\n\0"), ainputs_conf.imin[io_channel], ainputs_conf.imax[io_channel] );
	xprintf_PD( debug, PSTR("                         : mmin=%.03f, mmax=%.03f\r\n\0"), ainputs_conf.mmin[io_channel], ainputs_conf.mmax[io_channel] );

	// Calculo la magnitud
	P = 0;
	D = ainputs_conf.imax[io_channel] - ainputs_conf.imin[io_channel];
	an_mag_val = 0.0;
	if ( D != 0 ) {
		// Pendiente
		P = (float) ( ainputs_conf.mmax[io_channel]  -  ainputs_conf.mmin[io_channel] ) / D;
		// Magnitud
		M = (float) ( ainputs_conf.mmin[io_channel] + ( I - ainputs_conf.imin[io_channel] ) * P);

		// Al calcular la magnitud, al final le sumo el offset.
		an_mag_val = M + ainputs_conf.offset[io_channel];
		// Corrijo el 0 porque sino al imprimirlo con 2 digitos puede dar negativo
		if ( fabs(an_mag_val) < 0.01 )
			an_mag_val = 0.0;

	} else {
		// Error: denominador = 0.
		an_mag_val = -999.0;
	}

	xprintf_PD( debug, PSTR("                         : D=%d, P=%.03f, M=%.03f\r\n\0"), D, P, M );
	xprintf_PD( debug, PSTR("                         : an_raw_val=%d, an_mag_val=%.03f\r\n\0"), an_raw_val, an_mag_val );

	*raw = an_raw_val;
	*mag = an_mag_val;

}
//------------------------------------------------------------------------------------
static void pv_ainputs_read_battery(float *battery)
{
	// Convierto el raw_value a la magnitud ( 8mV por count del A/D)
	*battery =  0.008 * pv_ainputs_read_battery_raw();
}
//------------------------------------------------------------------------------------
static uint16_t pv_ainputs_read_battery_raw(void)
{
	return( pv_ainputs_read_channel_raw(99));
}
//------------------------------------------------------------------------------------

