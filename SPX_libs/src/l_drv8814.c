/*
 * l_drv8814.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#include "l_drv8814.h"

//------------------------------------------------------------------------------------
void DRV8814_init(void)
{
	// Configura los pines del micro que son interface del driver DRV8814.

	IO_config_ENA();
	IO_config_PHA();
	IO_config_ENB();
	IO_config_PHB();
	IO_config_V12_OUTS_CTL();
	IO_config_RES();
	IO_config_SLP();

}
//------------------------------------------------------------------------------------
void DRV8814_power_on(void)
{
	// Prende la fuente de 12V que alimenta el DRV8814

	IO_set_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
void DRV8814_power_off(void)
{
	IO_clr_V12_OUTS_CTL();
}
//------------------------------------------------------------------------------------
// Valvulas
// open,close, pulse
// Los pulsos son de abre-cierra !!!
// Al operar sobre las valvulas se asume que hay fisicamente valvulas conectadas
// por lo tanto se debe propocionar corriente, sacar al driver del estado de reposo, generar
// la apertura/cierre, dejar al driver en reposo y quitar la corriente.
// No se aplica cuando queremos una salida FIJA !!!!

void DRV8814_vopen( char valve_id, uint8_t duracion )
{

	// Saco al driver 8814 de reposo.
	IO_set_RES();
	IO_set_SLP();

	switch (valve_id ) {
	case 'A':
		IO_set_PHA();
		IO_set_ENA();
		vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
		IO_clr_ENA();
		IO_clr_PHA();
		break;
	case 'B':
		IO_set_PHB();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
		IO_clr_ENB();
		IO_clr_PHB();
		break;
	}

	IO_clr_RES();
	IO_clr_SLP();

}
//------------------------------------------------------------------------------------
void DRV8814_vclose( char valve_id, uint8_t duracion )
{

	// Saco al driver 8814 de reposo.
	IO_set_RES();
	IO_set_SLP();

	switch (valve_id ) {
	case 'A':
		IO_clr_PHA();
		IO_set_ENA();
		vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
		IO_clr_ENA();
		break;
	case 'B':
		IO_clr_PHB();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( duracion / portTICK_RATE_MS ) );
		IO_clr_ENB();
		break;
	}

	IO_clr_RES();
	IO_clr_SLP();

}
//------------------------------------------------------------------------------------
void DRV8814_enable_pin( char driver_id, uint8_t val )
{

	switch (driver_id) {

	case 'A':
		switch(val) {
		case 0:
			IO_clr_ENA();
			break;
		case 1:
			IO_set_ENA();
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(val) {
		case 0:
			IO_clr_ENB();
			break;
		case 1:
			IO_set_ENB();
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

}
//------------------------------------------------------------------------------------
void DRV8814_sleep_pin( uint8_t val )
{
	( val == 0 ) ? IO_clr_SLP() : IO_set_SLP();
}
//------------------------------------------------------------------------------------
void DRV8814_reset_pin( uint8_t val )
{
	( val == 0 ) ? IO_clr_RES() : IO_set_RES();

}
//------------------------------------------------------------------------------------
void DRV8814_phase_pin( char driver_id, uint8_t val )
{

	switch (driver_id) {

	case 'A':
		switch(val) {
		case 0:
			IO_clr_PHA();
			break;
		case 1:
			IO_set_PHA();
			break;
		default:
			break;
		}
		break;

	case 'B':
		switch(val) {
		case 0:
			IO_clr_PHB();
			break;
		case 1:
			IO_set_PHB();
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

}
//------------------------------------------------------------------------------------
void DRV8814_sleep(void)
{
	DRV8814_sleep_pin(0);
}
//------------------------------------------------------------------------------------
void DRV8814_awake(void)
{
	DRV8814_sleep_pin(1);
}
//------------------------------------------------------------------------------------
void DRV8814_AOUT12_HZ(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_ENA();
}
//------------------------------------------------------------------------------------
void DRV8814_AOUT12_01(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_PHA();
	IO_set_ENA();
}
//------------------------------------------------------------------------------------
void DRV8814_AOUT12_10(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_set_PHA();
	IO_set_ENA();
}
//------------------------------------------------------------------------------------
void DRV8814_AOUT12_BRAKE(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_ENA();
}
//------------------------------------------------------------------------------------
void DRV8814_BOUT12_HZ(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_ENB();
}
//------------------------------------------------------------------------------------
void DRV8814_BOUT12_01(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_PHB();
	IO_set_ENB();
}
//------------------------------------------------------------------------------------
void DRV8814_BOUT12_10(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_set_PHB();
	IO_set_ENB();
}
//------------------------------------------------------------------------------------
void DRV8814_BOUT12_BRAKE(void)
{
	IO_set_RES();
	IO_set_SLP();
	IO_clr_ENB();
}
//------------------------------------------------------------------------------------
// FUNCIONES DE TESTING
//------------------------------------------------------------------------------------
bool DRV8814_write_test( char *s_param1, char *s_param2 )
{
	// Valores de s_param1: A,B,sleep, awake
	// VAlores de s_param2: 00,01,10,11

uint8_t drv_output_id = 0;
uint8_t drv_fase_id = 0;

	// PARAM1:
	if ( strcmp_P( strupr(s_param1), PSTR("A\0")) == 0 ) {
		drv_output_id = 0;
	} else 	if ( strcmp_P( strupr(s_param1), PSTR("B\0")) == 0 ) {
		drv_output_id = 1;
	} else 	if ( strcmp_P( strupr(s_param1), PSTR("SLEEP\0")) == 0 ) {
		DRV8814_sleep();
		return(true);
	} else 	if ( strcmp_P( strupr(s_param1), PSTR("AWAKE\0")) == 0 ) {
		DRV8814_awake();
		return(true);
	} else {
		return (false);
	}

	// PARAM2:
	if ( strcmp_P( strupr(s_param2), PSTR("00\0")) == 0 ) {
		drv_fase_id = 0;
	} else 	if ( strcmp_P( strupr(s_param2), PSTR("01\0")) == 0 ) {
		drv_fase_id = 1;
	} else 	if ( strcmp_P( strupr(s_param2), PSTR("10\0")) == 0 ) {
		drv_fase_id = 2;
	} else 	if ( strcmp_P( strupr(s_param2), PSTR("11\0")) == 0 ) {
		drv_fase_id = 3;
	} else {
		return (false);
	}

	switch(drv_output_id) {
	case 0:
		switch ( drv_fase_id) {
		case 0:
			DRV8814_AOUT12_HZ();
			break;
		case 1:
			DRV8814_AOUT12_01();
			break;
		case 2:
			DRV8814_AOUT12_10();
			break;
		case 3:
			DRV8814_AOUT12_BRAKE();
			break;
		}
		break;

	case 1:
		switch ( drv_fase_id) {
		case 0:
			DRV8814_BOUT12_HZ();
			break;
		case 1:
			DRV8814_BOUT12_01();
			break;
		case 2:
			DRV8814_BOUT12_10();
			break;
		case 3:
			DRV8814_BOUT12_BRAKE();
			break;
		}
		break;
	}

	return(true);
}
//------------------------------------------------------------------------------------

