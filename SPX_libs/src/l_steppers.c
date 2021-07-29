/*
 * l_steppers.c
 *
 *  Created on: 11 ene. 2021
 *      Author: pablo
 */

#include "l_steppers.h"
#include "l_drv8814.h"
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include "l_printf.h"

void stepper_drive( t_stepper_dir dir, uint16_t npulses, uint16_t dtime, uint16_t ptime );

//------------------------------------------------------------------------------------
void stepper_cmd( char *s_dir, char *s_npulses, char *s_dtime, char *s_ptime )
{
uint16_t npulses, dtime, ptime;
t_stepper_dir dir;

	//xprintf_P(PSTR("STEPPER DEBUG: %s, %s, %s\r\n"), s_dir, s_npulses, s_dtime);

	if ( strcmp_P( strupr(s_dir), PSTR("FW")) == 0 ) {
		dir = STEPPER_FWD;
	} else if ( strcmp_P( strupr(s_dir), PSTR("RV")) == 0) {
		dir = STEPPER_REV;
	} else {
		xprintf_P(PSTR("Error en direccion\r\n"));
		return;
	}

	npulses = atoi(s_npulses);
	dtime = atoi(s_dtime);
	ptime = atoi(s_ptime);

	//xprintf_P(PSTR("STEPPER DEBUG: %s, %s, %s\r\n"), s_dir, s_npulses, s_dtime);

	stepper_drive(dir, npulses, dtime, ptime);
}
//------------------------------------------------------------------------------------
void stepper_drive( t_stepper_dir dir, uint16_t npulses, uint16_t dtime, uint16_t ptime )
{
	/*
	 * Genera en el stepper una cantidad de pulsos npulses, separados
	 * un tiempo dtime entre c/u, de modo de girar el motor en la
	 * direccion dir.
	 *
	 */

uint16_t steps;
int8_t sequence;

	// Activo el driver
	xprintf_P(PSTR("STEPPER driver pwr on\r\n"));
	stepper_pwr_on();
	// Espero 15s que se carguen los condensasores
	vTaskDelay( ( TickType_t)( ptime*1000 / portTICK_RATE_MS ) );
	stepper_awake();
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	xprintf_P(PSTR("STEPPER steps...\r\n"));
	// Pongo la secuencia incial en 2 para que puede moverme para adelante o atras
	// sin problemas de incializacion
	sequence = 2;
	for (steps=0; steps < npulses; steps++) {

		sequence = stepper_sequence(sequence, dir);

		xprintf_P(PSTR("STEPPER pulse %03d, sec=%d\r\n"), steps, sequence);

		stepper_pulse(sequence, dtime);
	}

	xprintf_P(PSTR("STEPPER driver pwr off\r\n"));
	// Desactivo el driver
	stepper_sleep();
	stepper_pwr_off();

}
//------------------------------------------------------------------------------------
void stepper_awake(void)
{
	// Saco al driver 8814 de reposo.
	IO_set_RES();
	IO_set_SLP();
}
//------------------------------------------------------------------------------------
void stepper_sleep(void)
{
	// Pongo en reposo
	IO_clr_RES();
	IO_clr_SLP();
}
//------------------------------------------------------------------------------------
void pulse_Amas_Amenos(uint16_t dtime )
{
	// Aout1 = H, Aout2 = L
	IO_set_PHA();	// Direccion del pulso forward

	IO_set_ENA();	// Habilito el pulso
	vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
	IO_clr_ENA();	// Deshabilito el pulso


}
//------------------------------------------------------------------------------------
void pulse_Amenos_Amas(uint16_t dtime )
{
	// Aout1 = L, Aout2 = H

	IO_clr_PHA();	// Direccion del pulso reverse

	IO_set_ENA();	// Habilito el pulso
	vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
	IO_clr_ENA();	// Deshabilito el pulso

}
//------------------------------------------------------------------------------------
void pulse_Bmas_Bmenos(uint16_t dtime )
{
	// Bout1 = H, Bout2 = L

	IO_set_PHB();	// Direccion del pulso forward

	IO_set_ENB();	// Habilito el pulso
	vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
	IO_clr_ENB();	// Deshabilito el pulso
}
//------------------------------------------------------------------------------------
void pulse_Bmenos_Bmas(uint16_t dtime )
{
	// Bout1 = L, Bout2 = H

	IO_clr_PHB();	// Direccion del pulso reverse

	IO_set_ENB();	// Habilito el pulso
	vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
	IO_clr_ENB();	// Deshabilito el pulso
}
//------------------------------------------------------------------------------------
int8_t stepper_sequence( int8_t sequence, t_stepper_dir dir)
{
int8_t lseq = sequence;

	if ( dir == STEPPER_FWD ) {
		lseq++;
		if ( lseq == 4) {
			lseq = 0;
		}
	}

	if ( dir == STEPPER_REV ) {
		lseq--;
		if ( lseq == -1 ) {
			lseq = 3;
		}
	}

	return(lseq);
}
//------------------------------------------------------------------------------------
void stepper_pulse(uint8_t sequence, uint16_t dtime)
{
	//xprintf_P(PSTR("STEPPER pulse: sec=%d\r\n"), sequence);
	switch (sequence) {
	case 0:
		// A+A-
		pulse_Amas_Amenos(dtime);
		break;
	case 1:
		// B+B-
		pulse_Bmas_Bmenos(dtime);
		break;
	case 2:
		// B- 180 degree
		pulse_Amenos_Amas(dtime);
		break;
	case 3:
		// A-, 90 degree
		pulse_Bmenos_Bmas(dtime);
		break;
	}
}
//------------------------------------------------------------------------------------
void stepper_pulse1(uint8_t sequence, uint16_t dtime)
{

	switch (sequence) {
	case 0:
		// A+, B+
		IO_set_PHA();
		IO_set_PHB();
		//
		IO_set_ENA();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
		IO_clr_ENA();
		IO_clr_ENB();
		break;
	case 1:
		// A-, B+
		IO_clr_PHA();
		IO_set_PHB();
		//
		IO_set_ENA();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
		IO_clr_ENA();
		IO_clr_ENB();
		break;
	case 2:
		// A-, B-
		IO_clr_PHA();
		IO_clr_PHB();
		//
		IO_set_ENA();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
		IO_clr_ENA();
		IO_clr_ENB();
		break;
	case 3:
		// A+, B-
		IO_set_PHA();
		IO_clr_PHB();
		//
		IO_set_ENA();
		IO_set_ENB();
		vTaskDelay( ( TickType_t)( dtime / portTICK_RATE_MS ) );
		IO_clr_ENA();
		IO_clr_ENB();
		break;
	}
}
//------------------------------------------------------------------------------------

