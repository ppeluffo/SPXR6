/*
 * spx_tkPlt.c
 *
 *  Created on: 12 ene. 2021
 *      Author: pablo
 *
 *  Aproximarme siempre por abajo. Apuntar al 75% ( modificarlo x loop nbr)
 *  Los pilotos demoran en ajustar la presion mucho tiempo !!!
 *  Resultado al finalizar el ajuste.
 *  Numca mas de 1 vuelta
 *  Al bajar la presion el pilot demora mas en ajustar. !!!
 *  Controlar c/15 mins que este en el intervalo de p+-error
 *
 *
 *
 */

#include "spx.h"

//------------------------------------------------------------------------------------

#define WDG_PLT_TIMEOUT	WDG_TO120

void pv_plt_init(void);

//------------------------------------------------------------------------------------
void tkPlt(void * pvParameters)
{

( void ) pvParameters;
uint8_t slot_actual = -1;
uint8_t slot;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	pv_plt_init();
	// Registro el watchdog de la tarea.
	u_wdg_register( WDG_PLT, &piloto_wdg );

	xprintf_P( PSTR("starting tkPiloto..\r\n\0"));

	// Esperamos 60s que todo este estabilizado para comenzar a regular
	vTaskDelay( ( TickType_t)( 30000 / portTICK_RATE_MS ) );

	for( ;; )
	{
		// Paso c/10s plt 30s es suficiente.
		u_wdg_kick(&piloto_wdg, 120);

		vTaskDelay( ( TickType_t)( 10000 / portTICK_RATE_MS ) );

		// Callback para arrancar el test
		if ( PLTCB.start_test) {
			PLTCB.start_test = false;
			piloto_ajustar_presion();
			continue;
		}

		// Si no esta habilitada la funcion del piloto no hago mas nada.
		if ( ! piloto_conf.piloto_enabled )
			continue;

		// Veo si estoy en un slot para iniciar un ciclo. Esto me sirve tambien al incializar
		// C/ciclo veo a que slot corresponde.
		// Si cambie de slot, procedo a ajustar la presion.
		if ( piloto_leer_slot_actual( &slot ) ) {
			if ( slot_actual != slot ) {
				slot_actual = slot;
				xprintf_P(PSTR("PILOTO: Inicio de ciclo.\r\n"));
				xprintf_P(PSTR("PILOTO: slot=%d, pRef=%.03f\r\n"), slot_actual, piloto_conf.pltSlots[slot_actual].presion);
				PLTCB.pRef = piloto_conf.pltSlots[slot_actual].presion;
				piloto_ajustar_presion();
			}
		}
	}
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
void pv_plt_init(void)
{
	PLTCB.start_test = false;
	u_wdg_kick(&piloto_wdg, 120);
}
//------------------------------------------------------------------------------------
