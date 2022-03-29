/*
 * ul_genpulsos.h
 *
 *  Created on: 17 mar. 2022
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_GENPULSOS_H_
#define SPX_ULIBS_INCLUDE_UL_GENPULSOS_H_


//#include "spx.h"
#include "ul_utils.h"
#include "l_printf.h"
#include "timers.h"
#include "l_drv8814.h"
#include "l_steppers.h"
#include "ul_ainputs.h"
#include "ul_counters.h"
#include "ul_modbus.h"

typedef enum { GP_NONE = 0, GP_MODBUS, GP_ANALOG, GP_COUNTER, GP_DIGITAL } genpulsos_Q_channel_t;

// GENPULSOS

typedef struct {
	uint16_t pulsosXmt3;
	uint16_t pulsoWidth;
} genpulsos_conf_t;

typedef struct {
	float caudal;				// Caudal medido en el intervalo de poleo
	float litrosXtimertick;		// Se calcula en base al caudal actual los litros en c/tick del timer
	float litros_acumulados;	// Contador
	uint16_t litrosXpulso;		// Referencia del contador ( cuantos litros debo contar para un pulso, en base a pulsosXmt3)
	uint16_t periodoPulsos;
	bool modo_testing;
} genpulsos_svars_t;

genpulsos_conf_t genpulsos_conf;
genpulsos_svars_t genpulsos_svars;

#define GENPULSOS_TIMER_TICK_MS	100

#define CONFIG_LITROS_X_PULSO() ( genpulsos_svars.litrosXpulso = 1000 / genpulsos_conf.pulsosXmt3 )

void genpulsos_config_defaults(void);
void genpulsos_config_status(void);
bool genpulsos_config_pulsosXmt3( char *s_pulsesXmt3 );
bool genpulsos_config_pulsoWidth( char *s_pulseWidth);
void genpulsos_config_caudal( char *s_caudal);
bool genpulsos_config_testing( char *s_modo_testing);

uint8_t genpulsos_hash(void);

void genpulsos_init(void);
void genpulsos_Callback( TimerHandle_t xTimer );
void genpulsos_ajustar_litrosXtimertick(bool debug_flag);
void genpulsos_print_vars(void);


bool f_GENPULSOS;

#endif /* SPX_ULIBS_INCLUDE_UL_GENPULSOS_H_ */
