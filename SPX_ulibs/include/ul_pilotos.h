/*
 * ul_pilotos.h
 *
 *  Created on: 24 may. 2021
 *      Author: pablo
 */


#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"
#include "math.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "l_steppers.h"
#include "l_rtc79410.h"
#include "l_float_ringBuffer.h"

#include "spx.h"
#include "ul_utils.h"
#include "ul_ainputs.h"

#ifndef SRC_SPX_ULIBS_UL_PILOTOS_H_
#define SRC_SPX_ULIBS_UL_PILOTOS_H_

// LAS PRESIONES SE MIDEN EN GRS. !!!

#define MAX_PILOTO_PSLOTS	12

typedef struct {		// Elemento de piloto: presion, hora.
	st_time_t pTime;
	float presion;
} st_piloto_slot_t;

// PILOTO
typedef struct {
	uint16_t pulsesXrev;
	uint16_t pWidth;
	st_piloto_slot_t pltSlots[ MAX_PILOTO_PSLOTS ];
} piloto_conf_t;

piloto_conf_t piloto_conf;

typedef enum { AJUSTE70x100 = 0, AJUSTE_BASICO = 1 } t_ajuste_npulses;

typedef enum { PLT_ENTRY = 0, PLT_READ_INPUTS, PLT_CHECK_CONDITIONS, PLT_AJUSTE, PLT_OUTPUT_STATUS, PLT_EXIT } t_plt_states;
typedef enum { ST_CHECK_FIFO = 0, ST_CHECK_SLOT, ST_PENDIENTES, ST_AWAIT } t_fsm_pilotos;

typedef enum { MAX_TRYES=0, POUT_REACHED, PA_ERR, PB_ERR, PA_LESS_PB, BAND_ERR, ADJUST_ERR, CAUDAL_ERR, UNKNOWN	} t_exit_conditions;


#define MAX_INTENTOS			5
#define MAX_P_SAMPLES			10
#define P_SAMPLES				5
#define PULSOS_X_REV			3000		// 3000 pulsos para girar 1 rev
#define DPRES_X_REV				0.500		// 500 gr c/rev del piloto
#define PERROR					0.065
#define INTERVALO_PB_SECS		5
#define INTERVALO_TRYES_SECS	15
#define MIN_PA_PB				0.5
#define DELTA_PA_PB				0.3
#define DELTA_PA_PREF			0.3

struct {
	float pulsos_calculados;
	float pulsos_a_aplicar;
	float pulsos_rollback;
	uint16_t pulse_counts;
	uint16_t pwidth;
	t_stepper_dir dir;
	int8_t pA_channel;
	int8_t pB_channel;
	float pRef;
	float pA;
	float pB;
	float pB0;
	float pError;
	bool motor_running;
	uint8_t loops;
	float dync_pB0;
	float dync_pulsos;
	t_exit_conditions exit_code;
} PLTCB;	// Piloto Control Block

uint16_t piloto_wdg;

bool ajuste_pendiente;

#define PFIFO_STORAGE_SIZE 5
float pFifo_storage[PFIFO_STORAGE_SIZE];
float_ringBuffer_s pFIFO;

void piloto_setup_outofrtos(void );
void piloto_run_presion_test(char *s_pRef );
void piloto_run_stepper_test(char *s_dir, char *s_npulses, char *s_pwidth );
bool piloto_config_slot( char *s_slot, char *s_hhmm, char *s_presion );
void piloto_config_ppr( char *s_pulseXrev );
void piloto_config_pwidth( char *s_pwidth );
void piloto_config_status(void);
void piloto_config_defaults(void);
void piloto_app_service( uint8_t app_wdt );
uint8_t piloto_hash(void);
void piloto_set_presion_momentanea( float presion);



#endif /* SRC_SPX_ULIBS_UL_PILOTOS_H_ */
