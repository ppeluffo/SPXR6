/*
 * ul_counters.h
 *
 *  Created on: 14 jul. 2021
 *      Author: pablo
 */

#ifndef SPX_ULIBS_INCLUDE_UL_COUNTERS_H_
#define SPX_ULIBS_INCLUDE_UL_COUNTERS_H_

#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "l_printf.h"
#include "l_counters.h"

#include "ul_utils.h"

#define COUNTER_CHANNELS	2

// Configuracion de canales de contadores
typedef struct {
	t_counters_hw_type hw_type;							// OPTO | NORMAL
	char name[COUNTER_CHANNELS][PARAMNAME_LENGTH];
	float magpp[COUNTER_CHANNELS];
	uint16_t pwidth[COUNTER_CHANNELS];
	uint16_t period[COUNTER_CHANNELS];
	t_sensing_edge sensing_edge[COUNTER_CHANNELS];
} counters_conf_t;

counters_conf_t counters_conf;

bool debug_counters;

void counters_setup_outofrtos(void);
void counters_init(void);
void counters_config_defaults(void);
bool counters_config_channel( uint8_t channel,char *s_name, char *s_magpp, char *s_pw, char *s_period, char *s_sensing );
bool counters_config_hw( char *s_type );
void counters_clear(void);
void counters_run(void);
void counters_read(float cnt[]);
void counters_print(file_descriptor_t fd, float cnt[] );
char *counters_sprintf( char *sbuffer, float cnt[] );
uint8_t counters_hash(void);
void counters_set_debug(void);
void counters_clr_debug(void);
void counters_print_status(void);
t_counters_hw_type counters_get_hwType(void);



#endif /* SPX_ULIBS_INCLUDE_UL_COUNTERS_H_ */
