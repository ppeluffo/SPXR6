/*
 * l_steppers.h
 *
 *  Created on: 11 ene. 2021
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_STEPPERS_H_
#define SRC_SPX_LIBS_L_STEPPERS_H_

#include "l_iopines.h"
#include "l_drv8814.h"

typedef enum { STEPPER_REV = 0, STEPPER_FWD = 1 } t_stepper_dir;

#define stepper_pwr_on() 	DRV8814_power_on()
#define stepper_pwr_off() 	DRV8814_power_off()
#define stepper_init() 		DRV8814_init()

void stepper_cmd( char *s_dir, char *s_npulses, char *s_dtime, char *s_ptime );

void stepper_awake(void);
void stepper_sleep(void);

void pulse_Amas_Amenos(uint16_t dtime );
void pulse_Amenos_Amas(uint16_t dtime );
void pulse_Bmas_Bmenos(uint16_t dtime );
void pulse_Bmenos_Bmas(uint16_t dtime );

int8_t stepper_sequence( int8_t sequence, t_stepper_dir dir);
void stepper_pulse(uint8_t sequence, uint16_t dtime);
void stepper_pulse1(uint8_t sequence, uint16_t dtime);

#endif /* SRC_SPX_LIBS_L_STEPPERS_H_ */
