/*
 * spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_H_
#define SRC_SPX_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <avr_compiler.h>
#include <clksys_driver.h>
#include <pmic_driver.h>
#include <TC_driver.h>
#include <wdt_driver.h>
#include <ctype.h>
#include <frtos_cmd.h>
#include <inttypes.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"
#include "math.h"

#include "frtos-io.h"
#include "l_counters.h"
#include "l_drv8814.h"
#include "l_iopines.h"
#include "l_eeprom.h"
#include "l_file.h"
#include "l_i2c.h"

#include "l_iopines.h"
#include "l_rtc79410.h"
#include "l_nvm.h"
#include "l_printf.h"
#include "l_bytes.h"
#include "l_bps120.h"
#include "l_adt7410.h"
#include "l_steppers.h"

#include "ul_pilotos.h"
#include "ul_dinputs.h"
#include "ul_counters.h"
#include "ul_ainputs.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.0a"
#define SPX_FW_DATE "@ 20210802"

#define SPX_HW_MODELO "spxR6 HW:xmega256A3B R1.1"
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"

//#define F_CPU (32000000UL)
//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32
//
#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkData_STACK_SIZE		512
#define tkPlt_STACK_SIZE		384
#define tkComms_STACK_SIZE		640
#define tkCommsRX_STACK_SIZE	384

#define XPRINT_TICKS() xprintf_PD( DF_COMMS,  PSTR("ticks: %lu\r\n"), sysTicks );

StaticTask_t xTask_Ctl_Buffer_Ptr;
StackType_t xTask_Ctl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t xTask_Cmd_Buffer_Ptr;
StackType_t xTask_Cmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t xTask_Data_Buffer_Ptr;
StackType_t xTask_Data_Buffer [tkData_STACK_SIZE];

StaticTask_t xTask_Plt_Buffer_Ptr;
StackType_t xTask_Plt_Buffer [tkPlt_STACK_SIZE];

StaticTask_t xTask_Comms_Buffer_Ptr;
StackType_t xTask_Comms_Buffer [tkComms_STACK_SIZE];

StaticTask_t xTask_CommsRX_Buffer_Ptr;
StackType_t xTask_CommsRX_Buffer [tkCommsRX_STACK_SIZE];


#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkPlt_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkComms_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkCommsRX_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

// Estructura que maneja las señales del sistema
struct {
	bool sgn_mon_sqe;
	bool sgn_redial;
	bool sgn_frame_ready;
	bool sgn_reset_comms_device;
	bool sgn_sms;
	bool sgn_poll_now;
} system_signals;

typedef enum { DEBUG_NONE = 0, DEBUG_COUNTER, DEBUG_DATA, DEBUG_COMMS } t_debug;

TaskHandle_t xHandle_idle, xHandle_tkCtl, xHandle_tkCmd, xHandle_tkData, xHandle_tkPlt, xHandle_tkComms, xHandle_tkCommsRX ;

bool startTask;
uint32_t sysTicks;

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_WDGS;
StaticSemaphore_t WDGS_xMutexBuffer;
#define MSTOTAKEWDGSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_RXBUFF;
StaticSemaphore_t RXBUFF_xMutexBuffer;
#define MSTOTAKERXBUFFSEMPH ((  TickType_t ) 10 )

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkData(void * pvParameters);
void tkPlt(void * pvParameters);
void tkComms(void * pvParameters);
void tkCommsRX(void * pvParameters);

#define DLGID_LENGTH		12
#define IP_LENGTH			24
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15

#define SIM_PASSWD_LENGTH	5

// Estructura de un registro
typedef struct {
	float ainputs[ANALOG_CHANNELS];			// 4 * 2 = 8
	uint8_t dinputs[DINPUTS_CHANNELS];		// 2 * 1 = 2
	float counters[COUNTER_CHANNELS];		// 4 * 2 = 8
	float battery;							// 4 * 1 = 4
} st_io_t;									// ----- = 22

// Estructura de datos comun independiente de la arquitectura de IO
typedef union u_dataframe {
	st_io_t io;	// 24
} u_dataframe_t;	// 56

typedef struct {
	u_dataframe_t df;	// 56
	RtcTimeType_t rtc;	//  7
} st_dataRecord_t;		// 63

typedef struct {
	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char simpwd[SIM_PASSWD_LENGTH];
	uint32_t timerDial;
} xComms_conf_t;

typedef struct {
	// Variables de trabajo.
	t_debug debug;
	uint16_t timerPoll;
	counters_conf_t counters_conf;	// Estructura con la configuracion de los contadores
	dinputs_conf_t dinputs_conf;	// Estructura con la configuracion de las entradas digitales
	ainputs_conf_t ainputs_conf;	// Estructura con la configuracion de las entradas analogicas
	piloto_conf_t piloto_conf;
	xComms_conf_t comms_conf;

	uint8_t checksum;
} systemVarsType;

systemVarsType systemVars;


// UTILS
void xCOMMS_config_defaults( char *opt );
void xCOMMS_status(void);
bool u_check_more_Rcds4Del ( FAT_t *fat );
bool u_check_more_Rcds4Tx(void);
void u_config_timerdial ( char *s_timerdial );

// TKCTL
void ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs );
uint16_t ctl_readTimeToNextPoll(void);
void ctl_reload_timerPoll( uint16_t new_time );
bool ctl_terminal_connected(void);
uint32_t ctl_read_timeToNextDial(void);
void ctl_set_timeToNextDial( uint32_t new_time );

// TKDATA
void data_read_inputs(st_dataRecord_t *dst, bool f_copy );
void data_print_inputs(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl);


int32_t xcomms_time_to_next_dial(void);

// WATCHDOG


uint8_t wdg_resetCause;

#define WDG_CTL			0
#define WDG_CMD			1
#define WDG_DATA		2
#define WDG_PLT			3
#define WDG_COMMS		4
#define WDG_COMMSRX		5

#define NRO_WDGS		5

uint16_t *wdg_timers[NRO_WDGS];
uint16_t watchdog_timers[NRO_WDGS];


#define WDG_TO30		30
#define WDG_TO60		60
#define WDG_TO120		120
#define WDG_TO180	 	180
#define WDG_TO300		300
#define WDG_TO600		600
#define WDG_TO900		900

//------------------------------------------------------------------------


#endif /* SRC_SPX_H_ */
