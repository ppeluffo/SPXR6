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
#include "ul_consignas.h"
#include "ul_pilotos.h"
#include "ul_modbus.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "4.0.2c"
#define SPX_FW_DATE "@ 20211101"

#define SPX_HW_MODELO "spxR6 HW:xmega256A3B R1.1"
#if configUSE_TICKLESS_IDLE == 2
#define SPX_FTROS_VERSION "FW:FRTOS10 TICKLESS"
#else
#define SPX_FTROS_VERSION "FW:FRTOS10"
#endif

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
#define tkData_STACK_SIZE		384
#define tkApp_STACK_SIZE		384
#define tkComms_STACK_SIZE		384
#define tkGprsRX_STACK_SIZE		384
#define tkAuxRX_STACK_SIZE		384

#define XPRINT_TICKS() xprintf_PD( DF_COMMS,  PSTR("ticks: %lu "), sysTicks );
#define XPRINT_TICKSrn() xprintf_PD( DF_COMMS,  PSTR("ticks: %lu\r\n"), sysTicks );

StaticTask_t xTask_Ctl_Buffer_Ptr;
StackType_t xTask_Ctl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t xTask_Cmd_Buffer_Ptr;
StackType_t xTask_Cmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t xTask_Data_Buffer_Ptr;
StackType_t xTask_Data_Buffer [tkData_STACK_SIZE];

StaticTask_t xTask_App_Buffer_Ptr;
StackType_t xTask_App_Buffer [tkApp_STACK_SIZE];

StaticTask_t xTask_Comms_Buffer_Ptr;
StackType_t xTask_Comms_Buffer [tkComms_STACK_SIZE];

StaticTask_t xTask_GprsRX_Buffer_Ptr;
StackType_t xTask_GprsRX_Buffer [tkGprsRX_STACK_SIZE];

StaticTask_t xTask_AuxRX_Buffer_Ptr;
StackType_t xTask_AuxRX_Buffer [tkGprsRX_STACK_SIZE];


#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkApp_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkComms_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkGprsRX_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define tkAuxRX_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )

// Estructura que maneja las señales del sistema
struct {
	bool sgn_mon_sqe;
	bool sgn_redial;
	bool sgn_frame_ready;
	bool sgn_reset_comms_device;
	bool sgn_sms;
	bool sgn_poll_now;
} system_signals;

typedef enum { DEBUG_NONE = 0, DEBUG_COUNTER, DEBUG_DATA, DEBUG_COMMS, DEBUG_APP, DEBUG_MODBUS } t_debug;
typedef enum { NONE = 0, MODBUS, ANALOG, COUNTER, DIGITAL } t_channel_type;

#define DF_DATA ( systemVars.debug == DEBUG_DATA )

TaskHandle_t xHandle_idle, xHandle_tkCtl, xHandle_tkCmd, xHandle_tkData, xHandle_tkApp, xHandle_tkComms, xHandle_tkGprsRX, xHandle_tkAuxRX ;

uint32_t sysTicks;
uint8_t start_byte;

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_WDGS;
StaticSemaphore_t WDGS_xMutexBuffer;
#define MSTOTAKEWDGSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_RXBUFF;
StaticSemaphore_t RXBUFF_xMutexBuffer;
#define MSTOTAKERXBUFFSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_MBUS;
StaticSemaphore_t MBUS_xMutexBuffer;
#define MSTOTAKEMBUSSEMPH ((  TickType_t ) 10 )


void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkData(void * pvParameters);
void tkApp(void * pvParameters);
void tkComms(void * pvParameters);
void tkGprsRX(void * pvParameters);
void tkAuxRX(void * pvParameters);

#define DLGID_LENGTH		12
#define IP_LENGTH			24
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15

#define SIM_PASSWD_LENGTH	5

// Estructura de un registro
typedef struct {
	float ainputs[ANALOG_CHANNELS];			// 4 * 5 = 20
	uint8_t dinputs[DINPUTS_CHANNELS];		// 2 * 1 =  2
	float counters[COUNTER_CHANNELS];		// 4 * 2 =  8
	float battery;							// 4 * 1 =  4
	float modbus[MODBUS_CHANNELS];			// 4 * 20 = 80
	RtcTimeType_t  rtc;						//   7
} st_dataRecord_t;							// 121

typedef struct {
	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char simpwd[SIM_PASSWD_LENGTH];
	uint32_t timerDial;
} comms_conf_t;

typedef struct {
	int8_t aplicacion;
	consigna_conf_t consigna_conf;
	piloto_conf_t piloto_conf;
} aplicacion_conf_t;

typedef struct {
	// Variables de trabajo.
	t_debug debug;
	uint16_t timerPoll;
	counters_conf_t counters_conf;	// Estructura con la configuracion de los contadores
	dinputs_conf_t dinputs_conf;	// Estructura con la configuracion de las entradas digitales
	ainputs_conf_t ainputs_conf;	// Estructura con la configuracion de las entradas analogicas
	comms_conf_t comms_conf;
	aplicacion_conf_t aplicacion_conf;
	modbus_conf_t modbus_conf;
	uint8_t checksum;
} systemVarsType;

systemVarsType systemVars;

// Estructura de datos de caudal para los pilotos
typedef struct {
	int8_t pA_channel;
	int8_t pB_channel;
	int8_t Q_channel;
	int8_t Q_module;
	float pA;
	float pB;
	float caudal;
} plt_vars_t;

plt_vars_t plt_ctl_vars;

// UTILS
void xCOMMS_config_defaults( char *opt );
void xCOMMS_status(void);
bool u_check_more_Rcds4Del ( FAT_t *fat );
bool u_check_more_Rcds4Tx(void);
void u_config_timerdial ( char *s_timerdial );
void XPRINT_ELAPSED( uint32_t ticks );
float ELAPSED_TIME_SECS( uint32_t ticks );

// TKCTL
uint16_t ctl_readTimeToNextPoll(void);
void ctl_reload_timerPoll( uint16_t new_time );
bool ctl_terminal_connected(void);
uint32_t ctl_read_timeToNextDial(void);
void ctl_set_timeToNextDial( uint32_t new_time );
void ctl_read_wdt(void);

// TKDATA
void data_read_inputs(st_dataRecord_t *dst, bool f_copy );
void data_print_inputs(file_descriptor_t fd, st_dataRecord_t *dr, uint16_t ctl);
int16_t data_sprintf_inputs( char *sbuffer ,st_dataRecord_t *dr, uint16_t ctl );


int32_t xcomms_time_to_next_dial(void);

// WATCHDOG
uint8_t wdg_resetCause;

#define WDG_CTL			0
#define WDG_CMD			1
#define WDG_DATA		2
#define WDG_APP			3
#define WDG_COMMSRX		4
#define WDG_COMMS		5
#define WDG_AUXRX		6

#define NRO_WDGS		7

uint16_t watchdog_timers[NRO_WDGS];

#define WDG_TO30		30
#define WDG_TO60		60
#define WDG_TO120		120
#define WDG_TO180	 	180
#define WDG_TO300		300
#define WDG_TO600		600
#define WDG_TO900		900

#define BIT_POS_MODEMSTATUS		0
#define BIT_VAL_MODEM_OFFLINE	0
#define BIT_VAL_MODEM_ONLINE	1
//------------------------------------------------------------------------


#endif /* SRC_SPX_H_ */
