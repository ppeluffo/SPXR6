/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "spx.h"
#include "tkComms.h"

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static void pv_cmd_read_fuses(void);
static void pv_cmd_read_memory(void);
static void pv_cmd_rwGPRS(uint8_t cmd_mode );
static void pv_cmd_I2Cscan(bool busscan);
static bool pv_cmd_configGPRS(void);
static void pv_RTC_drift_test(void);

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);

static void status_canales_estandard(void);


#define WR_CMD 0
#define RD_CMD 1

RtcTimeType_t rtc;

uint16_t cmd_wdg;

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c = 0;
uint8_t ticks = 0;

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	FRTOS_CMD_init( xputChar, xprintf_cmd );

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdTERM,ioctl_SET_TIMEOUT, &ticks );

	u_wdg_register( WDG_CMD, &cmd_wdg );

	xprintf_P( PSTR("starting tkCmd..\r\n") );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		// Con la terminal desconectada paso c/5s plt 30s es suficiente.
		u_wdg_kick(&cmd_wdg, 120);

	//	PORTF.OUTTGL = 0x02;	// Toggle F1 Led Comms

		// Si no tengo terminal conectada, duermo 25s lo que me permite entrar en tickless.
		while ( ! ctl_terminal_connected() ) {
			u_wdg_kick(&cmd_wdg, 120);
			vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );
		}

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
		while ( frtos_read( fdTERM, (char *)&c, 1 ) == 1 ) {
			FRTOS_CMD_process(c);
		}

	}
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

FAT_t l_fat;
st_dataRecord_t dr;

	FRTOS_CMD_makeArgv();

	memset( &l_fat, '\0', sizeof(FAT_t));

	xprintf_P( PSTR("\r\nSpymovilA %s %s %s %s \r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );

	// SIGNATURE ID
	xprintf_P( PSTR("uID=%s\r\n\0"), NVMEE_readID() );

	// Last reset cause
	xprintf_P( PSTR("WRST=0x%02X\r\n\0") ,wdg_resetCause );

	RTC_read_time();

	// DlgId
	xprintf_P( PSTR("dlgid: %s\r\n\0"), sVarsComms.dlgId );

	// Memoria
	FAT_read(&l_fat);
	xprintf_P( PSTR("memory: rcdSize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), sizeof(st_dataRecord_t), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );

#ifdef GPRS_RX_LINEAL_BUFFER
	xprintf_P(PSTR("xComms Lineal buffers\r\n"));
#else
	xprintf_P(PSTR("xComms Circular buffers\r\n"));
#endif

	// COMMS Status
	gprs_modem_status();

	// CONFIG
	xprintf_P( PSTR(">Config:\r\n\0"));

	// debug
	switch(systemVars.debug) {
	case DEBUG_NONE:
		xprintf_P( PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_COUNTER:
		xprintf_P( PSTR("  debug: counter\r\n\0") );
		break;
	case DEBUG_DATA:
		xprintf_P( PSTR("  debug: data\r\n\0") );
		break;
	case DEBUG_COMMS:
		xprintf_P( PSTR("  debug: comms\r\n\0") );
		break;
	default:
		xprintf_P( PSTR("  debug: ???\r\n\0") );
		break;
	}

	// Timerpoll
	xprintf_P( PSTR("  timerPoll: [%d s]/ %d\r\n\0"), systemVars.timerPoll, ctl_readTimeToNextPoll() );

	// Timerdial
	xprintf_P( PSTR("  timerDial: [%ld s]/\0"), sVarsComms.timerDial );
	//xprintf_P( PSTR(" %d \r\n\0"), xcomms_time_to_next_dial() );

	// Sensor Pwr Time
	xprintf_P( PSTR("  timerPwrSensor: [%d s]\r\n\0"), ainputs_get_timePwrSensor() );

	// contadores( Solo hay 2 )
	switch ( counters_get_hwType() ) {
	case COUNTERS_HW_SIMPLE:
		xprintf_P( PSTR("  counters hw: simple\r\n\0"));
		break;
	case COUNTERS_HW_OPTO:
		xprintf_P( PSTR("  counters hw: opto\r\n\0"));
		break;
	}

	// Piloto task
	if ( piloto_conf.piloto_enabled ) {
		xprintf_P( PSTR("  Piloto: ON\r\n"));
		piloto_print_status();
	} else {
		xprintf_P( PSTR("  Piloto: OFF\r\n"));
	}

	status_canales_estandard();

	// Muestro los datos
	// CONFIG
	xprintf_P( PSTR(">Frame:\r\n\0"));
	data_read_inputs(&dr, true );
	data_print_inputs(fdTERM, &dr, 0);
}
//-----------------------------------------------------------------------------------
static void status_canales_estandard(void)
{
	xprintf_P( PSTR(">Channels:\r\n\0"));
	ainputs_print_channel_status();
	dinputs_print_status();
	counters_print_status();
}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	// Reset memory ??
	if ( strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))  == 0) {

		// Nadie debe usar la memoria !!!
		ctl_watchdog_kick(WDG_CMD, 0x8000 );

		vTaskSuspend( xHandle_tkData );
		ctl_watchdog_kick(WDG_DATA, 0x8000 );

		vTaskSuspend( xHandle_tkComms );
		ctl_watchdog_kick(WDG_COMMS, 0x8000 );

		if (strcmp_P( strupr(argv[2]), PSTR("SOFT\0")) == 0) {
			FF_format(false );
		} else if (strcmp_P( strupr(argv[2]), PSTR("HARD\0")) == 0) {
			FF_format(true);
		} else {
			xprintf_P( PSTR("ERROR\r\nUSO: reset memory {hard|soft}\r\n\0"));
			return;
		}
	}

	cmdClearScreen();

//	while(1)
//		;

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// PILOTO
	// write pilototest pRef(kg/cm2)
	if ( strcmp_P( strupr(argv[1]), PSTR("PILOTOTEST\0")) == 0 ) {
		piloto_run_presion_test( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// STEPPER
	// write steppertest {dir} {pulses} {pwidth}
	if ( strcmp_P( strupr(argv[1]), PSTR("STEPPERTEST\0")) == 0 ) {
		piloto_run_stepper_test( argv[2], argv[3], argv[4]);
		pv_snprintfP_OK();
		return;
	}

	// DRV8814
	// write drv8814 {A|B} {00,01,10,11}, sleep, awake
	if ( strcmp_P( strupr(argv[1]), PSTR("DRV8814\0")) == 0 ) {
		( DRV8814_write_test( argv[2], argv[3] )  > 0)?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}


	// GPRS
	// write gprs pwr|sw|rts {on|off}
	// write gprs cmd {atcmd}
	// write gprs sms nbr msg
	// write gprs qsms nbr msg
	if ( ( strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) == 0) ) {
		pv_cmd_rwGPRS(WR_CMD);
		return;
	}

	// ANALOG
	// write analog wakeup/sleep
	if ((strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) == 0) ) {
		if ( strcmp_P( strupr(argv[2]), PSTR("WAKEUP\0")) == 0 ) {
			ainputs_awake();
			return;
		}
		if (strcmp_P( strupr(argv[2]), PSTR(" SLEEP\0")) == 0 ) {
			ainputs_sleep();
			return;
		}
		return;
	}

	// RTC
	// write rtc YYMMDDhhmm
	if ( strcmp_P( strupr(argv[1]), PSTR("RTC\0")) == 0 ) {
		( RTC_write_time( argv[2]) > 0)?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE
	// write ee pos string
	if ((strcmp_P( strupr(argv[1]), PSTR("EE\0")) == 0) ) {
		( EE_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// NVMEE
	// write nvmee pos string
	if ( (strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) == 0)) {
		NVMEE_test_write ( argv[2], argv[3] );
		pv_snprintfP_OK();
		return;
	}

	// RTC SRAM
	// write rtcram pos string
	if ( (strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0")) == 0)  ) {
		( RTCSRAM_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SENS12V
	// write sens12V {on|off}
	if ( (strcmp_P( strupr(argv[1]), PSTR("SENS12V\0")) == 0) ) {

		if ( strcmp_P( strupr(argv[2]), PSTR("ON\0")) == 0 ) {
			IO_set_SENS_12V_CTL();
			pv_snprintfP_OK();
		} else if  ( strcmp_P( strupr(argv[2]), PSTR("OFF\0")) == 0 ) {
			IO_clr_SENS_12V_CTL();
			pv_snprintfP_OK();
		} else {
			xprintf_P( PSTR("cmd ERROR: ( write sens12V on{off} )\r\n\0"));
			pv_snprintfP_ERR();
		}
		return;
	}

	// INA
	// write ina id rconfValue
	// Solo escribimos el registro 0 de configuracion.
	if ((strcmp_P( strupr(argv[1]), PSTR("INA\0")) == 0) ) {
		( INA_test_write ( argv[2], argv[3] ) > 0)?  pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

st_dataRecord_t dr;
uint8_t cks;

	FRTOS_CMD_makeArgv();


	if (!strcmp_P( strupr(argv[1]), PSTR("RTCDTEST\0")) ) {
		pv_RTC_drift_test();
		return;
	}

	// HASHES
	// read hashes
	if (!strcmp_P( strupr(argv[1]), PSTR("HASHES\0"))  ) {
		cks = u_base_hash();
		xprintf_P( PSTR("Base hash = [0x%02x]\r\n\0"), cks );
		cks = ainputs_hash();
		xprintf_P( PSTR("Analog hash = [0x%02x]\r\n\0"), cks );
		cks = dinputs_hash();
		xprintf_P( PSTR("Digital hash = [0x%02x]\r\n\0"), cks );
		cks = counters_hash();
		xprintf_P( PSTR("Counters hash = [0x%02x]\r\n\0"), cks );
		return;
	}

	// I2Cscan
	// read i2cscan busaddr
	if (!strcmp_P( strupr(argv[1]), PSTR("I2CSCAN\0"))  ) {
		pv_cmd_I2Cscan(false);
		return;
	}

	// I2Cscanbus
	// read i2cscanbus
	if (!strcmp_P( strupr(argv[1]), PSTR("I2CSCANBUS\0"))  ) {
		pv_cmd_I2Cscan(true);
		return;
	}

	// FUSES
 	if (!strcmp_P( strupr(argv[1]), PSTR("FUSES\0"))) {
 		pv_cmd_read_fuses();
 		return;
 	}

	// RTC
	// read rtc
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0")) ) {
		RTC_read_time();
		return;
	}

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0")) ) {
		 EE_test_read ( argv[2], argv[3] );
		return;
	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) ) {
		NVMEE_test_read ( argv[2], argv[3] );
		return;
	}

	// RTC SRAM
	// read rtcram address length
	if (!strcmp_P( strupr(argv[1]), PSTR("RTCRAM\0"))) {
		RTCSRAM_test_read ( argv[2], argv[3] );
		return;
	}

	// INA
	// read ina id regName
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))  ) {
		ainputs_awake();
		INA_test_read ( argv[2], argv[3] );
		ainputs_sleep();
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		data_read_inputs(&dr, false );
		data_print_inputs(fdTERM, &dr, 0);
		return;
	}

	// MEMORY
	// read memory
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))  ) {
		pv_cmd_read_memory();
		return;
	}

	// GPRS
	// read gprs (rsp,cts,dcd,ri, sms)
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))  ) {
		pv_cmd_rwGPRS(RD_CMD);
		return;
	}

	// ACH {n}
	// read ach n
	if (!strcmp_P( strupr(argv[1]), PSTR("ACH\0")) ) {
		ainputs_test_channel( atoi(argv[2]));
		return;
	}

	// DIN
	// read din
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0") ) ) {
		dinputs_test_read();
		return;
	}

	// battery
	// read battery
	if (!strcmp_P( strupr(argv[1]), PSTR("BATTERY\0")) ) {
		ainputs_test_channel(99);
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;

	FRTOS_CMD_makeArgv();

	// PILOTO
	// config piloto slot {idx} {hhmm} {pout}
	if (!strcmp_P( strupr(argv[1]), PSTR("PILOTO\0")) ) {
		retS = piloto_config( argv[2], argv[3], argv[4], argv[5] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// GPRS SAT
	// config gprs SAT {enable|disable}
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0")) ) {
		retS =  pv_cmd_configGPRS();
		retS ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// COUNTERS
	// config counter {0..1} cname magPP pulseWidth period speed sensing
	// counter hw {SIMPLE/OPTO)
	if ( strcmp_P( strupr(argv[1]), PSTR("COUNTER\0")) == 0 ) {

		if (strcmp_P( strupr(argv[2]), PSTR("HW\0")) == 0 ) {
			retS = counters_config_hw( argv[3]);
		} else {
			retS = counters_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		}

		retS ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}


	// DEFAULT
	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		u_load_defaults( strupr(argv[2]) );
		pv_snprintfP_OK();
		return;
	}

	// SAVE
	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		u_save_params_in_NVMEE();
		pv_snprintfP_OK();
		return;
	}

	// Parametros ENTRADAS DIGITALES------------------------------------------------------------------------
	// DIGITAL
	// config digital {0..N} dname
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0")) ) {
		dinputs_config_channel( atoi(argv[2]), argv[3] ) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// Parametros ENTRADAS ANALOGICAS------------------------------------------------------------------------
	// ANALOG
	// config analog {0..n} aname imin imax mmin mmax offset
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0")) ) {
		ainputs_config_channel( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7], argv[8] ) ? pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// Parametros COMUNICACIONES ---------------------------------------------------------------------------------
	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(sVarsComms.apn, '\0', sizeof(sVarsComms.apn));
			memcpy(sVarsComms.apn, argv[2], sizeof(sVarsComms.apn));
			sVarsComms.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PORT ( SERVER IP PORT)
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(sVarsComms.server_tcp_port, '\0', sizeof(sVarsComms.server_tcp_port));
			memcpy(sVarsComms.server_tcp_port, argv[2], sizeof(sVarsComms.server_tcp_port));
			sVarsComms.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// IP (SERVER IP ADDRESS)
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(sVarsComms.server_ip_address, '\0', sizeof(sVarsComms.server_ip_address));
			memcpy(sVarsComms.server_ip_address, argv[2], sizeof(sVarsComms.server_ip_address));
			sVarsComms.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SCRIPT ( SERVER SCRIPT SERVICE )
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(sVarsComms.serverScript, '\0', sizeof(sVarsComms.serverScript));
			memcpy(sVarsComms.serverScript, argv[2], sizeof(sVarsComms.serverScript));
			sVarsComms.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DEBUG
	// config debug
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		counters_clr_debug();
		ainputs_clr_debug();
		if (!strcmp_P( strupr(argv[2]), PSTR("NONE\0"))) {
			systemVars.debug = DEBUG_NONE;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("COUNTER\0"))) {
			systemVars.debug = DEBUG_COUNTER;
			counters_set_debug();
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("DATA\0"))) {
			systemVars.debug = DEBUG_DATA;
			ainputs_set_debug();
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("COMMS\0"))) {
			systemVars.debug = DEBUG_COMMS;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// TIMEPWRSENSOR
	// config timepwrsensor
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMEPWRSENSOR\0")) ) {
		ainputs_config_timepwrsensor( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// TIMERPOLL
	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		u_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// TIMERDIAL
	// config timerdial
	if ( !strcmp_P( strupr(argv[1]), PSTR("TIMERDIAL\0"))) {
		u_config_timerdial( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
			} else {
				memset(sVarsComms.dlgId,'\0', sizeof(sVarsComms.dlgId) );
				memcpy(sVarsComms.dlgId, argv[2], sizeof(sVarsComms.dlgId));
				sVarsComms.dlgId[DLGID_LENGTH - 1] = '\0';
				retS = true;
			}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	pv_snprintfP_ERR();
	return;
}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		xprintf_P( PSTR("-write\r\n\0"));
		xprintf_P( PSTR("  rtc YYMMDDhhmm\r\n\0"));
		xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos} {string}\r\n\0"));
		xprintf_P( PSTR("  ina {id} {rconfValue}, sens12V {on|off}\r\n\0"));
		xprintf_P( PSTR("  analog (wakeup | sleep )\r\n\0"));

		xprintf_P( PSTR("  drv8814 {pA|pB} {00,01,10,11}, sleep, awake\r\n"));
		xprintf_P( PSTR("  steppertest {dir} {pulses} {pwidth_ms}\r\n"));
		xprintf_P( PSTR("  pilototest pRef(kg/cm2)\r\n"));

		xprintf_P( PSTR("  gprs (pwr|sw|rts|dtr) {on|off}\r\n\0"));
		xprintf_P( PSTR("       cmd {atcmd}, redial, monsqe\r\n\0"));
		xprintf_P( PSTR("       sms,qsms,fsms {nbr,msg}\r\n\0"));

		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		xprintf_P( PSTR("-read\r\n\0"));
		xprintf_P( PSTR("  rtc, frame, fuses\r\n\0"));
		xprintf_P( PSTR("  id\r\n\0"));
		xprintf_P( PSTR("  (ee,nvmee,rtcram) {pos} {lenght}\r\n\0"));
		xprintf_P( PSTR("  ina (id) {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
		xprintf_P( PSTR("  i2cscan {busaddr}, i2cscanbus\r\n\0"));
		xprintf_P( PSTR("  ach {n}, din, battery\r\n\0"));
		xprintf_P( PSTR("  memory {full}\r\n\0"));
		xprintf_P( PSTR("  gprs (rsp,cts,dcd,ri,sms)\r\n\0"));
		return;

	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
		xprintf_P( PSTR("  memory {soft|hard}\r\n\0"));
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		xprintf_P( PSTR("-config\r\n\0"));
		xprintf_P( PSTR("  dlgid, apn, port, ip, script\r\n\0"));
		xprintf_P( PSTR("  gprs SAT {check|enable|disable}\r\n\0"));
		xprintf_P( PSTR("  timerpoll {val}, timerdial {val}, timepwrsensor {val}\r\n\0"));

		xprintf_P( PSTR("  debug {none,counter,data,comms}\r\n\0"));

		xprintf_P( PSTR("  digital {0..%d} {dname}\r\n\0"), ( DINPUTS_CHANNELS - 1 ) );

		xprintf_P( PSTR("  counter {0..%d} cname magPP pw(ms) period(ms) edge(RISE/FALL)\r\n\0"), ( COUNTER_CHANNELS - 1 ) );
		xprintf_P( PSTR("          hw {SIMPLE/OPTO)\r\n\0") );

		xprintf_P( PSTR("  analog {0..%d} aname imin imax mmin mmax offset\r\n\0"),( ANALOG_CHANNELS - 1 ) );

		xprintf_P( PSTR("  piloto slot {idx} {hhmm} {pout}\r\n\0"));
		xprintf_P( PSTR("         {on,off}\r\n\0"));

		xprintf_P( PSTR("  default {SPY|OSE|CLARO}\r\n\0"));
		xprintf_P( PSTR("  save\r\n\0"));
	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0")) ) {
		xprintf_P( PSTR("-kill {data, piloto, commstx }\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-status\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-kill...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));
		xprintf_P( PSTR("-config...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		ctl_watchdog_kick(WDG_DATA, 0x8000 );
		pv_snprintfP_OK();
		return;
	}

	// KILL PILOTO
	if (!strcmp_P( strupr(argv[1]), PSTR("PILOTO\0"))) {
		vTaskSuspend( xHandle_tkPlt );
		ctl_watchdog_kick(WDG_PLT, 0x8000 );
		pv_snprintfP_OK();
		return;
	}

	// KILL COMMSTX
	if (!strcmp_P( strupr(argv[1]), PSTR("COMMSTX\0"))) {
		vTaskSuspend( xHandle_tkComms );
		ctl_watchdog_kick(WDG_COMMS, 0x8000 );
		// Dejo la flag de modem prendido para poder leer comandos
		xCOMMS_stateVars.gprs_prendido = true;
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_read_fuses(void)
{
	// Lee los fuses.

uint8_t fuse0 = 0;
uint8_t fuse1 = 0;
uint8_t fuse2 = 0;
uint8_t fuse4 = 0;
uint8_t fuse5 = 0;

	fuse0 = nvm_fuses_read(0x00);	// FUSE0
	xprintf_P( PSTR("FUSE0=0x%x\r\n\0"),fuse0);

	fuse1 = nvm_fuses_read(0x01);	// FUSE1
	xprintf_P( PSTR("FUSE1=0x%x\r\n\0"),fuse1);

	fuse2 = nvm_fuses_read(0x02);	// FUSE2
	xprintf_P( PSTR("FUSE2=0x%x\r\n\0"),fuse2);

	fuse4 = nvm_fuses_read(0x04);	// FUSE4
	xprintf_P( PSTR("FUSE4=0x%x\r\n\0"),fuse4);

	fuse5 = nvm_fuses_read(0x05);	// FUSE5
	xprintf_P( PSTR("FUSE5=0x%x\r\n\0"),fuse5);

	if ( (fuse0 != 0xFF) || ( fuse1 != 0xAA) || (fuse2 != 0xFD) || (fuse4 != 0xF5) || ( fuse5 != 0xD6) ) {
		xprintf_P( PSTR("FUSES ERROR !!!.\r\n\0"));
		xprintf_P( PSTR("Los valores deben ser: FUSE0=0xFF,FUSE1=0xAA,FUSE2=0xFD,FUSE4=0xF5,FUSE5=0xD6\r\n\0"));
		xprintf_P( PSTR("Deben reconfigurarse !!.\r\n\0"));
		pv_snprintfP_ERR();
		return;
	}
	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_read_memory(void)
{
	// Si hay datos en memoria los lee todos y los muestra en pantalla
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.
	// El proceso es similar a tkGprs.transmitir_dataframe


FAT_t l_fat;
size_t bRead = 0;
uint16_t rcds = 0;
bool detail = false;
st_dataRecord_t dr;

	memset( &l_fat, '\0', sizeof(FAT_t));

	if (!strcmp_P( strupr(argv[2]), PSTR("FULL\0"))) {
		detail = true;
	}

	FF_rewind();
	while(1) {

		bRead = FF_readRcd( &dr, sizeof(st_dataRecord_t));
		FAT_read(&l_fat);

		if ( bRead == 0) {
			xprintf_P(PSTR( "MEM Empty\r\n"));
			break;
		}

		if ( ( rcds++ % 32) == 0 ) {
			u_wdg_kick(&cmd_wdg, 120);
		}

		// imprimo
		if ( detail ) {
			xprintf_P( PSTR("memory: wrPtr=%d,rdPtr=%d,delPtr=%d,r4wr=%d,r4rd=%d,r4del=%d \r\n\0"), l_fat.wrPTR,l_fat.rdPTR, l_fat.delPTR,l_fat.rcds4wr,l_fat.rcds4rd,l_fat.rcds4del );
		}

		// Imprimo el registro
		data_print_inputs(fdTERM, &dr, l_fat.rdPTR );

		xprintf_P(PSTR( "\r\n"));
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwGPRS(uint8_t cmd_mode )
{

uint8_t pin = 0;
//char *p;

	if ( cmd_mode == WR_CMD ) {

		// write gprs test at
/*		if ( strcmp_P( strupr(argv[2]), PSTR("TEST\0")) == 0 ) {
			gprs_CSOCKAUTH();
			pv_snprintfP_OK();
			return;
		}
*/
		// write gprs sms nbr msg
		if ( strcmp_P( strupr(argv[2]), PSTR("SMS\0")) == 0 ) {
			//xSMS_enqueue( argv[3], (char *) xSMS_format(argv[4]) );
			pv_snprintfP_OK();
			return;
		}

		// write gprs qsms nbr msg
		if ( strcmp_P( strupr(argv[2]), PSTR("QSMS\0")) == 0 ) {
//C			u_gprs_quick_send_sms( argv[3], argv[4] );
			pv_snprintfP_OK();
			return;
		}

		// write gprs fsms nbr msg
		if ( strcmp_P( strupr(argv[2]), PSTR("FSMS\0")) == 0) {
		//	xSMS_send( argv[3], xSMS_format(argv[4]) );
			pv_snprintfP_OK();
			return;
		}

		// write gprs (pwr|sw|rts|dtr) {on|off}
		if ( strcmp_P( strupr(argv[2]), PSTR("PWR\0")) == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON\0")) == 0 ) {
				// Para que pueda constestar!!!
				xCOMMS_stateVars.gprs_prendido = true;
				IO_set_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF\0")) == 0 ) {
				IO_clr_GPRS_PWR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if ( strcmp_P( strupr(argv[2]), PSTR("SW\0"))  == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON\0")) == 0 ) {
				IO_set_GPRS_SW();
				pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF\0")) == 0 ) {
				IO_clr_GPRS_SW(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		if ( strcmp_P( strupr(argv[2]), PSTR("RTS\0")) == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON\0")) == 0 ) {
				IO_set_GPRS_RTS(); pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF\0")) == 0 ) {
				IO_clr_GPRS_RTS(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// Por ahora cableo DTR a CTS.

		if ( strcmp_P( strupr(argv[2]), PSTR("DTR\0")) == 0 ) {
			if ( strcmp_P( strupr(argv[3]), PSTR("ON\0")) == 0 ) {
				IO_set_GPRS_DTR(); pv_snprintfP_OK(); return;
			}
			if ( strcmp_P( strupr(argv[3]), PSTR("OFF\0")) == 0 ) {
				IO_clr_GPRS_DTR(); pv_snprintfP_OK(); return;
			}
			pv_snprintfP_ERR();
			return;
		}

		// write gprs monsqe
		if ( strcmp_P( strupr(argv[2]), PSTR("MONSQE\0")) == 0 ) {
			SPX_SEND_SIGNAL( SGN_MON_SQE );
			return;
		}
		// write gprs redial
		if ( strcmp_P( strupr(argv[2]), PSTR("REDIAL\0")) == 0 ) {
			SPX_SEND_SIGNAL( SGN_REDIAL );
			return;
		}

		// ATCMD
		// write gprs cmd {atcmd}
		if ( strcmp_P(strupr(argv[2]), PSTR("CMD\0")) == 0 ) {
			//xprintf_P( PSTR("%s\r\0"),argv[3] );

			gprs_flush_RX_buffer();
			if (strcmp_P( argv[3], PSTR("+++")) == 0 ) {
				xfprintf_P( fdGPRS,PSTR("%s"),argv[3] );
			} else {
				xfprintf_P( fdGPRS,PSTR("%s\r\0"),argv[3] );
			}

			xprintf_P( PSTR("sent->%s\r\n\0"),argv[3] );
			return;
		}

		return;
	}


	if ( cmd_mode == RD_CMD ) {
		// read gprs (rsp,cts,dcd,ri)

		// SMS
		// read gprs sms
		if ( strcmp_P(strupr(argv[2]), PSTR("SMS\0")) == 0 ) {
//C			u_gprs_sms_rxcheckpoint();
			return;
		}

		// ATCMD RSP
		// read gprs rsp
		if ( strcmp_P(strupr(argv[2]), PSTR("RSP\0")) == 0 ) {
			gprs_print_RX_buffer();
			//p = pub_gprs_rxbuffer_getPtr();
			//xprintf_P( PSTR("rx->%s\r\n\0"),p );
			return;
		}

		// DCD
		if ( strcmp_P( strupr(argv[2]), PSTR("DCD\0")) == 0 ) {
			pin = IO_read_DCD();
			xprintf_P( PSTR("DCD=%d\r\n\0"),pin);
			pv_snprintfP_OK();
			return;
		}

		// RI
		if ( strcmp_P( strupr(argv[2]), PSTR("RI\0")) == 0 ) {
			pin = IO_read_RI();
			xprintf_P( PSTR("RI=%d\r\n\0"),pin);
			pv_snprintfP_OK();
			return;
		}

		// CTS
		if ( strcmp_P( strupr(argv[2]), PSTR("CTS\0")) == 0 ) {
			pin = IO_read_CTS();
			xprintf_P( PSTR("CTS=%d\r\n\0"),pin);
			pv_snprintfP_OK();
			return;
		}

		if ( strcmp_P( strupr(argv[2]), PSTR("STATUS\0")) == 0 ) {
			pin = IO_read_CTS();
			xprintf_P( PSTR("CTS=%d\r\n\0"),pin);
			pin = IO_read_DCD();
			xprintf_P( PSTR("DCD=%d\r\n\0"),pin);
			pin = IO_read_RI();
			xprintf_P( PSTR("RI=%d\r\n\0"),pin);

			xprintf_P( PSTR("CTRLA=0X%02x\r\n\0"),USARTE0.CTRLA);
			xprintf_P( PSTR("CTRLB=0X%02x\r\n\0"),USARTE0.CTRLB);
			xprintf_P( PSTR("BAUDCTRLA=0X%02x\r\n\0"),USARTE0.BAUDCTRLA);
			xprintf_P( PSTR("BAUDCTRLB=0X%02x\r\n\0"),USARTE0.BAUDCTRLB);
			xprintf_P( PSTR("DIRSET=0X%02x\r\n\0"),PORTE.DIRSET);
			xprintf_P( PSTR("DIRCLR=0X%02x\r\n\0"),PORTE.DIRCLR);

			pv_snprintfP_OK();
			return;
		}

		pv_snprintfP_ERR();
		return;
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_I2Cscan(bool busscan)
{

bool retS = false;
uint8_t i2c_address;


	// Scan de una direccion
	if ( busscan == false ) {
		i2c_address = atoi(argv[2]);
		retS = I2C_scan_device(i2c_address);
		if (retS) {
			xprintf_P( PSTR("I2C device found at 0x%02x\r\n\0"), i2c_address );
		} else {
			xprintf_P( PSTR("I2C device NOT found at 0x%02x\r\n\0"), i2c_address );
		}
		return;
	}

	// Scan de todo el bus.00..FF.
	// Solo muestro las direcciones donde encuentro un device.
	for ( i2c_address = 0x00; i2c_address < 0xFF; i2c_address++ ) {
		retS = I2C_scan_device(i2c_address);
		if (retS) {
			xprintf_P( PSTR("I2C device found at 0x%02x\r\n\0"), i2c_address );
		};
	}

}
//------------------------------------------------------------------------------------
static bool pv_cmd_configGPRS(void)
{

	// config gprs SAT {check|enable|disable}
	if ( strcmp_P( strupr(argv[2]), PSTR("SAT\0")) == 0 ) {

		if ( strcmp_P( strupr(argv[3]), PSTR("DISABLE\0")) == 0 ) {
			return( gprs_SAT_set(0));
		}

		if ( strcmp_P( strupr(argv[3]), PSTR("ENABLE\0")) == 0 ) {
			return( gprs_SAT_set(1));
		}

		if ( strcmp_P( strupr(argv[3]), PSTR("CHECK\0")) == 0 ) {
			return( gprs_SAT_set(2));
		}
	}

	return(false);
}
//------------------------------------------------------------------------------------
static void pv_RTC_drift_test(void)
{

RtcTimeType_t rtc_new;

	RTC_str2rtc(argv[2], &rtc_new);
	if ( RTC_has_drift( &rtc_new, atoi(argv[3]) )) {
		xprintf_P(PSTR("DRIFT positivo\r\n"));
	} else {
		xprintf_P(PSTR("NO DRIFT\r\n"));
	}


}
//------------------------------------------------------------------------------------