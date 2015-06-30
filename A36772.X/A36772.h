/*
  -------------------------------------------
  This file contains configuration data specific to the A35975-000
  
  Dan Parker
  2012-06-09

  --------------------------------------------
*/

#ifndef __A35975_250_H
#define __A35975_250_H

#include <xc.h>
#include <libpic30.h>
#include <adc12.h>
#include <timer.h>
#include <spi.h>
#include "ETM.h"
#include "P1395_CAN_SLAVE.h"
#include "FIRMWARE_VERSION.h"
//#include "faults.h"

#define FCY_CLK                    10000000


// --------- Resource Summary  -----------------
/*
  Hardware Module Resource Usage

  CAN2   - Used/Configured by ETM CAN (optical CAN) - DPARKER NEED TO RECOMPILE LIBRARY TO USE CAN2
  Timer4 - Used/Configured by ETM CAN - Used to Time sending of messages (status update / logging data and such) 
  Timer5 - Used/Configured by ETM CAN - Used for detecting error on can bus

  SPI1   - Used/Configured by Gun Driver

  Timer1 - Used to generate 100ms Timer - DPARKER this apears to be used by multiple 100ms timing operations
  Timer2 - Used for 10msTicToc (route this over to Timer)

  ADC Module - Not used

*/




// --------- Compile Time Options -----------------

#define TEST_MODE_BYP_FIBER_OFF        1    /* don't turn off hv or trig because fiber off, for test only */

#define USE_ENGINEERING_UNIT_ON_GUN_DRIVER    1   /* use engineering units for all parameters on CAN */

//#define TEST_BYP_FPGA_FAULTS       1




// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set

// ----------------- DIGITAL INPUT PINS --------------- //
/*
  RA15 - optical HV enable1
  RA14 - optical HV enable2
  RA13 - test pulse good2
  RA12 - test pulse good1
  
  RB15 - CS DAC Enable 

  RB11 - optical Trig enable
  RB9  - optical test pulse enable


*/

//   ------------------  Digital Output Pins ---------------
/*

  
 */

//					         fedcba9876543210
#define A35975_TRISA_VALUE 0b1111000000000000 
#define A35975_TRISB_VALUE 0b1000101000000000 
#define A35975_TRISC_VALUE 0b0000000000000000 
#define A35975_TRISD_VALUE 0b0000000000000000 
#define A35975_TRISF_VALUE 0b0000000010000101 
#define A35975_TRISG_VALUE 0b0000000000000001

//------------------- GUN Driver Interface I/O ------------------------- //
#define PIN_CS_DAC                          _LATD13
#define OLL_PIN_CS_DAC_SELECTED             1

#define PIN_CS_ADC                          _LATD14
#define OLL_PIN_CS_ADC_SELECTED             1

#define PIN_CS_FPGA                         _LATD15
#define OLL_PIN_CS_FPGA_SELECTED            1


// LOGIC Output Pins
#define PIN_OPT_GD_READY                     _LATA10	   
#define OLL_OPT_GD_READY                     1

#define PIN_CAN_HV_ENABLE                    _LATB12	   
#define OLL_CAN_HV_ENABLE                    1

#define PIN_CAN_PULSETOP_ENABLE              _LATB10	   
#define OLL_CAN_PULSETOP_ENABLE              1

#define PIN_CAN_TRIGGER_ENABLE               _LATB8	   
#define OLL_CAN_TRIGGER_ENABLE               1


//#define PIN_OPT_CAN_XMIT_OUT                 _LATG1		   
//#define OLL_OPT_CAN_XMIT_ON                  1


// LOGIC Input Pins
//#define PIN_CS_DAC_ENABLE_INPUT            _RB15		   // was designed for DAC output 


#ifndef DEMO
#define PIN_OPT_HV_ENABLE1_INPUT             _RA15	   
#define ILL_OPT_HV_ENABLE                    1

#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

#define PIN_OPT_TRIG_ENABLE_INPUT            _RB11
#define ILL_OPT_TRIG_ENABLE                  1

#define PIN_OPT_TEST_PULSE_ENABLE_INPUT       _RB9	   
#define ILL_OPT_TEST_PULSE_ENABLE             1

#else
#define PIN_OPT_HV_ENABLE1_INPUT             1	   
#define ILL_OPT_HV_ENABLE                    1

#define PIN_OPT_HV_ENABLE2_INPUT             _RA14	   // A14 and A15 inputs are tied together

#define PIN_OPT_TRIG_ENABLE_INPUT              1
#define ILL_OPT_TRIG_ENABLE                    1

#define PIN_OPT_TEST_PULSE_ENABLE_INPUT        1	   
#define ILL_OPT_TEST_PULSE_ENABLE              1
#endif

#define PIN_TEST_PULSE_GOOD1_INPUT           _RA12
#define ILL_TEST_PULSE_GOOD                  1

#define PIN_TEST_PULSE_GOOD2_INPUT           _RA13	   // A12 and A13 inputs are tied together



// MCP4822 DAC Output Pins
#define PIN_CS_MCP4822_ENABLE                _LATB14	   
#define OLL_CS_MCP4822_ENABLE                0

#define PIN_LDAC_MCP4822_ENABLE              _LATB13	   
#define OLL_LDAC_MCP4822_ENABLE              0



// UART TX enable
#define PIN_RS422_DE                         _LATF4
#define OLL_RS422_DE_ENABLE_RS422_DRIVER     1




// LED Indicator Output Pins
#define OLL_LED_ON                            0

#define PIN_LED_24DC_OK                      _LATG14	   

#define PIN_LED_LAST_PULSE_GOOD              _LATG12	   

#define PIN_LED_GD_READY                     _LATG13	   

#define PIN_LED_HV_ENABLE                    _LATG15   

#define PIN_LED_AC_ON                        _LATC1   

#define PIN_LED_LAST_PULSE_FAIL              _LATC2  
//#define TRIS_PIN_LED_LAST_PULSE_FAIL         _TRISC2		   
//#define OLL_LED_LAST_PULSE_FAIL              0

#define PIN_LED_WARMUP                       _LATC3   

#define PIN_LED_SUM_FAULT                    _LATC4   


// -----------------------  END IO PIN CONFIGURATION ------------------------ //


/* ------------------------------ CLOCK AND TIMING CONFIGURATION ------------------------- */
//#define FCY_CLK                    10000000      // 29.495 MHz   defined in ETM CAN
//#define FCY_CLK_MHZ                10.000        // 29.495 MHz   defined in ETM CAN

#define UART1_BAUDRATE             124000        // U1 Baud Rate
#define I2C_CLK                    100000        // Target I2C Clock frequency of 100KHz



// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//


/*
  --- SPI1 Port --- 
  This SPI port is used to connect with the gun driver
  The prescales of 16:1 and 1:1 will generate a clock = Fcy/16.  In this case 1.843MHz
  This must be slower to compensate for the 2x delay across the optocoupler 200ns with filtering in one direction, 80ns (without filtering) in the other direction
  Minimum clock period is therefore 280ns + holdtime + margins
*/
#define A35975_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A35975_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   
//#define A35975_SPI1CON_CLOCK (SEC_PRESCAL_4_1 & PRI_PRESCAL_4_1)
//#define A35975_SPI2CON_CLOCK (SEC_PRESCAL_2_1 & PRI_PRESCAL_1_1)



/*
  --- Timer1 Setup ---
  Period of 100mS
*/
#define A35975_T1CON_VALUE  (T1_ON & T1_IDLE_CON & T1_GATE_OFF & T1_PS_1_64 & T1_SOURCE_INT)
#define A35975_PR1_ROLL_US  100000      // 100mS
#define A35975_PR1_VALUE    ((FCY_CLK/1000000)*A35975_PR1_ROLL_US/64)


/*
  --- Timer2 Setup ---
  Period of 10mS
*/
#define A35975_T2CON_VALUE     (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define A35975_PR2_VALUE_US    10000   // 10mS
#define A35975_PR2_VALUE       ((FCY_CLK/1000000)*A35975_PR2_VALUE_US/8)

 
// ---- Hard Coded Delays ---- //
#define DELAY_TCY_10US                          100       // 10us us at 10MHz Clock

#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US  1        // 1uS
// This delay must be longer than the propogation delay on the Isolated DAC Cable Select Line
#define DELAY_PULSE_CABLE_SELECT_PROP_DELAY     ((FCY_CLK/1000000)*DELAY_PULSE_CABLE_SELECT_PROP_DELAY_US)



// ----------- Data Structures ------------ //

#define ANALOG_SET_SIZE   8
#define ANALOG_READ_SIZE  10


// analog set pointers
enum {
  ANA_SET_EK = 0,
  ANA_SET_EG,
  ANA_SET_EF,
  ANA_SET_HV_ON,
  ANA_SET_HTR_ON,
  ANA_SET_PULSETOP_ON,
  ANA_SET_TRIG_ON,
  ANA_SET_WDOG,
};





// analog read pointers
enum {
  ANA_RD_EK = 0,
  ANA_RD_IKA,
  ANA_RD_IKP,
  ANA_RD_EF,
  ANA_RD_IF,
  ANA_RD_EG, // grid V
  ANA_RD_EC, // bias V
  ANA_RD_24V, // 24DC
  ANA_RD_TEMP, // temperature
   
  ANA_RD_HTR_WARMUP,
  ANA_RD_WATCHDOG,
  ANA_RD_ARC,
  ANA_RD_OT,
  ANA_RD_PW_DUTY,
  ANA_RD_BIASFLT,
  ANA_RD_DA_FDBK,
};



#ifdef USE_ENGINEERING_UNIT_ON_GUN_DRIVER 

#define CAN_SCALE_TABLE_SIZE  13
// analog read pointers
enum {
  CAN_RD_EK = 0,
  //   CAN_RD_IKA,
  CAN_RD_IKP,
  CAN_RD_EF,
  CAN_RD_IF,
  CAN_RD_EG, // grid V
  CAN_RD_EC, // bias V
  //  CAN_RD_24V, // 24DC
  CAN_RD_TEMP, // temperature
   
  CAN_RD_EKSET,
  CAN_RD_EFSET,
  CAN_RD_EGSET,
  CAN_SET_EKSET,
  CAN_SET_EFSET,
  CAN_SET_EGSET,
   
};
/*
  Name 	      |   cal factor | offset|	CAN Interface |	CAN Unit/bit |	CAN Range|	CAN Scaling| ScaleFactor |Scale Offset
  --------------------------------------------------------------------------------------------------------------------------
  EK_RD		  |	  0.005555	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	5.555	   | 22753 		 | 0
  IKA_RD		  |	  0.001667	 | 0	 |	1 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
  IKP_RD		  |	  0.277		 | 0	 |	100 mA/bit    |    0.1		 |	6553.5	 |	2.77	   | 11345 		 | 0
  EF_RD		  |	  0.00222	 | 0	 |	1 mV/bit	  |  0.001		 |	65.535	 |	2.22	   | 9093  		 | 0
  IF_RD		  |	  0.001667	 | 0	 |	10 mA/bit	  |  0.001		 |	65.535	 |	1.667	   | 54624 		 | 0
  EG_RD		  |	  0.1111	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	1.111	   | 36405 		 | 0
  EC_RD		  |	  0.05555	 | 0	 |	100 mV/bit    |    0.1		 |	6553.5	 |	0.5555	   | 18202 		 | 0
  TEMP_RD		  |	  0.0133	 | 0	 |	0.01 C/bit    |   0.01		 |	655.35	 |	1.33	   | 43581 		 | 0
  |				 |		 |				  |				 |		  	 |			   |	   		 |	
  Ekset bits-val|	  0.0003333	 | 0	 |	1 V/bit	      |  0.001		 |	65.535	 |	0.3333	   | 10921 		 | 0
  Ekset val-bits|				 |		 |				  |				 |		  	 |	3.00030003 | 12289 		 | 0
  Efset bits-val|	  0.000133	 | 0	 |	10 mV/bit	  |  0.001		 |	65.535	 |	0.133	   | 4358  		 | 0
  Efset val-bits|				 |		 |				  |				 |		  	 |	7.518796992| 30796 		 | 0
  Egset bits-val|	  0.00666	 | 80	 |   100 mV/bit	  |    0.1		 |	6553.5	 |	0.0666	   | 2182  		 | 0
  Egset val-bits|				 |		 |				  |				 |			 |	15.01501502| 61501 		 | 0  


  Note:  Scale offset for EG read/set is handled by GUI.
			  
*/			  

#define CAL_EK_RD    0.005555
#define CAL_IKA_RD   0.001667
#define CAL_IKP_RD   0.277
#define CAL_EF_RD    0.00222
#define CAL_IF_RD    0.001667
#define CAL_EG_RD    0.1111
#define CAL_EC_RD	 0.05555
#define CAL_TEMP_RD	 0.0133

#define CAL_EKSET    0.0003333
#define CAL_EFSET    0.000133
#define CAL_EGSET    0.00666

#define CAN_EK_SCALE     0.001
#define CAN_IKA_SCALE 	 0.001	
#define CAN_IKP_SCALE 	 0.1	
#define CAN_EF_SCALE 	 0.001	
#define CAN_IF_SCALE 	 0.001	
#define CAN_EG_SCALE 	 0.1	
#define CAN_EC_SCALE 	 0.1	
#define CAN_TEMP_SCALE 	 0.01	
								
#define CAN_EKSET_SCALE  0.001
#define CAN_EFSET_SCALE  0.001
#define CAN_EGSET_SCALE  0.1
					 
					 




#endif /* USE_ENGINEERING_UNIT_ON_GUN_DRIVER */




#define FAULT_SIZE  22  /* 6 ADC + 16 FPGA_ID */

//#define DIGI_ADC_HTR_FLT_WARMUP    0
#define DIGI_ADC_FPGA_WATCHDOG     1
#define DIGI_ADC_ARC      		   2
//#define DIGI_ADC_TEMP			   3
//#define DIGI_ADC_PW_DUTY  		   4
//#define DIGI_ADC_BIAS_TOP          5

#define DIGI_ID_ARC_COUNT          6
//#define DIGI_ID_ARC_HV_INHIBIT     7
//#define DIGI_ID_EF_LESS_4p5V       8
//#define DIGI_ID_TEMP_65C		   9
#define DIGI_ID_TEMP_75C		   10
#define DIGI_ID_PW_LIMITING        11
#define DIGI_ID_PRF			       12
#define DIGI_ID_CURR_PW		       13

#define DIGI_ID_GRID_HW		  	   14
#define DIGI_ID_GRID_OV		       15
#define DIGI_ID_GRID_UV		       16
#define DIGI_ID_BIAS_V			   17
//#define DIGI_ID_HV_REGULATION      18
#define DIGI_ID_DIP_SW             19
#define DIGI_ID_TEST_MODE		   20
#define DIGI_ID_LOCAL_MODE         21
 





/*
  --- LOGIC  STATE DEFINITIONS ---
  See flow diagram for more information
  DPARKER add flow diagram doc number
*/


/* 
   --- SYSTEM STATE BYTE DEFINITIONS --- 
*/
#define SYS_BYTE_HTR_ON						 0x01
#define SYS_BYTE_LOGIC_READY  				 0x02
#define SYS_BYTE_HV_ON						 0x04
#define SYS_BYTE_PULSETOP_ON    		     0x08

#define SYS_BYTE_TRIG_ON					 0x10
#define SYS_BYTE_FAULT_ACTIVE                0x20
#define SYS_BYTE_HTR_WARMUP				     0x40  /* htr off or warmup */
#define SYS_BYTE_HV_DRIVEUP				     0x80

/*
  --- Public Functions ---
*/


#define SYSTEM_WARM_UP_TIME      3000 /* 100ms Units  //DPARKER this is way to short */
#define EF_READ_MAX              2838 /*  -6.3/-0.00222 */
#define IF_READ_MAX              1051 /*  1.75/0.001666 */
//#define IF_READ_MAX_95P          1824 // 95% of IF_MAX
//#define IF_READ_MAX_85P          1633 // 85% of IF_MAX

#define IF_READ_MAX_90P           945 /* 90% of IF_MAX */

#define EF_SET_MAX              47369 /*  -6.3/-0.000133  */
#define EG_SET_MAX              33033 /* 140V, 0.00666    */
#define EK_SET_MAX              60060 /* -20kV/-0.000333  */
 




#define _STATUS_GD_HV_DISABLE                           _STATUS_0	
#define _STATUS_GD_HTR_NOT_READY                        _STATUS_1
#define _STATUS_GD_TRIG_NOT_ENABLED                     _STATUS_2
#define _STATUS_GD_TOP_NOT_ENABLED                      _STATUS_3
#define _STATUS_GD_HV_NOT_ENABLED    			_STATUS_4
#define _STATUS_GD_HTR_NOT_ENABLED                      _STATUS_5	



#define _FAULT_GD_SUM_FAULT                             _FAULT_0
#define _FAULT_GD_FPGA_COMM_LOST                        _FAULT_1
#define _FAULT_GD_SW_HTR_OVOC                           _FAULT_2
#define _FAULT_GD_SW_BIAS_UV                            _FAULT_3
#define _FAULT_GD_SW_EK_OV                              _FAULT_4
#define _FAULT_GD_SW_EK_UV                              _FAULT_5
#define _FAULT_GD_SW_GRID_OV                            _FAULT_6
#define _FAULT_GD_FPGA_TEMP_75C                         _FAULT_7
#define _FAULT_CAN_COMMUNICATION_LATCHED                _FAULT_8
#define _FAULT_GD_FPGA_ARC_FAULT                        _FAULT_9
#define _FAULT_GD_FPGA_PULSE_FAULT                      _FAULT_A
#define _FAULT_GD_FPGA_GRID_FAULT                       _FAULT_B
#define _FAULT_GD_SW_HTR_UV                             _FAULT_C
#define _FAULT_GD_SW_24V_FAULT                          _FAULT_D
#define _FAULT_GD_SYS_FAULTS                            _FAULT_E


#define _FAULT_PIC_HEATER_TURN_OFF                      _FAULT_0



typedef struct {
  unsigned converter_logic_pcb_rev:6;
  unsigned fpga_firmware_major_rev:4;
  unsigned fpga_firmware_minor_rev:6;
  unsigned arc:1;
  unsigned arc_high_voltage_inihibit_active:1;
  unsigned heater_voltage_greater_than_4_5_volts:1;
  unsigned module_temp_greater_than_65_C:1;
  unsigned module_temp_greater_than_75_C:1;
  unsigned pulse_width_limiting_active:1;
  unsigned prf_fault:1;
  unsigned current_monitor_pulse_width_fault:1;
  unsigned grid_module_hardware_fault:1;
  unsigned grid_module_over_voltage_fault:1;
  unsigned grid_module_under_voltage_fault:1;
  unsigned grid_module_bias_voltage_fault:1;
  unsigned hv_regulation_warning:1;
  unsigned dipswitch_1_on:1;
  unsigned test_mode_toggle_switch_set_to_test:1;
  unsigned local_mode_toggle_switch_set_to_local:1;
} TYPE_FPGA_DATA;

typedef struct {
  unsigned int watchdog_count_error;
  unsigned int control_state;
  unsigned int start_up_counter;
  unsigned int led_flash_counter;
  unsigned int power_supply_startup_up_counter;

  unsigned int heater_voltage_target;   // This is the targeted heater voltage set poing
  unsigned int heater_ramp_counter;


  // These are the DAC outputs
  AnalogOutput analog_output_high_voltage;
  AnalogOutput analog_output_top_voltage;
  AnalogOutput analog_output_heater_voltage;
  unsigned int dac_digital_hv_enable;
  unsigned int dac_digital_heater_enable;
  unsigned int dac_digital_top_enable;
  unsigned int dac_digital_trigger_enable;
  unsigned int dac_digital_watchdog_oscillator;

  // These are the ADC inputs
  AnalogInput  input_adc_temperature;
  AnalogInput  input_hv_v_mon;
  AnalogInput  input_hv_i_mon;
  AnalogInput  input_gun_i_peak;
  AnalogInput  input_htr_v_mon;
  AnalogInput  input_htr_i_mon;
  AnalogInput  input_top_v_mon;
  AnalogInput  input_bias_v_mon;
  AnalogInput  input_24_v_mon;
  AnalogInput  input_temperature_mon;
  unsigned int adc_digital_warmup_flt;
  unsigned int adc_digital_watchdog_flt;
  unsigned int adc_digital_arc_flt;
  unsigned int adc_digital_over_temp_flt;
  unsigned int adc_digital_pulse_width_duty_flt;
  unsigned int adc_digital_grid_flt;
  AnalogInput  input_dac_monitor;


  TYPE_FPGA_DATA fpga_data;
  unsigned int adc_read_error_count;
  unsigned int adc_read_error_test;

  unsigned int dac_write_error_count;
  unsigned int dac_write_failure;
  unsigned int dac_write_failure_count;

} TYPE_GLOBAL_DATA_A35975_250;


extern TYPE_GLOBAL_DATA_A35975_250 global_data_A35975_250;

#define STATE_START_UP                       0x10
#define STATE_WAIT_FOR_CONFIG                0x20
#define STATE_HEATER_RAMP_UP                 0x30
#define STATE_HEATER_RAMP_UP_DONE            0x40
#define STATE_POWER_SUPPLY_RAMP_UP           0x50
#define STATE_HV_ON                          0x60
#define STATE_FAULT_HEATER_OFF               0x70
#define STATE_FAULT_HEATER_ON                0x80
#define STATE_FAULT_HEATER_FAILURE           0x90

#endif
