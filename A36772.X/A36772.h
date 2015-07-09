/*
  -------------------------------------------
  This file contains configuration data specific to the A36772-000
  
  Dan Parker
  2012-06-09

  --------------------------------------------
*/

#ifndef __A36772_H
#define __A36772_H

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
  SPI2   - Used by local DAC 

  Timer2 - Used for 10msTicToc 

  ADC Module - AN3,AN4,AN5,AN6,AN7,VREF+,VREF-,AN13,AN14,AN15

*/





// ----------------- IO PIN CONFIGURATION -------------------- //
// All unused pins will be set to outputs and logic zero
// LAT values default to 0 at startup so they do not need to be manually set


// Pins to be configured as inputs


/*
  RA9  - ADC VREF-
  RA10 - ADC VREF+
  RA15 - PIN_CUSTOMER_HV_ON
 
  RB0  - ICD  - PROGRAM
  RB1  - ICD  - PROGRAM
  RB3  - AN3  - POT EK
  RB4  - AN4  - POT VTOP
  RB5  - AN5  - POT HTR
  RB6  - AN6  - REF HTR
  RB7  - AN7  - REF VTOP
  RB13 - AN13 - REF EK
  RB14 - AN14 - PIC ADC +15V MON
  RB15 - AN15 - PIC ADC -15V MON

  RC1  - DAC LDAC  (Configured by DAC module)

  RD8  - PIN_CUSTOMER_BEAM_ENABLE


  RF0  - CAN 1 (Configured By Pic Module)
  RF1  - CAN 1 (Configured By Pic Module)
  RF2  - UART 1 (Configured By Pic Module)
  RF3  - UART 1 (Configured By Pic Module)
  RF6  - SPI 1 (Configured By Pic Module)
  RF7  - SPI 1 (Configured By Pic Module)
  RF8  - SPI 1 (Configured By Pic Module)


  RG0  - CAN 2 (Configured By Pic Module)
  RG1  - CAN 2 (Configured By Pic Module)
  RG2  - I2C   (Configured By Pic Module)
  RG3  - I2C   (Configured By Pic Module)
  RG6  - SPI2 CLK
  RG7  - SPI2 DI
  RG8  - SPI2 DO
  RG14 - Reset Detect
  RG15 - DAC CS/LD (Configured by DAC module)

*/




/*
  This sets up the ADC to work as following
  AUTO Sampeling
  External Vref+/Vref-
  With 10MHz System Clock, ADC Clock is 450ns, Sample Time is 6 ADC Clock so total sample time is 9.0uS
  8 Samples per Interrupt, use alternating buffers
  Conversion rate of 111KHz (13.875 Khz per Channel), 138 Samples per 10mS interrupt
  Scan Through Selected Inputs (8 selected at any point in time)

*/

#define ADCON1_SETTING          (ADC_MODULE_OFF & ADC_IDLE_STOP & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON)
#define ADCON2_SETTING          (ADC_VREF_EXT_EXT & ADC_SCAN_ON & ADC_SAMPLES_PER_INT_8 & ADC_ALT_BUF_ON & ADC_ALT_INPUT_OFF)
#define ADCON3_SETTING          (ADC_SAMPLE_TIME_4 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_9Tcy2)
#define ADCHS_SETTING           (ADC_CH0_POS_SAMPLEA_AN3 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN3 & ADC_CH0_NEG_SAMPLEB_VREFN)
#define ADPCFG_SETTING          (ENABLE_AN3_ANA & ENABLE_AN4_ANA & ENABLE_AN5_ANA & ENABLE_AN6_ANA & ENABLE_AN7_ANA & ENABLE_AN13_ANA & ENABLE_AN14_ANA & ENABLE_AN15_ANA)

#define ADCSSL_SETTING          (SKIP_SCAN_AN0 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 & SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12)




#define A36772_TRISA_VALUE 0b1000011000000000 
#define A36772_TRISB_VALUE 0b1110000011111011 
#define A36772_TRISC_VALUE 0b0000000000000010 
#define A36772_TRISD_VALUE 0b0000000100000000 
#define A36772_TRISF_VALUE 0b0000000111001111 
#define A36772_TRISG_VALUE 0b1100000111001111 




// Digital Inputs
#define PIN_CUSTOMER_HV_ON                  _RA15
#define ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV    1

#define PIN_CUSTOMER_BEAM_ENABLE            _RD8
#define ILL_PIN_CUSTOMER_BEAM_ENABLE_BEAM_ENABLED 1

//------------------- GUN Driver Interface I/O ------------------------- //
#define PIN_CS_DAC                          _LATD13
#define OLL_PIN_CS_DAC_SELECTED             1

#define PIN_CS_ADC                          _LATD14
#define OLL_PIN_CS_ADC_SELECTED             1

#define PIN_CS_FPGA                         _LATD15
#define OLL_PIN_CS_FPGA_SELECTED            1



// Digital Outputs
#define PIN_CPU_WARMUP_STATUS               _LATD0
#define PIN_CPU_STANDBY_STATUS              _LATD11
#define PIN_CPU_HV_ON_STATUS                _LATD10
#define PIN_CPU_BEAM_ENABLE_STATUS          _LATD9  // DPARKER THERE IS ERROR ON SCHEMATIC
#define PIN_CPU_SYSTEM_OK_STATUS            _LATA15
#define PIN_CPU_EXTRA_STATUS                _LATD3

#define PIN_CPU_HV_ENABLE                   _LATD2
#define PIN_CPU_BEAM_ENABLE                 _LATD8

#define PIN_RS485_ENABLE                    _LATF4  // DPARKER THERE IS ERROR ON SCHEMATIC


// LED Indicator Output Pins
#define OLL_LED_ON                           0

#define PIN_LED_I1B                          _LATG12
#define PIN_LED_I1C                          _LATG13
#define PIN_LED_I1D                          _LATA7

#define PIN_LED_I2A                          _LATC2
#define PIN_LED_I2B                          _LATC3
#define PIN_LED_I2C                          _LATC4
#define PIN_LED_I2D                          _LATA6

#define PIN_RESET_DETECT_OUTPUT              _LATG14
#define PIN_RESET_DETECT_INPUT               _RG14

#define PIN_TEST_POINT_B                     _LATF5
#define PIN_TEST_POINT_E                     _LATB8
#define PIN_TEST_POINT_F                     _LATB9


#define PIN_LED_AC_ON                        PIN_LED_I2A
#define PIN_LED_LAST_PULSE_FAIL              PIN_LED_I2B
#define PIN_LED_WARMUP                       PIN_LED_I2C
#define PIN_LED_SUM_FAULT                    PIN_LED_I2D



// -----------------------  END IO PIN CONFIGURATION ------------------------ //



// -------------------------------------------- INTERNAL MODULE CONFIGURATION --------------------------------------------------//

/*
  --- SPI1 Port --- 
  This SPI port is used to connect with the gun driver
  This must be slower to compensate for the 2x delay across the optocoupler 200ns with filtering in one direction, 80ns (without filtering) in the other direction
  Minimum clock period is therefore 280ns + holdtime + margins
*/
#define A36772_SPI1CON_VALUE  (FRAME_ENABLE_OFF & ENABLE_SDO_PIN & SPI_MODE16_OFF & SPI_SMP_OFF & SPI_CKE_OFF & SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH & MASTER_ENABLE_ON)
#define A36772_SPI1STAT_VALUE (SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR)   


/*
  --- Timer2 Setup ---
  Period of 10mS
*/
#define A36772_T2CON_VALUE     (T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_8 & T2_32BIT_MODE_OFF & T2_SOURCE_INT)
#define A36772_PR2_VALUE_US    10000   // 10mS
#define A36772_PR2_VALUE       ((FCY_CLK/1000000)*A36772_PR2_VALUE_US/8)

 
// ---- Hard Coded Delays ---- //
#define DELAY_FPGA_CABLE_DELAY 10


#if(0)

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
 
#endif



#define _STATUS_GD_HV_DISABLE                           _STATUS_0	
#define _STATUS_GD_HTR_NOT_READY                        _STATUS_1
#define _STATUS_GD_TRIG_NOT_ENABLED                     _STATUS_2
#define _STATUS_GD_TOP_NOT_ENABLED                      _STATUS_3
#define _STATUS_GD_HV_NOT_ENABLED    			_STATUS_4
#define _STATUS_GD_HTR_NOT_ENABLED                      _STATUS_5	


//#define _STATUS_HV_DISABLE
//#define _STATUS_



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
  unsigned int run_time_counter;
  unsigned int power_supply_startup_up_counter;

  unsigned int heater_voltage_target;   // This is the targeted heater voltage set poing
  unsigned int heater_ramp_interval;
  unsigned int heater_warm_up_time_counter;

  unsigned int request_hv_enable;
  unsigned int request_beam_enable;

  unsigned int can_high_voltage_set_point;
  unsigned int can_pulse_top_set_point;
  unsigned int can_heater_voltage_set_point;




  // These are the off board DAC outputs
  AnalogOutput analog_output_high_voltage;
  AnalogOutput analog_output_top_voltage;
  AnalogOutput analog_output_heater_voltage;
  //unsigned int dac_digital_hv_enable;
  //unsigned int dac_digital_heater_enable;
  //unsigned int dac_digital_top_enable;
  //unsigned int dac_digital_trigger_enable;
  //unsigned int dac_digital_watchdog_oscillator;

  // These are the on board DAC outputs
  AnalogOutput monitor_heater_voltage;
  AnalogOutput monitor_heater_current;
  AnalogOutput monitor_cathode_voltage;
  AnalogOutput monitor_grid_voltage;
  


  // These are the ADC input from the external device on the SPI BUS
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

  // These are the anlog input from the PICs internal DAC
  AnalogInput  pot_htr;     // an3
  AnalogInput  pot_vtop;    // an4
  AnalogInput  pot_ek;      // an5
  AnalogInput  ref_htr;     // an6
  AnalogInput  ref_vtop;    // an7
  AnalogInput  ref_ek;      // an13
  AnalogInput  pos_15v_mon; // an14
  AnalogInput  neg_15v_mon; // an15
  
  unsigned int accumulator_counter;



  TYPE_FPGA_DATA fpga_data;
  unsigned int adc_read_error_count;
  unsigned int adc_read_error_test;

  unsigned int dac_write_error_count;
  unsigned int dac_write_failure;
  unsigned int dac_write_failure_count;

} TYPE_GLOBAL_DATA_A36772;


extern TYPE_GLOBAL_DATA_A36772 global_data_A36772;

#define STATE_START_UP                       0x10
#define STATE_WAIT_FOR_CONFIG                0x20
#define STATE_HEATER_RAMP_UP                 0x30
#define STATE_HEATER_WARM_UP                 0x38
#define STATE_HEATER_WARM_UP_DONE            0x40
#define STATE_POWER_SUPPLY_RAMP_UP           0x50
#define STATE_HV_ON                          0x60
#define STATE_FAULT_HEATER_OFF               0x70
#define STATE_FAULT_HEATER_ON                0x80
#define STATE_FAULT_HEATER_FAILURE           0x90

#endif
