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
#define OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED  1
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED      1

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


/*
typedef struct {
  fpga_converter_logic_pcb_rev_mismatch;
  fpga_firmware_major_rev_mismatch;
  fpga_firmware_minor_rev_mismatch;
  fpga_heater_voltage_less_than_4_5_volts;
  fpga_temp_greater_than_65_c;
  fpga_temp_greater_than_75_c;
  fpga_pulse_width_limiting;
  fpga_prf_fault;
  fpga_current_monitor_pulse_width;
  fpga_grid_module_hardware;
  fpga_grid_module_over_voltage;
  fpga_grid_module_under_voltage;
  fpga_grid_module_bias_voltage;
  fpga_hv_regulation_warn;
  fpga_dipswitch_1_on;
  fpga_test_mode_toggle_switch;
  fpga_local_mode_toggle_switch;
  
} TYPE_FAULT_COUNTERS;
*/



#define _FPGA_FIRMWARE_MAJOR_REV_MISMATCH              _FAULT_0
#define _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH         _STATUS_7
#define _FPGA_FIRMWARE_MINOR_REV_MISMATCH              _STATUS_7
#define _FPGA_ARC_COUNTER_GREATER_ZERO                 _STATUS_7
#define _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE          _STATUS_7
#define _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS       _STATUS_7
#define _FPGA_MODULE_TEMP_GREATER_THAN_65_C            _STATUS_7
#define _FPGA_MODULE_TEMP_GREATER_THAN_75_C            _STATUS_7
#define _FPGA_PULSE_WIDTH_LIMITING                     _STATUS_6
#define _FPGA_PRF_FAULT                                _FAULT_1
#define _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT        _FAULT_1
#define _FPGA_GRID_MODULE_HARDWARE_FAULT               _FAULT_1
#define _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT           _FAULT_1
#define _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT          _FAULT_1
#define _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT           _FAULT_1
#define _FPGA_HV_REGULATION_WARNING                    _STATUS_5
#define _FPGA_DIPSWITCH_1_ON                           _STATUS_6
#define _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE        _STATUS_6
#define _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE      _STATUS_6






#define _FAULT_ADC_DIGITAL_WARMUP                      _FAULT_2
#define _FAULT_ADC_DIGITAL_WATCHDOG                    _FAULT_3
#define _FAULT_ADC_DIGITAL_ARC                         _FAULT_4
#define _FAULT_ADC_DIGITAL_OVER_TEMP                   _FAULT_5
#define _FAULT_ADC_DIGITAL_PULSE_WIDTH_DUTY            _FAULT_6
#define _FAULT_ADC_DIGITAL_GRID                        _FAULT_7
#define _FAULT_ADC_HV_I_MON_OVER_ABSOLUTE              _FAULT_8
#define _FAULT_ADC_GUN_I_PEAK_OVER_ABSOLUTE            _FAULT_9
#define _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE             _FAULT_A



#define _FAULT_HEATER_RAMP_TIMEOUT                     _FAULT_B







typedef struct {
  unsigned int filtered_reading;
  unsigned int accumulator;
  unsigned int filter_time;
} TYPE_DIGITAL_INPUT;







typedef struct {
  unsigned int watchdog_count_error;
  unsigned int control_state;
  unsigned int request_hv_enable;
  unsigned int request_beam_enable;
  unsigned int reset_active;



  unsigned int heater_start_up_attempts;       // This counts the number of times the heater has started up without successfully completing it's ramp up.

  unsigned int run_time_counter;              // This counts how long the unit has been running for.  It wraps every 11 minutes
  unsigned int fault_restart_counter;         // This counts the delay of the heater automatic restart
  unsigned int power_supply_startup_counter;  // This counts counts the ramp up time of the HV supply
  unsigned int heater_warm_up_time_counter;   // This counts the time of the heater warm up

  unsigned int heater_voltage_target;         // This is the targeted heater voltage set point
  unsigned int heater_ramp_interval;          // This counts the interval between heater ramp voltage changes



  unsigned int can_high_voltage_set_point;
  unsigned int can_pulse_top_set_point;
  unsigned int can_heater_voltage_set_point;






  // These are the off board DAC outputs
  AnalogOutput analog_output_high_voltage;
  AnalogOutput analog_output_top_voltage;
  AnalogOutput analog_output_heater_voltage;
  unsigned int dac_digital_hv_enable;
  unsigned int dac_digital_heater_enable;
  unsigned int dac_digital_top_enable;
  unsigned int dac_digital_trigger_enable;
  unsigned int dac_digital_watchdog_oscillator;

  // These are the on board DAC outputs
  AnalogOutput monitor_heater_voltage;
  AnalogOutput monitor_heater_current;
  AnalogOutput monitor_cathode_voltage;
  AnalogOutput monitor_grid_voltage;
  


  // Digital Data from the FPGA
  TYPE_DIGITAL_INPUT fpga_coverter_logic_pcb_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_firmware_major_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_firmware_minor_rev_mismatch;
  TYPE_DIGITAL_INPUT fpga_arc;
  TYPE_DIGITAL_INPUT fpga_arc_high_voltage_inihibit_active;
  TYPE_DIGITAL_INPUT fpga_heater_voltage_less_than_4_5_volts;
  TYPE_DIGITAL_INPUT fpga_module_temp_greater_than_65_C;
  TYPE_DIGITAL_INPUT fpga_module_temp_greater_than_75_C;
  TYPE_DIGITAL_INPUT fpga_pulse_width_limiting_active;
  TYPE_DIGITAL_INPUT fpga_prf_fault;
  TYPE_DIGITAL_INPUT fpga_current_monitor_pulse_width_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_hardware_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_over_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_under_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_grid_module_bias_voltage_fault;
  TYPE_DIGITAL_INPUT fpga_hv_regulation_warning;
  TYPE_DIGITAL_INPUT fpga_dipswitch_1_on;
  TYPE_DIGITAL_INPUT fpga_test_mode_toggle_switch_set_to_test;
  TYPE_DIGITAL_INPUT fpga_local_mode_toggle_switch_set_to_local;


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
  TYPE_DIGITAL_INPUT adc_digital_warmup_flt;
  TYPE_DIGITAL_INPUT adc_digital_watchdog_flt;
  TYPE_DIGITAL_INPUT adc_digital_arc_flt;
  TYPE_DIGITAL_INPUT adc_digital_over_temp_flt;
  TYPE_DIGITAL_INPUT adc_digital_pulse_width_duty_flt;
  TYPE_DIGITAL_INPUT adc_digital_grid_flt;
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


  unsigned int adc_read_error_count;
  unsigned int adc_read_error_test;
  unsigned int adc_read_ok;

  unsigned int dac_write_error_count;
  unsigned int dac_write_failure;
  unsigned int dac_write_failure_count;

} TYPE_GLOBAL_DATA_A36772;




extern TYPE_GLOBAL_DATA_A36772 global_data_A36772;

#define STATE_FAULT_HEATER_OFF               0x00
#define STATE_FAULT_HEATER_ON                0x10
#define STATE_FAULT_HEATER_FAILURE           0x20
#define STATE_START_UP                       0x30
#define STATE_WAIT_FOR_CONFIG                0x40
#define STATE_HEATER_RAMP_UP                 0x50
#define STATE_HEATER_WARM_UP                 0x60
#define STATE_HEATER_WARM_UP_DONE            0x70
#define STATE_POWER_SUPPLY_RAMP_UP           0x80
#define STATE_HV_ON                          0x90
#define STATE_BEAM_ENABLE                    0xA0


#endif
