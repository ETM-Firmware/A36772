#ifndef __A36772_CONFIG_H
#define __A36772_CONFIG_H

//-----------------------Specific board selections---------------------
//#define ZAP_A36772_000
//#define Sameer_A36772_100
//#define MD51_A36772_150
//#define Xcision_A36772_200
//#define TecHaus_A36772_300
//#define RTX_A36772_600_2AMPS
//#define RTX_A36772_600_4AMPS
#define Intraop_A36772_700

// Make sure that at least one board is selected
#ifndef ZAP_A36772_000
#ifndef Sameer_A36772_100
#ifndef MD51_A36772_150
#ifndef Xcision_A36772_200
#ifndef TecHaus_A36772_300
#ifndef RTX_A36772_600_2AMPS
#ifndef RTX_A36772_600_4AMPS
#ifndef Intraop_A36772_700
#error "No Specific Board Selected"
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif

#ifdef ZAP_A36772_000
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000        // 5.0min.  If the heater does not reach it's programed voltage in this time a fault will be generated.
#define MAX_PROGRAM_HTR_VOLTAGE                 7000         // 7.0 V.  Limit on what customer can set
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       1650         // 1.650 Amps
#define HTR_OC_ABS                              1750         // 1.750 Amps
#define HV_MAX_SET_BOARD_SPEC                   20000        // -20KV
#define HV_MIN_SET_BOARD_SPEC                   5000         // -5V
#define TOP_MAX_SET_BOARD_SPEC                  40000        // 320V
#define TOP_MIN_SET_BOARD_SPEC                  4000         // -40V
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 50V Eg
#define REF_VTOP_OFFSET_SELECTED                0
#define REF_EK_SCALE_SELECTED                   .33784       // 0.37V = -1kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       1.9733       // 0.37V = -1kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.0667       // 1V = 50V Eg
#define REF_HTR_SCALE_SELECTED                  .15625
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     0
#define BOARD_DASH_NUMBER                       000
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  Sameer_A36772_100
#error "Multiple boards selected"
#endif
#ifdef  MD51_A36772_150
#error "Multiple boards selected"
#endif
#ifdef  Xcision_A36772_200
#error "Multiple boards selected"
#endif
#ifdef  TecHaus_A36772_300
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_2AMPS
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif

#ifdef Sameer_A36772_100
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 60000        // 10min    
#define MAX_PROGRAM_HTR_VOLTAGE                 8000         // 8.000 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       5200         // 5.200 Amps  
#define HTR_OC_ABS                              5500         // 5.500 Amps 
#define HV_MAX_SET_BOARD_SPEC                   20000        // -20KV
#define HV_MIN_SET_BOARD_SPEC                   0            // 0V
#define TOP_MAX_SET_BOARD_SPEC                  40000        // 320V
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V
#define BIAS_OVER_VOLTAGE                       23000        // -230V
#define BIAS_UNDER_VOLTAGE                      17000        // -170V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 40V above -100V Eg
#define REF_VTOP_OFFSET_SELECTED                -3200
#define REF_EK_SCALE_SELECTED                   .31250       // 1V = 2kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      5.3333      // 1V = 1A If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       2.6667       // 1V = 2kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.0667       // 1V = 50V above -100V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     2000         //  ?
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       100
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        0
#ifdef  MD51_A36772_150
#error "Multiple boards selected"
#endif
#ifdef  Xcision_A36772_200
#error "Multiple boards selected"
#endif
#ifdef  TecHaus_A36772_300
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_2AMPS
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif

#ifdef MD51_A36772_150
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 7000         // 7.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       1900         // 1.900 Amps
#define HTR_OC_ABS                              2400         // 2.400 Amps
#define HV_MAX_SET_BOARD_SPEC                   15000        // -15KV    -13kV needed
#define HV_MIN_SET_BOARD_SPEC                   5000         // -5kV     -10kV needed
#define TOP_MAX_SET_BOARD_SPEC                  22000        // 140V     +50V needed
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V     -30V needed
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 50V Eg
#define REF_VTOP_OFFSET_SELECTED                0
#define REF_EK_SCALE_SELECTED                   .33784       // 0.37V = -1kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       1.9733       // 0.37V = -1kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.0667       // 1V = 50V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     0
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       150
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  Xcision_A36772_200
#error "Multiple boards selected"
#endif
#ifdef  TecHaus_A36772_300
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_2AMPS
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif


#ifdef Xcision_A36772_200
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 7000         // 7.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       1800         // 1.800 Amps
#define HTR_OC_ABS                              2000         // 2.000 Amps
#define HV_MAX_SET_BOARD_SPEC                   15000        // -15KV
#define HV_MIN_SET_BOARD_SPEC                   0            // 0V
#define TOP_MAX_SET_BOARD_SPEC                  22000        // 140V
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 50V Eg
#define REF_VTOP_OFFSET_SELECTED                0
#define REF_EK_SCALE_SELECTED                   .33784       // 0.37V = -1kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       1.9733       // 0.37V = -1kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.0667       // 1V = 50V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     0
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       200
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  TecHaus_A36772_300
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_2AMPS
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif


#ifdef TecHaus_A36772_300
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 8000         // 8.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       2850         // 2.850 Amps
#define HTR_OC_ABS                              3000         // 3.000 Amps
#define HV_MAX_SET_BOARD_SPEC                   20000        // -20KV
#define HV_MIN_SET_BOARD_SPEC                   5000         // -5KV
#define TOP_MAX_SET_BOARD_SPEC                  28000        // 200V
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 50V Eg
#define REF_VTOP_OFFSET_SELECTED                0
#define REF_EK_SCALE_SELECTED                   .33784       // 0.37V = -1kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       1.9733       // 0.37V = -1kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.0667       // 1V = 50V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     0
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       300
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  RTX_A36772_600_2AMPS
#error "Multiple boards selected"
#endif
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif


#ifdef RTX_A36772_600_2AMPS
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 8000         // 8.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       2000         // 2.000 Amps
#define HTR_OC_ABS                              2200         // 2.200 Amps
#define HV_MAX_SET_BOARD_SPEC                   20000        // -20KV
#define HV_MIN_SET_BOARD_SPEC                   5000         // -5KV
#define TOP_MAX_SET_BOARD_SPEC                  38000        // 300V
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .62500       // 1V = 40V above -100V Eg
#define REF_VTOP_OFFSET_SELECTED                -3200
#define REF_EK_SCALE_SELECTED                   .31250       // 1V = 2kV Ek 
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       2.6667       // 1V = 2kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.3333       // 1V = 40V above -100V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     2000         //  ?
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       600
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  RTX_A36772_600_4AMPS
#error "Multiple boards selected"
#endif
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif


#ifdef RTX_A36772_600_4AMPS
#define __MODE_MODBUS_INTERFACE
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 8000         // 8.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       4000         // 4.000 Amps
#define HTR_OC_ABS                              4200         // 4.200 Amps
#define HV_MAX_SET_BOARD_SPEC                   20000        // -20KV.
#define HV_MIN_SET_BOARD_SPEC                   5000         // -5KV
#define TOP_MAX_SET_BOARD_SPEC                  38000        // 300V
#define TOP_MIN_SET_BOARD_SPEC                  0            // -80V
#define BIAS_OVER_VOLTAGE                       18000        // -180V
#define BIAS_UNDER_VOLTAGE                      14000        // -140V
#define REF_VTOP_SCALE_SELECTED                 .77495       //.62500 prev w 0.4gain // 1V = 40V above -100V Eg
#define REF_VTOP_OFFSET_SELECTED                -2581        //-3200 previous
#define REF_EK_SCALE_SELECTED                   .38748       //.31250 prev w 0.4gain // 1V = 2kV Ek 
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       2.6667       // 1V = 2kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      1.3333       // 1V = 40V above -100V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     2000         //  ?
#define REF_HTR_SCALE_SELECTED                  .15625
#define BOARD_DASH_NUMBER                       600
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#ifdef  Intraop_A36772_700
#error "Multiple boards selected"
#endif
#endif


#ifdef Intraop_A36772_700
#define __MODE_DISCRETE_INTERFACE
#define __MODBUS_ENABLED
#define MAX_HEATER_RAMP_UP_TIME                 30000
#define MAX_PROGRAM_HTR_VOLTAGE                 7000         // 7.0 V
#define HEATER_VOLTAGE_MAX_SET_POINT            8000         // 8V
#define HEATER_VOLTAGE_MIN_SET_POINT            0            // 0V
#define MAX_HEATER_CURRENT_DURING_RAMP_UP       1650         // 1.650 Amps
#define HTR_OC_ABS                              1750         // 1.750 Amps
#define HV_MAX_SET_BOARD_SPEC                   20000        //-20KV
#define HV_MIN_SET_BOARD_SPEC                   5000         //-5KV
#define TOP_MAX_SET_BOARD_SPEC                  26000        //180V
#define TOP_MIN_SET_BOARD_SPEC                  0            //-80V
#define BIAS_OVER_VOLTAGE                       18000        //-180V
#define BIAS_UNDER_VOLTAGE                      14000        //-140V
#define REF_VTOP_SCALE_SELECTED                 .38748       // 1V = 20V above -100V Eg
#define REF_VTOP_OFFSET_SELECTED                -6400
#define REF_EK_SCALE_SELECTED                   .38748       // 1V = -2kV Ek
#define DAC_MON_HTR_CURRENT_SCALE_SELECTED      10.6667      // 1V = 500mA If
#define DAC_MON_EK_VOLTAGE_SCALE_SELECTED       2.6667       // 1V = -2kV Ek
#define DAC_MON_TOP_VOLTAGE_SCALE_SELECTED      2.6667       // 1V = 20V above -100V Eg
#define DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED     2000
#define REF_HTR_SCALE_SELECTED                  .19374
#define BOARD_DASH_NUMBER                       700
#define OLL_PIN_CPU_HV_ENABLE_HV_ENABLED        1
#endif


/*----------------------------------------------------------------------*/
/*
  We have a couple of compile time options (selected by board option):
  __MODE_CAN_INTERFACE        
    In this mode, the gun driver is controlled over the CAN interface.
    The discrete digital/analog is not used.
    R126 and R127 should be installed

  __MODE_POT_INTERFACE
    In this mode, the gun driver is controlled by discrete fiber or signal lines.
    The pulse top, high voltage, and heater references are generated from the on board pots
    R126 and R127 should not be installed 

  __MODE_DISCRETE_INTERFACE
    In this mode, the gun driver is controlled by discrete fiber or signal lines.
    The pulse top, high voltage, and heater references are generated from the external interface
    R126 and R127 should not be installed 

  __OPTION_ENABLE_CAN
    This is only valid for __MODE_POT_INTERFACE and __MODE_DISCRETE_INTERFACE
    This allows the CAN port to be used for test and debugging while operating in one of these modes

 */

// Make sure that at least one mode is selected
#ifndef __MODE_CAN_INTERFACE
#ifndef __MODE_POT_INTERFACE
#ifndef __MODE_DISCRETE_INTERFACE
#ifndef __MODE_MODBUS_INTERFACE
#error "No reference Source Selected"
#endif
#endif
#endif
#endif


// Create and check compile time options based on configuration above
#ifdef __MODE_CAN_INTERFACE
#define __CAN_CONTROLS
#define __CAN_ENABLED
#define __CAN_REFERENCE
#ifdef __OPTION_ENABLE_CAN
#error "OPTION_ENABLE_CAN not valid modifier to MODE_CAN_INTERFACE"
#endif
#endif

#ifdef __MODE_DISCRETE_INTERFACE
#define __DISCRETE_REFERENCE
#define __DISCRETE_CONTROLS
#ifdef  __CAN_REFERENCE
#error "Multiple references selected"
#endif
#ifdef  __MODBUS_REFERENCE
#error "Multiple references selected"
#endif
#endif

#ifdef __MODE_POT_INTERFACE
#define __POT_REFERENCE
#define __DISCRETE_CONTROLS
#ifdef  __CAN_REFERENCE
#error "Multiple references selected"
#endif
#ifdef  __DISCRETE_REFERENCE
#error "Multiple references selected"
#endif
#endif

#ifdef __MODE_MODBUS_INTERFACE
#define __MODBUS_REFERENCE
#define __MODBUS_CONTROLS
#endif

#ifdef __OPTION_ENABLE_CAN
#define __CAN_ENABLED
#endif


// ----------- Timers configurations - ALL Times are in 10ms Units --------------------
#define LED_STARTUP_FLASH_TIME                500      // Time LEDs will flash at startup
#define HEATER_AUTO_RESTART_TIME              500      // Time delay between a heater fault and when the heater gets restarted
#define HEATER_RAMP_UP_TIME_PERIOD            5        // Durring heater ramp up, the heater voltage will be increased every N 10ms (see HEATER_RAMP_UP_INCREMENT)
#define GUN_DRIVER_POWER_SUPPLY_STARTUP_TIME  100      // Wait this long between enabling High Voltage / Pulse Top / Bias and cheching that they are at correct values

// System control Parameters
#define MAX_CONVERTER_LOGIC_ADC_READ_ERRORS   20       // If the ADC read exceeds this number a fault will be created
#define MAX_HEATER_START_UP_ATTEMPTS          5        // If the heater ramp up process does not succeed in this many attempts, a fault will be generated that requires power cycle
#define MAX_DAC_TX_ATTEMPTS                   10       // The pic will attempt to write to the Converter Logic DAC this many times before giving up
#define HEATER_RAMP_UP_INCREMENT              50       // mV Units.  When ramping up the heater voltage it is increased by this amount each HEATER_RAMP_UP_TIME_PERIOD


#define HEATER_VOLTAGE_CURRENT_LIMITED_FAULT_TIME       500     // 5 Seconds
#define CURRENT_LIMITED_FAULT_HOLDOFF_TIME              10      // 10 seconds at the start of heater warmup before current limit fault is timed
#define FAULT_HOLDOFF_STATE                             22


#ifdef __CAN_CONTROLS
#define HEATER_WARM_UP_TIME 100       //18000     // In Can control mode the heater warm up time is enforced by the ECB
#else
#define HEATER_WARM_UP_TIME 12000     // 2 minutes
#endif


// ------------- Converter Logic Board ADC Input Settings ---------------------

// DPARKER figure out how to convert the temperature 
// It is in 2's compliment  This only works for positive temperatures
#define ADC_TEMPERATURE_SENSOR_FIXED_SCALE    1.25
#define ADC_TEMPERATURE_SENSOR_FIXED_OFFSET   0


#define ADC_HV_VMON_FIXED_SCALE               .34722
#define ADC_HV_VMON_FIXED_OFFSET              0
#define ADC_HV_VMON_RELATIVE_TRIP_SCALE       MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HV_VMON_RELATIVE_TRIP_FLOOR       1000                     
#define ADC_HV_VMON_RELATIVE_TRIP_COUNT       500              // 500mS


#define ADC_HV_IMON_FIXED_SCALE               .10419
#define ADC_HV_IMON_FIXED_OFFSET              0


#define ADC_GUN_I_PEAK_FIXED_SCALE            .17313
#define ADC_GUN_I_PEAK_FIXED_OFFSET           0


#define ADC_HTR_V_MON_FIXED_SCALE             .13875
#define ADC_HTR_V_MON_FIXED_OFFSET            0
#define ADC_HTR_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HTR_V_MON_RELATIVE_TRIP_FLOOR     200              // Minimum 200mV
#define ADC_HTR_V_MON_RELATIVE_TRIP_COUNT     500              // 500mS


#define ADC_HTR_I_MON_FIXED_SCALE             .10419
#define ADC_HTR_I_MON_FIXED_OFFSET            0
#define ADC_HTR_I_MON_OVER_LIMIT_ABSOLUTE     HTR_OC_ABS
#define ADC_HTR_I_MON_UNDER_LIMIT_ABSOLUTE    200              // 0.200 Amps
#define ADC_HTR_I_MON_ABSOLUTE_TRIP_TIME      500              // 500mS


#define ADC_TOP_V_MON_FIXED_SCALE             .69438
#define ADC_TOP_V_MON_FIXED_OFFSET            0
#define ADC_TOP_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_TOP_V_MON_RELATIVE_TRIP_FLOOR     1000             // 10 Volts
#define ADC_TOP_V_MON_RELATIVE_TRIP_TIME      500              // 500mS 


#define ADC_BIAS_V_MON_FIXED_SCALE            .34688
#define ADC_BIAS_V_MON_FIXED_OFFSET           0
#define ADC_BIAS_V_MON_OVER_LIMIT_ABSOLUTE    BIAS_OVER_VOLTAGE
#define ADC_BIAS_V_MON_UNDER_LIMIT_ABSOLUTE   BIAS_UNDER_VOLTAGE
#define ADC_BIAS_V_MON_ABSOLUTE_TRIP_TIME     500              // 500mS 


#define ADC_24_V_MON_FIXED_SCALE              .41688
#define ADC_24_V_MON_FIXED_OFFSET             0


#define ADC_TEMPERATURE_MON_FIXED_SCALE       .08331
#define ADC_TEMPERATURE_MON_FIXED_OFFSET      20400



// --------------- Converter Logic Board DAC output Settings -------------- //
#define DAC_HIGH_VOLTAGE_FIXED_SCALE          3.0000
#define DAC_HIGH_VOLTAGE_FIXED_OFFSET         0
#define HIGH_VOLTAGE_MAX_SET_POINT            HV_MAX_SET_BOARD_SPEC
#define HIGH_VOLTAGE_MIN_SET_POINT            HV_MIN_SET_BOARD_SPEC


#define DAC_TOP_VOLTAGE_FIXED_SCALE           1.5000
#define DAC_TOP_VOLTAGE_FIXED_OFFSET          0
#define TOP_VOLTAGE_MAX_SET_POINT             TOP_MAX_SET_BOARD_SPEC
#define TOP_VOLTAGE_MIN_SET_POINT             TOP_MIN_SET_BOARD_SPEC


#define DAC_HEATER_VOLTAGE_FIXED_SCALE        7.5188
#define DAC_HEATER_VOLTAGE_FIXED_OFFSET       0


// ------------- A36772 Internal PIC ADC Input Settings --------------------- //
#define POT_HTR_FIXED_SCALE                   .15625
#define POT_HTR_FIXED_OFFSET                  0

#define POT_VTOP_FIXED_SCALE                  .78125
#define POT_VTOP_FIXED_OFFSET                 0

#define POT_EK_FIXED_SCALE                    .42230
#define POT_EK_FIXED_OFFSET                   0

#define REF_HTR_FIXED_SCALE                   REF_HTR_SCALE_SELECTED
#define REF_HTR_FIXED_OFFSET                  0

#define REF_VTOP_FIXED_SCALE                  REF_VTOP_SCALE_SELECTED
#define REF_VTOP_FIXED_OFFSET                 REF_VTOP_OFFSET_SELECTED

#define REF_EK_FIXED_SCALE                    REF_EK_SCALE_SELECTED
#define REF_EK_FIXED_OFFSET                   0


// ------------- A36772 Onboard DAC Output Settings --------------------- //
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE    5.3333         // 1V = -1V Ef
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET   0

#define DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE    DAC_MON_HTR_CURRENT_SCALE_SELECTED
#define DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET   0

#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE   DAC_MON_EK_VOLTAGE_SCALE_SELECTED
#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET  0

#define DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE      DAC_MON_TOP_VOLTAGE_SCALE_SELECTED
#define DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET     DAC_MON_TOP_VOLTAGE_OFFSET_SELECTED


#endif
