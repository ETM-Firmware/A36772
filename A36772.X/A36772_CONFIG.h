#ifndef __A36772_CONFIG_H
#define __A36772_CONFIG_H



/*
  We have a couple of compile time options
  __MODE_CAN_INTERFACE        
    In this mode, the gun driver is controlled over the CAN interface.
    The descrete digital/analog is not used.
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

//#define __MODE_CAN_INTERFACE
#define __MODE_POT_INTERFACE
//#define __MODE_DISCRETE_INTERFACE
#define __OPTION_ENABLE_CAN


#ifndef __MODE_CAN_INTERFACE
#ifndef __MODE_POT_INTERFACE
#ifndef __MODE_DISCRETE_INTERFACE
#error "No reference Source Selected"
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
#endif

#ifdef __MODE_POT_INTERFACE
#define __POT_REFERENCE
#define __DISCRETE_CONTROLS
//#define __CAN_CONTROLS
#ifdef  __CAN_REFERENCE
#error "Multiple references selected"
#endif
#ifdef  __DISCRETE_REFERENCE
#error "Multiple references selected"
#endif
#endif

#ifdef __OPTION_ENABLE_CAN
#define __CAN_ENABLED
#endif






#define MAX_CONVERTER_LOGIC_ADC_READ_ERRORS 20




#define LED_STARTUP_FLASH_TIME   500 // 5 Seconds
#define MAX_HEATER_RAMP_UP_TIME  12000 // 2 minutes
#define MAX_HEATER_START_UP_ATTEMPTS   5
#define HEATER_AUTO_RESTART_TIME       500 // 5 seconds


#define MAX_DAC_TX_ATTEMPTS       10

#define MAX_HEATER_CURRENT_DURING_RAMP_UP  1600   // 1.6 Amps
#define HEATER_RAMP_UP_INCREMENT           50     // Increase the heater voltage 10mV per step
#define HEATER_RAMP_UP_TIME_PERIOD         5      // Increase the heater voltage once every 50ms
#define HEATER_VOLTAGE_CURRENT_LIMITED_FAULT_TIME (500 / HEATER_RAMP_UP_TIME_PERIOD)  // 5 Seconds



#define GUN_DRIVER_POWER_SUPPLY_STATUP_TIME  500 // 5 seconds



#ifdef __CAN_CONTROLS
#define HEATER_WARM_UP_TIME 1800//18000     // In Can control mode the heater warm up time is enforced by the ECB
#else
#define HEATER_WARM_UP_TIME 1800//18000 // 3 minutes
#endif





// ------------- Converter Logic Board ADC Input Settings ---------------------
#define NO_RELATIVE_COUNTER  NO_COUNTER // DPARKER MOVE to ETM_ANALOG.h
#define NO_ABSOLUTE_COUNTER  NO_COUNTER // DPARKER MOVE to ETM_ANALOG.h

  

// DPARKER figure out how to convert the temperature 
// It is in 2's compliment  This only works for positive temperatures
#define ADC_TEMPERATURE_SENSOR_FIXED_SCALE    1.25
#define ADC_TEMPERATURE_SENSOR_FIXED_OFFSET   0


#define ADC_HV_VMON_FIXED_SCALE               .34722
#define ADC_HV_VMON_FIXED_OFFSET              0
#define ADC_HV_VMON_RELATIVE_TRIP_SCALE       MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HV_VMON_RELATIVE_TRIP_FLOOR       1000
#define ADC_HV_VMON_RELATIVE_TRIP_COUNT       50 // 500mS


#define ADC_HV_IMON_FIXED_SCALE               .10419
#define ADC_HV_IMON_FIXED_OFFSET              0


#define ADC_GUN_I_PEAK_FIXED_SCALE            .17313
#define ADC_GUN_I_PEAK_FIXED_OFFSET           0


#define ADC_HTR_V_MON_FIXED_SCALE             .13875
#define ADC_HTR_V_MON_FIXED_OFFSET            0
#define ADC_HTR_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HTR_V_MON_RELATIVE_TRIP_FLOOR     200      // Minimum 200mV
#define ADC_HTR_V_MON_RELATIVE_TRIP_COUNT     50       // 500mS


#define ADC_HTR_I_MON_FIXED_SCALE             .10419
#define ADC_HTR_I_MON_FIXED_OFFSET            0
#define ADC_HTR_I_MON_OVER_LIMIT_ABSOLUTE     1750   // 1.750 Amps
#define ADC_HTR_I_MON_UNDER_LIMIT_ABSOLUTE    200    // 0.200 Amps
#define ADC_HTR_I_MON_ABSOLUTE_TRIP_TIME      50     // 500mS


#define ADC_TOP_V_MON_FIXED_SCALE             .69438
#define ADC_TOP_V_MON_FIXED_OFFSET            0
#define ADC_TOP_V_MON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_TOP_V_MON_RELATIVE_TRIP_FLOOR     1000  // 10 Volts
#define ADC_TOP_V_MON_RELATIVE_TRIP_TIME      50


#define ADC_BIAS_V_MON_FIXED_SCALE            .34688
#define ADC_BIAS_V_MON_FIXED_OFFSET           0
#define ADC_BIAS_V_MON_OVER_LIMIT_ABSOLUTE    18000
#define ADC_BIAS_V_MON_UNDER_LIMIT_ABSOLUTE   14000
#define ADC_BIAS_V_MON_ABSOLUTE_TRIP_TIME     50


#define ADC_24_V_MON_FIXED_SCALE              .41688
#define ADC_24_V_MON_FIXED_OFFSET             0


#define ADC_TEMPERATURE_MON_FIXED_SCALE       .08331
#define ADC_TEMPERATURE_MON_FIXED_OFFSET      20400



// --------------------- Converter Logic Board DAC output Settings -------------- //
#define DAC_HIGH_VOLTAGE_FIXED_SCALE          3.0000
#define DAC_HIGH_VOLTAGE_FIXED_OFFSET         0
#define HIGH_VOLTAGE_MAX_SET_POINT            20000
#define HIGH_VOLTAGE_MIN_SET_POINT            5000


#define DAC_TOP_VOLTAGE_FIXED_SCALE           1.5000
#define DAC_TOP_VOLTAGE_FIXED_OFFSET          0
#define TOP_VOLTAGE_MAX_SET_POINT             26000
#define TOP_VOLTAGE_MIN_SET_POINT             0


#define DAC_HEATER_VOLTAGE_FIXED_SCALE        7.5188
#define DAC_HEATER_VOLTAGE_FIXED_OFFSET       0
#define HEATER_VOLTAGE_MAX_SET_POINT          8000
#define HEATER_VOLTAGE_MIN_SET_POINT          0






// ------------- A36772 Internal PIC ADC Input Settings --------------------- //
#define POT_HTR_FIXED_SCALE                   .15625
#define POT_HTR_FIXED_OFFSET                  0

#define POT_VTOP_FIXED_SCALE                  .78125
#define POT_VTOP_FIXED_OFFSET                 0

#define POT_EK_FIXED_SCALE                    .42230
#define POT_EK_FIXED_OFFSET                   0

#define REF_HTR_FIXED_SCALE                   .15625
#define REF_HTR_FIXED_OFFSET                  0

#define REF_VTOP_FIXED_SCALE                  .78125
#define REF_VTOP_FIXED_OFFSET                 0

#define REF_EK_FIXED_SCALE                    .42230
#define REF_EK_FIXED_OFFSET                   0


// ------------- A36772 Onboard DAC Output Settings --------------------- //
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE    5.3333
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET   0

#define DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE    10.6667
#define DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET   0

#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE   1.9733
#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET  0

#define DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE      1.0667
#define DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET     0
































// MOVE THESE to A36772.h I think

#define WATCHDOG_HIGH     48000
#define WATCHDOG_LOW      16000


#define DAC_DIGITAL_OFF   0x0000
#define DAC_DIGITAL_ON    0xFFFF




#define ADC_DATA_DIGITAL_HIGH   0x0800

    
#define TARGET_CONVERTER_LOGIC_PCB_REV   0b000000
#define TARGET_FPGA_FIRMWARE_MAJOR_REV   0b0001
#define TARGET_FPGA_FIRMWARE_MINOR_REV   0b000000
  
// Define the input data byte
#define MAX1230_CONVERSION_BYTE    0b10000011
#define MAX1230_SETUP_BYTE         0b01101000
#define MAX1230_AVERAGE_BYTE       0b00111000
#define MAX1230_RESET_BYTE         0b00010000


  

    





#endif
