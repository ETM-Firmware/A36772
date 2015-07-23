// This is firmware for the Gun Driver Board


// DPARKER - IF there is an error writing to the offboard DAC we probably need to re-write the entire DAC to ensure that data is correct

#include "A36772.h"

_FOSC(EC & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_64 & PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


void ETMDigitalInitializeInput(TYPE_DIGITAL_INPUT* input, unsigned int initial_value, unsigned int filter_time);
void ETMDigitalUpdateInput(TYPE_DIGITAL_INPUT* input, unsigned int current_value);

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




void EnableBeam(void);
void DisableBeam(void);

TYPE_GLOBAL_DATA_A36772 global_data_A36772;


unsigned char SPICharInvertered(unsigned char transmit_byte);


LTC265X U32_LTC2654;

void DoStartupLEDs(void);

unsigned int dac_test = 0;

void DoStateMachine(void);
void DoA36772(void);
void UpdateFaults(void);
//void UpdateFaultsHeaterRampUp(void);
//void UpdateFaultsHeaterOn(void);
//void UpdateFaultsPowerSupplyRampUp(void);
//void UpdateFaultsPowerSupplyOn(void);
void EnableHeater(void);
void DisableHeater(void);
void EnableHighVoltage(void);
void DisableHighVoltage(void);
void ResetFPGA(void);
void ADCConfigure(void);
void ADCStartAcquisition(void);
void UpdateADCResults(void);
void DACWriteChannel(unsigned int command_word, unsigned int data_word);
void FPGAReadData(void);
void DoHeaterRampUp(void);
void InitializeA36772(void);
unsigned int CheckFault(void);
unsigned int CheckHeaterFault(void);
int main(void) {
  global_data_A36772.control_state = STATE_START_UP;

  while (1) {
    DoStateMachine();
  }
}


#ifdef __CAN_CONTROLS
#define HEATER_WARM_UP_TIME 1800//18000     // In Can control mode the heater warm up time is enforced by the ECB
#else
#define HEATER_WARM_UP_TIME 1800//18000 // 3 minutes
#endif



void DoStateMachine(void) {
  switch (global_data_A36772.control_state) {

  case STATE_START_UP:
    InitializeA36772();
    DisableBeam();
    DisableHighVoltage();
    DisableHeater();
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    global_data_A36772.heater_start_up_attempts = 0;
    global_data_A36772.run_time_counter = 0;
#ifndef __CAN_REFERENCE
    _CONTROL_NOT_CONFIGURED = 0;
#endif
    global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
    break;

#define LED_STARTUP_FLASH_TIME   500 // 2 Seconds
#define MAX_HEATER_RAMP_UP_TIME  12000 // 2 minutes

  case STATE_WAIT_FOR_CONFIG:
    DisableBeam();
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A36772.control_state == STATE_WAIT_FOR_CONFIG) {
      DoA36772();
      DoStartupLEDs();
      if ((global_data_A36772.run_time_counter >= LED_STARTUP_FLASH_TIME) && (_CONTROL_NOT_CONFIGURED == 0)) {
	global_data_A36772.control_state = STATE_RESET_FPGA;
      }
    }
    break;

  case STATE_RESET_FPGA:
    ResetFPGA();
    global_data_A36772.control_state = STATE_HEATER_RAMP_UP;
    break;


  case STATE_HEATER_RAMP_UP:
    _CONTROL_NOT_READY = 1;

    global_data_A36772.analog_output_heater_voltage.set_point = 0;
    global_data_A36772.heater_start_up_attempts++;
    global_data_A36772.heater_warm_up_time_remaining = HEATER_WARM_UP_TIME;
    global_data_A36772.heater_ramp_up_time = MAX_HEATER_RAMP_UP_TIME;
    DisableBeam();
    DisableHighVoltage();
    EnableHeater();
    while (global_data_A36772.control_state == STATE_HEATER_RAMP_UP) {
      DoA36772();
      if (global_data_A36772.analog_output_heater_voltage.set_point >= global_data_A36772.heater_voltage_target) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP;
      }
      if (global_data_A36772.heater_ramp_up_time == 0) {
	_STATUS_HEATER_RAMP_TIMEOUT = 1;
	global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
      }
      if (CheckHeaterFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
      }
    }
    break;
    

  case STATE_HEATER_WARM_UP:
    DisableBeam();
    DisableHighVoltage();
    global_data_A36772.heater_ramp_up_time = 0;
    _CONTROL_NOT_READY = 1;
    while (global_data_A36772.control_state == STATE_HEATER_WARM_UP) {
      DoA36772();
      if (global_data_A36772.heater_warm_up_time_remaining == 0) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
      }
      if (CheckHeaterFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
      }
    }
    break;


  case STATE_HEATER_WARM_UP_DONE:
    DisableBeam();
    DisableHighVoltage();
    _CONTROL_NOT_READY = 1;
    global_data_A36772.heater_start_up_attempts = 0;
    while (global_data_A36772.control_state == STATE_HEATER_WARM_UP_DONE) {
      DoA36772();
      if (global_data_A36772.request_hv_enable) {
	global_data_A36772.control_state = STATE_POWER_SUPPLY_RAMP_UP;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

    
#define GUN_DRIVER_POWER_SUPPLY_STATUP_TIME  500 // 5 seconds

  case STATE_POWER_SUPPLY_RAMP_UP:
    DisableBeam();
    EnableHighVoltage();
    _CONTROL_NOT_READY = 1;
    global_data_A36772.power_supply_startup_remaining = GUN_DRIVER_POWER_SUPPLY_STATUP_TIME;
    while (global_data_A36772.control_state == STATE_POWER_SUPPLY_RAMP_UP) {
      DoA36772();
      if (global_data_A36772.power_supply_startup_remaining == 0) {
	global_data_A36772.control_state = STATE_HV_ON;
      }
      if (!global_data_A36772.request_hv_enable) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_HV_ON:
    DisableBeam();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36772.control_state == STATE_HV_ON) {
      DoA36772();
      if (global_data_A36772.request_beam_enable) {
	global_data_A36772.control_state = STATE_BEAM_ENABLE;
      }
      if (!global_data_A36772.request_hv_enable) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_BEAM_ENABLE:
    EnableBeam();
    _CONTROL_NOT_READY = 0;
    while (global_data_A36772.control_state == STATE_BEAM_ENABLE) {
      DoA36772();
      if (!global_data_A36772.request_beam_enable) {
	global_data_A36772.control_state = STATE_HV_ON;
      }
      if (!global_data_A36772.request_hv_enable) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;
    
  case STATE_FAULT_HEATER_ON:
    DisableHighVoltage();
    DisableBeam();
    _CONTROL_NOT_READY = 1;
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_ON) {
      DoA36772();
      if (global_data_A36772.reset_active) {
	global_data_A36772.control_state = STATE_HEATER_WARM_UP_DONE;
      }
      if (CheckFault()) { // DPARKER need better fault check here
	global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
      }
    }
    break;

#define MAX_HEATER_START_UP_ATTEMPTS   5
#define HEATER_AUTO_RESTART_TIME       500 // 5 seconds

  case STATE_FAULT_HEATER_OFF:
    _CONTROL_NOT_READY = 1;
    DisableHighVoltage();
    DisableHeater();
    DisableBeam();
    global_data_A36772.fault_restart_remaining = HEATER_AUTO_RESTART_TIME;
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_OFF) {
      DoA36772();
      if (global_data_A36772.fault_restart_remaining == 0) {
	global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
      }
      if (global_data_A36772.heater_start_up_attempts > MAX_HEATER_START_UP_ATTEMPTS) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_FAILURE;
      }
    }
    break;

  case STATE_FAULT_HEATER_FAILURE:
    _CONTROL_NOT_READY = 1;
    DisableHighVoltage();
    DisableHeater();
    DisableBeam();
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_FAILURE) {
      // Can't leave this state without power cycle
      DoA36772();
    }
    break;

  }
}

void DoStartupLEDs(void) {
  switch (((global_data_A36772.run_time_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_I2A = OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = OLL_LED_ON;
    PIN_LED_I2D = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_I2A = !OLL_LED_ON;
    PIN_LED_I2B = !OLL_LED_ON;
    PIN_LED_I2C = !OLL_LED_ON;
    PIN_LED_I2D = OLL_LED_ON;
    break;
  }
}


unsigned int CheckFault(void) {
  if (global_data_A36772.fault_reset_fpga || global_data_A36772.fault_reset_high_voltage) {
    return 1;
  } else {
    return 0;
  }
}

unsigned int CheckHeaterFault(void) {
  if (global_data_A36772.fault_reset_fpga) {
    return 1;
  } else {
    return 0;
  }
}

void DoA36772(void) {
  
#ifdef __CAN_ENABLED
  ETMCanSlaveDoCan();
#endif

#ifndef __CAN_REQUIRED
  ClrWdt();
#endif


#ifdef __DISCRETE_CONTROLS
  if (PIN_CUSTOMER_HV_ON == ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV) {
    global_data_A36772.request_hv_enable = 1;
    _STATUS_CUSTOMER_HV_ON = 1;
  } else {
    global_data_A36772.request_hv_enable = 0;
    _STATUS_CUSTOMER_HV_ON = 0;
  }

  if (PIN_CUSTOMER_BEAM_ENABLE == ILL_PIN_CUSTOMER_BEAM_ENABLE_BEAM_ENABLED) {
    global_data_A36772.request_beam_enable = 1;
    _STATUS_CUSTOMER_BEAM_ENABLE = 1;
  } else {
    global_data_A36772.request_beam_enable = 0;
    _STATUS_CUSTOMER_BEAM_ENABLE = 0;
  }
#endif
  
#ifdef __CAN_CONTROLS
  global_data_A36772.request_hv_enable   = !_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV;
  global_data_A36772.request_beam_enable = !_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY;
  
  _STATUS_CUSTOMER_HV_ON = !_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV;
  _STATUS_CUSTOMER_BEAM_ENABLE = !_SYNC_CONTROL_PULSE_SYNC_DISABLE_XRAY;
#endif


  if (_T2IF) {
    // Run once every 10ms
    _T2IF = 0;


#ifdef __CAN_CONTROLS
    if (_SYNC_CONTROL_RESET_ENABLE) {
      global_data_A36772.reset_active = 1;
    } else {
      global_data_A36772.reset_active = 0;
    }
#endif

#ifdef __DISCRETE_CONTROLS
    if ((PIN_CUSTOMER_BEAM_ENABLE == !ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV) && (global_data_A36772.previous_state_pin_customer_hv_on == ILL_PIN_CUSTOMER_HV_ON_ENABLE_HV)) {
      global_data_A36772.reset_active = 1;
    }
    global_data_A36772.previous_state_pin_customer_hv_on = PIN_CUSTOMER_HV_ON;
#endif


    if (global_data_A36772.reset_active) {
      ResetAllFaultInfo();
    }



    // Update to counter used to flash the LEDs at startup and time transmits to DACs
    if (global_data_A36772.power_supply_startup_remaining) {
      global_data_A36772.power_supply_startup_remaining--;
    }

    if (global_data_A36772.heater_warm_up_time_remaining) {
      global_data_A36772.heater_warm_up_time_remaining--;
    }

    if (global_data_A36772.heater_ramp_up_time) {
      global_data_A36772.heater_ramp_up_time--;
    }

    if (global_data_A36772.fault_restart_remaining) {
      global_data_A36772.fault_restart_remaining--;
    }



    global_data_A36772.watchdog_counter++;
    global_data_A36772.run_time_counter++;

    if (global_data_A36772.run_time_counter & 0x0010) {
      PIN_LED_OPERATIONAL = 1;
    } else {
      PIN_LED_OPERATIONAL = 0;
    }

    // Update Data from the FPGA
    FPGAReadData();

    // Read all the data from the external ADC
    UpdateADCResults();

    // Start the next acquisition from the external ADC
    ADCStartAcquisition();


#define WATCHDOG_HIGH     48000
#define WATCHDOG_LOW      16000
    
    if (global_data_A36772.watchdog_counter >= 3) {
      global_data_A36772.watchdog_counter = 0;
      if (global_data_A36772.dac_digital_watchdog_oscillator < ((WATCHDOG_HIGH >> 1) + (WATCHDOG_LOW >> 1))) {
	global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_HIGH;
      } else {
	global_data_A36772.dac_digital_watchdog_oscillator = WATCHDOG_LOW;
      }
    }
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, global_data_A36772.dac_digital_watchdog_oscillator);
    
    // Scale and Calibrate the internal ADC Readings
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_htr);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_vtop);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pot_ek);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_htr);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_vtop);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.ref_ek);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.pos_15v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.neg_15v_mon);

    //local_debug_data.debug_A = global_data_A36772.pot_htr.reading_scaled_and_calibrated;
    //local_debug_data.debug_B = global_data_A36772.pot_vtop.reading_scaled_and_calibrated;
    //local_debug_data.debug_C = global_data_A36772.pot_ek.reading_scaled_and_calibrated;
    //local_debug_data.debug_D = global_data_A36772.ref_htr.reading_scaled_and_calibrated;
    //local_debug_data.debug_E = global_data_A36772.ref_vtop.reading_scaled_and_calibrated;
    //local_debug_data.debug_F = global_data_A36772.ref_ek.reading_scaled_and_calibrated;
    local_debug_data.debug_A = global_data_A36772.run_time_counter;
    local_debug_data.debug_B = global_data_A36772.fault_restart_remaining;
    local_debug_data.debug_C = global_data_A36772.power_supply_startup_remaining;
    local_debug_data.debug_D = global_data_A36772.heater_warm_up_time_remaining;
    local_debug_data.debug_E = global_data_A36772.heater_ramp_up_time;
    local_debug_data.debug_F = global_data_A36772.control_state;
    
    
    

    
    // Scale and Calibrate the external ADC Readings
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_adc_temperature);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_hv_v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_hv_i_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_gun_i_peak);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_htr_v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_htr_i_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_top_v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_bias_v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_24_v_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_temperature_mon);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36772.input_dac_monitor);
    /*
    local_debug_data.debug_0 = global_data_A36772.input_adc_temperature.reading_scaled_and_calibrated;
    local_debug_data.debug_1 = global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_2 = global_data_A36772.input_hv_i_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_3 = global_data_A36772.input_gun_i_peak.reading_scaled_and_calibrated;
    local_debug_data.debug_4 = global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_5 = global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_6 = global_data_A36772.input_bias_v_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_7 = global_data_A36772.input_24_v_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_8 = global_data_A36772.input_temperature_mon.reading_scaled_and_calibrated;
    local_debug_data.debug_9 = global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated;
    */
    local_debug_data.debug_7 = global_data_A36772.dac_write_failure_count;

#ifdef __POT_REFERENCE
    // The set points should be based on the pots
    ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.pot_ek.reading_scaled_and_calibrated);
    ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.pot_vtop.reading_scaled_and_calibrated);
    global_data_A36772.heater_voltage_target                = global_data_A36772.pot_htr.reading_scaled_and_calibrated;
    //global_data_A36772.analog_output_high_voltage.set_point = global_data_A36772.pot_ek.reading_scaled_and_calibrated;
    //global_data_A36772.analog_output_top_voltage.set_point  = global_data_A36772.pot_vtop.reading_scaled_and_calibrated;
 #endif

#ifdef __DISCRETE_REFERENCE
    // The set points should be based on the analog references
    ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.ref_ek.reading_scaled_and_calibrated);
    ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.ref_vtop.reading_scaled_and_calibrated);
    global_data_A36772.heater_voltage_target                = global_data_A36772.ref_htr.reading_scaled_and_calibrated;    
    //global_data_A36772.analog_output_high_voltage.set_point = global_data_A36772.ref_ek.reading_scaled_and_calibrated;
    //global_data_A36772.analog_output_top_voltage.set_point  = global_data_A36772.ref_vtop.reading_scaled_and_calibrated;

#endif

#ifdef __CAN_REFERENCE
    ETMAnalogSetOutput(&global_data_A36772.analog_output_high_voltage, global_data_A36772.can_high_voltage_set_point);
    ETMAnalogSetOutput(&global_data_A36772.analog_output_top_voltage, global_data_A36772.can_pulse_top_set_point);
    global_data_A36772.heater_voltage_target                = global_data_A36772.can_heater_voltage_set_point;
    //global_data_A36772.analog_output_high_voltage.set_point = global_data_A36772.can_high_voltage_set_point;
    //global_data_A36772.analog_output_top_voltage.set_point  = global_data_A36772.can_pulse_top_set_point;
#endif


#define MAX_HEATER_CURRENT_DURING_RAMP_UP  1600   // 1.6 Amps
#define HEATER_RAMP_UP_INCREMENT           50     // Increase the heater voltage 10mV per step
#define HEATER_RAMP_UP_TIME_PERIOD         5      // Increase the heater voltage once every 50ms

    // Ramp the heater voltage
    global_data_A36772.heater_ramp_interval++;
    if (global_data_A36772.heater_ramp_interval >= HEATER_RAMP_UP_TIME_PERIOD) {
      global_data_A36772.heater_ramp_interval = 0;
      if (global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated < MAX_HEATER_CURRENT_DURING_RAMP_UP) {
	global_data_A36772.analog_output_heater_voltage.set_point += HEATER_RAMP_UP_INCREMENT;
      }
    }
    if (global_data_A36772.analog_output_heater_voltage.set_point > global_data_A36772.heater_voltage_target) {
      global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
    }

    // update the DAC programs based on the new set points.
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_high_voltage);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_top_voltage);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.analog_output_heater_voltage);
    
    
    
    ETMAnalogSetOutput(&global_data_A36772.monitor_heater_voltage, global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated);
    ETMAnalogSetOutput(&global_data_A36772.monitor_heater_current, global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated);
    ETMAnalogSetOutput(&global_data_A36772.monitor_cathode_voltage, global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated);
    ETMAnalogSetOutput(&global_data_A36772.monitor_grid_voltage, global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated);
 
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_heater_voltage);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_heater_current);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_cathode_voltage);
    ETMAnalogScaleCalibrateDACSetting(&global_data_A36772.monitor_grid_voltage);

    // Send out Data to local DAC and offboard.  Each channel will be updated once every 40mS
    // Do not send out while in state "STATE_WAIT_FOR_CONFIG" because the module is not ready to recieve data and
    // you will just get data transfer errors
    if (global_data_A36772.control_state != STATE_WAIT_FOR_CONFIG) {
      switch ((global_data_A36772.run_time_counter & 0b111)) {
	
      case 0:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.monitor_heater_voltage.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.analog_output_high_voltage.dac_setting_scaled_and_calibrated);
	local_debug_data.debug_0 = global_data_A36772.analog_output_high_voltage.dac_setting_scaled_and_calibrated;
	break;
	

      case 1:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.monitor_heater_current.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.analog_output_top_voltage.dac_setting_scaled_and_calibrated);
	local_debug_data.debug_1 = global_data_A36772.analog_output_top_voltage.dac_setting_scaled_and_calibrated;
	break;

    
      case 2:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.monitor_cathode_voltage.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.analog_output_heater_voltage.dac_setting_scaled_and_calibrated);
	local_debug_data.debug_2 = global_data_A36772.analog_output_heater_voltage.dac_setting_scaled_and_calibrated;
	break;

      
      case 3:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.monitor_grid_voltage.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.dac_digital_hv_enable);
	local_debug_data.debug_3 = global_data_A36772.dac_digital_hv_enable;
	break;


      case 4:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_A, global_data_A36772.monitor_heater_voltage.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable);
	local_debug_data.debug_4 = global_data_A36772.dac_digital_heater_enable;
	break;

      
      case 5:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_B, global_data_A36772.monitor_heater_current.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_F, global_data_A36772.dac_digital_top_enable);
	local_debug_data.debug_5 = global_data_A36772.dac_digital_top_enable;
	break;

    
      case 6:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_C, global_data_A36772.monitor_cathode_voltage.dac_setting_scaled_and_calibrated);
	DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);
	local_debug_data.debug_6 = global_data_A36772.dac_digital_trigger_enable;
	break;
    
  
      case 7:
	WriteLTC265X(&U32_LTC2654, LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.monitor_grid_voltage.dac_setting_scaled_and_calibrated);
	break;
      }
    }
  
    // Update Faults
    UpdateFaults();
  
    // Mange LED and Status Outputs
    

    // Warmup status
    if ((global_data_A36772.control_state >= STATE_START_UP) && (global_data_A36772.control_state <= STATE_HEATER_WARM_UP)) {
      PIN_LED_WARMUP = OLL_LED_ON;
      PIN_CPU_WARMUP_STATUS = OLL_STATUS_ACTIVE;
    } else {
      PIN_LED_WARMUP = !OLL_LED_ON;
      PIN_CPU_WARMUP_STATUS = !OLL_STATUS_ACTIVE;
    }

    // Standby Status
    if ((global_data_A36772.control_state >= STATE_HEATER_WARM_UP_DONE) && (global_data_A36772.control_state <= STATE_POWER_SUPPLY_RAMP_UP)) {
      PIN_LED_STANDBY = OLL_LED_ON;
      PIN_CPU_STANDBY_STATUS = OLL_STATUS_ACTIVE;
    } else {
      PIN_LED_STANDBY = !OLL_LED_ON;
      PIN_CPU_STANDBY_STATUS = !OLL_STATUS_ACTIVE;
    }
    
    // HV ON Status
    if (global_data_A36772.control_state >= STATE_HV_ON) {
      PIN_LED_HV_ON = OLL_LED_ON;
      PIN_CPU_HV_ON_STATUS = OLL_STATUS_ACTIVE;
    } else {
      PIN_LED_HV_ON = !OLL_LED_ON;
      PIN_CPU_HV_ON_STATUS = !OLL_STATUS_ACTIVE;
    }
    
    // Beam enabled Status
    if (global_data_A36772.control_state == STATE_BEAM_ENABLE) {
      PIN_LED_BEAM_ENABLE = OLL_LED_ON;
      PIN_CPU_BEAM_ENABLE_STATUS = OLL_STATUS_ACTIVE;
    } else {
      PIN_LED_BEAM_ENABLE = !OLL_LED_ON;
      PIN_CPU_BEAM_ENABLE_STATUS = !OLL_STATUS_ACTIVE;
    }
    
    // System OK Status
    if (global_data_A36772.control_state <= STATE_FAULT_HEATER_ON) {
      PIN_CPU_SYSTEM_OK_STATUS = !OLL_STATUS_ACTIVE;
      PIN_LED_SYSTEM_OK = !OLL_LED_ON;
    } else {
      PIN_CPU_SYSTEM_OK_STATUS = OLL_STATUS_ACTIVE;
      PIN_LED_SYSTEM_OK = OLL_LED_ON;
    }
  }
}


void UpdateFaults(void) {

#define TARGET_CONVERTER_LOGIC_PCB_REV   0b000000
#define TARGET_FPGA_FIRMWARE_MAJOR_REV   0b0001
#define TARGET_FPGA_FIRMWARE_MINOR_REV   0b000000

  // Evaluate the readings from the Coverter Logic Board ADC
  if (global_data_A36772.adc_read_ok) {
    // There was a valid read of the data from the converter logic board
    global_data_A36772.fault_reset_fpga = 0;
    global_data_A36772.fault_reset_high_voltage = 0;
    

    // ------------------- Evaluate the digital readings from the Coverter Logic Board ADC ---------------------//  
    
    if (global_data_A36772.adc_digital_warmup_flt.filtered_reading == 0) {
      _STATUS_ADC_DIGITAL_HEATER_NOT_READY = 1;
    } else {
      _STATUS_ADC_DIGITAL_HEATER_NOT_READY = 0;
    }
    
    if (global_data_A36772.adc_digital_watchdog_flt.filtered_reading == 0) {
      _FAULT_ADC_DIGITAL_WATCHDOG = 1;
      global_data_A36772.fault_reset_fpga = 1;
    }
    
    if (global_data_A36772.adc_digital_arc_flt.filtered_reading == 0) {
      _FAULT_ADC_DIGITAL_ARC = 1;
      global_data_A36772.fault_reset_high_voltage = 1;
    }
    
    if (global_data_A36772.adc_digital_over_temp_flt.filtered_reading == 0) {
      _FAULT_ADC_DIGITAL_OVER_TEMP = 1;
      global_data_A36772.fault_reset_fpga = 1;
    }

    if (global_data_A36772.adc_digital_pulse_width_duty_flt.filtered_reading == 0) {
      _FAULT_ADC_DIGITAL_PULSE_WIDTH_DUTY = 1;
      global_data_A36772.fault_reset_high_voltage = 1;
    }

    if (global_data_A36772.adc_digital_grid_flt.filtered_reading == 0) {
      _FAULT_ADC_DIGITAL_GRID = 1;
      global_data_A36772.fault_reset_fpga = 1;
    }
  
    // ------------------- Evaluate the analog readings from the Coverter Logic Board ADC ---------------------//


    global_data_A36772.input_htr_v_mon.target_value = global_data_A36772.analog_output_heater_voltage.set_point;
    global_data_A36772.input_hv_v_mon.target_value = global_data_A36772.analog_output_high_voltage.set_point;
    global_data_A36772.input_top_v_mon.target_value = global_data_A36772.analog_output_top_voltage.set_point;


    if (ETMAnalogCheckOverAbsolute(&global_data_A36772.input_htr_i_mon)) {
      _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE = 1;
      global_data_A36772.fault_reset_fpga = 1;
    }

    
    // Only check for these faults if the heater has completed it's ramp up
    if (global_data_A36772.control_state >= STATE_HEATER_WARM_UP) {
      if (ETMAnalogCheckOverRelative(&global_data_A36772.input_htr_v_mon)) {
	_FAULT_ADC_HTR_V_MON_OVER_RELATIVE = 1;
	global_data_A36772.fault_reset_fpga = 1;
      }
      
      if (ETMAnalogCheckUnderRelative(&global_data_A36772.input_htr_v_mon)) {
	_FAULT_ADC_HTR_V_MON_UNDER_RELATIVE = 1;
	global_data_A36772.fault_reset_fpga = 1;
      }
      
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36772.input_htr_i_mon)) {
	_FAULT_ADC_HTR_I_MON_UNDER_ABSOLUTE = 1;
	global_data_A36772.fault_reset_fpga = 1;
      }
    } else {
      global_data_A36772.input_htr_v_mon.over_trip_counter = 0;
      global_data_A36772.input_htr_v_mon.under_trip_counter = 0;
      global_data_A36772.input_htr_v_mon.absolute_under_counter = 0;
    }

    // Only check for these faults if the high voltage ramp time has completed
    if (global_data_A36772.control_state >= STATE_HV_ON) {
      if (ETMAnalogCheckOverRelative(&global_data_A36772.input_hv_v_mon)) {
	_FAULT_ADC_HV_V_MON_OVER_RELATIVE = 1;
	global_data_A36772.fault_reset_high_voltage = 1;
      }
      
      if (ETMAnalogCheckUnderRelative(&global_data_A36772.input_hv_v_mon)) {
	_FAULT_ADC_HV_V_MON_UNDER_RELATIVE = 1;
	global_data_A36772.fault_reset_high_voltage = 1;
      }
      
      if (ETMAnalogCheckOverRelative(&global_data_A36772.input_top_v_mon)) {
	_FAULT_ADC_TOP_V_MON_OVER_RELATIVE = 1;
	global_data_A36772.fault_reset_high_voltage = 1;
      }
      
      if (ETMAnalogCheckUnderRelative(&global_data_A36772.input_top_v_mon)) {
	_FAULT_ADC_TOP_V_MON_UNDER_RELATIVE = 1;
	global_data_A36772.fault_reset_high_voltage = 1;
      }
      
      if (ETMAnalogCheckOverAbsolute(&global_data_A36772.input_bias_v_mon)) {
	_FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE = 1;
      }
      
      if (ETMAnalogCheckUnderAbsolute(&global_data_A36772.input_bias_v_mon)) {
	_FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE = 1;
	global_data_A36772.fault_reset_high_voltage = 1;
      }
    } else {
      global_data_A36772.input_hv_v_mon.over_trip_counter = 0;
      global_data_A36772.input_hv_v_mon.under_trip_counter = 0;
      global_data_A36772.input_top_v_mon.over_trip_counter = 0;
      global_data_A36772.input_top_v_mon.under_trip_counter = 0;
      global_data_A36772.input_bias_v_mon.absolute_over_counter = 0;
      global_data_A36772.input_bias_v_mon.absolute_under_counter = 0;
   
    }
  } else if (global_data_A36772.adc_read_error_test > MAX_CONVERTER_LOGIC_ADC_READ_ERRORS) {
    global_data_A36772.adc_read_error_test = MAX_CONVERTER_LOGIC_ADC_READ_ERRORS;
    _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE = 1; 
    global_data_A36772.fault_reset_fpga = 1;
  }
  
}




#define DAC_DIGITAL_OFF   0x0000
#define DAC_DIGITAL_ON    0xFFFF


void EnableBeam(void) {
  global_data_A36772.dac_digital_trigger_enable = DAC_DIGITAL_ON;
  DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);
  PIN_CPU_BEAM_ENABLE = OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED;
}

void DisableBeam(void) {
  global_data_A36772.dac_digital_trigger_enable = DAC_DIGITAL_OFF;
  DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_G, global_data_A36772.dac_digital_trigger_enable);
  PIN_CPU_BEAM_ENABLE = !OLL_PIN_CPU_BEAM_ENABLE_BEAM_ENABLED;
}

void EnableHeater(void) {
  /* 
     Set the heater ref
     Set the heater enable control voltage
  */
  global_data_A36772.analog_output_heater_voltage.enabled = 1;
  global_data_A36772.dac_digital_heater_enable = DAC_DIGITAL_ON;
  //DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable);
}

void DisableHeater(void) {
  /* 
     Set the heater ref to zero
     Clear the heater enable control voltage
   */
  global_data_A36772.analog_output_heater_voltage.enabled = 0;
  global_data_A36772.dac_digital_heater_enable = DAC_DIGITAL_OFF;
  DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_E, global_data_A36772.dac_digital_heater_enable);
}

void EnableHighVoltage(void) {
  /*
    Set the HVPS reference
    Set the grid top reference 
    Set the HVPS enable control voltage
    Set the grid top enable control voltage
  */
  global_data_A36772.analog_output_high_voltage.enabled = 1;
  global_data_A36772.analog_output_top_voltage.enabled = 1;
  global_data_A36772.dac_digital_hv_enable = DAC_DIGITAL_ON;
  global_data_A36772.dac_digital_top_enable = DAC_DIGITAL_ON;
  //DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.dac_digital_hv_enable);
  //DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_F, global_data_A36772.dac_digital_top_enable);
  PIN_CPU_HV_ENABLE = OLL_PIN_CPU_HV_ENABLE_HV_ENABLED;
}

void DisableHighVoltage(void) {
  /*
    Set the HVPS reference to zero
    Set the grid top reference to zero 
    Clear the HVPS enable control voltage
    Clear the grid top enable control voltage
  */
  global_data_A36772.analog_output_high_voltage.enabled = 0;
  global_data_A36772.analog_output_top_voltage.enabled = 0;
  global_data_A36772.dac_digital_hv_enable = DAC_DIGITAL_OFF;
  global_data_A36772.dac_digital_top_enable = DAC_DIGITAL_OFF;
  DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_D, global_data_A36772.dac_digital_hv_enable);
  DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_F, global_data_A36772.dac_digital_top_enable);
  PIN_CPU_HV_ENABLE = !OLL_PIN_CPU_HV_ENABLE_HV_ENABLED;
}



void ResetFPGA(void) {
  PIN_CS_FPGA = OLL_PIN_CS_FPGA_SELECTED;
  PIN_CS_DAC  = OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_ADC  = OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  PIN_CS_DAC  = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);
}



// Define the input data byte
#define MAX1230_CONVERSION_BYTE    0b10000011
#define MAX1230_SETUP_BYTE         0b01101000
#define MAX1230_AVERAGE_BYTE       0b00111000
#define MAX1230_RESET_BYTE         0b00010000


void ADCConfigure(void) {
  /*
    Configure for read of all channels + temperature with 8x (or 16x) Averaging
  */
  unsigned char temp;

  PIN_CS_DAC  = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  PIN_CS_ADC  = OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  temp = SPICharInvertered(MAX1230_SETUP_BYTE);
  temp = SPICharInvertered(MAX1230_AVERAGE_BYTE);
  temp = SPICharInvertered(MAX1230_RESET_BYTE);


  PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);
}

void ADCStartAcquisition(void) {
  /* 
     Start the acquisition process
  */
  unsigned char temp;

  PIN_CS_DAC  = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  PIN_CS_ADC  = OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  temp = SPICharInvertered(MAX1230_CONVERSION_BYTE);

  PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

}

#define ADC_DATA_DIGITAL_HIGH   0x0800

void UpdateADCResults(void) {
  unsigned int n;
  unsigned int read_error;
  unsigned int read_data[17];
  
  /*
    Read all the results of the 16 Channels + temp sensor
    16 bits per channel
    17 channels
    272 bit message
    Approx 400us (counting processor overhead)
  */
  
  // Select the ADC
  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  PIN_CS_DAC  = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_ADC  = OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  for (n = 0; n < 17; n++) {
    read_data[n]   = SPICharInvertered(0);
    read_data[n] <<= 8;
    read_data[n]  += SPICharInvertered(0);
  }
  

  PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);
  


  // ERROR CHECKING ON RETURNED DATA.  IF THERE APPEARS TO BE A BIT ERROR, DO NOT LOAD THE DATA

  read_error = 0;
  read_error |= read_data[0];
  read_error |= read_data[1];
  read_error |= read_data[2];
  read_error |= read_data[3];
  read_error |= read_data[4];
  read_error |= read_data[5];
  read_error |= read_data[6];
  read_error |= read_data[7];
  read_error |= read_data[8];
  read_error |= read_data[9];
  read_error |= read_data[10];
  read_error |= read_data[11];
  read_error |= read_data[12];
  read_error |= read_data[13];
  read_error |= read_data[14];
  read_error |= read_data[15];
  read_error |= read_data[16];
  read_error  &= 0xF000;
  
  if (read_data[8] < 0x0200) {
    // The 24V supply is less than the minimum needed to operate
    read_error = 1;
  }
  
  if (read_error) {
    // There clearly is a data error
    global_data_A36772.adc_read_error_count++;
    global_data_A36772.adc_read_error_test++;
    global_data_A36772.adc_read_ok = 0;
  } else {
    // The data passed the most basic test.  Load the values into RAM
    global_data_A36772.adc_read_ok = 1;
    if (global_data_A36772.adc_read_error_test) {
      global_data_A36772.adc_read_error_test--;
    }
    
    global_data_A36772.input_adc_temperature.filtered_adc_reading = read_data[0];
    global_data_A36772.input_hv_v_mon.filtered_adc_reading = read_data[1] << 4;
    global_data_A36772.input_hv_i_mon.filtered_adc_reading = read_data[2] << 4;
    global_data_A36772.input_gun_i_peak.filtered_adc_reading = read_data[3] << 4;
    global_data_A36772.input_htr_v_mon.filtered_adc_reading = read_data[4] << 4;
    global_data_A36772.input_htr_i_mon.filtered_adc_reading = read_data[5] << 4;
    global_data_A36772.input_top_v_mon.filtered_adc_reading = read_data[6] << 4;
    global_data_A36772.input_bias_v_mon.filtered_adc_reading = read_data[7] << 4;
    global_data_A36772.input_24_v_mon.filtered_adc_reading = read_data[8] << 4;
    global_data_A36772.input_temperature_mon.filtered_adc_reading = read_data[9] << 4;
    global_data_A36772.input_dac_monitor.filtered_adc_reading = read_data[16] << 4;    
    
    if (read_data[10] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_warmup_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_warmup_flt, 0);
    }
    
    if (read_data[11] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_watchdog_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_watchdog_flt, 0);
    }
    
    if (read_data[12] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_arc_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_arc_flt, 0); 
    }
    
    if (read_data[13] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_over_temp_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_over_temp_flt, 0);
    }
    
    if (read_data[14] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_pulse_width_duty_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_pulse_width_duty_flt, 0);
    }
    
    if (read_data[15] > ADC_DATA_DIGITAL_HIGH) {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_grid_flt, 1);
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.adc_digital_grid_flt, 0);
    }
  }
}



#define MAX_DAC_TX_ATTEMPTS       10

void DACWriteChannel(unsigned int command_word, unsigned int data_word) {
  unsigned int command_word_check;
  unsigned int data_word_check;
  unsigned int transmission_complete;
  unsigned int loop_counter;
  unsigned int spi_char;

  transmission_complete = 0;
  loop_counter = 0;
  while (transmission_complete == 0) {
    loop_counter++;

    // -------------- Send Out the Data ---------------------//

    // Select the DAC
    PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
    PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
    PIN_CS_DAC  = OLL_PIN_CS_DAC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);
    
    spi_char = (command_word >> 8) & 0x00FF;
    command_word_check   = SPICharInvertered(spi_char);
    command_word_check <<= 8;
    spi_char = command_word & 0x00FF; 
    command_word_check  += SPICharInvertered(spi_char);
    

    spi_char = (data_word >> 8) & 0x00FF;
    data_word_check      = SPICharInvertered(spi_char);
    data_word_check    <<= 8;
    spi_char = data_word & 0x00FF; 
    data_word_check     += SPICharInvertered(spi_char);
    
    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);
    


    // ------------- Confirm the data was written correctly ------------------- //

    PIN_CS_DAC = OLL_PIN_CS_DAC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);

    spi_char = (LTC265X_CMD_NO_OPERATION >> 8) & 0x00FF;
    command_word_check   = SPICharInvertered(spi_char);
    command_word_check <<= 8;
    spi_char = LTC265X_CMD_NO_OPERATION & 0x00FF; 
    command_word_check  += SPICharInvertered(spi_char);
    
    spi_char = 0;
    data_word_check      = SPICharInvertered(spi_char);
    data_word_check    <<= 8;
    spi_char = 0;
    data_word_check     += SPICharInvertered(spi_char);


    PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
    __delay32(DELAY_FPGA_CABLE_DELAY);
    

    if ((command_word_check == command_word) && (data_word_check == data_word)) {
      transmission_complete = 1;
      global_data_A36772.dac_write_failure = 0;
    } else {
      global_data_A36772.dac_write_error_count++;
    }
    
    if ((transmission_complete == 0) & (loop_counter >= MAX_DAC_TX_ATTEMPTS)) {
      transmission_complete = 1;
      global_data_A36772.dac_write_failure_count++;
      global_data_A36772.dac_write_failure = 1;
      _STATUS_DAC_WRITE_FAILURE = 1;
    }
  }
}



typedef struct {
  unsigned converter_logic_pcb_rev:6;
  unsigned fpga_firmware_major_rev:4;
  unsigned fpga_firmware_minor_rev:6;
  unsigned arc:1;
  unsigned arc_high_voltage_inihibit_active:1;
  unsigned heater_voltage_less_than_4_5_volts:1;
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


void FPGAReadData(void) {
  unsigned long bits;
  TYPE_FPGA_DATA fpga_bits;
  /*
    Reads 32 bits from the FPGA
  */
  
  PIN_CS_ADC  = !OLL_PIN_CS_ADC_SELECTED;
  PIN_CS_DAC  = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_FPGA = OLL_PIN_CS_FPGA_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  bits   = SPICharInvertered(0xFF);
  bits <<= 8;
  bits  += SPICharInvertered(0xFF);
  bits <<= 8;
  bits  += SPICharInvertered(0xFF);
  bits <<= 8;
  bits  += SPICharInvertered(0xFF);
  

  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);

  // error check the data and update digital inputs  
  fpga_bits = *(TYPE_FPGA_DATA*)&bits;
  
  // Check the firmware major rev (LATCHED)    
  if (fpga_bits.fpga_firmware_major_rev != TARGET_FPGA_FIRMWARE_MAJOR_REV) {
    ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_major_rev_mismatch, 1);   
  } else { 
    ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_major_rev_mismatch, 0);
  }
  if (global_data_A36772.fpga_firmware_major_rev_mismatch.filtered_reading) {
    _FPGA_FIRMWARE_MAJOR_REV_MISMATCH = 1;
  }
  
  // Only check the rest of the data bits if the Major Rev Matches
  if (fpga_bits.fpga_firmware_major_rev == TARGET_FPGA_FIRMWARE_MAJOR_REV) {

    // Check the logic board pcb rev (NOT LATCHED)
    if (fpga_bits.converter_logic_pcb_rev != TARGET_CONVERTER_LOGIC_PCB_REV) {
      ETMDigitalUpdateInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch, 0);
    }
    if (global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch.filtered_reading) {
      _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH = 1;
    } else {
      _FPGA_CONVERTER_LOGIC_PCB_REV_MISMATCH = 0;
    }

    // Check the firmware minor rev (NOT LATCHED)
    if (fpga_bits.fpga_firmware_minor_rev != TARGET_FPGA_FIRMWARE_MINOR_REV) {
      ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch, 1);   
    } else {
      ETMDigitalUpdateInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch, 0);
    }
    if (global_data_A36772.fpga_firmware_minor_rev_mismatch.filtered_reading) {
      _FPGA_FIRMWARE_MINOR_REV_MISMATCH = 1;
    } else {
      _FPGA_FIRMWARE_MINOR_REV_MISMATCH = 0;
    }
    
    // Check the Arc Count (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_arc, fpga_bits.arc); 
    if (global_data_A36772.fpga_arc.filtered_reading) {
      _FPGA_ARC_COUNTER_GREATER_ZERO = 1;
    } else {
      _FPGA_ARC_COUNTER_GREATER_ZERO = 0;
    }
    
    // Check Arc High Voltage Inhibit Active (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_arc_high_voltage_inihibit_active, fpga_bits.arc_high_voltage_inihibit_active); 
    if (global_data_A36772.fpga_arc_high_voltage_inihibit_active.filtered_reading) {
      _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE = 1;
    } else {
      _FPGA_ARC_HIGH_VOLTAGE_INHIBIT_ACTIVE = 0;
    }

    // Check the heater voltage less than 4.5 Volts (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_heater_voltage_less_than_4_5_volts, fpga_bits.heater_voltage_less_than_4_5_volts);
    if (global_data_A36772.fpga_heater_voltage_less_than_4_5_volts.filtered_reading) {
      _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS = 1;
    } else {
      _FPGA_HEATER_VOLTAGE_LESS_THAN_4_5_VOLTS = 0;
    }

    // Check module temp greater than 65 C (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_module_temp_greater_than_65_C, fpga_bits.module_temp_greater_than_65_C);
    if (global_data_A36772.fpga_module_temp_greater_than_65_C.filtered_reading) {
      _FPGA_MODULE_TEMP_GREATER_THAN_65_C = 1;
    } else {
      _FPGA_MODULE_TEMP_GREATER_THAN_65_C = 0;
    }

    // Check module temp greater than 75 C (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_module_temp_greater_than_75_C, fpga_bits.module_temp_greater_than_75_C);
    if (global_data_A36772.fpga_module_temp_greater_than_75_C.filtered_reading) {
      _FPGA_MODULE_TEMP_GREATER_THAN_75_C = 1;
    } else {
      _FPGA_MODULE_TEMP_GREATER_THAN_75_C = 0;
    }
    
    // Check Pulse Width Limiting (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_pulse_width_limiting_active, fpga_bits.pulse_width_limiting_active);
    if (global_data_A36772.fpga_pulse_width_limiting_active.filtered_reading) {
      _FPGA_PULSE_WIDTH_LIMITING = 1;
    } else {
      _FPGA_PULSE_WIDTH_LIMITING = 0;
    }
    
    // Check prf fault (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_prf_fault, fpga_bits.prf_fault);
    if (global_data_A36772.fpga_prf_fault.filtered_reading) {
      _FPGA_PRF_FAULT = 1;
    } else {
      _FPGA_PRF_FAULT = 0;
    }

    // Check Current Monitor Pulse Width Fault (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_current_monitor_pulse_width_fault, fpga_bits.current_monitor_pulse_width_fault);
    if (global_data_A36772.fpga_current_monitor_pulse_width_fault.filtered_reading) {
      _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT = 1;
    } else {
      _FPGA_CURRENT_MONITOR_PULSE_WIDTH_FAULT = 0;
    }

    // Check grid module hardware fault (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_hardware_fault, fpga_bits.grid_module_hardware_fault);
    if (global_data_A36772.fpga_grid_module_hardware_fault.filtered_reading) {
      _FPGA_GRID_MODULE_HARDWARE_FAULT = 1;
    } else {
      _FPGA_GRID_MODULE_HARDWARE_FAULT = 0;
    }

    // Check grid module over voltage (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_over_voltage_fault, fpga_bits.grid_module_over_voltage_fault);
    if (global_data_A36772.fpga_grid_module_over_voltage_fault.filtered_reading) {
      _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT = 1;
    } else {
      _FPGA_GRID_MODULE_OVER_VOLTAGE_FAULT = 0;
    }

    // Check grid module under voltage (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_under_voltage_fault, fpga_bits.grid_module_under_voltage_fault);
    if (global_data_A36772.fpga_grid_module_under_voltage_fault.filtered_reading) {
      _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT = 1;
    } else {
      _FPGA_GRID_MODULE_UNDER_VOLTAGE_FAULT = 0;
    }

    // Check grid module bias voltage (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_grid_module_bias_voltage_fault, fpga_bits.grid_module_bias_voltage_fault);
    if (global_data_A36772.fpga_grid_module_bias_voltage_fault.filtered_reading) {
      _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT = 1;
    } else {
      _FPGA_GRID_MODULE_BIAS_VOLTAGE_FAULT = 0;
    }

    // High Voltage regulation Warning (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_hv_regulation_warning, fpga_bits.hv_regulation_warning);
    if (global_data_A36772.fpga_hv_regulation_warning.filtered_reading) {
      _FPGA_HV_REGULATION_WARNING = 1;
    } else {
      _FPGA_HV_REGULATION_WARNING = 0;
    }

    
    // FPGA DIPSWITCH 1 ON (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_dipswitch_1_on, fpga_bits.dipswitch_1_on);
    if (&global_data_A36772.fpga_dipswitch_1_on.filtered_reading) {
      _FPGA_DIPSWITCH_1_ON = 1;
    } else {
      _FPGA_DIPSWITCH_1_ON = 0;
    }

    // Check test mode toggle switch (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test, fpga_bits.test_mode_toggle_switch_set_to_test);
    if (&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test.filtered_reading) {
      _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE = 1;
    } else {
      _FPGA_TEST_MODE_TOGGLE_SWITCH_TEST_MODE = 0;
    }
    
    // Check local mode toggle switch (NOT LATCHED)
    ETMDigitalUpdateInput(&global_data_A36772.fpga_local_mode_toggle_switch_set_to_local, fpga_bits.local_mode_toggle_switch_set_to_local);
    if (global_data_A36772.fpga_local_mode_toggle_switch_set_to_local.filtered_reading) {
      _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE = 1;
    } else {
      _FPGA_LOCAL_MODE_TOGGLE_SWITCH_LOCAL_MODE = 0;
    }

  }
}



unsigned char SPICharInvertered(unsigned char transmit_byte) {
  unsigned int transmit_word;
  unsigned int receive_word;
  transmit_word = ((~transmit_byte) & 0x00FF);
  receive_word = SendAndReceiveSPI(transmit_word, ETM_SPI_PORT_1);
  receive_word = ((~receive_word) & 0x00FF);
  return (receive_word & 0x00FF);
}



void InitializeA36772(void) {
 
  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  etm_can_status_register.data_word_A = 0x0000;
  etm_can_status_register.data_word_B = 0x0000;

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;


  // --------- BEGIN IO PIN CONFIGURATION ------------------

  // Initialize Ouput Pin Latches BEFORE setting the pins to Output
  PIN_CS_DAC = !OLL_PIN_CS_DAC_SELECTED;
  PIN_CS_ADC = !OLL_PIN_CS_ADC_SELECTED;
  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
	  
  // ---- Configure the dsPIC ADC Module Analog Inputs------------ //
  ADPCFG = 0xFFFF;             // all are digital I/O
 
  // Initialize all I/O Registers
  TRISA = A36772_TRISA_VALUE;
  TRISB = A36772_TRISB_VALUE;
  TRISC = A36772_TRISC_VALUE;
  TRISD = A36772_TRISD_VALUE;
  TRISF = A36772_TRISF_VALUE;
  TRISG = A36772_TRISG_VALUE;

  // Config SPI1 for Gun Driver
  ConfigureSPI(ETM_SPI_PORT_1, A36772_SPI1CON_VALUE, 0, A36772_SPI1STAT_VALUE, SPI_CLK_1_MBIT, FCY_CLK);  
  

  // ---------- Configure Timers ----------------- //

  // Initialize TMR2
  PR2   = A36772_PR2_VALUE;
  TMR2  = 0;
  _T2IF = 0;
  _T2IP = 5;
  T2CON = A36772_T2CON_VALUE;

  // Configure on-board DAC
  SetupLTC265X(&U32_LTC2654, ETM_SPI_PORT_2, FCY_CLK, LTC265X_SPI_2_5_M_BIT, _PIN_RG15, _PIN_RC1);


  // ------------- Configure Internal ADC --------- //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters
  
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned

  _ADIF = 0;
  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)
  _ADIE = 1;
  _ADON = 1;

#ifdef __CAN_ENABLED
  // Initialize the Can module
  ETMCanSlaveInitialize(FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RC3, 4);
  ETMCanSlaveLoadConfiguration(36772, 250, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);
#endif

  ADCConfigure();
  
  if (!ETMAnalogCheckEEPromInitialized()) {
    ETMAnalogLoadDefaultCalibration();
  }

#define NO_RELATIVE_COUNTER  NO_COUNTER
#define NO_ABSOLUTE_COUNTER  NO_COUNTER

  

  // DPARKER figure out how to convert the temperature 
  // It is in 2's compliment  This only works for positive temperatures
#define ADC_TEMPERATURE_SENSOR_FIXED_SCALE   1.25
#define ADC_TEMPERATURE_SENSOR_FIXED_OFFSET  0


#define ADC_HV_VMON_FIXED_SCALE             .34722
#define ADC_HV_VMON_FIXED_OFFSET            0
#define ADC_HV_VMON_RELATIVE_TRIP_SCALE     MACRO_DEC_TO_CAL_FACTOR_2(.2)
#define ADC_HV_VMON_RELATIVE_TRIP_FLOOR     1000
#define ADC_HV_VMON_RELATIVE_TRIP_COUNT     50 // 500mS


#define ADC_HV_IMON_FIXED_SCALE             .10419
#define ADC_HV_IMON_FIXED_OFFSET            0
  //#define ADC_HV_IMON_OVER_LIMIT_ABSOLUTE     1000    // 1Amp
  //#define ADC_HV_IMON_OVER_LIMIT_TIME         10      // 100mS


#define ADC_GUN_I_PEAK_FIXED_SCALE          .17313
#define ADC_GUN_I_PEAK_FIXED_OFFSET         0
  //#define ADC_GUN_I_PEAK_OVER_LIMIT_ABSOLUTE  1000    // 1 Amp - DPARKER THIS NUMBER MAKES NO SENSE


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

#define ADC_24_V_MON_FIXED_SCALE               .41688
#define ADC_24_V_MON_FIXED_OFFSET              0

#define ADC_TEMPERATURE_MON_FIXED_SCALE             .08331
#define ADC_TEMPERATURE_MON_FIXED_OFFSET            20400



  // Initialize off board ADC Inputs
  ETMAnalogInitializeInput(&global_data_A36772.input_adc_temperature,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TEMPERATURE_SENSOR_FIXED_SCALE),
			   ADC_TEMPERATURE_SENSOR_FIXED_OFFSET,
			   ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);


  ETMAnalogInitializeInput(&global_data_A36772.input_hv_v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HV_VMON_FIXED_SCALE),
			   ADC_HV_VMON_FIXED_OFFSET,
			   ANALOG_INPUT_0,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   ADC_HV_VMON_RELATIVE_TRIP_SCALE,
			   ADC_HV_VMON_RELATIVE_TRIP_FLOOR,
			   ADC_HV_VMON_RELATIVE_TRIP_COUNT,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_hv_i_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HV_IMON_FIXED_SCALE),
			   ADC_HV_IMON_FIXED_OFFSET,
			   ANALOG_INPUT_1,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);
  

  ETMAnalogInitializeInput(&global_data_A36772.input_gun_i_peak,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_GUN_I_PEAK_FIXED_SCALE),
			   ADC_GUN_I_PEAK_FIXED_OFFSET,
			   ANALOG_INPUT_2,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_htr_v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HTR_V_MON_FIXED_SCALE),
			   ADC_HTR_V_MON_FIXED_OFFSET,
			   ANALOG_INPUT_3,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   ADC_HTR_V_MON_RELATIVE_TRIP_SCALE,
			   ADC_HTR_V_MON_RELATIVE_TRIP_FLOOR,
			   ADC_HTR_V_MON_RELATIVE_TRIP_COUNT,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_htr_i_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_HTR_I_MON_FIXED_SCALE),
			   ADC_HTR_I_MON_FIXED_OFFSET,
			   ANALOG_INPUT_4,
			   ADC_HTR_I_MON_OVER_LIMIT_ABSOLUTE,
			   ADC_HTR_I_MON_UNDER_LIMIT_ABSOLUTE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ADC_HTR_I_MON_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36772.input_top_v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TOP_V_MON_FIXED_SCALE),
			   ADC_TOP_V_MON_FIXED_OFFSET,
			   ANALOG_INPUT_5,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   ADC_TOP_V_MON_RELATIVE_TRIP_SCALE,
			   ADC_TOP_V_MON_RELATIVE_TRIP_FLOOR,
			   ADC_TOP_V_MON_RELATIVE_TRIP_TIME,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_bias_v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_BIAS_V_MON_FIXED_SCALE),
			   ADC_BIAS_V_MON_FIXED_OFFSET,
			   ANALOG_INPUT_6,
			   ADC_BIAS_V_MON_OVER_LIMIT_ABSOLUTE,
			   ADC_BIAS_V_MON_UNDER_LIMIT_ABSOLUTE,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ADC_BIAS_V_MON_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36772.input_24_v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_24_V_MON_FIXED_SCALE),
			   ADC_24_V_MON_FIXED_OFFSET,
			   ANALOG_INPUT_7,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_temperature_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(ADC_TEMPERATURE_MON_FIXED_SCALE),
			   ADC_TEMPERATURE_MON_FIXED_OFFSET,
			   ANALOG_INPUT_8,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.input_dac_monitor,
			   MACRO_DEC_TO_SCALE_FACTOR_16(1),
			   0,
			   ANALOG_INPUT_NO_CALIBRATION,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);





  // Initialize internal ADC Inputs

#define POT_HTR_FIXED_SCALE                   .15625
#define POT_HTR_FIXED_OFFSET                  0

#define POT_VTOP_FIXED_SCALE                   .78125
#define POT_VTOP_FIXED_OFFSET                  0

#define POT_EK_FIXED_SCALE                   .42230
#define POT_EK_FIXED_OFFSET                  0

#define REF_HTR_FIXED_SCALE                   .15625
#define REF_HTR_FIXED_OFFSET                  0

#define REF_VTOP_FIXED_SCALE                   .78125
#define REF_VTOP_FIXED_OFFSET                  0

#define REF_EK_FIXED_SCALE                   .42230
#define REF_EK_FIXED_OFFSET                  0



  ETMAnalogInitializeInput(&global_data_A36772.pot_htr,
			   MACRO_DEC_TO_SCALE_FACTOR_16(POT_HTR_FIXED_SCALE),
			   POT_HTR_FIXED_OFFSET,
			   ANALOG_INPUT_9,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);


  ETMAnalogInitializeInput(&global_data_A36772.pot_vtop,
			   MACRO_DEC_TO_SCALE_FACTOR_16(POT_VTOP_FIXED_SCALE),
			   POT_VTOP_FIXED_OFFSET,
			   ANALOG_INPUT_A,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.pot_ek,
			   MACRO_DEC_TO_SCALE_FACTOR_16(POT_EK_FIXED_SCALE),
			   POT_EK_FIXED_OFFSET,
			   ANALOG_INPUT_B,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);


  ETMAnalogInitializeInput(&global_data_A36772.ref_htr,
			   MACRO_DEC_TO_SCALE_FACTOR_16(REF_HTR_FIXED_SCALE),
			   REF_HTR_FIXED_OFFSET,
			   ANALOG_INPUT_9,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);


  ETMAnalogInitializeInput(&global_data_A36772.ref_vtop,
			   MACRO_DEC_TO_SCALE_FACTOR_16(REF_VTOP_FIXED_SCALE),
			   REF_VTOP_FIXED_OFFSET,
			   ANALOG_INPUT_A,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36772.ref_ek,
			   MACRO_DEC_TO_SCALE_FACTOR_16(REF_EK_FIXED_SCALE),
			   REF_EK_FIXED_OFFSET,
			   ANALOG_INPUT_B,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   NO_ABSOLUTE_COUNTER);


#define DAC_HIGH_VOLTAGE_FIXED_SCALE      3.0000
#define DAC_HIGH_VOLTAGE_FIXED_OFFSET     0
#define HIGH_VOLTAGE_MAX_SET_POINT        20000
#define HIGH_VOLTAGE_MIN_SET_POINT        5000

#define DAC_TOP_VOLTAGE_FIXED_SCALE       1.5000
#define DAC_TOP_VOLTAGE_FIXED_OFFSET      0
#define TOP_VOLTAGE_MAX_SET_POINT         26000
#define TOP_VOLTAGE_MIN_SET_POINT         0


#define DAC_HEATER_VOLTAGE_FIXED_SCALE       7.5188
#define DAC_HEATER_VOLTAGE_FIXED_OFFSET      0
#define HEATER_VOLTAGE_MAX_SET_POINT         8000
#define HEATER_VOLTAGE_MIN_SET_POINT         0

#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE    5.3333
#define DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET   0

#define DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE    10.6667
#define DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET   0

#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE    1.9733
#define DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET   0

#define DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE    1.0667
#define DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET   0



  // Initialize Converter Logic Board DAC Outputs
  ETMAnalogInitializeOutput(&global_data_A36772.analog_output_high_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_HIGH_VOLTAGE_FIXED_SCALE),
			    DAC_HIGH_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_0,
			    HIGH_VOLTAGE_MAX_SET_POINT,
			    HIGH_VOLTAGE_MIN_SET_POINT,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36772.analog_output_top_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_TOP_VOLTAGE_FIXED_SCALE),
			    DAC_TOP_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_1,
			    TOP_VOLTAGE_MAX_SET_POINT,
			    TOP_VOLTAGE_MIN_SET_POINT,
			    0);
  
  ETMAnalogInitializeOutput(&global_data_A36772.analog_output_heater_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_HEATER_VOLTAGE_FIXED_SCALE),
			    DAC_HEATER_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_2,
			    HEATER_VOLTAGE_MAX_SET_POINT,
			    HEATER_VOLTAGE_MIN_SET_POINT,
			    0);


  // Initialize on Board DAC Outputs  
  ETMAnalogInitializeOutput(&global_data_A36772.monitor_heater_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_HEATER_VOLTAGE_FIXED_SCALE),
			    DAC_MONITOR_HEATER_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_3,
			    0xFFFF,
			    0,
			    0);
  
  ETMAnalogInitializeOutput(&global_data_A36772.monitor_heater_current,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_HEATER_CURRENT_FIXED_SCALE),
			    DAC_MONITOR_HEATER_CURRENT_FIXED_OFFSET,
			    ANALOG_OUTPUT_4,
			    0xFFFF,
			    0,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36772.monitor_cathode_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_CATHODE_VOLTAGE_FIXED_SCALE),
			    DAC_MONITOR_CATHODE_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_5,
			    0xFFFF,
			    0,
			    0);

  ETMAnalogInitializeOutput(&global_data_A36772.monitor_grid_voltage,
			    MACRO_DEC_TO_SCALE_FACTOR_16(DAC_MONITOR_GRID_VOLTAGE_FIXED_SCALE),
			    DAC_MONITOR_GRID_VOLTAGE_FIXED_OFFSET,
			    ANALOG_OUTPUT_6,
			    0xFFFF,
			    0,
			    0);


  global_data_A36772.monitor_heater_voltage.enabled = 1;
  global_data_A36772.monitor_heater_current.enabled = 1;
  global_data_A36772.monitor_grid_voltage.enabled = 1;
  global_data_A36772.monitor_cathode_voltage.enabled = 1;



  ResetAllFaultInfo();
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;
  
  // Copy Data From Buffer to RAM
  if (_BUFS) {
    // read ADCBUF 0-7
    global_data_A36772.pot_ek.adc_accumulator       += ADCBUF0;
    global_data_A36772.pot_vtop.adc_accumulator     += ADCBUF1;
    global_data_A36772.pot_htr.adc_accumulator      += ADCBUF2;
    global_data_A36772.ref_htr.adc_accumulator      += ADCBUF3;
    global_data_A36772.ref_vtop.adc_accumulator     += ADCBUF4;
    global_data_A36772.ref_ek.adc_accumulator       += ADCBUF5;
    global_data_A36772.pos_15v_mon.adc_accumulator  += ADCBUF6;
    global_data_A36772.neg_15v_mon.adc_accumulator  += ADCBUF7;
  } else {
    // read ADCBUF 8-15
    global_data_A36772.pot_ek.adc_accumulator       += ADCBUF8;
    global_data_A36772.pot_vtop.adc_accumulator     += ADCBUF9;
    global_data_A36772.pot_htr.adc_accumulator      += ADCBUFA;
    global_data_A36772.ref_htr.adc_accumulator      += ADCBUFB;
    global_data_A36772.ref_vtop.adc_accumulator     += ADCBUFC;
    global_data_A36772.ref_ek.adc_accumulator       += ADCBUFD;
    global_data_A36772.pos_15v_mon.adc_accumulator  += ADCBUFE;
    global_data_A36772.neg_15v_mon.adc_accumulator  += ADCBUFF;
  }
  
  global_data_A36772.accumulator_counter += 1;
  
  if (global_data_A36772.accumulator_counter >= 128) {

    global_data_A36772.accumulator_counter = 0;    


    // average the 128 12 bit samples into a single 16 bit sample
    global_data_A36772.pot_htr.adc_accumulator      >>= 3;
    global_data_A36772.pot_vtop.adc_accumulator     >>= 3; 
    global_data_A36772.pot_ek.adc_accumulator       >>= 3; 
    global_data_A36772.ref_htr.adc_accumulator      >>= 3; 
    global_data_A36772.ref_vtop.adc_accumulator     >>= 3; 
    global_data_A36772.ref_ek.adc_accumulator       >>= 3; 
    global_data_A36772.pos_15v_mon.adc_accumulator  >>= 3; 
    global_data_A36772.neg_15v_mon.adc_accumulator  >>= 3; 

    // Store the filtred results
    global_data_A36772.pot_htr.filtered_adc_reading = global_data_A36772.pot_htr.adc_accumulator;
    global_data_A36772.pot_vtop.filtered_adc_reading = global_data_A36772.pot_vtop.adc_accumulator;
    global_data_A36772.pot_ek.filtered_adc_reading = global_data_A36772.pot_ek.adc_accumulator;
    global_data_A36772.ref_htr.filtered_adc_reading = global_data_A36772.ref_htr.adc_accumulator;
    global_data_A36772.ref_vtop.filtered_adc_reading = global_data_A36772.ref_vtop.adc_accumulator;
    global_data_A36772.ref_ek.filtered_adc_reading = global_data_A36772.ref_ek.adc_accumulator;
    global_data_A36772.pos_15v_mon.filtered_adc_reading = global_data_A36772.pos_15v_mon.adc_accumulator;
    global_data_A36772.neg_15v_mon.filtered_adc_reading = global_data_A36772.neg_15v_mon.adc_accumulator;
    
    // clear the accumulators
    global_data_A36772.pot_htr.adc_accumulator      = 0;
    global_data_A36772.pot_vtop.adc_accumulator     = 0; 
    global_data_A36772.pot_ek.adc_accumulator       = 0; 
    global_data_A36772.ref_htr.adc_accumulator      = 0; 
    global_data_A36772.ref_vtop.adc_accumulator     = 0; 
    global_data_A36772.ref_ek.adc_accumulator       = 0; 
    global_data_A36772.pos_15v_mon.adc_accumulator  = 0; 
    global_data_A36772.neg_15v_mon.adc_accumulator  = 0; 
  }
}




/*
  Needs to perform the following 
    
  1) Read all ADC channel from the Logic Board (there are 16 Channels + ADC Temperature Reading)
    AIN X - ADC Temperature
    AIN 0 - Vmon
    AIN 1 - Imon
    AIN 2 - Gun I Peak
    AIN 3 - Heater Voltage
    AIN 4 - Heater Current
    AIN 5 - Top Voltage
    AIN 6 - Bias Voltage
    AIN 7 - 24V divided by 6.67 = 3.6 Volts
    AIN 8 - Temperature
    AIN 9 - "DIGITAL" warmup / flt
    AIN 10 - "DIGITAL" watchdog fault
    AIN 11 - "DIGITAL" Arc fault
    AIN 12 - "DIGITAL" Temperature greater than 65 C
    AIN 13 - "DIGITAL" Pulse Width / Duty Fault
    AIN 14 - "DIGITAL" Grid Fault
    AIN 15 - Dac Feedback Voltage
    

  2) Write a DAC Channel to the Logic Board
    CHN A - High Voltage Adjust
    CHN B - Top Voltage
    CHN C - Heater Voltage
    CHN D - "Digital" High Voltage Enable
    CHN E - "Digital" Heater Enable
    CHN F - "Digital" Top Enable
    CHN G - "Digital" Trigger Enable
    CHN H - Watch Dog Analog Setting
    

  3) Read bits from the FPGA
    Read 32 bits from FPGA - See documentation for bit meaning

*/


/*
  Once every 10ms - SPI is 1 Mbit
  1) Read the entire 32 bits from FPGA - ~100uS
  2) Read all 17 Channels from the ADC (averaged) - ~300us
  3) Start Next ADC acquisition sequence (This acquisition will completed in less than 3ms)
  
  The DAC is not continuously updated
  ONLY the Watch Dog is Continuously Written (this is toggled once every 30ms)
*/

void ETMDigitalInitializeInput(TYPE_DIGITAL_INPUT* input, unsigned int initial_value, unsigned int filter_time) {
  if (filter_time > 0x7000) {
    filter_time = 0x7000;
  }
  input->filter_time = filter_time;
  if (initial_value == 0) {
    input->accumulator = 0;
    input->filtered_reading = 0;
  } else {
    input->accumulator = (filter_time << 1);
    input->filtered_reading = 1;
  }
}

void ETMDigitalUpdateInput(TYPE_DIGITAL_INPUT* input, unsigned int current_value) {
  if (input->filter_time < 2) {
    input->filtered_reading = current_value;
  } else {
    if (current_value) {
      if (++input->accumulator > (input->filter_time << 1)) {
	input->accumulator--;
      }
    } else {
      if (input->accumulator) {
	input->accumulator--;
      }
    }
    if (input->accumulator >= input->filter_time) {
      if (input->filtered_reading == 0) {
	// we are changing state from low to high
	input->accumulator = (input->filter_time << 1);
      }
      input->filtered_reading = 1;
    } else {
      if (input->filtered_reading == 1) {
	// we are changing state from high to low
	input->accumulator = 0;
      }
      input->filtered_reading = 0;
    }
  }
}

void ResetAllFaultInfo(void) {
  _FPGA_FIRMWARE_MAJOR_REV_MISMATCH = 0;
  _FAULT_ADC_HV_V_MON_OVER_RELATIVE = 0;
  _FAULT_ADC_HV_V_MON_UNDER_RELATIVE = 0;
  _FAULT_ADC_HTR_V_MON_OVER_RELATIVE = 0;
  _FAULT_ADC_HTR_V_MON_UNDER_RELATIVE = 0;
  _FAULT_ADC_HTR_I_MON_OVER_ABSOLUTE = 0;
  _FAULT_ADC_HTR_I_MON_UNDER_ABSOLUTE = 0;
  _FAULT_ADC_TOP_V_MON_OVER_RELATIVE = 0;
  _FAULT_ADC_TOP_V_MON_UNDER_RELATIVE = 0;
  _FAULT_ADC_BIAS_V_MON_OVER_ABSOLUTE = 0;
  _FAULT_ADC_BIAS_V_MON_UNDER_ABSOLUTE = 0;
  _FAULT_ADC_DIGITAL_WATCHDOG = 0;
  _FAULT_ADC_DIGITAL_ARC = 0;
  _FAULT_ADC_DIGITAL_OVER_TEMP = 0;
  _FAULT_ADC_DIGITAL_PULSE_WIDTH_DUTY = 0;
  _FAULT_ADC_DIGITAL_GRID = 0;
  _FAULT_CONVERTER_LOGIC_ADC_READ_FAILURE = 0;
  

  // Initialize Digital Input Filters for FPGA Status
  ETMDigitalInitializeInput(&global_data_A36772.fpga_coverter_logic_pcb_rev_mismatch       , 0, 10);   
  ETMDigitalInitializeInput(&global_data_A36772.fpga_firmware_major_rev_mismatch           , 0, 10);   
  ETMDigitalInitializeInput(&global_data_A36772.fpga_firmware_minor_rev_mismatch           , 0, 10);   
  ETMDigitalInitializeInput(&global_data_A36772.fpga_arc                                   , 0, 5);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_arc_high_voltage_inihibit_active      , 0, 0);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_heater_voltage_less_than_4_5_volts    , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_module_temp_greater_than_65_C         , 0, 10); 
  ETMDigitalInitializeInput(&global_data_A36772.fpga_module_temp_greater_than_75_C         , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_pulse_width_limiting_active           , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_prf_fault                             , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_current_monitor_pulse_width_fault     , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_hardware_fault            , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_over_voltage_fault        , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_under_voltage_fault       , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_grid_module_bias_voltage_fault        , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_hv_regulation_warning                 , 0, 50);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_dipswitch_1_on                        , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_test_mode_toggle_switch_set_to_test   , 0, 10);
  ETMDigitalInitializeInput(&global_data_A36772.fpga_local_mode_toggle_switch_set_to_local , 0, 10);

  // Initialize Digital Input Filters For ADC "Digital" Inputs
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_warmup_flt                     , 1, 20);
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_watchdog_flt                   , 1, 50);
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_arc_flt                        , 1, 10);
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_over_temp_flt                  , 1, 50);
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_pulse_width_duty_flt           , 1, 10);
  ETMDigitalInitializeInput(&global_data_A36772.adc_digital_grid_flt                       , 1, 10);

  // Reset all the Analog input counters
  


}
