// This is firmware for the Gun Driver Board

#include "A36772.h"

_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_64 & PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);


TYPE_GLOBAL_DATA_A36772 global_data_A36772;


unsigned char SPICharInvertered(unsigned char transmit_byte);


void DoStartupLEDs(void);

unsigned int dac_test = 0;

void DoStateMachine(void);
void DoA36772(void);
void UpdateFaultsHeaterRampUp(void);
void UpdateFaultsHeaterOn(void);
void UpdateFaultsPowerSupplyRampUp(void);
void UpdateFaultsPowerSupplyOn(void);
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

int main(void) {
  global_data_A36772.control_state = STATE_START_UP;

  while (1) {
    DoStateMachine();
  }
}

void DoStateMachine(void) {
  switch (global_data_A36772.control_state) {

  case STATE_START_UP:
    InitializeA36772();
    DisableHighVoltage();
    DisableHeater();
    _CONTROL_NOT_CONFIGURED = 1;
    _CONTROL_NOT_READY = 1;
    global_data_A36772.start_up_counter = 0;
    global_data_A36772.led_flash_counter = 0;
    global_data_A36772.control_state = STATE_WAIT_FOR_CONFIG;
    break;

#define LED_STARTUP_FLASH_TIME   500 // 5 Seconds

  case STATE_WAIT_FOR_CONFIG:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A36772.control_state == STATE_WAIT_FOR_CONFIG) {
      DoA36772();
      DoStartupLEDs();
      /*
      if ((global_data_A36772.led_flash_counter >= LED_STARTUP_FLASH_TIME) && (_CONTROL_NOT_CONFIGURED == 0)) {
	global_data_A36772.control_state = STATE_HEATER_RAMP_UP;
      }
      DPARKER HANG OUT HERE FOR TESTING PHASE 1
      */
    }
    break;


    // DPARKER the function that sets the heater voltage must set the "heater_voltage_target", not the "set_point"
  case STATE_HEATER_RAMP_UP:
    global_data_A36772.analog_output_heater_voltage.set_point = 0;
    global_data_A36772.start_up_counter++;
    DisableHighVoltage();
    EnableHeater();
    while (global_data_A36772.control_state == STATE_HEATER_RAMP_UP) {
      DoA36772();
      DoHeaterRampUp();
      if (global_data_A36772.analog_output_heater_voltage.set_point >= global_data_A36772.heater_voltage_target) {
	global_data_A36772.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (_FAULT_PIC_HEATER_TURN_OFF) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_OFF;
      }
    }
    break;
    

  case STATE_HEATER_RAMP_UP_DONE:
    DisableHighVoltage();
    while (global_data_A36772.control_state == STATE_HEATER_RAMP_UP_DONE) {
      global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
      if (!_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A36772.control_state = STATE_POWER_SUPPLY_RAMP_UP;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

    
#define GUN_DRIVER_POWER_SUPPLY_STATUP_TIME  500 // 5 seconds

  case STATE_POWER_SUPPLY_RAMP_UP:
    EnableHighVoltage();
    global_data_A36772.power_supply_startup_up_counter = 0;
    while (global_data_A36772.control_state == STATE_POWER_SUPPLY_RAMP_UP) {
      global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
      if (global_data_A36772.power_supply_startup_up_counter >= GUN_DRIVER_POWER_SUPPLY_STATUP_TIME) {
	global_data_A36772.control_state = STATE_HV_ON;
      }
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A36772.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_HV_ON:
    while (global_data_A36772.control_state == STATE_HV_ON) {
      global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
      if (_SYNC_CONTROL_PULSE_SYNC_DISABLE_HV) {
	global_data_A36772.control_state = STATE_HEATER_RAMP_UP_DONE;
      }
      if (CheckFault()) {
	global_data_A36772.control_state = STATE_FAULT_HEATER_ON;
      }
    }
    break;

  case STATE_FAULT_HEATER_ON:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_ON) {

    }
    break;


  case STATE_FAULT_HEATER_OFF:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_OFF) {
    }
    break;

  case STATE_FAULT_HEATER_FAILURE:
    DisableHighVoltage();
    DisableHeater();
    while (global_data_A36772.control_state == STATE_FAULT_HEATER_FAILURE) {
    }
    break;

  }
}

void DoStartupLEDs(void) {
  switch (((global_data_A36772.led_flash_counter >> 4) & 0b11)) {
    
  case 0:
    PIN_LED_AC_ON = OLL_LED_ON;
    PIN_LED_LAST_PULSE_FAIL = !OLL_LED_ON;
    PIN_LED_WARMUP = !OLL_LED_ON;
    PIN_LED_SUM_FAULT = !OLL_LED_ON;
    break;
    
  case 1:
    PIN_LED_AC_ON = !OLL_LED_ON;
    PIN_LED_LAST_PULSE_FAIL = OLL_LED_ON;
    PIN_LED_WARMUP = !OLL_LED_ON;
    PIN_LED_SUM_FAULT = !OLL_LED_ON;
    break;
    
  case 2:
    PIN_LED_AC_ON = !OLL_LED_ON;
    PIN_LED_LAST_PULSE_FAIL = !OLL_LED_ON;
    PIN_LED_WARMUP = OLL_LED_ON;
    PIN_LED_SUM_FAULT = !OLL_LED_ON;
    break;
    
  case 3:
    PIN_LED_AC_ON = !OLL_LED_ON;
    PIN_LED_LAST_PULSE_FAIL = !OLL_LED_ON;
    PIN_LED_WARMUP = !OLL_LED_ON;
    PIN_LED_SUM_FAULT = OLL_LED_ON;
    break;
  }
}


unsigned int CheckFault(void) {
  return 0;
}


void DoA36772(void) {
  
  ETMCanSlaveDoCan();

  if (_T2IF) {
    _T2IF = 0;

    // Run once every 10ms
    // Update to counter used to flash the LEDs at startup
    global_data_A36772.led_flash_counter++;
    if (global_data_A36772.led_flash_counter >= LED_STARTUP_FLASH_TIME) {
      global_data_A36772.led_flash_counter = LED_STARTUP_FLASH_TIME;
    }

    global_data_A36772.power_supply_startup_up_counter++;
    if (global_data_A36772.power_supply_startup_up_counter >= GUN_DRIVER_POWER_SUPPLY_STATUP_TIME) {
      global_data_A36772.power_supply_startup_up_counter = GUN_DRIVER_POWER_SUPPLY_STATUP_TIME;
    }

    // Update Data from the FPGA
    FPGAReadData();
    Nop();
    Nop();
    Nop();
    Nop();
    // Read all the data from the ADC
    UpdateADCResults();
    Nop();
    Nop();
    Nop();
    Nop();
    dac_test += 0x7FF;
    DACWriteChannel(LTC265X_WRITE_AND_UPDATE_DAC_H, dac_test);
    
    Nop();
    Nop();
    Nop();
    Nop();

    ADCStartAcquisition();
        
    Nop();
    Nop();
    Nop();
    Nop();
    

    Nop();
    Nop();
    Nop();
    Nop();

    __delay32(90000);


    local_debug_data.debug_0++; 
    local_debug_data.debug_1 = global_data_A36772.adc_read_error_count;
    local_debug_data.debug_2 = global_data_A36772.adc_read_error_test;
    local_debug_data.debug_3 = global_data_A36772.dac_write_error_count;
    local_debug_data.debug_4 = global_data_A36772.dac_write_failure;
    local_debug_data.debug_5 = global_data_A36772.dac_write_failure_count;
  }
}




/*
  Based on the control state only a subset of faults are checked
  There are four fault checking functions that are called based upon the operation state
*/


void UpdateFaultsHeaterRampUp(void) {
  /*
    This updates all faults related to the heater ramping up
  */
}

void UpdateFaultsHeaterOn(void) {
  /*
    This includes all faults in UpdateFaultsHeaterRampUp as well as 
    faults that are checked once the heater has finished ramping up
  */
}


void UpdateFaultsPowerSupplyRampUp(void) {
  /*
    All faults in UpdateFaultsHeaterOn as well as
    faults that are checked when the power supplies are ramping up
  */
}


void UpdateFaultsPowerSupplyOn(void) {
  /*
    All faults in UpdateFaultsPowerSupplyRampUp as well as
    all other remaining faults
  */

}


void EnableHeater(void) {

}

void DisableHeater(void) {

}

void EnableHighVoltage(void) {
  /*
    Set the HVPS reference
    Set the grid top reference // DPARKER is this voltage always fixed?
    Set the HVPS enable control voltage
    Set the grid top enable control voltage
    Set the trigger enable control voltage
  */
}

void DisableHighVoltage(void) {
/*
    Set the HVPS reference to zero
    Set the grid top reference to zero // DPARKER is this voltage always fixed?
    Clear the HVPS enable control voltage
    Clear the grid top enable control voltage
    Clear the trigger enable control voltage
*/
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
// DPARKER DO we need to configure the unipolar mode register?????
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

  /*
  temp = SendAndReceiveSPI(MAX1230_SETUP_BYTE, ETM_SPI_PORT_1);
  temp = SendAndReceiveSPI(MAX1230_AVERAGE_BYTE, ETM_SPI_PORT_1);
  temp = SendAndReceiveSPI(MAX1230_RESET_BYTE, ETM_SPI_PORT_1);
  */
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
  } else {
    // The data passed the most basic test.  Load the values into RAM
    if (global_data_A36772.adc_read_error_test) {
      global_data_A36772.adc_read_error_test--;
    }

    global_data_A36772.input_adc_temperature.filtered_adc_reading = read_data[0];
    global_data_A36772.input_hv_v_mon.filtered_adc_reading = read_data[1];
    global_data_A36772.input_hv_i_mon.filtered_adc_reading = read_data[2];
    global_data_A36772.input_gun_i_peak.filtered_adc_reading = read_data[3];
    global_data_A36772.input_htr_v_mon.filtered_adc_reading = read_data[4];
    global_data_A36772.input_htr_i_mon.filtered_adc_reading = read_data[5];
    global_data_A36772.input_top_v_mon.filtered_adc_reading = read_data[6];
    global_data_A36772.input_bias_v_mon.filtered_adc_reading = read_data[7];
    global_data_A36772.input_24_v_mon.filtered_adc_reading = read_data[8];
    global_data_A36772.input_temperature_mon.filtered_adc_reading = read_data[9];
    global_data_A36772.input_dac_monitor.filtered_adc_reading = read_data[16];    
    
    if (read_data[10] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_warmup_flt = 1;
    } else {
      global_data_A36772.adc_digital_warmup_flt = 0;
    }
    
    if (read_data[11] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_watchdog_flt = 1;
    } else {
      global_data_A36772.adc_digital_watchdog_flt = 0;
    }
    
    if (read_data[12] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_arc_flt = 1;
    } else {
      global_data_A36772.adc_digital_arc_flt = 0;
    }
    
    if (read_data[13] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_over_temp_flt = 1;
    } else {
      global_data_A36772.adc_digital_over_temp_flt = 0;
    }
    
    if (read_data[14] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_pulse_width_duty_flt = 1;
    } else {
      global_data_A36772.adc_digital_pulse_width_duty_flt = 0;
    }
    
    if (read_data[15] > ADC_DATA_DIGITAL_HIGH) {
      global_data_A36772.adc_digital_grid_flt = 1;
    } else {
      global_data_A36772.adc_digital_grid_flt = 0;
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
      // DPARKER INCREMENT SOME FAULT COUNTER AND INDICATE STATUS (NOT FAULT)
    }
  }
}



void FPGAReadData(void) {
  unsigned long bits;
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
  

  global_data_A36772.fpga_data = *(TYPE_FPGA_DATA*)&bits;

  PIN_CS_FPGA = !OLL_PIN_CS_FPGA_SELECTED;
  __delay32(DELAY_FPGA_CABLE_DELAY);
  
}



unsigned char SPICharInvertered(unsigned char transmit_byte) {
  unsigned int transmit_word;
  unsigned int receive_word;
  transmit_word = ((~transmit_byte) & 0x00FF);
  receive_word = SendAndReceiveSPI(transmit_word, ETM_SPI_PORT_1);
  receive_word = ((~receive_word) & 0x00FF);
  return (receive_word & 0x00FF);
}


#define MAX_HEATER_CURRENT_DURING_RAMP_UP  1000   // DPARKER Figure out Correct Value
#define HEATER_RAMP_UP_INCREMENT           100    // DPARKER Figure out Correct Value


void DoHeaterRampUp(void) {
  if (global_data_A36772.heater_ramp_counter >= 100) {
    // We only update the ramp up once per second durring the ramp
    global_data_A36772.heater_ramp_counter = 0;

    // If the current is less than the max ramp up current, then increase the heater program voltage
    if (global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated < MAX_HEATER_CURRENT_DURING_RAMP_UP) {
      global_data_A36772.analog_output_heater_voltage.set_point += HEATER_RAMP_UP_INCREMENT;
      if (global_data_A36772.analog_output_heater_voltage.set_point > global_data_A36772.heater_voltage_target) {
	global_data_A36772.analog_output_heater_voltage.set_point = global_data_A36772.heater_voltage_target;
      }
    }
  }  
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

  ResetFPGA();
  

  // ------------- Configure Internal ADC --------- //



  // Initialize the Can module
  ETMCanSlaveInitialize(FCY_CLK, ETM_CAN_ADDR_GUN_DRIVER_BOARD, _PIN_RG12, 4);
  ETMCanSlaveLoadConfiguration(36772, 250, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);

  ADCConfigure();
}







/*
  Needs to perform the following 
    
  1) Read an ADC channel from the Logic Board (there are 16 Channels)
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
  3) Start Next ADC acquisition sequence (This will completed in less than 3ms)
  
  The DAC is not continuously updated
  ONLY the Watch Dog is Continuously Written (this is toggled once every 30ms)
*/

