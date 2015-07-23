#include "A36772.h"

void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
  //unsigned int value;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      /*
    case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_GRID_TOP_SET_POINT:
      value = ETMScaleFactor16(message_ptr->word1, CAN_scale_table[CAN_SET_EGSET].fixed_scale, 0);            
      SetEg(value);    
      // word0 for low Eg, not used 
      _CONTROL_NOT_CONFIGURED = AreAnyReferenceNotConfigured();
      break;
      
    case ETM_CAN_REGISTER_GUN_DRIVER_SET_1_HEATER_CATHODE_SET_POINT:
      value = ETMScaleFactor16(message_ptr->word1, CAN_scale_table[CAN_SET_EKSET].fixed_scale, 0);
      SetEk(value);    
      value = ETMScaleFactor16(message_ptr->word0, CAN_scale_table[CAN_SET_EFSET].fixed_scale, 0);            
      SetEf(value);    
      _CONTROL_NOT_CONFIGURED = AreAnyReferenceNotConfigured();    
      break;

    default:
      local_can_errors.invalid_index++;
      break;
      */
    }
  
}


void ETMCanSlaveLogCustomPacketC(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_PULSE_TOP_MON, 
		     global_data_A36772.input_top_v_mon.reading_scaled_and_calibrated, 
		     0,
		     global_data_A36772.input_hv_v_mon.reading_scaled_and_calibrated, 
		     global_data_A36772.input_hv_i_mon.reading_scaled_and_calibrated
		     );
}


void ETMCanSlaveLogCustomPacketD(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_HEATER_MON, 
		     global_data_A36772.input_htr_v_mon.reading_scaled_and_calibrated, 
		     global_data_A36772.input_htr_i_mon.reading_scaled_and_calibrated, 
		     0, // DPARKER figure this out - Heater time remaining 
		     global_data_A36772.input_temperature_mon.reading_scaled_and_calibrated
		     );
}


void ETMCanSlaveLogCustomPacketE(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_SET_POINTS, 
		     global_data_A36772.analog_output_top_voltage.set_point,
		     global_data_A36772.analog_output_top_voltage.set_point,
		     global_data_A36772.heater_voltage_target,
		     global_data_A36772.analog_output_high_voltage.set_point
		     );
}

void ETMCanSlaveLogCustomPacketF(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_FPGA_DATA,
		     0,
		     0,
		     global_data_A36772.control_state,
		     global_data_A36772.input_bias_v_mon.reading_scaled_and_calibrated
		     );
}


  



