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
  /*
  unsigned int word3 = ETMScaleFactor2(analog_reads[ANA_RD_EG].read_cur, CAN_scale_table[CAN_RD_EG].fixed_scale, CAN_scale_table[CAN_RD_EG].fixed_offset);
  unsigned int word2 = 0; // low energy Eg((faults_from_ADC << 8) | control_state);        
  unsigned int word1 = ETMScaleFactor16(analog_reads[ANA_RD_EK].read_cur, CAN_scale_table[CAN_RD_EK].fixed_scale, CAN_scale_table[CAN_RD_EK].fixed_offset);
  unsigned int word0 = ETMScaleFactor16(analog_reads[ANA_RD_IKP].read_cur, CAN_scale_table[CAN_RD_IKP].fixed_scale, CAN_scale_table[CAN_RD_IKP].fixed_offset);

  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_PULSE_TOP_MON, word3, word2, word1, word0);
  */
}


void ETMCanSlaveLogCustomPacketD(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  
  /*
  unsigned int word3 = ETMScaleFactor16(analog_reads[ANA_RD_EF].read_cur, CAN_scale_table[CAN_RD_EF].fixed_scale, CAN_scale_table[CAN_RD_EF].fixed_offset);
  unsigned int word2 = ETMScaleFactor2(analog_reads[ANA_RD_IF].read_cur, CAN_scale_table[CAN_RD_IF].fixed_scale, CAN_scale_table[CAN_RD_IF].fixed_offset);
  unsigned int word1 = 0; // htd remaining 
  unsigned int word0 = ETMScaleFactor2(analog_reads[ANA_RD_TEMP].read_cur, CAN_scale_table[CAN_RD_TEMP].fixed_scale, CAN_scale_table[CAN_RD_TEMP].fixed_offset);

  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_HEATER_MON, word3, word2, word1, word0);

  */
}


void ETMCanSlaveLogCustomPacketE(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */

  /*

  unsigned int word3 = ETMScaleFactor2(analog_sets[ANA_SET_EG].ip_set, CAN_scale_table[CAN_RD_EGSET].fixed_scale, CAN_scale_table[CAN_RD_EGSET].fixed_offset);
  unsigned int word2 = 0; // low energy Eg set
  unsigned int word1 = ETMScaleFactor2(analog_sets[ANA_SET_EF].ip_set, CAN_scale_table[CAN_RD_EFSET].fixed_scale, CAN_scale_table[CAN_RD_EFSET].fixed_offset);
  unsigned int word0 = ETMScaleFactor2(analog_sets[ANA_SET_EK].ip_set, CAN_scale_table[CAN_RD_EKSET].fixed_scale, CAN_scale_table[CAN_RD_EKSET].fixed_offset);

  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_SLOW_SET_POINTS, word3, word2, word1, word0);

  */
}

void ETMCanSlaveLogCustomPacketF(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  /*
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_GUN_DRIVER_FPGA_DATA,
		     fpga_ASDR,
		     faults_from_ADC,
		     control_state,
		     ETMScaleFactor2(analog_reads[ANA_RD_EC].read_cur, CAN_scale_table[CAN_RD_EC].fixed_scale, 0));

  */
}


  



