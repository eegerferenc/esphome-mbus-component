#include "mbus_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mbus {

static const char *const TAG = "mbus.sensor";

void MbusSensor::setup() {
  this->telegram_seen = 0;
}

float MbusSensor::get_setup_priority() const {   return setup_priority::BUS - 1.0f; }
void MbusSensor::dump_config() {
  LOG_SENSOR("", "Mbus Sensor", this);
  ESP_LOGCONFIG(TAG, "  Secondary address: %llX" , this->parent_->secondary_address);
  ESP_LOGCONFIG(TAG, "  Storage number: %lld" , this->mbus_storage_requested_);
  ESP_LOGCONFIG(TAG, "  Function: %s" , MbusDIFFunctionToStr(this->mbus_function_requested_));
  ESP_LOGCONFIG(TAG, "  Tariff: %d" , this->mbus_tariff_requested_);
  ESP_LOGCONFIG(TAG, "  Subunit: %d" , this->mbus_subunit_requested_);
  ESP_LOGCONFIG(TAG, "  VIF/VIFE: 0x%llX" , this->mbus_vif_vife_requested_);
}

void MbusSensor::loop() {

  //check if new telegram is available
  if(this->telegram_seen == this->parent_->telegram_count)return;
  this->telegram_seen = this->parent_->telegram_count;
  
  const char* sensorname=this->get_name().c_str();
  
  //new telegram is available, parsing it
  uint8_t* tg = this->parent_->telegram;
  uint16_t len = tg[1] + 6; //payload + (start + length + length + start + checksum + stop)
  
  //tg[0] to tg[2] checked by statemachine
  //tg[3] start byte
  
  if(tg[4] != MBUS_CONTROL_RSP_UD){
	  ESP_LOGE(TAG, " %s: Unexpected control field %d", sensorname, tg[4]);
	  return;
  }
  
  //tg[5] primary address (usually 0 anyway)
  
  if(tg[6] != MBUS_CI_RESP_VARIABLE){
	  ESP_LOGE(TAG, " %s: Unexpected control information field %d", sensorname, tg[6]);
	  return;
  }
  
  //tg[7] to tg[14] secondary address
  
  std::string secondary_address_str;
  char buf[5];
  uint16_t pos;
  for(pos=10;pos>=7;pos--){
	sprintf(buf, "%02X", tg[pos]);
    secondary_address_str += buf;
  }
  for(pos=11;pos<=14;pos++){
	sprintf(buf, "%02X", tg[pos]);
    secondary_address_str += buf;
  }
  ESP_LOGD(TAG, " %s: Secondary address received: %s", sensorname, secondary_address_str.c_str());
  
  //tg[15] access number
  
  if(tg[16] & MBUS_STATUS_APP_ERROR){
	  ESP_LOGE(TAG, " %s: Application error", sensorname);
	  return;
  }
  if(tg[16] & MBUS_STATUS_TEMPORARY_ERROR){
	   ESP_LOGW(TAG, " %s: Temporary error status bit set", sensorname);
  }
  if(tg[16] & MBUS_STATUS_PERMANENT_ERROR){
	  ESP_LOGW(TAG, " %s: Permanent error status bit set. Consider replacing meter.", sensorname);
  }
  if(tg[16] & MBUS_STATUS_LOW_POWER){
	  ESP_LOGW(TAG, " %s: Low power status bit set. Consider replacing meter.", sensorname);
  }
  
  //tg[17] to tg[18] signature
  if(tg[17] || tg[18]){
	  ESP_LOGE(TAG, " %s: Nonzero signature field, possible encryption", sensorname);
	  return;
  }
  
  ESP_LOGD(TAG, " %s: Parsing fixed header done", sensorname);
  
  //variable payload fields follow
  pos = 19;
  this->mbus_data_record_parse_status_ = 0;
  while( ( pos <= (len-3) ) &&
	( tg[pos] != MBUS_DIF_MANUFACTURER_SPECIFIC ) &&
	( tg[pos] != MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME ) )
  { //each iteration of the while loop processes one Data Record,
	  //beginning with pos pointing at DIF. Iteration ends with
	  //pos pointing at DIF of next Data Record.
	  
	  if(tg[pos] == MBUS_DIF_FILLER) {
		  pos++;
		  continue;
	  }

	  uint8_t ret = MbusParseDataRecord(&tg[pos]);
	  if(!ret) { //error logging is done in MbusParseDataRecord
	    pos=0;
	    ESP_LOGW(TAG, " %s: Variable payload parsing aborted", sensorname);
	    break;
	  } 
	  pos+=ret;
  } //end while
  
  if(pos) { ESP_LOGD(TAG, " %s: Parsing variable payload done", sensorname); }

  if( pos > (len-3) ){
	  ESP_LOGW(TAG, " %s: Overrun while parsing telegram.", sensorname);
	  pos=0;
  }
    
  //manufacturer-specific data
  if(pos && (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME)){
	  ESP_LOGW(TAG, " %s: Multitelegram readout not supported, some data may be unavailable.", sensorname);
  }
  

  if(pos && ( (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC)
    || (tg[pos] == MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME) ) ) {
    pos++;
    uint16_t mfg_specific_begin = pos;
    std::string manufacturer_specific_data;
    for(pos=len-3;pos>=mfg_specific_begin;pos--){
	  sprintf(buf, "%02X ", tg[pos]);
      manufacturer_specific_data += buf;
    }
    ESP_LOGD(TAG, " %s: Manufacturer-specific data: %s", sensorname, manufacturer_specific_data.c_str());
  }
  
  //tg[len-2] checksum (verified by statemachine)
  //tg[len-1] stop byte
  
  if( !this->mbus_data_record_parse_status_ ){
	  ESP_LOGE(TAG, " %s: Specified data record not in telegram", sensorname);
	  return;
  }
  
  if( this->mbus_data_record_parse_status_ > 1 ){
	  ESP_LOGE(TAG, " %s: Multiple matching data records in telegram", sensorname);
	  return;
  }
  
  ESP_LOGI(TAG, "%s: New raw value: %.1f", sensorname, this->mbus_data_record_parse_result_);
  this->publish_state(this->mbus_data_record_parse_result_);
  
}

}  // namespace mbus
}  // namespace esphome
