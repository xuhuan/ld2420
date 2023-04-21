#include "ld2420.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) &0xff)

#define byte0(val) (uint8_t)((val) &0xFF)
#define byte1(val) (uint8_t)((val) >>  8)
#define byte2(val) (uint8_t)((val) >> 16)
#define byte3(val) (uint8_t)((val) >> 24)

namespace esphome {
namespace ld2420 {

static const char *const TAG = "ld2420";

float LD2420Component::get_setup_priority() const { return setup_priority::BUS; }

void LD2420Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2420:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "HasPresenceSensor", this->presence_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "MovingSensor", this->moving_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "StillSensor", this->still_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "Moving Distance", this->moving_target_distance_sensor_);
  LOG_SENSOR("  ", "Still Distance", this->still_target_distance_sensor_);
  LOG_SENSOR("  ", "Moving Energy", this->moving_target_energy_sensor_);
  LOG_SENSOR("  ", "Still Energy", this->still_target_energy_sensor_);
  LOG_SENSOR("  ", "Detection Distance", this->detection_distance_sensor_);
#endif
  this->set_config_mode_(true);
  this->get_firmware_version_();
  this->set_config_mode_(false);
  ESP_LOGCONFIG(TAG, "  Firmware Version : %7s",this->ld2420_firmware_ver_);
}

void LD2420Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LD2420...");
  this->set_config_mode_(true);
  this->get_firmware_version_();
  this->set_min_max_distances_timeout_(this->max_gate_distance_, this->min_gate_distance_, this->timeout_);
  // Configure Gates sensitivity
  this->set_gate_thresholds_(0, this->rg0_move_threshold_, this->rg0_still_threshold_);
  this->set_gate_thresholds_(1, this->rg1_move_threshold_, this->rg1_still_threshold_);
  this->set_gate_thresholds_(2, this->rg2_move_threshold_, this->rg2_still_threshold_);
  this->set_gate_thresholds_(3, this->rg3_move_threshold_, this->rg3_still_threshold_);
  this->set_gate_thresholds_(4, this->rg4_move_threshold_, this->rg4_still_threshold_);
  this->set_gate_thresholds_(5, this->rg5_move_threshold_, this->rg5_still_threshold_);
  this->set_gate_thresholds_(6, this->rg6_move_threshold_, this->rg6_still_threshold_);
  this->set_gate_thresholds_(7, this->rg7_move_threshold_, this->rg7_still_threshold_);
  this->set_gate_thresholds_(8, this->rg8_move_threshold_, this->rg8_still_threshold_);
  this->set_gate_thresholds_(9, this->rg9_move_threshold_, this->rg9_still_threshold_);
  this->set_gate_thresholds_(10, this->rg10_move_threshold_, this->rg10_still_threshold_);
  this->set_gate_thresholds_(11, this->rg11_move_threshold_, this->rg11_still_threshold_);
  this->set_gate_thresholds_(12, this->rg12_move_threshold_, this->rg12_still_threshold_);
  this->set_gate_thresholds_(13, this->rg13_move_threshold_, this->rg13_still_threshold_);
  this->set_gate_thresholds_(14, this->rg14_move_threshold_, this->rg14_still_threshold_);
  this->set_gate_thresholds_(15, this->rg15_move_threshold_, this->rg15_still_threshold_);
  this->set_config_mode_(false);
  ESP_LOGCONFIG(TAG, "Firmware Version : %7s",this->ld2420_firmware_ver_);
  ESP_LOGCONFIG(TAG, "LD2420 setup complete.");
}

void LD2420Component::loop() {
  const int max_line_length = 80;
  static uint8_t buffer[max_line_length];

  while (available()) {
    this->readline_(read(), buffer, max_line_length);
  }
}


void LD2420Component::handle_periodic_data_(uint8_t *buffer, int len) {
  if (len < 12)
    return;  // 4 cmd_frame start bytes + 2 length bytes + 1 data end byte + 1 crc byte + 4 cmd_frame end bytes
  if (buffer[0] != 0xF4 || buffer[1] != 0xF3 || buffer[2] != 0xF2 || buffer[3] != 0xF1)  // check 4 cmd_frame start bytes
    return;
  if (buffer[7] != HEAD || buffer[len - 6] != END || buffer[len - 5] != CHECK)  // Check constant values
    return;  // data head=0xAA, data end=0x55, crc=0x00

    /*
      Data Type: 6th
      0x01: Engineering mode
      0x02: Normal mode
    */
    // char data_type = buffer[DATA_TYPES];
    /*
      Target states: 9th
      0x00 = No target
      0x01 = Moving targets
      0x02 = Still targets
      0x03 = Moving+Still targets
    */
// #ifdef USE_BINARY_SENSOR
//   char target_state = buffer[TARGET_STATES];
//   if (this->target_binary_sensor_ != nullptr) {
//     this->target_binary_sensor_->publish_state(target_state != 0x00);
//   }
// #endif

  /*
    Reduce data update rate to prevent home assistant database size grow fast
  */
  int32_t current_millis = millis();
  if (current_millis - last_periodic_millis < 1000)
    return;
  last_periodic_millis = current_millis;

// #ifdef USE_BINARY_SENSOR
//   if (this->moving_binary_sensor_ != nullptr) {
//     this->moving_binary_sensor_->publish_state(CHECK_BIT(target_state, 0));
//   }
//   if (this->still_binary_sensor_ != nullptr) {
//     this->still_binary_sensor_->publish_state(CHECK_BIT(target_state, 1));
//   }
// #endif
  /*
    Moving target distance: 10~11th bytes
    Moving target energy: 12th byte
    Still target distance: 13~14th bytes
    Still target energy: 15th byte
    Detect distance: 16~17th bytes
  */
#ifdef USE_SENSOR
  // if (this->moving_target_distance_sensor_ != nullptr) {
  //   int new_moving_target_distance = this->two_byte_to_int_(buffer[MOVING_TARGET_LOW], buffer[MOVING_TARGET_HIGH]);
  //   if (this->moving_target_distance_sensor_->get_state() != new_moving_target_distance)
  //     this->moving_target_distance_sensor_->publish_state(new_moving_target_distance);
  // }
  // if (this->moving_target_energy_sensor_ != nullptr) {
  //   int new_moving_target_energy = buffer[MOVING_ENERGY];
  //   if (this->moving_target_energy_sensor_->get_state() != new_moving_target_energy)
  //     this->moving_target_energy_sensor_->publish_state(new_moving_target_energy);
  // }
  // if (this->still_target_distance_sensor_ != nullptr) {
  //   int new_still_target_distance = this->two_byte_to_int_(buffer[STILL_TARGET_LOW], buffer[STILL_TARGET_HIGH]);
  //   if (this->still_target_distance_sensor_->get_state() != new_still_target_distance)
  //     this->still_target_distance_sensor_->publish_state(new_still_target_distance);
  // }
  // if (this->still_target_energy_sensor_ != nullptr) {
  //   int new_still_target_energy = buffer[STILL_ENERGY];
  //   if (this->still_target_energy_sensor_->get_state() != new_still_target_energy)
  //     this->still_target_energy_sensor_->publish_state(new_still_target_energy);
  // }
  // if (this->detection_distance_sensor_ != nullptr) {
  //   int new_detect_distance = this->two_byte_to_int_(buffer[DETECT_DISTANCE_LOW], buffer[DETECT_DISTANCE_HIGH]);
  //   if (this->detection_distance_sensor_->get_state() != new_detect_distance)
  //     this->detection_distance_sensor_->publish_state(new_detect_distance);
  // }
#endif
}


void LD2420Component::handle_normal_mode_(uint8_t *inbuf, int len) {
  const uint8_t bufsize = 16;
  uint8_t index = 0;
  uint8_t pos = 0;
  uint8_t range = 0;
  bool present;
  char* endptr;
  char outbuf[bufsize];
  while (true) {
    if (inbuf[pos] >= '0' && inbuf[pos] <='9') {
      if (index < bufsize - 1) {
        outbuf[index++] = inbuf[pos];
        pos++;
      }
    } else {
      if (pos < len - 1) {
        pos++;
      } else {
        break;
      }
    }
    if (inbuf[pos - 2] == 'O' && inbuf[pos - 1] == 'F' && inbuf[pos] == 'F') {
      set_object_presence_(false);
      break;
    }
    if (inbuf[pos - 1] == 'O' && inbuf[pos] == 'N' ) {
      set_object_presence_(true);
      break;
    }
  }
  outbuf[index] = '\0';
  if (index > 0) this->set_object_range_(strtol(outbuf,&endptr,10));

  /*
    Reduce data update rate to prevent home assistant database size grow fast
  */
  int32_t current_millis = millis();
  if (current_millis - last_normal_periodic_millis < 1000)
    return;
  last_normal_periodic_millis = current_millis;


#ifdef USE_BINARY_SENSOR
  if (this->presence_binary_sensor_ != nullptr) {
      if (this->presence_binary_sensor_->state != this->is_present_()) {
      this->presence_binary_sensor_->publish_state(this->is_present_());
    }
  }
#endif

#ifdef USE_SENSOR
  if (this->moving_target_distance_sensor_ != nullptr) {
    int new_moving_target_distance = this->object_range_;
    if (this->moving_target_distance_sensor_->get_state() != new_moving_target_distance)
      this->moving_target_distance_sensor_->publish_state(new_moving_target_distance);
  }
#endif
}


void LD2420Component::handle_ack_data_(uint8_t *buffer, int len) {
  if (len < 12) {
    ESP_LOGE(TAG, "LD2420 acknowledge response frame too short with %u bytes.", len);
    return;
  }
  if (buffer[COMMAND_STATUS] != 0x01) {
    ESP_LOGE(TAG, "LD2420 acknowledge status byte was %u, command failed.", buffer[COMMAND_STATUS]);
    return;
  }

#ifdef LD2420_LOG_VERY_VERBOSE
  ESP_LOGVV(TAG,"");
  printf("UART Tx: ");
  for (int i = 0; i < len; i++) {
    printf("%02X ", buffer[i]);
  }
  printf("\n");
#endif

  switch (buffer[COMMAND]) {
    case lowbyte(CMD_ENABLE_CONF):
      ESP_LOGV(TAG, "LD2420 acknowledged config enable: %2X", CMD_ENABLE_CONF);
      break;
    case lowbyte(CMD_DISABLE_CONF):
      ESP_LOGV(TAG, "LD2420 acknowledged config disable: %2X", CMD_DISABLE_CONF);
      break;
    case lowbyte(CMD_WRITE_ABD_PARAM):
      ESP_LOGV(TAG, "LD2420 acknowledged write gate parameters: %2X", CMD_WRITE_ABD_PARAM);
      break;
    case lowbyte(CMD_READ_VERSION):
      memcpy(this->ld2420_firmware_ver_, &buffer[12], 6);
      ESP_LOGV(TAG, "Read LD2420 module firmware ver: %7s",this->ld2420_firmware_ver_);
    break;
      default:
      break;
  }
}

void LD2420Component::readline_(int readch, uint8_t *buffer, int len) {
  static int pos = 0;

  if (readch >= 0) {
    if (pos < len - 1) {
      buffer[pos++] = readch;
      buffer[pos] = 0;
    } else {
      pos = 0;
    }
    if (pos >= 4) {
      if (buffer[pos - 4] == 0xF8 && buffer[pos - 3] == 0xF7 && buffer[pos - 2] == 0xF6 && buffer[pos - 1] == 0xF5) {
        ESP_LOGV(TAG, "Will handle Periodic Data");
        this->handle_periodic_data_(buffer, pos);
        // this->received_frame_handler_(buffer,pos);
        pos = 0;  // Reset position index ready for next time
      } else if (buffer[pos - 4] == 0x04 && buffer[pos - 3] == 0x03 && buffer[pos - 2] == 0x02 && buffer[pos - 1] == 0x01) {
        ESP_LOGV(TAG, "Will handle ACK Data");
        this->handle_ack_data_(buffer, pos);
        pos = 0;  // Reset position index ready for next time
      } else if (buffer[pos - 2] == 0x0D && buffer[pos - 1] == 0x0A) {
        this->handle_normal_mode_(buffer, pos);
        pos = 0;  // Reset position index ready for next time
      }
    }
  }
}

void LD2420Component::send_cmd_from_array_(uint8_t* cmdArray, cmd_frame_t frame) {
  frame.length = 0;
  frame.data_length += 2; // Add on the command byte
  memcpy(&cmdArray[frame.length], &frame.header, sizeof(frame.header));
  frame.length += sizeof(frame.header);
  memcpy(&cmdArray[frame.length], &frame.data_length, sizeof(frame.length));
  frame.length += sizeof(frame.data_length);
  memcpy(cmdArray + frame.length, &frame.command, sizeof(frame.command));
  frame.length += sizeof(frame.command);
  if (frame.data_length > 2) { // Minimun 2 for the command byte, otherwise there is additional data
    for (uint8_t index; index < frame.data_length - 2 ; index++) {
      memcpy(cmdArray + frame.length, &frame.data[index], sizeof(frame.data[index]));
      frame.length += sizeof(frame.data[index]);
    }
  }
  memcpy(cmdArray + frame.length, &frame.footer, sizeof(frame.footer));
  frame.length += sizeof(frame.footer);
  for (uint8_t index; index < frame.length; index++) {
    this->write_byte(cmdArray[index]);
  }
#ifdef LD2420_LOG_VERY_VERBOSE
  ESP_LOGVV(TAG,"");
  printf("UART Tx: ");
  for (int i = 0; i < frame.length; i++) {
    printf("%02X ", cmdArray[i]);
  }
  printf("\n");
#endif
}

void LD2420Component::received_frame_handler_(uint8_t *buffer, int len) {
//rec_frame.
}

void LD2420Component::set_config_mode_(bool enable) {
  uint8_t cmdArray[128];
  cmd_frame_t cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header =  CMD_FRAME_HEADER;
  cmd_frame.command = enable ? CMD_ENABLE_CONF : CMD_DISABLE_CONF;
  if (enable) {
    memcpy(&cmd_frame.data[0], &CMD_PROTOCOL_VER, sizeof(CMD_PROTOCOL_VER));
    cmd_frame.data_length += sizeof(CMD_PROTOCOL_VER);
  }
  cmd_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG,"Sending set config mode command: %2X",cmd_frame.command);
  send_cmd_from_array_(cmdArray, cmd_frame);
}

void LD2420Component::get_firmware_version_(void) {
  uint8_t cmdArray[128];
  cmd_frame_t cmd_frame;
  cmd_frame.data_length = 0;
  cmd_frame.header =  CMD_FRAME_HEADER;
  cmd_frame.command = CMD_READ_VERSION;
  cmd_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGV(TAG,"Sending get firmware version command: %2X",cmd_frame.command);
  send_cmd_from_array_(cmdArray, cmd_frame);
}

void LD2420Component::set_min_max_distances_timeout_(uint32_t max_gate_distance, uint32_t min_gate_distance, uint32_t timeout) {

  // Header H, Length L, Register R, Value V, Footer F
  // HH HH HH HH LL LL CC CC RR RR VV VV VV VV RR RR VV VV VV VV RR RR VV VV VV VV FF FF FF FF
  // FD FC FB FA 14 00 07 00 00 00 00 00 00 00 01 00 0F 00 00 00 02 00 0A 00 00 00 04 03 02 01 e.g.

  uint8_t cmdArray[128];
  cmd_frame_t reg_frame;
  reg_frame.data_length = 0;
  reg_frame.header = CMD_FRAME_HEADER;
  reg_frame.command = CMD_WRITE_ABD_PARAM;
  memcpy(&reg_frame.data[reg_frame.data_length], &CMD_MAX_GATE_REG, sizeof(CMD_MAX_GATE_REG)); // Register: global max detect gate number
  reg_frame.data_length += sizeof(CMD_MAX_GATE_REG);
  memcpy(&reg_frame.data[reg_frame.data_length], &max_gate_distance, sizeof(max_gate_distance));
  reg_frame.data_length += sizeof(max_gate_distance);
  memcpy(&reg_frame.data[reg_frame.data_length], &CMD_MIN_GATE_REG, sizeof(CMD_MIN_GATE_REG)); // Register: global min detect gate number
  reg_frame.data_length += sizeof(CMD_MIN_GATE_REG);
  memcpy(&reg_frame.data[reg_frame.data_length], &min_gate_distance, sizeof(min_gate_distance));
  reg_frame.data_length += sizeof(min_gate_distance);
  memcpy(&reg_frame.data[reg_frame.data_length], &CMD_TIMEOUT_REG, sizeof(CMD_TIMEOUT_REG)); // Register: global delay time
  reg_frame.data_length += sizeof(CMD_TIMEOUT_REG);
  memcpy(&reg_frame.data[reg_frame.data_length], &timeout, sizeof(timeout));;
  reg_frame.data_length += sizeof(timeout);
  reg_frame.footer = CMD_FRAME_FOOTER;

  ESP_LOGV(TAG,"Sending global gate detect min max and delay params: %2X",reg_frame.command);
  send_cmd_from_array_(cmdArray, reg_frame);
}

void LD2420Component::set_gate_thresholds_(uint8_t gate, uint32_t move_sens, uint32_t still_sens) {

  // Header H, Length L, Register R, Value V, Footer F
  // HH HH HH HH LL LL CC CC RR RR VV VV VV VV RR RR VV VV VV VV RR RR VV VV VV VV FF FF FF FF
  // FD FC FB FA 14 00 07 00 00 00 00 00 00 00 01 00 0F 00 00 00 02 00 0A 00 00 00 04 03 02 01 e.g.

  uint16_t move_gate = CMD_MOVE_GATE[gate];
  uint16_t still_gate = CMD_STILL_GATE[gate];
  uint8_t cmdArray[128];
  cmd_frame_t reg_frame;
  reg_frame.data_length = 0;
  reg_frame.header = CMD_FRAME_HEADER;
  reg_frame.command = CMD_WRITE_ABD_PARAM;
  memcpy(&reg_frame.data[reg_frame.data_length], &move_gate, sizeof(move_gate));
  reg_frame.data_length += sizeof(move_gate);
  memcpy(&reg_frame.data[reg_frame.data_length], &move_sens, sizeof(move_sens));
  reg_frame.data_length += sizeof(move_sens);
  memcpy(&reg_frame.data[reg_frame.data_length], &still_gate, sizeof(still_gate));
  reg_frame.data_length += sizeof(still_gate);
    memcpy(&reg_frame.data[reg_frame.data_length], &still_sens, sizeof(still_sens));
  reg_frame.data_length += sizeof(still_sens);
  reg_frame.footer = CMD_FRAME_FOOTER;
  ESP_LOGV(TAG,"Sending gate sensitivity params: %2X",reg_frame.command);
  send_cmd_from_array_(cmdArray, reg_frame);
}

}  // namespace ld2420
}  // namespace esphome
