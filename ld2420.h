#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace ld2420 {

//#define LD2420_LOG_VERY_VERBOSE   // Provides uart hex I/O logged as readable strings

// Commands
static const uint16_t CMD_PROTOCOL_VER = 0x0001;
static const uint16_t CMD_ENABLE_CONF = 0x00FF;
static const uint16_t CMD_DISABLE_CONF = 0x00FE;
static const uint16_t CMD_READ_VERSION = 0x0000;
static const uint16_t CMD_WRITE_REGISTER = 0x0001;
static const uint16_t CMD_READ_REGISTER = 0x0002;
static const uint16_t CMD_PARM_HIGH_TRESH = 0x0012;
static const uint16_t CMD_PARM_LOW_TRESH = 0x0021;
static const uint16_t CMD_WRITE_ABD_PARAM = 0x0007;
static const uint16_t CMD_READ_ABD_PARAM = 0x0008;
static const uint16_t CMD_WRITE_SYS_PARAM = 0x0012;
static const uint16_t CMD_READ_SYS_PARAM = 0x0013;
static const uint16_t CMD_READ_SERIAL_NUM = 0x0011;

static const uint16_t CMD_MAXDIST_DURATION = 0x0060;
static const uint16_t CMD_QUERY = 0x0061;
static const uint16_t CMD_ENGINEERING_MODE = 0x0062;
static const uint16_t CMD_NORMAL_MODE = 0x0063;
static const uint16_t CMD_GATE_SENS = 0x0064;

static const uint16_t CMD_BAUD_RATE = 0x00A1;  // 0x0001-0x0008 9600 .. doubles on increment up to 460800
static const uint16_t CMD_FACTORY_RESET = 0x00A2;
static const uint16_t CMD_RESTART = 0x00A3;



// Register address values
static const uint16_t CMD_MIN_GATE_REG =  0x0000;
static const uint16_t CMD_MAX_GATE_REG =  0x0001;
static const uint16_t CMD_TIMEOUT_REG =   0x0004;
static const uint16_t CMD_MOVE_GATE[16] =  {0x0010, 0x0011, 0x0012, 0x0013,
                                            0x0014, 0x0015, 0x0016, 0x0017,
                                            0x0018, 0x0019, 0x001A, 0x001B,
                                            0x001C, 0x001D, 0x001E, 0x001F};
static const uint16_t CMD_STILL_GATE[16] = {0x0020, 0x0021, 0x0022, 0x0023,
                                            0x0024, 0x0025, 0x0026, 0x0027,
                                            0x0028, 0x0029, 0x002A, 0x002B,
                                            0x002C, 0x002D, 0x002E, 0x002F};

// Command Header & Footer
static const uint32_t CMD_FRAME_HEADER = 0xFAFBFCFD;
static const uint32_t CMD_FRAME_FOOTER = 0x01020304;
static const uint8_t CMD_FRAME_HEADER_OLD[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FRAME_END[4] = {0x04, 0x03, 0x02, 0x01};
// Data Header & Footer
static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_END[4] = {0xF8, 0xF7, 0xF6, 0xF5};
/*
Data Type: 6th byte
Target states: 9th byte
    Moving target distance: 10~11th bytes
    Moving target energy: 12th byte
    Still target distance: 13~14th bytes
    Still target energy: 15th byte
    Detect distance: 16~17th bytes
*/
enum PeriodicDataStructure : uint8_t {
  DATA_TYPES = 5,
  TARGET_STATES = 8,
  MOVING_TARGET_LOW = 9,
  MOVING_TARGET_HIGH = 10,
  MOVING_ENERGY = 11,
  STILL_TARGET_LOW = 12,
  STILL_TARGET_HIGH = 13,
  STILL_ENERGY = 14,
  DETECT_DISTANCE_LOW = 15,
  DETECT_DISTANCE_HIGH = 16,
};
enum PeriodicDataValue : uint8_t { HEAD = 0xAA, END = 0x55, CHECK = 0x00 };
enum AckDataStructure : uint8_t { COMMAND = 6, COMMAND_STATUS = 7 };

enum MoveGateRegAddress : uint8_t { GATE1 = 0x0A, GATE2 = 0x0B };

//  char cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
class LD2420Component : public Component, public uart::UARTDevice {
#ifdef USE_SENSOR
  SUB_SENSOR(moving_target_distance)
  SUB_SENSOR(still_target_distance)
  SUB_SENSOR(moving_target_energy)
  SUB_SENSOR(still_target_energy)
  SUB_SENSOR(detection_distance)
#endif

public:
  void setup() override;
  void dump_config() override;
  void loop() override;

  struct cmd_frame_t {
    uint32_t header;
    uint16_t length;
    uint16_t command;
    uint8_t data[18];
    uint16_t data_length;
    uint8_t elements;
    uint32_t footer;
  };


#ifdef USE_BINARY_SENSOR
  void set_presence_sensor(binary_sensor::BinarySensor *sens) { this->presence_binary_sensor_ = sens; };
  void set_moving_target_sensor(binary_sensor::BinarySensor *sens) { this->moving_binary_sensor_ = sens; };
  void set_still_target_sensor(binary_sensor::BinarySensor *sens) { this->still_binary_sensor_ = sens; };
#endif
  float get_setup_priority() const override;
  void send_cmd_from_array_(uint8_t* cmdArray, cmd_frame_t cmd_frame);
  void get_firmware_version_(void);
  void set_object_range_(uint16_t range) {this->object_range_ = range; };
  void set_object_presence_(bool presence) {this->presence_ = presence; };
  void set_timeout(uint16_t value) { this->timeout_ = value; };
  void set_max_gate(uint8_t value) { this->max_gate_distance_ = value; };
  void set_min_gate(uint8_t value) { this->min_gate_distance_ = value; };
  void set_range_config(uint32_t rg0_move, uint32_t rg0_still, uint32_t rg1_move, uint32_t rg1_still,
                        uint32_t rg2_move, uint32_t rg2_still, uint32_t rg3_move, uint32_t rg3_still,
                        uint32_t rg4_move, uint32_t rg4_still, uint32_t rg5_move, uint32_t rg5_still,
                        uint32_t rg6_move, uint32_t rg6_still, uint32_t rg7_move, uint32_t rg7_still,
                        uint32_t rg8_move, uint32_t rg8_still, uint32_t rg9_move, uint32_t rg9_still,
                        uint32_t rg10_move, uint32_t rg10_still, uint32_t rg11_move, uint32_t rg11_still,
                        uint32_t rg12_move, uint32_t rg12_still, uint32_t rg13_move, uint32_t rg13_still,
                        uint32_t rg14_move, uint32_t rg14_still, uint32_t rg15_move, uint32_t rg15_still) {
    this->rg0_move_threshold_ = rg0_move;
    this->rg0_still_threshold_ = rg0_still;
    this->rg1_move_threshold_ = rg1_move;
    this->rg1_still_threshold_ = rg1_still;
    this->rg2_move_threshold_ = rg2_move;
    this->rg2_still_threshold_ = rg2_still;
    this->rg3_move_threshold_ = rg3_move;
    this->rg3_still_threshold_ = rg3_still;
    this->rg4_move_threshold_ = rg4_move;
    this->rg4_still_threshold_ = rg4_still;
    this->rg5_move_threshold_ = rg5_move;
    this->rg5_still_threshold_ = rg5_still;
    this->rg6_move_threshold_ = rg6_move;
    this->rg6_still_threshold_ = rg6_still;
    this->rg7_move_threshold_ = rg7_move;
    this->rg7_still_threshold_ = rg7_still;
    this->rg8_move_threshold_ = rg8_move;
    this->rg8_still_threshold_ = rg8_still;
    this->rg9_move_threshold_ = rg9_move;
    this->rg9_still_threshold_ = rg9_still;
    this->rg10_move_threshold_ = rg10_move;
    this->rg10_still_threshold_ = rg10_still;
    this->rg11_move_threshold_ = rg11_move;
    this->rg11_still_threshold_ = rg11_still;
    this->rg12_move_threshold_ = rg12_move;
    this->rg12_still_threshold_ = rg12_still;
    this->rg13_move_threshold_ = rg13_move;
    this->rg13_still_threshold_ = rg13_still;
    this->rg14_move_threshold_ = rg14_move;
    this->rg14_still_threshold_ = rg14_still;
    this->rg15_move_threshold_ = rg15_move;
    this->rg15_still_threshold_ = rg15_still;
  };

  int moving_sensitivities[9] = {0};
  int still_sensitivities[9] = {0};

  int32_t last_periodic_millis = millis();
  int32_t last_normal_periodic_millis = millis();


 protected:
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *presence_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *moving_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *still_binary_sensor_{nullptr};
#endif

  std::vector<uint8_t> rx_buffer_;
  int two_byte_to_int_(char firstbyte, char secondbyte) { return (int16_t) (secondbyte << 8) + firstbyte; }
  void send_command_(uint8_t command_str, uint8_t *command_value, int command_value_len);
  bool is_present_() { return presence_; }
  void set_min_max_distances_timeout_(uint32_t max_gate_distance, uint32_t min_gate_distance, uint32_t timeout);
  void set_gate_thresholds_(uint8_t gate, uint32_t move_sens, uint32_t still_sens);
  void set_config_mode_(bool enable);
  void handle_periodic_data_(uint8_t *buffer, int len);
  void received_frame_handler_(uint8_t *buffer, int len);
  void handle_normal_mode_(uint8_t *buffer, int len);
  void handle_ack_data_(uint8_t *buffer, int len);
  void readline_(int readch, uint8_t *buffer, int len);
  void query_parameters_();
  void get_version_();

  uint32_t timeout_;
  uint32_t max_gate_distance_;
  uint32_t min_gate_distance_;
  uint16_t object_range_;
  bool presence_{false};
  char ld2420_firmware_ver_[8];


  uint32_t rg0_move_threshold_, rg0_still_threshold_, rg1_move_threshold_, rg1_still_threshold_,
      rg2_move_threshold_, rg2_still_threshold_, rg3_move_threshold_, rg3_still_threshold_,
      rg4_move_threshold_, rg4_still_threshold_, rg5_move_threshold_, rg5_still_threshold_,
      rg6_move_threshold_, rg6_still_threshold_, rg7_move_threshold_, rg7_still_threshold_,
      rg8_move_threshold_, rg8_still_threshold_, rg9_move_threshold_, rg9_still_threshold_,
      rg10_move_threshold_, rg10_still_threshold_, rg11_move_threshold_, rg11_still_threshold_,
      rg12_move_threshold_, rg12_still_threshold_, rg13_move_threshold_, rg13_still_threshold_,
      rg14_move_threshold_, rg14_still_threshold_, rg15_move_threshold_, rg15_still_threshold_;
};

}  // namespace ld2420
}  // namespace esphome
