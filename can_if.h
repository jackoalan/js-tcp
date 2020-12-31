#pragma once

#include "fd.h"
#include <cstdint>
#include <linux/can/raw.h>
#include <vector>

class pnet_if;

template <class payload_t> struct can_frame_t : can_frame {
  explicit can_frame_t(canid_t dev_id)
      : can_frame{payload_t::id | dev_id, payload_t::payload_sz} {
    new (data) payload_t();
  }

  payload_t *operator->() { return reinterpret_cast<payload_t *>(data); }

  payload_t &operator*() { return reinterpret_cast<payload_t &>(data); }

  const payload_t *operator->() const {
    return reinterpret_cast<const payload_t *>(data);
  }

  const payload_t &operator*() const {
    return reinterpret_cast<payload_t &>(data);
  }
};

struct status1 {
  // 0x02041400, faults
  static constexpr canid_t id = 0x02041400 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 8;
  __u64 data = 0;
};

struct status2 {
  // 0x02041440, feedback
  static constexpr canid_t id = 0x02041440 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 8;
  __u64 data = 0;

  int32_t get_sensor_position() const {
    unsigned ret = ((data & 0xffull) << 16ull) | (data & 0xff00ull) |
                   ((data & 0xff0000ull) >> 16ull);
    return int32_t(ret << 8u) >> 8;
  }

  uint16_t get_current() const {
    unsigned high = (data >> 40ull) & 0xffull;
    unsigned low = (data >> 48ull) & 0xc0ull;
    return ((high << 8u) | low) >> 6u;
  }
};

struct status3 {
  // 0x02041480, logical feedback
  static constexpr canid_t id = 0x02041480 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 8;
  __u64 data = 0;
};

struct status4 {
  // 0x020414C0, analog
  static constexpr canid_t id = 0x020414C0 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 8;
  __u64 data = 0;

  uint8_t get_batt_v() const { return (data >> 48ull) & 0xffull; }

  uint8_t get_temp() const { return (data >> 40ull) & 0xffull; }
};

struct ctrl1 {
  // 0x02040000, enable, 10ms period
  static constexpr canid_t id = 0x02040000 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 2;
  __u16 enable = 0;
};

enum class EControlMode : __u8 {
  Throttle = 0,
  FollowerMode = 5,
  VoltageMode = 4,
  PositionMode = 1,
  SpeedMode = 2,
  CurrentMode = 3,
  MotionProfileMode = 6,
  MotionMagic = 7,
  Disabled = 15
};

enum class ELimitSwitchOverride : __u8 {
  UseDefaultsFromFlash = 1,
  DisableFwd_DisableRev = 4,
  DisableFwd_EnableRev = 5,
  EnableFwd_DisableRev = 6,
  EnableFwd_EnableRev = 7
};

enum class EFeedbackDevice : __u8 {
  QuadEncoder = 0,
  AnalogPot = 2,
  AnalogEncoder = 3,
  EncRising = 4,
  EncFalling = 5,
  CtreMagEncoder_Relative = 6,
  CtreMagEncoder_Absolute = 7,
  PulseWidth = 8
};

struct ctrl5 {
  // 0x02040100, primary control, 50ms period
  static constexpr canid_t id = 0x02040100 | CAN_EFF_FLAG;
  static constexpr __u8 payload_sz = 8;
  __u64 data = 0;

  void set_demand(__u32 value, EControlMode ctrl_mode) {
    data &= ~(0xffull << 16ull);
    data &= ~(0xffull << 24ull);
    data &= ~(0xffull << 32ull);
    data |= (value & 0xff0000ull);
    data |= (value & 0x00ff00ull) << 16ull;
    data |= (value & 0x0000ffull) << 32ull;
    data &= ~(0xfull << 52ull);
    data |= (__u8(ctrl_mode) & 0xfull) << 52ull;
  }

  void set_ramp_throttle(uint8_t ramp) {
    data &= ~(0xffull << 56ull);
    data |= (ramp & 0xffull) << 56ull;
  }

  void set_override_limit_switch_en(ELimitSwitchOverride mode) {
    data &= ~(0x7ull << 45ull);
    data |= (__u8(mode) & 0x7ull) << 45ull;
  }

  void set_feedback_device(EFeedbackDevice device) {
    data &= ~(0xfull << 41ull);
    data |= (__u8(device) & 0xfull) << 41ull;
  }
};

struct can_input_state {
  int32_t potentiometer;
  uint16_t current;
  uint8_t batt_v;
  uint8_t temp;
};

struct can_output_state {
  int32_t demand;
};

struct talon_srx {
  canid_t m_dev_id;
#define CAN_FRAME_DEF(struc) can_frame_t<struc> m_##struc;
#include "can_if.def"
#define CAN_FRAME_SEND(struc, cycle_div) int m_##struc##_rem = cycle_div;
#include "can_if.def"
  bool m_watchdog = false;
  explicit talon_srx(canid_t dev_id)
      : m_dev_id(dev_id)
#define CAN_FRAME_DEF(struc) , m_##struc(dev_id)
#include "can_if.def"
  {
    m_ctrl5->set_override_limit_switch_en(
        ELimitSwitchOverride::UseDefaultsFromFlash);
    m_ctrl5->set_feedback_device(EFeedbackDevice::AnalogPot);
    m_ctrl5->set_ramp_throttle(128);
  }

  int32_t get_sensor_position() const {
    return m_status2->get_sensor_position();
  }

  uint16_t get_current() const { return m_status2->get_current(); }

  uint8_t get_batt_v() const { return m_status4->get_batt_v(); }

  uint8_t get_temp() const { return m_status4->get_temp(); }

  can_input_state get_can_input_state() const {
    return {__builtin_bswap32(get_sensor_position()), __builtin_bswap16(get_current()), get_batt_v(), get_temp()};
  }

  void set_demand(int32_t demand) {
    m_ctrl5->set_demand(demand, EControlMode::Throttle);
  }

  void set_can_output_state(const can_output_state &state) {
    m_watchdog = true;
    set_demand(__builtin_bswap32(state.demand));
  }
};

class can_if {
  fd m_can_sock;
  std::vector<talon_srx> m_talons;
  bool m_messages_read = false;
  bool read_messages();
  bool write_messages(); // Call every 10ms
  bool write_message(const can_frame &frame) const;

public:
  explicit can_if(const char *canif);
  bool valid() const { return m_can_sock != -1; }
  talon_srx &find_or_create_talon(canid_t id);
  int event_loop(pnet_if &pnet, uint32_t periodic_us);
};
