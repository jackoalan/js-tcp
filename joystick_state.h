#pragma once

#include <cstdint>

struct js_event;

struct joystick_state {
  void receive_event(const js_event &event);
  static constexpr unsigned num_axis = 6;
  static constexpr unsigned num_buttons = 32;
  uint16_t m_axis[num_axis] = {};
  uint32_t m_button;
};
