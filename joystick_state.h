#pragma once

#include <array>
#include <cstdint>
#include <linux/joystick.h>

struct js_event;

struct joystick_state {
  static constexpr unsigned num_axis = 6;
  static constexpr unsigned num_buttons = 32;
  std::array<uint16_t, num_axis> m_axis{};
  uint32_t m_button{};

  void receive_event(const js_event &event) {
    switch (event.type & uint8_t(~uint8_t(JS_EVENT_INIT))) {
    case JS_EVENT_BUTTON:
      if (event.number < num_buttons) {
        if (event.value)
          m_button |= 1u << event.number;
        else
          m_button &= ~(1u << event.number);
      }
      break;
    case JS_EVENT_AXIS:
      if (event.number < num_axis)
        m_axis[event.number] = __builtin_bswap16(event.value);
      break;
    }
  }

  void reset() {
    std::fill(m_axis.begin(), m_axis.end(), 0);
    m_button = 0;
  }
};
