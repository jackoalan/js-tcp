#include "joystick_state.h"
#include <linux/joystick.h>

void joystick_state::receive_event(const js_event &event) {
  switch (event.type & uint8_t(~uint8_t(JS_EVENT_INIT))) {
  case JS_EVENT_BUTTON:
    if (event.number < num_buttons) {
      if (event.value)
        m_button |= 1u << event.number;
      else
        m_button &= ~(1u << event.number);
    }
  case JS_EVENT_AXIS:
    if (event.number < num_axis)
      m_axis[event.number] = event.value;
    break;
  }
}
