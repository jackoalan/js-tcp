#pragma once

#include "fd.h"
#include "joystick_state.h"
#include <libudev.h>

class pnet_if;

#define DECLARE_UDEV_PTR(type)                                                 \
  extern "C" type *type##_unref(type *);                                       \
  class type##_ptr {                                                           \
    type *m_ptr = nullptr;                                                     \
                                                                               \
  public:                                                                      \
    type##_ptr() = default;                                                    \
    type##_ptr(type *ptr) : m_ptr(ptr) {}                                      \
    type##_ptr(const type##_ptr &) = delete;                                   \
    type##_ptr(type##_ptr &&other) : m_ptr(other.m_ptr) {                      \
      other.m_ptr = nullptr;                                                   \
    }                                                                          \
    type##_ptr &operator=(const type##_ptr &) = delete;                        \
    type##_ptr &operator=(type##_ptr &&) = delete;                             \
    type##_ptr &operator=(type *ptr) {                                         \
      unref();                                                                 \
      m_ptr = ptr;                                                             \
      return *this;                                                            \
    }                                                                          \
    operator type *() const { return m_ptr; }                                  \
    ~type##_ptr() { unref(); }                                                 \
                                                                               \
    void unref() {                                                             \
      if (m_ptr != nullptr)                                                    \
        type##_unref(m_ptr);                                                   \
      m_ptr = nullptr;                                                         \
    }                                                                          \
  };

DECLARE_UDEV_PTR(udev)
DECLARE_UDEV_PTR(udev_monitor)
DECLARE_UDEV_PTR(udev_device)
#undef DECLARE_UDEV_PTR

class joystick_fd : public fd {
public:
  using fd::fd;
  bool read_event(js_event &event_out) const {
    return ::read(m_fd, &event_out, sizeof(event_out)) == sizeof(event_out);
  }
};

class udev_if {
  template <typename A>
  bool handle_udev_monitor(A on_add);
  template <typename E, typename L>
  bool handle_joystick(E on_event, L on_lost);

public:
  explicit udev_if(const char *sysname);

  int event_loop(joystick_state &state, pnet_if &pnet, uint32_t periodic_us);
  bool valid() const { return m_monitor_fd != -1; }

private:
  const char *m_sysname;
  udev_ptr m_udev;
  udev_monitor_ptr m_monitor;
  int m_monitor_fd = -1;
  joystick_fd m_joystick_fd;
};
