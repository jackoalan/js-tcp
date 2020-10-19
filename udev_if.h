#pragma once

struct udev;
struct udev_monitor;
struct joystick_state;
class net_if;

class udev_if {
public:
  explicit udev_if(const char *sysname);
  ~udev_if();

  void event_loop(joystick_state &state, net_if &net);
  bool valid() const { return m_monitor_fd != -1; }

private:
  const char *m_sysname;
  udev *m_udev;
  udev_monitor *m_monitor;
  int m_monitor_fd = -1;
  int m_device_fd = -1;
};
