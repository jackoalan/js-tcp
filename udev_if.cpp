#include "udev_if.h"
#include "joystick_state.h"
#include "net_if.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <libudev.h>
#include <linux/joystick.h>
#include <unistd.h>

class udev_device_ptr {
  udev_device *m_dev;

public:
  udev_device_ptr(udev_device *dev) : m_dev(dev) {}
  operator udev_device *() const { return m_dev; }
  ~udev_device_ptr() { udev_device_unref(m_dev); }
};

udev_if::udev_if(const char *sysname) : m_sysname(sysname) {
  m_udev = udev_new();
  if (!m_udev) {
    std::cerr << "error calling udev_new(): " << std::strerror(errno)
              << std::endl;
    return;
  }

  /* Set up monitor to handle device reconnections */
  m_monitor = udev_monitor_new_from_netlink(m_udev, "udev");
  if (!m_monitor) {
    std::cerr << "error calling udev_monitor_new_from_netlink(): "
              << std::strerror(errno) << std::endl;
    return;
  }

  udev_monitor_filter_add_match_subsystem_devtype(m_monitor, "input", nullptr);
  udev_monitor_filter_update(m_monitor);
  udev_monitor_enable_receiving(m_monitor);
  m_monitor_fd = udev_monitor_get_fd(m_monitor);

  /* Set up device if initially connected */
  if (udev_device_ptr dev =
          udev_device_new_from_subsystem_sysname(m_udev, "input", sysname)) {
    const char *devnode = udev_device_get_devnode(dev);
    m_device_fd = ::open(devnode, O_RDONLY | O_NONBLOCK);
    if (m_device_fd == -1) {
      if (errno == EINTR) {
        m_monitor_fd = -1;
        return;
      }
      std::cerr << "error opening " << devnode
                << " for reading: " << std::strerror(errno) << std::endl;
    } else {
      std::cout << "Opened " << devnode << std::endl;
    }
  }
}

void udev_if::event_loop(joystick_state &state, net_if &net) {
  while (m_monitor_fd != -1) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_monitor_fd, &read_fds);
    int max_fd = m_monitor_fd;
    if (m_device_fd != -1) {
      FD_SET(m_device_fd, &read_fds);
      max_fd = std::max(max_fd, m_device_fd);
    }
    if (net.listen_sock() != -1) {
      FD_SET(net.listen_sock(), &read_fds);
      max_fd = std::max(max_fd, net.listen_sock());
    }
    if (select(max_fd + 1, &read_fds, nullptr, nullptr, nullptr) == -1)
      return;

    /* Poll for reconnected device */
    if (FD_ISSET(m_monitor_fd, &read_fds)) {
      while (udev_device_ptr dev = udev_monitor_receive_device(m_monitor)) {
        const char *sysname = udev_device_get_sysname(dev);
        if (!std::strcmp(sysname, m_sysname)) {
          const char *action = udev_device_get_action(dev);
          if (!std::strcmp(action, "add")) {
            const char *devnode = udev_device_get_devnode(dev);
            if (m_device_fd != -1) {
              if (::close(m_device_fd) == -1 && errno == EINTR)
                return;
            }
            m_device_fd = ::open(devnode, O_RDONLY | O_NONBLOCK);
            if (m_device_fd == -1) {
              if (errno == EINTR)
                return;
              std::cerr << "error opening " << devnode
                        << " for reading: " << std::strerror(errno)
                        << std::endl;
            } else {
              std::cout << "Opened " << devnode << std::endl;
            }
          }
        }
      }
      if (errno == EINTR)
        return;
    }

    /* Process joystick event data */
    bool js_update = false;
    if (m_device_fd != -1 && FD_ISSET(m_device_fd, &read_fds)) {
      js_event event;
      while (::read(m_device_fd, &event, sizeof(event)) == sizeof(event)) {
        state.receive_event(event);
        js_update = true;
      }
      if (errno == EINTR)
        return;
    }

    if (js_update || FD_ISSET(net.listen_sock(), &read_fds)) {
      if (!net.send_segments(state))
        return;
    }
  }
}

udev_if::~udev_if() {
  ::close(m_device_fd);
  udev_monitor_unref(m_monitor);
  udev_unref(m_udev);
}
