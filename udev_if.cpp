#include "udev_if.h"
#include "joystick_state.h"
#include "net_if.h"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>

extern bool volatile g_terminate;

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
    m_joystick_fd = ::open(devnode, O_RDONLY | O_NONBLOCK);
    if (m_joystick_fd == -1) {
      std::cerr << "error opening " << devnode
                << " for reading: " << std::strerror(errno) << std::endl;
    } else {
      std::cout << "Opened " << devnode << std::endl;
    }
  }
}

#define return_if_interrupted                                                  \
  if (errno == EINTR)                                                          \
  return

static constexpr time_t update_interval = 10;

void udev_if::event_loop(joystick_state &state, net_if &net) {
  timeval timeout{update_interval, 0};
  while (!g_terminate) {
    /*
     * Wait on one of three event sources:
     * - udev monitor for controller reconnections
     * - device file for controller events
     * - TCP listen socket for new incoming connections
     */
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_monitor_fd, &read_fds);
    int max_fd = m_monitor_fd;
    if (m_joystick_fd != -1) {
      FD_SET(m_joystick_fd, &read_fds);
      max_fd = std::max(max_fd, int(m_joystick_fd));
    }
    if (net.listen_sock() != -1) {
      FD_SET(net.listen_sock(), &read_fds);
      max_fd = std::max(max_fd, net.listen_sock());
    }
    int n_fd_trig = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (n_fd_trig == -1) {
      return_if_interrupted;
      std::cerr << "error calling select(): " << std::strerror(errno)
                << std::endl;
      continue;
    } else if (n_fd_trig == 0) {
      /* timeout occurred - send updates to keep connections alive */
      if (!net.send_updates(state))
        return;
      timeout.tv_sec = update_interval;
      continue;
    }

    bool js_update = false;

    /* Handle udev monitor events */
    if (FD_ISSET(m_monitor_fd, &read_fds)) {
      while (udev_device_ptr dev = udev_monitor_receive_device(m_monitor)) {
        const char *sysname = udev_device_get_sysname(dev);
        if (!std::strcmp(sysname, m_sysname)) {
          const char *action = udev_device_get_action(dev);
          if (!std::strcmp(action, "add")) {
            state.reset();
            js_update = true;
            const char *devnode = udev_device_get_devnode(dev);
            if (m_joystick_fd.close())
              return_if_interrupted;
            m_joystick_fd = ::open(devnode, O_RDONLY | O_NONBLOCK);
            if (m_joystick_fd == -1) {
              return_if_interrupted;
              std::cerr << "error opening " << devnode
                        << " for reading: " << std::strerror(errno)
                        << std::endl;
            } else {
              std::cout << "Opened " << devnode << std::endl;
            }
          }
        }
      }
      return_if_interrupted;
    }

    /* Handle joystick events */
    if (m_joystick_fd != -1 && FD_ISSET(m_joystick_fd, &read_fds)) {
      js_event event;
      while (m_joystick_fd.read_event(event)) {
        state.receive_event(event);
        js_update = true;
      }
      return_if_interrupted;
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        state.reset();
        js_update = true;
        std::cout << "Lost Joystick" << std::endl;
        m_joystick_fd.close();
      }
    }
    if (js_update) {
      if (!net.send_updates(state))
        return;
    }

    /* Handle new TCP connections */
    if (FD_ISSET(net.listen_sock(), &read_fds)) {
      if (!net.accept_connections(state))
        return;
    }
  }
}
