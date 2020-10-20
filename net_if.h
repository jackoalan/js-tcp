#pragma once

#include "fd.h"
#include <vector>

struct joystick_state;

class net_if {
public:
  explicit net_if(int port);

  bool accept_connections(const joystick_state &state);
  bool send_updates(const joystick_state &state);
  bool valid() const { return m_listen_sock != -1; }
  int listen_sock() const { return m_listen_sock; }

private:
  fd m_listen_sock;
  std::vector<fd> m_conns;
};
