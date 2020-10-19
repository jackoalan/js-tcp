#pragma once

#include <vector>

struct joystick_state;

class net_if {
public:
  explicit net_if(int port);
  ~net_if();

  bool send_segments(const joystick_state &state);
  bool valid() const { return m_listen_sock != -1; }
  int listen_sock() const { return m_listen_sock; }

private:
  int m_listen_sock = -1;
  std::vector<int> m_conns;
};
