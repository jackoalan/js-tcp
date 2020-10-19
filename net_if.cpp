#include "net_if.h"
#include "joystick_state.h"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

net_if::net_if(int port) {
  int listen_sock = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);
  if (listen_sock == -1) {
    std::cerr << "error calling socket(): " << std::strerror(errno)
              << std::endl;
    return;
  }

  int enable = 1;
  if (::setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &enable,
                   sizeof(int)) == -1) {
    std::cerr << "error calling setsockopt(SO_REUSEADDR): "
              << std::strerror(errno) << std::endl;
    return;
  }

  struct sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (::bind(listen_sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) ==
      -1) {
    std::cerr << "error binding port " << port << ": " << std::strerror(errno)
              << std::endl;
    ::close(listen_sock);
    return;
  }

  if (::listen(listen_sock, 128) == -1) {
    std::cerr << "error calling listen(): " << std::strerror(errno)
              << std::endl;
    ::close(listen_sock);
    return;
  }

  m_listen_sock = listen_sock;
  std::cout << "Listening on port " << port << std::endl;
}

bool net_if::send_segments(const joystick_state &state) {
  if (m_listen_sock == -1)
    return false;

  for (auto it = m_conns.begin(); it != m_conns.end();) {
    if (::send(*it, &state, sizeof(state), MSG_NOSIGNAL) == -1) {
      if (errno == EINTR)
        return false;
      std::cout << "Closed connection" << std::endl;
      if (::close(*it) == -1 && errno == EINTR)
        return false;
      it = m_conns.erase(it);
      continue;
    }
    ++it;
  }

  struct sockaddr_in addr {};
  socklen_t addr_len = sizeof(addr);

  int new_fd;
  while ((new_fd = ::accept(m_listen_sock, reinterpret_cast<sockaddr *>(&addr),
                            &addr_len)) != -1) {
    char hostname[256] = {};
    if (::getnameinfo(reinterpret_cast<sockaddr *>(&addr), sizeof(addr),
                      hostname, 256, nullptr, 0, 0)) {
      if (errno == EINTR)
        return false;
    }
    std::cout << "Accepted connection from " << hostname << std::endl;

    if (::send(new_fd, &state, sizeof(state), MSG_NOSIGNAL) == -1) {
      if (errno == EINTR)
        return false;
      std::cout << "Closed connection" << std::endl;
      if (::close(new_fd) == -1 && errno == EINTR)
        return false;
      continue;
    }
    m_conns.push_back(new_fd);
  }

  return errno != EINTR;
}

net_if::~net_if() {
  for (int conn : m_conns)
    ::close(conn);
  ::close(m_listen_sock);
  std::cout << "Closed all connections" << std::endl;
}
