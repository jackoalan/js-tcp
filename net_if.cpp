#include "net_if.h"
#include "joystick_state.h"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <sys/socket.h>

net_if::net_if(int port) {
  fd listen_sock = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, IPPROTO_TCP);
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
    return;
  }

  if (::listen(listen_sock, 128) == -1) {
    std::cerr << "error calling listen(): " << std::strerror(errno)
              << std::endl;
    return;
  }

  m_listen_sock = std::move(listen_sock);
  std::cout << "Listening on port " << port << std::endl;

  m_conns.reserve(16);
}

#define return_if_interrupted                                                  \
  if (errno == EINTR)                                                          \
  return false

bool net_if::accept_connections(const joystick_state &state) {
  struct sockaddr_in addr {};
  socklen_t addr_len = sizeof(addr);

  fd new_fd;
  while ((new_fd = ::accept(m_listen_sock, reinterpret_cast<sockaddr *>(&addr),
                            &addr_len)) != -1) {
    char hostname[256] = {};
    if (::getnameinfo(reinterpret_cast<sockaddr *>(&addr), sizeof(addr),
                      hostname, 256, nullptr, 0, 0)) {
      return_if_interrupted;
    }
    std::cout << "Accepted connection " << new_fd << " from " << hostname
              << std::endl;

    if (::send(new_fd, &state, sizeof(state), MSG_NOSIGNAL) == -1) {
      return_if_interrupted;
      std::cout << "Closed connection " << new_fd << std::endl;
      if (new_fd.close())
        return_if_interrupted;
      continue;
    }
    m_conns.emplace_back(std::move(new_fd));
  }
  return_if_interrupted;

  return true;
}

bool net_if::send_updates(const joystick_state &state) {
  for (auto it = m_conns.begin(); it != m_conns.end();) {
    if (::send(*it, &state, sizeof(state), MSG_NOSIGNAL) == -1) {
      return_if_interrupted;
      std::cout << "Closed connection " << *it << std::endl;
      it = m_conns.erase(it);
      return_if_interrupted;
      continue;
    }
    ++it;
  }

  return true;
}
