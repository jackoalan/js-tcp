#pragma once

#include <algorithm>
#include <unistd.h>

class fd {
protected:
  int m_fd = -1;

public:
  fd() = default;
  fd(int fd) : m_fd(fd) {}
  fd(const fd &) = delete;
  fd(fd &&other) : m_fd(other.m_fd) { other.m_fd = -1; }
  fd &operator=(const fd &) = delete;
  fd &operator=(fd &&other) {
    std::swap(m_fd, other.m_fd);
    return *this;
  }
  fd &operator=(int fd) {
    close();
    m_fd = fd;
    return *this;
  }
  operator int() const { return m_fd; }
  ~fd() { close(); }

  bool close() {
    bool err = false;
    if (m_fd != -1)
      err = ::close(m_fd) == -1;
    m_fd = -1;
    return err;
  }
};
