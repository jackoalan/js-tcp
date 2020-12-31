#include "can_if.h"
#include "pnet_if.h"
#include "timespec_util.h"
#include <cstring>
#include <iostream>
#include <net/if.h>
#include <sys/ioctl.h>

extern bool volatile g_terminate;

#define return_if_interrupted                                                  \
  if (errno == EINTR)                                                          \
  return false

bool can_if::update_read_messages(pnet_if &pnet) {
  /* Read all incoming can messages and route to talon records */
  if (!read_messages())
    return false;

  /* Update profinet for each talon if new data has been read */
  if (m_messages_read) {
    m_messages_read = false;
    for (auto &t : m_talons) {
      if (t.m_messages_read) {
        t.m_messages_read = false;
        pnet.send_updates(t);
      }
    }
  }

  return true;
}

bool can_if::read_messages() {
  struct can_frame frame {};

  while (true) {
    int nbytes;
    if ((nbytes = ::read(m_can_sock, &frame, sizeof(frame))) == -1) {
      return_if_interrupted;
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        return true;
      std::cerr << "bad can read" << std::endl;
      return false;
    }

    if (nbytes < sizeof(frame)) {
      std::cerr << "incomplete can read" << std::endl;
      return false;
    }

    /* lower 10 bits is the talon's unique id */
    canid_t dev_id = frame.can_id & 0x3fu;
    talon_srx &t = find_or_create_talon(dev_id);

    /* upper 19 bits is the status record within the talon */
    switch (frame.can_id & ~0x3fu) {
#define CAN_FRAME_RECV(struc)                                                  \
  case struc::id:                                                              \
    static_cast<can_frame &>(t.m_##struc) = frame;                             \
    t.m_messages_read = true;                                                  \
    m_messages_read = true;                                                    \
    break;
#include "can_if.def"
    default:
      break;
    }
  }
}

bool can_if::write_messages() {
  for (auto &t : m_talons) {
    /* The watchdog is set as long as error-free profinet data is available
     * since last CAN cycle */
    if (!t.m_watchdog)
      continue;
    t.m_watchdog = false;

    /* Conditionally send can messages based on cycle interval */
#define CAN_FRAME_SEND(struc, cycle_div)                                       \
  if (--t.m_##struc##_rem == 0) {                                              \
    t.m_##struc##_rem = cycle_div;                                             \
    if (!write_message(t.m_##struc))                                           \
      return false;                                                            \
  }
#include "can_if.def"
  }

  return true;
}

bool can_if::write_message(const can_frame &frame) const {
  int nbytes;
  if ((nbytes = ::write(m_can_sock, &frame, sizeof(frame))) == -1) {
    return_if_interrupted;
    if (errno == ENOBUFS)
      return true;
    std::cerr << "bad can write " << strerror(errno) << std::endl;
    return true;
  }

  if (nbytes < sizeof(frame)) {
    std::cerr << "incomplete can write" << std::endl;
    return false;
  }

  return true;
}

talon_srx &can_if::find_or_create_talon(canid_t id) {
  auto search =
      std::find_if(m_talons.begin(), m_talons.end(),
                   [id](const talon_srx &t) { return t.m_dev_id == id; });
  if (search == m_talons.end()) {
    std::cerr << "Created talon " << id << std::endl;
    return m_talons.emplace_back(id);
  }
  return *search;
}

#undef return_if_interrupted
#define return_if_interrupted                                                  \
  if (errno == EINTR)                                                          \
  return 0

int can_if::event_loop(pnet_if &pnet, uint32_t periodic_us) {
  /* Attempt to use real-time scheduling */
  static_assert(_POSIX_THREAD_PRIORITY_SCHEDULING > 0,
                "SCHED_FIFO not available");
  struct sched_param param = {.sched_priority = 5};
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    std::cerr << "Unable to setup SCHED_FIFO: " << strerror(errno) << std::endl;
  }

  const timespec periodic_ts{0, periodic_us * 1000};
  timespec next_update{};
  clock_gettime(CLOCK_MONOTONIC, &next_update);
  auto set_next_update = [&]() {
    timespec cur_time{};
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    timespec tmp{};
    while (!timespec_subtract(tmp, cur_time, next_update))
      timespec_add(next_update, next_update, periodic_ts);
  };
  set_next_update();

  const uint32_t can_cycle_div = 10000 / periodic_us;
  uint32_t rem_can_cycles = can_cycle_div;

  while (!g_terminate) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(m_can_sock, &read_fds);
    int max_fd = m_can_sock;
    timespec cur_time{};
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    timespec rem_time{};
    int n_fd_trig = -1;
    if (timespec_subtract(rem_time, next_update, cur_time) ||
        (n_fd_trig = pselect(max_fd + 1, &read_fds, nullptr, nullptr, &rem_time,
                             nullptr)) == 0) {
      /* timeout occurred - handle periodic updates */
      set_next_update();

      /* first ensure profinet has latest can read data */
      if (!update_read_messages(pnet))
        return 0;

      /* profinet communication cycle */
      if (pnet.periodic(*this))
        return 1;
      return_if_interrupted;

      /* perform can write cycle every 10ms */
      if (--rem_can_cycles == 0) {
        rem_can_cycles = can_cycle_div;
        if (!write_messages())
          return errno != EINTR;
      }
    } else if (n_fd_trig == -1) {
      /* pselect error (normally just EINTR for program termination) */
      return_if_interrupted;
      std::cerr << "error calling pselect(): " << std::strerror(errno)
                << std::endl;
    } else if (FD_ISSET(m_can_sock, &read_fds)) {
      /* can read data available */
      if (!update_read_messages(pnet))
        return 0;
    }
  }

  return 0;
}

can_if::can_if(const char *canif) {
  fd s = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

  struct ifreq ifr {};
  std::strcpy(ifr.ifr_name, canif);
  if (::ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
    std::cerr << "error running ioctl(SIOCGIFINDEX) for " << canif << std::endl;
    return;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    std::cerr << "error running bind for " << canif << std::endl;
    return;
  }

  m_can_sock = std::move(s);
}
