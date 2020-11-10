#include "joystick_state.h"
#include "pnet_if.h"
#include "can_if.h"
#include <csignal>
#include <iostream>

bool volatile g_terminate = false;
void term_handler(int) {
  std::cout << "Terminating..." << std::endl;
  g_terminate = true;
}

static int run() {
  can_if can("can0");
  constexpr uint32_t tick_us = 1000;
  pnet_if pnet("eth0", tick_us);
  if (!can.valid() || !pnet.valid())
    return 1;
  return can.event_loop(pnet, tick_us);
}

int main(int argc, char **argv) {
  ::signal(SIGINT, term_handler);
  ::signal(SIGTERM, term_handler);
  int ret = run();
  std::cout << "Closed all connections" << std::endl;
  return ret;
}
