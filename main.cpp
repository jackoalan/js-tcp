#include "joystick_state.h"
#include "net_if.h"
#include "udev_if.h"
#include <csignal>
#include <iostream>

bool volatile g_terminate = false;
void term_handler(int) {
  std::cout << "Terminating..." << std::endl;
  g_terminate = true;
}

static int run() {
  udev_if udev("js0");
  net_if net(9042);
  if (!udev.valid() || !net.valid())
    return 1;
  joystick_state state;
  udev.event_loop(state, net);
  return 0;
}

int main(int argc, char **argv) {
  ::signal(SIGINT, term_handler);
  ::signal(SIGTERM, term_handler);
  int ret = run();
  std::cout << "Closed all connections" << std::endl;
  return ret;
}
