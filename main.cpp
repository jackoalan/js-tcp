#include "joystick_state.h"
#include "net_if.h"
#include "udev_if.h"
#include <csignal>
#include <iostream>

void term_handler(int sig) { std::cout << "Terminating..." << std::endl; }

int main(int argc, char **argv) {
  ::signal(SIGINT, term_handler);
  ::signal(SIGTERM, term_handler);
  udev_if udev("js0");
  net_if net(9042);
  if (!udev.valid() || !net.valid())
    return 1;
  joystick_state state;
  udev.event_loop(state, net);
  return 0;
}
