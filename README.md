# pnet-tiberius

Two Linux services to bridge joystick state and Talon SRX CAN network to Profinet.

* Compatible with any Joystick that appears on `/dev/input/js0`.
* Compatible with any SocketCAN interface that appears in network
  subsystem as `can0`.
* Full controller state is buffered and transferred on initial 
  connections and subsequent Joystick events.
* Designed to handle Joystick disconnections and reconnections.
* Includes two systemd units for easy integration.

### Requirements

* C++17 build environment (tested with gcc8)
* CMake 3.10+
* udev development package (`libudev-dev` on debian-like distros)

### Building and Installing

```shell script
cd <path-to-repository>
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<path-to-distro-prefix> ..
cmake --build .
cmake --install .
```
