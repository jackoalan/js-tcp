# js-tcp

Simple Linux service to bridge joystick state to one or more push-transfer TCP connections.

* Accepts unlimited connections on port 9042.
* Compatible with any Joystick that appears on `/dev/input/js0`.
* Full controller state is buffered and transferred on initial 
  connections and subsequent Joystick events.
* Designed to handle Joystick disconnections and reconnections.
* Includes a systemd unit for easy integration.

### Requirements

* C++11 build environment (tested with gcc8)
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
