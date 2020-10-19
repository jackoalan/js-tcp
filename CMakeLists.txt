cmake_minimum_required(VERSION 3.10)
project(js-tcp)

include(FindPkgConfig)
pkg_check_modules(udev REQUIRED IMPORTED_TARGET libudev)

add_executable(js-tcp main.cpp udev_if.cpp joystick_state.cpp net_if.cpp)
target_link_libraries(js-tcp PUBLIC PkgConfig::udev)

install(TARGETS js-tcp RUNTIME DESTINATION bin)
install(FILES js-tcp.service DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/systemd/system/")
