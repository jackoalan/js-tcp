cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
project(js-tcp)

include(FindPkgConfig)
pkg_check_modules(udev REQUIRED IMPORTED_TARGET libudev)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/p-net/cmake")
add_subdirectory(p-net)

add_executable(js-tcp main.cpp udev_if.cpp net_if.cpp pnet_if.cpp)
target_link_libraries(js-tcp PUBLIC PkgConfig::udev profinet)
target_include_directories(js-tcp PUBLIC "${PROFINET_SOURCE_DIR}/src/osal/linux"
        "${PROFINET_SOURCE_DIR}/src" "${PROFINET_BINARY_DIR}/src")

install(TARGETS js-tcp RUNTIME DESTINATION bin)
install(FILES js-tcp.service DESTINATION lib/systemd/system)
install(FILES "${PROFINET_SOURCE_DIR}/src/osal/linux/set_network_parameters" DESTINATION /
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
