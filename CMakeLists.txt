cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
project(pnet-tiberius)

include(FindPkgConfig)
pkg_check_modules(udev REQUIRED IMPORTED_TARGET libudev)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/p-net/cmake")
set(BUILD_TESTING No)
add_subdirectory(p-net)

set(INCLUDE_DIRS "${PROFINET_SOURCE_DIR}/src/ports/linux"
        "${PROFINET_SOURCE_DIR}/src" "${PROFINET_BINARY_DIR}/src")

add_executable(pnet-js main_js.cpp udev_if.cpp pnet_if.cpp)
target_link_libraries(pnet-js PUBLIC PkgConfig::udev profinet)
target_include_directories(pnet-js PUBLIC ${INCLUDE_DIRS})
target_compile_definitions(pnet-js PUBLIC TIBERIUS_JS)

add_executable(pnet-can main_can.cpp can_if.cpp pnet_if.cpp)
target_link_libraries(pnet-can PUBLIC PkgConfig::udev profinet)
target_include_directories(pnet-can PUBLIC ${INCLUDE_DIRS})
target_compile_definitions(pnet-can PUBLIC TIBERIUS_CAN)

install(TARGETS pnet-js pnet-can RUNTIME DESTINATION bin)
install(FILES pnet-js.service pnet-can.service DESTINATION lib/systemd/system)
install(FILES "${PROFINET_SOURCE_DIR}/src/ports/linux/set_network_parameters" DESTINATION /
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
