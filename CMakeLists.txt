cmake_minimum_required (VERSION 3.1)
project (virtual_pads)

set (CMAKE_C_STANDARD 11)
add_executable(virtual_pads src/virtual_pads.c)

# evdev
find_path(EVDEV_INCLUDE_DIR libevdev/libevdev.h PATH_SUFFIXES libevdev-1.0)
find_library(EVDEV_LIB evdev)
target_include_directories(virtual_pads PUBLIC ${EVDEV_INCLUDE_DIR})
target_link_libraries(virtual_pads "${EVDEV_LIB}")
