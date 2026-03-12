cmake_minimum_required(VERSION 3.20)
project(aausat6-vision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # For clangd / IDE support

# --- Build type default ---
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# --- Dependencies ---
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
    gstreamer-1.0
    gstreamer-app-1.0
    gstreamer-video-1.0
)

# --- Common interface target (shared flags + includes) ---
add_library(vision_common INTERFACE)
target_include_directories(vision_common INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
)
target_link_libraries(vision_common INTERFACE
    ${OpenCV_LIBS}
    ${GSTREAMER_LIBRARIES}
    yaml-cpp
)
target_compile_options(vision_common INTERFACE
    -Wall -Wextra -Wpedantic
    $<$<CONFIG:Release>:-O2>
    $<$<CONFIG:Debug>:-g -O0>
)
target_link_directories(vision_common INTERFACE
    ${GSTREAMER_LIBRARY_DIRS}
)

# --- Subdirectories ---
add_subdirectory(src)

option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()