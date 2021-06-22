cmake_minimum_required(VERSION 2.8)

project(dpdk-ring)

#add_definitions(-std=c++17)

# generating build/compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-march=native)

option(RINGBUF_ALWAYS_INLINE "Enable always inline" ON)
if (RINGBUF_ALWAYS_INLINE)
    add_definitions(-DRINGBUF_ALWAYS_INLINE)
endif()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No CMAKE_BUILD_TYPE selected, defaulting to ${CMAKE_BUILD_TYPE}")
endif()
set(CMAKE_C_FLAGS_DEBUG "$ENV{CFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_C_FLAGS_RELEASE "$ENV{CFLAGS} -O3 -Wall")

include_directories(${PROJECT_SOURCE_DIR})
set(SRCS ringbuf.c)

add_library(ringbuf SHARED ${SRCS})
set_target_properties(ringbuf PROPERTIES PUBLIC_HEADER ${PROJECT_SOURCE_DIR}/ringbuf.h)
set_target_properties(ringbuf PROPERTIES VERSION 1.0 SOVERSION 1)

add_library(ringbuf_static STATIC ${SRCS})
set_target_properties(ringbuf_static PROPERTIES OUTPUT_NAME "ringbuf")
set_target_properties(ringbuf_static PROPERTIES PUBLIC_HEADER ${PROJECT_SOURCE_DIR}/ringbuf.h)


include(GNUInstallDirs)
install(TARGETS ringbuf ringbuf_static
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# -DENABLE_TESTING=ON
option(ENABLE_TESTING "Enable make test" ON)
if (ENABLE_TESTING)
    # apt install libgtest-dev
    enable_testing()
    add_subdirectory(tests)
endif()

# -DENABLE_BENCH=ON
option(ENABLE_BENCH "Enable make bench" ON)
if (ENABLE_BENCH)
    # using the following command to install benchmark library
    # apt install libbenchmark-dev
    add_subdirectory(bench)
endif()
