# Copyright 2021 Qiying Wang. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

# using the following command to install benchmark library
# apt install libbenchmark-dev
find_package(benchmark REQUIRED)

add_executable(ringbuf_bench
    ringbuf_bench.cc)

target_link_libraries(ringbuf_bench
    benchmark::benchmark
    ringbuf
)

add_custom_target(bench
    COMMAND ./ringbuf_bench
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} # build/bench/
)
