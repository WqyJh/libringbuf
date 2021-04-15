# using the following command to install gtest library
# apt install libgtest-dev
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

set(GTEST_LIBS ${GTEST_BOTH_LIBRARIES} pthread)

add_library(test_ring SHARED test_ring.c)
set_target_properties(test_ring PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(test_ring ringbuf)

add_executable(main test_main.cc)
set_target_properties(main PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(main test_ring  ${GTEST_LIBS} pthread)

gtest_discover_tests(main)
