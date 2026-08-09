cmake_minimum_required(VERSION 3.16)

include("../extra_dirs_common.cmake")

project(test_ina226_driver)

set(EXTRACT_PATTERNS_test_ina226_driver "*/ina226_driver/src/*.cpp" "*/ina226_driver/include/*.hpp")
set(REMOVE_PATTERNS_test_ina226_driver "*/ina226_driver/include/interfaces/*.hpp")

include("../coverage_common.cmake")
setup_gtest_coverage(test_ina226_driver test_ina226_driver.elf)
