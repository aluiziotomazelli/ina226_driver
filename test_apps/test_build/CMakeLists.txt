# test_apps/test_build/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# PROJECT_ROOT is components/time_manager
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../" ABSOLUTE)

list(APPEND EXTRA_COMPONENT_DIRS 
    "${PROJECT_ROOT}" 
    "${PROJECT_ROOT}/external"
    "$ENV{IDF_PATH}/components"
)

set(COMPONENTS main time_manager idf_hals)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_build)
