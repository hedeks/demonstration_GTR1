cmake_minimum_required(VERSION 3.16)
project(GTR1_MVP VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(Protobuf REQUIRED)

# Protobuf Generation
file(GLOB PROTO_FILES "02_data_models/*.proto" "04_interfaces/*.proto")
if(PROTO_FILES)
    protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
else()
    set(PROTO_SRCS "")
endif()

# Core Library Sources
file(GLOB_RECURSE CORE_SRCS "src/core/*.cpp" "src/operators/*.cpp" "07_infrastructure/*.cpp")

# Define the shared library for the GTR1 Core
add_library(gtr1_core SHARED ${CORE_SRCS} ${PROTO_SRCS})

# Compiler warnings
if(MSVC)
    target_compile_options(gtr1_core PRIVATE /W4)
else()
    target_compile_options(gtr1_core PRIVATE -Wall -Wextra)
endif()

# Include directories
target_include_directories(gtr1_core PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/operators>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/07_infrastructure>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(gtr1_core PUBLIC 
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    ${Protobuf_LIBRARIES}
    zmq
)

# Main Executable
add_executable(gtr1_main src/main.cpp)
target_link_libraries(gtr1_main PUBLIC gtr1_core)

# --- Подключение GoogleTest ---
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.12.1
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(gtr1_tests tests/test_gtr1.cpp)
target_link_libraries(gtr1_tests PUBLIC gtr1_core gtest_main)

add_executable(test_data_models tests/test_data_models.cpp)
target_link_libraries(test_data_models PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_data_models PRIVATE -Wall -Wextra -Werror)
endif()

add_executable(test_slgw tests/test_slgw.cpp)
target_link_libraries(test_slgw PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_slgw PRIVATE -Wall -Wextra)
endif()

add_executable(test_mp11_merge tests/test_mp11_merge.cpp)
target_link_libraries(test_mp11_merge PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_mp11_merge PRIVATE -Wall -Wextra -Werror)
endif()

add_executable(test_rtr_cycle tests/test_rtr_cycle.cpp)
target_link_libraries(test_rtr_cycle PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_rtr_cycle PRIVATE -Wall -Wextra -Werror)
endif()

add_executable(test_golden_dataset tests/test_golden_dataset.cpp)
target_link_libraries(test_golden_dataset PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_golden_dataset PRIVATE -Wall -Wextra)
endif()

include(GoogleTest)
gtest_discover_tests(gtr1_tests)
gtest_discover_tests(test_data_models)
gtest_discover_tests(test_slgw)
gtest_discover_tests(test_mp11_merge)
gtest_discover_tests(test_rtr_cycle)
gtest_discover_tests(test_golden_dataset)
add_executable(test_pybullet_sim tests/test_pybullet_sim.cpp)
target_link_libraries(test_pybullet_sim PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_pybullet_sim PRIVATE -Wall -Wextra)
endif()

gtest_discover_tests(test_pybullet_sim)

add_executable(test_filters tests/test_filters.cpp)
target_link_libraries(test_filters PUBLIC gtr1_core gtest_main)
if(NOT MSVC)
    target_compile_options(test_filters PRIVATE -Wall -Wextra)
endif()

gtest_discover_tests(test_filters)
