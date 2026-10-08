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

# Include directories
target_include_directories(gtr1_core PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/core/include>
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

include(GoogleTest)
gtest_discover_tests(gtr1_tests)
