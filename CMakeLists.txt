cmake_minimum_required(VERSION 3.7)

# set the project name and version
project(smc_storm VERSION 0.1.2)

# Use RelWithDebInfo by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
# Enable compile commands export for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Required for perf
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")
# Additional flags
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O3")

# Make sure lib and binaries are separated
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(storm REQUIRED)  # search for Storm library
find_package(smc_verifiable_plugins REQUIRED)  # TODO: Once released, this can be made optional and use FetchContent

include(FetchContent)
# Argparse
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG v3.0
)
FetchContent_MakeAvailable(argparse)

include_directories(include)

# Add all storm sources except cli.cpp
file(GLOB_RECURSE LIB_SRC_FILES ${PROJECT_SOURCE_DIR}/src/*/*.cpp)

add_library(${PROJECT_NAME}_lib SHARED ${LIB_SRC_FILES})
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${storm_INCLUDE_DIR} ${smc_verifiable_plugins_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_lib PUBLIC argparse storm storm-parsers)

add_executable(${PROJECT_NAME} src/cli.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

# Provide some smc_verifiable_plugins implementations
add_library(uniform_random_smc_plugin SHARED ${PROJECT_SOURCE_DIR}/plugins/src/uniform_random_smc_plugin.cpp)
target_include_directories(uniform_random_smc_plugin PUBLIC ${smc_verifiable_plugins_INCLUDE_DIR})

include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  # Copy the test files in the build folder
  set(TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
  set(MODEL_DIRECTORY ${TEST_DIRECTORY}/models)
  set(PLUGIN_STORM_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  add_definitions(-DSMC_PLUGINS_PATHS="${PLUGIN_STORM_DIR},${smc_verifiable_plugins_PLUGINS_PATH}")
  file(GLOB_RECURSE models RELATIVE ${MODEL_DIRECTORY} ${MODEL_DIRECTORY}/*)
  foreach(test_file ${models})
    # Use configure_file so this is rerun each time make is invoked (as opposed to just cmake)
    configure_file(${MODEL_DIRECTORY}/${test_file} models/${test_file} COPYONLY)
  endforeach()

  # Unit tests
  add_executable(test_batch_buffer test/unit_tests/test_batch_buffer.cpp)
  target_link_libraries(test_batch_buffer ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_batch_buffer WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  add_executable(test_states_generator test/unit_tests/test_states_generator.cpp)
  target_link_libraries(test_states_generator ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_states_generator WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  # System tests
  add_executable(test_jani_models test/system_tests/test_jani_models.cpp)
  target_link_libraries(test_jani_models ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_jani_models WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  add_executable(test_prism_models test/system_tests/test_prism_models.cpp)
  target_link_libraries(test_prism_models ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_prism_models WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  add_executable(test_trace_export test/system_tests/test_trace_export.cpp)
  target_link_libraries(test_trace_export ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_trace_export WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()
