cmake_minimum_required(VERSION 3.5)

# set the project name and version
project(smc_storm VERSION 1.0)

# Enable compile commands export for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Make sure lib and binaries are separated
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# search for Storm library
find_package(storm REQUIRED)

include(FetchContent)
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
target_include_directories(${PROJECT_NAME}_lib PUBLIC ${storm_INCLUDE_DIR} ${storm-parsers_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_lib PUBLIC storm storm-parsers argparse)

add_executable(${PROJECT_NAME} src/cli.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

include(CTest)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  # Copy the test files in the build folder
  set(TEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
  file(GLOB_RECURSE test_files RELATIVE ${TEST_DIRECTORY} ${TEST_DIRECTORY}/test_files/*)
  foreach(test_file ${test_files})
    # Use configure_file so this is rerun each time make is invoked (as opposed to just cmake)
    configure_file(${TEST_DIRECTORY}/${test_file} ${test_file} COPYONLY)
  endforeach()

  add_executable(test_jani_models test/test_jani_models.cpp)
  target_link_libraries(test_jani_models ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_jani_models WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  add_executable(test_prism_models test/test_prism_models.cpp)
  target_link_libraries(test_prism_models ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_prism_models WORKING_DIRECTORY ${CMAKE_BINARY_DIR})

  add_executable(test_trace_export test/test_trace_export.cpp)
  target_link_libraries(test_trace_export ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_trace_export WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()
