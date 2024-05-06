cmake_minimum_required(VERSION 3.2)

# set the project name and version
project(smc_storm VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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

set(LIB_SRC_FILES
    src/model_checker/smc_engine.cpp
    src/model_checker/state_generation.cpp
    src/model_checker/statistical_model_checker.cpp
    src/parser/parsers.cpp
    src/properties/property_description.cpp
    src/samples/batch_statistics.cpp
    src/samples/exploration_information.cpp
    src/samples/model_sampling.cpp
    src/samples/sampling_results.cpp
)

add_library(${PROJECT_NAME}_lib ${LIB_SRC_FILES})
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

  add_executable(test_models test/test_models.cpp)
  target_link_libraries(test_models ${PROJECT_NAME}_lib GTest::gtest GTest::gtest_main)
  gtest_add_tests(TARGET test_models)
endif()
