#!/bin/bash

# Get the path to the directory of script
CODE_ROOT=$(dirname $(dirname $(realpath "${BASH_SOURCE[0]}")))
# Move to the root of the project
pushd $CODE_ROOT > /dev/null || exit
# Apply clang-format to all source files
clang-tidy-14 --config-file=.clang-tidy -p=build src/**/*.cpp src/*.cpp include/**/*.hpp test/**/*.cpp --fix-errors
clang-format-14 --style=file -i src/**/*.cpp src/*.cpp include/**/*.hpp test/**/*.cpp
# Return to the original directory
popd > /dev/null || exit
