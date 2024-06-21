#!/bin/bash

# Get the path to the directory of script
CODE_ROOT=$(dirname $(dirname $(realpath "${BASH_SOURCE[0]}")))
# Move to the root of the project
pushd $CODE_ROOT > /dev/null || exit
# Apply clang-format to all source files
clang-format --style=file -i src/**/*.cpp include/**/*.h* test/*.cpp
clang-tidy --config-file=.clang-tidy -p=build src/**/*.cpp include/**/*.hpp **/*.cpp
# Return to the original directory
popd > /dev/null || exit