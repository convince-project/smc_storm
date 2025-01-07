#!/bin/bash

# Get the path to the directory of script
CODE_ROOT=$(dirname $(dirname $(realpath "${BASH_SOURCE[0]}")))
# Move to the root of the project
pushd $CODE_ROOT > /dev/null || exit
# Apply clang-format to all source files

clang-tidy-14 --config-file=.clang-tidy --warnings-as-errors=* -p=build src/**/*.cpp src/*.cpp include/**/*.hpp test/**/*.cpp && \
clang-format-14 --style=file src/**/*.cpp src/*.cpp include/**/*.hpp test/**/*.cpp --dry-run --Werror

if [ $? -ne 0 ]; then
    echo "Error: clang-tidy or clang-format failed"
    popd > /dev/null
    exit 1
fi
# Return to the original directory
popd > /dev/null
exit
