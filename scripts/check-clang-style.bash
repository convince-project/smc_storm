#!/bin/bash

# Get the path to the directory of script
CODE_ROOT=$(dirname $(dirname $(realpath "${BASH_SOURCE[0]}")))
# Move to the root of the project
pushd $CODE_ROOT > /dev/null || exit
# Apply clang-format to all source files

clang-tidy --config-file=.clang-tidy --warnings-as-errors=* -p=build src/**/*.cpp include/**/*.hpp **/*.cpp && \
clang-format --style=file src/**/*.cpp include/**/*.hpp test/*.cpp --dry-run --Werror

if [ $? -ne 0 ]; then
    echo "Error: clang-tidy or clang-format failed"
    popd > /dev/null
    exit 1
fi
# Return to the original directory
popd > /dev/null
exit
