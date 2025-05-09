# smc_storm
A Statistical Model Checker implementation building on top of STORM

## Installation

### Using deployed binaries

We provide pre-built binaries that can be used on Ubuntu. They can be found at the [Releases page](https://github.com/convince-project/smc_storm/releases).

To install them on your machine, extract `smc_storm_executable.tar.gz` and follow these steps:

```bash
cd smc_storm_executable            # This is the extracted archive
install.sh --install-dependencies  # This flag will install all  packages required by smc_storm and its dependencies
export PATH=$PATH:$PWD/bin         # This way, smc_storm can be called from anywhere
smc_storm --help                   # Make sure the binary runs
```

### Compile from source

#### Get the Official Storm installed
This package needs Storm to be built on your local machine. To achieve that, follow the [official documentation](https://www.stormchecker.org/documentation/obtain-storm/build.html).

We used the following command to build storm:
```bash
export STORM_DIR=<path-to-storm-repo>
cmake -DSTORM_USE_SPOT_SHIPPED=ON $STORM_DIR && make -j10
```

We recommend to use the `master` branch, since it provides the latest features as the trigonometric operators.

#### Build this module
After cloning this repository, execute the following commands:
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
# optionally, run the automatic unit tests to make sure everything works as expected
ctest
```
## Run the executable
To make the smc_storm executable usable from any folder, we suggest to add the following line to your `~/.bashrc` file or run it before start using the tool:
```bash
# Make smc_storm executable from anywhere
export PATH=$PATH:<path-to-smc-storm-repo>/build/bin
```

## Available parameters
The list of available parameters with its related description can be obtained using following command:
```bash
smc_storm --help
```

## Examples
```bash
# Example 1
smc_storm --model <path-to-smc-storm-repo>/test/models/leader_sync.3-2.v1.jani --property-name eventually_elected --batch-size 200
# Example 2
smc_storm --model <path-to-smc-storm-repo>/test/models/nand.v1.jani --property-name reliable --constants "N=20,K=2" --epsilon 0.01 --confidence 0.95 --n-threads 5 --show-statistics
# Example 3 (rm added to make sure the csv file doesn't exist at smc_storm execution time)
rm -f exported_traces.csv && smc_storm --model <path-to-smc-storm-repo>/test/models/leader_sync.3-2.v1.jani --property-name time --traces-file exported_traces.csv --show-statistics --max-n-traces 5
```
