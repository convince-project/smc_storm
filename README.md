# smc_storm
A Statistical Model Checker implementation building on top of STORM

## Installation

Please refer to the [online documentation](https://convince-project.github.io/smc_storm/installation.html#how-to-install-smc-storm) for instruction on how to install this package.

## Usage

### List all available parameters

The list of available parameters with its related description can be obtained using following command:
```bash
smc_storm --help
```

### Run some examples

```bash
# Example 1
smc_storm --model <path-to-smc-storm-repo>/test/models/leader_sync.3-2.v1.jani --property-name eventually_elected --batch-size 200
# Example 2
smc_storm --model <path-to-smc-storm-repo>/test/models/nand.v1.jani --property-name reliable --constants "N=20,K=2" --epsilon 0.01 --confidence 0.95 --n-threads 5 --show-statistics
# Example 3 (rm added to make sure the csv file doesn't exist at smc_storm execution time)
rm -f exported_traces.csv && smc_storm --model <path-to-smc-storm-repo>/test/models/leader_sync.3-2.v1.jani --property-name time --traces-file exported_traces.csv --show-statistics --max-n-traces 5
```

## Citing this work

If you want to cite this tool, you can refer to the paper [Towards verifying robotic systems using statistical model checking in STORM](https://link.springer.com/chapter/10.1007/978-3-031-75434-0_28).
